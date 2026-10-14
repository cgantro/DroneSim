#include "scene/Entity.h"
#include "physics/PhysicsWorld.h"
#include "TestSupport.h"
#include "model/GltfLoader.h"
#include "scene/TransformComponents.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "scene/TransformSystemModule.h"
#include "simulation/robotics/GripperTransformAdapter.h"
#include "simulation/robotics/RobotEnvironmentCollisionGuard.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotTransformAdapter.h"

#include <gtest/gtest.h>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
using grasplink::model::ModelResource;
using grasplink::model::NodeData;
using grasplink::scene::Entity;
using grasplink::scene::Rotation;
using grasplink::scene::Local;
using grasplink::scene::Scene;
using grasplink::scene::TransformSystemModule;

#define ASSERT_TRUE_MESSAGE(condition, message) ASSERT_TRUE(condition) << message
#define EXPECT_TRUE_MESSAGE(condition, message) EXPECT_TRUE(condition) << message
#define ASSERT_NEAR_FINITE(actual, expected, tolerance, message) \
    ASSERT_TRUE(std::isfinite(actual) && std::abs((actual) - (expected)) <= (tolerance)) << message

template<typename Exception, typename Function>
void GTestExpectThrows(Function&& function, const std::string& message)
{
    EXPECT_THROW(function(), Exception) << message;
}


using namespace grasplink::robotics;
using grasplink::physics::BodyMotionType;
using grasplink::physics::CollisionLayer;

void RequireVector(const glm::vec3& actual, const glm::vec3& expected, const std::string& label)
{
    // GLB에서 만든 Entity 변환은 float로 저장하지만 기구학 계산은 double 정밀도를 쓴다. 두 계산 경로의 반올림 차이를 고려해 위치 비교에 20 μm의 허용 오차를 둔다.
    for (int axis = 0; axis < 3; ++axis)
        ASSERT_NEAR_FINITE(actual[axis], expected[axis], 2.0e-5, label);
}

void RequireMatrix(const glm::mat4& actual, const glm::mat4& expected, const std::string& label)
{
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            ASSERT_NEAR_FINITE(actual[column][row], expected[column][row], 2.0e-5, label);
}

void RequireRotation(const glm::quat& actual, const glm::quat& expected, const std::string& label)
{
    // quaternion q와 -q는 같은 회전을 나타낸다. 단위 길이로 정규화한 두 값의 내적을 사용해 부호가 달라도 회전 방향이 같은지 비교한다.
    ASSERT_TRUE_MESSAGE(std::abs(glm::dot(glm::normalize(actual), glm::normalize(expected))) > 0.99999F, label);
}

glm::vec3 InGripper(const Entity& entity, const Entity& gripper)
{
    return glm::vec3(glm::inverse(gripper.GetWorldMatrix()) * entity.GetWorldMatrix()[3]);
}

/**
 * @brief 실제 GLB의 전체 부모 계층에서 Controller 상태부터 관절과 Jolt까지의 자세 전달을 실행한다.
 * @details 실제 모델 node의 위치·회전·크기와 부모 연결을 사용하되 화면 모양을 GPU에 올리지는 않는다. Entity는 장면 물체 참조이고 Scene이 삭제한다.
 * 따라서 변환 adapter와 물리 연결은 Scene 및 물체 저장소인 Flecs World보다 먼저 정리한다.
 * 팔 FK는 관절 각도에서 링크 위치·방향을 계산한다. 그 결과와 여러 갈래 손가락 관절의 회전을 같은 4 ms 간격에 적용한다.
 */
struct Fixture
{
    flecs::world world;
    grasplink::physics::PhysicsWorld physics;
    ModelResource model;
    std::vector<Entity> nodes;
    Entity robotRoot;
    Entity gripper;
    backends::simulation::SimGripperController controller{models::robotiq::kTwoF85};
    kinematics::GripperKinematics gripperMath{models::robotiq::kTwoF85};
    kinematics::RobotKinematics armMath{models::hanwha::kHcr12a};
    RobotState armState;
    std::unique_ptr<grasplink::simulation::robotics::RobotTransformAdapter> armAdapter;
    std::unique_ptr<grasplink::simulation::robotics::GripperTransformAdapter> gripperAdapter;
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> integration;
    std::unique_ptr<Scene> scene;

    Fixture()
    {
        world.import<TransformSystemModule>();
        scene = std::make_unique<Scene>(world);
        EXPECT_TRUE_MESSAGE(scene->GetSceneRoot().is_alive(), "Scene owns a live hierarchy root");
        model = LoadTestGlb("HCR12A_2F-85.glb");
        EXPECT_TRUE_MESSAGE(model.rootNodeIndex >= 0, "model root exists");
        nodes.resize(model.nodes.size());
        robotRoot = CreateTree(model.rootNodeIndex, Entity{scene->GetSceneRoot()});
        gripper = robotRoot.FindChildByNameRecursive("Gripper");
        EXPECT_TRUE_MESSAGE(static_cast<bool>(gripper), "authored Gripper exists");
        EXPECT_TRUE_MESSAGE(controller.Connect().Ok() && controller.Activate().Ok(), "controller activated");
        armState.valid = true;
        armState.jointPositionRadians.assign(6, 0.0);
        armAdapter = std::make_unique<grasplink::simulation::robotics::RobotTransformAdapter>(robotRoot, models::hanwha::kHcr12a);
        gripperAdapter = std::make_unique<grasplink::simulation::robotics::GripperTransformAdapter>(gripper, models::robotiq::kTwoF85);
        ApplyPoses();
        grasplink::simulation::ConfigureTwoF85Colliders(*scene, robotRoot, model);
        TransformSystemModule::UpdateWorldTransforms(world);
        integration = CreateTestPhysicsSystem(world, physics);
    }

    Entity CreateTree(int index, const Entity& parent)
    {
        const NodeData& node = model.nodes.at(static_cast<std::size_t>(index));
        Entity entity = scene->CreateEntity(node.name);
        nodes[static_cast<std::size_t>(index)] = entity;
        entity.SetParent(parent);
        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);
        for (int child : node.childrenIndices) CreateTree(child, entity);
        return entity;
    }

    void ApplyPoses()
    {
        armAdapter->Apply(armMath.Update(armState));
        gripperAdapter->Apply(gripperMath.Update(controller.GetState()));
        TransformSystemModule::UpdateWorldTransforms(world);
    }

    void Step(int count)
    {
        for (int step = 0; step < count; ++step)
        {
            controller.Update(0.004);
            ApplyPoses();
            integration->Step(0.004);
            TransformSystemModule::UpdateWorldTransforms(world);
        }
    }

    void Command(std::uint8_t position, std::uint8_t speed = 255)
    {
        ASSERT_TRUE_MESSAGE(controller.Command({position, speed, 128}).Ok(), "valid gripper command accepted");
    }

    void CheckLocalPositions() const
    {
        for (std::size_t index = 0; index < nodes.size(); ++index)
            if (nodes[index] != robotRoot)
            {
                RequireVector(nodes[index].GetLocalPosition(), model.nodes[index].translation, "GLB Local position preserved");
                RequireVector(nodes[index].GetLocalScale(), model.nodes[index].scale, "GLB Local scale preserved");
            }
    }

    void CheckProxies()
    {
        std::size_t count = 0;
        const auto check = [&](const char* owner)
        {
            Entity joint = std::string(owner) == "Gripper" ? gripper : gripper.FindChildByNameRecursive(owner);
            Entity proxy = joint.GetChild(std::string(owner) + "_CollisionProxy");
            ASSERT_TRUE_MESSAGE(static_cast<bool>(proxy), "proxy remains under owning joint");
            ASSERT_TRUE_MESSAGE(proxy.Get<RigidBody>().motionType == BodyMotionType::Kinematic &&
                    proxy.Get<RigidBody>().collisionLayer == CollisionLayer::Gripper, "proxy motion and layer unchanged");
            if (std::string(owner) == "Gripper")
                ASSERT_TRUE(proxy.Get<Colliders>().shapes.size() > 1)
                    << "separate GLB primitives retain separate gripper body hulls";
            RequireMatrix(proxy.GetWorldMatrix(), joint.GetWorldMatrix(), "proxy shares owning joint World pose");
            ++count;
        };
        check("Gripper");
        for (const auto& joint : models::robotiq::kTwoF85Joints) check(joint.name.data());
        ASSERT_TRUE_MESSAGE(count == 7, "all seven gripper proxies checked");
    }

    double Gap()
    {
        // 각 손끝의 충돌용 단순 형상 정점을 그리퍼 기준 좌표로 되돌린다. 서로 마주 보는 X 방향 경계 사이의 거리 [m]를 계산한다.
        const glm::mat4 inverseRoot = glm::inverse(gripper.GetWorldMatrix());
        float leftMaximum = -std::numeric_limits<float>::infinity();
        float rightMinimum = std::numeric_limits<float>::infinity();
        for (const char* side : {"Left", "Right"})
        {
            Entity proxy = gripper.FindChildByNameRecursive(std::string(side) + "FingerTipJoint_CollisionProxy");
            const glm::mat4 toRoot = inverseRoot * proxy.GetWorldMatrix();
            for (const auto& shape : proxy.Get<Colliders>().shapes)
                for (const auto& point : shape.pointsMeters)
                {
                    const float x = (toRoot * glm::vec4(point, 1.0F)).x;
                    if (std::string(side) == "Left") leftMaximum = std::max(leftMaximum, x);
                    else rightMinimum = std::min(rightMinimum, x);
                }
        }
        return rightMinimum - leftMaximum;
    }
};

void CheckBranchedPose(Fixture& fixture, const std::array<glm::vec3, 2>& outerBind,
    const std::array<glm::vec3, 2>& tipBind)
{
    const double q = fixture.controller.GetState().closureFraction * models::robotiq::kTwoF85NominalClosedMasterRadians;
    for (std::size_t side = 0; side < 2; ++side)
    {
        // GLB 중간 노드의 기준 회전은 항등이다. 손끝 관절의 원점은 바깥 관절의 -Z 회전에 따라 이동하는 기준 위치를 사용한다.
        // 손끝 관절 자체에 반대 방향 회전을 적용하면 원점은 그대로 두고 손끝 방향만 맞출 수 있다. 왼쪽과 오른쪽 계층은 각각 계산한다.
        const double angle = side == 0 ? -q : q;
        const glm::vec3 offset = tipBind[side] - outerBind[side];
        const glm::vec3 expected = outerBind[side] + glm::vec3{
            static_cast<float>(std::cos(angle) * offset.x - std::sin(angle) * offset.y),
            static_cast<float>(std::sin(angle) * offset.x + std::cos(angle) * offset.y), offset.z};
        Entity tip = fixture.gripper.FindChildByNameRecursive(side == 0 ? "LeftFingerTipJoint" : "RightFingerTipJoint");
        RequireVector(InGripper(tip, fixture.gripper), expected, "nested fingertip follows only its own branch");
        const glm::mat3 tipInRoot = glm::mat3(glm::inverse(fixture.gripper.GetWorldMatrix()) * tip.GetWorldMatrix());
        const glm::quat rotation = glm::normalize(glm::quat_cast(tipInRoot));
        ASSERT_TRUE_MESSAGE(std::abs(rotation.w) > 0.99999F, "opposite tip rotation cancels outer joint orientation");
    }
    fixture.CheckLocalPositions();
    fixture.CheckProxies();
}

void CheckNonidentityBindRotation()
{
    flecs::world world;
    Scene scene(world);
    Entity root = scene.CreateEntity("SyntheticGripper");
    Entity joint = scene.CreateEntity("LeftOuterKnuckleJoint");
    joint.SetParent(root);
    joint.SetLocalPosition({0.03F, 0.05F, 0.09F});
    const glm::vec3 bindEuler{0.25F, -0.4F, 0.6F};
    joint.SetLocalRotation(glm::quat{bindEuler});
    auto specification = models::robotiq::kTwoF85;
    specification.jointCount = 1;
    grasplink::simulation::robotics::GripperTransformAdapter adapter(root, specification);
    kinematics::GripperKinematicState pose;
    pose.masterAngleRadians = 0.4;
    pose.jointAnglesRadians = {0.4};
    pose.jointLocalRotations = {{std::cos(0.2), 0.0, 0.0, -std::sin(0.2)}};
    adapter.Apply(pose);
    const glm::quat expected = glm::quat(bindEuler) * glm::quat{
        static_cast<float>(std::cos(0.2)), 0.0F, 0.0F, static_cast<float>(-std::sin(0.2))};
    ASSERT_TRUE_MESSAGE(std::abs(glm::dot(glm::normalize(joint.GetLocalRotation()), glm::normalize(expected))) > 0.99999F,
        "nonidentity bind rotation multiplies delta on the right");
    RequireVector(joint.GetLocalPosition(), {0.03F, 0.05F, 0.09F}, "nonidentity bind position untouched");
    // 정상 setter를 거치지 않고 ECS root 회전을 직접 손상시킨 경우에도 adapter가 GLB 관절을 연결할 때 잘못된 기준 자세를 거부하는지 확인한다.
    Rotation& rootRotation = root.GetHandle().get_mut<Rotation, Local>();
    rootRotation.w = rootRotation.x = rootRotation.y = rootRotation.z = 0.0F;
    GTestExpectThrows<std::invalid_argument>([&]
    {
        grasplink::simulation::robotics::GripperTransformAdapter invalid(root, specification);
    }, "zero root bind quaternion rejected");
}

void CheckMotionAndHierarchy()
{
    Fixture fixture;
    const glm::mat4 mountBind = TransformSystemModule::ComposeLocalMatrix(
        fixture.gripper.GetLocalPosition(), fixture.gripper.GetLocalRotation(), fixture.gripper.GetLocalScale());
    ASSERT_TRUE_MESSAGE(glm::length(fixture.gripper.GetLocalPosition()) > 0.09F, "GLB Gripper matrix is not mistaken for identity");
    const std::array<glm::vec3, 2> outerBind{
        InGripper(fixture.gripper.FindChildByNameRecursive("LeftOuterKnuckleJoint"), fixture.gripper),
        InGripper(fixture.gripper.FindChildByNameRecursive("RightOuterKnuckleJoint"), fixture.gripper)};
    const std::array<glm::vec3, 2> tipBind{
        InGripper(fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint"), fixture.gripper),
        InGripper(fixture.gripper.FindChildByNameRecursive("RightFingerTipJoint"), fixture.gripper)};
    CheckBranchedPose(fixture, outerBind, tipBind);
    ASSERT_NEAR_FINITE(fixture.Gap(), 0.085, 2.0e-5, "authored open gap is 85 mm");

    fixture.Command(255, 0);
    fixture.Step(1);
    ASSERT_TRUE_MESSAGE(fixture.controller.GetState().actualPosition == 0 && fixture.controller.GetState().closureFraction > 0.0,
        "continuous motion precedes first 8-bit feedback increment");
    ASSERT_TRUE_MESSAGE(std::abs(fixture.gripper.FindChildByNameRecursive("LeftOuterKnuckleJoint").GetLocalRotation().z) > 1.0e-5F,
        "continuous snapshot moves geometry without raw quantization");
    fixture.Command(128);
    fixture.Step(125);
    CheckBranchedPose(fixture, outerBind, tipBind);
    const double halfGap = fixture.Gap();
    ASSERT_TRUE_MESSAGE(halfGap > 0.015 && halfGap < 0.06, "intermediate command narrows aperture");
    fixture.Command(255);
    fixture.Step(250);
    CheckBranchedPose(fixture, outerBind, tipBind);
    ASSERT_TRUE_MESSAGE(fixture.Gap() >= 0.0 && fixture.Gap() < 0.003, "nominal closed geometry preserves small asset gap");
    RequireMatrix(TransformSystemModule::ComposeLocalMatrix(fixture.gripper.GetLocalPosition(),
        fixture.gripper.GetLocalRotation(), fixture.gripper.GetLocalScale()), mountBind, "nonidentity mounting matrix preserved");

    fixture.Command(0);
    fixture.Step(35);
    ASSERT_TRUE_MESSAGE(fixture.controller.Stop().Ok(), "stop accepted during reopening");
    const double stopped = fixture.controller.GetState().closureFraction;
    fixture.Step(100);
    ASSERT_NEAR_FINITE(fixture.controller.GetState().closureFraction, stopped, 1.0e-12, "stop holds continuous position");
    CheckBranchedPose(fixture, outerBind, tipBind);

    // 팔과 모델 root 위치를 함께 바꿔도 GLB에 저장된 그리퍼 기준 변환과 좌우 관절 회전은 그대로여야 한다. 부모의 변화만 자손 World 행렬에 누적되어야 한다.
    fixture.armState.jointPositionRadians = {0.4, -0.2, 0.1, 0.25, -0.15, 0.3};
    fixture.robotRoot.SetLocalPosition({0.7F, 0.4F, -0.6F});
    fixture.robotRoot.SetLocalRotation(glm::quat{glm::vec3{0.2F, 0.1F, -0.3F}});
    fixture.Step(2);
    CheckBranchedPose(fixture, outerBind, tipBind);
    auto pose = fixture.gripperMath.Update(fixture.controller.GetState());
    std::vector<glm::quat> before;
    for (const auto& joint : models::robotiq::kTwoF85Joints)
        before.push_back(fixture.gripper.FindChildByNameRecursive(std::string(joint.name)).GetLocalRotation());
    for (int repeat = 0; repeat < 50; ++repeat) fixture.gripperAdapter->Apply(pose);
    for (std::size_t index = 0; index < before.size(); ++index)
        RequireRotation(fixture.gripper.FindChildByNameRecursive(std::string(models::robotiq::kTwoF85Joints[index].name)).GetLocalRotation(),
            before[index], "repeated apply does not accumulate rotation");

    auto bad = pose;
    bad.jointLocalRotations.back().w = std::numeric_limits<double>::quiet_NaN();
    GTestExpectThrows<std::invalid_argument>([&] { fixture.gripperAdapter->Apply(bad); }, "invalid late quaternion rejected atomically");
    for (std::size_t index = 0; index < before.size(); ++index)
        RequireRotation(fixture.gripper.FindChildByNameRecursive(std::string(models::robotiq::kTwoF85Joints[index].name)).GetLocalRotation(),
            before[index], "invalid apply leaves every joint unchanged");
    Entity finger = fixture.gripper.FindChildByNameRecursive("LeftFinger");
    const Entity parent = finger.GetParent();
    finger.SetParent(fixture.gripper);
    GTestExpectThrows<std::runtime_error>([&] { fixture.gripperAdapter->Apply(pose); }, "changed intermediate ancestry rejected");
    finger.SetParent(parent);
    fixture.gripperAdapter->Apply(pose);
    fixture.scene.reset();
    GTestExpectThrows<std::runtime_error>([&] { fixture.gripperAdapter->Apply(pose); }, "removed scene invalidates borrowed joint handles");
}

glm::vec3 SupportPoint(Entity proxy, float& top)
{
    std::vector<glm::vec3> points;
    top = -std::numeric_limits<float>::infinity();
    for (const auto& shape : proxy.Get<Colliders>().shapes)
        for (const auto& point : shape.pointsMeters)
        {
            const glm::vec3 world = glm::vec3(proxy.GetWorldMatrix() * glm::vec4(point, 1.0F));
            points.push_back(world);
            top = std::max(top, world.y);
        }
    glm::vec3 center(0.0F);
    std::size_t count = 0;
    for (const auto& point : points)
        if (top - point.y < 0.002F) { center += point; ++count; }
    EXPECT_TRUE_MESSAGE(count != 0, "tip has gravity-facing hull surface");
    if (count == 0) return center;
    return center / static_cast<float>(count);
}

void CheckPhysicalFollowing()
{
    Fixture fixture;
    // 물리 접촉 시험에서는 그리퍼를 X축 기준 +90도로 돌려 중력이 손가락이 열리고 닫히는 평면에 수직으로 작용하게 한다.
    // 부모 팔의 누적 변환을 역으로 적용해 원하는 Scene 기준 자세를 부모 기준 Local 값으로 구한다. 원본 모델이 특정 한 축에만 장착된다고 가정하지 않는다.
    const glm::mat4 parentWorld = fixture.gripper.GetParent().GetWorldMatrix();
    const glm::mat4 desiredWorld = glm::translate(glm::mat4(1.0F), {0.0F, 0.5F, 0.0F}) *
        glm::mat4_cast(glm::angleAxis(glm::half_pi<float>(), glm::vec3{1.0F, 0.0F, 0.0F}));
    const glm::mat4 local = glm::inverse(parentWorld) * desiredWorld;
    fixture.gripper.SetLocalPosition(glm::vec3(local[3]));
    fixture.gripper.SetLocalRotation(glm::quat_cast(glm::mat3(local)));
    fixture.Command(128);
    fixture.Step(125);
    Entity tip = fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint_CollisionProxy");
    float surfaceY;
    const glm::vec3 surface = SupportPoint(tip, surfaceY);
    Entity drop = fixture.scene->CreateEntity("RuntimeTipContact");
    drop.SetLocalPosition({surface.x, surfaceY + 0.07F, surface.z});
    drop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(300);
    ASSERT_TRUE_MESSAGE(drop.GetLocalPosition().y > surfaceY + 0.001F && drop.GetLocalPosition().y < surfaceY + 0.02F,
        "Jolt contacts fingertip at controller-driven intermediate pose");
    // 손끝과 이미 접촉한 물체는 마찰 때문에 함께 움직일 수 있다. 따라서 이전 접촉 위치가 아니라 새 위치에서 물체를 떨어뜨려 손끝 지지 형상이 이동했는지 독립적으로 확인한다.
    drop.Destroy();
    fixture.Command(0);
    fixture.Step(125);
    Entity oldPositionDrop = fixture.scene->CreateEntity("FormerRuntimeTipPosition");
    oldPositionDrop.SetLocalPosition({surface.x, surfaceY + 0.07F, surface.z});
    oldPositionDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(300);
    ASSERT_TRUE_MESSAGE(oldPositionDrop.GetLocalPosition().y < surfaceY - 0.04F, "old fingertip position loses support after reopening");
    fixture.CheckProxies();
    float openSurfaceY;
    const glm::vec3 openSurface = SupportPoint(tip, openSurfaceY);
    const auto pose = fixture.gripperMath.Update(fixture.controller.GetState());
    fixture.scene.reset();
    GTestExpectThrows<std::runtime_error>([&] { fixture.gripperAdapter->Apply(pose); }, "removed Scene rejects stale adapter");
    fixture.scene = std::make_unique<Scene>(fixture.world);
    Entity ghostCheck = fixture.scene->CreateEntity("NoStaleGripperBody");
    ghostCheck.SetLocalPosition({openSurface.x, openSurfaceY + 0.07F, openSurface.z});
    ghostCheck.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    for (int step = 0; step < 300; ++step)
    {
        TransformSystemModule::UpdateWorldTransforms(fixture.world);
        fixture.integration->Step(0.004);
    }
    ASSERT_TRUE_MESSAGE(ghostCheck.GetLocalPosition().y < openSurfaceY - 0.2F, "scene cleanup leaves no stale gripper collision body");
}

void CheckCandidateValidationDoesNotChangeScene()
{
    Fixture fixture;
    grasplink::simulation::RobotPhysicsAdapter armColliders(
        *fixture.scene, fixture.robotRoot, models::hanwha::kHcr12a, fixture.model);
    TransformSystemModule::UpdateWorldTransforms(fixture.world);
    fixture.integration->Step(0.004);

    backends::simulation::SimRobotController armController{models::hanwha::kHcr12a};
    ASSERT_TRUE_MESSAGE(armController.Connect().Ok(), "arm controller connects before collision validation");
    grasplink::simulation::robotics::RobotEnvironmentCollisionGuard collisionGuard{
        armController, fixture.controller, fixture.armMath, *fixture.armAdapter,
        armColliders, fixture.gripperMath, *fixture.gripperAdapter,
        fixture.world, fixture.physics, *fixture.integration, fixture.robotRoot};

    struct ScenePose
    {
        Entity entity;
        glm::mat4 local;
        glm::mat4 world;
    };
    std::vector<ScenePose> before;
    std::vector<Entity> pending{Entity{fixture.scene->GetSceneRoot()}};
    while (!pending.empty())
    {
        Entity entity = pending.back();
        pending.pop_back();
        before.push_back({entity,
            TransformSystemModule::ComposeLocalMatrix(
                entity.GetLocalPosition(), entity.GetLocalRotation(), entity.GetLocalScale()),
            entity.GetWorldMatrix()});
        const auto children = entity.GetChildren();
        pending.insert(pending.end(), children.begin(), children.end());
    }

    JointMoveCommand command;
    command.targetPositionRadians = armController.GetStateView().jointPositionRadians;
    command.targetPositionRadians[0] = 0.1;
    ASSERT_TRUE_MESSAGE(armController.MoveJoint(command).Ok(),
        "a nearby collision-free command reaches the candidate state checker");
    for (const ScenePose& pose : before)
    {
        RequireMatrix(TransformSystemModule::ComposeLocalMatrix(
                pose.entity.GetLocalPosition(), pose.entity.GetLocalRotation(), pose.entity.GetLocalScale()),
            pose.local, "candidate validation leaves every Scene local transform unchanged");
        RequireMatrix(pose.entity.GetWorldMatrix(), pose.world,
            "candidate validation leaves every Scene world transform unchanged");
    }
}

void CheckCandidateProxyTransformsMatchScene()
{
    Fixture fixture;
    fixture.robotRoot.SetLocalPosition({0.31F, 0.08F, -0.27F});
    fixture.robotRoot.SetLocalRotation(glm::angleAxis(0.63F, glm::normalize(glm::vec3{0.2F, 1.0F, -0.3F})));

    grasplink::simulation::RobotPhysicsAdapter armColliders(
        *fixture.scene, fixture.robotRoot, models::hanwha::kHcr12a, fixture.model);
    TransformSystemModule::UpdateWorldTransforms(fixture.world);
    fixture.integration->Step(0.004);

    const glm::mat4 rootWorld = fixture.robotRoot.GetWorldMatrix();
    const auto buildRobotCandidates = [&](const grasplink::robotics::RobotState& state,
                                          std::vector<grasplink::simulation::RobotPhysicsAdapter::CandidateWorldTransform>& output)
    {
        armColliders.BuildCandidateWorldTransforms(fixture.armMath.Update(state), rootWorld, output);
    };

    std::vector<grasplink::simulation::RobotPhysicsAdapter::CandidateWorldTransform> initialRobotCandidates;
    buildRobotCandidates(fixture.armState, initialRobotCandidates);
    const auto link6Candidate = std::find_if(initialRobotCandidates.begin(), initialRobotCandidates.end(),
        [](auto candidate)
        {
            return candidate.entity.Has<RobotCollisionProxy>() &&
                candidate.entity.Get<RobotCollisionProxy>().linkIndex == 5;
        });
    ASSERT_TRUE_MESSAGE(link6Candidate != initialRobotCandidates.end(), "initial Link6 candidate exists");
    const glm::mat4 gripperRootInLink6 = glm::inverse(link6Candidate->worldTransform) * fixture.gripper.GetWorldMatrix();

    fixture.armState.jointPositionRadians = {
        1.21, -0.44, 0.73, -1.08, 0.92, 2.31};
    const auto robotState = fixture.armMath.Update(fixture.armState);
    fixture.armAdapter->Apply(robotState);
    armColliders.Apply(robotState);
    TransformSystemModule::UpdateWorldTransforms(fixture.world);

    std::vector<grasplink::simulation::RobotPhysicsAdapter::CandidateWorldTransform> robotCandidates;
    armColliders.BuildCandidateWorldTransforms(robotState, fixture.robotRoot.GetWorldMatrix(), robotCandidates);
    for (auto& candidate : robotCandidates)
        RequireMatrix(candidate.worldTransform, candidate.entity.GetWorldMatrix(),
            "candidate robot collider frame matches the live Scene frame at a nonzero joint pose");

    for (auto& candidate : robotCandidates)
    {
        if (!candidate.entity.Has<RobotCollisionProxy>() ||
            candidate.entity.Get<RobotCollisionProxy>().linkIndex == RobotCollisionProxy::InvalidLinkIndex)
            continue;

        const std::size_t linkIndex = candidate.entity.Get<RobotCollisionProxy>().linkIndex;
        const std::string linkName(models::hanwha::kHcr12a.links[linkIndex].name);
        std::vector<glm::vec3> renderedVertices;
        for (std::size_t nodeIndex = 0; nodeIndex < fixture.model.nodes.size(); ++nodeIndex)
        {
            const auto& node = fixture.model.nodes[nodeIndex];
            if (node.meshIndex < 0)
                continue;

            std::size_t ancestor = nodeIndex;
            bool belongsToLink = false;
            while (true)
            {
                const auto& ancestorNode = fixture.model.nodes[ancestor];
                if (ancestorNode.name == linkName)
                {
                    belongsToLink = true;
                    break;
                }
                const bool isMovingJoint = std::any_of(
                    models::hanwha::kHcr12a.joints,
                    models::hanwha::kHcr12a.joints + models::hanwha::kHcr12a.jointCount,
                    [&](const auto& joint) { return joint.name == ancestorNode.name; });
                if (isMovingJoint || ancestorNode.name == "Gripper" || ancestorNode.parentIndex < 0)
                    break;
                ancestor = static_cast<std::size_t>(ancestorNode.parentIndex);
            }
            if (!belongsToLink)
                continue;

            const auto& mesh = fixture.model.meshes[static_cast<std::size_t>(node.meshIndex)];
            const glm::mat4 meshWorld = fixture.nodes[nodeIndex].GetWorldMatrix();
            for (const auto& vertex : mesh.vertices)
                renderedVertices.emplace_back(meshWorld * glm::vec4(vertex.position, 1.0F));
        }
        ASSERT_FALSE(renderedVertices.empty()) << "rendered mesh vertices exist for " << linkName;

        const auto& shapes = candidate.entity.Get<Colliders>().shapes;
        for (const auto& shape : shapes)
            for (const glm::vec3& point : shape.pointsMeters)
            {
                const glm::vec3 worldPoint = glm::vec3(candidate.worldTransform * glm::vec4(point, 1.0F));
                const bool matchesRenderedVertex = std::any_of(
                    renderedVertices.begin(), renderedVertices.end(),
                    [&](const glm::vec3& vertex)
                    {
                        const glm::vec3 difference = vertex - worldPoint;
                        return glm::dot(difference, difference) < 1.0e-8F;
                    });
                ASSERT_TRUE(matchesRenderedVertex)
                    << linkName << " collider point is not in the rendered mesh world frame: "
                    << worldPoint.x << ", " << worldPoint.y << ", " << worldPoint.z;
            }
    }

    const auto link6 = std::find_if(robotCandidates.begin(), robotCandidates.end(),
        [](auto candidate)
        {
            return candidate.entity.Has<RobotCollisionProxy>() &&
                candidate.entity.Get<RobotCollisionProxy>().linkIndex == 5;
        });
    ASSERT_TRUE_MESSAGE(link6 != robotCandidates.end(), "posed Link6 candidate exists");
    const glm::mat4 candidateGripperRoot = link6->worldTransform * gripperRootInLink6;

    std::vector<Entity> gripperProxies;
    std::vector<Entity> pending{fixture.robotRoot};
    while (!pending.empty())
    {
        Entity entity = pending.back();
        pending.pop_back();
        if (entity.Has<GripperCollisionProxy>())
            gripperProxies.push_back(entity);
        const auto children = entity.GetChildren();
        pending.insert(pending.end(), children.begin(), children.end());
    }
    ASSERT_FALSE(gripperProxies.empty()) << "gripper collision proxies exist";

    std::vector<glm::mat4> candidateGripperTransforms;
    fixture.gripperAdapter->BuildCandidateWorldTransforms(
        fixture.gripperMath.Update(fixture.controller.GetState()), candidateGripperRoot,
        gripperProxies, candidateGripperTransforms);
    ASSERT_EQ(candidateGripperTransforms.size(), gripperProxies.size());
    for (std::size_t index = 0; index < gripperProxies.size(); ++index)
    {
        RequireMatrix(candidateGripperTransforms[index], gripperProxies[index].GetWorldMatrix(),
            "candidate gripper collider frame matches the live Scene frame at a nonzero arm pose");

        const char* ownerName = gripperProxies[index].GetParent().GetHandle().name();
        ASSERT_NE(ownerName, nullptr);
        std::vector<glm::vec3> renderedVertices;
        for (std::size_t nodeIndex = 0; nodeIndex < fixture.model.nodes.size(); ++nodeIndex)
        {
            const auto& node = fixture.model.nodes[nodeIndex];
            if (node.meshIndex < 0)
                continue;

            std::size_t ancestor = nodeIndex;
            bool belongsToOwner = false;
            while (true)
            {
                const auto& ancestorNode = fixture.model.nodes[ancestor];
                if (ancestorNode.name == ownerName)
                {
                    belongsToOwner = true;
                    break;
                }
                if (ancestorNode.parentIndex < 0)
                    break;
                ancestor = static_cast<std::size_t>(ancestorNode.parentIndex);
            }
            if (!belongsToOwner)
                continue;

            const auto& mesh = fixture.model.meshes[static_cast<std::size_t>(node.meshIndex)];
            const glm::mat4 meshWorld = fixture.nodes[nodeIndex].GetWorldMatrix();
            for (const auto& vertex : mesh.vertices)
                renderedVertices.emplace_back(meshWorld * glm::vec4(vertex.position, 1.0F));
        }
        ASSERT_FALSE(renderedVertices.empty()) << "rendered mesh vertices exist for " << ownerName;

        for (const auto& shape : gripperProxies[index].Get<Colliders>().shapes)
            for (const glm::vec3& point : shape.pointsMeters)
            {
                const glm::vec3 worldPoint = glm::vec3(candidateGripperTransforms[index] * glm::vec4(point, 1.0F));
                const bool matchesRenderedVertex = std::any_of(
                    renderedVertices.begin(), renderedVertices.end(),
                    [&](const glm::vec3& vertex)
                    {
                        const glm::vec3 difference = vertex - worldPoint;
                        return glm::dot(difference, difference) < 1.0e-8F;
                    });
                ASSERT_TRUE(matchesRenderedVertex)
                    << ownerName << " collider point is not in the rendered mesh world frame: "
                    << worldPoint.x << ", " << worldPoint.y << ", " << worldPoint.z;
            }
    }
}

}

/** @brief 실제 GLB의 갈라진 관절 계층에서 연속 개폐가 화면 Entity와 Jolt 충돌 형상에 같은 자세로 반영되는지 확인한다. */
TEST(GripperPoseIntegration, NonidentityBindRotationAndInvalidRoot)
{
    CheckNonidentityBindRotation();
}

TEST(GripperPoseIntegration, MotionHierarchyAndAtomicPoseApplication)
{
    CheckMotionAndHierarchy();
}

TEST(GripperPoseIntegration, ColliderFollowsGripperAndSceneCleanup)
{
    CheckPhysicalFollowing();
}

TEST(GripperPoseIntegration, CandidateCollisionValidationLeavesSceneUnchanged)
{
    CheckCandidateValidationDoesNotChangeScene();
}

TEST(GripperPoseIntegration, CandidateProxyTransformsMatchScene)
{
    CheckCandidateProxyTransformsMatchScene();
}
