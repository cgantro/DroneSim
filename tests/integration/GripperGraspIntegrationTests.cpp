#include "scene/Entity.h"
#include "physics/PhysicsWorld.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "simulation/robotics/GripperGraspAdapter.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "scene/TransformSystemModule.h"
#include "TestSupport.h"
#include "model/GltfLoader.h"
#include "robotics/kinematics/GripperKinematics.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/robotics/GripperTransformAdapter.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotTransformAdapter.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <memory>
#include <filesystem>
#include <functional>
#include <vector>
#include <limits>

#include <gtest/gtest.h>

namespace
{
using grasplink::model::GltfLoader;
using grasplink::model::ModelResource;
using grasplink::model::NodeData;
using grasplink::scene::Entity;
using grasplink::scene::Scene;
using grasplink::scene::TransformSystemModule;

using namespace grasplink::physics;
using grasplink::robotics::backends::simulation::SimGripperController;
using grasplink::robotics::models::robotiq::kTwoF85;

struct TestCheckFailure {};

void CheckImpl(bool condition, const std::string& label, const char* file, int line)
{
    if (condition) return;
    ADD_FAILURE_AT(file, line) << label;
    throw TestCheckFailure{};
}

void CheckNearImpl(double actual, double expected, double tolerance, const std::string& label,
    const char* file, int line)
{
    if (std::isfinite(actual) && std::abs(actual - expected) <= tolerance) return;
    const std::string detail = label + ": actual=" + std::to_string(actual) +
        ", expected=" + std::to_string(expected) + ", tolerance=" + std::to_string(tolerance);
    ADD_FAILURE_AT(file, line) << detail;
    throw TestCheckFailure{};
}

#define Check(...) CheckImpl(__VA_ARGS__, __FILE__, __LINE__)
#define CheckNear(...) CheckNearImpl(__VA_ARGS__, __FILE__, __LINE__)

template<typename Exception, typename Function>
void CheckThrows(Function&& function, const std::string& label)
{
    try
    {
        function();
        Check(false, label);
    }
    catch (const TestCheckFailure&)
    {
        throw;
    }
    catch (const Exception&)
    {
    }
}

CollisionShapeDescription SphereLikeHull(float radius)
{
    constexpr float goldenRatio = 1.61803398875F;
    const float scale = radius / std::sqrt(1.0F + goldenRatio * goldenRatio);
    CollisionShapeDescription shape;
    shape.type = CollisionShapeType::ConvexHull;
    for (const float signA : {-1.0F, 1.0F})
        for (const float signB : {-1.0F, 1.0F})
        {
            shape.pointsMeters.emplace_back(0.0F, signA * scale, signB * goldenRatio * scale);
            shape.pointsMeters.emplace_back(signA * scale, signB * goldenRatio * scale, 0.0F);
            shape.pointsMeters.emplace_back(signA * goldenRatio * scale, 0.0F, signB * scale);
        }
    return shape;
}

/**
 * @brief 양쪽 Box 손끝과 Dynamic 상자로 실제 Jolt 접촉이 발생하는 작은 장면을 구성한다.
 * @details 화면 모델의 복잡한 기구와 별개로 접촉 판정, Controller 정지, 물리 연결, Body 수명을 검증한다.
 * root만 움직여도 본체와 손끝이 함께 이동하고 상자는 Scene 부모 없이 Jolt constraint로 따라가야 한다.
 */
struct Fixture
{
    flecs::world world;
    PhysicsWorld physics;
    SimGripperController controller{kTwoF85};
    Entity root, anchor, left, right, object;
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> system;
    std::unique_ptr<Scene> scene;
    std::unique_ptr<grasplink::simulation::GripperGraspAdapter> grasp;

    explicit Fixture(bool unilateral = false, bool sameSide = false)
    {
        scene = std::make_unique<Scene>(world);
        root = scene->CreateEntity("RobotRoot");
        anchor = Box(*scene, "AnchorProxy", {0.0F, 1.0F, 0.8F}, {0.04F, 0.04F, 0.04F}, BodyMotionType::Kinematic);
        left = Box(*scene, "LeftTipProxy", {-0.145F, sameSide ? 0.93F : 1.0F, 0.0F},
            {0.05F, sameSide ? 0.06F : 0.2F, 0.15F}, BodyMotionType::Kinematic);
        right = Box(*scene, "RightTipProxy", {unilateral ? 0.6F : sameSide ? -0.145F : 0.145F, sameSide ? 1.07F : 1.0F, 0.0F},
            {0.05F, sameSide ? 0.06F : 0.2F, 0.15F}, BodyMotionType::Kinematic);
        anchor.set<GripperCollisionProxy>({GripperCollisionPart::Body});
        left.set<GripperCollisionProxy>({GripperCollisionPart::LeftFingerTip});
        right.set<GripperCollisionProxy>({GripperCollisionPart::RightFingerTip});
        anchor.SetParent(root);
        left.SetParent(root);
        right.SetParent(root);
        object = Box(*scene, "Object", {0.0F, 1.0F, 0.0F}, {0.1F, 0.1F, 0.1F}, BodyMotionType::Dynamic);
        TransformSystemModule::UpdateWorldTransforms(world);
        system = CreateTestPhysicsSystem(world, physics);
        Check(controller.Connect().Ok() && controller.Activate().Ok(), "controller activated");
        Check(controller.Command({255, 255, 128}).Ok(), "close command accepted");
        grasp = std::make_unique<grasplink::simulation::GripperGraspAdapter>(physics, *system, controller);
        Check(grasp->Bind(root), "synthetic finger proxies bound");
    }

    Entity Box(Scene& scene, const char* name, glm::vec3 position, glm::vec3 halfSize, BodyMotionType type)
    {
        Entity entity = scene.CreateEntity(name);
        entity.SetLocalPosition(position);
        entity.set<RigidBody>({type, type == BodyMotionType::Dynamic ? CollisionLayer::DynamicObject : CollisionLayer::Gripper})
            .set<Colliders>({{physics_colliders::Box(halfSize)}});
        return entity;
    }

    void Step(int count = 1)
    {
        for (int tick = 0; tick < count; ++tick)
        {
            grasp->BeforePhysicsStep();
            controller.Update(0.004);
            TransformSystemModule::UpdateWorldTransforms(world);
            system->Step(0.004);
            grasp->AfterPhysicsStep();
            TransformSystemModule::UpdateWorldTransforms(world);
        }
    }
};

void CheckCarryAndOpen()
{
    Fixture fixture;
    fixture.Step();
    Check(fixture.grasp->GetState().leftContact && fixture.grasp->GetState().rightContact, "both real Jolt fingers contact object");
    Check(fixture.grasp->GetState().grasped, "bilateral opposing contacts create grasp");
    Check(fixture.controller.GetState().objectStatus == grasplink::robotics::GripperObjectStatus::ContactWhileClosing,
        "contact stops closing controller");
    const double closure = fixture.controller.GetState().closureFraction;
    Check(fixture.controller.Command({255, 255, 128}).Ok(), "repeated close accepted");
    fixture.Step(3);
    CheckNear(fixture.controller.GetState().closureFraction, closure, 0.0, "repeated close preserves contact stop");
    const float initialY = fixture.object.GetLocalPosition().y;
    for (int tick = 1; tick <= 100; ++tick)
    {
        fixture.root.SetLocalPosition({0.0F, 0.002F * tick, 0.0F});
        fixture.Step();
    }
    CheckNear(fixture.object.GetLocalPosition().y - initialY, 0.2, 0.015, "dynamic object follows moving gripper via constraint");
    Check(fixture.controller.Command({0, 255, 128}).Ok(), "open command accepted");
    fixture.grasp->BeforePhysicsStep();
    Check(!fixture.grasp->GetState().grasped, "opening releases before physics");
    const float releasedY = fixture.object.GetLocalPosition().y;
    fixture.Step(150);
    Check(fixture.object.GetLocalPosition().y < releasedY - 0.5F, "released object falls under gravity");
}

void CheckRejectedContacts()
{
    Fixture unilateral(true);
    unilateral.Step();
    Check(unilateral.grasp->GetState().leftContact && !unilateral.grasp->GetState().rightContact, "unilateral real contact reported");
    Check(!unilateral.grasp->GetState().grasped, "unilateral contact cannot grasp");
    Check(unilateral.controller.GetState().mode == grasplink::robotics::GripperMode::Moving,
        "one finger touching first does not stop the other finger from closing");
    Fixture sameSide(false, true);
    sameSide.Step();
    Check(sameSide.grasp->GetState().leftContact && sameSide.grasp->GetState().rightContact, "both fingers hit same object side");
    Check(!sameSide.grasp->GetState().grasped, "same direction contacts cannot grasp");

    Fixture differentObjects;
    differentObjects.object.Remove<Colliders>();
    differentObjects.Box(*differentObjects.scene, "LeftObject", {-0.07F, 1.0F, 0.0F}, {0.03F, 0.1F, 0.1F}, BodyMotionType::Dynamic);
    differentObjects.Box(*differentObjects.scene, "RightObject", {0.07F, 1.0F, 0.0F}, {0.03F, 0.1F, 0.1F}, BodyMotionType::Dynamic);
    differentObjects.Step();
    Check(differentObjects.grasp->GetState().leftContact && differentObjects.grasp->GetState().rightContact,
        "both fingers contact different objects");
    Check(!differentObjects.grasp->GetState().grasped, "different object contacts cannot create grasp");
}

void CheckLifetimeAndRelease()
{
    Fixture deleted;
    deleted.Step();
    const auto handle = deleted.grasp->GetState().object;
    Check(handle.IsValid(), "held object has valid handle");
    deleted.object.Destroy();
    Check(!deleted.physics.IsBodyValid(handle), "deleted entity destroys held body");
    deleted.Step();
    Check(!deleted.grasp->GetState().grasped, "deleted body clears grasp safely");

    Fixture rebuilt;
    rebuilt.Step();
    const auto oldTip = rebuilt.system->GetBodyHandle(rebuilt.left.GetHandle());
    rebuilt.left.set<Colliders>({{physics_colliders::Box({0.05F, 0.2F, 0.15F})}});
    rebuilt.Step();
    Check(!rebuilt.physics.IsBodyValid(oldTip), "collider replacement invalidates tip body");
    Check(!rebuilt.grasp->GetState().grasped, "tip body rebuild releases old grasp");

    Fixture reset;
    reset.Step();
    Check(reset.controller.Reset().Ok() && reset.controller.Activate().Ok(), "reset followed immediately by activate");
    reset.grasp->BeforePhysicsStep();
    Check(!reset.grasp->GetState().grasped, "reset release survives immediate activate");

    Fixture disconnected;
    disconnected.Step();
    disconnected.controller.Disconnect();
    disconnected.grasp->BeforePhysicsStep();
    Check(!disconnected.grasp->GetState().grasped, "disconnect releases grasp");

    Fixture manual;
    manual.Step();
    manual.grasp->Release();
    manual.Step();
    Check(!manual.grasp->GetState().grasped, "manual release does not immediately regrasp old close request");
    Check(manual.controller.Command({255, 255, 128}).Ok(), "fresh close rearms grasp");
    manual.Step();
    Check(manual.grasp->GetState().grasped, "fresh command can grasp again");

    Fixture replacedScene;
    replacedScene.Step();
    replacedScene.scene.reset();
    replacedScene.grasp->BeforePhysicsStep();
    Check(!replacedScene.grasp->GetState().grasped, "scene replacement removes body connections safely");
}

void CheckSleepingAndHandleOwnership()
{
    PhysicsWorld world;
    BoxBodyDescription floor;
    floor.motionType = BodyMotionType::Static;
    floor.collisionLayer = CollisionLayer::Environment;
    floor.halfExtentsMeters = {1.0F, 0.1F, 1.0F};
    const auto ground = world.CreateBox(floor);
    BoxBodyDescription box;
    box.transform.position = {0.0F, 0.8F, 0.0F};
    box.halfExtentsMeters = {0.1F, 0.1F, 0.1F};
    const auto falling = world.CreateBox(box);
    for (int tick = 0; tick < 900; ++tick) world.Step(0.004);
    Check(!world.GetContacts().empty(), "resting sleeping body retains contact snapshot");
    floor.transform.position.x = 5.0F;
    world.SetBodyTransform(ground, floor.transform);
    world.Step(0.004);
    Check(world.GetContacts().empty(), "teleported sleeping support loses old contact snapshot");

    PhysicsWorld other;
    Check(!other.IsBodyValid(falling), "foreign world body handle rejected");
    CheckThrows<std::invalid_argument>([&] { other.CreateFixedConstraint(ground, falling); }, "foreign constraint handles rejected");
    BoxBodyDescription anchorDescription;
    anchorDescription.motionType = BodyMotionType::Kinematic;
    anchorDescription.collisionLayer = CollisionLayer::Gripper;
    anchorDescription.transform.position = {3.0F, 2.0F, 0.0F};
    const auto anchor = world.CreateBox(anchorDescription);
    const auto connection = world.CreateFixedConstraint(anchor, falling);
    Check(world.IsConstraintValid(connection) && !other.IsConstraintValid(connection), "constraint handle includes owner world");
    other.DestroyConstraint(connection);
    Check(world.IsConstraintValid(connection), "foreign release preserves owner constraint");
    world.DestroyBody(anchor);
    Check(!world.IsConstraintValid(connection), "anchor deletion removes constraint first");
    world.DestroyConstraint(connection);
}

void CheckGripperEnvironmentOverlapQuery()
{
    PhysicsWorld world;
    BoxBodyDescription floor;
    floor.motionType = BodyMotionType::Static;
    floor.collisionLayer = CollisionLayer::Environment;
    floor.halfExtentsMeters = {1.0F, 0.1F, 1.0F};
    const auto floorHandle = world.CreateBox(floor);

    BoxBodyDescription gripper;
    gripper.motionType = BodyMotionType::Kinematic;
    gripper.collisionLayer = CollisionLayer::Gripper;
    gripper.halfExtentsMeters = {0.1F, 0.1F, 0.1F};
    gripper.transform.position = {0.0F, 0.3F, 0.0F};
    const auto gripperHandle = world.CreateBox(gripper);
    Check(!world.OverlapsEnvironmentAt(gripperHandle, gripper.transform), "gripper starts above environment without overlap");
    gripper.transform.position.y = 0.15F;
    Check(world.OverlapsEnvironmentAt(gripperHandle, gripper.transform),
        "kinematic gripper target overlapping the floor is detected before movement");
    Check(world.GetCollisionLayer(floorHandle) == CollisionLayer::Environment,
        "floor remains classified as the environment");
}

void CheckOffsetCenterOfMassConstraint()
{
    PhysicsWorld world;
    BodyDescription anchorDescription;
    anchorDescription.motionType = BodyMotionType::Kinematic;
    anchorDescription.collisionLayer = CollisionLayer::Gripper;
    anchorDescription.transform.position = {1.0F, 2.0F, 0.0F};
    CollisionShapeDescription anchorShape;
    anchorShape.halfExtentsMeters = {0.1F, 0.1F, 0.1F};
    anchorShape.localTransform.position = {0.3F, 0.0F, 0.0F};
    anchorDescription.shapes.push_back(anchorShape);
    const auto anchor = world.CreateBody(anchorDescription);
    BodyDescription objectDescription;
    objectDescription.transform.position = {0.0F, 2.0F, 0.0F};
    CollisionShapeDescription objectShape;
    objectShape.halfExtentsMeters = {0.05F, 0.05F, 0.05F};
    objectShape.localTransform.position = {0.1F, 0.0F, 0.0F};
    objectDescription.shapes.push_back(objectShape);
    const auto object = world.CreateBody(objectDescription);
    const auto connection = world.CreateFixedConstraint(anchor, object);
    // 두 Body의 모델 원점과 무게중심(COM)을 일부러 어긋나게 둔다. 기준 Body를 회전하면 물체 모델 원점도 처음 상대 자세를 보존하며 원을 그려야 한다.
    Transform target = anchorDescription.transform;
    target.rotation = glm::angleAxis(0.5F, glm::vec3{0.0F, 0.0F, 1.0F});
    for (int tick = 1; tick <= 50; ++tick)
    {
        Transform next = anchorDescription.transform;
        next.rotation = glm::angleAxis(0.01F * tick, glm::vec3{0.0F, 0.0F, 1.0F});
        world.MoveKinematic(anchor, next, 0.004);
        world.Step(0.004);
    }
    world.StopKinematic(anchor);
    for (int tick = 0; tick < 10; ++tick) world.Step(0.004);
    const glm::vec3 expected = target.position + target.rotation * (objectDescription.transform.position - anchorDescription.transform.position);
    const auto actual = world.GetBodyTransform(object);
    CheckNear(glm::length(actual.position - expected), 0.0, 0.008, "fixed grasp preserves model origins despite offset COM");
    CheckNear(std::abs(glm::dot(actual.rotation, target.rotation)), 1.0, 0.002, "fixed grasp preserves relative rotation");
    world.DestroyConstraint(connection);
}

void CheckAuthoredGripperContacts()
{
    flecs::world world;
    PhysicsWorld physics;
    Scene scene{world};
    const ModelResource model = LoadTestGlb("HCR12A_2F-85.glb");
    Entity root = scene.CreateEntity("AuthoredRobotRoot");
    int gripperIndex = -1;
    for (std::size_t node = 0; node < model.nodes.size(); ++node)
        if (model.nodes[node].name == "Gripper") gripperIndex = static_cast<int>(node);
    Check(gripperIndex >= 0, "authored GLB contains Gripper");
    std::function<Entity(int, const Entity&)> createTree = [&](int index, const Entity& parent)
    {
        const auto& node = model.nodes.at(static_cast<std::size_t>(index));
        Entity entity = scene.CreateEntity(node.name);
        entity.SetParent(parent);
        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);
        for (const int child : node.childrenIndices) createTree(child, entity);
        return entity;
    };
    Entity gripper = createTree(gripperIndex, root);
    SimGripperController controller{kTwoF85};
    Check(controller.Connect().Ok() && controller.Activate().Ok() && controller.Command({255, 255, 128}).Ok(),
        "authored gripper closing command accepted");
    controller.Update(kTwoF85.nominalMasterClosedRadians * 0.5);
    grasplink::robotics::kinematics::GripperKinematics math{kTwoF85};
    grasplink::simulation::robotics::GripperTransformAdapter transform{gripper, kTwoF85};
    transform.Apply(math.Update(controller.GetState()));
    grasplink::simulation::ConfigureTwoF85Colliders(scene, root, model);
    TransformSystemModule::UpdateWorldTransforms(world);

    // 실제 손끝 외피 정점을 Scene 좌표로 모아 두 손끝 사이에 들어갈 구형 ConvexHull 시험 물체를 배치한다.
    // 두 외피의 평균점 방향으로 투영한 안쪽 면 사이 간격에 6 mm 여유 겹침을 더해 첫 물리 계산에서 확실한 양쪽 접촉을 만들며, 이후 이동은 실제 constraint로 검증한다.
    auto hullPoints = [&](const char* name)
    {
        Entity proxy = root.FindChildByNameRecursive(name);
        Check(static_cast<bool>(proxy), "authored finger collision proxy exists");
        std::vector<glm::vec3> points;
        for (const auto& shape : proxy.Get<Colliders>().shapes)
            for (const auto& point : shape.pointsMeters)
                points.push_back(glm::vec3(proxy.GetWorldMatrix() * glm::vec4(point, 1.0F)));
        Check(!points.empty(), "authored finger hull has vertices");
        return points;
    };
    const auto left = hullPoints("LeftFingerTipJoint_CollisionProxy");
    const auto right = hullPoints("RightFingerTipJoint_CollisionProxy");
    auto mean = [](const auto& points)
    {
        glm::vec3 result{0.0F};
        for (const auto& point : points) result += point;
        return result / static_cast<float>(points.size());
    };
    const glm::vec3 leftCenter = mean(left);
    const glm::vec3 rightCenter = mean(right);
    const glm::vec3 axis = glm::normalize(rightCenter - leftCenter);
    float leftFace = -std::numeric_limits<float>::infinity();
    float rightFace = std::numeric_limits<float>::infinity();
    for (const auto& point : left) leftFace = std::max(leftFace, glm::dot(point, axis));
    for (const auto& point : right) rightFace = std::min(rightFace, glm::dot(point, axis));
    glm::vec3 center = (leftCenter + rightCenter) * 0.5F;
    center += axis * ((leftFace + rightFace) * 0.5F - glm::dot(center, axis));
    const float radius = (rightFace - leftFace) * 0.5F + 0.006F;
    Check(radius > 0.0F, "authored open tips have positive test object half-size");
    Entity object = scene.CreateEntity("AuthoredGripObject");
    object.SetLocalPosition(center);
    object.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{SphereLikeHull(radius)}});
    TransformSystemModule::UpdateWorldTransforms(world);
    grasplink::simulation::PhysicsSystemModule system{world, physics};
    grasplink::simulation::GripperGraspAdapter grasp{physics, system, controller};
    Check(grasp.Bind(root), "authored proxies bind grasp adapter");
    auto step = [&]
    {
        grasp.BeforePhysicsStep();
        controller.Update(0.004);
        transform.Apply(math.Update(controller.GetState()));
        TransformSystemModule::UpdateWorldTransforms(world);
        system.Step(0.004);
        grasp.AfterPhysicsStep();
        TransformSystemModule::UpdateWorldTransforms(world);
    };
    step();
    Check(grasp.GetState().leftContact && grasp.GetState().rightContact && grasp.GetState().grasped,
        "actual GLB hull contacts create bilateral grasp");
    const glm::vec3 initial = object.GetLocalPosition();
    for (int tick = 1; tick <= 60; ++tick)
    {
        root.SetLocalPosition({0.0F, 0.001F * tick, 0.0F});
        step();
    }
    CheckNear(object.GetLocalPosition().y - initial.y, 0.06, 0.015, "actual GLB grasp carries dynamic object");
    Check(controller.Command({0, 255, 128}).Ok(), "authored gripper opens");
    step();
    Check(!grasp.GetState().grasped, "actual GLB opening removes constraint");
    Check(controller.GetState().goToActive, "separating authored fingers do not stop on previous closing contact");
}

/**
 * @brief 실제 HCR-12A 관절 Controller의 TCP 이동이 GLB와 Jolt 파지 물체까지 전달되는 전체 경로를 검증한다.
 * @details Robot root를 직접 옮기지 않고 MovePose와 MoveLinear이 계산한 관절각으로 팔과 그리퍼를 움직인다.
 * TCP는 ToolFrame에서 X로 40 mm 떨어진 시험 기준점이며 목표는 Robot base 기준이다.
 * 실제 손끝 외피의 반대 접촉으로 파지한 뒤 매 고정 간격에서 물체와 그리퍼의 상대 위치·회전을 확인하고 마지막에 열기 명령으로 해제한다.
 */
void CheckRobotControllerCarriesAuthoredGrasp()
{
    using namespace grasplink::robotics;
    using models::hanwha::kHcr12a;
    flecs::world world;
    PhysicsWorld physics;
    Scene scene{world};
    const ModelResource model = LoadTestGlb("HCR12A_2F-85.glb");
    Check(model.rootNodeIndex >= 0, "full robot GLB has authored root");
    std::function<Entity(int, const Entity&)> createTree = [&](int index, const Entity& parent)
    {
        const auto& node = model.nodes.at(static_cast<std::size_t>(index));
        Entity entity = scene.CreateEntity(node.name);
        entity.SetParent(parent);
        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);
        for (const int child : node.childrenIndices) createTree(child, entity);
        return entity;
    };
    Entity root = createTree(model.rootNodeIndex, Entity{scene.GetSceneRoot()});
    Entity gripper = root.FindChildByNameRecursive("Gripper");
    Check(static_cast<bool>(gripper), "full robot GLB includes gripper hierarchy");
    TransformSystemModule::UpdateWorldTransforms(world);
    const Entity toolFrame = root.FindChildByNameRecursive("ToolFrame");
    const glm::mat4 worldToGripper = glm::inverse(gripper.GetWorldMatrix());
    glm::vec3 tcpMidpoint{0.0F};
    // Viewer는 실제 GLB 손끝 Mesh 중심으로 TCP를 정하므로, 이 검사도 임의 TCP offset 대신 같은 계산을 사용한다.
    for (const char* name : {"LeftFingerTipMesh", "RightFingerTipMesh"})
    {
        const auto node = std::find_if(model.nodes.begin(), model.nodes.end(), [name](const NodeData& value) { return value.name == name; });
        const Entity meshEntity = root.FindChildByNameRecursive(name);
        Check(node != model.nodes.end() && meshEntity.IsValid(), "viewer TCP fingertip mesh exists");
        glm::vec3 minimum(std::numeric_limits<float>::max());
        glm::vec3 maximum(std::numeric_limits<float>::lowest());
        const glm::mat4 meshToGripper = worldToGripper * meshEntity.GetWorldMatrix();
        for (const auto& vertex : model.meshes.at(static_cast<std::size_t>(node->meshIndex)).vertices)
        {
            const glm::vec3 point(meshToGripper * glm::vec4(vertex.position, 1.0F));
            minimum = glm::min(minimum, point);
            maximum = glm::max(maximum, point);
        }
        tcpMidpoint += (minimum + maximum) * 0.25F;
    }
    const glm::mat4 gripperToTool = glm::inverse(toolFrame.GetWorldMatrix()) * gripper.GetWorldMatrix();
    const glm::vec3 tcpOffsetPosition(gripperToTool * glm::vec4(tcpMidpoint, 1.0F));
    const glm::quat tcpOffsetRotation = glm::normalize(glm::quat_cast(glm::mat3(gripperToTool)));
    const models::Pose3 viewerTcpOffset{
        {tcpOffsetPosition.x, tcpOffsetPosition.y, tcpOffsetPosition.z},
        {tcpOffsetRotation.w, tcpOffsetRotation.x, tcpOffsetRotation.y, tcpOffsetRotation.z}};
    BoxBodyDescription testFloor;
    testFloor.motionType = BodyMotionType::Static;
    testFloor.collisionLayer = CollisionLayer::Environment;
    testFloor.halfExtentsMeters = {3.0F, 0.02F, 3.0F};
    testFloor.transform.position = {0.0F, -0.015F, 0.0F};
    physics.CreateBox(testFloor);
    // 충돌 필터 없이 목표 자세를 먼저 검사해, 목표 자체가 도달 불가능한 문제와 이동 경로가 막히는 문제를 구분한다.
    kinematics::DampedLeastSquaresIk viewerIk{kHcr12a, viewerTcpOffset};
    CartesianPose approachTarget = viewerIk.EvaluateTcp({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    const glm::vec3 boxInBase = glm::vec3(glm::inverse(root.GetWorldMatrix()) * glm::vec4{0.35F, 0.276F, 0.65F, 1.0F});
    approachTarget.positionMeters = {boxInBase.x, boxInBase.y, boxInBase.z};
    const glm::dquat currentOrientation{
        tcpOffsetRotation.w, tcpOffsetRotation.x, tcpOffsetRotation.y, tcpOffsetRotation.z};
    const glm::dquat faceDown = glm::angleAxis(glm::half_pi<double>(), glm::dvec3{1.0, 0.0, 0.0}) * currentOrientation;
    approachTarget.orientationXyzw = {faceDown.x, faceDown.y, faceDown.z, faceDown.w};
    const glm::dvec3 jawAxis = faceDown * glm::dvec3{1.0, 0.0, 0.0};
    const glm::dvec3 approachAxis = faceDown * glm::dvec3{0.0, 1.0, 0.0};
    Check(std::abs(jawAxis.y) < 1.0e-6 && std::abs(approachAxis.y + 1.0) < 1.0e-6,
        "face-down orientation keeps the jaw opening horizontal and points the approach axis toward the floor");
    Check(viewerIk.Solve(approachTarget, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}).Ok(),
        "face-down viewer gripper reaches the high approach point above the box");
    for (const glm::vec2 candidate : {glm::vec2{0.31F, 0.59F}, glm::vec2{0.31F, 0.71F},
             glm::vec2{0.39F, 0.59F}, glm::vec2{0.39F, 0.71F}, glm::vec2{0.10F, 0.65F}})
    {
        CartesianPose candidateTarget = approachTarget;
        const glm::vec3 candidateInBase = glm::vec3(glm::inverse(root.GetWorldMatrix()) *
            glm::vec4{candidate.x, 0.276F, candidate.y, 1.0F});
        candidateTarget.positionMeters = {candidateInBase.x, candidateInBase.y, candidateInBase.z};
        Check(viewerIk.Solve(candidateTarget, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}).Ok(),
            "every random pickup boundary and the placement target is reachable from the home seed");
    }
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    backends::simulation::SimRobotController robot{kHcr12a, tcpOffset};
    SimGripperController controller{kTwoF85};
    Check(robot.Connect().Ok(), "full pipeline arm controller connected");
    JointMoveCommand seed;
    seed.targetPositionRadians = {0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    Check(robot.MoveJoint(seed).Ok(), "full pipeline regular pickup seed accepted");
    // 실제 파지 물체를 생성하기 전에 정칙 관절 자세로 준비한다. 정칙 자세는 작은 TCP 이동에 안정적인 관절 해를 찾을 수 있는 시작 상태다.
    for (int tick = 0; tick < 2000 && robot.GetState().mode == RobotMode::Moving; ++tick) robot.Update(0.004);
    Check(robot.GetState().mode == RobotMode::Idle, "full pipeline regular pickup seed reached");
    Check(controller.Connect().Ok() && controller.Activate().Ok() && controller.Command({255, 255, 128}).Ok(),
        "full pipeline close command accepted");
    controller.Update(kTwoF85.nominalMasterClosedRadians * 0.5);
    kinematics::RobotKinematics robotMath{kHcr12a};
    kinematics::GripperKinematics gripperMath{kTwoF85};
    grasplink::simulation::robotics::RobotTransformAdapter robotTransform{root, kHcr12a};
    grasplink::simulation::robotics::GripperTransformAdapter gripperTransform{gripper, kTwoF85};
    grasplink::simulation::RobotPhysicsAdapter robotColliders{scene, root, kHcr12a, model};
    auto applyPoses = [&]
    {
        const auto& armState = robotMath.Update(robot.GetState());
        robotTransform.Apply(armState);
        robotColliders.Apply(armState);
        gripperTransform.Apply(gripperMath.Update(controller.GetState()));
        TransformSystemModule::UpdateWorldTransforms(world);
    };
    applyPoses();
    grasplink::simulation::ConfigureTwoF85Colliders(scene, root, model);
    TransformSystemModule::UpdateWorldTransforms(world);

    auto hullPoints = [&](const char* name)
    {
        Entity proxy = root.FindChildByNameRecursive(name);
        Check(static_cast<bool>(proxy), "full pipeline authored finger proxy exists");
        std::vector<glm::vec3> points;
        for (const auto& shape : proxy.Get<Colliders>().shapes)
            for (const auto& point : shape.pointsMeters)
                points.push_back(glm::vec3(proxy.GetWorldMatrix() * glm::vec4(point, 1.0F)));
        Check(!points.empty(), "full pipeline authored finger hull has vertices");
        return points;
    };
    const auto leftPoints = hullPoints("LeftFingerTipJoint_CollisionProxy");
    const auto rightPoints = hullPoints("RightFingerTipJoint_CollisionProxy");
    auto mean = [](const auto& points)
    {
        glm::vec3 result{0.0F};
        for (const auto& point : points) result += point;
        return result / static_cast<float>(points.size());
    };
    const glm::vec3 leftCenter = mean(leftPoints);
    const glm::vec3 rightCenter = mean(rightPoints);
    const glm::vec3 axis = glm::normalize(rightCenter - leftCenter);
    float leftFace = -std::numeric_limits<float>::infinity();
    float rightFace = std::numeric_limits<float>::infinity();
    for (const auto& point : leftPoints) leftFace = std::max(leftFace, glm::dot(point, axis));
    for (const auto& point : rightPoints) rightFace = std::min(rightFace, glm::dot(point, axis));
    glm::vec3 center = (leftCenter + rightCenter) * 0.5F;
    center += axis * ((leftFace + rightFace) * 0.5F - glm::dot(center, axis));
    const float radius = (rightFace - leftFace) * 0.5F + 0.006F;
    Check(radius > 0.0F, "full pipeline test object half-size is positive");
    Entity object = scene.CreateEntity("RobotControllerHeldObject");
    object.SetLocalPosition(center);
    object.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{SphereLikeHull(radius)}});
    TransformSystemModule::UpdateWorldTransforms(world);
    grasplink::simulation::PhysicsSystemModule system{world, physics};
    grasplink::simulation::GripperGraspAdapter grasp{physics, system, controller};
    Check(grasp.Bind(root), "full pipeline grasp binds authored robot");
    auto step = [&]
    {
        // 앱과 같은 순서로 해제 확인, 두 Controller, FK, World 변환, Jolt 계산, 접촉 feedback을 실행한다.
        grasp.BeforePhysicsStep();
        robot.Update(0.004);
        controller.Update(0.004);
        applyPoses();
        system.Step(0.004);
        grasp.AfterPhysicsStep();
        TransformSystemModule::UpdateWorldTransforms(world);
    };
    step();
    Check(grasp.GetState().leftContact && grasp.GetState().rightContact && grasp.GetState().grasped,
        "regular arm pickup pose creates genuine bilateral authored contact");
    const Entity baseProxy = root.FindChildByNameRecursive("Base_CollisionProxy");
    Check(baseProxy.IsValid(), "robot base collision proxy exists for excluding its connected Link1 bearing");
    const auto baseBody = system.GetBodyHandle(baseProxy.GetHandle());
    Check(baseBody.IsValid(), "robot base collision body exists before floor clearance checks");
    for (const double clearance : {0.005, 0.010, 0.020, 0.025, 0.030, 0.040, 0.050})
    {
        CartesianPose candidate = approachTarget;
        candidate.positionMeters[1] = 0.026 + clearance;
        const auto solution = viewerIk.Solve(candidate, robot.GetState().jointPositionRadians);
        Check(solution.Ok(), "face-down pickup height remains IK-reachable");
        RobotState candidateState = robot.GetState();
        candidateState.jointPositionRadians = solution.jointPositionRadians;
        const auto& candidateKinematics = robotMath.Update(candidateState);
        robotTransform.Apply(candidateKinematics);
        robotColliders.Apply(candidateKinematics);
        gripperTransform.Apply(gripperMath.Update(controller.GetState()));
        TransformSystemModule::UpdateWorldTransforms(world);
        bool gripperOverlaps = false;
        bool robotArmOverlaps = false;
        std::vector<Entity> pending{root};
        while (!pending.empty())
        {
            Entity entity = pending.back();
            pending.pop_back();
            if (entity.Has<RigidBody>())
            {
                const CollisionLayer layer = entity.Get<RigidBody>().collisionLayer;
                if (layer != CollisionLayer::Robot && layer != CollisionLayer::Gripper)
                {
                    const auto children = entity.GetChildren();
                    pending.insert(pending.end(), children.begin(), children.end());
                    continue;
                }
                const auto handle = system.GetBodyHandle(entity.GetHandle());
                const auto matrix = entity.GetWorldMatrix();
                const Transform target{glm::vec3(matrix[3]), glm::normalize(glm::quat_cast(glm::mat3(matrix)))};
                const bool isLink1 = entity.GetHandle().name() == "Link1_CollisionProxy";
                const bool overlapsEnvironment = handle.IsValid() && (isLink1
                    ? physics.OverlapsEnvironmentAt(handle, target, baseBody)
                    : physics.OverlapsEnvironmentAt(handle, target));
                if (overlapsEnvironment)
                {
                    gripperOverlaps = gripperOverlaps || layer == CollisionLayer::Gripper;
                    robotArmOverlaps = robotArmOverlaps || layer == CollisionLayer::Robot;
                }
            }
            const auto children = entity.GetChildren();
            pending.insert(pending.end(), children.begin(), children.end());
        }
        Check(gripperOverlaps == (clearance < 0.020),
            "finger collision hulls overlap the floor below 20 mm clearance and clear it at 20 mm");
        Check(!robotArmOverlaps,
            "robot arm links clear the floor at every sampled pickup height after the Link1 bearing contact is excluded");
        applyPoses();
    }
    const auto objectHandle = system.GetBodyHandle(object.GetHandle());
    const Entity anchorEntity = root.FindChildByNameRecursive("Gripper_CollisionProxy");
    const auto anchorHandle = system.GetBodyHandle(anchorEntity.GetHandle());
    const auto initialAnchor = physics.GetBodyTransform(anchorHandle);
    const auto initialObject = physics.GetBodyTransform(objectHandle);
    const glm::vec3 initialRelativePosition = glm::inverse(initialAnchor.rotation) * (initialObject.position - initialAnchor.position);
    const glm::quat initialRelativeRotation = glm::inverse(initialAnchor.rotation) * initialObject.rotation;
    auto advanceHeldMotion = [&]
    {
        int movingTicks = 0;
        for (; movingTicks < 2000 && robot.GetState().mode == RobotMode::Moving; ++movingTicks)
        {
            step();
            Check(grasp.GetState().grasped, "arm Cartesian motion maintains actual grasp connection");
            const auto anchorPose = physics.GetBodyTransform(anchorHandle);
            const auto objectPose = physics.GetBodyTransform(objectHandle);
            const glm::vec3 relativePosition = glm::inverse(anchorPose.rotation) * (objectPose.position - anchorPose.position);
            const glm::quat relativeRotation = glm::inverse(anchorPose.rotation) * objectPose.rotation;
            CheckNear(glm::length(relativePosition - initialRelativePosition), 0.0, 0.012,
                "arm Cartesian motion preserves held object relative position");
            CheckNear(std::abs(glm::dot(relativeRotation, initialRelativeRotation)), 1.0, 0.001,
                "arm Cartesian motion preserves held object relative rotation");
        }
        Check(movingTicks > 1, "Cartesian grasp motion executes intermediate simulation states");
        Check(robot.GetState().mode == RobotMode::Idle, "Cartesian grasp motion finishes without runtime IK fault or stall");
    };
    kinematics::DampedLeastSquaresIk targetMath{kHcr12a, tcpOffset};
    const CartesianPose poseTarget = targetMath.EvaluateTcp({0.23, -0.365, 0.32, 0.19, -0.24, 0.105});
    Check(robot.MovePose(poseTarget, 0.25).Ok(), "held-object MovePose accepts regular reachable target");
    advanceHeldMotion();
    const CartesianPose linearTarget = targetMath.EvaluateTcp({0.25, -0.37, 0.34, 0.17, -0.23, 0.11});
    Check(robot.MoveLinear({linearTarget, 0.025, 0.2}).Ok(), "held-object MoveLinear accepts regular reachable path");
    advanceHeldMotion();
    Check(glm::length(physics.GetBodyTransform(anchorHandle).position - initialAnchor.position) > 0.005F,
        "Cartesian robot commands move actual gripper through scene");
    Check(glm::length(physics.GetBodyTransform(objectHandle).position - initialObject.position) > 0.005F,
        "Cartesian robot commands transport actual Dynamic object");
    const glm::vec3 rootOrigin = glm::vec3(root.GetWorldMatrix()[3]);
    CheckNear(glm::length(rootOrigin - glm::vec3(model.nodes.at(static_cast<std::size_t>(model.rootNodeIndex)).translation)),
        0.0, 1.0e-6, "robot root placement stays unchanged during controller transport");
    Check(controller.Command({0, 255, 128}).Ok(), "full pipeline open release accepted");
    grasp.BeforePhysicsStep();
    Check(!grasp.GetState().grasped, "full pipeline open releases transported object before Jolt step");
    step();
    Check(!grasp.GetState().grasped && physics.IsBodyValid(objectHandle), "transported object remains Dynamic after release");
}
}

TEST(GripperGrasp, CarriesAndReleasesOnOpen) { CheckCarryAndOpen(); }
TEST(GripperGrasp, RejectsInvalidContacts) { CheckRejectedContacts(); }
TEST(GripperGrasp, ReleasesOnLifetimeAndControllerChanges) { CheckLifetimeAndRelease(); }
TEST(GripperGrasp, TracksSleepingBodiesAndHandleOwnership) { CheckSleepingAndHandleOwnership(); }
TEST(GripperGrasp, QueriesGripperEnvironmentOverlap) { CheckGripperEnvironmentOverlapQuery(); }
TEST(GripperGrasp, ConstrainsOffsetCenterOfMass) { CheckOffsetCenterOfMassConstraint(); }
TEST(GripperGrasp, UsesAuthoredGripperContacts) { CheckAuthoredGripperContacts(); }
TEST(GripperGrasp, CarriesAuthoredGraspThroughRobotController) { CheckRobotControllerCarriesAuthoredGrasp(); }
