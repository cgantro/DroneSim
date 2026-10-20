#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/models/GripperSpecification.h"
#include "model/GltfLoader.h"
#include "model/RobotModel.h"
#include "scene/Scene.h"
#include "physics/PhysicsWorld.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "simulation/robotics/RobotModelAdapter.h"
#include "simulation/robotics/RobotTransformAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotCollisionGeometryBuilder.h"
#include "simulation/robotics/GripperTransformAdapter.h"
#include "simulation/robotics/RobotEnvironmentCollisionGuard.h"
#include "simulation/robotics/RobotSelfCollisionPolicy.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "scene/TransformSystemModule.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr std::array<grasplink::robotics::models::GripperJointSpecification, 1>
    kCollisionTestGripperJoints{{
        {"TestGripperJoint", {}, {0.0, 0.0, 1.0}, 1.0, 0.0, 0.8, true}}};
constexpr grasplink::robotics::models::GripperSpecification kCollisionTestGripper{
    "Test", "CollisionTest", 0, 255, 0, 255, 0, 255, 0.8,
    kCollisionTestGripperJoints.data(), kCollisionTestGripperJoints.size()};

constexpr std::size_t kMaximumHcrCollisionShapeCount = 150;
constexpr std::size_t kExpectedHcrCollisionProxyCount = 7;

using grasplink::model::RobotCollisionGeometry;
using grasplink::model::RobotCollisionType;
using grasplink::model::RobotLink;
using grasplink::model::RobotModel;
using grasplink::scene::Entity;
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


RobotLink BoxLink(const char* name, const glm::dvec3& size)
{
    RobotLink link;
    link.name = name;
    RobotCollisionGeometry geometry;
    geometry.type = RobotCollisionType::Box;
    geometry.dimensionsMeters = size;
    link.collisionGeometry.push_back(geometry);
    return link;
}

void CheckSyntheticGeometry()
{
    using namespace grasplink::robotics;
    RobotModel model;
    model.name = "TwoLink";
    model.baseLink = 0;
    RobotLink link1 = BoxLink("Link1", {0.1, 0.1, 0.1});
    RobotCollisionGeometry sphere;
    sphere.type = RobotCollisionType::Sphere;
    sphere.dimensionsMeters = glm::dvec3{0.05};
    sphere.positionMeters = glm::dvec3{0.0, 0.0, 0.1};
    link1.collisionGeometry.push_back(sphere);
    RobotCollisionGeometry cylinder;
    cylinder.type = RobotCollisionType::Cylinder;
    cylinder.dimensionsMeters = glm::dvec3{0.025, 0.1, 0.0};
    link1.collisionGeometry.push_back(cylinder);
    model.links = {BoxLink("base_link", {0.2, 0.2, 0.2}),
        BoxLink("BaseSpacer", {0.05, 0.05, 0.05}), std::move(link1),
        BoxLink("FixedSpacer", {0.06, 0.06, 0.06}), BoxLink("Link2", {0.1, 0.1, 0.1}),
        RobotLink{"tool0"}};
    model.toolLink = 5;
    model.activeJoints = {1, 3};
    model.joints = {
        {"base_fixed", grasplink::model::RobotJointType::Fixed, 0, 1,
            {0.0, 0.1, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {"J1", grasplink::model::RobotJointType::Revolute, 1, 2,
            {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, -3.1416, 3.1416, 1.0},
        {"spacer_fixed", grasplink::model::RobotJointType::Fixed, 2, 3,
            {0.0, 0.25, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {"J2", grasplink::model::RobotJointType::Prismatic, 3, 4,
            {0.0, 0.25, 0.0}, {0.7071067811865476, 0.7071067811865476, 0.0, 0.0},
            {0.0, 0.0, 1.0}, -3.1416, 3.1416, 1.0},
        {"tool_fixed", grasplink::model::RobotJointType::Fixed, 4, 5,
            {0.0, 0.5, 0.0}, {1.0, 0.0, 0.0, 0.0}}};
    auto sourceModel = std::make_unique<RobotModel>(model);
    sourceModel->name = "Synthetic model name owned by the adapter";
    sourceModel->links[2].name = "Synthetic link name owned by the adapter";
    grasplink::simulation::robotics::RobotModelAdapter modelAdapter(*sourceModel);
    sourceModel.reset();
    const models::RobotSpecification& spec = modelAdapter.Specification();
    ASSERT_TRUE(spec.manufacturer.empty());
    EXPECT_EQ(spec.model, "Synthetic model name owned by the adapter");
    EXPECT_EQ(spec.links[0].name, "Synthetic link name owned by the adapter");
    const auto builtGeometry = grasplink::simulation::robotics::collision_geometry::Build(model);
    auto unsupportedShapeModel = model;
    unsupportedShapeModel.links[2].collisionGeometry[0].type =
        static_cast<RobotCollisionType>(255);
    GTestExpectThrows<std::invalid_argument>([&]
    {
        (void)grasplink::simulation::robotics::collision_geometry::Build(unsupportedShapeModel);
    }, "unknown collision shapes are rejected instead of approximated as spheres");
    ASSERT_TRUE_MESSAGE(builtGeometry.links.size() == 3, "fixed links keep their own collision shapes");
    ASSERT_TRUE_MESSAGE(builtGeometry.links[0].jointIndex == 0 && builtGeometry.links[1].jointIndex == 0 &&
        builtGeometry.links[2].jointIndex == 1,
        "a fixed link follows the nearest moving joint");
    ASSERT_TRUE_MESSAGE(builtGeometry.links[0].shapes.size() == 3 &&
        builtGeometry.links[1].shapes.size() == 1 && builtGeometry.links[2].shapes.size() == 1,
        "URDF collision shapes are prepared before Scene entities are created");
    ASSERT_TRUE_MESSAGE(builtGeometry.links[0].shapes[0].type == grasplink::physics::CollisionShapeType::Box &&
        builtGeometry.links[0].shapes[1].type == grasplink::physics::CollisionShapeType::ConvexHull &&
        builtGeometry.links[0].shapes[2].type == grasplink::physics::CollisionShapeType::ConvexHull,
        "URDF keeps boxes and approximates spheres and cylinders with convex hulls");
    ASSERT_TRUE_MESSAGE(builtGeometry.baseShapes.size() == 2,
        "a fixed base child shares the static base collision body");
    ASSERT_NEAR_FINITE(builtGeometry.baseShapes[1].localTransform.position.y, 0.1, 1e-5,
        "fixed base collision keeps its URDF offset");
    flecs::world world;
    auto sceneOwner = std::make_unique<Scene>(world);
    Scene& scene = *sceneOwner;
    Entity root = scene.CreateEntity("RobotRoot");
    Entity baseSpacerVisual = scene.CreateEntity("BaseSpacer");
    Entity joint1 = scene.CreateEntity("J1");
    Entity link1Visual = scene.CreateEntity("Link1Visual");
    Entity fixedSpacerVisual = scene.CreateEntity("FixedSpacer");
    Entity joint2 = scene.CreateEntity("J2");
    Entity link2Visual = scene.CreateEntity("Link2Visual");
    baseSpacerVisual.SetParent(root);
    baseSpacerVisual.SetLocalPosition({0.0F, 0.1F, 0.0F});
    joint1.SetParent(baseSpacerVisual);
    link1Visual.SetParent(joint1);
    fixedSpacerVisual.SetParent(link1Visual);
    joint2.SetParent(fixedSpacerVisual);
    link2Visual.SetParent(joint2);
    fixedSpacerVisual.SetLocalPosition({0.0F, 0.25F, 0.0F});
    joint2.SetLocalPosition({0.0F, 0.25F, 0.0F});
    joint2.SetLocalRotation({0.70710678F, 0.70710678F, 0.0F, 0.0F});
    grasplink::simulation::robotics::RobotTransformAdapter visualAdapter(root, spec);
    grasplink::simulation::RobotPhysicsAdapter adapter(scene, root, model);
    std::size_t syntheticLinkIndex = 0;
    for (const char* name : {"Link1_CollisionProxy", "FixedSpacer_CollisionProxy", "Link2_CollisionProxy"})
    {
        Entity proxy = root.GetChild(name);
        ASSERT_TRUE_MESSAGE(proxy.IsValid(), "collision proxy created");
        const auto& colliders = proxy.Get<Colliders>();
        ASSERT_TRUE_MESSAGE(!colliders.shapes.empty(), std::string(name) + ": own geometry produces colliders");
        ASSERT_TRUE_MESSAGE(colliders.shapes.size() == builtGeometry.links[syntheticLinkIndex].shapes.size(),
            "adapter installs the builder's generated shape set");
        ASSERT_TRUE_MESSAGE(colliders.shapes.front().type == grasplink::physics::CollisionShapeType::Box,
            "the physics proxy keeps the shape declared in URDF");
        ++syntheticLinkIndex;
    }
    kinematics::RobotKinematics fk(spec);
    RobotState state;
    state.valid = true;
    state.jointPositions = {1.5707963267948966, 0.05};
    const auto& fkState = fk.Update(state);
    adapter.Apply(fkState);
    visualAdapter.Apply(fkState);
    TransformSystemModule::UpdateWorldTransforms(world);
    ASSERT_NEAR_FINITE(fkState.jointAxesInBaseFrame[1].x, 1.0, 1e-5,
        "the rotated URDF origin and first joint turn the second local Z axis toward base +X");
    const glm::mat4 joint2World = joint2.GetWorldMatrix();
    ASSERT_NEAR_FINITE(joint2World[3].x, -0.45, 1e-5,
        "the GLB hierarchy follows FK and slides along the rotated URDF axis");
    ASSERT_NEAR_FINITE(joint2.GetLocalPosition().y, 0.2, 1e-5,
        "the prismatic GLB joint keeps its bind offset and adds its travel distance");
    ASSERT_NEAR_FINITE(joint2.GetLocalRotation().x, 0.70710678, 1e-5,
        "the GLB joint keeps its bind rotation before applying joint motion");
    const auto link2 = root.GetChild("Link2_CollisionProxy");
    ASSERT_NEAR_FINITE(link2.GetLocalPosition().x, -0.45, 1e-5, "upstream joints move proxy pivot");
    ASSERT_NEAR_FINITE(link2.GetLocalPosition().y, 0.1, 1e-5, "proxy follows FK rather than bind geometry");
    const auto fixedSpacer = root.GetChild("FixedSpacer_CollisionProxy");
    ASSERT_NEAR_FINITE(fixedSpacer.GetLocalPosition().x, -0.25, 1e-5,
        "fixed-link collision follows its nearest moving parent");
    ASSERT_NEAR_FINITE(fixedSpacer.GetLocalPosition().y, 0.1, 1e-5,
        "fixed-link collision keeps the base placement");
    grasplink::physics::PhysicsWorld physics;
    grasplink::simulation::PhysicsSystemModule physicsSystem(world, physics);
    physicsSystem.Step(0.004);
    TransformSystemModule::UpdateWorldTransforms(world);
    const auto fixedSpacerBody = physicsSystem.GetBodyHandle(fixedSpacer.GetHandle());
    ASSERT_TRUE_MESSAGE(physics.IsBodyValid(fixedSpacerBody),
        "the fixed-link URDF shape is registered as a physics body");
    auto incompleteFkState = fkState;
    incompleteFkState.linkPosesInBaseFrame.resize(1);
    incompleteFkState.linkPosesInBaseFrame[0].positionMeters.x += 1.0;
    const Entity firstProxy = root.GetChild("Link1_CollisionProxy");
    const glm::vec3 stableProxyPosition = firstProxy.GetLocalPosition();
    GTestExpectThrows<std::invalid_argument>([&] { adapter.Apply(incompleteFkState); },
        "all FK poses are checked before any collision proxy moves");
    EXPECT_EQ(firstProxy.GetLocalPosition(), stableProxyPosition);
    std::vector<grasplink::simulation::RobotPhysicsAdapter::CandidateWorldTransform>
        incompleteCandidates;
    GTestExpectThrows<std::invalid_argument>([&]
    {
        adapter.BuildCandidateWorldTransforms(incompleteFkState, glm::mat4{1.0F}, incompleteCandidates);
    }, "candidate transforms reject incomplete FK poses");
    EXPECT_TRUE(incompleteCandidates.empty());
    sceneOwner.reset();
    GTestExpectThrows<std::runtime_error>([&] { adapter.Apply(fk.Update(state)); }, "removed Scene invalidates physics proxy bindings");
}

void CheckLoadedArmPipeline(std::size_t axisCount)
{
    using namespace grasplink;
    const auto modelDirectory = std::filesystem::path("tests/assets/robots") /
        ("axis" + std::to_string(axisCount));
    auto loaded = model::RobotModelLoader::Load(modelDirectory);
    if (axisCount == 6)
    {
        // 고정 오프셋 뒤의 tool0에만 충돌 도형을 두어 관절 프레임과 proxy 프레임을 다르게 만든다.
        const auto& finalJoint = loaded.robot.joints[loaded.robot.activeJoints.back()];
        loaded.robot.links[finalJoint.childLink].collisionGeometry.clear();
        RobotCollisionGeometry toolCollision;
        toolCollision.type = RobotCollisionType::Box;
        toolCollision.dimensionsMeters = glm::dvec3{0.04};
        loaded.robot.links[loaded.robot.toolLink].collisionGeometry.push_back(toolCollision);
    }
    simulation::robotics::RobotModelAdapter modelAdapter(loaded.robot);
    const auto& specification = modelAdapter.Specification();

    flecs::world world;
    Scene scene(world);
    std::vector<Entity> visualNodes(loaded.visual.nodes.size());
    for (std::size_t i = 0; i < loaded.visual.nodes.size(); ++i)
    {
        const auto& source = loaded.visual.nodes[i];
        Entity node = scene.CreateEntity(source.name);
        node.SetLocalPosition(source.translation);
        node.SetLocalRotation(source.rotation);
        node.SetLocalScale(source.scale);
        visualNodes[i] = node;
    }
    for (std::size_t i = 0; i < loaded.visual.nodes.size(); ++i)
    {
        const auto parentIndex = loaded.visual.nodes[i].parentIndex;
        if (parentIndex)
            visualNodes[i].SetParent(visualNodes[*parentIndex]);
    }

    Entity robotRoot = visualNodes[*loaded.visual.rootNodeIndex];
    simulation::robotics::RobotTransformAdapter visualAdapter(robotRoot, specification);
    simulation::RobotPhysicsAdapter collisionAdapter(scene, robotRoot, loaded.robot);
    robotics::kinematics::RobotKinematics kinematics(specification);

    robotics::RobotState state;
    state.valid = true;
    state.jointPositions.assign(specification.jointCount, 0.0);
    visualAdapter.Apply(kinematics.Update(state));
    collisionAdapter.Apply(kinematics.Update(state));
    TransformSystemModule::UpdateWorldTransforms(world);

    const auto& firstJoint = loaded.robot.joints[loaded.robot.activeJoints.front()];
    Entity jointEntity = robotRoot.FindChildByNameRecursive(firstJoint.name);
    Entity firstLinkProxy = robotRoot.GetChild(
        loaded.robot.links[firstJoint.childLink].name + "_CollisionProxy");
    ASSERT_TRUE_MESSAGE(jointEntity && firstLinkProxy, "loaded arm creates visual and collision bindings");
    const glm::quat initialJointRotation = jointEntity.GetLocalRotation();
    const glm::quat initialProxyRotation = glm::normalize(glm::quat_cast(
        glm::mat3{firstLinkProxy.GetWorldMatrix()}));

    state.jointPositions.front() = 0.15;
    const auto& movedState = kinematics.Update(state);
    visualAdapter.Apply(movedState);
    collisionAdapter.Apply(movedState);
    TransformSystemModule::UpdateWorldTransforms(world);

    ASSERT_GT(1.0F - std::abs(glm::dot(initialJointRotation, jointEntity.GetLocalRotation())),
        1.0e-5F);
    const glm::quat movedProxyRotation = glm::normalize(glm::quat_cast(
        glm::mat3{firstLinkProxy.GetWorldMatrix()}));
    ASSERT_GT(1.0F - std::abs(glm::dot(initialProxyRotation, movedProxyRotation)),
        1.0e-5F);

    physics::PhysicsWorld physicsWorld;
    simulation::PhysicsSystemModule physicsSystem(world, physicsWorld);
    physicsSystem.Step(0.004);
    Entity baseProxy = robotRoot.GetChild("Base_CollisionProxy");
    ASSERT_TRUE_MESSAGE(baseProxy && baseProxy.Has<Colliders>() && baseProxy.Has<RigidBody>(),
        "the URDF base creates a physical collision proxy");
    ASSERT_TRUE_MESSAGE(physicsWorld.IsBodyValid(physicsSystem.GetBodyHandle(baseProxy.GetHandle())),
        "the URDF base collision proxy reaches the physics engine");
    for (const std::size_t jointIndex : loaded.robot.activeJoints)
    {
        const auto& joint = loaded.robot.joints[jointIndex];
        if (loaded.robot.links[joint.childLink].collisionGeometry.empty())
            continue;
        Entity proxy = robotRoot.GetChild(loaded.robot.links[joint.childLink].name + "_CollisionProxy");
        ASSERT_TRUE_MESSAGE(proxy && proxy.Has<Colliders>() && proxy.Has<RigidBody>(),
            "each URDF arm link creates a physical collision proxy");
        ASSERT_TRUE_MESSAGE(physicsWorld.IsBodyValid(physicsSystem.GetBodyHandle(proxy.GetHandle())),
            "each URDF collision proxy reaches the physics engine");
    }

    robotics::backends::simulation::SimRobotController controller(specification);
    ASSERT_TRUE(controller.Connect());
    const auto& homeState = controller.GetStateView();
    const auto& homeKinematics = kinematics.Update(homeState);
    visualAdapter.Apply(homeKinematics);
    collisionAdapter.Apply(homeKinematics);

    std::unique_ptr<robotics::backends::simulation::SimGripperController> gripperController;
    std::unique_ptr<robotics::kinematics::GripperKinematics> gripperKinematics;
    std::unique_ptr<simulation::robotics::GripperTransformAdapter> gripperTransformAdapter;
    std::optional<simulation::robotics::RobotEnvironmentCollisionGuard::GripperSupport> gripperSupport;
    Entity gripperRoot;
    Entity gripperBodyProxy;
    if (axisCount == 6)
    {
        gripperController = std::make_unique<robotics::backends::simulation::SimGripperController>(
            kCollisionTestGripper);
        ASSERT_TRUE(gripperController->Connect());
        ASSERT_TRUE(gripperController->Activate());
        gripperKinematics = std::make_unique<robotics::kinematics::GripperKinematics>(
            kCollisionTestGripper);

        gripperRoot = scene.CreateEntity("TestGripperRoot");
        gripperRoot.SetParent(robotRoot);
        const auto& toolPosition = homeKinematics.toolFrameInBaseFrame.positionMeters;
        gripperRoot.SetLocalPosition(glm::vec3{static_cast<float>(toolPosition.x),
            static_cast<float>(toolPosition.y), static_cast<float>(toolPosition.z)});
        const auto& toolRotation = homeKinematics.toolFrameInBaseFrame.rotation;
        gripperRoot.SetLocalRotation(glm::quat{static_cast<float>(toolRotation.w),
            static_cast<float>(toolRotation.x), static_cast<float>(toolRotation.y),
            static_cast<float>(toolRotation.z)});
        Entity gripperJoint = scene.CreateEntity("TestGripperJoint");
        gripperJoint.SetParent(gripperRoot);
        gripperTransformAdapter = std::make_unique<simulation::robotics::GripperTransformAdapter>(
            gripperRoot, kCollisionTestGripper);

        gripperBodyProxy = scene.CreateEntity("GripperBody_CollisionProxy");
        gripperBodyProxy.SetParent(gripperRoot);
        gripperBodyProxy.SetLocalPosition({0.25F, 0.0F, 0.0F});
        gripperBodyProxy.set<GripperCollisionProxy>({GripperCollisionPart::Body})
            .set<RigidBody>({physics::BodyMotionType::Kinematic, physics::CollisionLayer::Gripper});
        physics::CollisionShapeDescription gripperShape;
        gripperShape.type = physics::CollisionShapeType::Box;
        gripperShape.halfExtentsMeters = glm::vec3{0.02F};
        gripperBodyProxy.set<Colliders>({{gripperShape}});
        const auto& gripperState = gripperKinematics->Update(gripperController->GetState());
        gripperTransformAdapter->Apply(gripperState);
        gripperSupport.emplace(simulation::robotics::RobotEnvironmentCollisionGuard::GripperSupport{
            gripperRoot, *gripperController, *gripperKinematics, *gripperTransformAdapter});
    }
    TransformSystemModule::UpdateWorldTransforms(world);
    physicsSystem.Step(0.004);
    TransformSystemModule::UpdateWorldTransforms(world);

    Entity gripperObstacle;
    if (gripperSupport)
    {
        // 로봇 팔이 아니라 tool0에 붙은 그리퍼만 겹치도록 장애물 위치를 잡는다.
        const glm::vec3 bodyPosition{gripperBodyProxy.GetWorldMatrix()[3]};
        gripperObstacle = scene.CreateEntity("GripperFrameObstacle");
        gripperObstacle.SetLocalPosition(bodyPosition);
        gripperObstacle.set<RigidBody>({physics::BodyMotionType::Static, physics::CollisionLayer::Environment});
        physics::CollisionShapeDescription obstacleShape;
        obstacleShape.type = physics::CollisionShapeType::Box;
        obstacleShape.halfExtentsMeters = glm::vec3{0.02F};
        gripperObstacle.set<Colliders>({{obstacleShape}});
        TransformSystemModule::UpdateWorldTransforms(world);
        physicsSystem.Step(0.004);
        TransformSystemModule::UpdateWorldTransforms(world);
    }

    simulation::robotics::RobotEnvironmentCollisionGuard collisionGuard(
        controller, kinematics, visualAdapter, collisionAdapter,
        world, physicsWorld, physicsSystem, robotRoot, std::move(gripperSupport));

    if (gripperObstacle)
    {
        const auto blockedPose = controller.MovePose(controller.GetStateView().tcpPose);
        ASSERT_FALSE(blockedPose);
        ASSERT_EQ(blockedPose.code, robotics::ErrorCode::EnvironmentContact);
        gripperObstacle.Destroy();
        TransformSystemModule::UpdateWorldTransforms(world);
        physicsSystem.Step(0.004);
        TransformSystemModule::UpdateWorldTransforms(world);
    }

    constexpr double targetAngleRadians = 1.0;
    robotics::JointMoveCommand command;
    command.targetPositions = controller.GetStateView().jointPositions;
    command.targetPositions.front() = targetAngleRadians;
    ASSERT_TRUE(controller.MoveJoint(command));

    const auto& targetKinematics = kinematics.Update(command.targetPositions);
    std::vector<simulation::RobotPhysicsAdapter::CandidateWorldTransform> candidateTransforms;
    collisionAdapter.BuildCandidateWorldTransforms(
        targetKinematics, robotRoot.GetWorldMatrix(), candidateTransforms);
    const auto& movingJoint = loaded.robot.joints[loaded.robot.activeJoints[1]];
    Entity movingLinkProxy = robotRoot.GetChild(
        loaded.robot.links[movingJoint.childLink].name + "_CollisionProxy");
    const auto targetLink = std::find_if(candidateTransforms.begin(), candidateTransforms.end(),
        [&](const auto& candidate) { return candidate.entity == movingLinkProxy; });
    ASSERT_NE(targetLink, candidateTransforms.end());
    const auto& movingCollisionShape = movingLinkProxy.Get<Colliders>().shapes.front();
    const glm::vec3 obstaclePosition{targetLink->worldTransform *
        glm::vec4{movingCollisionShape.localTransform.position, 1.0F}};

    Entity obstacle = scene.CreateEntity("CollisionStopObstacle");
    obstacle.SetLocalPosition(obstaclePosition);
    obstacle.set<RigidBody>({physics::BodyMotionType::Static, physics::CollisionLayer::Environment});
    physics::CollisionShapeDescription obstacleShape;
    obstacleShape.type = physics::CollisionShapeType::Box;
    obstacleShape.halfExtentsMeters = glm::vec3{0.02F};
    obstacle.set<Colliders>({{obstacleShape}});
    TransformSystemModule::UpdateWorldTransforms(world);
    physicsSystem.Step(0.004);

    bool stoppedByObstacle = false;
    for (std::size_t tick = 0; tick < 500; ++tick)
    {
        collisionGuard.CaptureSafeJointPose();
        controller.Update(0.004);
        const auto& currentKinematics = kinematics.Update(controller.GetStateView());
        visualAdapter.Apply(currentKinematics);
        collisionAdapter.Apply(currentKinematics);
        TransformSystemModule::UpdateWorldTransforms(world);
        collisionGuard.RestoreSafePoseIfOverlapping();
        physicsSystem.Step(0.004);
        TransformSystemModule::UpdateWorldTransforms(world);

        const auto& currentState = controller.GetStateView();
        if (currentState.mode != robotics::RobotMode::Moving)
        {
            stoppedByObstacle = currentState.errorCode == robotics::ErrorCode::EnvironmentContact;
            break;
        }
    }
    ASSERT_TRUE_MESSAGE(stoppedByObstacle, "environment contact restores the last safe joint pose");
    ASSERT_GT(controller.GetStateView().jointPositions.front(), 0.0);
    ASSERT_LT(controller.GetStateView().jointPositions.front(), targetAngleRadians);
}
}

/**
 * @brief URDF 모델 변환 결과와 충돌용 단순 물체가 같은 관절 자세를 사용하는지 확인한다.
 * @details 작은 두 관절 모델에서 URDF 관절을 제어 사양으로 바꾸고 FK 결과를 충돌용 물체에 적용한다.
 * 상자·구·원기둥 충돌 형상이 유지되는지, 관절 회전 뒤에도 링크 충돌체가 FK 위치를 따라가는지 함께 검사한다.
 */
TEST(RobotCollisionGeometry, SyntheticModelBuildsIndependentFkProxies)
{
    CheckSyntheticGeometry();
    for (const std::size_t axisCount : {4U, 6U, 7U})
        CheckLoadedArmPipeline(axisCount);
}

TEST(RobotCollisionGeometry, AllowedCollisionMatrixExcludesOnlyConnectedPairs)
{
    using namespace grasplink::simulation::robotics;
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 2, 4}, {SelfCollisionPart::Link, 7, 4}));
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 2, 4}, {SelfCollisionPart::Link, 8, 5}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 1, 4}, {SelfCollisionPart::Link, 3, 6}));
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Base, 0}, {SelfCollisionPart::Link, 8, 0}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Base, 0}, {SelfCollisionPart::Link, 0, 1}));
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 5, 7}, {SelfCollisionPart::GripperBody, 9, 7}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 4, 6}, {SelfCollisionPart::GripperBody, 9, 7}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 4, SelfCollisionIdentity::InvalidIndex},
        {SelfCollisionPart::GripperBody, 9, SelfCollisionIdentity::InvalidIndex}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 0, SelfCollisionIdentity::InvalidIndex},
        {SelfCollisionPart::Link, 1, SelfCollisionIdentity::InvalidIndex}));

    grasplink::physics::PhysicsWorld physics;
    grasplink::physics::BodyDescription first;
    first.motionType = grasplink::physics::BodyMotionType::Kinematic;
    first.collisionLayer = grasplink::physics::CollisionLayer::Robot;
    first.shapes.push_back(grasplink::physics::CollisionShapeDescription{});
    const auto firstBody = physics.CreateBody(first);
    const auto secondBody = physics.CreateBody(first);
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 1, 4}, {SelfCollisionPart::Link, 2, 5}))
        << "adjacent robot links are excluded from self-collision rejection";
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 1, 4}, {SelfCollisionPart::Link, 3, 6}));
    EXPECT_TRUE(physics.OverlapsBodiesAt(
        firstBody, first.transform, secondBody, first.transform))
        << "an overlapping non-adjacent pair remains detectable by the narrow-phase query";
}

TEST(RobotCollisionGeometry, Hcr12aBaseAndArmProxiesPassOverlapValidation)
{
    using namespace grasplink::robotics;
    flecs::world world;
    // 실제 HCR 모델도 URDF의 충돌 형상으로 만들어 기준 자세에서 겹침 검사를 통과해야 한다.
    const auto actualModel = grasplink::model::RobotModelLoader::Load(
        std::filesystem::path("assets/robots/HCR12A"));
    auto actualScene = std::make_unique<Scene>(world);
    Entity actualRoot = actualScene->CreateEntity("ActualRobot");
    grasplink::simulation::RobotPhysicsAdapter actualAdapter(
        *actualScene, actualRoot, actualModel.robot);
    std::size_t actualShapeCount = 0;
    Entity baseProxy = actualRoot.GetChild("Base_CollisionProxy");
    ASSERT_TRUE_MESSAGE(baseProxy.IsValid(), "URDF base link creates a fixed collision proxy");
    ASSERT_TRUE_MESSAGE(baseProxy.Get<RigidBody>().motionType == grasplink::physics::BodyMotionType::Static,
        "Base proxy does not move with the arm joints");
    ASSERT_TRUE_MESSAGE(baseProxy.Get<RigidBody>().collisionLayer == grasplink::physics::CollisionLayer::Environment,
        "Base proxy participates in Environment overlap checks");
    ASSERT_TRUE_MESSAGE(!baseProxy.Get<Colliders>().shapes.empty(), "URDF base link provides collision shapes");
    ASSERT_TRUE_MESSAGE(baseProxy.GetParent() == actualRoot, "Base collider follows the placed robot root");
    const auto& baseShape = baseProxy.Get<Colliders>().shapes.front();
    ASSERT_TRUE_MESSAGE(baseShape.type == grasplink::physics::CollisionShapeType::Box,
        "base collision follows the box recorded in URDF");
    ASSERT_NEAR_FINITE(baseShape.halfExtentsMeters.x, 0.11, 1e-5, "base width comes from URDF");
    ASSERT_NEAR_FINITE(baseShape.localTransform.position.y, 0.10425, 1e-5, "base position comes from URDF");

    grasplink::robotics::kinematics::RobotKinematics actualKinematics(models::hanwha::kHcr12a);
    RobotState homeState;
    homeState.valid = true;
    homeState.jointPositions.assign(models::hanwha::kHcr12a.jointCount, 0.0);
    actualAdapter.Apply(actualKinematics.Update(homeState));
    grasplink::physics::PhysicsWorld physics;
    grasplink::simulation::PhysicsSystemModule physicsSystem(world, physics);
    TransformSystemModule::UpdateWorldTransforms(world);
    physicsSystem.Step(0.004);
    TransformSystemModule::UpdateWorldTransforms(world);
    Entity link1Proxy = actualRoot.GetChild("Link1_CollisionProxy");
    const glm::mat4 link1World = link1Proxy.GetWorldMatrix();
    grasplink::physics::Transform link1Pose;
    link1Pose.position = glm::vec3(link1World[3]);
    const auto link1Body = physicsSystem.GetBodyHandle(link1Proxy.GetHandle());
    const auto baseBody = physicsSystem.GetBodyHandle(baseProxy.GetHandle());
    ASSERT_TRUE_MESSAGE(physics.IsBodyValid(link1Body), "home Link1 has a Jolt body for overlap validation");
    ASSERT_TRUE_MESSAGE(physics.IsBodyValid(baseBody), "fixed Base has a Jolt body for overlap validation");
    // 로봇 링크와 바닥이 겹치는 경우는 남기고, 연결된 Base와의 겹침만 제외한다.
    ASSERT_TRUE_MESSAGE(physics.OverlapsEnvironmentAt(link1Body, link1Pose),
        "home Link1 intersects Base at their modeled bearing");
    ASSERT_TRUE_MESSAGE(!physics.OverlapsEnvironmentAt(link1Body, link1Pose, baseBody),
        "home Link1 passes validation when only its exact adjacent Base body is excluded");

    std::vector<std::pair<grasplink::simulation::robotics::SelfCollisionIdentity,
        std::pair<Entity, grasplink::physics::PhysicsBodyHandle>>> selfCollisionProxies;
    for (Entity proxy : actualRoot.GetChildren())
    {
        if (proxy == baseProxy)
        {
            selfCollisionProxies.push_back({
                {grasplink::simulation::robotics::SelfCollisionPart::Base, 0,
                    grasplink::simulation::robotics::SelfCollisionIdentity::InvalidIndex},
                {proxy, baseBody}});
            continue;
        }
        if (!proxy.Has<RobotCollisionProxy>())
            continue;
        const auto handle = physicsSystem.GetBodyHandle(proxy.GetHandle());
        ASSERT_TRUE_MESSAGE(physics.IsBodyValid(handle), "each HCR link has a body for self-collision validation");
        selfCollisionProxies.push_back({
            {grasplink::simulation::robotics::SelfCollisionPart::Link,
                proxy.Get<RobotCollisionProxy>().linkIndex,
                proxy.Get<RobotCollisionProxy>().jointIndex},
            {proxy, handle}});
    }
    for (std::size_t first = 0; first < selfCollisionProxies.size(); ++first)
    {
        for (std::size_t second = first + 1; second < selfCollisionProxies.size(); ++second)
        {
            const auto& left = selfCollisionProxies[first];
            const auto& right = selfCollisionProxies[second];
            if (grasplink::simulation::robotics::IsAllowedSelfCollision(left.first, right.first))
                continue;
            const glm::mat4 leftMatrix = left.second.first.GetWorldMatrix();
            const glm::mat4 rightMatrix = right.second.first.GetWorldMatrix();
            const grasplink::physics::Transform leftPose{
                glm::vec3(leftMatrix[3]), glm::normalize(glm::quat_cast(glm::mat3(leftMatrix)))};
            const grasplink::physics::Transform rightPose{
                glm::vec3(rightMatrix[3]), glm::normalize(glm::quat_cast(glm::mat3(rightMatrix)))};
            ASSERT_FALSE(physics.OverlapsBodiesAt(
                left.second.second, leftPose, right.second.second, rightPose))
                << "known HCR home posture has no overlap between non-allowed link pairs";
        }
    }

    for (const auto& proxy : actualRoot.GetChildren())
    {
        if (proxy.GetHandle().name() == "Base_CollisionProxy")
        {
            actualShapeCount += proxy.GetHandle().get<Colliders>().shapes.size();
            continue;
        }
        const auto& shapes = proxy.GetHandle().get<Colliders>().shapes;
        ASSERT_TRUE_MESSAGE(!shapes.empty(), "actual robot link retains collision coverage");
        actualShapeCount += shapes.size();
    }
    ASSERT_TRUE_MESSAGE(actualRoot.GetChildren().size() == kExpectedHcrCollisionProxyCount,
        "the HCR model has one base and six arm collision proxies");
    ASSERT_TRUE_MESSAGE(actualShapeCount <= kMaximumHcrCollisionShapeCount,
        "robot collision shape count stays within the test limit");
}
