#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "model/GltfLoader.h"
#include "scene/Scene.h"
#include "physics/PhysicsWorld.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotCollisionGeometryBuilder.h"
#include "simulation/robotics/RobotSelfCollisionPolicy.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "scene/TransformSystemModule.h"
#include "TestSupport.h"

#include <gtest/gtest.h>

#include <iostream>
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

namespace
{
using grasplink::model::MeshData;
using grasplink::model::ModelResource;
using grasplink::model::SubMeshInfo;
using grasplink::model::Vertex;
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


// 물리 충돌용 단순 형상 추출을 검증하는 CPU 시험 자료다. 꼭짓점은 각 로봇 링크 기준 좌표이고, 메시 외곽은 한 변 0.1 m인 닫힌 정육면체다.
MeshData Cube()
{
    MeshData mesh;
    for (unsigned i = 0; i < 8; ++i)
    {
        Vertex vertex{};
        vertex.position = {(i & 1) ? 0.05F : -0.05F, (i & 2) ? 0.05F : -0.05F, (i & 4) ? 0.05F : -0.05F};
        mesh.vertices.push_back(vertex);
    }
    mesh.indices = {0,1,3,0,3,2,4,6,7,4,7,5,0,4,5,0,5,1,2,3,7,2,7,6,0,2,6,0,6,4,1,5,7,1,7,3};
    SubMeshInfo part;
    part.indexCount = static_cast<std::uint32_t>(mesh.indices.size());
    mesh.subMeshes.push_back(part);
    return mesh;
}

void CheckSyntheticGeometry()
{
    using namespace grasplink::robotics;
    const models::JointSpecification joints[] = {
        {"J1", {0,0,0}, {0,0,1}, -3.1416, 3.1416, 1},
        {"J2", {0,0.5,0}, {0,0,1}, -3.1416, 3.1416, 1}};
    const models::LinkSpecification links[] = {{"Link1",0},{"Link2",1}};
    const models::RobotSpecification spec{"Test","TwoLink",joints,2,links,2};
    ModelResource model;
    model.rootNodeIndex = 0;
    model.meshes = {Cube(), Cube(), Cube()};
    model.nodes.resize(6);
    const char* names[] = {"RobotRoot","J1","Link1","J2","Link2","Gripper"};
    for (int i = 0; i < 6; ++i)
    {
        model.nodes[i].name = names[i];
        model.nodes[i].parentIndex = i - 1;
    }
    model.nodes[2].meshIndex = 0;
    model.nodes[3].translation.y = 0.5F;
    model.nodes[4].meshIndex = 1;
    model.nodes[5].meshIndex = 2;
    model.nodes[5].translation.y = 0.5F;
    const auto builtGeometry = grasplink::simulation::robot_collision_geometry::Build(spec, model);
    ASSERT_TRUE_MESSAGE(builtGeometry.links.size() == 2, "geometry builder returns one result per specified link");
    ASSERT_TRUE_MESSAGE(builtGeometry.links[0].jointIndex == 0 && builtGeometry.links[1].jointIndex == 1,
        "geometry builder keeps each link's FK joint index");
    ASSERT_TRUE_MESSAGE(!builtGeometry.links[0].shapes.empty() && !builtGeometry.links[1].shapes.empty(),
        "geometry builder returns collision shapes without creating Scene entities");
    ASSERT_TRUE_MESSAGE(builtGeometry.baseShapes.empty(), "model without a Base node has no base geometry");
    flecs::world world;
    auto sceneOwner = std::make_unique<Scene>(world);
    Scene& scene = *sceneOwner;
    Entity root = scene.CreateEntity("RobotRoot");
    grasplink::simulation::RobotPhysicsAdapter adapter(scene, root, spec, model);
    std::size_t syntheticLinkIndex = 0;
    for (const char* name : {"Link1_CollisionProxy", "Link2_CollisionProxy"})
    {
        Entity proxy = root.GetChild(name);
        ASSERT_TRUE_MESSAGE(proxy.IsValid(), "collision proxy created");
        const auto& colliders = proxy.Get<Colliders>();
        ASSERT_TRUE_MESSAGE(!colliders.shapes.empty(), std::string(name) + ": own geometry produces colliders");
        ASSERT_TRUE_MESSAGE(colliders.shapes.size() == builtGeometry.links[syntheticLinkIndex].shapes.size(),
            "adapter installs the builder's generated shape set");
        for (const auto& shape : colliders.shapes)
            for (const auto& point : shape.pointsMeters)
                ASSERT_TRUE_MESSAGE(std::abs(point.x) < 0.051F && std::abs(point.y) < 0.051F && std::abs(point.z) < 0.051F,
                    "downstream joint and Gripper geometry excluded");
        ++syntheticLinkIndex;
    }
    kinematics::RobotKinematics fk(spec);
    RobotState state;
    state.valid = true;
    state.jointPositionRadians = {1.5707963267948966, 0};
    adapter.Apply(fk.Update(state));
    TransformSystemModule::UpdateWorldTransforms(world);
    const auto link2 = root.GetChild("Link2_CollisionProxy");
    ASSERT_NEAR_FINITE(link2.GetLocalPosition().x, -0.5, 1e-5, "upstream joint moves proxy pivot");
    ASSERT_NEAR_FINITE(link2.GetLocalPosition().y, 0.0, 1e-5, "proxy follows FK rather than bind geometry");
    auto cycle = model;
    cycle.nodes[0].parentIndex = 5;
    GTestExpectThrows<std::invalid_argument>([&] { grasplink::simulation::RobotPhysicsAdapter bad(scene, root, spec, cycle); }, "GLB cycle rejected before recursion");
    GTestExpectThrows<std::invalid_argument>([&] { adapter.Apply({}); }, "missing FK poses rejected");
    sceneOwner.reset();
    GTestExpectThrows<std::runtime_error>([&] { adapter.Apply(fk.Update(state)); }, "removed Scene invalidates physics proxy bindings");
}
}

/**
 * @brief 로봇 링크마다 만든 충돌용 단순 물체가 올바른 모델 조각을 따라가는지 확인한다.
 * @details 화면에 보이는 로봇 모델과 별도로 충돌 계산에 쓰는 단순 물체를 만든다. 각 물체는 링크에 붙어 FK가 관절 각도에서 계산한 위치와 방향을 따라야 한다.
 * 작은 시험 계층으로 하위 관절과 그리퍼가 부모 링크에 중복 포함되지 않는지, 잘못된 모델 연결과 삭제된 Scene 참조가 거부되는지 검사한다.
 * 실제 HCR12A GLB에서 여섯 팔 링크의 단순 볼록 형상 수가 예상 범위에 있는지도 확인한다. 이 시험은 CPU 자료로 설정을 만드는 단계까지만 다루며 GPU 업로드나 Jolt 접촉 계산은 하지 않는다.
 */
TEST(RobotCollisionGeometry, SyntheticModelBuildsIndependentFkProxies)
{
    CheckSyntheticGeometry();
}

TEST(RobotCollisionGeometry, AllowedCollisionMatrixExcludesOnlyConnectedPairs)
{
    using namespace grasplink::simulation::robotics;
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 2}, {SelfCollisionPart::Link, 3}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 1}, {SelfCollisionPart::Link, 3}));
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Base, 0}, {SelfCollisionPart::Link, 0}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Base, 0}, {SelfCollisionPart::Link, 1}));
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 5}, {SelfCollisionPart::GripperBody, 0}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 4}, {SelfCollisionPart::GripperBody, 0}));
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, static_cast<std::size_t>(-1)},
        {SelfCollisionPart::Link, static_cast<std::size_t>(-1)}));

    grasplink::physics::PhysicsWorld physics;
    grasplink::physics::BodyDescription first;
    first.motionType = grasplink::physics::BodyMotionType::Kinematic;
    first.collisionLayer = grasplink::physics::CollisionLayer::Robot;
    first.shapes.push_back(grasplink::physics::CollisionShapeDescription{});
    const auto firstBody = physics.CreateBody(first);
    const auto secondBody = physics.CreateBody(first);
    EXPECT_TRUE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 1}, {SelfCollisionPart::Link, 2}))
        << "adjacent robot links are excluded from self-collision rejection";
    EXPECT_FALSE(IsAllowedSelfCollision(
        {SelfCollisionPart::Link, 1}, {SelfCollisionPart::Link, 3}));
    EXPECT_TRUE(physics.OverlapsBodiesAt(
        firstBody, first.transform, secondBody, first.transform))
        << "an overlapping non-adjacent pair remains detectable by the narrow-phase query";
}

TEST(RobotCollisionGeometry, Hcr12aBaseAndArmProxiesPassOverlapValidation)
{
    using namespace grasplink::robotics;
    flecs::world world;
    // 여섯 팔 Link에는 현재 121개 볼록 껍질이 만들어지고 Base의 hull은 별도로 집계한다. 전체 수의 150개 상한으로 형상 복잡도의 예상 밖 증가를 검출한다.
    const auto actualModel = LoadTestGlb("HCR12A_2F-85.glb");
    auto actualScene = std::make_unique<Scene>(world);
    Entity actualRoot = actualScene->CreateEntity("ActualRobot");
    grasplink::simulation::RobotPhysicsAdapter actualAdapter(
        *actualScene, actualRoot, models::hanwha::kHcr12a, actualModel);
    std::size_t actualHullCount = 0;
    Entity baseProxy = actualRoot.GetChild("Base_CollisionProxy");
    ASSERT_TRUE_MESSAGE(baseProxy.IsValid(), "actual Base mesh creates a fixed collision proxy");
    ASSERT_TRUE_MESSAGE(baseProxy.Get<RigidBody>().motionType == grasplink::physics::BodyMotionType::Static,
        "Base proxy does not move with the arm joints");
    ASSERT_TRUE_MESSAGE(baseProxy.Get<RigidBody>().collisionLayer == grasplink::physics::CollisionLayer::Environment,
        "Base proxy participates in Environment overlap checks");
    ASSERT_TRUE_MESSAGE(!baseProxy.Get<Colliders>().shapes.empty(), "actual Base mesh produces collision hulls");
    ASSERT_TRUE_MESSAGE(baseProxy.GetParent() == actualRoot, "Base collider follows the placed robot root");
    const auto& baseShape = baseProxy.Get<Colliders>().shapes.front();
    ASSERT_TRUE_MESSAGE(baseShape.type == grasplink::physics::CollisionShapeType::ConvexHull,
        "Base geometry uses the same supported convex hull shape type as arm links");
    for (const glm::vec3& point : baseShape.pointsMeters)
        ASSERT_TRUE_MESSAGE(std::abs(point.x) < 0.25F && point.y >= -0.01F && point.y < 0.25F && std::abs(point.z) < 0.25F,
            "Base hull points come from the GLB Base geometry in robot-root coordinates");

    grasplink::robotics::kinematics::RobotKinematics actualKinematics(models::hanwha::kHcr12a);
    RobotState homeState;
    homeState.valid = true;
    homeState.jointPositionRadians.assign(models::hanwha::kHcr12a.jointCount, 0.0);
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
    // 두 GLB 충돌 형상은 J1 베어링에서 실제로 겹치므로, Base 하나만 제외한 검사와 기본 검사를 함께 확인한다.
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
                {grasplink::simulation::robotics::SelfCollisionPart::Base, 0},
                {proxy, baseBody}});
            continue;
        }
        if (!proxy.Has<RobotCollisionProxy>())
            continue;
        const auto handle = physicsSystem.GetBodyHandle(proxy.GetHandle());
        ASSERT_TRUE_MESSAGE(physics.IsBodyValid(handle), "each HCR link has a body for self-collision validation");
        selfCollisionProxies.push_back({
            {grasplink::simulation::robotics::SelfCollisionPart::Link,
                proxy.Get<RobotCollisionProxy>().linkIndex},
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
            actualHullCount += proxy.GetHandle().get<Colliders>().shapes.size();
            continue;
        }
        const auto& shapes = proxy.GetHandle().get<Colliders>().shapes;
        ASSERT_TRUE_MESSAGE(!shapes.empty(), "actual robot link retains collision coverage");
        actualHullCount += shapes.size();
        std::cout << proxy.GetHandle().name() << ": " << shapes.size() << " hulls\n";
    }
    std::cout << "Actual robot hulls: " << actualHullCount << '\n';
    ASSERT_TRUE_MESSAGE(actualRoot.GetChildren().size() == 7, "actual robot keeps one Base and six independent arm collision proxies");
    ASSERT_TRUE_MESSAGE(actualHullCount <= 150, "actual robot collider complexity stays within runtime budget");
}
