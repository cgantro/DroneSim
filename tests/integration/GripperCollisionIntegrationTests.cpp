#include "scene/Entity.h"
#include "physics/PhysicsWorld.h"
#include "model/GltfLoader.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "scene/TransformSystemModule.h"
#include "TestSupport.h"

#include <gtest/gtest.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <filesystem>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace
{
using grasplink::model::ModelResource;
using grasplink::model::NodeData;
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


using grasplink::physics::BodyMotionType;
using grasplink::physics::CollisionLayer;

struct Fixture
{
    flecs::world world;
    grasplink::physics::PhysicsWorld physics;
    ModelResource model;
    Entity robotRoot;
    Entity gripper;
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> integration;
    std::unique_ptr<Scene> scene;

    Fixture()
    {
        // 실제 GLB에 저장된 것과 같은 그리퍼 자식 계층을 구성한다. 충돌용 단순 형상의 자세는 이 ECS 관절들의 변환을 따라야 한다.
        scene = std::make_unique<Scene>(world);
        EXPECT_TRUE_MESSAGE(scene->GetSceneRoot().is_alive(), "Scene owns a live hierarchy root");
        model = LoadTestGlb("HCR12A_2F-85.glb");
        robotRoot = scene->CreateEntity("RobotRoot");
        const int gripperIndex = FindNode("Gripper");
        EXPECT_TRUE_MESSAGE(gripperIndex >= 0, "GLB has Gripper frame");
        gripper = CreateSubtree(gripperIndex, robotRoot);
    }

    int FindNode(const std::string& name) const
    {
        for (std::size_t i = 0; i < model.nodes.size(); ++i)
            if (model.nodes[i].name == name) return static_cast<int>(i);
        return -1;
    }

    Entity CreateSubtree(int nodeIndex, const Entity& parent)
    {
        const NodeData& node = model.nodes.at(static_cast<std::size_t>(nodeIndex));
        Entity entity = scene->CreateEntity(node.name);
        entity.SetParent(parent);
        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);
        for (int child : node.childrenIndices) CreateSubtree(child, entity);
        return entity;
    }

    std::size_t CountDescendants(const Entity& entity) const
    {
        std::size_t count = 0;
        for (const Entity& child : entity.GetChildren())
            count += 1 + CountDescendants(child);
        return count;
    }

    void Step(int count = 1)
    {
        // GLB 관절의 Local 값을 Scene 기준 World 행렬로 누적한 뒤 Kinematic 목표를 Jolt에 전달한다.
        // Dynamic Body 결과는 ECS의 부모 기준 값으로 되돌리고, 새 World 행렬을 계산한다.
        // Controller가 없는 시험용 그리퍼이므로 충돌용 단순 형상이 이 관절 계층의 자세를 정확히 따라야 한다.
        TransformSystemModule::UpdateWorldTransforms(world);
        for (int i = 0; i < count; ++i)
        {
            integration->Step(0.004);
            TransformSystemModule::UpdateWorldTransforms(world);
        }
    }
};

glm::vec3 WorldPoint(const Entity& entity, const glm::vec3& local)
{
    return glm::vec3(entity.GetWorldMatrix() * glm::vec4(local, 1.0F));
}

glm::vec3 TopSurfacePoint(Entity proxy, float& topY)
{
    // 충돌용 단순 형상의 꼭짓점을 자체 좌표에서 Scene 좌표로 변환해 가장 높은 점을 찾는다.
    // 표면에서 2 mm 안에 있는 여러 꼭짓점의 평균을 사용하면 한 꼭짓점만 고르는 것보다 GLB 메시 분할 차이에 덜 민감한 지지 위치를 얻는다.
    std::vector<glm::vec3> points;
    for (const auto& shape : proxy.Get<Colliders>().shapes)
        for (const auto& point : shape.pointsMeters)
            points.push_back(WorldPoint(proxy, point));
    topY = -100.0F;
    for (const auto& point : points) topY = std::max(topY, point.y);
    glm::vec3 center(0.0F);
    std::size_t count = 0;
    for (const auto& point : points)
        if (topY - point.y < 0.002F)
        {
            center += point;
            ++count;
        }
    EXPECT_TRUE_MESSAGE(count != 0, "tip hull has a gravity-facing support point");
    if (count == 0) return center;
    return center / static_cast<float>(count);
}

void DropOnTip(Fixture& fixture, const char* tipJoint, const char* proxyName)
{
    Entity proxy = fixture.gripper.FindChildByNameRecursive(proxyName);
    ASSERT_TRUE_MESSAGE(proxy.IsValid(), std::string(proxyName) + " exists");
    const auto shapes = proxy.Get<Colliders>().shapes;
    ASSERT_TRUE_MESSAGE(!shapes.empty(), std::string(proxyName) + " has collider geometry");
    const Entity joint = fixture.gripper.FindChildByNameRecursive(tipJoint);
    ASSERT_TRUE_MESSAGE(joint.IsValid(), std::string(tipJoint) + " exists");
    float surfaceY = 0.0F;
    const glm::vec3 surface = TopSurfacePoint(proxy, surfaceY);

    // 상자를 표면에서 70 mm 위에 놓고 한 변 8 mm로 만든다. 300회의 4 ms 물리 계산 뒤 상자 중심은 표면보다 2~20 mm 위에 있어야 한다.
    // 너무 낮은 결과는 표면 관통을, 너무 높은 결과는 충돌 형상이 상자에 닿지 못한 경우를 가리킨다.
    Entity drop = fixture.scene->CreateEntity(std::string("Drop_") + tipJoint);
    drop.SetLocalPosition({surface.x, surfaceY + 0.07F, surface.z});
    drop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.004F, 0.004F, 0.004F})}});
    fixture.Step(300);
    ASSERT_TRUE_MESSAGE(drop.GetLocalPosition().y > surfaceY + 0.002F && drop.GetLocalPosition().y < surfaceY + 0.02F,
        std::string("dynamic object contacts ") + tipJoint + " collider");
}
}

TEST(GripperCollisionIntegration, RejectsInvalidMeshReferencesBeforeCreatingProxies)
{
    Fixture fixture;
    const std::size_t beforeInvalid = fixture.CountDescendants(fixture.robotRoot);
    ModelResource missingMesh = fixture.model;
    for (NodeData& node : missingMesh.nodes)
        if (node.name == "LeftFingerTipMesh") node.name = "MissingFingerTipMesh";
    GTestExpectThrows<std::invalid_argument>([&]
    {
        grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, missingMesh);
    }, "missing required mesh rejected");
    ASSERT_TRUE_MESSAGE(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid,
        "missing mesh is rejected before creating proxy entities");
    Entity misplacedMesh = fixture.gripper.FindChildByNameRecursive("LeftFingerTipMesh");
    const Entity originalMeshParent = misplacedMesh.GetParent();
    misplacedMesh.SetParent(fixture.robotRoot);
    GTestExpectThrows<std::invalid_argument>([&]
    {
        grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, fixture.model);
    }, "mesh outside its owning joint hierarchy rejected");
    ASSERT_TRUE_MESSAGE(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid,
        "wrong hierarchy is rejected before creating proxy entities");
    misplacedMesh.SetParent(originalMeshParent);
}

TEST(GripperCollisionIntegration, ConfiguredProxiesFollowTheGripperAndContactObjects)
{
    Fixture fixture;
    const std::size_t beforeInvalid = fixture.CountDescendants(fixture.robotRoot);

    // 잘못된 Mesh 이름과 소유 관절 바깥으로 이동한 Mesh를 각각 거부하는지 확인한다. 실패하면 충돌 형상이 일부만 만들어진 상태로 남지 않아야 한다.
    grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, fixture.model);
    TransformSystemModule::UpdateWorldTransforms(fixture.world);

    const std::vector<std::string> proxies{
        "Gripper_CollisionProxy", "LeftOuterKnuckleJoint_CollisionProxy", "LeftInnerKnuckleJoint_CollisionProxy",
        "LeftFingerTipJoint_CollisionProxy", "RightOuterKnuckleJoint_CollisionProxy", "RightInnerKnuckleJoint_CollisionProxy",
        "RightFingerTipJoint_CollisionProxy"};
    for (const auto& name : proxies)
    {
        Entity proxy = fixture.gripper.FindChildByNameRecursive(name);
        ASSERT_TRUE_MESSAGE(proxy.IsValid(), name + " exists");
        ASSERT_TRUE_MESSAGE(glm::length(proxy.GetLocalPosition()) < 1e-6F, name + " identity local translation");
        const glm::quat localRotation = proxy.GetLocalRotation();
        ASSERT_TRUE_MESSAGE(std::abs(std::abs(localRotation.w) - 1.0F) < 1e-6F &&
                glm::length(glm::vec3{localRotation.x, localRotation.y, localRotation.z}) < 1e-6F,
            name + " identity local rotation");
        ASSERT_TRUE_MESSAGE(glm::length(proxy.GetLocalScale() - glm::vec3(1.0F)) < 1e-6F, name + " identity local scale");
        ASSERT_TRUE_MESSAGE(proxy.Get<RigidBody>().motionType == BodyMotionType::Kinematic, name + " is Kinematic");
        ASSERT_TRUE_MESSAGE(proxy.Get<RigidBody>().collisionLayer == CollisionLayer::Gripper, name + " uses Gripper layer");
        ASSERT_TRUE_MESSAGE(!proxy.Get<Colliders>().shapes.empty(), name + " has own hulls");
        for (const auto& shape : proxy.Get<Colliders>().shapes)
            ASSERT_TRUE_MESSAGE(shape.type == grasplink::physics::CollisionShapeType::ConvexHull && shape.pointsMeters.size() >= 4,
                name + " consists of convex support point hulls");
    }

    Entity leftTip = fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint_CollisionProxy");
    Entity rightTip = fixture.gripper.FindChildByNameRecursive("RightFingerTipJoint_CollisionProxy");
    // 손끝 충돌 형상의 Scene 좌표 꼭짓점을 그리퍼 Local 좌표로 되돌려 열린 손가락 사이의 폭을 잰다.
    // 85 mm보다 넓어야 GLB의 열린 자세에서 충돌 형상이 손가락 사이 통로를 막지 않았다고 볼 수 있다.
    const glm::mat4 worldToGripper = glm::inverse(fixture.gripper.GetWorldMatrix());
    float leftMaxX = -1.0F;
    float rightMinX = 1.0F;
    for (const auto& shape : leftTip.Get<Colliders>().shapes)
        for (const auto& point : shape.pointsMeters)
        {
            const glm::vec3 inFrame(worldToGripper * glm::vec4(WorldPoint(leftTip, point), 1.0F));
            leftMaxX = std::max(leftMaxX, inFrame.x);
        }
    for (const auto& shape : rightTip.Get<Colliders>().shapes)
        for (const auto& point : shape.pointsMeters)
        {
            const glm::vec3 inFrame(worldToGripper * glm::vec4(WorldPoint(rightTip, point), 1.0F));
            rightMinX = std::min(rightMinX, inFrame.x);
        }
    ASSERT_NEAR_FINITE(rightMinX - leftMaxX, 0.085, 1.0e-5, "open fingertip aperture is 85 mm");

    // 모델 root에 이동과 항등이 아닌 회전을 적용해도 충돌 형상의 World 자세가 GLB 관절 계층에서 계산한 자세와 같아야 한다.
    fixture.robotRoot.SetLocalPosition({0.0F, 0.3F, 0.0F});
    fixture.gripper.SetLocalPosition(glm::vec3(0.0F));
    fixture.gripper.SetLocalRotation(glm::angleAxis(1.5707963F, glm::vec3{1.0F, 0.0F, 0.0F}));
    TransformSystemModule::UpdateWorldTransforms(fixture.world);
    ASSERT_TRUE_MESSAGE(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid + 7,
        "successful configuration creates exactly seven proxies");
    fixture.integration = CreateTestPhysicsSystem(fixture.world, fixture.physics);
    TransformSystemModule::UpdateWorldTransforms(fixture.world);

    DropOnTip(fixture, "LeftFingerTipJoint", "LeftFingerTipJoint_CollisionProxy");
    DropOnTip(fixture, "RightFingerTipJoint", "RightFingerTipJoint_CollisionProxy");

    Entity baseProxy = fixture.gripper.FindChildByNameRecursive("Gripper_CollisionProxy");
    float baseSurfaceY = 0.0F;
    const glm::vec3 baseSurface = TopSurfacePoint(baseProxy, baseSurfaceY);
    std::vector<std::pair<Entity, Colliders>> otherProxyColliders;
    for (const auto& name : proxies)
    {
        if (name == "Gripper_CollisionProxy") continue;
        Entity proxy = fixture.gripper.FindChildByNameRecursive(name);
        otherProxyColliders.emplace_back(proxy, proxy.Get<Colliders>());
        proxy.Remove<Colliders>();
    }
    Entity baseDrop = fixture.scene->CreateEntity("BaseBodyDrop");
    baseDrop.SetLocalPosition({baseSurface.x, baseSurfaceY + 0.07F, baseSurface.z});
    baseDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.004F, 0.004F, 0.004F})}});
    fixture.Step(300);
    ASSERT_TRUE_MESSAGE(baseDrop.GetLocalPosition().y > baseSurfaceY + 0.002F &&
            baseDrop.GetLocalPosition().y < baseSurfaceY + 0.02F,
        "dynamic object contacts fixed Gripper body collider: body y=" +
            std::to_string(baseDrop.GetLocalPosition().y) + ", surface y=" + std::to_string(baseSurfaceY));
    for (auto& [proxy, colliders] : otherProxyColliders) proxy.set<Colliders>(colliders);
    fixture.Step(1);

    // 상자가 열린 손끝 사이 통로를 통과해 아래로 떨어져야 한다. 잘못해서 손바닥이나 손가락 전체를 막힌 볼록 형상으로 만든 회귀를 검출한다.
    Entity apertureDrop = fixture.scene->CreateEntity("OpenApertureDrop");
    apertureDrop.SetLocalPosition({0.0F, 0.7F, 0.13F});
    apertureDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(350);
    ASSERT_TRUE_MESSAGE(apertureDrop.GetLocalPosition().y < -0.15F, "dynamic object passes through open finger aperture");

    // 모델 root를 올린 새 위치에서는 상자가 손끝에 닿고, 예전 위치에서는 바닥으로 떨어져야 한다. 이 결과로 자세 갱신과 기존 Jolt Body의 재생성을 함께 확인한다.
    float oldSurfaceY = 0.0F;
    const glm::vec3 oldSurface = TopSurfacePoint(leftTip, oldSurfaceY);
    fixture.robotRoot.SetLocalPosition({0.0F, 0.6F, 0.0F});
    fixture.Step(2);
    float newSurfaceY = 0.0F;
    const glm::vec3 newSurface = TopSurfacePoint(leftTip, newSurfaceY);
    ASSERT_TRUE_MESSAGE(newSurfaceY > oldSurfaceY + 0.25F, "moving Gripper frame moves Kinematic collider pose");
    Entity movedRootContact = fixture.scene->CreateEntity("MovedRootContact");
    movedRootContact.SetLocalPosition({newSurface.x, newSurfaceY + 0.07F, newSurface.z});
    movedRootContact.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(300);
    ASSERT_TRUE_MESSAGE(movedRootContact.GetLocalPosition().y > newSurfaceY + 0.002F &&
            movedRootContact.GetLocalPosition().y < newSurfaceY + 0.02F,
        "dynamic object contacts fingertip after root collider moves");
    Entity rootMoveDrop = fixture.scene->CreateEntity("RootMovedDrop");
    rootMoveDrop.SetLocalPosition({oldSurface.x, oldSurfaceY + 0.07F, oldSurface.z});
    rootMoveDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(280);
    ASSERT_TRUE_MESSAGE(rootMoveDrop.GetLocalPosition().y < oldSurfaceY - 0.05F,
        "object at old fingertip position falls after root collider moves");

    // 모델 root는 고정한 채 손가락 관절 하나만 회전한다. 그 관절에 연결된 Kinematic 충돌 형상이 같은 Local 회전을 따라가는지 확인한다.
    Entity leftJoint = fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint");
    float oldFingerSurfaceY = 0.0F;
    const glm::vec3 oldFingerSurface = TopSurfacePoint(leftTip, oldFingerSurfaceY);
    leftJoint.SetLocalRotation(leftJoint.GetLocalRotation() *
        glm::angleAxis(0.45F, glm::vec3{0.0F, 0.0F, 1.0F}));
    fixture.Step(2);
    float newFingerSurfaceY = 0.0F;
    const glm::vec3 newFingerSurface = TopSurfacePoint(leftTip, newFingerSurfaceY);
    ASSERT_TRUE_MESSAGE(glm::length(newFingerSurface - oldFingerSurface) > 0.01F,
        "finger joint rotation moves its Kinematic proxy surface");
    Entity movedFingerDrop = fixture.scene->CreateEntity("MovedFingerDrop");
    movedFingerDrop.SetLocalPosition({newFingerSurface.x, newFingerSurfaceY + 0.07F, newFingerSurface.z});
    movedFingerDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(280);
    ASSERT_TRUE_MESSAGE(movedFingerDrop.GetLocalPosition().y > newFingerSurfaceY + 0.002F &&
            movedFingerDrop.GetLocalPosition().y < newFingerSurfaceY + 0.02F,
        "object contacts the physically rotated fingertip collider");

    // 동일한 설정을 두 번 적용하면 이름이 이미 등록되어 있다는 이유로 두 번째 생성이 거부되고 충돌 형상이 중복되지 않아야 한다.
    GTestExpectThrows<std::invalid_argument>([&]
    {
        grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, fixture.model);
    }, "repeated configuration rejected");
    ASSERT_TRUE_MESSAGE(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid + 7,
        "repeated configuration does not create partial proxies");

    // Scene을 교체할 때 Scene이 소유한 충돌 형상 자손 계층과 PhysicsSystemModule의 Entity 연결 기록도 함께 정리되어야 한다.
    const Entity oldSceneRoot(fixture.scene->GetSceneRoot());
    const Entity oldProxy = fixture.gripper.FindChildByNameRecursive("Gripper_CollisionProxy");
    fixture.scene.reset();
    ASSERT_TRUE_MESSAGE(!oldSceneRoot.GetHandle().is_alive() && !oldProxy.IsValid(), "Scene cleanup removes proxy hierarchy");

}
