#include "scene/Entity.h"
#include "physics/PhysicsWorld.h"
#include "scene/TransformComponents.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "scene/TransformSystemModule.h"
#include "TestSupport.h"

#include <gtest/gtest.h>
#include <memory>

using grasplink::physics::BodyMotionType;
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


/**
 * @brief 물리 물체 종류에 따라 누가 위치와 회전을 정하는지 확인한다.
 * @details BodyMotionType은 물체를 움직이는 주체를 나타낸다. Static 바닥은 고정되고 Kinematic 로봇 팔은 프로그램이 정한 목표를 따른다.
 * Dynamic 상자는 Jolt가 계산한 중력 낙하와 다른 물체와의 충돌 결과를 장면 물체에 돌려준다.
 */
TEST(PhysicsEcsIntegration, PoseAuthorityMatchesBodyMotionType)
{
    using grasplink::physics::IsPhysicsDriven;
    using grasplink::physics::IsSceneDriven;
    ASSERT_TRUE_MESSAGE(IsSceneDriven(BodyMotionType::Static) && !IsPhysicsDriven(BodyMotionType::Static),
        "Static pose is Scene driven");
    ASSERT_TRUE_MESSAGE(IsSceneDriven(BodyMotionType::Kinematic) && !IsPhysicsDriven(BodyMotionType::Kinematic),
        "Kinematic pose is Scene driven");
    ASSERT_TRUE_MESSAGE(!IsSceneDriven(BodyMotionType::Dynamic) && IsPhysicsDriven(BodyMotionType::Dynamic),
        "Dynamic pose is Physics driven");
}

/**
 * @brief Scene에서 지정한 위치와 Jolt가 계산한 물리 위치가 올바른 순서로 전달되는지 확인한다.
 * @details Dynamic 상자는 위치·회전·크기가 없는 중간 부모 아래에서만 물리 위치를 받을 수 있다. Static 바닥과 Kinematic 목표 물체는 부모를 옮기거나 돌리면 그 변환을 따라간다.
 * Kinematic 물체가 목표에 도착하면 더 움직이지 않고 상자를 일정 높이에서 받쳐야 한다. 충돌 모양 설정을 바꾸거나 Scene을 삭제할 때 연결된 Jolt 물체도 제거되는지 확인한다.
 */
TEST(PhysicsEcsIntegration, SceneHierarchyPhysicsContactsAndLifetime)
{
    // 이 fixture는 Flecs의 collider 설정, Scene 삭제, 변환 시스템과 Jolt 사이에서 자세가 왕복하는 전체 흐름을 함께 준비한다.
    flecs::world world;
    grasplink::physics::PhysicsWorld physics;
    auto sceneOwner = std::make_unique<Scene>(world);
    Scene& scene = *sceneOwner;
    ASSERT_TRUE_MESSAGE(scene.GetSceneRoot().is_alive(), "Scene owns a live hierarchy root");
    // 이 중간 물체는 이름·위치·회전·크기가 없지만 자식의 부모다. 계산은 이를 건너뛰되 조상에 설정한 위치와 회전은 자식 World 행렬에 반영되어야 한다.
    Entity ancestor = scene.CreateEntity("Ancestor");
    ancestor.SetLocalPosition({2.0F, 0.0F, 0.0F});
    auto group = world.entity().child_of(ancestor.GetHandle());
    Entity blocked = scene.CreateEntity("BlockedDynamic");
    blocked.SetParent(Entity{group});
    ASSERT_TRUE_MESSAGE(ancestor.FindChildByNameRecursive("BlockedDynamic") == blocked, "named lookup traverses anonymous grouping Entity");
    blocked.SetLocalPosition({0.0F, 2.0F, 0.0F});
    blocked.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
    auto identityGroup = world.entity().child_of(scene.GetSceneRoot());
    Entity falling = scene.CreateEntity("Falling");
    falling.SetParent(Entity{identityGroup});
    // 다른 충돌 검사와 간섭하지 않는 x=-5 m 위치에서, 변환이 없는 grouping 부모 아래 Dynamic 물체가 낙하하는지 확인한다.
    falling.SetLocalPosition({-5.0F, 2.0F, 0.0F});
    falling.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
    TransformSystemModule::UpdateWorldTransforms(world);
    ASSERT_NEAR_FINITE(blocked.GetWorldMatrix()[3].x, 2.0, 1e-5, "ancestor through transform-less group");
    grasplink::simulation::PhysicsSystemModule integration(world, physics);
    auto step = [&]
    {
        // 매 4 ms 고정 간격으로 Local 값을 World 행렬로 누적하고, Dynamic 물체의 물리 결과를 Local 값에 되돌린 다음 새 World 행렬을 계산한다.
        TransformSystemModule::UpdateWorldTransforms(world);
        integration.Step(0.004);
        TransformSystemModule::UpdateWorldTransforms(world);
    };
    // 짧은 낙하 뒤 Jolt가 반환한 Scene 기준 위치를 부모 기준 Local 위치로 변환해 저장하는 방향이 맞는지 확인한다.
    for (int i = 0; i < 30; ++i) step();
    ASSERT_NEAR_FINITE(blocked.GetLocalPosition().y, 2.0, 1e-6, "Dynamic with translated ancestor hierarchy rejected");
    // 이어지는 부모 변경 검증에서 조상의 변환을 항등으로 돌려도 이 격리된 fixture는 다른 물체와 접촉하지 않는다.
    blocked.Remove<Colliders>();
    ASSERT_TRUE_MESSAGE(falling.GetLocalPosition().y < 1.99F, "Dynamic pose written to Local");
    ASSERT_NEAR_FINITE(falling.GetLocalPosition().x, -5.0, 1e-4, "identity group preserves Dynamic World x");
    // Collider는 Jolt가 물체끼리 부딪힐 모양을 정하는 설정이다. 이를 제거하면 해당 Jolt 물체도 없어야 하고, 다시 붙이면 현재 위치에서 새 물체가 만들어져 낙하를 시작해야 한다.
    falling.Remove<Colliders>();
    const float frozen = falling.GetLocalPosition().y;
    for (int i = 0; i < 30; ++i) step();
    ASSERT_NEAR_FINITE(falling.GetLocalPosition().y, frozen, 1e-6, "removed collider disconnects Dynamic feedback");
    falling.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
    for (int i = 0; i < 30; ++i) step();
    ASSERT_TRUE_MESSAGE(falling.GetLocalPosition().y < frozen - 0.01F, "reconfigured collider resumes Physics");

    // Body를 만든 뒤 부모의 위치·회전·크기를 바꾸면 지원되지 않는 변환이므로 기존 Body를 제거해야 한다. 새 설정이 거부되는 동안 ECS의 Local 자세가 유지되어야 한다.
    falling.SetParent(Entity{group});
    falling.SetLocalPosition({-5.0F, 3.0F, 0.0F});
    for (int i = 0; i < 20; ++i) step();
    ASSERT_NEAR_FINITE(falling.GetLocalPosition().y, 3.0, 1e-6, "runtime translated parent disconnects Dynamic feedback");
    ancestor.SetLocalPosition({0.0F, 0.0F, 0.0F});
    step();
    ASSERT_TRUE_MESSAGE(falling.GetLocalPosition().y < 3.0F, "identity ancestor recreates Dynamic Body");
    ancestor.SetLocalRotation(glm::angleAxis(0.4F, glm::vec3{0.0F, 1.0F, 0.0F}));
    falling.SetLocalPosition({-5.0F, 3.0F, 0.0F});
    for (int i = 0; i < 20; ++i) step();
    ASSERT_NEAR_FINITE(falling.GetLocalPosition().y, 3.0, 1e-6, "runtime rotated parent disconnects Dynamic feedback");
    ancestor.SetLocalRotation(glm::quat{1.0F, 0.0F, 0.0F, 0.0F});
    step();
    ASSERT_TRUE_MESSAGE(falling.GetLocalPosition().y < 3.0F, "identity rotation recreates Dynamic Body");
    ancestor.SetLocalScale({2.0F, 2.0F, 2.0F});
    falling.SetLocalPosition({-5.0F, 3.0F, 0.0F});
    for (int i = 0; i < 20; ++i) step();
    ASSERT_NEAR_FINITE(falling.GetLocalPosition().y, 3.0, 1e-6, "runtime scaled parent disconnects Dynamic feedback");
    ancestor.SetLocalScale({1.0F, 1.0F, 1.0F});
    falling.SetParent(Entity{identityGroup});
    for (int i = 0; i < 30; ++i) step();
    ASSERT_TRUE_MESSAGE(falling.GetLocalPosition().y < 2.99F && falling.GetLocalPosition().y > 2.8F,
        "supported parent recreates Dynamic Body from current Scene pose");

    // Dynamic 부모와 자식 Body를 각각 움직인 뒤 그 결과를 계층 위치로 합치는 규칙은 없다. 따라서 이런 자식 Body 설정은 물리 시스템이 거부해야 한다.
    Entity unsupported = scene.CreateEntity("UnsupportedChild");
    unsupported.SetLocalPosition({0.0F, 0.5F, 0.0F});
    unsupported.SetParent(falling);
    unsupported.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.05F, 0.05F, 0.05F})}});
    for (int i = 0; i < 20; ++i) step();
    ASSERT_NEAR_FINITE(unsupported.GetLocalPosition().y, 0.5, 1e-6, "Dynamic ancestor hierarchy rejected");
    GTestExpectThrows<std::invalid_argument>([&] { falling.SetParent(Entity{world.entity("External")}); }, "cannot leave owning Scene");
    GTestExpectThrows<std::invalid_argument>([&] { falling.SetParent(unsupported); }, "hierarchy cycle rejected");

    // 바닥과 상자의 반쪽 높이를 더하면 접촉 중심은 바닥에서 0.2 m에 있어야 한다. 20 mm 범위는 계산 오차를 허용하면서 실제 접촉 높이를 확인한다.
    Entity floorParent = scene.CreateEntity("FloorParent");
    floorParent.SetLocalRotation(glm::angleAxis(0.3F, glm::vec3{0.0F, 1.0F, 0.0F}));
    Entity floor = scene.CreateEntity("Floor");
    floor.SetParent(floorParent);
    floor.set<RigidBody>({BodyMotionType::Static, grasplink::physics::CollisionLayer::Environment})
        .set<Colliders>({{physics_colliders::Box({1.0F, 0.1F, 1.0F})}});
    Entity drop = scene.CreateEntity("Drop");
    drop.SetLocalPosition({0.0F, 1.0F, 0.0F});
    drop.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
    // 4 ms step을 500번 실행해 2초 동안 상자가 낙하하고 바닥에 안착하도록 한다.
    for (int i = 0; i < 500; ++i) step();
    ASSERT_NEAR_FINITE(drop.GetLocalPosition().y, 0.2, 0.02, "ECS configured Static contact");
    // Static Body는 움직이지 않는 물체이므로 Scene에서 바꾼 World 위치를 물리 계산 전에 Body에 반영해야 새 높이에서 접촉한다.
    floorParent.SetLocalPosition({0.0F, 0.4F, 0.0F});
    drop.SetLocalPosition({0.0F, 1.4F, 0.0F});
    drop.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
    for (int i = 0; i < 500; ++i) step();
    ASSERT_NEAR_FINITE(drop.GetLocalPosition().y, 0.6, 0.02, "moved Static parent updates physical contact height");
    floor.Destroy();
    // Jolt는 물체가 움직이지 않으면 계산을 줄이려고 일시 중지할 수 있다. Collider를 다시 설정해 새 물리 물체를 만들고 이전 상태에 기대지 않는다.
    // 바닥 Entity를 삭제한 뒤 상자가 더는 멈추지 않고 아래로 떨어지는지 확인한다.
    drop.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
    for (int i = 0; i < 150; ++i) step();
    ASSERT_TRUE_MESSAGE(drop.GetLocalPosition().y < -0.2F, "Entity deletion removes Jolt Collider");

    Entity platformParent = scene.CreateEntity("PlatformParent");
    platformParent.SetLocalPosition({5.0F, 0.0F, 0.0F});
    platformParent.SetLocalRotation(glm::angleAxis(-0.4F, glm::vec3{0.0F, 1.0F, 0.0F}));
    Entity platform = scene.CreateEntity("KinematicPlatform");
    platform.SetParent(platformParent);
    platform.set<RigidBody>({BodyMotionType::Kinematic, grasplink::physics::CollisionLayer::Environment})
        .set<Colliders>({{physics_colliders::Box({1.0F, 0.1F, 1.0F})}});
    step();
    platform.SetLocalPosition({0.0F, 0.4F, 0.0F});
    platform.SetLocalRotation(glm::angleAxis(0.2F, glm::vec3{0.0F, 1.0F, 0.0F}));
    step();
    // Kinematic 이동 목표에 도달하면 남은 이동 시간 동안 속도가 0이 된다. 같은 목표를 다시 보내도 물체가 재차 움직이지 않아야 한다.
    step();
    Entity platformDrop = scene.CreateEntity("PlatformDrop");
    platformDrop.SetLocalPosition({5.3F, 1.4F, 0.0F});
    platformDrop.set<RigidBody>({BodyMotionType::Dynamic})
        .set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
    for (int i = 0; i < 500; ++i) step();
    ASSERT_NEAR_FINITE(platformDrop.GetLocalPosition().y, 0.6, 0.02, "Kinematic transformed parent sets physical contact height");
    const glm::vec3 settled = platformDrop.GetLocalPosition();
    for (int i = 0; i < 250; ++i) step();
    ASSERT_NEAR_FINITE(platformDrop.GetLocalPosition().y, settled.y, 0.005, "unchanged Kinematic target has no vertical contact drift");
    ASSERT_NEAR_FINITE(platformDrop.GetLocalPosition().x, settled.x, 0.005, "unchanged Kinematic target has no horizontal contact drift");
    ASSERT_NEAR_FINITE(platformDrop.GetLocalPosition().z, settled.z, 0.005, "unchanged Kinematic target has no depth contact drift");

    // PhysicsSystemModule이 살아 있더라도 이전 Scene 계층과 Entity 연결 기록이 제거된 뒤 다음 물리 step이 안전하게 실행되어야 한다.
    const auto oldRoot = scene.GetSceneRoot();
    sceneOwner.reset();
    ASSERT_TRUE_MESSAGE(!oldRoot.is_alive() && !falling.IsValid() && !drop.IsValid(), "Scene destruction removes its hierarchy and physics bodies");
    step();
}
