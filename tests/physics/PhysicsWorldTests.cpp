#include "physics/PhysicsWorld.h"

#include <gtest/gtest.h>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace grasplink::physics;

namespace
{
/**
 * @brief 물리 물체 기준점과 실제 바닥 상자 중심이 다른 플랫폼 시험 자료를 만든다.
 * @details Body는 Jolt에서 위치와 회전을 가진 물체다. 이 플랫폼의 상자 중심은 Body 기준점에서 Local X로 0.5 m 떨어져 있고, Z축으로 90도 돌리면 그 차이는 Scene Y 방향이 된다.
 * 상자 반높이 0.25 m까지 더하면 윗면은 Y=0.75 m에 있어야 한다. 반높이 0.1 m인 작은 상자가 여기에 안착하는 높이를 확인하면 계산이 실제 형상 중심 대신 Body 기준점을 사용한 오류를 찾을 수 있다.
 */
BodyDescription OffsetPlatform()
{
    BodyDescription body;
    body.motionType = BodyMotionType::Static;
    body.collisionLayer = CollisionLayer::Environment;
    body.transform.rotation = glm::angleAxis(glm::radians(90.0F), glm::vec3{0.0F, 0.0F, 1.0F});
    CollisionShapeDescription box;
    box.halfExtentsMeters = {0.25F, 0.5F, 0.5F};
    box.localTransform.position = {0.5F, 0.0F, 0.0F};
    body.shapes.push_back(box);
    return body;
}

float ContactHeight(PhysicsWorld& world, float x)
{
    BodyDescription fallingBox;
    CollisionShapeDescription shape;
    shape.halfExtentsMeters = glm::vec3{0.1F};
    fallingBox.shapes.push_back(shape);
    fallingBox.transform.position = {x, 2.0F, 0.0F};
    const auto falling = world.CreateBody(fallingBox);
    for (int i = 0; i < 750; ++i) world.Step(0.004);
    const float height = world.GetBodyTransform(falling).position.y;
    world.DestroyBody(falling);
    return height;
}
}

TEST(PhysicsWorld, BodyOriginAndEnvironmentOverlap)
{
    // 환경 플랫폼 원점과 로봇 proxy의 겹침 조회 및 제외할 환경 물체 지정 동작을 확인한다.
    PhysicsWorld world;
    const auto handle = world.CreateBody(OffsetPlatform());
    ASSERT_EQ(world.GetCollisionLayer(handle), CollisionLayer::Environment) << "body reports its collision category";
    ASSERT_NEAR(world.GetBodyTransform(handle).position.y, 0.0, 1e-5) << "body origin after creation";

    BodyDescription robotProxy;
    robotProxy.motionType = BodyMotionType::Kinematic;
    robotProxy.collisionLayer = CollisionLayer::Robot;
    robotProxy.transform.position = {0.0F, 0.72F, 0.0F};
    CollisionShapeDescription robotBox;
    robotBox.halfExtentsMeters = {0.1F, 0.1F, 0.1F};
    robotProxy.shapes.push_back(robotBox);
    const auto robotHandle = world.CreateBody(robotProxy);
    const auto robotPose = robotProxy.transform;
    ASSERT_TRUE(world.OverlapsEnvironmentAt(robotHandle, robotPose))
        << "robot query detects the offset environment platform";
    for (int query = 0; query < 5000; ++query)
        ASSERT_TRUE(world.OverlapsEnvironmentAt(robotHandle, robotPose))
            << "repeated environment overlap query stays valid";

    BodyDescription secondEnvironment;
    secondEnvironment.motionType = BodyMotionType::Static;
    secondEnvironment.collisionLayer = CollisionLayer::Environment;
    secondEnvironment.transform = robotPose;
    CollisionShapeDescription secondEnvironmentBox;
    secondEnvironmentBox.halfExtentsMeters = {0.1F, 0.1F, 0.1F};
    secondEnvironment.shapes.push_back(secondEnvironmentBox);
    const auto secondEnvironmentHandle = world.CreateBody(secondEnvironment);
    ASSERT_TRUE(world.OverlapsEnvironmentAt(robotHandle, robotPose, handle))
        << "ignoring one environment body still detects another overlapping environment body";
    secondEnvironment.transform.position.x = 10.0F;
    world.SetBodyTransform(secondEnvironmentHandle, secondEnvironment.transform);
    ASSERT_FALSE(world.OverlapsEnvironmentAt(robotHandle, robotPose, handle))
        << "ignoring the only overlapping environment body clears the overlap result";
    world.DestroyBody(secondEnvironmentHandle);
}

TEST(PhysicsWorld, PairOverlapUsesBothCandidateTransforms)
{
    PhysicsWorld world;
    BodyDescription first;
    first.motionType = BodyMotionType::Kinematic;
    first.collisionLayer = CollisionLayer::Robot;
    first.transform.position = {-2.0F, 0.0F, 0.0F};
    first.shapes.push_back(CollisionShapeDescription{});
    const auto firstHandle = world.CreateBody(first);

    BodyDescription second = first;
    second.transform.position = {2.0F, 0.0F, 0.0F};
    const auto secondHandle = world.CreateBody(second);
    ASSERT_FALSE(world.OverlapsBodiesAt(firstHandle, first.transform, secondHandle, second.transform))
        << "separated link candidates do not collide";

    Transform candidateFirst;
    Transform candidateSecond;
    candidateFirst.position = {-0.05F, 0.0F, 0.0F};
    candidateSecond.position = {0.05F, 0.0F, 0.0F};
    ASSERT_TRUE(world.OverlapsBodiesAt(firstHandle, candidateFirst, secondHandle, candidateSecond))
        << "pair query uses proposed FK transforms rather than the stale Jolt body transforms";
    candidateSecond.position.x = 1.0F;
    ASSERT_FALSE(world.OverlapsBodiesAt(firstHandle, candidateFirst, secondHandle, candidateSecond))
        << "a candidate pose that separates both bodies is clear";
    ASSERT_FALSE(world.OverlapsBodiesAt(firstHandle, candidateFirst, firstHandle, candidateFirst))
        << "a proxy is not tested against itself";
}

TEST(PhysicsWorld, OffsetPlatformContactAndTeleport)
{
    PhysicsWorld world;
    auto platform = OffsetPlatform();
    const auto handle = world.CreateBody(platform);
    ASSERT_NEAR(ContactHeight(world, 0.0F), 0.85F, 0.015) << "offset platform contact height";

    platform.transform.position.x = 1.5F;
    world.SetBodyTransform(handle, platform.transform);
    ASSERT_NEAR(world.GetBodyTransform(handle).position.x, 1.5, 1e-5) << "body origin after teleport";
    ASSERT_NEAR(ContactHeight(world, 1.5F), 0.85F, 0.015) << "offset platform contact height";
}

TEST(PhysicsWorld, DynamicTeleportClearsVelocity)
{
    PhysicsWorld world;
    BodyDescription teleportedDynamic;
    teleportedDynamic.motionType = BodyMotionType::Dynamic;
    teleportedDynamic.collisionLayer = CollisionLayer::DynamicObject;
    CollisionShapeDescription teleportShape;
    teleportShape.halfExtentsMeters = glm::vec3{0.1F};
    teleportedDynamic.shapes.push_back(teleportShape);
    teleportedDynamic.transform.position.y = 10.0F;
    const auto dynamicHandle = world.CreateBody(teleportedDynamic);
    world.Step(0.5);
    Transform resetPose;
    resetPose.position = {2.0F, 5.0F, 0.0F};
    world.SetBodyTransform(dynamicHandle, resetPose);
    world.Step(0.004);
    ASSERT_NEAR(world.GetBodyTransform(dynamicHandle).position.y,
        5.0 - 0.5 * 9.81 * 0.004 * 0.004, 1e-4) << "dynamic teleport clears previous velocity";
}

TEST(PhysicsWorld, KinematicMoveAndStop)
{
    PhysicsWorld world;
    const auto staticBody = world.CreateBody(OffsetPlatform());
    auto platform = OffsetPlatform();
    platform.motionType = BodyMotionType::Kinematic;
    platform.collisionLayer = CollisionLayer::Robot;
    platform.transform.position = {3.0F, 0.0F, 0.0F};
    const auto moving = world.CreateBody(platform);
    ASSERT_EQ(world.GetCollisionLayer(moving), CollisionLayer::Robot)
        << "moving robot proxy category is available for contact policy";
    auto target = platform.transform;
    target.position.y = 0.25F;
    target.rotation = glm::angleAxis(0.4F, glm::vec3{0.0F, 1.0F, 0.0F}) * target.rotation;
    world.MoveKinematic(moving, target, 0.004);
    world.Step(0.004);
    ASSERT_NEAR(world.GetBodyTransform(moving).position.y, 0.25, 1e-4) << "kinematic body origin";

    // 질량 중심은 물체의 질량이 균형을 이루는 위치로 Body 기준점과 다를 수 있다. 이 중심이 어긋난 회전 물체도 Kinematic 목표 전송을 멈춘 뒤 계속 미끄러지거나 돌지 않아야 한다.
    world.StopKinematic(moving);
    const Transform stopped = world.GetBodyTransform(moving);
    for (int tick = 0; tick < 250; ++tick) world.Step(0.004);
    const Transform afterStop = world.GetBodyTransform(moving);
    ASSERT_NEAR(glm::length(afterStop.position - stopped.position), 0.0, 1e-5)
        << "stopped kinematic has no linear drift";
    ASSERT_NEAR(std::abs(glm::dot(afterStop.rotation, stopped.rotation)), 1.0, 1e-5)
        << "stopped kinematic has no angular drift";
    ASSERT_THROW(world.StopKinematic(staticBody), std::logic_error);
}

TEST(PhysicsWorld, BodyHandleOwnershipAndReuse)
{
    PhysicsWorld world;
    PhysicsWorld otherWorld;
    const auto handle = world.CreateBody(OffsetPlatform());
    const auto other = otherWorld.CreateBody(OffsetPlatform());
    ASSERT_TRUE(otherWorld.IsBodyValid(other)) << "own World handle";
    ASSERT_FALSE(otherWorld.IsBodyValid(handle)) << "cross-World handle must fail";
    ASSERT_THROW(otherWorld.StopKinematic(handle), std::invalid_argument);
    ASSERT_THROW(otherWorld.GetBodyTransform(handle), std::invalid_argument);
    world.DestroyBody(handle);
    ASSERT_FALSE(world.IsBodyValid(handle)) << "destroyed handle must fail";
    const auto replacement = world.CreateBody(OffsetPlatform());
    ASSERT_TRUE(world.IsBodyValid(replacement) && !world.IsBodyValid(handle))
        << "reused ID must not revive old handle";
}

TEST(PhysicsWorld, RejectsInvalidBodyInputs)
{
    PhysicsWorld world;
    // 잘못된 숫자와 회전값은 Jolt 내부로 전달하기 전에 검사해, 호출자가 입력 오류를 예측 가능한 예외로 확인할 수 있게 한다.
    auto invalid = OffsetPlatform();
    invalid.transform.position.x = std::numeric_limits<float>::quiet_NaN();
    ASSERT_THROW(world.CreateBody(invalid), std::invalid_argument);
    invalid = OffsetPlatform();
    invalid.shapes.front().localTransform.rotation = glm::quat{0.0F, 0.0F, 0.0F, 0.0F};
    ASSERT_THROW(world.CreateBody(invalid), std::invalid_argument);
}
