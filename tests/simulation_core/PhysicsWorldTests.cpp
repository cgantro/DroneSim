#include "simulation_core/PhysicsWorld.h"
#include "simulation_core/QuadrotorModel.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace aerosimlab::simulation
{
namespace
{
RigidBodyHandle MakeBody(PhysicsWorld& world, glm::dvec3 position)
{
    RigidBodyDefinition definition;
    definition.halfExtentsMeters = {0.1F, 0.1F, 0.1F};
    definition.positionMeters = position;
    definition.massKilograms = 2.0;
    return world.CreateBox(definition);
}
}

TEST(PhysicsWorld, GravityAcceleratesAFreeBodyDownward)
{
    PhysicsWorld world;
    constexpr double stepSeconds = 1.0 / 60.0;
    constexpr int tickCount = 60;
    constexpr double gravityMetersPerSecondSquared = 9.81;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 10.0});

    for (int tick = 0; tick < tickCount; ++tick)
        world.Step(stepSeconds);

    const RigidBodyState state = world.State(body);
    // Jolt는 매 단계에서 속도를 먼저 갱신한 뒤, 그 속도로 위치를 움직인다.
    const double expectedPosition = 10.0
        - gravityMetersPerSecondSquared * stepSeconds * stepSeconds
            * tickCount * (tickCount + 1) / 2.0;
    EXPECT_NEAR(state.linearVelocityMetersPerSecond.z,
        -gravityMetersPerSecondSquared * tickCount * stepSeconds, 0.03);
    EXPECT_NEAR(state.positionMeters.z, expectedPosition, 0.01);
}

TEST(PhysicsWorld, UpwardForceRaisesTheBody)
{
    PhysicsWorld world;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 10.0});

    for (int tick = 0; tick < 60; ++tick)
    {
        world.ApplyForce(body, {0.0, 0.0, 24.0});
        world.Step(1.0 / 60.0);
    }

    EXPECT_GT(world.State(body).linearVelocityMetersPerSecond.z, 1.0);
}

TEST(PhysicsWorld, X500RotorThrustKeepsTheBodyHovering)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    const double hoverSpeed = std::sqrt(
        parameters.massKilograms * 9.81
        / (QuadrotorModel::rotorCount * parameters.thrustCoefficientNewtonsPerSpeedSquared));
    QuadrotorState motors;
    motors.motorSpeedRadiansPerSecond.fill(hoverSpeed);
    const QuadrotorWrench wrench = QuadrotorModel::CalculateWrench(motors, parameters);

    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.positionMeters = {0.0, 0.0, 2.0};
    definition.massKilograms = parameters.massKilograms;
    definition.principalInertiaKilogramMetersSquared =
        parameters.principalInertiaKilogramMetersSquared;
    const RigidBodyHandle body = world.CreateBox(definition);

    for (int tick = 0; tick < 120; ++tick)
    {
        world.ApplyForce(body, wrench.forceNewtons);
        world.ApplyTorque(body, wrench.torqueNewtonMeters);
        world.Step(1.0 / 60.0);
    }

    const RigidBodyState result = world.State(body);
    EXPECT_NEAR(result.positionMeters.z, 2.0, 0.01);
    EXPECT_NEAR(result.linearVelocityMetersPerSecond.z, 0.0, 0.01);
}

TEST(PhysicsWorld, AngularResponseUsesProvidedInertia)
{
    PhysicsWorld world;
    RigidBodyDefinition lowInertiaDefinition;
    lowInertiaDefinition.positionMeters = {-1.0, 0.0, 10.0};
    lowInertiaDefinition.principalInertiaKilogramMetersSquared = {0.02, 0.02, 0.02};
    const RigidBodyHandle lowInertiaBody = world.CreateBox(lowInertiaDefinition);

    RigidBodyDefinition highInertiaDefinition;
    highInertiaDefinition.positionMeters = {1.0, 0.0, 10.0};
    highInertiaDefinition.principalInertiaKilogramMetersSquared = {0.02, 0.02, 0.08};
    const RigidBodyHandle highInertiaBody = world.CreateBox(highInertiaDefinition);

    for (int tick = 0; tick < 60; ++tick)
    {
        world.ApplyTorque(lowInertiaBody, {0.0, 0.0, 0.2});
        world.ApplyTorque(highInertiaBody, {0.0, 0.0, 0.2});
        world.Step(1.0 / 60.0);
    }

    EXPECT_GT(
        world.State(lowInertiaBody).angularVelocityRadiansPerSecond.z,
        world.State(highInertiaBody).angularVelocityRadiansPerSecond.z);
}

TEST(PhysicsWorld, TorqueChangesAngularVelocityAndOrientation)
{
    PhysicsWorld world;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 10.0});

    for (int tick = 0; tick < 30; ++tick)
    {
        world.ApplyTorque(body, {0.0, 0.0, 0.2});
        world.Step(1.0 / 60.0);
    }

    const RigidBodyState state = world.State(body);
    EXPECT_GT(state.angularVelocityRadiansPerSecond.z, 0.1);
    EXPECT_GT(std::abs(state.orientation.z), 0.01);
}

TEST(PhysicsWorld, GroundStopsAFallingBody)
{
    PhysicsWorld world;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 2.0});

    for (int tick = 0; tick < 240; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(body);
    EXPECT_GE(state.positionMeters.z, 0.08);
    EXPECT_LT(std::abs(state.linearVelocityMetersPerSecond.z), 0.1);
}

TEST(PhysicsWorld, RejectsInvalidBodySize)
{
    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.halfExtentsMeters.x = 0.0F;
    EXPECT_THROW(static_cast<void>(world.CreateBox(definition)), std::invalid_argument);
}

TEST(PhysicsWorld, RejectsInvalidPrincipalInertia)
{
    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.principalInertiaKilogramMetersSquared = {0.02, 0.0, 0.04};

    EXPECT_THROW(static_cast<void>(world.CreateBox(definition)), std::invalid_argument);
}
}
