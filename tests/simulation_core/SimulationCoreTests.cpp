#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <glm/geometric.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aerosimlab::simulation
{
namespace
{
DroneCommand Command(
    DroneCommand::Kind kind,
    glm::dvec3 vector = glm::dvec3{0.0},
    double yawRate = 0.0)
{
    return DroneCommand{kind, vector, yawRate};
}

void ReachHovering(SimulationCore& simulation)
{
    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::Accepted);
    for (int tick = 0; tick < 120; ++tick)
    {
        simulation.AdvanceOneTick();
    }
    ASSERT_EQ(simulation.State().mode, FlightMode::Hovering);
}
}

TEST(SimulationCore, ExecutesTakeoffMovementAndLandingOnFixedTicks)
{
    SimulationCore simulation;
    ReachHovering(simulation);
    EXPECT_NEAR(simulation.State().positionMeters.z, 1.5, 1.0e-9);

    ASSERT_EQ(simulation.Apply(Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{2.0, 0.0, 0.0})), CommandResult::Accepted);
    for (int tick = 0; tick < 60; ++tick)
    {
        simulation.AdvanceOneTick();
    }
    EXPECT_NEAR(simulation.State().positionMeters.x, 2.0, 1.0e-9);

    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::Land)), CommandResult::Accepted);
    EXPECT_EQ(simulation.Apply(Command(DroneCommand::Kind::Land)), CommandResult::InvalidFlightMode);
    for (int tick = 0; tick < 120; ++tick)
    {
        simulation.AdvanceOneTick();
    }
    EXPECT_EQ(simulation.State().mode, FlightMode::Grounded);
    EXPECT_DOUBLE_EQ(simulation.State().positionMeters.z, 0.0);
}

TEST(SimulationCore, KinematicVelocityCommandStopsAtGround)
{
    SimulationCore simulation;
    ReachHovering(simulation);
    ASSERT_EQ(simulation.Apply(Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{0.0, 0.0, -5.0})),
        CommandResult::Accepted);

    for (int tick = 0; tick < 30; ++tick)
    {
        simulation.AdvanceOneTick();
    }

    EXPECT_EQ(simulation.State().mode, FlightMode::Grounded);
    EXPECT_DOUBLE_EQ(simulation.State().positionMeters.z, 0.0);
    EXPECT_EQ(simulation.State().velocityMetersPerSecond, glm::dvec3{0.0});
}

TEST(FixedTickAccumulator, CarriesFractionalTimeAndLimitsCatchUp)
{
    FixedTickAccumulator accumulator;
    const double halfTick = FixedTickAccumulator::fixedStepSeconds * 0.5;

    EXPECT_EQ(accumulator.Consume(halfTick), 0);
    EXPECT_EQ(accumulator.Consume(halfTick), 1);
    EXPECT_EQ(accumulator.Consume(FixedTickAccumulator::maximumFrameSeconds), 6);
    EXPECT_EQ(accumulator.Consume(1.0), 6);
    EXPECT_EQ(accumulator.Consume(0.0), 0);
}

TEST(SimulationCore, RejectsCommandsThatDoNotMatchFlightStateOrContainInvalidNumbers)
{
    SimulationCore simulation;
    EXPECT_EQ(simulation.Apply(Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{1.0, 0.0, 0.0})),
        CommandResult::InvalidFlightMode);
    EXPECT_EQ(simulation.Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::Accepted);
    EXPECT_EQ(simulation.Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::InvalidFlightMode);
    EXPECT_EQ(simulation.Apply(Command(DroneCommand::Kind::SetWaypoint,
        glm::dvec3{1.0, 0.0, 1.0})), CommandResult::InvalidFlightMode);

    for (int tick = 0; tick < 90; ++tick)
    {
        simulation.AdvanceOneTick();
    }
    EXPECT_EQ(simulation.Apply(Command(DroneCommand::Kind::SetWaypoint,
        glm::dvec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0})), CommandResult::InvalidValue);
    EXPECT_EQ(simulation.State().mode, FlightMode::Hovering);
}

TEST(SimulationCore, FollowsWaypointAndClampsVelocityAndYaw)
{
    SimulationCore simulation;
    ReachHovering(simulation);
    ASSERT_EQ(simulation.Apply(Command(
        DroneCommand::Kind::SetWaypoint, glm::dvec3{1.0, 0.0, 1.5})),
        CommandResult::Accepted);
    for (int tick = 0; tick < 120; ++tick)
    {
        simulation.AdvanceOneTick();
    }
    EXPECT_EQ(simulation.State().mode, FlightMode::Hovering);
    EXPECT_DOUBLE_EQ(simulation.State().positionMeters.x, 1.0);

    ASSERT_EQ(simulation.Apply(Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{50.0, 0.0, 0.0})),
        CommandResult::Accepted);
    EXPECT_DOUBLE_EQ(glm::length(simulation.State().velocityMetersPerSecond), 5.0);

    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::SetYawRate, glm::dvec3{0.0}, 100.0)),
        CommandResult::Accepted);
    for (int tick = 0; tick < 60; ++tick)
    {
        simulation.AdvanceOneTick();
    }
    EXPECT_NEAR(simulation.State().yawRadians, 1.5, 1.0e-9);

    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::SetYawRate)), CommandResult::Accepted);
    simulation.AdvanceOneTick();
    EXPECT_NEAR(simulation.State().yawRadians, 1.5, 1.0e-9);
}

TEST(SimulationCore, PauseResumeAndResetControlSimulationTime)
{
    SimulationCore simulation;
    ReachHovering(simulation);
    const glm::dvec3 positionBeforePause = simulation.State().positionMeters;
    simulation.Pause();
    EXPECT_EQ(simulation.Apply(Command(DroneCommand::Kind::SetVelocity,
        glm::dvec3{3.0, 0.0, 0.0})), CommandResult::SimulationPaused);
    simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.TickCount(), 120);
    EXPECT_EQ(simulation.State().positionMeters, positionBeforePause);
    EXPECT_EQ(simulation.State().mode, FlightMode::Hovering);

    simulation.Resume();
    simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.TickCount(), 121);
    EXPECT_NEAR(simulation.SimulationTimeSeconds(), 121.0 / 60.0, 1.0e-12);

    simulation.Reset();
    EXPECT_EQ(simulation.TickCount(), 0);
    EXPECT_DOUBLE_EQ(simulation.SimulationTimeSeconds(), 0.0);
    EXPECT_EQ(simulation.State().mode, FlightMode::Grounded);
}

TEST(SimulationCore, SameCommandSequenceProducesTheSameState)
{
    SimulationCore first;
    SimulationCore second;
    for (SimulationCore* simulation : {&first, &second})
    {
        ASSERT_EQ(simulation->Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::Accepted);
        for (int tick = 0; tick < 90; ++tick)
        {
            simulation->AdvanceOneTick();
        }
        ASSERT_EQ(simulation->Apply(Command(
            DroneCommand::Kind::SetWaypoint, glm::dvec3{3.0, 2.0, 1.5})),
            CommandResult::Accepted);
        for (int tick = 0; tick < 60; ++tick)
        {
            simulation->AdvanceOneTick();
        }
    }
    EXPECT_EQ(first.State().positionMeters, second.State().positionMeters);
    EXPECT_EQ(first.State().velocityMetersPerSecond, second.State().velocityMetersPerSecond);
    EXPECT_EQ(first.State().mode, second.State().mode);
    EXPECT_EQ(first.TickCount(), second.TickCount());
}

TEST(SimulationCore, RigidBodyModeUsesRotorForcesForTakeoffAndLanding)
{
    SimulationCore simulation{SimulationDynamics::RigidBody};
    simulation.SetPhysicsStepTimingEnabled(true);
    simulation.AdvanceOneTick();
    EXPECT_GT(simulation.LastPhysicsStepDurationMicroseconds(), 0.0);
    simulation.SetPhysicsStepTimingEnabled(false);

    ASSERT_EQ(
        simulation.Apply(Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);

    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();

    ASSERT_EQ(simulation.State().mode, FlightMode::Hovering);
    EXPECT_NEAR(simulation.State().positionMeters.z, DroneModel::takeoffHeightMeters, 0.08);
    EXPECT_NEAR(simulation.State().velocityMetersPerSecond.z, 0.0, 0.15);
    EXPECT_NEAR(glm::length(simulation.State().orientation), 1.0, 1.0e-5);

    ASSERT_EQ(simulation.Apply(Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{1.0, 0.0, 0.0})), CommandResult::Accepted);
    for (int tick = 0; tick < 120; ++tick)
        simulation.AdvanceOneTick();
    EXPECT_GT(simulation.State().positionMeters.x, 0.2);
    EXPECT_GT(std::abs(simulation.State().orientation.y), 0.005);
    EXPECT_NEAR(glm::length(simulation.State().orientation), 1.0, 1.0e-5);

    const glm::dvec3 waypoint = simulation.State().positionMeters + glm::dvec3{1.0, 0.0, 0.0};
    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::SetWaypoint, waypoint)),
        CommandResult::Accepted);
    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.State().mode, FlightMode::Hovering);
    EXPECT_NEAR(simulation.State().positionMeters.x, waypoint.x, 0.08);

    ASSERT_EQ(
        simulation.Apply(Command(DroneCommand::Kind::SetYawRate, glm::dvec3{0.0}, 0.5)),
        CommandResult::Accepted);
    for (int tick = 0; tick < 120; ++tick)
        simulation.AdvanceOneTick();
    EXPECT_GT(simulation.State().yawRadians, 0.3);

    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::Land)), CommandResult::Accepted);
    for (int tick = 0; tick < 360; ++tick)
        simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.State().mode, FlightMode::Grounded);
    EXPECT_NEAR(
        simulation.State().positionMeters.z,
        QuadrotorFlightController::groundedModelOriginHeightMeters,
        0.02);
    EXPECT_NEAR(simulation.State().velocityMetersPerSecond.z, 0.0, 0.15);

    simulation.Reset();
    EXPECT_EQ(simulation.TickCount(), 0);
    EXPECT_NEAR(simulation.State().positionMeters.z,
        QuadrotorFlightController::groundedModelOriginHeightMeters, 1.0e-6);
}

TEST(SimulationCore, MultipleKinematicDronesKeepIndependentStateAndCommands)
{
    SimulationCore simulation;
    const DroneId first = simulation.PrimaryDroneId();
    const DroneId second = simulation.AddDrone(glm::dvec3{10.0, 0.0, 0.0});
    ASSERT_EQ(simulation.DroneCount(), 2U);

    ASSERT_EQ(simulation.Apply(first, Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);
    ASSERT_EQ(simulation.Apply(second, Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);
    for (int tick = 0; tick < 90; ++tick)
        simulation.AdvanceOneTick();

    ASSERT_EQ(simulation.State(first).mode, FlightMode::Hovering);
    ASSERT_EQ(simulation.State(second).mode, FlightMode::Hovering);
    EXPECT_DOUBLE_EQ(simulation.State(first).positionMeters.x, 0.0);
    EXPECT_DOUBLE_EQ(simulation.State(second).positionMeters.x, 10.0);

    ASSERT_EQ(simulation.Apply(first, Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{1.0, 0.0, 0.0})),
        CommandResult::Accepted);
    ASSERT_EQ(simulation.Apply(second, Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{-2.0, 0.0, 0.0})),
        CommandResult::Accepted);
    for (int tick = 0; tick < 30; ++tick)
        simulation.AdvanceOneTick();

    EXPECT_NEAR(simulation.State(first).positionMeters.x, 0.5, 1.0e-12);
    EXPECT_NEAR(simulation.State(second).positionMeters.x, 9.0, 1.0e-12);
    EXPECT_EQ(simulation.Apply(999, Command(DroneCommand::Kind::Stop)),
        CommandResult::DroneNotFound);

    ASSERT_TRUE(simulation.RemoveDrone(first));
    EXPECT_EQ(simulation.DroneCount(), 1U);
    EXPECT_EQ(simulation.PrimaryDroneId(), second);
    EXPECT_NEAR(simulation.State().positionMeters.x, 9.0, 1.0e-12);
    EXPECT_FALSE(simulation.RemoveDrone(second));
    EXPECT_THROW((void)simulation.State(first), std::out_of_range);
}

TEST(SimulationCore, RigidBodyDronesShareOnePhysicsStepAndCanBeRemoved)
{
    SimulationCore simulation{SimulationDynamics::RigidBody};
    const DroneId first = simulation.PrimaryDroneId();
    const DroneId second = simulation.AddDrone(glm::dvec3{
        2.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters});
    ASSERT_EQ(simulation.Apply(first, Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);
    ASSERT_EQ(simulation.Apply(second, Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);

    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();

    ASSERT_EQ(simulation.State(first).mode, FlightMode::Hovering);
    ASSERT_EQ(simulation.State(second).mode, FlightMode::Hovering);
    EXPECT_NEAR(simulation.State(second).positionMeters.x
            - simulation.State(first).positionMeters.x,
        2.0, 0.02);
    EXPECT_NEAR(simulation.State(second).positionMeters.z,
        simulation.State(first).positionMeters.z, 0.02);

    ASSERT_TRUE(simulation.RemoveDrone(second));
    EXPECT_EQ(simulation.DroneCount(), 1U);
    simulation.AdvanceOneTick();
    EXPECT_TRUE(std::isfinite(simulation.State(first).positionMeters.z));
}

TEST(SimulationCore, CollidingRigidBodyDronesRemainSeparated)
{
    SimulationCore simulation{SimulationDynamics::RigidBody};
    const DroneId first = simulation.PrimaryDroneId();
    const DroneId second = simulation.AddDrone({
        2.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters});
    ASSERT_EQ(simulation.Apply(first, Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);
    ASSERT_EQ(simulation.Apply(second, Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);
    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();

    ASSERT_EQ(simulation.Apply(first, Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{1.0, 0.0, 0.0})),
        CommandResult::Accepted);
    ASSERT_EQ(simulation.Apply(second, Command(
        DroneCommand::Kind::SetVelocity, glm::dvec3{-1.0, 0.0, 0.0})),
        CommandResult::Accepted);
    double minimumCenterDistance = std::numeric_limits<double>::infinity();
    for (int tick = 0; tick < 120; ++tick)
    {
        simulation.AdvanceOneTick();
        const glm::dvec3 offset = simulation.State(second).positionMeters
            - simulation.State(first).positionMeters;
        minimumCenterDistance = std::min(minimumCenterDistance, glm::length(offset));
    }

    EXPECT_GT(minimumCenterDistance, 0.30);
    EXPECT_LT(minimumCenterDistance, 0.55);
}

TEST(SimulationCore, SameRigidBodyCommandsProduceTheSameState)
{
    SimulationCore first{SimulationDynamics::RigidBody};
    SimulationCore second{SimulationDynamics::RigidBody};
    for (SimulationCore* simulation : {&first, &second})
    {
        ASSERT_EQ(simulation->Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::Accepted);
        for (int tick = 0; tick < 240; ++tick)
            simulation->AdvanceOneTick();
        ASSERT_EQ(simulation->Apply(Command(
            DroneCommand::Kind::SetVelocity, glm::dvec3{1.0, -0.5, 0.0})),
            CommandResult::Accepted);
        for (int tick = 0; tick < 120; ++tick)
            simulation->AdvanceOneTick();
    }

    EXPECT_EQ(first.State().positionMeters, second.State().positionMeters);
    EXPECT_EQ(first.State().velocityMetersPerSecond, second.State().velocityMetersPerSecond);
    EXPECT_EQ(first.State().orientation, second.State().orientation);
    EXPECT_EQ(first.State().angularVelocityRadiansPerSecond,
        second.State().angularVelocityRadiansPerSecond);
}

TEST(SimulationCore, RigidBodyHoverStaysStableForTenMinutes)
{
    SimulationCore simulation{SimulationDynamics::RigidBody};
    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::Accepted);
    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();
    ASSERT_EQ(simulation.State().mode, FlightMode::Hovering);

    constexpr int ticksPerSecond = 60;
    constexpr int hoverSeconds = 10 * 60;
    for (int second = 0; second < hoverSeconds; ++second)
    {
        for (int tick = 0; tick < ticksPerSecond; ++tick)
            simulation.AdvanceOneTick();

        const DroneState& state = simulation.State();
        const std::array stateValues{
            state.positionMeters.x, state.positionMeters.y, state.positionMeters.z,
            state.velocityMetersPerSecond.x, state.velocityMetersPerSecond.y,
            state.velocityMetersPerSecond.z, state.orientation.w, state.orientation.x,
            state.orientation.y, state.orientation.z,
            state.angularVelocityRadiansPerSecond.x,
            state.angularVelocityRadiansPerSecond.y,
            state.angularVelocityRadiansPerSecond.z};
        ASSERT_TRUE(std::all_of(stateValues.begin(), stateValues.end(), [](double value)
        {
            return std::isfinite(value);
        })) << "simulated second " << second + 1;
        ASSERT_EQ(state.mode, FlightMode::Hovering);
        EXPECT_NEAR(state.positionMeters.z, DroneModel::takeoffHeightMeters, 0.08)
            << "simulated second " << second + 1;
        EXPECT_NEAR(glm::length(state.orientation), 1.0, 1.0e-5)
            << "simulated second " << second + 1;
    }

    EXPECT_EQ(simulation.TickCount(), 240 + hoverSeconds * ticksPerSecond);
    EXPECT_LT(glm::length(simulation.State().velocityMetersPerSecond), 0.15);
}

TEST(SimulationCore, CameraStopsOnDisableOrFaultAndResumesAfterRecovery)
{
    SimulationCore simulation;
    simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.Camera().frameCount, 1);
    EXPECT_EQ(simulation.Camera().lastFrameTick, 1);

    simulation.SetCameraEnabled(false);
    simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.Camera().status, CameraStatus::Disabled);
    EXPECT_EQ(simulation.Camera().frameCount, 1);

    simulation.SetCameraEnabled(true);
    simulation.SetCameraFault(true);
    simulation.SetCameraEnabled(true);
    simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.Camera().status, CameraStatus::Fault);
    EXPECT_EQ(simulation.Camera().frameCount, 1);

    simulation.SetCameraFault(false);
    simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.Camera().status, CameraStatus::Active);
    EXPECT_EQ(simulation.Camera().frameCount, 2);
    EXPECT_EQ(simulation.Camera().lastFrameTick, 4);
}
}
