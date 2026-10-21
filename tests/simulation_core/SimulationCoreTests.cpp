#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <glm/geometric.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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
