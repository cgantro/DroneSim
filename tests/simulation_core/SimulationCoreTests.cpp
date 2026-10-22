#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
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

void ReachHovering(SimulationCore& simulation, DroneId droneId)
{
    ASSERT_EQ(simulation.Apply(droneId, Command(DroneCommand::Kind::TakeOff)),
        CommandResult::Accepted);
    for (int tick = 0; tick < 120; ++tick)
        simulation.AdvanceOneTick();
    ASSERT_EQ(simulation.State(droneId).mode, FlightMode::Hovering);
}

void ExpectCameraPoseMatchesPrimaryDrone(const SimulationCore& simulation)
{
    const DroneState& drone = simulation.State();
    const CameraState& camera = simulation.Camera();
    const glm::dvec3 mountPositionBodyMeters{0.35, 0.0, 0.12};
    const glm::dvec3 expectedPosition = drone.positionMeters
        + drone.orientation * mountPositionBodyMeters;

    EXPECT_NEAR(glm::length(camera.positionMeters - expectedPosition), 0.0, 1.0e-10);
    EXPECT_EQ(camera.orientation, drone.orientation);
}
}

void Verify_SimulationCore_ExecutesTakeoffMovementAndLandingOnFixedTicks()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
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

void Verify_SimulationCore_InitialYawSetsOrientationAndSurvivesReset()
{
    constexpr double initialYawRadians = 1.2;
    const glm::dquat expectedOrientation = glm::angleAxis(
        initialYawRadians, glm::dvec3{0.0, 0.0, 1.0});

    SimulationCore kinematic{SimulationDynamics::Kinematic};
    const DroneId kinematicDrone = kinematic.AddDrone(
        glm::dvec3{2.0, 3.0, 1.0}, initialYawRadians);
    EXPECT_EQ(kinematic.DroneCount(), 2U);
    EXPECT_EQ(kinematic.DroneIds(), (std::vector<DroneId>{1U, kinematicDrone}));
    EXPECT_NEAR(kinematic.State(kinematicDrone).yawRadians, initialYawRadians, 1.0e-12);
    EXPECT_EQ(kinematic.State(kinematicDrone).orientation, expectedOrientation);
    kinematic.Reset();
    EXPECT_NEAR(kinematic.State(kinematicDrone).yawRadians, initialYawRadians, 1.0e-12);
    EXPECT_EQ(kinematic.State(kinematicDrone).orientation, expectedOrientation);

#ifndef __EMSCRIPTEN__
    SimulationCore rigidBody{SimulationDynamics::RigidBody};
    const DroneId rigidBodyDrone = rigidBody.AddDrone(
        glm::dvec3{2.0, 3.0, 1.5}, initialYawRadians);
    EXPECT_NEAR(rigidBody.State(rigidBodyDrone).yawRadians, initialYawRadians, 1.0e-6);
    rigidBody.Reset();
    EXPECT_NEAR(rigidBody.State(rigidBodyDrone).yawRadians, initialYawRadians, 1.0e-6);
    EXPECT_DOUBLE_EQ(
        rigidBody.State(rigidBodyDrone).positionMeters.z,
        1.5);
#endif
}

void Verify_SimulationCore_KinematicVelocityCommandStopsAtGround()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
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

#ifndef __EMSCRIPTEN__
void Verify_SimulationCore_RejectsTerrainWhenRigidBodyModeIsNotEnabled()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
    terrain::TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.spacingXMeters = 1.0F;
    terrain.spacingYMeters = 1.0F;
    terrain.heightOffsetsMeters.assign(9, 0.0F);

    EXPECT_THROW(simulation.SetTerrain(terrain), std::logic_error);
}
#endif

void Verify_FixedTickAccumulator_CarriesFractionalTimeAndLimitsCatchUp()
{
    FixedTickAccumulator accumulator;
    const double halfTick = FixedTickAccumulator::fixedStepSeconds * 0.5;

    EXPECT_EQ(accumulator.Consume(halfTick), 0);
    EXPECT_EQ(accumulator.Consume(halfTick), 1);
    EXPECT_EQ(accumulator.Consume(FixedTickAccumulator::maximumFrameSeconds), 6);
    EXPECT_EQ(accumulator.Consume(1.0), 6);
    EXPECT_EQ(accumulator.Consume(0.0), 0);

    FixedTickAccumulator invalidInputAccumulator;
    EXPECT_EQ(invalidInputAccumulator.Consume(halfTick), 0);
    EXPECT_EQ(invalidInputAccumulator.Consume(-halfTick), 0);
    EXPECT_EQ(invalidInputAccumulator.Consume(std::numeric_limits<double>::quiet_NaN()), 0);
    EXPECT_EQ(invalidInputAccumulator.Consume(halfTick), 1);
}

void Verify_SimulationCore_RejectsCommandsThatDoNotMatchFlightStateOrContainInvalidNumbers()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
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

void Verify_SimulationCore_FollowsThreeWaypointsAndClampsVelocityAndYaw()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
    ReachHovering(simulation);
    const std::array<glm::dvec3, 3> waypoints{
        glm::dvec3{1.0, 0.0, 1.5},
        glm::dvec3{1.0, 1.0, 1.5},
        glm::dvec3{0.0, 1.0, 1.5}};
    for (const glm::dvec3& waypoint : waypoints)
    {
        ASSERT_EQ(simulation.Apply(Command(
            DroneCommand::Kind::SetWaypoint, waypoint)), CommandResult::Accepted);
        for (int tick = 0; tick < 20; ++tick)
            simulation.AdvanceOneTick();
        EXPECT_EQ(simulation.State().mode, FlightMode::Hovering);
        EXPECT_EQ(simulation.State().positionMeters, waypoint);
    }

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

void Verify_SimulationCore_PauseResumeAndResetControlSimulationTime()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
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

void Verify_SimulationCore_DefaultsToAirSimAndUsesRotorForcesForTakeoffAndLanding()
{
    SimulationCore simulation;
    simulation.SetPhysicsStepTimingEnabled(true);
    simulation.AdvanceOneTick();
    EXPECT_GT(simulation.LastPhysicsStepDurationMicroseconds(), 0.0);
    EXPECT_GT(simulation.LastContactGenerationDurationMicroseconds(), 0.0);
    simulation.SetPhysicsStepTimingEnabled(false);
    simulation.AdvanceOneTick();
    EXPECT_DOUBLE_EQ(simulation.LastContactGenerationDurationMicroseconds(), 0.0);

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

#ifndef __EMSCRIPTEN__
void Verify_SimulationCore_WorldWindMovesRotatedHoveringDroneDownwind()
{
    constexpr double yawRadians = std::numbers::pi / 2.0;
    const glm::dvec3 startPosition{5.0, 0.0, 1.5};
    const glm::dvec3 windVelocity{2.0, 0.0, 0.0};

    SimulationCore calmAir{SimulationDynamics::RigidBody};
    SimulationCore withWind{SimulationDynamics::RigidBody};
    const DroneId calmDrone = calmAir.AddDrone(startPosition, yawRadians);
    const DroneId windDrone = withWind.AddDrone(startPosition, yawRadians);
    ReachHovering(calmAir, calmDrone);
    ReachHovering(withWind, windDrone);

    withWind.SetWindVelocityMetersPerSecond(windVelocity);
    calmAir.AdvanceOneTick();
    withWind.AdvanceOneTick();

    // Yaw가 90°이면 월드 +X 바람은 기체 옆으로 흐르고, 드론을 바람 방향으로 밉니다.
    const glm::dvec3 velocityChange =
        withWind.State(windDrone).velocityMetersPerSecond
        - calmAir.State(calmDrone).velocityMetersPerSecond;
    EXPECT_GT(velocityChange.x, 1.0e-5);
    EXPECT_NEAR(velocityChange.y, 0.0, 1.0e-5);
}
#endif

void Verify_SimulationCore_MultipleKinematicDronesKeepIndependentStateAndCommands()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
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
    ExpectCameraPoseMatchesPrimaryDrone(simulation);
    EXPECT_NEAR(simulation.State().positionMeters.x, 9.0, 1.0e-12);
    EXPECT_FALSE(simulation.RemoveDrone(second));
    EXPECT_THROW((void)simulation.State(first), std::out_of_range);
}

void Verify_SimulationCore_RigidBodyDronesShareOnePhysicsStepAndCanBeRemoved()
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

void Verify_SimulationCore_CollidingRigidBodyDronesRemainSeparated()
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
    EXPECT_LT(minimumCenterDistance, 0.9);
}

void Verify_SimulationCore_RigidBodyRemainsStableDuringOneMinuteHover()
{
    SimulationCore simulation{SimulationDynamics::RigidBody};
    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::Accepted);
    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();
    ASSERT_EQ(simulation.State().mode, FlightMode::Hovering);

    constexpr int ticksPerSecond = 60;
    constexpr int hoverSeconds = 60;
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
        EXPECT_NEAR(state.positionMeters.z, DroneModel::takeoffHeightMeters, 0.15)
            << "simulated second " << second + 1;
        EXPECT_NEAR(glm::length(state.orientation), 1.0, 1.0e-5)
            << "simulated second " << second + 1;
    }

    EXPECT_EQ(simulation.TickCount(), 240 + hoverSeconds * ticksPerSecond);
    EXPECT_LT(glm::length(simulation.State().velocityMetersPerSecond), 0.15);
}

void Verify_SimulationCore_RigidBodyTakesOffFromTheSelectedTerrainStart()
{
    SimulationCore simulation{SimulationDynamics::RigidBody};
    const aerosimlab::terrain::TerrainGrid terrain = aerosimlab::terrain::TerrainGrid::Load(
        std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/bukhansan-demo/terrain.grid");
    simulation.SetTerrain(terrain);
    const glm::dvec2 startMeters{171.3545, -92.7667};
    simulation.SetInitialPose({startMeters,
        terrain.HeightAtLocalMeters(startMeters)
            + QuadrotorFlightController::groundedModelOriginHeightMeters});
    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::TakeOff)), CommandResult::Accepted);
    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();

    EXPECT_EQ(simulation.State().mode, FlightMode::Hovering)
        << "height=" << simulation.State().positionMeters.z;

    ASSERT_EQ(simulation.Apply(Command(DroneCommand::Kind::Land)), CommandResult::Accepted);
    for (int tick = 0; tick < 240; ++tick)
        simulation.AdvanceOneTick();
    EXPECT_EQ(simulation.State().mode, FlightMode::Grounded);
    EXPECT_NEAR(simulation.State().positionMeters.z,
        terrain.HeightAtLocalMeters(startMeters)
            + QuadrotorFlightController::groundedModelOriginHeightMeters,
        0.05);

    simulation.Reset();
    EXPECT_EQ(simulation.State().mode, FlightMode::Grounded);
    EXPECT_NEAR(simulation.State().positionMeters.z,
        terrain.HeightAtLocalMeters(startMeters)
            + QuadrotorFlightController::groundedModelOriginHeightMeters,
        1.0e-6);
}

void Verify_SimulationCore_CameraStopsOnDisableOrFaultAndResumesAfterRecovery()
{
    SimulationCore simulation{SimulationDynamics::Kinematic};
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

void Verify_SimulationCore_EoCameraPoseTracksPrimaryDroneThroughTicksAndReset()
{
    SimulationCore kinematic{SimulationDynamics::Kinematic};
    kinematic.SetInitialPose({2.0, -3.0, 4.0}, 1.1);
    ExpectCameraPoseMatchesPrimaryDrone(kinematic);
    kinematic.AdvanceOneTick();
    ExpectCameraPoseMatchesPrimaryDrone(kinematic);
    kinematic.Reset();
    ExpectCameraPoseMatchesPrimaryDrone(kinematic);

#ifndef __EMSCRIPTEN__
    SimulationCore rigidBody{SimulationDynamics::RigidBody};
    rigidBody.SetInitialPose({2.0, -3.0, 1.0}, 1.1);
    ExpectCameraPoseMatchesPrimaryDrone(rigidBody);
    rigidBody.AdvanceOneTick();
    ExpectCameraPoseMatchesPrimaryDrone(rigidBody);
    rigidBody.Reset();
    ExpectCameraPoseMatchesPrimaryDrone(rigidBody);
#endif
}

TEST(SimulationCoreTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_ExecutesTakeoffMovementAndLandingOnFixedTicks());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_InitialYawSetsOrientationAndSurvivesReset());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_KinematicVelocityCommandStopsAtGround());
#ifndef __EMSCRIPTEN__
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_RejectsTerrainWhenRigidBodyModeIsNotEnabled());
#endif
    EXPECT_NO_FATAL_FAILURE(Verify_FixedTickAccumulator_CarriesFractionalTimeAndLimitsCatchUp());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_RejectsCommandsThatDoNotMatchFlightStateOrContainInvalidNumbers());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_FollowsThreeWaypointsAndClampsVelocityAndYaw());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_PauseResumeAndResetControlSimulationTime());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_DefaultsToAirSimAndUsesRotorForcesForTakeoffAndLanding());
#ifndef __EMSCRIPTEN__
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_WorldWindMovesRotatedHoveringDroneDownwind());
#endif
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_MultipleKinematicDronesKeepIndependentStateAndCommands());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_RigidBodyDronesShareOnePhysicsStepAndCanBeRemoved());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_CollidingRigidBodyDronesRemainSeparated());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_RigidBodyRemainsStableDuringOneMinuteHover());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_RigidBodyTakesOffFromTheSelectedTerrainStart());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_CameraStopsOnDisableOrFaultAndResumesAfterRecovery());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_EoCameraPoseTracksPrimaryDroneThroughTicksAndReset());
}

}
