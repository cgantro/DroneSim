#include "simulation_core/SimulationCore.h"
#include "simulation_core/BatteryModel.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

#include <glm/geometric.hpp>

namespace aerosimlab::simulation
{
namespace
{
void VerifyTakeoffAndCameraAdvanceTogether()
{
    SimulationCore simulation;
    const DroneId drone = simulation.PrimaryDroneId();
    ASSERT_EQ(simulation.Apply({DroneCommand::Kind::TakeOff}), CommandResult::Accepted);

    for (int tick = 0; tick < 360; ++tick)
        simulation.AdvanceOneTick();

    EXPECT_EQ(simulation.State(drone).mode, FlightMode::Hovering);
    EXPECT_GT(simulation.State(drone).positionMeters.z, 1.0);
    EXPECT_EQ(simulation.Camera(drone).status, CameraStatus::Active);
    EXPECT_EQ(simulation.Camera(drone).frameCount, 360U);
}

void VerifyLongHoverKeepsTheRigidBodyStable()
{
    SimulationCore simulation;
    const DroneId drone = simulation.PrimaryDroneId();
    ASSERT_EQ(simulation.Apply({DroneCommand::Kind::TakeOff}), CommandResult::Accepted);

    for (int tick = 0; tick < 360; ++tick)
        simulation.AdvanceOneTick();
    ASSERT_EQ(simulation.State(drone).mode, FlightMode::Hovering);

    for (int tick = 0; tick < 3600; ++tick)
        simulation.AdvanceOneTick();

    const DroneState& state = simulation.State(drone);
    EXPECT_EQ(state.mode, FlightMode::Hovering);
    EXPECT_NEAR(state.positionMeters.z, DroneModel::takeoffHeightMeters, 0.2);
    EXPECT_LT(glm::length(state.velocityMetersPerSecond), 0.2);
    EXPECT_LT(glm::length(state.angularVelocityRadiansPerSecond), 0.2);
}

void VerifyWaypointCommandMovesTheRigidBody()
{
    SimulationCore simulation;
    const DroneId drone = simulation.PrimaryDroneId();
    ASSERT_EQ(simulation.Apply({DroneCommand::Kind::TakeOff}), CommandResult::Accepted);
    for (int tick = 0; tick < 360; ++tick)
        simulation.AdvanceOneTick();

    DroneCommand waypoint;
    waypoint.kind = DroneCommand::Kind::SetWaypoint;
    waypoint.vector = {1.0, 0.0, DroneModel::takeoffHeightMeters};
    ASSERT_EQ(simulation.Apply(drone, waypoint), CommandResult::Accepted);
    for (int tick = 0; tick < 600; ++tick)
        simulation.AdvanceOneTick();

    EXPECT_NEAR(simulation.State(drone).positionMeters.x, 1.0, 0.25);
    EXPECT_EQ(simulation.State(drone).mode, FlightMode::Hovering);
}

void VerifyDroneAndPhysicsBodyAreRemovedTogether()
{
    SimulationCore simulation;
    const DroneId first = simulation.PrimaryDroneId();
    const DroneId second = simulation.AddDrone({4.0, 0.0, 5.0});
    const DroneId third = simulation.AddDrone({8.0, 0.0, 5.0});

    ASSERT_EQ(simulation.DroneCount(), 3U);
    ASSERT_TRUE(simulation.RemoveDrone(first));
    EXPECT_EQ(simulation.PrimaryDroneId(), second);
    ASSERT_TRUE(simulation.RemoveDrone(third));
    EXPECT_FALSE(simulation.RemoveDrone(second));
    EXPECT_THROW(static_cast<void>(simulation.State(first)), std::out_of_range);
    EXPECT_EQ(simulation.DroneCount(), 1U);
    EXPECT_NO_THROW(simulation.AdvanceOneTick());
}

void VerifyBatteryUsesThePostPhysicsFlightState()
{
    SimulationCore simulation;
    const DroneId drone = simulation.PrimaryDroneId();
    simulation.ConfigureBattery(drone, {10.0, 40.0, 120.0, 20.0}, 50.0);
    const double initialEnergy = simulation.Battery(drone).RemainingWattHours();

    ASSERT_EQ(simulation.Apply({DroneCommand::Kind::TakeOff}), CommandResult::Accepted);
    for (int tick = 0; tick < 360; ++tick)
        simulation.AdvanceOneTick();

    EXPECT_LT(simulation.Battery(drone).RemainingWattHours(), initialEnergy);
    EXPECT_EQ(simulation.State(drone).mode, FlightMode::Hovering);
}

void VerifyFourDronesTakeOffMoveAndLandIndependently()
{
    SimulationCore simulation;
    const std::array<DroneId, 4> drones{
        simulation.PrimaryDroneId(),
        simulation.AddDrone({8.0, 0.0, 0.24}),
        simulation.AddDrone({0.0, 8.0, 0.24}),
        simulation.AddDrone({8.0, 8.0, 0.24})};

    for (const DroneId drone : drones)
        ASSERT_EQ(simulation.Apply(drone, {DroneCommand::Kind::TakeOff}),
            CommandResult::Accepted);

    for (int tick = 0; tick < 360; ++tick)
        simulation.AdvanceOneTick();
    for (const DroneId drone : drones)
    {
        EXPECT_EQ(simulation.State(drone).mode, FlightMode::Hovering);
        EXPECT_GT(simulation.State(drone).positionMeters.z, 1.0);
    }

    for (std::size_t index = 0; index < drones.size(); ++index)
    {
        DroneCommand waypoint;
        waypoint.kind = DroneCommand::Kind::SetWaypoint;
        waypoint.vector = simulation.State(drones[index]).positionMeters
            + glm::dvec3{1.0, 0.0, 0.0};
        ASSERT_EQ(simulation.Apply(drones[index], waypoint), CommandResult::Accepted);
    }
    for (int tick = 0; tick < 600; ++tick)
        simulation.AdvanceOneTick();
    for (const DroneId drone : drones)
        EXPECT_EQ(simulation.State(drone).mode, FlightMode::Hovering);

    for (const DroneId drone : drones)
        ASSERT_EQ(simulation.Apply(drone, {DroneCommand::Kind::Land}),
            CommandResult::Accepted);
    for (int tick = 0; tick < 360; ++tick)
        simulation.AdvanceOneTick();
    for (const DroneId drone : drones)
    {
        const DroneState& state = simulation.State(drone);
        EXPECT_EQ(state.mode, FlightMode::Grounded);
        EXPECT_NEAR(state.positionMeters.z,
            QuadrotorFlightController::groundedModelOriginHeightMeters, 0.08);
    }
}

TEST(SimulationCoreTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(VerifyTakeoffAndCameraAdvanceTogether());
    EXPECT_NO_FATAL_FAILURE(VerifyLongHoverKeepsTheRigidBodyStable());
    EXPECT_NO_FATAL_FAILURE(VerifyWaypointCommandMovesTheRigidBody());
    EXPECT_NO_FATAL_FAILURE(VerifyDroneAndPhysicsBodyAreRemovedTogether());
    EXPECT_NO_FATAL_FAILURE(VerifyBatteryUsesThePostPhysicsFlightState());
    EXPECT_NO_FATAL_FAILURE(VerifyFourDronesTakeOffMoveAndLandIndependently());
}
}
}
