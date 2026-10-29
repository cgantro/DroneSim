#include "simulation_core/SimulationCore.h"
#include "simulation_core/BatteryModel.h"

#include <gtest/gtest.h>

#include <stdexcept>

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

TEST(SimulationCoreTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(VerifyTakeoffAndCameraAdvanceTogether());
    EXPECT_NO_FATAL_FAILURE(VerifyWaypointCommandMovesTheRigidBody());
    EXPECT_NO_FATAL_FAILURE(VerifyDroneAndPhysicsBodyAreRemovedTogether());
    EXPECT_NO_FATAL_FAILURE(VerifyBatteryUsesThePostPhysicsFlightState());
}
}
}
