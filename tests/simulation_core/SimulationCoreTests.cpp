#include "simulation_core/SimulationCore.h"
#include "simulation_core/BatteryModel.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

#include <glm/geometric.hpp>

namespace dronesim::simulation
{
namespace
{
// 고정 Tick마다 비행 상태와 카메라 프레임 번호가 함께 진행되는지 확인한다.
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

// 긴 호버 동안 위치·속도·자세가 발산하지 않는지 확인한다.
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

// 경유점 명령이 실제 위치를 바꾸고 도착 후 호버 상태로 끝나는지 확인한다.
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

// 드론을 지우면 연결된 강체도 정리되고 나머지 드론은 계속 진행하는지 확인한다.
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

// 배터리 사용량이 물리 Tick 이후의 비행 상태를 기준으로 계산되는지 확인한다.
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

// 네 대의 드론이 서로 독립적으로 이륙·이동·착륙하는지 확인한다.
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
