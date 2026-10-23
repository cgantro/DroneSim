#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <gtest/gtest.h>

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

void Verify_SimulationCore_AddsAndRemovesEcsAndPhysicsDronesTogether()
{
    const auto verifyLifecycle = [](SimulationDynamics dynamics)
    {
        SimulationCore simulation{dynamics};
        const DroneId first = simulation.PrimaryDroneId();
        const DroneId second = simulation.AddDrone({4.0, 0.0, 5.0}, 0.2);
        const DroneId third = simulation.AddDrone({8.0, 0.0, 5.0}, -0.2);

        ASSERT_EQ(simulation.DroneCount(), 3U);
        EXPECT_TRUE(simulation.RemoveDrone(first));
        EXPECT_EQ(simulation.PrimaryDroneId(), second);
        EXPECT_EQ(simulation.DroneCount(), 2U);
        EXPECT_FALSE(simulation.RemoveDrone(first));
        EXPECT_THROW(static_cast<void>(simulation.State(first)), std::out_of_range);

        EXPECT_TRUE(simulation.RemoveDrone(third));
        EXPECT_FALSE(simulation.RemoveDrone(second));
        EXPECT_EQ(simulation.DroneCount(), 1U);
        simulation.AdvanceOneTick();
        EXPECT_NO_THROW(static_cast<void>(simulation.State(second)));
    };

    verifyLifecycle(SimulationDynamics::Kinematic);
#ifndef __EMSCRIPTEN__
    verifyLifecycle(SimulationDynamics::RigidBody);
#endif
}

TEST(SimulationCoreTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_ExecutesTakeoffMovementAndLandingOnFixedTicks());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_RejectsCommandsThatDoNotMatchFlightStateOrContainInvalidNumbers());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_AddsAndRemovesEcsAndPhysicsDronesTogether());
}

}
