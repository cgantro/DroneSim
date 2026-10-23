#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <gtest/gtest.h>

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

TEST(SimulationCoreTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_ExecutesTakeoffMovementAndLandingOnFixedTicks());
#ifndef __EMSCRIPTEN__
#endif
    EXPECT_NO_FATAL_FAILURE(Verify_SimulationCore_RejectsCommandsThatDoNotMatchFlightStateOrContainInvalidNumbers());
#ifndef __EMSCRIPTEN__
#endif
#ifndef __EMSCRIPTEN__
#endif
}

}
