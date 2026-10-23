#include "manual_control/GamepadCommandMapper.h"

#include <gtest/gtest.h>

#include <cmath>

namespace aerosimlab::manual_control
{
void Verify_GamepadCommandMapper_AppliesDeadZoneAndRotatesMovement()
{
    EXPECT_DOUBLE_EQ(ApplyDeadZone(0.08), 0.0);
    EXPECT_DOUBLE_EQ(ApplyDeadZone(-0.08), 0.0);
    EXPECT_DOUBLE_EQ(ApplyDeadZone(1.0), 1.0);
    EXPECT_DOUBLE_EQ(ApplyDeadZone(-1.0), -1.0);
    const auto command = MapAxes(GamepadAxes{.forward = 1.0}, 0.5 * std::acos(-1.0));

    EXPECT_NEAR(command.velocityMetersPerSecond.x, 0.0, 1.0e-12);
    EXPECT_NEAR(command.velocityMetersPerSecond.y, 3.0, 1.0e-12);
}

void Verify_GamepadDisconnect_ChoosesActionFromConnectionAndFlightState()
{
    using Action = GamepadDisconnectAction;
    using FlightMode = simulation::FlightMode;

    EXPECT_EQ(ResolveGamepadDisconnectAction(false, false, FlightMode::Flying),
        Action::NoChange);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, FlightMode::TakingOff),
        Action::Land);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, FlightMode::Hovering),
        Action::Stop);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, FlightMode::Flying),
        Action::Stop);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, FlightMode::Grounded),
        Action::ClearPendingAction);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, FlightMode::Landing),
        Action::ClearPendingAction);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, true, FlightMode::Flying),
        Action::ClearPendingAction);
}

TEST(GamepadCommandMapperTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_GamepadCommandMapper_AppliesDeadZoneAndRotatesMovement());
    EXPECT_NO_FATAL_FAILURE(Verify_GamepadDisconnect_ChoosesActionFromConnectionAndFlightState());
}

}
