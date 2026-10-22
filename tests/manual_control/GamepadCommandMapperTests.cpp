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

void Verify_GamepadCommandMapper_LimitsSpeedAndMapsHeightAndYaw()
{
    const auto command = MapAxes(
        GamepadAxes{.forward = 1.0, .right = 1.0, .vertical = 0.5, .yaw = -1.0}, 0.0);

    EXPECT_NEAR(std::hypot(command.velocityMetersPerSecond.x, command.velocityMetersPerSecond.y),
        3.0, 1.0e-12);
    EXPECT_NEAR(command.velocityMetersPerSecond.z, 0.4 * ApplyDeadZone(0.5)
        + 0.6 * std::pow(ApplyDeadZone(0.5), 3.0), 1.0e-12);
    EXPECT_NEAR(command.yawRateRadiansPerSecond, -0.7853981633974483, 1.0e-12);
}

void Verify_GamepadCommandMapper_RespectsControlSourceAndResolvesDisconnectAction()
{
    using Action = GamepadDisconnectAction;
    using Mode = simulation::FlightMode;

    EXPECT_TRUE(CanGamepadControlSimulation(false, false));
    EXPECT_FALSE(CanGamepadControlSimulation(true, false));
    EXPECT_FALSE(CanGamepadControlSimulation(false, true));
    EXPECT_EQ(ResolveGamepadDisconnectAction(false, false, Mode::Hovering), Action::NoChange);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, Mode::TakingOff), Action::Land);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, Mode::Flying), Action::Stop);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, Mode::Grounded), Action::ClearPendingAction);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, true, Mode::Flying), Action::ClearPendingAction);
}

TEST(GamepadCommandMapperTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_GamepadCommandMapper_AppliesDeadZoneAndRotatesMovement());
    EXPECT_NO_FATAL_FAILURE(Verify_GamepadCommandMapper_LimitsSpeedAndMapsHeightAndYaw());
    EXPECT_NO_FATAL_FAILURE(Verify_GamepadCommandMapper_RespectsControlSourceAndResolvesDisconnectAction());
}

}
