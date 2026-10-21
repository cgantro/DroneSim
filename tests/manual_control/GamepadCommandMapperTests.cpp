#include "manual_control/GamepadCommandMapper.h"

#include <gtest/gtest.h>

#include <cmath>

namespace aerosimlab::manual_control
{
TEST(GamepadCommandMapper, AppliesDeadZoneAndRotatesMovement)
{
    EXPECT_DOUBLE_EQ(ApplyDeadZone(0.08), 0.0);
    EXPECT_DOUBLE_EQ(ApplyDeadZone(-0.08), 0.0);
    EXPECT_DOUBLE_EQ(ApplyDeadZone(1.0), 1.0);
    EXPECT_DOUBLE_EQ(ApplyDeadZone(-1.0), -1.0);
    const auto command = MapAxes(GamepadAxes{.forward = 1.0}, 0.5 * std::acos(-1.0));

    EXPECT_NEAR(command.velocityMetersPerSecond.x, 0.0, 1.0e-12);
    EXPECT_NEAR(command.velocityMetersPerSecond.y, simulation::DroneModel::maximumSpeedMetersPerSecond, 1.0e-12);
}

TEST(GamepadCommandMapper, LimitsSpeedAndMapsHeightAndYaw)
{
    const auto command = MapAxes(
        GamepadAxes{.forward = 1.0, .right = 1.0, .vertical = 0.5, .yaw = -1.0}, 0.0);

    EXPECT_NEAR(std::hypot(command.velocityMetersPerSecond.x, command.velocityMetersPerSecond.y),
        simulation::DroneModel::maximumSpeedMetersPerSecond, 1.0e-12);
    EXPECT_NEAR(command.velocityMetersPerSecond.z,
        ApplyDeadZone(0.5) * simulation::DroneModel::maximumSpeedMetersPerSecond, 1.0e-12);
    EXPECT_DOUBLE_EQ(command.yawRateRadiansPerSecond,
        -simulation::DroneModel::maximumYawRateRadiansPerSecond);
}

TEST(GamepadCommandMapper, ResolvesDisconnectActionFromConnectionAndFlightState)
{
    using Action = GamepadDisconnectAction;
    using Mode = simulation::FlightMode;

    EXPECT_EQ(ResolveGamepadDisconnectAction(false, false, Mode::Hovering), Action::NoChange);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, Mode::TakingOff), Action::Land);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, Mode::Flying), Action::Stop);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, false, Mode::Grounded), Action::ClearPendingAction);
    EXPECT_EQ(ResolveGamepadDisconnectAction(true, true, Mode::Flying), Action::ClearPendingAction);
}
}
