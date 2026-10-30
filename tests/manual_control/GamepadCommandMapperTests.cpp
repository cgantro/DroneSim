#include "manual_control/GamepadCommandMapper.h"

#include <gtest/gtest.h>

#include <cmath>

namespace dronesim::manual_control
{
// 스틱 중앙의 작은 흔들림이 사라지고, 기체 방향에 맞춰 이동축이 회전하는지 확인한다.
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

// 연결 해제 시점과 비행 상태에 따라 정지·착륙·입력 취소를 고르는지 확인한다.
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
