#include "manual_control/GamepadController.h"

namespace dronesim::manual_control
{
/** @brief 브라우저 빌드에서는 GLFW 조이스틱 입력을 읽지 않아 빈 갱신을 돌려준다. */
GamepadController::Update GamepadController::Poll(bool missionOwnsInput,
    simulation::FlightMode flightMode, double yawRadians)
{
    static_cast<void>(missionOwnsInput);
    static_cast<void>(flightMode);
    static_cast<void>(yawRadians);
    return {};
}
}
