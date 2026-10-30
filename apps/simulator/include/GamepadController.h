#pragma once

#include "manual_control/GamepadCommandMapper.h"

#include <optional>
#include <string>

namespace dronesim::simulator
{
/** @brief 게임패드 장치 상태와 입력 가장자리를 앱 실행 흐름에서 분리해 소유한다. */
class GamepadController final
{
public:
    struct Update
    {
        bool connected = false;
        bool connectionChanged = false;
        bool externalInputReceived = false;
        std::optional<simulation::DroneCommand> flightAction;
        std::optional<manual_control::PilotCommand> pilotCommand;
        std::optional<std::string> windowTitle;
    };

    /** @brief 현재 기체 상태와 임무 입력 잠금 여부에 맞춰 새 조종 입력을 읽는다. */
    [[nodiscard]] Update Poll(bool missionOwnsInput,
        simulation::FlightMode flightMode, double yawRadians);

    [[nodiscard]] bool IsConnected() const noexcept { return connected_; }

private:
    bool connected_ = false;
    bool takeoffButtonWasPressed_ = false;
    bool landButtonWasPressed_ = false;
    int joystickId_ = -1;
    int unsupportedJoystickId_ = -1;
};
}
