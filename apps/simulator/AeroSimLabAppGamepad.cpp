#include "AeroSimLabApp.h"

#include "graphics/Window.h"

#ifndef __EMSCRIPTEN__
#include <GLFW/glfw3.h>
#endif

#include <string>

namespace aerosimlab::simulator
{
void AeroSimLabApp::PollGamepad()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    GLFWgamepadstate state{};
    int connectedJoystickId = -1;
    int unsupportedJoystickId = -1;
    for (int joystickId = GLFW_JOYSTICK_1; joystickId <= GLFW_JOYSTICK_LAST; ++joystickId)
    {
        if (glfwJoystickPresent(joystickId) != GLFW_TRUE)
            continue;

        if (glfwJoystickIsGamepad(joystickId) == GLFW_TRUE
            && glfwGetGamepadState(joystickId, &state) == GLFW_TRUE)
        {
            connectedJoystickId = joystickId;
            break;
        }

        if (unsupportedJoystickId < 0)
            unsupportedJoystickId = joystickId;
    }

    if (connectedJoystickId < 0)
    {
        HandleGamepadDisconnected();
        gamepadJoystickId_ = -1;

        if (unsupportedJoystickId >= 0
            && unsupportedJoystickId_ != unsupportedJoystickId)
        {
            const char* deviceName = glfwGetJoystickName(unsupportedJoystickId);
            const std::string name = deviceName != nullptr ? deviceName : "이름 없음";
            window_->SetTitle("AeroSimLab | 게임패드 형식 미지원: " + name);
            unsupportedJoystickId_ = unsupportedJoystickId;
        }
        else if (unsupportedJoystickId < 0 && unsupportedJoystickId_ >= 0)
        {
            window_->SetTitle(groundControlActive_
                ? "AeroSimLab | 지상통제 입력 사용 중"
                : "AeroSimLab | 드론 관찰 · EO 영상");
            unsupportedJoystickId_ = -1;
        }
        return;
    }

    unsupportedJoystickId_ = -1;

    if (gamepadJoystickId_ != connectedJoystickId)
    {
        previousTakeoffButton_ = false;
        previousLandButton_ = false;
        pendingPilotCommand_.reset();
        gamepadJoystickId_ = connectedJoystickId;
    }

    const bool takeoffPressed = state.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS;
    const bool landPressed = state.buttons[GLFW_GAMEPAD_BUTTON_B] == GLFW_PRESS;
    std::optional<aerosimlab::manual_control::GamepadAxes> pilotAxes;
    if (!groundControlActive_)
    {
        const auto mode = simulation_.State().mode;
        const bool airborne = mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying;
        if (airborne)
        {
            // GLFW 트리거 값은 -1부터 시작하므로 0~1로 바꾼 뒤 양쪽 입력 차이를 높이 명령으로 쓴다.
            const auto triggerValue = [](float value)
            {
                return (static_cast<double>(value) + 1.0) * 0.5;
            };
            pilotAxes = aerosimlab::manual_control::GamepadAxes{
                .forward = -static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]),
                .right = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_X]),
                .vertical = triggerValue(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER])
                    - triggerValue(state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]),
                .yaw = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X])
            };
        }
    }
    HandleGamepadConnected(takeoffPressed, landPressed, pilotAxes);
#endif
}

void AeroSimLabApp::HandleGamepadDisconnected()
{
    using DisconnectAction = aerosimlab::manual_control::GamepadDisconnectAction;
    const auto action = aerosimlab::manual_control::ResolveGamepadDisconnectAction(
        gamepadConnected_, groundControlActive_ || commandRecording_.IsReplaying(), simulation_.State().mode);
    if (action == DisconnectAction::NoChange)
        return;

    pendingControllerAction_.reset();
    window_->SetTitle(groundControlActive_
        ? "AeroSimLab | 지상통제 입력 사용 중"
        : "AeroSimLab | 드론 관찰 · EO 영상");
    switch (action)
    {
    case DisconnectAction::Land:
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
            aerosimlab::simulation::DroneCommand::Kind::Land};
        break;
    case DisconnectAction::Stop:
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
            aerosimlab::simulation::DroneCommand::Kind::Stop};
        break;
    case DisconnectAction::NoChange:
    case DisconnectAction::ClearPendingAction:
        break;
    }
    if (!groundControlActive_)
        pendingPilotCommand_.reset();
    gamepadConnected_ = false;
    previousTakeoffButton_ = false;
    previousLandButton_ = false;
}

void AeroSimLabApp::HandleGamepadConnected(bool takeoffPressed, bool landPressed,
    const std::optional<aerosimlab::manual_control::GamepadAxes>& pilotAxes)
{
    if (!gamepadConnected_ && !groundControlActive_)
        window_->SetTitle("AeroSimLab | 게임패드 연결됨");
    gamepadConnected_ = true;
    externalControl_ = true;

    if (aerosimlab::manual_control::CanGamepadControlSimulation(
            groundControlActive_, commandRecording_.IsReplaying()))
    {
        if (takeoffPressed && !previousTakeoffButton_
            && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded)
        {
            pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::TakeOff};
        }
        if (landPressed && !previousLandButton_
            && simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
        {
            pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::Land};
        }

        if (pilotAxes)
            pendingPilotCommand_ = aerosimlab::manual_control::MapAxes(
                *pilotAxes, simulation_.State().yawRadians);
    }
    else if (commandRecording_.IsReplaying())
    {
        pendingControllerAction_.reset();
        pendingPilotCommand_.reset();
    }

    previousTakeoffButton_ = takeoffPressed;
    previousLandButton_ = landPressed;
}
}
