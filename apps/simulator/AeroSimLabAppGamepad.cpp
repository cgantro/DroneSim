#include "AeroSimLabApp.h"

#include "graphics/Window.h"

#ifndef __EMSCRIPTEN__
#include <GLFW/glfw3.h>
#endif

#include <string>

namespace aerosimlab::simulator
{
#ifndef __EMSCRIPTEN__
namespace
{
struct GamepadScanResult
{
    GLFWgamepadstate state{};
    int connectedJoystickId = -1;
    int unsupportedJoystickId = -1;
};

/**
 * @brief 연결된 조이스틱을 훑어 GLFW가 읽을 수 있는 게임패드를 찾는다.
 * @details 게임패드가 없을 때 창에 안내할 수 있도록, 처음 발견한 미지원 장치도 함께 기억한다.
 */
GamepadScanResult ScanForGamepad()
{
    GamepadScanResult result;
    for (int joystickId = GLFW_JOYSTICK_1; joystickId <= GLFW_JOYSTICK_LAST; ++joystickId)
    {
        if (glfwJoystickPresent(joystickId) != GLFW_TRUE)
            continue;

        if (glfwJoystickIsGamepad(joystickId) == GLFW_TRUE
            && glfwGetGamepadState(joystickId, &result.state) == GLFW_TRUE)
        {
            result.connectedJoystickId = joystickId;
            break;
        }

        if (result.unsupportedJoystickId < 0)
            result.unsupportedJoystickId = joystickId;
    }
    return result;
}
}
#endif

void AeroSimLabApp::PollGamepad()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    const GamepadScanResult scan = ScanForGamepad();

    if (scan.connectedJoystickId < 0)
    {
        HandleGamepadDisconnected();
        gamepadJoystickId_ = -1;

        if (scan.unsupportedJoystickId >= 0
            && unsupportedJoystickId_ != scan.unsupportedJoystickId)
        {
            const char* deviceName = glfwGetJoystickName(scan.unsupportedJoystickId);
            const std::string name = deviceName != nullptr ? deviceName : "이름 없음";
            window_->SetTitle("AeroSimLab | 게임패드 형식 미지원: " + name);
            unsupportedJoystickId_ = scan.unsupportedJoystickId;
        }
        else if (scan.unsupportedJoystickId < 0 && unsupportedJoystickId_ >= 0)
        {
            window_->SetTitle(groundControlActive_
                ? "AeroSimLab | 지상통제 입력 사용 중"
                : "AeroSimLab | 드론 관찰 · EO 영상");
            unsupportedJoystickId_ = -1;
        }
        return;
    }

    unsupportedJoystickId_ = -1;

    if (gamepadJoystickId_ != scan.connectedJoystickId)
    {
        previousTakeoffButton_ = false;
        previousLandButton_ = false;
        pendingPilotCommand_.reset();
        gamepadJoystickId_ = scan.connectedJoystickId;
    }

    const bool takeoffPressed = scan.state.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS;
    const bool landPressed = scan.state.buttons[GLFW_GAMEPAD_BUTTON_B] == GLFW_PRESS;
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
                .forward = -static_cast<double>(scan.state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]),
                .right = static_cast<double>(scan.state.axes[GLFW_GAMEPAD_AXIS_LEFT_X]),
                .vertical = triggerValue(scan.state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER])
                    - triggerValue(scan.state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]),
                .yaw = static_cast<double>(scan.state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X])
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
