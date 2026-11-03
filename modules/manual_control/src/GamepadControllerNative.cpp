#include "manual_control/GamepadController.h"

#include <GLFW/glfw3.h>

namespace dronesim::manual_control
{
namespace
{
struct GamepadScan
{
    GLFWgamepadstate state{};
    int supportedJoystickId = -1;
    int unsupportedJoystickId = -1;
};

GamepadScan FindGamepad()
{
    GamepadScan scan;
    for (int id = GLFW_JOYSTICK_1; id <= GLFW_JOYSTICK_LAST; ++id)
    {
        if (glfwJoystickPresent(id) != GLFW_TRUE)
            continue;

        if (glfwJoystickIsGamepad(id) == GLFW_TRUE
            && glfwGetGamepadState(id, &scan.state) == GLFW_TRUE)
        {
            scan.supportedJoystickId = id;
            break;
        }
        if (scan.unsupportedJoystickId < 0)
            scan.unsupportedJoystickId = id;
    }
    return scan;
}

GamepadAxes ReadAxes(const GLFWgamepadstate& state)
{
    // GLFW 트리거는 -1부터 시작하므로 0~1로 바꾼 뒤 양쪽 입력 차이를 고도 명령으로 쓴다.
    const auto trigger = [](float value)
    {
        return (static_cast<double>(value) + 1.0) * 0.5;
    };
    return {
        .forward = -static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]),
        .right = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_X]),
        .vertical = trigger(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER])
            - trigger(state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]),
        .yaw = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X])};
}
}

GamepadController::Update GamepadController::Poll(bool missionOwnsInput,
    simulation::FlightMode flightMode, double yawRadians)
{
    Update update;
    const GamepadScan scan = FindGamepad();
    if (scan.supportedJoystickId < 0)
    {
        update.connectionChanged = connected_;
        if (connected_)
        {
            const auto action = ResolveGamepadDisconnectAction(
                true, missionOwnsInput, flightMode);
            if (action == GamepadDisconnectAction::Land)
                update.flightAction = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::Land};
            else if (action == GamepadDisconnectAction::Stop)
                update.flightAction = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::Stop};
        }
        connected_ = false;
        joystickId_ = -1;
        takeoffButtonWasPressed_ = false;
        landButtonWasPressed_ = false;

        if (scan.unsupportedJoystickId >= 0
            && unsupportedJoystickId_ != scan.unsupportedJoystickId)
        {
            const char* name = glfwGetJoystickName(scan.unsupportedJoystickId);
            update.notice = GamepadNotice::UnsupportedDevice;
            update.deviceName = name != nullptr ? name : "이름 없음";
            unsupportedJoystickId_ = scan.unsupportedJoystickId;
        }
        else if (scan.unsupportedJoystickId < 0 && unsupportedJoystickId_ >= 0)
        {
            update.notice = GamepadNotice::UnsupportedDeviceRemoved;
            unsupportedJoystickId_ = -1;
        }
        return update;
    }

    unsupportedJoystickId_ = -1;
    if (joystickId_ != scan.supportedJoystickId)
    {
        joystickId_ = scan.supportedJoystickId;
        takeoffButtonWasPressed_ = false;
        landButtonWasPressed_ = false;
        update.connectionChanged = !connected_;
    }

    connected_ = true;
    update.connected = true;
    update.externalInputReceived = true;
    const bool takeoffPressed = scan.state.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS;
    const bool landPressed = scan.state.buttons[GLFW_GAMEPAD_BUTTON_B] == GLFW_PRESS;
    const bool airborne = flightMode == simulation::FlightMode::Hovering
        || flightMode == simulation::FlightMode::Flying;

    if (!missionOwnsInput)
    {
        if (takeoffPressed && !takeoffButtonWasPressed_
            && flightMode == simulation::FlightMode::Grounded)
            update.flightAction = simulation::DroneCommand{
                simulation::DroneCommand::Kind::TakeOff};
        if (landPressed && !landButtonWasPressed_ && flightMode != simulation::FlightMode::Grounded)
            update.flightAction = simulation::DroneCommand{
                simulation::DroneCommand::Kind::Land};
        if (airborne)
            update.pilotCommand = MapAxes(ReadAxes(scan.state), yawRadians);
    }

    if (update.connectionChanged)
        update.notice = GamepadNotice::Connected;
    takeoffButtonWasPressed_ = takeoffPressed;
    landButtonWasPressed_ = landPressed;
    return update;
}
}
