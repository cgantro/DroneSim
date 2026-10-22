#include "manual_control/GamepadCommandMapper.h"

#include <algorithm>
#include <cmath>

namespace aerosimlab::manual_control
{
namespace
{
constexpr double stickDeadZone = 0.12;
constexpr double stickExpo = 0.6;
constexpr double maximumHorizontalSpeedMetersPerSecond = 3.0;
constexpr double maximumAscentSpeedMetersPerSecond = 1.0;
constexpr double maximumDescentSpeedMetersPerSecond = 1.5;
constexpr double maximumYawRateRadiansPerSecond = 0.7853981633974483;

// 스틱 가운데는 세밀하게, 끝까지 밀었을 때는 최대 명령에 닿도록 곡선을 완만하게 만든다.
double ShapeStick(double value) noexcept
{
    const double deadZoneValue = ApplyDeadZone(value);
    const double magnitude = std::abs(deadZoneValue);
    const double shapedMagnitude = (1.0 - stickExpo) * magnitude
        + stickExpo * magnitude * magnitude * magnitude;
    return std::copysign(shapedMagnitude, deadZoneValue);
}
}

double ApplyDeadZone(double value) noexcept
{
    const double clipped = std::clamp(value, -1.0, 1.0);
    const double magnitude = std::abs(clipped);
    if (magnitude <= stickDeadZone)
        return 0.0;

    const double scaledMagnitude = (magnitude - stickDeadZone) / (1.0 - stickDeadZone);
    return std::copysign(scaledMagnitude, clipped);
}

PilotCommand MapAxes(const GamepadAxes& axes, double droneYawRadians) noexcept
{
    const double forward = ShapeStick(axes.forward);
    const double right = ShapeStick(axes.right);
    const double vertical = ShapeStick(axes.vertical);
    const double yaw = ShapeStick(axes.yaw);

    const double horizontalLength = std::hypot(forward, right);
    const double horizontalScale = horizontalLength > 1.0
        ? 1.0 / horizontalLength
        : 1.0;
    const double localForward = forward * horizontalScale * maximumHorizontalSpeedMetersPerSecond;
    const double localRight = right * horizontalScale * maximumHorizontalSpeedMetersPerSecond;
    const double cosine = std::cos(droneYawRadians);
    const double sine = std::sin(droneYawRadians);

    PilotCommand command;
    command.velocityMetersPerSecond = {
        localForward * cosine - localRight * sine,
        localForward * sine + localRight * cosine,
        vertical * (vertical >= 0.0
                ? maximumAscentSpeedMetersPerSecond
                : maximumDescentSpeedMetersPerSecond)
    };
    command.yawRateRadiansPerSecond = yaw * maximumYawRateRadiansPerSecond;
    return command;
}

GamepadDisconnectAction ResolveGamepadDisconnectAction(
    bool wasConnected, bool manualControlBlocked, simulation::FlightMode flightMode) noexcept
{
    if (!wasConnected)
        return GamepadDisconnectAction::NoChange;
    if (manualControlBlocked)
        return GamepadDisconnectAction::ClearPendingAction;
    if (flightMode == simulation::FlightMode::TakingOff)
        return GamepadDisconnectAction::Land;
    if (flightMode == simulation::FlightMode::Hovering
        || flightMode == simulation::FlightMode::Flying)
        return GamepadDisconnectAction::Stop;
    return GamepadDisconnectAction::ClearPendingAction;
}

bool CanGamepadControlSimulation(bool groundControlActive, bool replaying) noexcept
{
    return !groundControlActive && !replaying;
}
}
