#include "manual_control/GamepadCommandMapper.h"

#include <algorithm>
#include <cmath>

namespace aerosimlab::manual_control
{
namespace
{
constexpr double stickDeadZone = 0.12;
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
    const double forward = ApplyDeadZone(axes.forward);
    const double right = ApplyDeadZone(axes.right);
    const double vertical = ApplyDeadZone(axes.vertical);
    const double yaw = ApplyDeadZone(axes.yaw);

    const double horizontalLength = std::hypot(forward, right);
    const double horizontalScale = horizontalLength > 1.0
        ? 1.0 / horizontalLength
        : 1.0;
    const double localForward = forward * horizontalScale * simulation::DroneModel::maximumSpeedMetersPerSecond;
    const double localRight = right * horizontalScale * simulation::DroneModel::maximumSpeedMetersPerSecond;
    const double cosine = std::cos(droneYawRadians);
    const double sine = std::sin(droneYawRadians);

    PilotCommand command;
    command.velocityMetersPerSecond = {
        localForward * cosine - localRight * sine,
        localForward * sine + localRight * cosine,
        vertical * simulation::DroneModel::maximumSpeedMetersPerSecond
    };
    command.yawRateRadiansPerSecond = yaw * simulation::DroneModel::maximumYawRateRadiansPerSecond;
    return command;
}
}
