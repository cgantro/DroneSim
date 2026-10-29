#include "simulation_core/DroneModel.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>

namespace aerosimlab::simulation
{
namespace
{
bool IsFinite(const glm::dvec3& value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

glm::dvec3 ClampLength(const glm::dvec3& value, double maximumLength) noexcept
{
    const double length = std::hypot(value.x, value.y, value.z);
    if (length <= maximumLength || length == 0.0)
    {
        return value;
    }
    return value * (maximumLength / length);
}

struct FlightModeName
{
    std::string_view name;
    FlightMode mode;
};

constexpr std::array flightModeNames{
    FlightModeName{"grounded", FlightMode::Grounded},
    FlightModeName{"taking_off", FlightMode::TakingOff},
    FlightModeName{"hovering", FlightMode::Hovering},
    FlightModeName{"flying", FlightMode::Flying},
    FlightModeName{"following_waypoint", FlightMode::FollowingWaypoint},
    FlightModeName{"landing", FlightMode::Landing}
};
}

std::string_view ToString(FlightMode mode) noexcept
{
    for (const FlightModeName& entry : flightModeNames)
    {
        if (entry.mode == mode)
            return entry.name;
    }
    return "unknown";
}

std::optional<FlightMode> TryParseFlightMode(std::string_view name) noexcept
{
    for (const FlightModeName& entry : flightModeNames)
    {
        if (entry.name == name)
            return entry.mode;
    }
    return std::nullopt;
}

CommandResult DroneModel::Apply(
    DroneState& state, DroneControlState& control, const DroneCommand& command) noexcept
{
    switch (command.kind)
    {
    case DroneCommand::Kind::TakeOff:
        if (state.mode != FlightMode::Grounded)
        {
            return CommandResult::InvalidFlightMode;
        }
        state.mode = FlightMode::TakingOff;
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;

    case DroneCommand::Kind::Land:
        if (!IsAirborne(state) && state.mode != FlightMode::TakingOff)
        {
            return CommandResult::InvalidFlightMode;
        }
        state.mode = FlightMode::Landing;
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetVelocity:
        if (!IsAirborne(state))
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!IsFinite(command.vector))
        {
            return CommandResult::InvalidValue;
        }
        control.targetVelocityMetersPerSecond = ClampLength(
            command.vector, maximumSpeedMetersPerSecond);
        state.mode = glm::length(control.targetVelocityMetersPerSecond) == 0.0
                && glm::length(state.velocityMetersPerSecond) < 0.05
            ? FlightMode::Hovering
            : FlightMode::Flying;
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetWaypoint:
    {
        if (!IsAirborne(state))
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!IsFinite(command.vector) || command.vector.z < 0.0)
        {
            return CommandResult::InvalidValue;
        }
        if (command.targetYawRadians && !std::isfinite(*command.targetYawRadians))
        {
            return CommandResult::InvalidValue;
        }
        control.waypointMeters = command.vector;
        const double offsetX = command.vector.x - state.positionMeters.x;
        const double offsetY = command.vector.y - state.positionMeters.y;
        if (command.targetYawRadians)
        {
            control.desiredYawRadians = *command.targetYawRadians;
        }
        else if (std::hypot(offsetX, offsetY) > waypointToleranceMeters)
        {
            control.desiredYawRadians = std::atan2(offsetY, offsetX);
        }
        state.mode = FlightMode::FollowingWaypoint;
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;
    }

    case DroneCommand::Kind::SetYawRate:
        if (!IsAirborne(state))
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!std::isfinite(command.yawRateRadiansPerSecond))
        {
            return CommandResult::InvalidValue;
        }
        control.commandedYawRateRadiansPerSecond = std::clamp(
            command.yawRateRadiansPerSecond,
            -maximumYawRateRadiansPerSecond,
            maximumYawRateRadiansPerSecond);
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetRotorThrusts:
        if (!std::all_of(command.rotorThrustRatios.begin(), command.rotorThrustRatios.end(),
            [](double ratio) { return std::isfinite(ratio) && ratio >= 0.0 && ratio <= 1.0; }))
        {
            return CommandResult::InvalidValue;
        }
        control.commandedRotorThrustRatios = command.rotorThrustRatios;
        return CommandResult::Accepted;

    case DroneCommand::Kind::Stop:
        if (!IsAirborne(state))
        {
            return CommandResult::InvalidFlightMode;
        }
        control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
        state.mode = glm::length(state.velocityMetersPerSecond) < 0.05
            ? FlightMode::Hovering
            : FlightMode::Flying;
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;
    }

    return CommandResult::InvalidValue;
}

double DroneModel::YawRadians(const DroneState& state) noexcept
{
    const glm::dquat& orientation = state.orientation;
    return std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

void DroneModel::Reset(DroneState& state, DroneControlState& control) noexcept
{
    state = {};
    control = {};
}

bool DroneModel::IsAirborne(const DroneState& state) noexcept
{
    return state.mode == FlightMode::Hovering
        || state.mode == FlightMode::Flying
        || state.mode == FlightMode::FollowingWaypoint;
}
}
