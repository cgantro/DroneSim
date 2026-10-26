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

glm::dvec3 ApproachVelocity(
    const glm::dvec3& current, const glm::dvec3& target, double maximumChange) noexcept
{
    return current + ClampLength(target - current, maximumChange);
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
        state.velocityMetersPerSecond = {0.0, 0.0, climbSpeedMetersPerSecond};
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;

    case DroneCommand::Kind::Land:
        if (!IsAirborne(state) && state.mode != FlightMode::TakingOff)
        {
            return CommandResult::InvalidFlightMode;
        }
        state.mode = FlightMode::Landing;
        state.velocityMetersPerSecond = {0.0, 0.0, -climbSpeedMetersPerSecond};
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
        if (!IsAirborne(state))
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!IsFinite(command.vector) || command.vector.z < 0.0)
        {
            return CommandResult::InvalidValue;
        }
        control.waypointMeters = command.vector;
        state.mode = FlightMode::FollowingWaypoint;
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;

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

void DroneModel::AdvanceOneTick(DroneState& state, DroneControlState& control) noexcept
{
    constexpr double step = fixedStepSeconds;
    const double groundElevation = control.groundElevationMeters;
    const double takeoffTarget = groundElevation + takeoffHeightMeters;
    state.yawRadians += control.commandedYawRateRadiansPerSecond * step;
    state.orientation = glm::angleAxis(state.yawRadians, glm::dvec3{0.0, 0.0, 1.0});
    state.angularVelocityRadiansPerSecond = {
        0.0, 0.0, control.commandedYawRateRadiansPerSecond};

    switch (state.mode)
    {
    case FlightMode::Grounded:
        state.velocityMetersPerSecond = glm::dvec3{0.0};
        break;

    case FlightMode::Hovering:
        state.velocityMetersPerSecond = ApproachVelocity(
            state.velocityMetersPerSecond, glm::dvec3{0.0},
            maximumAccelerationMetersPerSecondSquared * step);
        state.positionMeters += state.velocityMetersPerSecond * step;
        break;

    case FlightMode::TakingOff:
        if (takeoffTarget - state.positionMeters.z
            <= climbSpeedMetersPerSecond * step + altitudeToleranceMeters)
        {
            state.positionMeters.z = takeoffTarget;
            state.velocityMetersPerSecond = glm::dvec3{0.0};
            state.mode = FlightMode::Hovering;
        }
        else
        {
            state.positionMeters.z += climbSpeedMetersPerSecond * step;
        }
        break;

    case FlightMode::Flying:
        state.velocityMetersPerSecond = ApproachVelocity(
            state.velocityMetersPerSecond, control.targetVelocityMetersPerSecond,
            maximumAccelerationMetersPerSecondSquared * step);
        state.positionMeters += state.velocityMetersPerSecond * step;
        if (glm::length(state.velocityMetersPerSecond) < 0.05
            && glm::length(control.targetVelocityMetersPerSecond) < 0.05)
        {
            state.velocityMetersPerSecond = glm::dvec3{0.0};
            state.mode = FlightMode::Hovering;
        }
        break;

    case FlightMode::FollowingWaypoint:
    {
        const glm::dvec3 offset = control.waypointMeters - state.positionMeters;
        const double distance = glm::length(offset);
        const double stepDistance = maximumSpeedMetersPerSecond * step;
        if (distance <= std::max(waypointToleranceMeters, stepDistance))
        {
            state.positionMeters = control.waypointMeters;
            state.velocityMetersPerSecond = glm::dvec3{0.0};
            state.mode = FlightMode::Hovering;
            break;
        }

        state.velocityMetersPerSecond = offset * (maximumSpeedMetersPerSecond / distance);
        state.positionMeters += state.velocityMetersPerSecond * step;
        break;
    }

    case FlightMode::Landing:
        if (state.positionMeters.z <= groundElevation
            + climbSpeedMetersPerSecond * step + altitudeToleranceMeters)
        {
            state.positionMeters.z = groundElevation;
            state.velocityMetersPerSecond = glm::dvec3{0.0};
            state.mode = FlightMode::Grounded;
        }
        else
        {
            state.positionMeters.z -= climbSpeedMetersPerSecond * step;
        }
        break;
    }

    if (IsAirborne(state) && state.positionMeters.z <= groundElevation)
    {
        state.positionMeters.z = groundElevation;
        state.velocityMetersPerSecond = glm::dvec3{0.0};
        state.angularVelocityRadiansPerSecond = glm::dvec3{0.0};
        state.mode = FlightMode::Grounded;
        control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
        control.commandedYawRateRadiansPerSecond = 0.0;
    }
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
