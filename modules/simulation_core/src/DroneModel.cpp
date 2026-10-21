#include "simulation_core/DroneModel.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>

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
        state.velocityMetersPerSecond = ClampLength(command.vector, maximumSpeedMetersPerSecond);
        state.mode = glm::length(state.velocityMetersPerSecond) == 0.0
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

    case DroneCommand::Kind::Stop:
        if (!IsAirborne(state))
        {
            return CommandResult::InvalidFlightMode;
        }
        state.velocityMetersPerSecond = glm::dvec3{0.0};
        state.mode = FlightMode::Hovering;
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;
    }

    return CommandResult::InvalidValue;
}

void DroneModel::AdvanceOneTick(DroneState& state, DroneControlState& control) noexcept
{
    constexpr double step = fixedStepSeconds;
    state.yawRadians += control.commandedYawRateRadiansPerSecond * step;
    state.orientation = glm::angleAxis(state.yawRadians, glm::dvec3{0.0, 0.0, 1.0});
    state.angularVelocityRadiansPerSecond = {
        0.0, 0.0, control.commandedYawRateRadiansPerSecond};

    switch (state.mode)
    {
    case FlightMode::Grounded:
    case FlightMode::Hovering:
        state.velocityMetersPerSecond = glm::dvec3{0.0};
        break;

    case FlightMode::TakingOff:
        if (takeoffHeightMeters - state.positionMeters.z
            <= climbSpeedMetersPerSecond * step + altitudeToleranceMeters)
        {
            state.positionMeters.z = takeoffHeightMeters;
            state.velocityMetersPerSecond = glm::dvec3{0.0};
            state.mode = FlightMode::Hovering;
        }
        else
        {
            state.positionMeters.z += climbSpeedMetersPerSecond * step;
        }
        break;

    case FlightMode::Flying:
        state.positionMeters += state.velocityMetersPerSecond * step;
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
        if (state.positionMeters.z <= climbSpeedMetersPerSecond * step + altitudeToleranceMeters)
        {
            state.positionMeters.z = 0.0;
            state.velocityMetersPerSecond = glm::dvec3{0.0};
            state.mode = FlightMode::Grounded;
        }
        else
        {
            state.positionMeters.z -= climbSpeedMetersPerSecond * step;
        }
        break;
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
