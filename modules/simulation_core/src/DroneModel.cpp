#include "simulation_core/DroneModel.h"

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

const DroneState& DroneModel::State() const noexcept
{
    return state_;
}

CommandResult DroneModel::Apply(const DroneCommand& command) noexcept
{
    switch (command.kind)
    {
    case DroneCommand::Kind::TakeOff:
        if (state_.mode != FlightMode::Grounded)
        {
            return CommandResult::InvalidFlightMode;
        }
        state_.mode = FlightMode::TakingOff;
        state_.velocityMetersPerSecond = {0.0, 0.0, climbSpeedMetersPerSecond};
        commandedYawRateRadiansPerSecond_ = 0.0;
        return CommandResult::Accepted;

    case DroneCommand::Kind::Land:
        if (!IsAirborne() && state_.mode != FlightMode::TakingOff)
        {
            return CommandResult::InvalidFlightMode;
        }
        state_.mode = FlightMode::Landing;
        state_.velocityMetersPerSecond = {0.0, 0.0, -climbSpeedMetersPerSecond};
        commandedYawRateRadiansPerSecond_ = 0.0;
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetVelocity:
        if (!IsAirborne())
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!IsFinite(command.vector))
        {
            return CommandResult::InvalidValue;
        }
        state_.velocityMetersPerSecond = ClampLength(command.vector, maximumSpeedMetersPerSecond);
        state_.mode = glm::length(state_.velocityMetersPerSecond) == 0.0
            ? FlightMode::Hovering
            : FlightMode::Flying;
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetWaypoint:
        if (!IsAirborne())
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!IsFinite(command.vector) || command.vector.z < 0.0)
        {
            return CommandResult::InvalidValue;
        }
        waypointMeters_ = command.vector;
        state_.mode = FlightMode::FollowingWaypoint;
        commandedYawRateRadiansPerSecond_ = 0.0;
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetYawRate:
        if (!IsAirborne())
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!std::isfinite(command.yawRateRadiansPerSecond))
        {
            return CommandResult::InvalidValue;
        }
        commandedYawRateRadiansPerSecond_ = std::clamp(
            command.yawRateRadiansPerSecond,
            -maximumYawRateRadiansPerSecond,
            maximumYawRateRadiansPerSecond);
        return CommandResult::Accepted;

    case DroneCommand::Kind::Stop:
        if (!IsAirborne())
        {
            return CommandResult::InvalidFlightMode;
        }
        state_.velocityMetersPerSecond = glm::dvec3{0.0};
        state_.mode = FlightMode::Hovering;
        commandedYawRateRadiansPerSecond_ = 0.0;
        return CommandResult::Accepted;
    }

    return CommandResult::InvalidValue;
}

void DroneModel::AdvanceOneTick() noexcept
{
    constexpr double step = fixedStepSeconds;
    state_.yawRadians += commandedYawRateRadiansPerSecond_ * step;

    switch (state_.mode)
    {
    case FlightMode::Grounded:
    case FlightMode::Hovering:
        state_.velocityMetersPerSecond = glm::dvec3{0.0};
        break;

    case FlightMode::TakingOff:
        if (takeoffHeightMeters - state_.positionMeters.z
            <= climbSpeedMetersPerSecond * step + altitudeToleranceMeters)
        {
            state_.positionMeters.z = takeoffHeightMeters;
            state_.velocityMetersPerSecond = glm::dvec3{0.0};
            state_.mode = FlightMode::Hovering;
        }
        else
        {
            state_.positionMeters.z += climbSpeedMetersPerSecond * step;
        }
        break;

    case FlightMode::Flying:
        state_.positionMeters += state_.velocityMetersPerSecond * step;
        break;

    case FlightMode::FollowingWaypoint:
    {
        const glm::dvec3 offset = waypointMeters_ - state_.positionMeters;
        const double distance = glm::length(offset);
        const double stepDistance = maximumSpeedMetersPerSecond * step;
        if (distance <= std::max(waypointToleranceMeters, stepDistance))
        {
            state_.positionMeters = waypointMeters_;
            state_.velocityMetersPerSecond = glm::dvec3{0.0};
            state_.mode = FlightMode::Hovering;
            break;
        }

        state_.velocityMetersPerSecond = offset * (maximumSpeedMetersPerSecond / distance);
        state_.positionMeters += state_.velocityMetersPerSecond * step;
        break;
    }

    case FlightMode::Landing:
        if (state_.positionMeters.z <= climbSpeedMetersPerSecond * step + altitudeToleranceMeters)
        {
            state_.positionMeters.z = 0.0;
            state_.velocityMetersPerSecond = glm::dvec3{0.0};
            state_.mode = FlightMode::Grounded;
        }
        else
        {
            state_.positionMeters.z -= climbSpeedMetersPerSecond * step;
        }
        break;
    }
}

void DroneModel::Reset() noexcept
{
    state_ = {};
    waypointMeters_ = glm::dvec3{0.0};
    commandedYawRateRadiansPerSecond_ = 0.0;
}

bool DroneModel::IsAirborne() const noexcept
{
    return state_.mode == FlightMode::Hovering
        || state_.mode == FlightMode::Flying
        || state_.mode == FlightMode::FollowingWaypoint;
}
}
