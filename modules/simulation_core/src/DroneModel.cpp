#include "simulation_core/DroneModel.h"
#include "geometry/Finite.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string_view>

namespace dronesim::simulation
{
namespace
{
glm::dvec3 ClampLength(const glm::dvec3& value, double maximumLength) noexcept
{
    const double length = std::hypot(value.x, value.y, value.z);
    if (length <= maximumLength || length == 0.0)
    {
        return value;
    }
    return value * (maximumLength / length);
}

FlightMode ModeAfterVelocityCommand(
    const glm::dvec3& targetVelocityMetersPerSecond,
    const glm::dvec3& currentVelocityMetersPerSecond) noexcept
{
    // 목표 속도가 0이어도 기체가 움직이는 중이면 Flying으로 두어 감속을 계속한다.
    const bool targetIsStopped = glm::length(targetVelocityMetersPerSecond) == 0.0;
    const bool droneHasStopped = glm::length(currentVelocityMetersPerSecond) < 0.05;
    return targetIsStopped && droneHasStopped
        ? FlightMode::Hovering
        : FlightMode::Flying;
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

constexpr double takeoffHeightToleranceMeters = 0.03;
constexpr double waypointArrivalToleranceMeters = 0.15;
constexpr double stoppedSpeedMetersPerSecond = 0.15;
constexpr double landingHeightToleranceMeters = 0.02;
}

/** @brief 화면·기록에 쓰는 비행 상태 이름을 반환한다. */
std::string_view ToString(FlightMode mode) noexcept
{
    for (const FlightModeName& entry : flightModeNames)
    {
        if (entry.mode == mode)
            return entry.name;
    }
    return "unknown";
}

/** @brief 저장된 상태 이름을 비행 상태로 바꾸며 모르는 이름은 거부한다. */
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
        if (!CanAcceptInFlightCommands(state.mode)
            && state.mode != FlightMode::TakingOff)
        {
            return CommandResult::InvalidFlightMode;
        }
        state.mode = FlightMode::Landing;
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetVelocity:
        if (!CanAcceptInFlightCommands(state.mode))
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!geometry::IsFinite(command.vector))
        {
            return CommandResult::InvalidValue;
        }
        control.targetVelocityMetersPerSecond = ClampLength(
            command.vector, maximumSpeedMetersPerSecond);
        state.mode = ModeAfterVelocityCommand(
            control.targetVelocityMetersPerSecond,
            state.velocityMetersPerSecond);
        return CommandResult::Accepted;

    case DroneCommand::Kind::SetWaypoint:
    {
        if (!CanAcceptInFlightCommands(state.mode))
        {
            return CommandResult::InvalidFlightMode;
        }
        if (!geometry::IsFinite(command.vector) || command.vector.z < 0.0)
        {
            return CommandResult::InvalidValue;
        }
        if (command.targetYawRadians && !std::isfinite(*command.targetYawRadians))
        {
            return CommandResult::InvalidValue;
        }
        control.waypointMeters = command.vector;
        // 목표 방향을 따로 주지 않으면 현재 위치에서 경유점까지의 방위로 기수를 돌린다.
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
        if (!CanAcceptInFlightCommands(state.mode))
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
        if (!CanAcceptInFlightCommands(state.mode))
        {
            return CommandResult::InvalidFlightMode;
        }
        control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
        state.mode = ModeAfterVelocityCommand(
            control.targetVelocityMetersPerSecond,
            state.velocityMetersPerSecond);
        control.commandedYawRateRadiansPerSecond = 0.0;
        return CommandResult::Accepted;
    }

    return CommandResult::InvalidValue;
}

DroneState DroneModel::PrepareInitialState(
    DroneState state, double maximumMotorSpeedRadiansPerSecond)
{
    const double orientationLength = glm::length(state.orientation);
    const bool validMotorSpeeds = std::all_of(
        state.motorSpeedsRadiansPerSecond.begin(),
        state.motorSpeedsRadiansPerSecond.end(),
        [](double speed) { return std::isfinite(speed) && speed >= 0.0; });

    if (!geometry::IsFinite(state.positionMeters)
        || !geometry::IsFinite(state.velocityMetersPerSecond)
        || !geometry::IsFinite(state.angularVelocityRadiansPerSecond)
        || !std::isfinite(orientationLength) || orientationLength < 1.0e-12
        || !validMotorSpeeds)
    {
        throw std::invalid_argument("드론 시작 상태의 위치·속도·자세를 확인해 주세요.");
    }
    if (std::any_of(state.motorSpeedsRadiansPerSecond.begin(),
        state.motorSpeedsRadiansPerSecond.end(),
        [maximumMotorSpeedRadiansPerSecond](double speed)
        {
            return speed > maximumMotorSpeedRadiansPerSecond;
        }))
    {
        throw std::invalid_argument("초기 로터 속도가 기체의 최대 속도를 넘었습니다.");
    }

    // 물리 엔진에는 길이가 1인 쿼터니언을 넘겨야 회전으로 안정적으로 해석된다.
    state.orientation = glm::normalize(state.orientation);
    state.yawRadians = YawRadians(state);
    return state;
}

void DroneModel::UpdateFlightModeAfterPhysics(
    DroneState& state, DroneControlState& control,
    double groundedModelOriginHeightMeters) noexcept
{
    // 위치뿐 아니라 하강·이동 속도도 확인해 아직 움직이는 기체를 완료 상태로 바꾸지 않는다.
    switch (state.mode)
    {
    case FlightMode::TakingOff:
        if (state.positionMeters.z >= control.groundElevationMeters
                + takeoffHeightMeters - takeoffHeightToleranceMeters
            && std::abs(state.velocityMetersPerSecond.z) < stoppedSpeedMetersPerSecond)
        {
            state.mode = FlightMode::Hovering;
        }
        break;

    case FlightMode::FollowingWaypoint:
        if (glm::length(control.waypointMeters - state.positionMeters)
                < waypointArrivalToleranceMeters
            && glm::length(state.velocityMetersPerSecond) < stoppedSpeedMetersPerSecond)
        {
            state.mode = FlightMode::Hovering;
            control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
        }
        break;

    case FlightMode::Landing:
        if (state.positionMeters.z <= control.groundElevationMeters
                + groundedModelOriginHeightMeters + landingHeightToleranceMeters
            && std::abs(state.velocityMetersPerSecond.z) < stoppedSpeedMetersPerSecond)
        {
            state.mode = FlightMode::Grounded;
            control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
            control.commandedYawRateRadiansPerSecond = 0.0;
        }
        break;

    case FlightMode::Grounded:
    case FlightMode::Hovering:
    case FlightMode::Flying:
        break;
    }
}

double DroneModel::YawRadians(const DroneState& state) noexcept
{
    // 자세 쿼터니언에서 수직축 기준 회전각만 뽑아 게임패드 좌표 변환에 쓴다.
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

bool DroneModel::CanAcceptInFlightCommands(FlightMode mode) noexcept
{
    return mode == FlightMode::Hovering
        || mode == FlightMode::Flying
        || mode == FlightMode::FollowingWaypoint;
}
}
