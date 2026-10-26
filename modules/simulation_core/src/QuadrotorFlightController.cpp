#include "simulation_core/QuadrotorFlightController.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace aerosimlab::simulation
{
namespace
{
constexpr double gravityMetersPerSecondSquared = 9.81;
constexpr double maximumAccelerationMetersPerSecondSquared = 4.0;
constexpr double maximumTorqueNewtonMeters = 0.5;
constexpr double maximumWaypointSpeedMetersPerSecond = 2.0;

struct DesiredWrench
{
    glm::dvec3 forceNewtons{0.0};
    glm::dvec3 torqueNewtonMeters{0.0};
};

glm::dvec3 LimitMagnitude(const glm::dvec3& value, double maximumMagnitude)
{
    const double magnitude = glm::length(value);
    if (magnitude <= maximumMagnitude || magnitude == 0.0)
        return value;
    return value * (maximumMagnitude / magnitude);
}

double DirectionSign(RotorRotation direction)
{
    return direction == RotorRotation::CounterClockwise ? 1.0 : -1.0;
}

/** @brief 위치 오차로 목표점에 다가가고 현재 속도로 지나침을 줄인다. */
glm::dvec3 PositionControlAcceleration(
    const DroneState& state,
    const glm::dvec3& targetPosition,
    const QuadrotorControllerParameters& controller)
{
    const glm::dvec3 offset = targetPosition - state.positionMeters;
    const double distance = glm::length(offset);
    if (distance <= 1.0e-9)
        return -controller.velocityGain * state.velocityMetersPerSecond;

    // 정찰 경유점은 촬영에 맞춰 초속 2m를 넘지 않도록 천천히 접근한다.
    const double brakingSpeed = std::sqrt(
        2.0 * maximumAccelerationMetersPerSecondSquared * distance);
    const double approachSpeed = controller.positionGain * 0.2 * distance;
    const double targetSpeed = std::min(
        maximumWaypointSpeedMetersPerSecond,
        std::min(brakingSpeed, approachSpeed));
    const glm::dvec3 targetVelocity = offset * (targetSpeed / distance);
    return controller.velocityGain
        * (targetVelocity - state.velocityMetersPerSecond);
}

/** @brief 로터 추력 방향을 기체 위쪽 축에 맞추고 yaw 방향을 유지한다. */
glm::dquat DesiredOrientation(const glm::dvec3& worldThrust, double yawRadians)
{
    glm::dvec3 bodyZ = glm::normalize(worldThrust);
    const glm::dvec3 heading{std::cos(yawRadians), std::sin(yawRadians), 0.0};
    glm::dvec3 bodyX = heading - bodyZ * glm::dot(heading, bodyZ);
    if (glm::length(bodyX) < 1.0e-8)
        bodyX = glm::dvec3{1.0, 0.0, 0.0} - bodyZ * bodyZ.x;
    bodyX = glm::normalize(bodyX);
    const glm::dvec3 bodyY = glm::normalize(glm::cross(bodyZ, bodyX));
    return glm::normalize(glm::quat_cast(glm::dmat3{bodyX, bodyY, bodyZ}));
}

/** @brief 가장 짧은 자세 오차와 각속도로 기체 축 기준 토크를 계산한다. */
glm::dvec3 AttitudeTorque(
    const DroneState& state,
    const QuadrotorParameters& parameters,
    const glm::dquat& desiredOrientation,
    const QuadrotorControllerParameters& controller)
{
    const glm::dquat currentOrientation = glm::normalize(state.orientation);
    glm::dquat error = glm::normalize(glm::inverse(currentOrientation) * desiredOrientation);
    if (error.w < 0.0)
        error = -error;

    const glm::dvec3 vectorPart{error.x, error.y, error.z};
    const double vectorLength = glm::length(vectorPart);
    glm::dvec3 angleError{0.0};
    if (vectorLength > 1.0e-10)
    {
        const double angle = 2.0 * std::atan2(vectorLength, error.w);
        angleError = vectorPart * (angle / vectorLength);
    }

    // 상태의 각속도는 AirSim에서 기체 축 기준으로 전달되므로 다시 돌리지 않는다.
    const glm::dvec3& angularVelocityBody = state.angularVelocityRadiansPerSecond;
    const glm::dvec3 angularAcceleration =
        controller.attitudeGain * angleError - controller.angularRateGain * angularVelocityBody;
    return LimitMagnitude(
        parameters.principalInertiaKilogramMetersSquared * angularAcceleration,
        maximumTorqueNewtonMeters);
}

/**
 * @brief 네 로터로 목표 수직 힘과 세 축 토크를 나눈다.
 * @details 각 단계에서 절댓값이 가장 큰 계수를 기준으로 행을 바꿔 연립식을 푼다.
 */
std::array<double, QuadrotorParameters::rotorCount> SolveRotorThrusts(
    const DesiredWrench& desiredWrench,
    const QuadrotorParameters& parameters)
{
    constexpr std::size_t rotorCount = QuadrotorParameters::rotorCount;
    std::array<std::array<double, rotorCount + 1>, rotorCount> equation{};

    for (std::size_t rotor = 0; rotor < rotorCount; ++rotor)
    {
        const glm::dvec3& position = parameters.rotors[rotor].positionMeters;
        equation[0][rotor] = 1.0;
        equation[1][rotor] = position.y;
        equation[2][rotor] = -position.x;
        equation[3][rotor] = -DirectionSign(parameters.rotors[rotor].rotation)
            * parameters.momentToThrustRatio;
    }
    equation[0][rotorCount] = desiredWrench.forceNewtons.z;
    equation[1][rotorCount] = desiredWrench.torqueNewtonMeters.x;
    equation[2][rotorCount] = desiredWrench.torqueNewtonMeters.y;
    equation[3][rotorCount] = desiredWrench.torqueNewtonMeters.z;

    for (std::size_t column = 0; column < rotorCount; ++column)
    {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < rotorCount; ++row)
        {
            if (std::abs(equation[row][column]) > std::abs(equation[pivot][column]))
                pivot = row;
        }
        if (std::abs(equation[pivot][column]) < 1.0e-12)
            throw std::invalid_argument("로터 배치로 목표 힘과 토크를 만들 수 없습니다.");

        std::swap(equation[column], equation[pivot]);
        const double divisor = equation[column][column];
        for (std::size_t entry = column; entry <= rotorCount; ++entry)
            equation[column][entry] /= divisor;

        for (std::size_t row = 0; row < rotorCount; ++row)
        {
            if (row == column)
                continue;
            const double factor = equation[row][column];
            for (std::size_t entry = column; entry <= rotorCount; ++entry)
                equation[row][entry] -= factor * equation[column][entry];
        }
    }

    std::array<double, rotorCount> thrusts{};
    for (std::size_t rotor = 0; rotor < rotorCount; ++rotor)
        thrusts[rotor] = std::max(0.0, equation[rotor][rotorCount]);
    return thrusts;
}
}

std::array<double, QuadrotorParameters::rotorCount> QuadrotorFlightController::MotorTargets(
    const DroneState& state,
    const DroneControlState& control,
    const QuadrotorParameters& parameters,
    const QuadrotorControllerParameters& controller)
{
    glm::dvec3 desiredAcceleration{0.0};
    switch (state.mode)
    {
    case FlightMode::Grounded:
        return {};
    case FlightMode::TakingOff:
    {
        const glm::dvec3 targetPosition{
            state.positionMeters.x, state.positionMeters.y,
            control.groundElevationMeters + DroneModel::takeoffHeightMeters};
        desiredAcceleration = PositionControlAcceleration(state, targetPosition, controller);
        break;
    }
    case FlightMode::Landing:
    {
        const glm::dvec3 targetPosition{
            state.positionMeters.x, state.positionMeters.y,
            control.groundElevationMeters + groundedModelOriginHeightMeters};
        desiredAcceleration = PositionControlAcceleration(state, targetPosition, controller);
        break;
    }
    case FlightMode::FollowingWaypoint:
        desiredAcceleration = PositionControlAcceleration(
            state, control.waypointMeters, controller);
        break;
    case FlightMode::Hovering:
    case FlightMode::Flying:
        desiredAcceleration = controller.velocityGain * (
            control.targetVelocityMetersPerSecond - state.velocityMetersPerSecond);
        break;
    }

    desiredAcceleration = LimitMagnitude(
        desiredAcceleration, maximumAccelerationMetersPerSecondSquared);
    const glm::dvec3 gravity{0.0, 0.0, gravityMetersPerSecondSquared};
    glm::dvec3 desiredWorldThrust = parameters.massKilograms * (desiredAcceleration + gravity);
    desiredWorldThrust.z = std::max(0.0, desiredWorldThrust.z);

    const glm::dquat desiredOrientation = DesiredOrientation(
        desiredWorldThrust, control.desiredYawRadians);
    const glm::dvec3 torqueBody = AttitudeTorque(
        state, parameters, desiredOrientation, controller);
    const glm::dvec3 forceBody{0.0, 0.0, glm::length(desiredWorldThrust)};
    const DesiredWrench desiredWrench{forceBody, torqueBody};
    const auto rotorThrusts = SolveRotorThrusts(desiredWrench, parameters);

    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    for (std::size_t rotor = 0; rotor < motorTargets.size(); ++rotor)
    {
        motorTargets[rotor] = std::min(
            std::sqrt(rotorThrusts[rotor] / parameters.thrustCoefficientNewtonsPerSpeedSquared),
            parameters.maximumMotorSpeedRadiansPerSecond);
    }
    return motorTargets;
}
}
