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
constexpr double positionGain = 3.0;
constexpr double velocityGain = 2.5;
constexpr double attitudeGain = 20.0;
constexpr double angularRateGain = 7.0;
constexpr double maximumAccelerationMetersPerSecondSquared = 4.0;
constexpr double maximumTorqueNewtonMeters = 0.5;

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

std::array<double, QuadrotorModel::rotorCount> SolveRotorThrusts(
    const QuadrotorWrench& desiredWrench,
    const QuadrotorParameters& parameters)
{
    constexpr std::size_t rotorCount = QuadrotorModel::rotorCount;
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

    // 네 로터가 만드는 힘과 세 축 토크를 한 번에 풀어 각 로터의 추력을 얻는다.
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

glm::dvec3 AttitudeTorque(
    const DroneState& state,
    const QuadrotorParameters& parameters,
    const glm::dquat& desiredOrientation)
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

    const glm::dvec3 angularVelocityBody =
        glm::inverse(currentOrientation) * state.angularVelocityRadiansPerSecond;
    const glm::dvec3 angularAcceleration =
        attitudeGain * angleError - angularRateGain * angularVelocityBody;
    return LimitMagnitude(
        parameters.principalInertiaKilogramMetersSquared * angularAcceleration,
        maximumTorqueNewtonMeters);
}
}

std::array<double, QuadrotorModel::rotorCount> QuadrotorFlightController::MotorTargets(
    const DroneState& state,
    const DroneControlState& control,
    const QuadrotorParameters& parameters,
    double stepSeconds)
{
    if (!std::isfinite(stepSeconds) || !(stepSeconds > 0.0))
        throw std::invalid_argument("제어기 시간 간격은 0보다 큰 유한한 값이어야 합니다.");

    if (state.mode == FlightMode::Grounded)
        return {};

    glm::dvec3 desiredAcceleration{0.0};
    glm::dvec3 desiredVelocity = control.targetVelocityMetersPerSecond;
    if (state.mode == FlightMode::TakingOff)
    {
        const glm::dvec3 targetPosition{
            state.positionMeters.x, state.positionMeters.y,
            control.groundElevationMeters + DroneModel::takeoffHeightMeters};
        desiredAcceleration = positionGain * (targetPosition - state.positionMeters)
            - velocityGain * state.velocityMetersPerSecond;
    }
    else if (state.mode == FlightMode::Landing)
    {
        const glm::dvec3 targetPosition{
            state.positionMeters.x, state.positionMeters.y,
            control.groundElevationMeters + groundedModelOriginHeightMeters};
        desiredAcceleration = positionGain * (targetPosition - state.positionMeters)
            - velocityGain * state.velocityMetersPerSecond;
    }
    else if (state.mode == FlightMode::FollowingWaypoint)
    {
        const glm::dvec3 positionError = control.waypointMeters - state.positionMeters;
        desiredAcceleration = positionGain * positionError
            - velocityGain * state.velocityMetersPerSecond;
    }
    else
    {
        desiredAcceleration = velocityGain
            * (desiredVelocity - state.velocityMetersPerSecond);
    }

    desiredAcceleration = LimitMagnitude(
        desiredAcceleration, maximumAccelerationMetersPerSecondSquared);
    const glm::dvec3 gravity{0.0, 0.0, gravityMetersPerSecondSquared};
    glm::dvec3 desiredWorldThrust = parameters.massKilograms * (desiredAcceleration + gravity);
    desiredWorldThrust.z = std::max(0.0, desiredWorldThrust.z);

    const glm::dquat desiredOrientation = DesiredOrientation(
        desiredWorldThrust, control.desiredYawRadians);
    const glm::dvec3 torqueBody = AttitudeTorque(state, parameters, desiredOrientation);
    const glm::dvec3 forceBody{0.0, 0.0, glm::length(desiredWorldThrust)};
    const QuadrotorWrench desiredWrench{forceBody, torqueBody};
    const auto rotorThrusts = SolveRotorThrusts(desiredWrench, parameters);

    std::array<double, QuadrotorModel::rotorCount> motorTargets{};
    for (std::size_t rotor = 0; rotor < motorTargets.size(); ++rotor)
    {
        motorTargets[rotor] = std::min(
            std::sqrt(rotorThrusts[rotor] / parameters.thrustCoefficientNewtonsPerSpeedSquared),
            parameters.maximumMotorSpeedRadiansPerSecond);
    }
    return motorTargets;
}
}
