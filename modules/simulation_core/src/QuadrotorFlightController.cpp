#include "simulation_core/QuadrotorFlightController.h"
#include "geometry/VectorMath.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace dronesim::simulation
{
namespace
{
constexpr double gravityMetersPerSecondSquared = 9.81;
constexpr double maximumAccelerationMetersPerSecondSquared = 4.0;
constexpr double maximumTorqueNewtonMeters = 0.5;
constexpr double waypointApproachTimeSeconds = 0.2;

struct DesiredWrench
{
    glm::dvec3 forceNewtons{0.0};
    glm::dvec3 torqueNewtonMeters{0.0};
};

double DirectionSign(RotorRotation direction)
{
    return direction == RotorRotation::CounterClockwise ? 1.0 : -1.0;
}

/**
 * @brief 위치 오차를 목표 속도로 바꾼 뒤 속도 오차에서 가속도를 구한다.
 * @details 위치 제어가 만든 목표 속도와 현재 속도의 차이에 속도 이득 [1/s]을
 *          곱한다. 따라서 반환값은 월드 좌표계의 가속도 [m/s²]다.
 */
glm::dvec3 PositionControlAcceleration(
    const DroneState& state,
    const glm::dvec3& targetPosition,
    const QuadrotorControllerParameters& controller)
{
    const glm::dvec3 targetVelocity =
        QuadrotorFlightController::PositionTargetVelocity(
            state, targetPosition, controller);
    return controller.velocityGain
        * (targetVelocity - state.velocityMetersPerSecond);
}

/**
 * @brief 현재 비행 모드의 위치 또는 속도 목표를 월드 가속도로 바꾼다.
 * @details 이륙·착륙·경유점 이동은 위치 제어를 쓰고, 호버링·수동 비행은
 *          직접 지정된 목표 속도를 따른다. 지상 상태에서는 가속도를 요구하지 않는다.
 */
glm::dvec3 AccelerationForFlightMode(
    const DroneState& state,
    const DroneControlState& control,
    const QuadrotorControllerParameters& controller)
{
    switch (state.mode)
    {
    case FlightMode::Grounded:
        return glm::dvec3{0.0};

    case FlightMode::TakingOff:
    {
        const glm::dvec3 targetPosition{
            state.positionMeters.x, state.positionMeters.y,
            control.groundElevationMeters + DroneModel::takeoffHeightMeters};
        return PositionControlAcceleration(state, targetPosition, controller);
    }

    case FlightMode::Landing:
    {
        const glm::dvec3 targetPosition{
            state.positionMeters.x, state.positionMeters.y,
            control.groundElevationMeters
                + QuadrotorFlightController::groundedModelOriginHeightMeters};
        return PositionControlAcceleration(state, targetPosition, controller);
    }

    case FlightMode::FollowingWaypoint:
        return PositionControlAcceleration(
            state, control.waypointMeters, controller);

    case FlightMode::Hovering:
    case FlightMode::Flying:
        return controller.velocityGain
            * (control.targetVelocityMetersPerSecond
                - state.velocityMetersPerSecond);
    }

    return glm::dvec3{0.0};
}

/** @brief 로터 추력 방향을 기체 위쪽 축에 맞추고 yaw 방향을 유지한다. */
glm::dquat DesiredOrientation(const glm::dvec3& worldThrust, double yawRadians)
{
    // 로터 추력은 기체 Z축을 따라 작용하므로, 먼저 그 축을 필요한 추력 방향에 맞춘다.
    glm::dvec3 bodyZ = glm::normalize(worldThrust);
    const glm::dvec3 heading{std::cos(yawRadians), std::sin(yawRadians), 0.0};
    // 헤딩을 추력축과 수직인 평면에 투영해, 기울어진 자세에서도 요청한 요 방향을 최대한 유지한다.
    glm::dvec3 bodyX = heading - bodyZ * glm::dot(heading, bodyZ);
    if (glm::length(bodyX) < 1.0e-8)
        bodyX = glm::dvec3{1.0, 0.0, 0.0} - bodyZ * bodyZ.x;
    bodyX = glm::normalize(bodyX);
    const glm::dvec3 bodyY = glm::normalize(glm::cross(bodyZ, bodyX));
    return glm::normalize(glm::quat_cast(glm::dmat3{bodyX, bodyY, bodyZ}));
}

/**
 * @brief 자세 오차와 각속도로 기체 축 기준 토크를 계산한다.
 * @details 현재 자세의 역회전을 목표 자세에 곱해 현재 기체 좌표계의 오차 회전을
 *          구한다. 쿼터니언의 벡터부를 축·각도로 바꾼 뒤 PD 제어로 각가속도를 만들고,
 *          주관성 모멘트를 곱해 토크 [N·m]로 변환한다.
 */
glm::dvec3 AttitudeTorque(
    const DroneState& state,
    const QuadrotorParameters& parameters,
    const glm::dquat& desiredOrientation,
    const QuadrotorControllerParameters& controller)
{
    const glm::dquat currentOrientation = glm::normalize(state.orientation);
    glm::dquat error = glm::normalize(glm::inverse(currentOrientation) * desiredOrientation);
    // q와 -q는 같은 자세를 나타낸다. 실수부를 양수로 맞춰 회전각이 더 작은 쪽을 고른다.
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

    // 물리 엔진이 기체 축으로 돌려준 각속도이므로, 자세 오차와 같은 좌표계에서 바로 쓴다.
    const glm::dvec3& angularVelocityBody = state.angularVelocityRadiansPerSecond;
    const glm::dvec3 angularAcceleration =
        controller.attitudeGain * angleError - controller.angularRateGain * angularVelocityBody;
    return geometry::ClampMagnitude(
        parameters.principalInertiaKilogramMetersSquared * angularAcceleration,
        maximumTorqueNewtonMeters);
}

/**
 * @brief 네 로터로 목표 수직 힘과 세 축 토크를 나눈다.
 * @details 각 로터의 추력을 미지수로 두고, 수직 힘·롤·피치·요 토크 식을 연립해 푼다.
 * 로터 위치와 회전 방향이 각 토크 식의 계수가 된다. 행을 바꿀 때는 현재 열에서
 * 절댓값이 가장 큰 계수를 골라 계산 오차를 줄인다.
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
        // 첫째 행은 네 추력의 합, 다음 두 행은 로터의 팔 길이로 만드는 롤·피치 토크다.
        equation[0][rotor] = 1.0;
        equation[1][rotor] = position.y;
        equation[2][rotor] = -position.x;
        // 회전 방향이 반대인 로터는 기체에 전달하는 요 반작용 토크 부호도 반대다.
        equation[3][rotor] = -DirectionSign(parameters.rotors[rotor].rotation)
            * parameters.momentToThrustRatio;
    }
    equation[0][rotorCount] = desiredWrench.forceNewtons.z;
    equation[1][rotorCount] = desiredWrench.torqueNewtonMeters.x;
    equation[2][rotorCount] = desiredWrench.torqueNewtonMeters.y;
    equation[3][rotorCount] = desiredWrench.torqueNewtonMeters.z;

    for (std::size_t column = 0; column < rotorCount; ++column)
    {
        // 부분 피벗팅으로 작은 수로 나누는 일을 피한다. 피벗이 0에 가까우면
        // 현재 로터 배치로 네 축의 요구량을 독립적으로 맞출 수 없다.
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

        // 피벗 행을 1로 만든 다음 다른 행에서 해당 열을 없애면 해가 마지막 열에 남는다.
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
    {
        // 로터는 아래 방향 힘을 만들 수 없으므로 음수 요구량은 0으로 제한한다.
        thrusts[rotor] = std::max(0.0, equation[rotor][rotorCount]);
    }
    return thrusts;
}
}

glm::dvec3 QuadrotorFlightController::PositionTargetVelocity(
    const DroneState& state,
    const glm::dvec3& targetPosition,
    const QuadrotorControllerParameters& controller)
{
    const glm::dvec3 offset = targetPosition - state.positionMeters;
    const double distanceMeters = glm::length(offset);
    if (distanceMeters <= 1.0e-9)
        return glm::dvec3{0.0};

    // v² = 2ad에서 남은 거리 안에 멈출 수 있는 최대 속도를 구한다.
    const double speedAllowedByBrakingMetersPerSecond = std::sqrt(
        2.0 * maximumAccelerationMetersPerSecondSquared * distanceMeters);
    // 위치 이득과 접근 시간을 곱해 목표점 가까이에서 속도를 부드럽게 낮춘다.
    const double speedAllowedByApproachMetersPerSecond =
        controller.positionGain * waypointApproachTimeSeconds * distanceMeters;
    const double speedAllowedByControlMetersPerSecond = std::min(
        speedAllowedByBrakingMetersPerSecond,
        speedAllowedByApproachMetersPerSecond);
    const double targetSpeedMetersPerSecond = std::min(
        maximumWaypointSpeedMetersPerSecond,
        speedAllowedByControlMetersPerSecond);
    return offset * (targetSpeedMetersPerSecond / distanceMeters);
}

std::array<double, QuadrotorParameters::rotorCount> QuadrotorFlightController::MotorTargets(
    const DroneState& state,
    const DroneControlState& control,
    const QuadrotorParameters& parameters,
    const QuadrotorControllerParameters& controller)
{
    if (control.commandedRotorThrustRatios)
    {
        std::array<double, QuadrotorParameters::rotorCount> targets{};
        for (std::size_t rotor = 0; rotor < targets.size(); ++rotor)
        {
            // 추력은 회전수 제곱에 비례하므로, 추력 비율을 회전수 비율로 바꾼다.
            targets[rotor] = parameters.maximumMotorSpeedRadiansPerSecond
                * std::sqrt((*control.commandedRotorThrustRatios)[rotor]);
        }
        return targets;
    }

    glm::dvec3 desiredAcceleration = AccelerationForFlightMode(
        state, control, controller);
    desiredAcceleration = geometry::ClampMagnitude(
        desiredAcceleration, maximumAccelerationMetersPerSecondSquared);
    const glm::dvec3 gravity{0.0, 0.0, gravityMetersPerSecondSquared};
    // F = m(a + g): 원하는 가속도에 중력 상쇄분을 더해 모터가 만들어야 할 힘을 구한다.
    glm::dvec3 desiredWorldThrust = parameters.massKilograms * (desiredAcceleration + gravity);
    desiredWorldThrust.z = std::max(0.0, desiredWorldThrust.z);

    const glm::dquat desiredOrientation = DesiredOrientation(
        desiredWorldThrust, control.desiredYawRadians);
    const glm::dvec3 torqueBody = AttitudeTorque(
        state, parameters, desiredOrientation, controller);
    // 목표 자세의 기체 Z축을 추력 방향에 맞췄으므로, 기체 좌표계에서는 수직 성분만 필요하다.
    const glm::dvec3 forceBody{0.0, 0.0, glm::length(desiredWorldThrust)};
    const DesiredWrench desiredWrench{forceBody, torqueBody};
    const auto rotorThrusts = SolveRotorThrusts(desiredWrench, parameters);

    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    for (std::size_t rotor = 0; rotor < motorTargets.size(); ++rotor)
    {
        // 추력식 T = k_f·ω²을 회전수로 풀고, 모터의 최대 회전수를 넘지 않게 한다.
        motorTargets[rotor] = std::min(
            std::sqrt(rotorThrusts[rotor] / parameters.thrustCoefficientNewtonsPerSpeedSquared),
            parameters.maximumMotorSpeedRadiansPerSecond);
    }
    return motorTargets;
}
}
