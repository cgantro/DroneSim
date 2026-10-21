#include "simulation_core/QuadrotorModel.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerosimlab::simulation
{
namespace
{
constexpr double gravityMetersPerSecondSquared = 9.81;

bool IsFinite(const glm::dvec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

void ValidateParameters(const QuadrotorParameters& parameters)
{
    if (!std::isfinite(parameters.massKilograms) || !(parameters.massKilograms > 0.0)
        || !IsFinite(parameters.principalInertiaKilogramMetersSquared)
        || !(parameters.principalInertiaKilogramMetersSquared.x > 0.0)
        || !(parameters.principalInertiaKilogramMetersSquared.y > 0.0)
        || !(parameters.principalInertiaKilogramMetersSquared.z > 0.0)
        || !std::isfinite(parameters.thrustCoefficientNewtonsPerSpeedSquared)
        || !(parameters.thrustCoefficientNewtonsPerSpeedSquared > 0.0)
        || !std::isfinite(parameters.momentToThrustRatio)
        || parameters.momentToThrustRatio < 0.0
        || !std::isfinite(parameters.motorTimeConstantUpSeconds)
        || !(parameters.motorTimeConstantUpSeconds > 0.0)
        || !std::isfinite(parameters.motorTimeConstantDownSeconds)
        || !(parameters.motorTimeConstantDownSeconds > 0.0)
        || !std::isfinite(parameters.maximumMotorSpeedRadiansPerSecond)
        || !(parameters.maximumMotorSpeedRadiansPerSecond > 0.0)
        || !std::isfinite(parameters.rotorDragCoefficientNewtonSecondsSquaredPerMeter)
        || parameters.rotorDragCoefficientNewtonSecondsSquaredPerMeter < 0.0
        || !std::isfinite(parameters.rollingMomentCoefficientNewtonMetersSecondsSquaredPerMeter)
        || parameters.rollingMomentCoefficientNewtonMetersSecondsSquaredPerMeter < 0.0)
    {
        throw std::invalid_argument("기체 질량, 관성, 로터와 모터 값을 확인해 주세요.");
    }

    for (const RotorDefinition& rotor : parameters.rotors)
    {
        if (!IsFinite(rotor.positionMeters))
            throw std::invalid_argument("로터 위치는 유한한 값이어야 합니다.");
    }
}

double DirectionSign(RotorRotation rotation)
{
    return rotation == RotorRotation::CounterClockwise ? 1.0 : -1.0;
}
}

QuadrotorParameters QuadrotorParameters::X500Gazebo()
{
    QuadrotorParameters parameters;
    parameters.massKilograms = 2.0;
    parameters.principalInertiaKilogramMetersSquared = {0.0216666667, 0.0216666667, 0.04};
    parameters.rotors = {{
        {{0.174, -0.174, 0.0}, RotorRotation::CounterClockwise},
        {{-0.174, 0.174, 0.0}, RotorRotation::CounterClockwise},
        {{0.174, 0.174, 0.0}, RotorRotation::Clockwise},
        {{-0.174, -0.174, 0.0}, RotorRotation::Clockwise}}};
    const glm::dquat legSlopeDown = glm::angleAxis(-0.35, glm::dvec3{1.0, 0.0, 0.0});
    const glm::dquat legSlopeUp = glm::angleAxis(0.35, glm::dvec3{1.0, 0.0, 0.0});
    parameters.collisionBoxes = {{
        {{0.1767767F, 0.1767767F, 0.025F}, {0.0, 0.0, 0.007}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0075F, 0.0075F, 0.105F}, {0.0, -0.098, -0.123}, legSlopeDown},
        {{0.0075F, 0.0075F, 0.105F}, {0.0, 0.098, -0.123}, legSlopeUp},
        {{0.125F, 0.0075F, 0.0075F}, {0.0, -0.132, -0.2195}, {1.0, 0.0, 0.0, 0.0}},
        {{0.125F, 0.0075F, 0.0075F}, {0.0, 0.132, -0.2195}, {1.0, 0.0, 0.0, 0.0}},
        {{0.1396154F, 0.00846154F, 0.00042308F}, {0.174, -0.174, 0.06}, {1.0, 0.0, 0.0, 0.0}},
        {{0.1396154F, 0.00846154F, 0.00042308F}, {-0.174, 0.174, 0.06}, {1.0, 0.0, 0.0, 0.0}},
        {{0.1396154F, 0.00846154F, 0.00042308F}, {0.174, 0.174, 0.06}, {1.0, 0.0, 0.0, 0.0}},
        {{0.1396154F, 0.00846154F, 0.00042308F}, {-0.174, -0.174, 0.06}, {1.0, 0.0, 0.0, 0.0}}}};
    parameters.thrustCoefficientNewtonsPerSpeedSquared = 8.54858e-6;
    parameters.momentToThrustRatio = 0.016;
    parameters.motorTimeConstantUpSeconds = 0.0125;
    parameters.motorTimeConstantDownSeconds = 0.025;
    parameters.maximumMotorSpeedRadiansPerSecond = 1000.0;
    parameters.rotorDragCoefficientNewtonSecondsSquaredPerMeter = 8.06428e-5;
    parameters.rollingMomentCoefficientNewtonMetersSecondsSquaredPerMeter = 1.0e-6;
    return parameters;
}

void QuadrotorModel::AdvanceMotors(
    QuadrotorState& state,
    const std::array<double, rotorCount>& targetSpeedRadiansPerSecond,
    const QuadrotorParameters& parameters,
    double stepSeconds)
{
    ValidateParameters(parameters);
    if (!std::isfinite(stepSeconds) || !(stepSeconds > 0.0))
        throw std::invalid_argument("모터 계산 시간은 0보다 큰 유한한 값이어야 합니다.");

    for (std::size_t rotor = 0; rotor < rotorCount; ++rotor)
    {
        double& currentSpeed = state.motorSpeedRadiansPerSecond[rotor];
        const double targetSpeed = targetSpeedRadiansPerSecond[rotor];
        if (!std::isfinite(currentSpeed) || currentSpeed < 0.0
            || !std::isfinite(targetSpeed) || targetSpeed < 0.0)
        {
            throw std::invalid_argument("모터 회전 속도는 0 이상의 유한한 값이어야 합니다.");
        }

        const double limitedTarget = std::min(
            targetSpeed, parameters.maximumMotorSpeedRadiansPerSecond);
        const double timeConstant = limitedTarget > currentSpeed
            ? parameters.motorTimeConstantUpSeconds
            : parameters.motorTimeConstantDownSeconds;
        const double response = 1.0 - std::exp(-stepSeconds / timeConstant);
        currentSpeed += (limitedTarget - currentSpeed) * response;
    }
}

QuadrotorWrench QuadrotorModel::CalculateWrench(
    const QuadrotorState& state,
    const QuadrotorParameters& parameters,
    const glm::dvec3& airRelativeVelocityBodyMetersPerSecond,
    const glm::dvec3& angularVelocityBodyRadiansPerSecond)
{
    ValidateParameters(parameters);
    if (!IsFinite(airRelativeVelocityBodyMetersPerSecond)
        || !IsFinite(angularVelocityBodyRadiansPerSecond))
    {
        throw std::invalid_argument("공기 상대 속도와 각속도는 유한한 값이어야 합니다.");
    }

    QuadrotorWrench wrench;
    for (std::size_t rotor = 0; rotor < rotorCount; ++rotor)
    {
        const double speed = state.motorSpeedRadiansPerSecond[rotor];
        if (!std::isfinite(speed) || speed < 0.0
            || speed > parameters.maximumMotorSpeedRadiansPerSecond)
        {
            throw std::invalid_argument("현재 모터 회전 속도를 확인해 주세요.");
        }

        const double thrust = parameters.thrustCoefficientNewtonsPerSpeedSquared * speed * speed;
        const glm::dvec3 rotorForce{0.0, 0.0, thrust};
        wrench.forceNewtons += rotorForce;
        wrench.torqueNewtonMeters += glm::cross(parameters.rotors[rotor].positionMeters, rotorForce);

        const glm::dvec3 rotorVelocityBody = airRelativeVelocityBodyMetersPerSecond
            + glm::cross(angularVelocityBodyRadiansPerSecond,
                parameters.rotors[rotor].positionMeters);
        const glm::dvec3 rotorAxisBody{0.0, 0.0, 1.0};
        const glm::dvec3 crossFlowVelocityBody = rotorVelocityBody
            - glm::dot(rotorVelocityBody, rotorAxisBody) * rotorAxisBody;
        const double absoluteSpeed = std::abs(speed);
        const glm::dvec3 rotorDragBody = -absoluteSpeed
            * parameters.rotorDragCoefficientNewtonSecondsSquaredPerMeter
            * crossFlowVelocityBody;
        wrench.forceNewtons += rotorDragBody;
        wrench.torqueNewtonMeters += glm::cross(
            parameters.rotors[rotor].positionMeters, rotorDragBody);
        wrench.torqueNewtonMeters -= absoluteSpeed
            * parameters.rollingMomentCoefficientNewtonMetersSecondsSquaredPerMeter
            * crossFlowVelocityBody;

        // 기체에는 프로펠러 회전 방향과 반대인 반작용 토크가 걸린다.
        wrench.torqueNewtonMeters.z -= DirectionSign(parameters.rotors[rotor].rotation)
            * parameters.momentToThrustRatio * thrust;
    }
    return wrench;
}
}
