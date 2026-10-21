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
    constexpr double baseMassKilograms = 2.0;
    constexpr double baseInertiaX = 0.02166666666666667;
    constexpr double baseInertiaY = 0.02166666666666667;
    constexpr double baseInertiaZ = 0.04000000000000001;
    constexpr double rotorMassKilograms = 0.016076923076923075;
    constexpr double rotorArmMeters = 0.174;
    constexpr double rotorHeightMeters = 0.06;
    constexpr double rotorInertiaX = 3.8464910483993325e-7;
    constexpr double rotorInertiaY = 2.6115851691700804e-5;
    constexpr double rotorInertiaZ = 2.649858234714004e-5;
    constexpr double rotorCount = static_cast<double>(QuadrotorModel::rotorCount);

    // 로터 링크의 질량과 관성을 중심 기체 기준으로 옮겨 한 강체의 관성으로 합친다.
    parameters.massKilograms = baseMassKilograms + rotorCount * rotorMassKilograms;
    parameters.principalInertiaKilogramMetersSquared = {
        baseInertiaX + rotorCount * (rotorInertiaX + rotorMassKilograms
            * (rotorArmMeters * rotorArmMeters + rotorHeightMeters * rotorHeightMeters)),
        baseInertiaY + rotorCount * (rotorInertiaY + rotorMassKilograms
            * (rotorArmMeters * rotorArmMeters + rotorHeightMeters * rotorHeightMeters)),
        baseInertiaZ + rotorCount * (rotorInertiaZ + rotorMassKilograms
            * (2.0 * rotorArmMeters * rotorArmMeters))};
    parameters.rotors = {{
        {{rotorArmMeters, -rotorArmMeters, rotorHeightMeters}, RotorRotation::CounterClockwise},
        {{-rotorArmMeters, rotorArmMeters, rotorHeightMeters}, RotorRotation::CounterClockwise},
        {{rotorArmMeters, rotorArmMeters, rotorHeightMeters}, RotorRotation::Clockwise},
        {{-rotorArmMeters, -rotorArmMeters, rotorHeightMeters}, RotorRotation::Clockwise}}};
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

QuadrotorParameters QuadrotorParameters::X500MotorTableEstimate()
{
    QuadrotorParameters parameters = X500Gazebo();
    parameters.thrustCoefficientNewtonsPerSpeedSquared = 1.067745911e-5;
    parameters.momentToThrustRatio = 0.01427859758;
    // 정격 한계가 아니라 공개 표에 실린 가장 높은 시험 회전수다.
    parameters.maximumMotorSpeedRadiansPerSecond = 1120.8155390457184;
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
