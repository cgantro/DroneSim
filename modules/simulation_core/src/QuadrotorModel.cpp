#include "simulation_core/QuadrotorModel.h"

#include <glm/gtc/quaternion.hpp>

#include <cmath>

namespace dronesim::simulation
{
namespace
{
constexpr double airDensityKilogramsPerCubicMeter = 1.225;
constexpr double quadrotorDragCoefficient = 1.0;

bool HasPositiveComponents(const glm::dvec3& value) noexcept
{
    return std::isfinite(value.x) && value.x > 0.0
        && std::isfinite(value.y) && value.y > 0.0
        && std::isfinite(value.z) && value.z > 0.0;
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
    constexpr double rotorCount = static_cast<double>(QuadrotorParameters::rotorCount);

    // Gazebo는 중심 기체와 로터를 링크로 나눈다. 여기서는 한 강체로 계산하므로
    // 로터 질량을 더하고, 평행축 정리 I = I_cm + m·d²로 각 로터 관성도 기체 중심에 옮긴다.
    // 수직축 관성은 로터가 기체 중심에서 x·y 양쪽으로 떨어져 있어 두 거리 제곱을 더한다.
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
    return parameters;
}

QuadrotorParameters QuadrotorParameters::X500MotorTableEstimate()
{
    QuadrotorParameters parameters = X500Gazebo();
    // 제조사 16V 모터·프로펠러 표의 전체 값을 T = k_f × ω²에 맞춘 결과다.
    parameters.thrustCoefficientNewtonsPerSpeedSquared = 1.2294935e-5;
    // 토크 표시는 소수 둘째 자리까지라 이 값도 근사치다.
    parameters.momentToThrustRatio = 0.01403352;
    // 표에 나온 가장 높은 시험 회전수다. 모터의 정격 한계는 아니다.
    parameters.maximumMotorSpeedRadiansPerSecond = 1032.2226;
    return parameters;
}

bool QuadrotorParameters::HasValidPhysicalProperties() const noexcept
{
    return std::isfinite(massKilograms) && massKilograms > 0.0
        && HasPositiveComponents(bodySizeMeters)
        && HasPositiveComponents(principalInertiaKilogramMetersSquared)
        && std::isfinite(thrustCoefficientNewtonsPerSpeedSquared)
        && thrustCoefficientNewtonsPerSpeedSquared > 0.0
        // 0은 로터의 반작용 토크를 무시하는 단순 모델에서 허용한다.
        && std::isfinite(momentToThrustRatio) && momentToThrustRatio >= 0.0
        && std::isfinite(motorTimeConstantUpSeconds)
        && motorTimeConstantUpSeconds > 0.0
        && std::isfinite(motorTimeConstantDownSeconds)
        && motorTimeConstantDownSeconds > 0.0
        && std::isfinite(maximumMotorSpeedRadiansPerSecond)
        && maximumMotorSpeedRadiansPerSecond > 0.0;
}

glm::dvec3 CalculateQuadrotorDragForceBody(
    const QuadrotorParameters& parameters,
    const glm::dvec3& relativeAirflowMetersPerSecond) noexcept
{
    const glm::dvec3& size = parameters.bodySizeMeters;
    const glm::dvec3 projectedAreas{
        size.y * size.z,
        size.x * size.z,
        size.x * size.y};
    glm::dvec3 dragForceBody{0.0};

    // 항력 식 F = -½ρC_dAv|v|를 기체의 세 축에 따로 적용한다.
    // v·|v|는 속도 부호를 보존하므로 힘이 언제나 상대 공기 흐름의 반대쪽을 향한다.
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const double airflow = relativeAirflowMetersPerSecond[axis];
        dragForceBody[axis] = -0.5 * airDensityKilogramsPerCubicMeter
            * quadrotorDragCoefficient * projectedAreas[axis]
            * airflow * std::abs(airflow);
    }
    return dragForceBody;
}

std::array<double, QuadrotorParameters::rotorCount> AdvanceMotorSpeeds(
    const QuadrotorParameters& parameters,
    const std::array<double, QuadrotorParameters::rotorCount>& currentSpeeds,
    const std::array<double, QuadrotorParameters::rotorCount>& targetSpeeds,
    double stepSeconds) noexcept
{
    std::array<double, QuadrotorParameters::rotorCount> nextSpeeds{};
    for (std::size_t rotor = 0; rotor < nextSpeeds.size(); ++rotor)
    {
        const bool accelerating = targetSpeeds[rotor] >= currentSpeeds[rotor];
        const double timeConstant = accelerating
            ? parameters.motorTimeConstantUpSeconds
            : parameters.motorTimeConstantDownSeconds;

        // 모터 속도를 목표값으로 즉시 바꾸지 않고 1차 응답으로 늦춘다.
        // 미분식 dω/dt=(목표-ω)/τ를 한 Tick 동안 푼 식이며, 상승·하강 시정수를 따로 쓴다.
        const double response = 1.0 - std::exp(-stepSeconds / timeConstant);
        nextSpeeds[rotor] = currentSpeeds[rotor]
            + (targetSpeeds[rotor] - currentSpeeds[rotor]) * response;
    }
    return nextSpeeds;
}

std::array<double, QuadrotorParameters::rotorCount> CalculateRotorThrusts(
    const QuadrotorParameters& parameters,
    const std::array<double, QuadrotorParameters::rotorCount>& motorSpeeds) noexcept
{
    std::array<double, QuadrotorParameters::rotorCount> thrusts{};
    for (std::size_t rotor = 0; rotor < thrusts.size(); ++rotor)
    {
        const double speed = motorSpeeds[rotor];
        // 이 모델의 모터 시험 계수 k_f에 따라 추력은 회전수 제곱으로 증가한다.
        thrusts[rotor] = parameters.thrustCoefficientNewtonsPerSpeedSquared
            * speed * speed;
    }
    return thrusts;
}

double CalculateRotorReactionTorqueBodyZ(
    const QuadrotorParameters& parameters,
    const std::array<double, QuadrotorParameters::rotorCount>& rotorThrusts) noexcept
{
    double reactionTorque = 0.0;
    for (std::size_t rotor = 0; rotor < rotorThrusts.size(); ++rotor)
    {
        // 프로펠러가 공기에 주는 회전 방향과 반대로 기체가 토크를 받는다.
        // 회전 방향 부호와 토크/추력 비율을 곱해 네 로터의 반작용 토크를 합산한다.
        const double sign = parameters.rotors[rotor].rotation
                == RotorRotation::CounterClockwise
            ? -1.0
            : 1.0;
        reactionTorque += sign * parameters.momentToThrustRatio
            * rotorThrusts[rotor];
    }
    return reactionTorque;
}
}
