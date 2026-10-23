#include "simulation_core/QuadrotorModel.h"

#include <glm/gtc/quaternion.hpp>

namespace aerosimlab::simulation
{
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

    // 로터 링크의 질량과 관성을 중심 기체 기준으로 옮겨 하나의 강체 값으로 합친다.
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
}
