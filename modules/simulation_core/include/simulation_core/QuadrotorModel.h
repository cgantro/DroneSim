#pragma once

#include <glm/vec3.hpp>

#include <array>
#include <cstddef>

namespace aerosimlab::simulation
{
enum class RotorRotation
{
    Clockwise,
    CounterClockwise
};

/** @brief 기체 중심을 기준으로 한 로터의 위치와 회전 방향이다. */
struct RotorDefinition
{
    glm::dvec3 positionMeters{0.0};
    RotorRotation rotation = RotorRotation::Clockwise;
};

/** @brief 한 단계에서 모터에 적용할 목표 속도와 현재 속도다. */
struct QuadrotorState
{
    std::array<double, 4> motorSpeedRadiansPerSecond{};
};

/**
 * @brief 기체 질량·관성과 로터·모터의 계산값을 보관한다.
 * @details X500 기본값의 질량과 관성은 Gazebo 모델의 중심 기체 링크 기준이다. 로터 링크 질량은 아직 합산하지 않는다.
 */
struct QuadrotorParameters
{
    double massKilograms = 2.0;
    glm::dvec3 principalInertiaKilogramMetersSquared{0.0216666667, 0.0216666667, 0.04};
    std::array<RotorDefinition, 4> rotors{};
    double thrustCoefficientNewtonsPerSpeedSquared = 8.54858e-6;
    double momentToThrustRatio = 0.016;
    double motorTimeConstantUpSeconds = 0.0125;
    double motorTimeConstantDownSeconds = 0.025;
    double maximumMotorSpeedRadiansPerSecond = 1000.0;

    /** @brief PX4가 공개한 X500 Gazebo 모델의 중심 기체와 모터 설정값을 반환한다. */
    [[nodiscard]] static QuadrotorParameters X500Gazebo();
};

/** @brief 기체 중심 좌표계에서 합한 힘과 토크다. */
struct QuadrotorWrench
{
    glm::dvec3 forceNewtons{0.0};
    glm::dvec3 torqueNewtonMeters{0.0};
};

/**
 * @brief 모터의 응답과 로터가 만드는 힘·토크를 계산한다.
 * @details 계산 결과는 기체 중심 좌표계 기준이다. 위치·자세 적분과 충돌은 물리 월드가 맡는다.
 */
class QuadrotorModel
{
public:
    static constexpr std::size_t rotorCount = 4;

    /** @brief 목표 회전 속도를 모터의 상승·하강 시간에 따라 현재 속도에 반영한다. */
    static void AdvanceMotors(
        QuadrotorState& state,
        const std::array<double, rotorCount>& targetSpeedRadiansPerSecond,
        const QuadrotorParameters& parameters,
        double stepSeconds);

    /** @brief 현재 로터 속도로 기체에 작용하는 힘과 토크를 계산한다. */
    [[nodiscard]] static QuadrotorWrench CalculateWrench(
        const QuadrotorState& state,
        const QuadrotorParameters& parameters);
};
}
