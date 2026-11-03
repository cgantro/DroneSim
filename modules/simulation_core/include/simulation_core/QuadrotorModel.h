#pragma once

#include <glm/vec3.hpp>

#include "simulation_core/RigidBodyTypes.h"

#include <array>
#include <cstddef>

namespace dronesim::simulation
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

/** @brief 기체 기준 좌표에 놓인 PX4 모델의 상자 충돌 형상이다. */
using QuadrotorCollisionBox = RigidBodyBox;

/**
 * @brief 기체 질량·관성과 로터·모터의 계산값을 보관한다.
 * @details X500 기본값은 Gazebo의 중심 기체와 네 로터 링크를 한 강체로 합친 값이다. 로터 회전 자체의 관성 효과는 따로 계산하지 않는다.
 */
struct QuadrotorParameters
{
    static constexpr std::size_t rotorCount = 4;

    double massKilograms = 2.0643076923076924;
    glm::dvec3 principalInertiaKilogramMetersSquared{
        0.023846692647701416, 0.02394961745804886, 0.043999953714003955};
    /** @brief 물리 계산에서 동체의 공기 저항과 크기를 정하는 값이다. */
    glm::dvec3 bodySizeMeters{0.18, 0.11, 0.04};
    std::array<RotorDefinition, rotorCount> rotors{};
    std::array<QuadrotorCollisionBox, 9> collisionBoxes{};
    double thrustCoefficientNewtonsPerSpeedSquared = 8.54858e-6;
    double momentToThrustRatio = 0.016;
    double motorTimeConstantUpSeconds = 0.0125;
    double motorTimeConstantDownSeconds = 0.025;
    double maximumMotorSpeedRadiansPerSecond = 1000.0;

    /** @brief 비행·공기력 계산에 필요한 기체 제원이 유효한지 확인한다. */
    [[nodiscard]] bool HasValidPhysicalProperties() const noexcept;

    /** @brief PX4가 공개한 X500 Gazebo 모델의 중심 기체와 모터 설정값을 반환한다. */
    [[nodiscard]] static QuadrotorParameters X500Gazebo();

    /**
     * @brief 제조사 시험표를 참고해 추력·토크 계수와 최대 속도를 추정한다.
     * @details 질량·관성·로터 배치·모터 응답은 PX4 설정을 사용한다.
     * 시험표는 X500 V2와 같은 KV·프로펠러 규격을 쓰지만, 동일 부품으로 확인된 자료는 아니다.
     */
    [[nodiscard]] static QuadrotorParameters X500MotorTableEstimate();
};

/**
 * @brief 기체 좌표계의 상대 바람 속도에서 공기 저항력을 계산한다.
 * @details 표준 공기 밀도 1.225kg/m³와 항력 계수 1.0을 쓰는 간단한 축별 모델이다.
 * 각 축의 투영 면적과 속도의 제곱으로 저항을 구하며, 결과 단위는 뉴턴이다.
 */
[[nodiscard]] glm::dvec3 CalculateQuadrotorDragForceBody(
    const QuadrotorParameters& parameters,
    const glm::dvec3& relativeAirflowMetersPerSecond) noexcept;

/** @brief 고정 시간 동안 모터의 현재 회전수를 목표 회전수 쪽으로 갱신한다. */
[[nodiscard]] std::array<double, QuadrotorParameters::rotorCount> AdvanceMotorSpeeds(
    const QuadrotorParameters& parameters,
    const std::array<double, QuadrotorParameters::rotorCount>& currentSpeeds,
    const std::array<double, QuadrotorParameters::rotorCount>& targetSpeeds,
    double stepSeconds) noexcept;

/** @brief 로터 회전수로 각 로터의 추력을 계산한다. */
[[nodiscard]] std::array<double, QuadrotorParameters::rotorCount> CalculateRotorThrusts(
    const QuadrotorParameters& parameters,
    const std::array<double, QuadrotorParameters::rotorCount>& motorSpeeds) noexcept;

/** @brief 회전 방향과 추력에 따른 기체 Z축 반작용 토크를 계산한다. */
[[nodiscard]] double CalculateRotorReactionTorqueBodyZ(
    const QuadrotorParameters& parameters,
    const std::array<double, QuadrotorParameters::rotorCount>& rotorThrusts) noexcept;

}
