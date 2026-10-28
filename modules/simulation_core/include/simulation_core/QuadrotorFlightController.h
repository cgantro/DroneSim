#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/QuadrotorModel.h"

#include <array>

namespace aerosimlab::simulation
{
/** @brief 비행 제어기의 응답 속도를 정하는 이득이다.
 * @details 위치·자세 이득의 단위는 1/s², 속도·각속도 이득의 단위는 1/s다.
 */
struct QuadrotorControllerParameters
{
    double positionGain = 3.0;
    double velocityGain = 2.5;
    double attitudeGain = 20.0;
    double angularRateGain = 7.0;
};

inline constexpr QuadrotorControllerParameters defaultQuadrotorControllerParameters{};

/**
 * @brief 비행 목표를 로터별 목표 회전 속도로 바꾼다.
 * @details 위치·속도 제어로 필요한 추력 방향과 크기를 구한 뒤, 자세 제어 토크와 함께 네 로터에 나눈다.
 */
class QuadrotorFlightController
{
public:
    /** @brief PX4 X500 모델 기준점은 지면에서 0.24m 높이에 놓인다. */
    static constexpr double groundedModelOriginHeightMeters = 0.24;
    /** @brief 강체 비행에서 경유점을 향해 이동할 때의 최고 속도다. */
    static constexpr double maximumWaypointSpeedMetersPerSecond = 2.0;

    /** @brief 위치 제어기가 경유점을 향해 만들려는 속도를 돌려준다. */
    [[nodiscard]] static glm::dvec3 WaypointVelocityTarget(
        const DroneState& state,
        const glm::dvec3& targetPosition,
        const QuadrotorControllerParameters& controller = defaultQuadrotorControllerParameters);

    /**
     * @brief 현재 비행 목표를 로터별 목표 회전 속도로 바꾼다.
     * @details 위치·속도 오차로 가속도를 만든 뒤, 자세 토크와 추력을 네 로터에 나눈다.
     */
    [[nodiscard]] static std::array<double, QuadrotorParameters::rotorCount> MotorTargets(
        const DroneState& state,
        const DroneControlState& control,
        const QuadrotorParameters& parameters,
        const QuadrotorControllerParameters& controller = defaultQuadrotorControllerParameters);
};
}
