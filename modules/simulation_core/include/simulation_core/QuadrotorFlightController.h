#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/QuadrotorModel.h"

#include <array>

namespace aerosimlab::simulation
{
/**
 * @brief 비행 목표를 로터별 목표 회전 속도로 바꾼다.
 * @details 위치·속도 제어로 필요한 추력 방향과 크기를 구한 뒤, 자세 제어 토크와 함께 네 로터에 나눈다.
 */
class QuadrotorFlightController
{
public:
    /** @brief PX4 X500 모델 기준점은 지면에서 0.24m 높이에 놓인다. */
    static constexpr double groundedModelOriginHeightMeters = 0.24;

    /**
     * @brief 현재 비행 목표를 로터별 목표 회전 속도로 바꾼다.
     * @details 위치·속도 오차로 가속도를 만든 뒤, 자세 토크와 추력을 네 로터에 나눈다.
     */
    [[nodiscard]] static std::array<double, QuadrotorParameters::rotorCount> MotorTargets(
        const DroneState& state,
        const DroneControlState& control,
        const QuadrotorParameters& parameters);
};
}
