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
    static constexpr double groundedCenterHeightMeters = 0.05;

    /** @brief 현재 상태와 명령으로 다음 고정 Tick의 모터 목표를 계산한다. */
    [[nodiscard]] static std::array<double, QuadrotorModel::rotorCount> MotorTargets(
        const DroneState& state,
        const DroneControlState& control,
        const QuadrotorParameters& parameters,
        double stepSeconds);
};
}
