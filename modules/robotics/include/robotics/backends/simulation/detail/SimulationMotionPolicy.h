#pragma once

#include <cstddef>

namespace grasplink::robotics::backends::simulation::detail
{

/**
 * @brief Simulation backend의 실행 중 경로 보조 동작을 조절한다.
 * @details 경로 재시도는 제어 주기 안에서 실제 관절 이동을 이어갈 수 있는지를 확인하는 실행 정책이다.
 * 선형 경로의 샘플 간격과 최대 계획 구간 수는 robotics::planning의 PlanningPolicy가 관리한다.
 */
struct SimulationMotionPolicy final
{
    std::size_t linearRuntimeRetryAttempts = 16;
    std::size_t reorientationRetryAttempts = 12;
    double runtimeRetryProgressScale = 0.8;
};

inline constexpr SimulationMotionPolicy kDefaultSimulationMotionPolicy{};

inline constexpr double kMotionEpsilon = 1e-8;
inline constexpr double kJointVelocityRampSeconds = 0.20;
/** @brief 속도 상한 비율을 비교할 때 허용하는 부동소수점 오차다. */
inline constexpr double kVelocityRatioTolerance = 1e-8;
/** @brief 경로 시간에서 가속과 감속 구간이 차지하는 최소 비율이다. */
inline constexpr double kMinimumPathRampFraction = 0.005;
/** @brief 가속과 감속 사이에 일정 속도 구간을 남기기 위한 최대 비율이다. */
inline constexpr double kMaximumPathRampFraction = 0.45;

} // namespace grasplink::robotics::backends::simulation::detail
