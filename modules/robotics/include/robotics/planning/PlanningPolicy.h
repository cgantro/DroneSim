#pragma once

#include <cstddef>

namespace grasplink::robotics::planning
{

/**
 * @brief 선형 TCP 경로를 계산할 때 정확도와 계산량을 조절한다.
 * @details 각 간격은 관절 충돌 검사나 TCP 경로 계산에서 사이의 점을 얼마나 촘촘히 검사할지 정한다.
 * IK 후보 수와 표본마다 허용할 IK 시도 수, TCP 직선 오차와 세분화 횟수도 제한해 계획 계산량을 상한 안에 둔다. IK 시도 한도에는 기존 후보를 잇는 필수 시도도 포함하므로 후보 수 이상이어야 한다.
 * 간격이 작거나 후보·세분화 한도가 크면 유효한 경로를 더 찾을 수 있지만 IK와 충돌 검사가 늘어난다.
 */
struct PlanningPolicy final
{
    /// 회전 관절 사이의 충돌 검사 간격 [rad]이다.
    double jointCollisionSampleSpacingRadians = 0.08;
    /// 직선 관절 사이의 충돌 검사 간격 [m]이다.
    double prismaticJointCollisionSampleSpacingMeters = 0.02;
    double linearPositionSampleSpacingMeters = 0.01;
    double linearOrientationSampleSpacingRadians = 0.05;
    std::size_t maximumPathIntervals = 4096;
    std::size_t maximumIkCandidatesPerSample = 8;
    std::size_t maximumIkSeedAttemptsPerSample = 16;
    // Cartesian 구간 하나가 오차를 만족하도록 허용하는 최대 이분 세분화 깊이다.
    std::size_t maximumTcpRefinementPasses = 12;
    double maximumLinearTcpErrorMeters = 0.001;
    double maximumAngularTcpErrorRadians = 0.001;
};

inline constexpr PlanningPolicy kDefaultPlanningPolicy{};

} // namespace grasplink::robotics::planning
