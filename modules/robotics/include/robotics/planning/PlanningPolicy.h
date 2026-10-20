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

/**
 * @brief MoveL의 각 TCP 표본에서 IK 시작 자세를 최대 16개까지 검사한다.
 * @details 이전 표본의 후보를 이어서 시도하고, 관절 한계나 충돌로 막히면 대체 관절 분기도 탐색한다.
 * Viewer는 계획 계산을 프레임별 작업량으로 나누므로 이 상한을 유지하면서 한 프레임의 긴 정지를 막는다.
 * 오프라인 계획은 별도 정책으로 탐색 폭을 조정할 수 있다.
 */
inline constexpr PlanningPolicy kInteractivePlanningPolicy = []
{
    PlanningPolicy policy{};
    policy.maximumIkSeedAttemptsPerSample = 16;
    return policy;
}();

} // namespace grasplink::robotics::planning
