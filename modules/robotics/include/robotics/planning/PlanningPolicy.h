#pragma once

#include <cstddef>

namespace grasplink::robotics::planning
{

/**
 * @brief 선형 TCP 경로를 계산할 때 정확도와 계산량을 조절한다.
 * @details 각 간격은 관절 충돌 검사나 TCP 경로 계산에서 사이의 점을 얼마나 촘촘히 검사할지 정한다.
 * IK 후보 수와 표본마다 허용할 IK 시도 수, TCP 직선 오차와 세분화 횟수도 제한해 계획 계산량을 상한 안에 둔다. IK 시도 한도에는 기존 후보를 잇는 필수 시도도 포함하므로 후보 수 이상이어야 한다.
 * 최대 횟수 설정은 구현이 허용하는 상한을 넘을 수 없다. 범위를 벗어난 정책은 경로 계획 요청에서 거부된다.
 * 간격이 작거나 후보·세분화 한도가 크면 유효한 경로를 더 찾을 수 있지만 IK와 충돌 검사가 늘어난다.
 */
struct PlanningPolicy final
{
    /// 회전 관절 사이의 충돌 검사 간격 [rad]이다.
    double jointCollisionSampleSpacingRadians = 0.08;
    /// 직선 관절 사이의 충돌 검사 간격 [m]이다.
    double prismaticJointCollisionSampleSpacingMeters = 0.02;
    /// 선형 TCP 이동에서 위치가 이 거리보다 크게 바뀌지 않도록 표본을 나누는 간격 [m]이다.
    double linearPositionSampleSpacingMeters = 0.01;
    /// 선형 TCP 이동에서 방향이 이 각도보다 크게 바뀌지 않도록 표본을 나누는 간격 [rad]이다.
    double linearOrientationSampleSpacingRadians = 0.05;
    /// 전체 경로를 나눌 구간 수의 상한이다.
    std::size_t maximumPathIntervals = 4096;
    /// 경로 표본 하나에서 비교할 서로 다른 IK 관절 해의 상한이다.
    std::size_t maximumIkCandidatesPerSample = 8;
    /// 경로 표본 하나에서 IK 해를 찾을 때 허용할 시작 관절 자세 수의 상한이다. 이전 표본의 해를 잇는 필수 시도도 포함한다.
    std::size_t maximumIkSeedAttemptsPerSample = 16;
    /// 한 TCP 구간의 위치·방향 오차를 맞추려고 중간 표본을 추가할 최대 반복 횟수다.
    std::size_t maximumTcpRefinementPasses = 12;
    /// TCP 경로 표본이 목표에서 벗어날 수 있는 최대 위치 오차 [m]다.
    double maximumLinearTcpErrorMeters = 0.001;
    /// TCP 경로 표본이 목표 방향에서 벗어날 수 있는 최대 각도 오차 [rad]다.
    double maximumAngularTcpErrorRadians = 0.001;
};

inline constexpr PlanningPolicy kDefaultPlanningPolicy{};

} // namespace grasplink::robotics::planning
