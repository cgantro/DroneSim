#pragma once

#include "robotics/planning/LinearPathPlanner.h"

#include <cmath>

namespace grasplink::robotics::planning::detail
{

/** @brief 경로 표본을 풀 때 사용하는 IK 허용 오차를 정한다. */
kinematics::IkOptions PathIkOptions(const models::RobotSpecification& specification);

/** @brief 경로 명령의 목표와 속도·가속도 제한을 확인한다. */
Result ValidateLinearPathCommand(const LinearPathMoveCommand& command);

/** @brief IK 실패 상태를 제어기 결과 코드로 바꾼다. */
Result MapIkFailure(const kinematics::IkResult& result);

/** @brief 변위를 최대 속도 안에서 이동하는 데 필요한 최소 시간 [s]를 계산한다. */
inline double RequiredTimeForVelocity(double displacement, double maximumVelocity)
{
    return std::abs(displacement) / maximumVelocity;
}

/** @brief 주어진 시간에 필요한 속도를 최대 속도와 비교한 비율을 계산한다. */
inline double VelocityRatio(double displacement, double availableSeconds, double maximumVelocity)
{
    return std::abs(displacement) / availableSeconds / maximumVelocity;
}

/** @brief 허용 범위에서 기준 자세와 가장 가까운 회전 관절 각을 고른다. */
void AlignEquivalentJointAngles(
    JointVector& target,
    const JointVector& reference,
    const models::RobotSpecification& specification);

/** @brief 시작 자세부터 유효한 관절 경로를 만드는 IK 해를 고른다. */
std::optional<kinematics::IkResult> SolveCollisionFreeIk(
    kinematics::LevenbergMarquardtIk& inverse,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const CartesianPose& target,
    const JointVector& start,
    const kinematics::IkOptions& options,
    const PlanningPolicy& policy,
    JointStateInvalidity& invalidity,
    kinematics::IkResult& ikFailure);

} // namespace grasplink::robotics::planning::detail
