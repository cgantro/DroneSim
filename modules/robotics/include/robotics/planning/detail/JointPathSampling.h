#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"
#include "robotics/planning/PlanningPolicy.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

namespace grasplink::robotics::planning::detail
{

inline constexpr std::size_t kMaximumJointPathIntervals = 4096;

/**
 * @brief 관절 단위에 맞는 간격으로 경로 표본 수를 계산한다.
 * @details 회전 관절은 [rad], 직선 관절은 [m] 간격으로 나눈다. 계산 결과가 설정한 상한을 넘으면 표본 수를 반환하지 않는다.
 */
[[nodiscard]] inline std::optional<std::size_t> JointPathIntervalCount(
    const JointVector& start,
    const JointVector& end,
    const models::RobotSpecification& specification,
    const PlanningPolicy& policy)
{
    if (specification.joints == nullptr || start.size() != end.size() ||
        start.size() != specification.jointCount || policy.maximumPathIntervals == 0)
    {
        return std::nullopt;
    }

    const std::size_t maximumIntervals = std::min(
        policy.maximumPathIntervals, kMaximumJointPathIntervals);
    double largestSpacingCount = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
    {
        const double spacing = specification.joints[joint].motionType == models::JointMotionType::Prismatic
            ? policy.prismaticJointCollisionSampleSpacingMeters
            : policy.jointCollisionSampleSpacingRadians;
        if (!std::isfinite(spacing) || spacing <= 0.0)
            return std::nullopt;

        const double spacingCount = std::abs(end[joint] - start[joint]) / spacing;
        if (!std::isfinite(spacingCount))
            return std::nullopt;
        largestSpacingCount = std::max(largestSpacingCount, spacingCount);
    }

    const double intervalCount = std::max(1.0, std::ceil(largestSpacingCount));
    // 실수 값을 정수로 바꾸기 전에 상한을 확인해 범위를 넘는 변환을 막는다.
    if (intervalCount > static_cast<double>(maximumIntervals))
        return std::nullopt;
    return static_cast<std::size_t>(intervalCount);
}

} // namespace grasplink::robotics::planning::detail
