#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace grasplink::robotics::kinematics::detail
{
inline double HaltonValue(std::size_t index, std::size_t base)
{
    double value = 0.0;
    double scale = 1.0;
    while (index > 0)
    {
        scale /= static_cast<double>(base);
        value += scale * static_cast<double>(index % base);
        index /= base;
    }
    return value;
}

inline void BuildIkRestartSeed(std::size_t sample, const JointVector& start,
    const models::RobotSpecification& specification, JointVector& seed)
{
    constexpr std::array<std::size_t, 6> primeBases{2, 3, 5, 7, 11, 13};
    for (std::size_t jointIndex = 0; jointIndex < seed.size(); ++jointIndex)
    {
        const auto& joint = specification.joints[jointIndex];
        const double range = joint.maximumPosition - joint.minimumPosition;
        if (range <= 1e-12)
        {
            seed[jointIndex] = start[jointIndex];
            continue;
        }
        const double startFraction = (start[jointIndex] - joint.minimumPosition) / range;
        const double shiftedFraction = HaltonValue(sample,
            primeBases[jointIndex % primeBases.size()]) + startFraction - 0.5;
        seed[jointIndex] = joint.minimumPosition +
            (shiftedFraction - std::floor(shiftedFraction)) * range;
    }
}

inline void AppendIkRestartSeeds(std::vector<JointVector>& seeds,
    const JointVector& start, const models::RobotSpecification& specification,
    std::size_t maximumCount)
{
    const std::size_t maximumSamples = maximumCount * 8;
    for (std::size_t sample = 1; seeds.size() < maximumCount && sample <= maximumSamples; ++sample)
    {
        JointVector candidate(start.size());
        BuildIkRestartSeed(sample, start, specification, candidate);
        const bool duplicate = std::any_of(seeds.begin(), seeds.end(), [&](const JointVector& existing)
        {
            for (std::size_t joint = 0; joint < candidate.size(); ++joint)
            {
                if (std::abs(existing[joint] - candidate[joint]) > 1e-8)
                    return false;
            }
            return true;
        });
        if (!duplicate)
            seeds.push_back(std::move(candidate));
    }
}

/**
 * @brief 각 관절을 허용 범위의 반대편으로 옮긴 대체 시작 자세를 만든다.
 * @details 관절 수와 관계없이 한 관절씩 반사해, 현재 자세와 다른 후보만 남긴다.
 */
inline std::vector<JointVector> BuildAlternativeIkSeeds(
    const JointVector& start, const models::RobotSpecification& specification)
{
    std::vector<JointVector> seeds;
    seeds.reserve(start.size());
    for (std::size_t jointIndex = 0; jointIndex < start.size(); ++jointIndex)
    {
        const auto& joint = specification.joints[jointIndex];
        const double reflected = joint.minimumPosition + joint.maximumPosition - start[jointIndex];
        if (std::abs(reflected - start[jointIndex]) <= 1e-8)
            continue;
        JointVector candidate = start;
        candidate[jointIndex] = std::clamp(reflected, joint.minimumPosition, joint.maximumPosition);
        seeds.push_back(std::move(candidate));
    }
    return seeds;
}
}
