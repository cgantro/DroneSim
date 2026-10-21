#include "application/PickPlaceScenarioSampler.h"
#include "application/PickPlaceConfig.h"

#include <cmath>
#include <random>

namespace grasplink::application
{
namespace config = pick_place::config;

void PickPlaceScenarioSampler::EnsureSeeded()
{
    if (!initialized_)
    {
        generator_.seed(std::random_device{}());
        initialized_ = true;
    }
}

void PickPlaceScenarioSampler::Seed(std::uint32_t seed)
{
    generator_.seed(seed);
    initialized_ = true;
    hasPreviousBoxPosition_ = false;
    hasPreviousPlacementPosition_ = false;
}

std::array<float, 3> PickPlaceScenarioSampler::SamplePosition(float heightMeters,
    float minimumRadiusMeters, float maximumRadiusMeters, float minimumSeparationMeters,
    std::array<float, 3>& previousPosition, bool& hasPreviousPosition)
{
    EnsureSeeded();
    std::uniform_real_distribution<float> coordinate{-config::scenarioCoordinateLimitMeters,
        config::scenarioCoordinateLimitMeters};
    std::array<float, 3> position{};
    float radius = 0.0F;
    do
    {
        position = {coordinate(generator_), heightMeters, coordinate(generator_)};
        radius = std::hypot(position[0], position[2]);
    } while (radius < minimumRadiusMeters || radius > maximumRadiusMeters ||
        (hasPreviousPosition && std::hypot(position[0] - previousPosition[0],
            position[2] - previousPosition[2]) < minimumSeparationMeters));

    previousPosition = position;
    hasPreviousPosition = true;
    return position;
}

std::array<float, 3> PickPlaceScenarioSampler::SampleBoxPosition()
{
    return SamplePosition(config::boxPositionHeightMeters, config::minimumScenarioRadiusMeters,
        config::maximumBoxRadiusMeters, config::minimumBoxRepeatSeparationMeters,
        previousBoxPosition_, hasPreviousBoxPosition_);
}

std::array<float, 3> PickPlaceScenarioSampler::SamplePlacementPosition()
{
    return SamplePosition(config::placementPositionHeightMeters, config::minimumScenarioRadiusMeters,
        config::maximumPlacementRadiusMeters, config::minimumPlacementRepeatSeparationMeters,
        previousPlacementPosition_, hasPreviousPlacementPosition_);
}

float PickPlaceScenarioSampler::SamplePlanarRotation()
{
    EnsureSeeded();
    std::uniform_real_distribution<float> angle{-config::scenarioRotationLimitRadians,
        config::scenarioRotationLimitRadians};
    return angle(generator_);
}

}
