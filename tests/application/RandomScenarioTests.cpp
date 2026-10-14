#include "application/PickPlaceScenarioSampler.h"
#include "application/PickPlaceConfig.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
struct ScenarioSample
{
    std::array<float, 3> box;
    std::array<float, 3> placement;
    float boxRotation;
    float placementRotation;
};

ScenarioSample SampleSequence(grasplink::application::PickPlaceScenarioSampler& sampler)
{
    return {sampler.SampleBoxPosition(), sampler.SamplePlacementPosition(),
        sampler.SamplePlanarRotation(), sampler.SamplePlanarRotation()};
}
}

TEST(RandomScenarioTests, SeedRestoresTheSameSequenceAndPositionHistory)
{
    grasplink::application::PickPlaceScenarioSampler sampler;
    sampler.Seed(13542);
    const auto first = SampleSequence(sampler);
    const auto second = SampleSequence(sampler);

    sampler.Seed(13542);

    EXPECT_EQ(SampleSequence(sampler).box, first.box);
    const auto replayedFirst = SampleSequence(sampler);
    EXPECT_EQ(replayedFirst.box, second.box);
    EXPECT_EQ(replayedFirst.placement, second.placement);
    EXPECT_FLOAT_EQ(replayedFirst.boxRotation, second.boxRotation);
    EXPECT_FLOAT_EQ(replayedFirst.placementRotation, second.placementRotation);
}

TEST(RandomScenarioTests, SamplerInstancesKeepIndependentSeedAndPositionHistory)
{
    grasplink::application::PickPlaceScenarioSampler firstSampler;
    grasplink::application::PickPlaceScenarioSampler secondSampler;
    firstSampler.Seed(251);
    secondSampler.Seed(251);

    EXPECT_EQ(firstSampler.SampleBoxPosition(), secondSampler.SampleBoxPosition());
    EXPECT_EQ(firstSampler.SamplePlacementPosition(), secondSampler.SamplePlacementPosition());
    EXPECT_FLOAT_EQ(firstSampler.SamplePlanarRotation(), secondSampler.SamplePlanarRotation());
}

TEST(RandomScenarioTests, SamplesCoverBothCoordinateSignsWithinConfiguredReach)
{
    namespace config = grasplink::application::pick_place::config;
    grasplink::application::PickPlaceScenarioSampler sampler;
    sampler.Seed(8061);
    bool negativeX = false;
    bool positiveX = false;
    bool negativeZ = false;
    bool positiveZ = false;
    std::array<float, 3> previousBox{};
    std::array<float, 3> previousPlacement{};

    for (int sample = 0; sample < 128; ++sample)
    {
        const auto box = sampler.SampleBoxPosition();
        const auto placement = sampler.SamplePlacementPosition();
        const float boxRadius = std::hypot(box[0], box[2]);
        const float placementRadius = std::hypot(placement[0], placement[2]);
        EXPECT_GE(boxRadius, config::minimumScenarioRadiusMeters);
        EXPECT_LE(boxRadius, config::maximumBoxRadiusMeters);
        EXPECT_GE(placementRadius, config::minimumScenarioRadiusMeters);
        EXPECT_LE(placementRadius, config::maximumPlacementRadiusMeters);
        EXPECT_FLOAT_EQ(box[1], config::boxPositionHeightMeters);
        EXPECT_FLOAT_EQ(placement[1], config::placementPositionHeightMeters);
        EXPECT_GE(std::hypot(box[0] - previousBox[0], box[2] - previousBox[2]),
            config::minimumBoxRepeatSeparationMeters);
        EXPECT_GE(std::hypot(placement[0] - previousPlacement[0], placement[2] - previousPlacement[2]),
            config::minimumPlacementRepeatSeparationMeters);
        previousBox = box;
        previousPlacement = placement;

        const float boxRotation = sampler.SamplePlanarRotation();
        EXPECT_GE(boxRotation, -config::scenarioRotationLimitRadians);
        EXPECT_LE(boxRotation, config::scenarioRotationLimitRadians);
        negativeX = negativeX || box[0] < 0.0F;
        positiveX = positiveX || box[0] > 0.0F;
        negativeZ = negativeZ || box[2] < 0.0F;
        positiveZ = positiveZ || box[2] > 0.0F;
    }

    EXPECT_TRUE(negativeX);
    EXPECT_TRUE(positiveX);
    EXPECT_TRUE(negativeZ);
    EXPECT_TRUE(positiveZ);
}
