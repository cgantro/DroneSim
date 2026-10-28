#include "simulation_core/FormationPlanner.h"

#include "terrain/TrainingRangeTerrain.h"

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace aerosimlab::simulation
{
TEST(FormationPlannerTests, PlacesFourDronesBehindLeaderAndRotatesWithRoute)
{
    const auto offsets = FormationOffsets(
        FormationShape::Vee, 4U, 8.0, glm::half_pi<double>());

    ASSERT_EQ(offsets.size(), 4U);
    EXPECT_NEAR(glm::length(offsets[0]), 0.0, 1.0e-12);
    EXPECT_NEAR(offsets[1].x, 4.0, 1.0e-12);
    EXPECT_NEAR(offsets[1].y, -8.0, 1.0e-12);
    EXPECT_NEAR(offsets[2].x, -4.0, 1.0e-12);
    EXPECT_NEAR(offsets[2].y, -8.0, 1.0e-12);
    EXPECT_NEAR(offsets[3].x, 0.0, 1.0e-12);
    EXPECT_NEAR(offsets[3].y, -16.0, 1.0e-12);
}

TEST(FormationPlannerTests, PlansFormationClearanceAndCompactColumnFallback)
{
    const auto openTerrain = terrain::CreateTrainingRangeTerrain();
    const auto openRoute = PlanFormationRoute(openTerrain, {},
        {-205.0, -155.0, 4.0}, {176.0, -148.0, 18.0}, 4U,
        FormationShape::Vee, 8.0, 1.0, 2.0, 18.0);

    ASSERT_TRUE(openRoute.has_value());
    EXPECT_EQ(openRoute->shape, FormationShape::Vee);
    EXPECT_DOUBLE_EQ(openRoute->spacingMeters, 8.0);
    EXPECT_GT(openRoute->requiredClearanceMeters, 10.0);
    ASSERT_GE(openRoute->leaderWaypointsMeters.size(), 2U);
    EXPECT_EQ(openRoute->leaderWaypointsMeters.back(), glm::dvec3(176.0, -148.0, 18.0));

    const auto narrowTerrain = terrain::CreateTrainingRangeTerrain();
    const double halfWidth = narrowTerrain.HalfWidthMeters();
    const double halfDepth = narrowTerrain.HalfDepthMeters();
    constexpr double corridorHalfWidthMeters = 17.0;
    const std::vector<terrain::MapBuilding> walls{
        {
            .osmId = "south-wall",
            .outlineMeters = {{-halfWidth, -halfDepth}, {halfWidth, -halfDepth},
                {halfWidth, -corridorHalfWidthMeters},
                {-halfWidth, -corridorHalfWidthMeters}},
            .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
            .heightMeters = 100.0,
            .heightSource = terrain::BuildingHeightSource::OsmHeightTag},
        {
            .osmId = "north-wall",
            .outlineMeters = {{-halfWidth, corridorHalfWidthMeters},
                {halfWidth, corridorHalfWidthMeters},
                {halfWidth, halfDepth}, {-halfWidth, halfDepth}},
            .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
            .heightMeters = 100.0,
            .heightSource = terrain::BuildingHeightSource::OsmHeightTag}
    };

    const auto columnRoute = PlanFormationRoute(narrowTerrain, walls,
        {-200.0, 0.0, 10.0}, {200.0, 0.0, 10.0}, 4U,
        FormationShape::Vee, 8.0, 1.5, 2.0, 10.0);

    ASSERT_TRUE(columnRoute.has_value());
    EXPECT_EQ(columnRoute->shape, FormationShape::Column);
    EXPECT_DOUBLE_EQ(columnRoute->spacingMeters, 4.0);
    EXPECT_LT(columnRoute->requiredClearanceMeters, 17.0);
    ASSERT_FALSE(columnRoute->leaderWaypointsMeters.empty());
    EXPECT_EQ(columnRoute->leaderWaypointsMeters.back(), glm::dvec3(200.0, 0.0, 10.0));

    const std::vector<terrain::MapBuilding> centralWalls{
        {
            .osmId = "central-south-wall",
            .outlineMeters = {{-40.0, -halfDepth}, {40.0, -halfDepth},
                {40.0, -corridorHalfWidthMeters},
                {-40.0, -corridorHalfWidthMeters}},
            .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
            .heightMeters = 100.0,
            .heightSource = terrain::BuildingHeightSource::OsmHeightTag},
        {
            .osmId = "central-north-wall",
            .outlineMeters = {{-40.0, corridorHalfWidthMeters},
                {40.0, corridorHalfWidthMeters}, {40.0, halfDepth},
                {-40.0, halfDepth}},
            .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
            .heightMeters = 100.0,
            .heightSource = terrain::BuildingHeightSource::OsmHeightTag}
    };
    const auto localTransitionRoute = PlanFormationRoute(openTerrain, centralWalls,
        {-200.0, 0.0, 10.0}, {200.0, 0.0, 10.0}, 4U,
        FormationShape::Vee, 8.0, 1.5, 2.0, 10.0);

    ASSERT_TRUE(localTransitionRoute.has_value());
    ASSERT_GE(localTransitionRoute->profile.size(), 4U);
    EXPECT_EQ(localTransitionRoute->profile.front().shape, FormationShape::Vee);
    EXPECT_TRUE(IsFormationTransitionAtDistance(*localTransitionRoute,
        localTransitionRoute->profile[1].distanceMeters + 1.0));

    double distanceAtCorridorCenterMeters = 0.0;
    double traveledMeters = 0.0;
    double closestXDistanceMeters = std::numeric_limits<double>::infinity();
    glm::dvec3 previous = localTransitionRoute->leaderStartMeters;
    for (const glm::dvec3& current : localTransitionRoute->leaderWaypointsMeters)
    {
        const double segmentLength = glm::distance(previous, current);
        const double fraction = std::abs(current.x - previous.x) > 1.0e-9
            ? std::clamp(-previous.x / (current.x - previous.x), 0.0, 1.0)
            : 0.0;
        const double closestDistance = std::abs(std::lerp(previous.x, current.x,
            fraction));
        if (closestDistance < closestXDistanceMeters)
        {
            closestXDistanceMeters = closestDistance;
            distanceAtCorridorCenterMeters = traveledMeters
                + segmentLength * fraction;
        }
        traveledMeters += segmentLength;
        previous = current;
    }
    const auto compactOffsets = FormationOffsetsAlongRoute(
        *localTransitionRoute, 4U, 0.0, distanceAtCorridorCenterMeters);
    ASSERT_EQ(compactOffsets.size(), 4U);
    EXPECT_NEAR(glm::length(compactOffsets.back()), 12.0, 1.0e-6);
    EXPECT_FALSE(IsFormationTransitionAtDistance(*localTransitionRoute,
        distanceAtCorridorCenterMeters));

    const double totalRouteLengthMeters = traveledMeters;
    const auto restoredOffsets = FormationOffsetsAlongRoute(
        *localTransitionRoute, 4U, 0.0, totalRouteLengthMeters);
    EXPECT_NEAR(glm::length(restoredOffsets.back()), 16.0, 1.0e-6);
}
}
