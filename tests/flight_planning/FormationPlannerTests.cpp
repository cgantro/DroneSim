#include "flight_planning/FormationPlanner.h"

#include "../terrain/TestTerrain.h"

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>
#include <glm/geometric.hpp>

#include <cmath>
#include <vector>

namespace dronesim::flight_planning
{
TEST(FormationPlannerTests, PlacesFourDronesBehindLeaderAndRotatesWithRoute)
{
    const auto offsets = FormationOffsets(4U, 8.0, glm::half_pi<double>());

    ASSERT_EQ(offsets.size(), 4U);
    EXPECT_NEAR(glm::length(offsets[0]), 0.0, 1.0e-12);
    EXPECT_NEAR(offsets[1].x, 4.0, 1.0e-12);
    EXPECT_NEAR(offsets[1].y, -8.0, 1.0e-12);
    EXPECT_NEAR(offsets[2].x, -4.0, 1.0e-12);
    EXPECT_NEAR(offsets[2].y, -8.0, 1.0e-12);
    EXPECT_NEAR(offsets[3].x, 0.0, 1.0e-12);
    EXPECT_NEAR(offsets[3].y, -16.0, 1.0e-12);
}

TEST(FormationPlannerTests, RejectsNarrowRouteInsteadOfChangingFormation)
{
    const auto openTerrain = terrain::MakeHillyTerrainForTest();
    const auto openRoute = PlanFormationRoute(openTerrain, {},
        {-205.0, -155.0, 4.0}, {176.0, -148.0, 18.0}, 4U,
        8.0, 1.0, 2.0, 18.0);

    ASSERT_TRUE(openRoute.has_value());
    EXPECT_DOUBLE_EQ(openRoute->spacingMeters, 8.0);
    EXPECT_GT(openRoute->requiredClearanceMeters, 10.0);
    ASSERT_GE(openRoute->leaderWaypointsMeters.size(), 2U);
    EXPECT_EQ(openRoute->leaderWaypointsMeters.back(), glm::dvec3(176.0, -148.0, 18.0));

    const auto narrowTerrain = terrain::MakeHillyTerrainForTest();
    const double halfWidth = narrowTerrain.HalfWidthMeters();
    const double halfDepth = narrowTerrain.HalfDepthMeters();
    constexpr double corridorHalfWidthMeters = 17.0;
    const std::vector<BuildingObstacle> walls{
        {
            .name = "south-wall",
            .outlineMeters = {{-halfWidth, -halfDepth}, {halfWidth, -halfDepth},
                {halfWidth, -corridorHalfWidthMeters},
                {-halfWidth, -corridorHalfWidthMeters}},
            .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
            .heightMeters = 100.0},
        {
            .name = "north-wall",
            .outlineMeters = {{-halfWidth, corridorHalfWidthMeters},
                {halfWidth, corridorHalfWidthMeters},
                {halfWidth, halfDepth}, {-halfWidth, halfDepth}},
            .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
            .heightMeters = 100.0}
    };

    // V자 대형이 지나갈 수 없으면 간격이나 모양을 임의로 바꾸지 않는다.
    EXPECT_FALSE(PlanFormationRoute(narrowTerrain, walls,
        {-200.0, 0.0, 10.0}, {200.0, 0.0, 10.0}, 4U,
        8.0, 1.5, 2.0, 10.0));
}
}
