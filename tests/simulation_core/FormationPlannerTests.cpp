#include "simulation_core/FormationPlanner.h"

#include "terrain/TrainingRangeTerrain.h"

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>
#include <glm/geometric.hpp>

#include <cmath>

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

TEST(FormationPlannerTests, ExpandsObstacleClearanceToCoverTheWholeFormation)
{
    const auto terrain = terrain::CreateTrainingRangeTerrain();
    const auto route = PlanFormationRoute(terrain, {},
        {-205.0, -155.0, 4.0}, {176.0, -148.0, 18.0}, 4U,
        FormationShape::Vee, 8.0, 1.0, 2.0, 18.0);

    ASSERT_TRUE(route.has_value());
    EXPECT_EQ(route->shape, FormationShape::Vee);
    EXPECT_GT(route->requiredClearanceMeters, 10.0);
    ASSERT_GE(route->leaderWaypointsMeters.size(), 2U);
    EXPECT_EQ(route->leaderWaypointsMeters.back(), glm::dvec3(176.0, -148.0, 18.0));
}
}
