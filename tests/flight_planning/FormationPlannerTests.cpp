#include "flight_planning/FormationPlanner.h"
#include "flight_planning/BuildingObstacle.h"
#include "flight_planning/RouteEnvironment.h"

#include "../terrain/TestTerrain.h"

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

namespace dronesim::flight_planning
{
namespace
{
class TestRouteEnvironment final : public RouteEnvironment
{
public:
    TestRouteEnvironment(const terrain::TerrainGrid& terrain,
        const std::vector<BuildingObstacle>& buildings)
        : terrain_(terrain)
    {
        for (const auto& building : buildings)
        {
            const double ground = building.groundElevationsMeters.empty()
                ? 0.0
                : *std::max_element(building.groundElevationsMeters.begin(),
                    building.groundElevationsMeters.end());
            obstacles_.push_back({building.outlineMeters,
                ground + building.heightMeters});
        }
    }

    glm::dvec4 BoundsMeters() const noexcept override
    {
        return {-terrain_.HalfWidthMeters(), -terrain_.HalfDepthMeters(),
            terrain_.HalfWidthMeters(), terrain_.HalfDepthMeters()};
    }
    bool Contains(glm::dvec2 position) const noexcept override
    {
        return terrain_.ContainsLocalPositionMeters(position);
    }
    double GroundAltitudeMeters(glm::dvec2 position) const override
    {
        return terrain_.HeightAtLocalMeters(position);
    }
    std::span<const RouteObstacle> Obstacles() const noexcept override
    {
        return obstacles_;
    }

private:
    const terrain::TerrainGrid& terrain_;
    std::vector<RouteObstacle> obstacles_;
};
}

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

    const FormationRoutePlan route{
        .leaderStartMeters = {0.0, 0.0, 0.0},
        .leaderWaypointsMeters = {
            {0.0, 2.0, 0.0}, {0.0, 5.0, 0.0}, {3.0, 5.0, 0.0}},
        .spacingMeters = 2.0};
    EXPECT_NEAR(route.HeadingAtWaypoint(0U, -1.0), glm::half_pi<double>(), 1.0e-12);
    EXPECT_NEAR(route.HeadingAtWaypoint(2U, -1.0), 0.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(route.HeadingAtWaypoint(3U, 0.75), 0.75);

    const auto targets = route.TargetsAtWaypoint(
        0U, 3U, {10.0, 20.0, 5.0}, 0.0);
    ASSERT_EQ(targets.size(), 3U);
    EXPECT_EQ(targets[0], glm::dvec3(0.0, 2.0, 0.0));
    EXPECT_EQ(targets[1], glm::dvec3(11.0, 18.0, 5.0));
    EXPECT_EQ(targets[2], glm::dvec3(9.0, 18.0, 5.0));
}

TEST(FormationPlannerTests, RejectsNarrowRouteInsteadOfChangingFormation)
{
    const auto openTerrain = terrain::MakeHillyTerrainForTest();
    const TestRouteEnvironment openEnvironment(openTerrain, {});
    const auto openRoute = PlanFormationRoute(openEnvironment,
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
    const TestRouteEnvironment narrowEnvironment(narrowTerrain, walls);
    EXPECT_FALSE(PlanFormationRoute(narrowEnvironment,
        {-200.0, 0.0, 10.0}, {200.0, 0.0, 10.0}, 4U,
        8.0, 1.5, 2.0, 10.0));
}
}
