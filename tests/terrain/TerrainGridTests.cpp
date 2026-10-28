#include "terrain/MapBridges.h"
#include "terrain/MapBuildings.h"
#include "terrain/MapFlightRoute.h"
#include "terrain/MapRoads.h"
#include "terrain/MapTrees.h"
#include "terrain/MapWaterBodies.h"
#include "terrain/TerrainGrid.h"
#include "terrain/TrainingRangeTerrain.h"
#include "simulation_core/EoCameraModel.h"

#include <algorithm>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <string>

#include <glm/geometric.hpp>

namespace aerosimlab::terrain
{
void Verify_TrainingRangeTerrainUsesThePlannedSizeAndHasRelief()
{
    const TerrainGrid terrain = CreateTrainingRangeTerrain();

    EXPECT_EQ(terrain.sampleCount, 101U);
    EXPECT_DOUBLE_EQ(terrain.HalfWidthMeters(), 250.0);
    EXPECT_DOUBLE_EQ(terrain.HalfDepthMeters(), 200.0);
    EXPECT_EQ(terrain.heightOffsetsMeters.size(), 101U * 101U);
    EXPECT_NEAR(*std::min_element(terrain.heightOffsetsMeters.begin(),
        terrain.heightOffsetsMeters.end()), 0.0F, 1.0e-5F);
    EXPECT_GT(*std::max_element(terrain.heightOffsetsMeters.begin(),
        terrain.heightOffsetsMeters.end()), 30.0F);
}

void Verify_TerrainGridConvertsLocalCoordinatesAndInterpolatesHeight()
{
    TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.spacingXMeters = 10.0F;
    terrain.spacingYMeters = 20.0F;
    terrain.referenceElevationMeters = 100.0F;
    terrain.centerLatitudeDegrees = 37.0;
    terrain.centerLongitudeDegrees = -122.0;
    terrain.heightOffsetsMeters = {0.0F, 10.0F, 20.0F,
        10.0F, 20.0F, 30.0F,
        20.0F, 30.0F, 40.0F};

    EXPECT_DOUBLE_EQ(terrain.HalfWidthMeters(), 10.0);
    EXPECT_DOUBLE_EQ(terrain.HalfDepthMeters(), 20.0);
    EXPECT_DOUBLE_EQ(terrain.ToLocalMeters(37.0, -122.0).x, 0.0);
    const glm::dvec2 northOffset = terrain.ToLocalMeters(
        37.0 + 20.0 / 111320.0, -122.0);
    const glm::dvec2 eastOffset = terrain.ToLocalMeters(
        37.0, -122.0 + 20.0 / (111320.0 * std::cos(37.0 * std::numbers::pi / 180.0)));
    EXPECT_NEAR(northOffset.x, 0.0, 1.0e-6);
    EXPECT_NEAR(northOffset.y, 20.0, 0.1);
    EXPECT_NEAR(eastOffset.x, 20.0, 0.1);
    EXPECT_NEAR(eastOffset.y, 0.0, 1.0e-4);
    EXPECT_TRUE(terrain.ContainsLocalPositionMeters({10.0, 20.0}));
    EXPECT_FALSE(terrain.ContainsLocalPositionMeters({10.01, 0.0}));
    EXPECT_FLOAT_EQ(terrain.HeightAtLocalMeters({0.0, 0.0}), 120.0F);
    EXPECT_EQ(terrain.ToSimulationPositionMeters({5.0, -2.0, 3.0}),
        glm::dvec3(5.0, -2.0, 123.0));
}

void Verify_RenderedTerrainHeightUsesTheSameTriangleAsTheMesh()
{
    TerrainGrid terrain;
    terrain.sampleCount = 2U;
    terrain.spacingXMeters = 2.0F;
    terrain.spacingYMeters = 2.0F;
    terrain.referenceElevationMeters = 100.0F;
    terrain.heightOffsetsMeters = {0.0F, 10.0F, 10.0F, 0.0F};

    // 이 지점에서 두 대각선의 높이가 달라져 쌍선형 보간과 삼각형 보간을 구분할 수 있다.
    EXPECT_FLOAT_EQ(terrain.HeightAtLocalMeters({0.0, 0.0}), 105.0F);
    EXPECT_FLOAT_EQ(terrain.HeightAtRenderedSurfaceMeters({0.0, 0.0}), 110.0F);
    EXPECT_FLOAT_EQ(terrain.HeightAtRenderedSurfaceMeters({1.0, -1.0}), 100.0F);
}

void Verify_AsciiGridKeepsFractionalElevationsAndRoundTrips()
{
    const auto sourcePath = std::filesystem::temp_directory_path()
        / "dronesim-terrain-grid-source.asc";
    const auto convertedPath = std::filesystem::temp_directory_path()
        / "dronesim-terrain-grid-converted.aerotrn";
    {
        std::ofstream output(sourcePath);
        ASSERT_TRUE(output);
        output << "ncols 3\n"
            "nrows 3\n"
            "xllcorner 0\n"
            "yllcorner 0\n"
            "cellsize 2\n"
            "NODATA_value -9999\n"
            "101.25 101.50 101.75\n"
            "100.75 101.00 101.25\n"
            "100.25 100.50 100.75\n";
    }

    const TerrainGrid imported = TerrainGrid::LoadAsciiGrid(
        sourcePath, 37.49, 127.10, 100.5F);
    EXPECT_EQ(imported.sampleCount, 3U);
    EXPECT_FLOAT_EQ(imported.spacingXMeters, 2.0F);
    EXPECT_FLOAT_EQ(imported.referenceElevationMeters, 100.5F);
    EXPECT_FLOAT_EQ(imported.HeightAt(0U, 0U), 0.75F);
    EXPECT_FLOAT_EQ(imported.HeightAt(2U, 2U), 0.25F);

    imported.Save(convertedPath);
    const TerrainGrid restored = TerrainGrid::Load(convertedPath);
    EXPECT_FLOAT_EQ(restored.HeightAt(0U, 0U), 0.75F);
    EXPECT_FLOAT_EQ(restored.HeightAt(1U, 1U), 0.5F);
    EXPECT_FLOAT_EQ(restored.HeightAt(2U, 2U), 0.25F);
    EXPECT_NEAR(restored.centerLatitudeDegrees, 37.49, 1.0e-12);
    EXPECT_NEAR(restored.centerLongitudeDegrees, 127.10, 1.0e-12);

    std::error_code ignored;
    std::filesystem::remove(sourcePath, ignored);
    std::filesystem::remove(convertedPath, ignored);
}

void Verify_AsciiGridRejectsMissingHeightSamples()
{
    const auto path = std::filesystem::temp_directory_path()
        / "dronesim-terrain-grid-incomplete.asc";
    {
        std::ofstream output(path);
        ASSERT_TRUE(output);
        output << "ncols 3\nnrows 3\nxllcorner 0\nyllcorner 0\n"
            "cellsize 2\nNODATA_value -9999\n1 2 3\n4 5 6\n7 8\n";
    }

    EXPECT_THROW(static_cast<void>(TerrainGrid::LoadAsciiGrid(
            path, 37.49, 127.10, 100.5F)), std::runtime_error);
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void Verify_MapFlightRouteGoesAroundBuildingFootprints()
{
    TerrainGrid terrain;
    terrain.sampleCount = 41;
    terrain.spacingXMeters = 5.0F;
    terrain.spacingYMeters = 5.0F;
    terrain.referenceElevationMeters = 0.0F;
    terrain.heightOffsetsMeters.assign(41U * 41U, 0.0F);
    const std::vector<MapBuilding> buildings{{
        .osmId = "test-building",
        .outlineMeters = {{-5.0, -20.0}, {5.0, -20.0},
            {5.0, 20.0}, {-5.0, 20.0}},
        .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
        .heightMeters = 0.0}};

    const auto route = PlanMapFlightRoute(
        terrain, buildings, {-40.0, 0.0, 5.0}, {40.0, 0.0, 5.0});

    ASSERT_TRUE(route.has_value());
    ASSERT_GT(route->size(), 1U);
    EXPECT_EQ(route->back(), glm::dvec3(40.0, 0.0, 5.0));
    EXPECT_TRUE(std::any_of(route->begin(), route->end(), [](const glm::dvec3& point)
    {
        return std::abs(point.y) > 22.0;
    }));

    MapBuilding estimatedBuilding{
        .osmId = "estimated-building",
        .outlineMeters = {{-5.0, -20.0}, {5.0, -20.0},
            {5.0, 20.0}, {-5.0, 20.0}},
        .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
        .heightMeters = 28.0,
        .heightSource = BuildingHeightSource::EstimatedFromFootprint};
    const auto descendingRoute = PlanMapFlightRoute(
        terrain, {estimatedBuilding}, {-40.0, 0.0, 48.0},
        {40.0, 0.0, 5.0}, 3.0, 48.0);

    ASSERT_TRUE(descendingRoute.has_value());
    ASSERT_GE(descendingRoute->size(), 2U);
    EXPECT_EQ((*descendingRoute)[descendingRoute->size() - 2U],
        glm::dvec3(40.0, 0.0, 48.0));
    for (std::size_t segment = 1; segment < descendingRoute->size(); ++segment)
    {
        const glm::dvec3 start = (*descendingRoute)[segment - 1U];
        const glm::dvec3 end = (*descendingRoute)[segment];
        const std::size_t sampleCount = static_cast<std::size_t>(
            std::ceil(glm::length(end - start) / 0.25));
        for (std::size_t sample = 0; sample <= sampleCount; ++sample)
        {
            const double fraction = sampleCount > 0U
                ? static_cast<double>(sample) / static_cast<double>(sampleCount)
                : 0.0;
            EXPECT_FALSE(ContainsMapBuildingVolume(estimatedBuilding,
                glm::mix(start, end, fraction)))
                << "segment=" << segment << " sample=" << sample;
        }
    }
}

void Verify_MapFlightRouteReportsABuildingWallWithoutAnOpening()
{
    TerrainGrid terrain;
    terrain.sampleCount = 41;
    terrain.spacingXMeters = 5.0F;
    terrain.spacingYMeters = 5.0F;
    terrain.referenceElevationMeters = 0.0F;
    terrain.heightOffsetsMeters.assign(41U * 41U, 0.0F);
    const std::vector<MapBuilding> buildings{{
        .osmId = "blocking-wall",
        .outlineMeters = {{-5.0, -110.0}, {5.0, -110.0},
            {5.0, 110.0}, {-5.0, 110.0}},
        .groundElevationsMeters = {0.0, 0.0, 0.0, 0.0},
        .heightMeters = 12.0}};

    const auto route = PlanMapFlightRoute(
        terrain, buildings, {-40.0, 0.0, 5.0}, {40.0, 0.0, 5.0});

    EXPECT_FALSE(route.has_value());
}

void Verify_MapBuildingFacadeFollowsASlopedFootprintEdge()
{
    const MapBuilding building{
        .outlineMeters = {{0.0, 0.0}, {4.0, 0.0}, {1.0, 3.0}},
        .groundElevationsMeters = {10.0, 20.0, 30.0},
        .heightMeters = 6.0};

    const MapBuildingFacade facade = GetNearestMapBuildingFacade(
        building, {3.0, 5.0});

    EXPECT_NEAR(facade.centerMeters.x, 2.5, 1.0e-9);
    EXPECT_NEAR(facade.centerMeters.y, 1.5, 1.0e-9);
    EXPECT_NEAR(facade.centerMeters.z, 28.0, 1.0e-9);
    EXPECT_NEAR(facade.widthMeters, std::sqrt(18.0), 1.0e-9);
    EXPECT_NEAR(facade.heightMeters, 6.0, 1.0e-9);
    EXPECT_GT(glm::dot(facade.outwardNormal,
        glm::dvec3{2.0, 2.0, 0.0}), 0.0);
}

void Verify_BridgeOutsideTerrainIsSkipped()
{
    TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.spacingXMeters = 10.0F;
    terrain.spacingYMeters = 10.0F;
    terrain.centerLatitudeDegrees = 37.0;
    terrain.centerLongitudeDegrees = 127.0;
    terrain.referenceElevationMeters = 0.0F;
    terrain.heightOffsetsMeters.assign(9U, 0.0F);

    const auto path = std::filesystem::temp_directory_path()
        / "dronesim-bridge-outside-terrain-test.geojson";
    {
        std::ofstream output(path);
        ASSERT_TRUE(output);
        output << R"json({"features":[{"geometry":{"type":"Polygon","coordinates":[[[128.0,38.0],[128.001,38.0],[128.001,38.001],[128.0,38.0]]]},"properties":{"osm_id":"outside","tags":{"man_made":"bridge","name":"시험 교량"}}}]})json";
    }

    const std::vector<MapBridgeFootprint> bridges =
        LoadMapBridgeFootprints(path, terrain);
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    EXPECT_TRUE(bridges.empty());
}

TEST(TerrainGridTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
        Verify_TrainingRangeTerrainUsesThePlannedSizeAndHasRelief());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TerrainGridConvertsLocalCoordinatesAndInterpolatesHeight());
    EXPECT_NO_FATAL_FAILURE(Verify_RenderedTerrainHeightUsesTheSameTriangleAsTheMesh());
    EXPECT_NO_FATAL_FAILURE(Verify_AsciiGridKeepsFractionalElevationsAndRoundTrips());
    EXPECT_NO_FATAL_FAILURE(Verify_AsciiGridRejectsMissingHeightSamples());
    EXPECT_NO_FATAL_FAILURE(Verify_MapFlightRouteGoesAroundBuildingFootprints());
    EXPECT_NO_FATAL_FAILURE(Verify_MapFlightRouteReportsABuildingWallWithoutAnOpening());
    EXPECT_NO_FATAL_FAILURE(Verify_MapBuildingFacadeFollowsASlopedFootprintEdge());
    EXPECT_NO_FATAL_FAILURE(Verify_BridgeOutsideTerrainIsSkipped());
}

}
