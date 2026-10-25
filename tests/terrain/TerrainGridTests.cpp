#include "terrain/MapBuildings.h"
#include "terrain/MapRoads.h"
#include "terrain/TerrainGrid.h"

#include <algorithm>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

namespace aerosimlab::terrain
{
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

    EXPECT_DOUBLE_EQ(terrain.ToLocalMeters(37.0, -122.0).x, 0.0);
    EXPECT_NEAR(terrain.ToLocalMeters(37.0 + 20.0 / 111320.0, -122.0).y,
        20.0, 0.02);
    EXPECT_TRUE(terrain.ContainsLocalPositionMeters({10.0, 20.0}));
    EXPECT_FALSE(terrain.ContainsLocalPositionMeters({10.01, 0.0}));
    EXPECT_FLOAT_EQ(terrain.HeightAtLocalMeters({0.0, 0.0}), 120.0F);
}

void Verify_TancheonTerrainAssetLoadsAtTheRecordedOrigin()
{
    const auto path = std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/tancheon/terrain.aerotrn";
    const TerrainGrid terrain = TerrainGrid::Load(path);

    EXPECT_EQ(terrain.sampleCount, 41U);
    EXPECT_FLOAT_EQ(terrain.spacingXMeters, 30.0F);
    EXPECT_FLOAT_EQ(terrain.spacingYMeters, 30.0F);
    EXPECT_NEAR(terrain.centerLatitudeDegrees, 37.4936231, 1.0e-7);
    EXPECT_NEAR(terrain.centerLongitudeDegrees, 127.1018462, 1.0e-7);
    EXPECT_NEAR(terrain.HeightAtLocalMeters({0.0, 0.0}), 18.0F, 0.01F);
}

void Verify_TancheonBuildingsUseMappedFootprintsAndMarkedHeightEstimates()
{
    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const std::vector<MapBuilding> buildings = LoadMapBuildings(
        root / "assets/maps/tancheon/site.geojson", terrain);

    std::size_t taggedHeightCount = 0U;
    std::size_t estimatedHeightCount = 0U;
    for (const MapBuilding& building : buildings)
    {
        EXPECT_FALSE(building.osmId.empty());
        EXPECT_GE(building.outlineMeters.size(), 3U);
        EXPECT_EQ(building.outlineMeters.size(), building.groundElevationsMeters.size());
        EXPECT_GT(building.heightMeters, 0.0);
        if (building.heightSource == BuildingHeightSource::EstimatedFromLevels)
            ++estimatedHeightCount;
        else
            ++taggedHeightCount;
    }
    EXPECT_GE(taggedHeightCount, 50U);
    EXPECT_GE(estimatedHeightCount, 10U);
}

void Verify_TancheonRoadsKeepBridgeTagsAndMarkEstimatedWidths()
{
    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const std::vector<MapRoad> roads = LoadMapRoads(
        root / "assets/maps/tancheon/site.geojson", terrain);

    std::size_t bridgeCount = 0U;
    bool foundTancheonBridge = false;
    for (const MapRoad& road : roads)
    {
        EXPECT_FALSE(road.osmId.empty());
        EXPECT_GE(road.centerlineMeters.size(), 2U);
        EXPECT_GT(road.widthMeters, 0.0);
        if (road.bridge)
        {
            ++bridgeCount;
            foundTancheonBridge = foundTancheonBridge || road.name == "탄천교";
        }
    }

    EXPECT_GT(roads.size(), 100U);
    EXPECT_GT(bridgeCount, 0U);
    EXPECT_TRUE(foundTancheonBridge);
    EXPECT_NE(std::find_if(roads.begin(), roads.end(), [](const MapRoad& road)
        { return road.widthSource == RoadWidthSource::EstimatedFromRoadType; }),
        roads.end());
}

TEST(TerrainGridTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
        Verify_TerrainGridConvertsLocalCoordinatesAndInterpolatesHeight());
    EXPECT_NO_FATAL_FAILURE(Verify_TancheonTerrainAssetLoadsAtTheRecordedOrigin());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonBuildingsUseMappedFootprintsAndMarkedHeightEstimates());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonRoadsKeepBridgeTagsAndMarkEstimatedWidths());
}

}
