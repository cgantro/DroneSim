#include "terrain/MapBridges.h"
#include "terrain/MapBuildings.h"
#include "terrain/MapRoads.h"
#include "terrain/MapWaterBodies.h"
#include "terrain/TerrainGrid.h"
#include "simulation_core/EoCameraModel.h"

#include <algorithm>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include <glm/geometric.hpp>

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
        const MapBuildingBoundsMeters bounds = GetMapBuildingBoundsMeters(building);
        EXPECT_GT(bounds.halfExtentsMeters.x, 0.0);
        EXPECT_GT(bounds.halfExtentsMeters.y, 0.0);
        EXPECT_GT(bounds.halfExtentsMeters.z, 0.0);
        if (building.heightSource == BuildingHeightSource::EstimatedFromLevels)
            ++estimatedHeightCount;
        else
            ++taggedHeightCount;
    }
    EXPECT_GE(taggedHeightCount, 50U);
    EXPECT_GE(estimatedHeightCount, 10U);
}

void Verify_TancheonBuildingBoundsBlockCameraViewThroughTheMappedBuilding()
{
    using namespace aerosimlab::simulation;

    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const std::vector<MapBuilding> buildings = LoadMapBuildings(
        root / "assets/maps/tancheon/site.geojson", terrain);
    ASSERT_FALSE(buildings.empty());

    const MapBuildingBoundsMeters bounds = GetMapBuildingBoundsMeters(buildings.front());
    const double cameraOffsetMeters = bounds.halfExtentsMeters.x + 10.0;
    CameraState camera;
    camera.positionMeters = {
        bounds.centerMeters.x - cameraOffsetMeters,
        bounds.centerMeters.y, bounds.centerMeters.z};
    camera.frameCount = 1;
    InspectionSurface surface;
    surface.centerMeters = {
        bounds.centerMeters.x + cameraOffsetMeters,
        bounds.centerMeters.y, bounds.centerMeters.z};
    surface.normal = {-1.0, 0.0, 0.0};
    surface.widthAxis = {0.0, 1.0, 0.0};
    surface.widthMeters = 2.0;
    surface.heightMeters = 2.0;
    surface.minimumDistanceMeters = 1.0;
    surface.maximumDistanceMeters = 1000.0;
    surface.maximumIncidenceAngleDegrees = 90.0;
    surface.minimumPixelWidth = 1;
    surface.minimumPixelHeight = 1;
    const InspectionOccluderBox buildingBox{
        bounds.centerMeters, bounds.halfExtentsMeters};

    EXPECT_EQ(EoCameraModel::AssessCapture(camera, surface), CaptureAssessment::Ready);
    EXPECT_EQ(EoCameraModel::AssessCapture(camera, surface,
        std::span<const InspectionOccluderBox>{&buildingBox, 1}),
        CaptureAssessment::Occluded);
}

void Verify_TancheonBuildingFacadeFacesTheNearestApproachSide()
{
    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const std::vector<MapBuilding> buildings = LoadMapBuildings(
        root / "assets/maps/tancheon/site.geojson", terrain);
    ASSERT_FALSE(buildings.empty());

    const MapBuilding& building = buildings.front();
    const MapBuildingBoundsMeters bounds = GetMapBuildingBoundsMeters(building);
    const glm::dvec2 approachPosition{
        bounds.centerMeters.x - bounds.halfExtentsMeters.x - 5.0,
        bounds.centerMeters.y};
    const MapBuildingFacade facade = GetNearestMapBuildingFacade(building,
        approachPosition);

    bool liesOnFootprintEdge = false;
    for (std::size_t index = 0; index < building.outlineMeters.size(); ++index)
    {
        const glm::dvec2& start = building.outlineMeters[index];
        const glm::dvec2 edge = building.outlineMeters[
            (index + 1U) % building.outlineMeters.size()] - start;
        const double edgeLengthSquared = glm::dot(edge, edge);
        if (edgeLengthSquared <= 1.0e-12)
            continue;
        const glm::dvec2 point = glm::dvec2{
            facade.centerMeters.x, facade.centerMeters.y} - start;
        const double fraction = glm::dot(point, edge) / edgeLengthSquared;
        const glm::dvec2 closestPoint = start + edge * std::clamp(fraction, 0.0, 1.0);
        if (glm::length(closestPoint - glm::dvec2{
            facade.centerMeters.x, facade.centerMeters.y}) < 1.0e-6)
            liesOnFootprintEdge = true;
    }
    EXPECT_TRUE(liesOnFootprintEdge);
    EXPECT_GT(glm::dot(facade.outwardNormal,
        glm::dvec3{approachPosition.x - facade.centerMeters.x,
            approachPosition.y - facade.centerMeters.y, 0.0}), 0.0);
    EXPECT_GT(facade.widthMeters, 0.0);
    EXPECT_GT(facade.heightMeters, 0.0);
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

void Verify_TancheonRiverUsesOSMWaterBoundaryAndKeepsItsIslands()
{
    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const auto waterBodies = LoadMapWaterBodies(
        root / "assets/maps/tancheon/site.geojson", terrain);

    ASSERT_EQ(waterBodies.size(), 1U);
    EXPECT_GT(waterBodies.front().outerBoundaryMeters.size(), 1000U);
    EXPECT_EQ(waterBodies.front().innerBoundariesMeters.size(), 25U);
    const glm::dvec2 riverPoint = terrain.ToLocalMeters(37.4932057, 127.1025281);
    EXPECT_TRUE(ContainsWater(waterBodies.front(), riverPoint));
}

void Verify_TancheonBridgeFootprintsKeepNamesAndGroundProjection()
{
    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const std::vector<MapBridgeFootprint> bridges = LoadMapBridgeFootprints(
        root / "assets/maps/tancheon/site.geojson", terrain);

    std::vector<std::string> names;
    for (const MapBridgeFootprint& bridge : bridges)
    {
        EXPECT_FALSE(bridge.osmId.empty());
        EXPECT_FALSE(bridge.name.empty());
        EXPECT_GE(bridge.outlineMeters.size(), 3U);
        EXPECT_EQ(bridge.outlineMeters.size(), bridge.groundElevationsMeters.size());
        EXPECT_GT(glm::length(bridge.outlineMeters.front()
            - bridge.outlineMeters.back()), 0.01);
        for (const glm::dvec2& point : bridge.outlineMeters)
            EXPECT_TRUE(terrain.ContainsLocalPositionMeters(point));
        names.push_back(bridge.name);
    }

    EXPECT_EQ(bridges.size(), 3U);
    EXPECT_NE(std::find(names.begin(), names.end(), "탄천교"), names.end());
    EXPECT_NE(std::find(names.begin(), names.end(), "수서IC교"), names.end());
    EXPECT_NE(std::find(names.begin(), names.end(), "광평교"), names.end());
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
        Verify_TerrainGridConvertsLocalCoordinatesAndInterpolatesHeight());
    EXPECT_NO_FATAL_FAILURE(Verify_TancheonTerrainAssetLoadsAtTheRecordedOrigin());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonBuildingsUseMappedFootprintsAndMarkedHeightEstimates());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonBuildingBoundsBlockCameraViewThroughTheMappedBuilding());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonBuildingFacadeFacesTheNearestApproachSide());
    EXPECT_NO_FATAL_FAILURE(Verify_MapBuildingFacadeFollowsASlopedFootprintEdge());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonRoadsKeepBridgeTagsAndMarkEstimatedWidths());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonRiverUsesOSMWaterBoundaryAndKeepsItsIslands());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonBridgeFootprintsKeepNamesAndGroundProjection());
    EXPECT_NO_FATAL_FAILURE(Verify_BridgeOutsideTerrainIsSkipped());
}

}
