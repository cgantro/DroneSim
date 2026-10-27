#include "terrain/MapBridges.h"
#include "terrain/MapBuildings.h"
#include "terrain/MapFlightRoute.h"
#include "terrain/MapRoads.h"
#include "terrain/MapTrees.h"
#include "terrain/MapWaterBodies.h"
#include "terrain/TerrainGrid.h"
#include "simulation_core/EoCameraModel.h"

#include <algorithm>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>

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
}

void Verify_TancheonInspectionRouteAvoidsAllMappedBuildingFootprints()
{
    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const auto buildings = LoadMapBuildingFootprints(
        root / "assets/maps/tancheon/site.geojson", terrain);
    const auto visibleBuildings = LoadMapBuildings(
        root / "assets/maps/tancheon/site.geojson", terrain);
    ASSERT_GT(buildings.size(), visibleBuildings.size());
    const auto estimatedFootprint = std::find_if(buildings.begin(), buildings.end(),
        [](const MapBuilding& building)
        {
            return building.heightSource == BuildingHeightSource::EstimatedFromFootprint;
        });
    ASSERT_NE(estimatedFootprint, buildings.end());
    EXPECT_DOUBLE_EQ(estimatedFootprint->heightMeters,
        estimatedFootprintBuildingHeightMeters);
    const auto estimatedBounds = GetMapBuildingBoundsMeters(*estimatedFootprint);
    EXPECT_TRUE(ContainsMapBuildingVolume(*estimatedFootprint,
        {estimatedBounds.centerMeters.x, estimatedBounds.centerMeters.y,
            estimatedBounds.centerMeters.z}));
    EXPECT_FALSE(ContainsMapBuildingVolume(*estimatedFootprint,
        {estimatedBounds.centerMeters.x, estimatedBounds.centerMeters.y,
            estimatedBounds.centerMeters.z + estimatedBounds.halfExtentsMeters.z + 0.1}));

    const std::array<std::string, 3> targetBuildingIds{
        "692770099", "692770087", "692770092"};
    const glm::dvec2 demoStart{-410.0, 457.0};
    const glm::dvec3 demoStartPosition{demoStart,
        terrain.HeightAtLocalMeters(demoStart) + 0.35};
    EXPECT_FALSE(std::any_of(buildings.begin(), buildings.end(),
        [&demoStartPosition](const MapBuilding& building)
        {
            return ContainsMapBuildingVolume(building, demoStartPosition);
        }));
    glm::dvec3 routeStart = demoStartPosition;
    for (const std::string& targetId : targetBuildingIds)
    {
        const auto target = std::find_if(visibleBuildings.begin(), visibleBuildings.end(),
            [&targetId](const MapBuilding& building)
            { return building.osmId == targetId; });
        ASSERT_NE(target, visibleBuildings.end());
        const auto facadeRoute = PlanMapFlightRouteToFacade(
            terrain, buildings, *target, routeStart, 8.0, 20.0);
        ASSERT_TRUE(facadeRoute.has_value()) << targetId
            << " start=" << routeStart.x << "," << routeStart.y
            << " facade search found no reachable approach";
        ASSERT_FALSE(facadeRoute->waypointsMeters.empty()) << targetId;
        EXPECT_EQ(facadeRoute->waypointsMeters.back(),
            facadeRoute->approachPositionMeters);
        if (routeStart.z < estimatedFootprintBuildingHeightMeters + 2.0)
        {
            EXPECT_EQ(facadeRoute->waypointsMeters.front().x, routeStart.x);
            EXPECT_EQ(facadeRoute->waypointsMeters.front().y, routeStart.y);
            EXPECT_GE(facadeRoute->waypointsMeters.front().z,
                estimatedFootprintBuildingHeightMeters + 2.0);
        }
        routeStart = facadeRoute->approachPositionMeters;
    }

    const auto bridges = LoadMapBridgeFootprints(
        root / "assets/maps/tancheon/site.geojson", terrain);
    const auto tancheonBridge = std::find_if(bridges.begin(), bridges.end(),
        [](const MapBridgeFootprint& bridge) { return bridge.name == "탄천교"; });
    ASSERT_NE(tancheonBridge, bridges.end());
    glm::dvec2 bridgeCenter{0.0};
    double bridgeGround = 0.0;
    for (std::size_t index = 0; index < tancheonBridge->outlineMeters.size(); ++index)
    {
        bridgeCenter += tancheonBridge->outlineMeters[index];
        bridgeGround += tancheonBridge->groundElevationsMeters[index];
    }
    bridgeCenter /= static_cast<double>(tancheonBridge->outlineMeters.size());
    bridgeGround /= static_cast<double>(tancheonBridge->groundElevationsMeters.size());
    const double centerGround = terrain.HeightAtLocalMeters({0.0, 0.0});
    const double bridgeDeckBottom = bridgeGround + 6.0;
    const double bridgeDeckTop = bridgeDeckBottom + 0.8;
    const glm::dvec3 bridgeApproach = terrain.ToSimulationPositionMeters(
        {bridgeCenter.x, bridgeCenter.y,
            bridgeGround - centerGround + 4.8});
    const auto bridgePiers = EstimateMapBridgePiers(
        *tancheonBridge, terrain, bridgeDeckBottom);
    constexpr double cameraTransitAltitudeMeters = 60.0;
    const auto bridgeRoute = PlanMapFlightRouteUnderBridge(
        terrain, buildings, *tancheonBridge, bridgePiers,
        routeStart, bridgeApproach, bridgeDeckBottom, bridgeDeckTop, 3.0,
        cameraTransitAltitudeMeters);
    ASSERT_TRUE(bridgeRoute.has_value())
        << "탄천교 하부 routeStart=" << routeStart.x << "," << routeStart.y
        << "," << routeStart.z << " goal=" << bridgeApproach.x << ","
        << bridgeApproach.y << "," << bridgeApproach.z;
    EXPECT_TRUE(std::any_of(bridgeRoute->begin(), bridgeRoute->end(),
        [](const glm::dvec3& waypoint)
        {
            return waypoint.z >= cameraTransitAltitudeMeters;
        }));
    std::vector<MapBuilding> bridgeCollisionShapes{{
        .outlineMeters = tancheonBridge->outlineMeters,
        .groundElevationsMeters = std::vector<double>(
            tancheonBridge->outlineMeters.size(), bridgeDeckBottom),
        .heightMeters = bridgeDeckTop - bridgeDeckBottom}};
    for (const auto& pier : bridgePiers)
        bridgeCollisionShapes.push_back({
            .outlineMeters = pier.outlineMeters,
            .groundElevationsMeters = std::vector<double>(
                pier.outlineMeters.size(), pier.bottomElevationMeters),
            .heightMeters = pier.topElevationMeters - pier.bottomElevationMeters});
    bool routeHitsBridgeStructure = false;
    glm::dvec3 previous = routeStart;
    for (const glm::dvec3& waypoint : *bridgeRoute)
    {
        const double segmentLength = glm::length(waypoint - previous);
        const int sampleCount = std::max(1,
            static_cast<int>(std::ceil(segmentLength / 0.5)));
        for (int sample = 0; sample <= sampleCount; ++sample)
        {
            const glm::dvec3 point = glm::mix(previous, waypoint,
                static_cast<double>(sample) / sampleCount);
            if (std::any_of(bridgeCollisionShapes.begin(), bridgeCollisionShapes.end(),
                    [&point](const MapBuilding& shape)
                    { return ContainsMapBuildingVolume(shape, point); }))
            {
                routeHitsBridgeStructure = true;
                break;
            }
        }
        if (routeHitsBridgeStructure)
            break;
        previous = waypoint;
    }
    EXPECT_FALSE(routeHitsBridgeStructure)
        << "다리 하부 경로가 상판 또는 추정 교각 충돌 영역을 통과합니다.";

    // 기본 임무와 같은 순서로 다리·물길을 지난 뒤 111동 경로도 이어지는지 확인한다.
    glm::dvec3 missionRouteStart = demoStartPosition;
    const auto bridgeFirstRoute = PlanMapFlightRouteUnderBridge(
        terrain, buildings, *tancheonBridge, bridgePiers,
        missionRouteStart, bridgeApproach, bridgeDeckBottom, bridgeDeckTop,
        3.0, cameraTransitAltitudeMeters);
    ASSERT_TRUE(bridgeFirstRoute.has_value());
    missionRouteStart = bridgeApproach;

    const auto waterBodies = LoadMapWaterBodies(
        root / "assets/maps/tancheon/site.geojson", terrain);
    glm::dvec2 riverPosition = bridgeCenter;
    double nearestWaterDistance = std::numeric_limits<double>::max();
    for (const auto& water : waterBodies)
    {
        for (int northStep = -12; northStep <= 12; ++northStep)
        {
            for (int eastStep = -12; eastStep <= 12; ++eastStep)
            {
                const glm::dvec2 point = bridgeCenter
                    + glm::dvec2{eastStep * 5.0, northStep * 5.0};
                if (!ContainsWater(water, point))
                    continue;
                const double distance = glm::length(point - bridgeCenter);
                if (distance < nearestWaterDistance)
                {
                    nearestWaterDistance = distance;
                    riverPosition = point;
                }
            }
        }
    }
    const double riverGround = terrain.HeightAtLocalMeters(riverPosition);
    const glm::dvec3 riverApproach = terrain.ToSimulationPositionMeters(
        {riverPosition.x, riverPosition.y,
            riverGround - centerGround + 7.0});
    const auto riverRoute = PlanMapFlightRoute(
        terrain, buildings, missionRouteStart, riverApproach, 3.0,
        cameraTransitAltitudeMeters);
    ASSERT_TRUE(riverRoute.has_value());
    missionRouteStart = riverApproach;

    const auto firstBuilding = std::find_if(visibleBuildings.begin(),
        visibleBuildings.end(), [](const MapBuilding& building)
        {
            return building.osmId == "692770099";
        });
    ASSERT_NE(firstBuilding, visibleBuildings.end());
    const auto firstBuildingRoute = PlanMapFlightRouteToFacade(
        terrain, buildings, *firstBuilding, missionRouteStart, 8.0, 20.0);
    ASSERT_TRUE(firstBuildingRoute.has_value())
        << "탄천교 하부와 물길을 지난 뒤 헬리오시티 111동에 접근할 수 없습니다.";

    const std::string helio110BuildingId = "692770092";
    const auto helio110 = std::find_if(visibleBuildings.begin(), visibleBuildings.end(),
        [&helio110BuildingId](const MapBuilding& building)
        { return building.osmId == helio110BuildingId; });
    ASSERT_NE(helio110, visibleBuildings.end());
    const glm::dvec2 reportedStart{-472.0, 442.0};
    const glm::dvec3 reportedStartMeters{reportedStart,
        terrain.HeightAtLocalMeters(reportedStart) + 0.35};
    const auto buildingAtOldStart = std::find_if(buildings.begin(), buildings.end(),
        [&reportedStartMeters](const MapBuilding& building)
        {
            return ContainsMapBuildingVolume(building, reportedStartMeters);
        });
    ASSERT_NE(buildingAtOldStart, buildings.end());
    EXPECT_EQ(buildingAtOldStart->heightSource,
        BuildingHeightSource::EstimatedFromFootprint);
    const auto helio110Route = PlanMapFlightRouteToFacade(
        terrain, buildings, *helio110, reportedStartMeters, 8.0, 20.0);
    EXPECT_FALSE(helio110Route.has_value())
        << "facade-110-dong start=" << reportedStart.x << "," << reportedStart.y;
}

void Verify_TancheonTreeLocationsStayInsideTheMappedScene()
{
    const auto root = std::filesystem::path{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/tancheon/terrain.aerotrn");
    const auto trees = LoadMapTrees(
        root / "assets/maps/tancheon/site.geojson", terrain);

    EXPECT_GT(trees.size(), 100U);
    EXPECT_TRUE(std::all_of(trees.begin(), trees.end(), [&terrain](const MapTree& tree)
    {
        return terrain.ContainsLocalPositionMeters(tree.positionMeters);
    }));
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

void Verify_TancheonTerrainAssetLoadsAtTheRecordedOrigin()
{
    const auto path = std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/tancheon/terrain.aerotrn";
    const TerrainGrid terrain = TerrainGrid::Load(path);

    EXPECT_EQ(terrain.sampleCount, 241U);
    EXPECT_FLOAT_EQ(terrain.spacingXMeters, 5.0F);
    EXPECT_FLOAT_EQ(terrain.spacingYMeters, 5.0F);
    EXPECT_NEAR(terrain.centerLatitudeDegrees, 37.4936231, 1.0e-7);
    EXPECT_NEAR(terrain.centerLongitudeDegrees, 127.1018462, 1.0e-7);
    EXPECT_NEAR(terrain.HeightAtLocalMeters({0.0, 0.0}), 18.0F, 0.01F);
    // 32.6m 표고점 주변의 굴곡이 낮은 해상도 DEM에 묻히지 않았는지 확인한다.
    EXPECT_GT(terrain.HeightAtLocalMeters({-195.5, -24.5}), 37.0F);
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

    EXPECT_EQ(bridges.size(), 7U);
    EXPECT_EQ(std::count_if(bridges.begin(), bridges.end(),
        [](const MapBridgeFootprint& bridge) { return bridge.name == "탄천교"; }), 2);
    EXPECT_NE(std::find(names.begin(), names.end(), "탄천교"), names.end());
    EXPECT_NE(std::find(names.begin(), names.end(), "수서IC교"), names.end());
    EXPECT_NE(std::find(names.begin(), names.end(), "광평교"), names.end());

    std::size_t estimatedPierCount = 0U;
    for (const MapBridgeFootprint& bridge : bridges)
    {
        if (bridge.name != "탄천교")
            continue;

        double groundElevation = 0.0;
        for (const double elevation : bridge.groundElevationsMeters)
            groundElevation += elevation;
        groundElevation /= static_cast<double>(bridge.groundElevationsMeters.size());

        const auto piers = EstimateMapBridgePiers(bridge, terrain,
            groundElevation + 6.0);
        estimatedPierCount += piers.size();
        for (const MapBridgePier& pier : piers)
        {
            EXPECT_EQ(pier.outlineMeters.size(), 4U);
            EXPECT_LT(pier.bottomElevationMeters, pier.topElevationMeters);
            EXPECT_DOUBLE_EQ(pier.topElevationMeters, groundElevation + 6.0);
            for (const glm::dvec2& corner : pier.outlineMeters)
            {
                EXPECT_TRUE(terrain.ContainsLocalPositionMeters(corner));
            }
        }
    }
    EXPECT_GT(estimatedPierCount, 0U);
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
    EXPECT_NO_FATAL_FAILURE(Verify_RenderedTerrainHeightUsesTheSameTriangleAsTheMesh());
    EXPECT_NO_FATAL_FAILURE(Verify_AsciiGridKeepsFractionalElevationsAndRoundTrips());
    EXPECT_NO_FATAL_FAILURE(Verify_AsciiGridRejectsMissingHeightSamples());
    EXPECT_NO_FATAL_FAILURE(Verify_MapFlightRouteGoesAroundBuildingFootprints());
    EXPECT_NO_FATAL_FAILURE(
        Verify_TancheonInspectionRouteAvoidsAllMappedBuildingFootprints());
    EXPECT_NO_FATAL_FAILURE(Verify_TancheonTreeLocationsStayInsideTheMappedScene());
    EXPECT_NO_FATAL_FAILURE(Verify_MapFlightRouteReportsABuildingWallWithoutAnOpening());
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
