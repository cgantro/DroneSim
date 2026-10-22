#include "terrain/BuildingFootprint.h"
#include "terrain/StaticFacility.h"
#include "terrain/TerrainGrid.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

namespace aerosimlab::terrain
{
void Verify_TerrainGrid_LoadsThePublicTestAreaWithItsLocalHeightOrigin()
{
    const TerrainGrid terrain = TerrainGrid::Load(
        std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/bukhansan-demo/terrain.grid");

    ASSERT_EQ(terrain.sampleCount, 385U);
    EXPECT_NEAR(terrain.centerLatitudeDegrees, 37.661111, 1.0e-5);
    EXPECT_NEAR(terrain.centerLongitudeDegrees, 126.983889, 1.0e-5);
    EXPECT_NEAR(terrain.ToLocalMeters(
        terrain.centerLatitudeDegrees, terrain.centerLongitudeDegrees).x, 0.0, 1.0e-9);
    EXPECT_NEAR(terrain.ToLocalMeters(
        terrain.centerLatitudeDegrees + 1.0 / 3600.0,
        terrain.centerLongitudeDegrees).y, terrain.spacingYMeters, 0.01);
    EXPECT_NEAR(terrain.ToLocalMeters(
        terrain.centerLatitudeDegrees,
        terrain.centerLongitudeDegrees + 1.0 / 3600.0).x, terrain.spacingXMeters, 0.01);
    EXPECT_NEAR(terrain.referenceElevationMeters, 504.0F, 1.0e-5F);
    EXPECT_NEAR(terrain.HeightAt(192, 192), 0.0F, 1.0e-5F);
    EXPECT_NEAR((terrain.sampleCount - 1U) * terrain.spacingXMeters,
        9399.4F, 1.0F);
    EXPECT_NEAR((terrain.sampleCount - 1U) * terrain.spacingYMeters,
        11874.1F, 1.0F);
    EXPECT_GT(terrain.spacingXMeters, 24.0F);
    EXPECT_LT(terrain.spacingXMeters, 25.0F);
    EXPECT_GT(terrain.spacingYMeters, 30.0F);
    EXPECT_LT(terrain.spacingYMeters, 32.0F);
    EXPECT_LT(*std::min_element(terrain.heightOffsetsMeters.begin(),
        terrain.heightOffsetsMeters.end()), -200.0F);
    EXPECT_GT(*std::max_element(terrain.heightOffsetsMeters.begin(),
        terrain.heightOffsetsMeters.end()), 300.0F);
}

void Verify_TerrainGrid_RejectsARequestedSampleOutsideTheGrid()
{
    TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.heightOffsetsMeters.resize(9);

    EXPECT_THROW(static_cast<void>(terrain.HeightAt(3, 0)), std::out_of_range);
}

void Verify_TerrainGrid_InterpolatesHeightAtALocalPosition()
{
    TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.spacingXMeters = 2.0F;
    terrain.spacingYMeters = 2.0F;
    terrain.heightOffsetsMeters = {0, 2, 4, 2, 4, 6, 4, 6, 8};

    EXPECT_FLOAT_EQ(4.0F, terrain.HeightAtLocalMeters({0.0, 0.0}));
    EXPECT_FLOAT_EQ(2.0F, terrain.HeightAtLocalMeters({-1.0, 1.0}));
    EXPECT_THROW(static_cast<void>(terrain.HeightAtLocalMeters({2.1, 0.0})),
        std::out_of_range);
}

void Verify_BuildingFootprint_LoadsThePublicMapWithoutInventingMissingHeights()
{
    const std::filesystem::path root{AEROSIMLAB_SOURCE_DIR};
    const TerrainGrid terrain = TerrainGrid::Load(
        root / "assets/maps/bukhansan-demo/terrain.grid");
    const std::vector<BuildingFootprint> buildings = LoadBuildingFootprints(
        root / "assets/maps/bukhansan-demo/buildings.geojson", terrain);

    ASSERT_EQ(buildings.size(), 17U);
    for (const BuildingFootprint& building : buildings)
    {
        EXPECT_FALSE(building.osmId.empty());
        EXPECT_FALSE(building.buildingType.empty());
        ASSERT_GE(building.outlineMeters.size(), 4U);
        EXPECT_DOUBLE_EQ(building.outlineMeters.front().x, building.outlineMeters.back().x);
        EXPECT_DOUBLE_EQ(building.outlineMeters.front().y, building.outlineMeters.back().y);
        EXPECT_FALSE(building.heightMeters.has_value());
    }
}

void Verify_StaticFacility_LoadsTheSurveyedParkFeaturesAndCollisionBounds()
{
    const std::vector<StaticFacility> facilities = LoadStaticFacilities(
        std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/bukhansan-demo/facilities.json");

    ASSERT_EQ(facilities.size(), 49U);
    for (const StaticFacility& facility : facilities)
    {
        EXPECT_FALSE(facility.id.empty());
        EXPECT_FALSE(facility.type.empty());
        EXPECT_TRUE(std::isfinite(facility.centerMeters.x));
        EXPECT_TRUE(std::isfinite(facility.centerMeters.y));
        EXPECT_TRUE(std::isfinite(facility.centerMeters.z));
        EXPECT_GT(facility.halfExtentsMeters.x, 0.0F);
        EXPECT_GT(facility.halfExtentsMeters.y, 0.0F);
        EXPECT_GT(facility.halfExtentsMeters.z, 0.0F);
        EXPECT_LT(std::abs(facility.centerMeters.x), 800.0);
        EXPECT_LT(std::abs(facility.centerMeters.y), 1'000.0);
    }
}

TEST(TerrainGridTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_TerrainGrid_LoadsThePublicTestAreaWithItsLocalHeightOrigin());
    EXPECT_NO_FATAL_FAILURE(Verify_TerrainGrid_RejectsARequestedSampleOutsideTheGrid());
    EXPECT_NO_FATAL_FAILURE(Verify_TerrainGrid_InterpolatesHeightAtALocalPosition());
    EXPECT_NO_FATAL_FAILURE(Verify_BuildingFootprint_LoadsThePublicMapWithoutInventingMissingHeights());
    EXPECT_NO_FATAL_FAILURE(Verify_StaticFacility_LoadsTheSurveyedParkFeaturesAndCollisionBounds());
}

}
