#include "terrain/TerrainGrid.h"

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

TEST(TerrainGridTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
        Verify_TerrainGridConvertsLocalCoordinatesAndInterpolatesHeight());
    EXPECT_NO_FATAL_FAILURE(Verify_TancheonTerrainAssetLoadsAtTheRecordedOrigin());
}

}
