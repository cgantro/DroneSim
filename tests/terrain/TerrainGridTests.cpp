#include "terrain/TerrainGrid.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>

namespace aerosimlab::terrain
{
TEST(TerrainGrid, LoadsThePublicTestAreaWithItsLocalHeightOrigin)
{
    const TerrainGrid terrain = TerrainGrid::Load(
        std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/bukhansan-demo/terrain.grid");

    ASSERT_EQ(terrain.sampleCount, 65U);
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
    EXPECT_NEAR(terrain.HeightAt(32, 32), 0.0F, 1.0e-5F);
    EXPECT_GT(terrain.spacingXMeters, 24.0F);
    EXPECT_LT(terrain.spacingXMeters, 25.0F);
    EXPECT_GT(terrain.spacingYMeters, 30.0F);
    EXPECT_LT(terrain.spacingYMeters, 32.0F);
    EXPECT_LT(*std::min_element(terrain.heightOffsetsMeters.begin(),
        terrain.heightOffsetsMeters.end()), -200.0F);
    EXPECT_GT(*std::max_element(terrain.heightOffsetsMeters.begin(),
        terrain.heightOffsetsMeters.end()), 300.0F);
}

TEST(TerrainGrid, RejectsARequestedSampleOutsideTheGrid)
{
    TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.heightOffsetsMeters.resize(9);

    EXPECT_THROW(static_cast<void>(terrain.HeightAt(3, 0)), std::out_of_range);
}
}
