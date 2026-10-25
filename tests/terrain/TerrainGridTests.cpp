#include "terrain/TerrainGrid.h"

#include <gtest/gtest.h>

#include <cmath>

namespace aerosimlab::terrain
{
void Verify_TerrainGridConvertsLocalCoordinatesAndInterpolatesHeight()
{
    TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.spacingXMeters = 10.0F;
    terrain.spacingYMeters = 20.0F;
    terrain.centerLatitudeDegrees = 37.0;
    terrain.centerLongitudeDegrees = -122.0;
    terrain.heightOffsetsMeters = {0.0F, 10.0F, 20.0F,
        10.0F, 20.0F, 30.0F,
        20.0F, 30.0F, 40.0F};

    EXPECT_DOUBLE_EQ(terrain.ToLocalMeters(37.0, -122.0).x, 0.0);
    EXPECT_NEAR(terrain.ToLocalMeters(37.0 + 20.0 / 111320.0, -122.0).y,
        20.0, 0.02);
    EXPECT_FLOAT_EQ(terrain.HeightAtLocalMeters({0.0, 0.0}), 20.0F);
}

TEST(TerrainGridTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
        Verify_TerrainGridConvertsLocalCoordinatesAndInterpolatesHeight());
}

}
