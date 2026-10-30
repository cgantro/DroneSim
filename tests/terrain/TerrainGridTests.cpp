#include "terrain/TerrainGrid.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace aerosimlab::terrain
{
// 격자 중심과 경계 좌표가 올바르고 표본 사이 높이를 보간하는지 확인한다.
void Verify_TerrainGridBoundsAndInterpolatesHeight()
{
    TerrainGrid terrain;
    terrain.sampleCount = 3;
    terrain.spacingXMeters = 10.0F;
    terrain.spacingYMeters = 20.0F;
    terrain.referenceElevationMeters = 100.0F;
    terrain.heightOffsetsMeters = {0.0F, 10.0F, 20.0F,
        10.0F, 20.0F, 30.0F,
        20.0F, 30.0F, 40.0F};

    EXPECT_DOUBLE_EQ(terrain.HalfWidthMeters(), 10.0);
    EXPECT_DOUBLE_EQ(terrain.HalfDepthMeters(), 20.0);
    EXPECT_TRUE(terrain.ContainsLocalPositionMeters({10.0, 20.0}));
    EXPECT_FALSE(terrain.ContainsLocalPositionMeters({10.01, 0.0}));
    EXPECT_FLOAT_EQ(terrain.HeightAtLocalMeters({0.0, 0.0}), 120.0F);
}

// 소수점 고도를 기준 높이와 분리해 저장하고 다시 읽어도 보존되는지 확인한다.
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

    const TerrainGrid imported = TerrainGrid::LoadAsciiGrid(sourcePath, 100.5F);
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
    std::error_code ignored;
    std::filesystem::remove(sourcePath, ignored);
    std::filesystem::remove(convertedPath, ignored);
}

// 표본 수가 머리 정보보다 적은 파일은 유효한 지형으로 받아들이지 않는다.
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
            path, 100.5F)), std::runtime_error);
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

TEST(TerrainGridTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
        Verify_TerrainGridBoundsAndInterpolatesHeight());
    EXPECT_NO_FATAL_FAILURE(Verify_AsciiGridKeepsFractionalElevationsAndRoundTrips());
    EXPECT_NO_FATAL_FAILURE(Verify_AsciiGridRejectsMissingHeightSamples());
}

}
