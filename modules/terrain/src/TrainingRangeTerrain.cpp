#include "terrain/TrainingRangeTerrain.h"

#include <algorithm>
#include <cmath>

namespace aerosimlab::terrain
{
TerrainGrid CreateTrainingRangeTerrain()
{
    constexpr std::uint32_t sampleCount = 101U;

    TerrainGrid terrain;
    terrain.sampleCount = sampleCount;
    terrain.spacingXMeters = 5.0F;
    terrain.spacingYMeters = 4.0F;
    terrain.referenceElevationMeters = 0.0F;
    terrain.heightOffsetsMeters.resize(
        static_cast<std::size_t>(sampleCount) * sampleCount);

    for (std::uint32_t row = 0; row < sampleCount; ++row)
    {
        const double north = terrain.HalfDepthMeters()
            - static_cast<double>(row) * terrain.spacingYMeters;
        for (std::uint32_t column = 0; column < sampleCount; ++column)
        {
            const double east = -terrain.HalfWidthMeters()
                + static_cast<double>(column) * terrain.spacingXMeters;
            const double westHill = 34.0 * std::exp(
                -std::pow((east + 145.0) / 72.0, 2.0)
                -std::pow((north - 105.0) / 58.0, 2.0));
            const double eastHill = 25.0 * std::exp(
                -std::pow((east - 155.0) / 66.0, 2.0)
                -std::pow((north - 92.0) / 78.0, 2.0));
            const double ridgeCenter = 142.0 + 0.11 * east;
            const double ridge = 11.0 * std::exp(
                -std::pow((north - ridgeCenter) / 27.0, 2.0));
            const double valley = std::exp(
                -std::pow((north + 22.0) / 70.0, 2.0));

            terrain.heightOffsetsMeters[
                static_cast<std::size_t>(row) * sampleCount + column]
                = static_cast<float>((westHill + eastHill + ridge) * (1.0 - 0.18 * valley));
        }
    }

    const float lowestHeight = *std::min_element(
        terrain.heightOffsetsMeters.begin(), terrain.heightOffsetsMeters.end());
    for (float& height : terrain.heightOffsetsMeters)
        height -= lowestHeight;

    return terrain;
}
}
