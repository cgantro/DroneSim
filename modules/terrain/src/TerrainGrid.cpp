#include "terrain/TerrainGrid.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace dronesim::terrain
{
namespace
{
struct TerrainCellCoordinates
{
    std::uint32_t northRow;
    std::uint32_t westColumn;
    float eastFraction;
    float southFraction;
};

bool HasValidGridExtent(const TerrainGrid& terrain) noexcept
{
    return terrain.sampleCount >= 2U
        && std::isfinite(terrain.spacingXMeters) && terrain.spacingXMeters > 0.0F
        && std::isfinite(terrain.spacingYMeters) && terrain.spacingYMeters > 0.0F;
}

TerrainCellCoordinates FindTerrainCell(
    const TerrainGrid& terrain, const glm::dvec2& positionMeters)
{
    if (!HasValidGridExtent(terrain)
        || !std::isfinite(positionMeters.x) || !std::isfinite(positionMeters.y)
        || !std::isfinite(terrain.referenceElevationMeters)
        || terrain.heightOffsetsMeters.size()
            != static_cast<std::size_t>(terrain.sampleCount) * terrain.sampleCount)
    {
        throw std::invalid_argument("지형 위치와 격자 자료를 확인해 주세요.");
    }
    if (!terrain.ContainsLocalPositionMeters(positionMeters))
        throw std::out_of_range("요청한 위치가 지형 격자 범위를 벗어났습니다: ("
            + std::to_string(positionMeters.x) + ", "
            + std::to_string(positionMeters.y) + ") m");

    // 동쪽으로 갈수록 열 번호가 커지고, 남쪽으로 갈수록 행 번호가 커진다.
    // 따라서 열은 서쪽 경계에서, 행은 북쪽 경계에서부터 거리를 재어 찾는다.
    const double column = (positionMeters.x + terrain.HalfWidthMeters())
        / terrain.spacingXMeters;
    const double row = (terrain.HalfDepthMeters() - positionMeters.y)
        / terrain.spacingYMeters;
    const auto westColumn = std::min(
        static_cast<std::uint32_t>(std::floor(column)), terrain.sampleCount - 2U);
    const auto northRow = std::min(
        static_cast<std::uint32_t>(std::floor(row)), terrain.sampleCount - 2U);
    return {northRow, westColumn,
        static_cast<float>(column - westColumn),
        static_cast<float>(row - northRow)};
}

}

float TerrainGrid::HeightAt(std::uint32_t row, std::uint32_t column) const
{
    if (row >= sampleCount || column >= sampleCount)
        throw std::out_of_range("지형 격자 위치가 범위를 벗어났습니다.");
    return heightOffsetsMeters[static_cast<std::size_t>(row) * sampleCount + column];
}

double TerrainGrid::HalfWidthMeters() const noexcept
{
    return sampleCount > 1U
        ? static_cast<double>(sampleCount - 1U) * spacingXMeters * 0.5
        : 0.0;
}

double TerrainGrid::HalfDepthMeters() const noexcept
{
    return sampleCount > 1U
        ? static_cast<double>(sampleCount - 1U) * spacingYMeters * 0.5
        : 0.0;
}

bool TerrainGrid::ContainsLocalPositionMeters(
    const glm::dvec2& positionMeters) const noexcept
{
    if (!HasValidGridExtent(*this)
        || !std::isfinite(positionMeters.x) || !std::isfinite(positionMeters.y))
    {
        return false;
    }

    const double halfWidth = HalfWidthMeters();
    const double halfDepth = HalfDepthMeters();
    return positionMeters.x >= -halfWidth && positionMeters.x <= halfWidth
        && positionMeters.y >= -halfDepth && positionMeters.y <= halfDepth;
}

float TerrainGrid::HeightAtLocalMeters(const glm::dvec2& positionMeters) const
{
    const auto cell = FindTerrainCell(*this, positionMeters);
    const std::uint32_t eastColumn = cell.westColumn + 1U;
    const std::uint32_t southRow = cell.northRow + 1U;
    // 먼저 북쪽·남쪽 변에서 동서 방향으로 보간하고, 두 값을 다시 남북 방향으로 섞는다.
    const float north = std::lerp(
        HeightAt(cell.northRow, cell.westColumn), HeightAt(cell.northRow, eastColumn),
        cell.eastFraction);
    const float south = std::lerp(
        HeightAt(southRow, cell.westColumn), HeightAt(southRow, eastColumn),
        cell.eastFraction);
    // 격자에는 기준 고도와의 차이를 저장하므로, 보간이 끝난 뒤 기준 고도를 더한다.
    return referenceElevationMeters + std::lerp(north, south, cell.southFraction);
}

}
