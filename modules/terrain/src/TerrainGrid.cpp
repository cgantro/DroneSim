#include "terrain/TerrainGrid.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace aerosimlab::terrain
{
namespace
{
constexpr std::array<char, 8> terrainMagic{'A', 'E', 'R', 'O', 'T', 'R', 'N', '1'};
constexpr std::uint64_t headerSizeBytes = 40;

template <typename T>
bool ReadLittleEndian(std::ifstream& input, T& value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    std::array<unsigned char, sizeof(T)> bytes{};
    if (!input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())))
        return false;
    if constexpr (std::endian::native == std::endian::big)
        std::reverse(bytes.begin(), bytes.end());
    std::memcpy(&value, bytes.data(), sizeof(T));
    return true;
}

void RequireFinitePositive(float value, const char* label)
{
    if (!std::isfinite(value) || !(value > 0.0F))
        throw std::invalid_argument(std::string("지형 ") + label + " 값은 0보다 커야 합니다.");
}
}

TerrainGrid TerrainGrid::Load(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
        throw std::runtime_error("지형 자료를 열지 못했습니다: " + path.string());

    const std::streamoff fileSize = input.tellg();
    if (fileSize < static_cast<std::streamoff>(headerSizeBytes))
        throw std::runtime_error("지형 자료의 머리 정보가 잘렸습니다.");
    input.seekg(0);

    std::array<char, terrainMagic.size()> magic{};
    TerrainGrid grid;
    if (!input.read(magic.data(), static_cast<std::streamsize>(magic.size()))
        || magic != terrainMagic
        || !ReadLittleEndian(input, grid.sampleCount)
        || !ReadLittleEndian(input, grid.spacingXMeters)
        || !ReadLittleEndian(input, grid.spacingYMeters)
        || !ReadLittleEndian(input, grid.referenceElevationMeters)
        || !ReadLittleEndian(input, grid.centerLatitudeDegrees)
        || !ReadLittleEndian(input, grid.centerLongitudeDegrees))
    {
        throw std::runtime_error("지형 자료의 형식이나 머리 정보가 올바르지 않습니다.");
    }

    RequireFinitePositive(grid.spacingXMeters, "동서 간격");
    RequireFinitePositive(grid.spacingYMeters, "남북 간격");
    if (!std::isfinite(grid.referenceElevationMeters)
        || !std::isfinite(grid.centerLatitudeDegrees)
        || !std::isfinite(grid.centerLongitudeDegrees)
        || grid.centerLatitudeDegrees < -90.0 || grid.centerLatitudeDegrees > 90.0
        || grid.centerLongitudeDegrees < -180.0 || grid.centerLongitudeDegrees > 180.0
        || grid.sampleCount < 3U)
    {
        throw std::runtime_error("지형 크기, 기준 높이 또는 중심 좌표를 확인해 주세요.");
    }

    const std::uint64_t sampleTotal = static_cast<std::uint64_t>(grid.sampleCount) * grid.sampleCount;
    const std::uint64_t expectedSize = headerSizeBytes + sampleTotal * sizeof(std::int16_t);
    if (expectedSize != static_cast<std::uint64_t>(fileSize)
        || sampleTotal > std::numeric_limits<std::size_t>::max())
    {
        throw std::runtime_error("지형 자료의 높이 개수가 크기 정보와 맞지 않습니다.");
    }

    grid.heightOffsetsMeters.resize(static_cast<std::size_t>(sampleTotal));
    for (float& height : grid.heightOffsetsMeters)
    {
        std::uint16_t encodedHeight = 0;
        if (!ReadLittleEndian(input, encodedHeight))
            throw std::runtime_error("지형 높이 자료가 중간에 끝났습니다.");
        const std::int16_t heightOffset = std::bit_cast<std::int16_t>(encodedHeight);
        if (heightOffset == std::numeric_limits<std::int16_t>::min())
            throw std::runtime_error("높이를 알 수 없는 지점이 지형 자료에 있습니다.");
        height = static_cast<float>(heightOffset);
    }
    return grid;
}

float TerrainGrid::HeightAt(std::uint32_t row, std::uint32_t column) const
{
    if (row >= sampleCount || column >= sampleCount)
        throw std::out_of_range("지형 격자 위치가 범위를 벗어났습니다.");
    return heightOffsetsMeters[static_cast<std::size_t>(row) * sampleCount + column];
}

bool TerrainGrid::ContainsLocalPositionMeters(
    const glm::dvec2& positionMeters) const noexcept
{
    if (sampleCount < 2U || !std::isfinite(positionMeters.x)
        || !std::isfinite(positionMeters.y)
        || !std::isfinite(spacingXMeters) || !(spacingXMeters > 0.0F)
        || !std::isfinite(spacingYMeters) || !(spacingYMeters > 0.0F))
    {
        return false;
    }

    const double halfWidth = static_cast<double>(sampleCount - 1U)
        * spacingXMeters * 0.5;
    const double halfDepth = static_cast<double>(sampleCount - 1U)
        * spacingYMeters * 0.5;
    return positionMeters.x >= -halfWidth && positionMeters.x <= halfWidth
        && positionMeters.y >= -halfDepth && positionMeters.y <= halfDepth;
}

float TerrainGrid::HeightAtLocalMeters(const glm::dvec2& positionMeters) const
{
    if (sampleCount < 2U || !std::isfinite(positionMeters.x)
        || !std::isfinite(positionMeters.y)
        || !std::isfinite(spacingXMeters) || !(spacingXMeters > 0.0F)
        || !std::isfinite(spacingYMeters) || !(spacingYMeters > 0.0F)
        || !std::isfinite(referenceElevationMeters)
        || heightOffsetsMeters.size() != static_cast<std::size_t>(sampleCount) * sampleCount)
    {
        throw std::invalid_argument("지형 위치와 격자 자료를 확인해 주세요.");
    }

    if (!ContainsLocalPositionMeters(positionMeters))
        throw std::out_of_range("요청한 위치가 지형 격자 범위를 벗어났습니다.");

    const double halfWidth = static_cast<double>(sampleCount - 1U)
        * spacingXMeters * 0.5;
    const double halfDepth = static_cast<double>(sampleCount - 1U)
        * spacingYMeters * 0.5;
    const double column = (positionMeters.x + halfWidth) / spacingXMeters;
    const double row = (halfDepth - positionMeters.y) / spacingYMeters;
    const auto columnBefore = static_cast<std::uint32_t>(std::floor(column));
    const auto rowBefore = static_cast<std::uint32_t>(std::floor(row));
    const std::uint32_t columnAfter = std::min(columnBefore + 1U, sampleCount - 1U);
    const std::uint32_t rowAfter = std::min(rowBefore + 1U, sampleCount - 1U);
    const float columnFraction = static_cast<float>(column - columnBefore);
    const float rowFraction = static_cast<float>(row - rowBefore);
    const float north = std::lerp(
        HeightAt(rowBefore, columnBefore), HeightAt(rowBefore, columnAfter), columnFraction);
    const float south = std::lerp(
        HeightAt(rowAfter, columnBefore), HeightAt(rowAfter, columnAfter), columnFraction);
    return referenceElevationMeters + std::lerp(north, south, rowFraction);
}

glm::dvec2 TerrainGrid::ToLocalMeters(
    double latitudeDegrees, double longitudeDegrees) const
{
    if (!std::isfinite(latitudeDegrees) || !std::isfinite(longitudeDegrees)
        || latitudeDegrees < -90.0 || latitudeDegrees > 90.0
        || longitudeDegrees < -180.0 || longitudeDegrees > 180.0)
    {
        throw std::invalid_argument("위도와 경도는 유효한 좌표여야 합니다.");
    }

    constexpr double metersPerLatitudeDegree = 111'320.0;
    constexpr double radiansPerDegree = std::numbers::pi_v<double> / 180.0;
    const double metersPerLongitudeDegree = metersPerLatitudeDegree
        * std::cos(centerLatitudeDegrees * radiansPerDegree);
    return {
        (longitudeDegrees - centerLongitudeDegrees) * metersPerLongitudeDegree,
        (latitudeDegrees - centerLatitudeDegrees) * metersPerLatitudeDegree};
}

glm::dvec3 TerrainGrid::ToSimulationPositionMeters(
    const glm::dvec3& eastNorthUpMeters) const
{
    return {
        eastNorthUpMeters.x,
        eastNorthUpMeters.y,
        eastNorthUpMeters.z + HeightAtLocalMeters({0.0, 0.0})};
}
}
