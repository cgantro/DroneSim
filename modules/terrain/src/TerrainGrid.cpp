#include "terrain/TerrainGrid.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <glm/vec3.hpp>

namespace aerosimlab::terrain
{
namespace
{
constexpr std::array<char, 8> terrainMagic{'A', 'E', 'R', 'O', 'T', 'R', 'N', '1'};
constexpr std::array<char, 8> floatTerrainMagic{'A', 'E', 'R', 'O', 'T', 'R', 'N', '2'};
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

template <typename T>
void WriteLittleEndian(std::ofstream& output, T value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    std::array<unsigned char, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
    if constexpr (std::endian::native == std::endian::big)
        std::reverse(bytes.begin(), bytes.end());
    output.write(reinterpret_cast<const char*>(bytes.data()),
        static_cast<std::streamsize>(bytes.size()));
}

void RequireFinitePositive(float value, const char* label)
{
    if (!std::isfinite(value) || !(value > 0.0F))
        throw std::invalid_argument(std::string("지형 ") + label + " 값은 0보다 커야 합니다.");
}

struct TerrainCellCoordinates
{
    std::uint32_t northRow;
    std::uint32_t westColumn;
    float eastFraction;
    float southFraction;
};

TerrainCellCoordinates FindTerrainCell(
    const TerrainGrid& terrain, const glm::dvec2& positionMeters)
{
    if (terrain.sampleCount < 2U || !std::isfinite(positionMeters.x)
        || !std::isfinite(positionMeters.y)
        || !std::isfinite(terrain.spacingXMeters) || !(terrain.spacingXMeters > 0.0F)
        || !std::isfinite(terrain.spacingYMeters) || !(terrain.spacingYMeters > 0.0F)
        || !std::isfinite(terrain.referenceElevationMeters)
        || terrain.heightOffsetsMeters.size()
            != static_cast<std::size_t>(terrain.sampleCount) * terrain.sampleCount)
    {
        throw std::invalid_argument("지형 위치와 격자 자료를 확인해 주세요.");
    }
    if (!terrain.ContainsLocalPositionMeters(positionMeters))
        throw std::out_of_range("요청한 위치가 지형 격자 범위를 벗어났습니다.");

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

glm::dvec3 Wgs84SurfacePositionEcef(
    double latitudeDegrees, double longitudeDegrees)
{
    constexpr double semiMajorAxisMeters = 6'378'137.0;
    constexpr double flattening = 1.0 / 298.257223563;
    constexpr double eccentricitySquared = flattening * (2.0 - flattening);
    constexpr double radiansPerDegree = std::numbers::pi_v<double> / 180.0;

    const double latitudeRadians = latitudeDegrees * radiansPerDegree;
    const double longitudeRadians = longitudeDegrees * radiansPerDegree;
    const double sinLatitude = std::sin(latitudeRadians);
    const double cosLatitude = std::cos(latitudeRadians);
    const double primeVerticalRadius = semiMajorAxisMeters
        / std::sqrt(1.0 - eccentricitySquared * sinLatitude * sinLatitude);
    return {
        primeVerticalRadius * cosLatitude * std::cos(longitudeRadians),
        primeVerticalRadius * cosLatitude * std::sin(longitudeRadians),
        primeVerticalRadius * (1.0 - eccentricitySquared) * sinLatitude};
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
        || (magic != terrainMagic && magic != floatTerrainMagic)
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
    const bool storesFloatHeights = magic == floatTerrainMagic;
    const std::uint64_t bytesPerHeight = storesFloatHeights
        ? sizeof(float) : sizeof(std::int16_t);
    const std::uint64_t expectedSize = headerSizeBytes + sampleTotal * bytesPerHeight;
    if (expectedSize != static_cast<std::uint64_t>(fileSize)
        || sampleTotal > std::numeric_limits<std::size_t>::max())
    {
        throw std::runtime_error("지형 자료의 높이 개수가 크기 정보와 맞지 않습니다.");
    }

    grid.heightOffsetsMeters.resize(static_cast<std::size_t>(sampleTotal));
    for (float& height : grid.heightOffsetsMeters)
    {
        if (storesFloatHeights)
        {
            if (!ReadLittleEndian(input, height) || !std::isfinite(height))
                throw std::runtime_error("지형 높이 자료가 잘렸거나 숫자가 아닙니다.");
        }
        else
        {
            std::uint16_t encodedHeight = 0;
            if (!ReadLittleEndian(input, encodedHeight))
                throw std::runtime_error("지형 높이 자료가 중간에 끝났습니다.");
            const std::int16_t heightOffset = std::bit_cast<std::int16_t>(encodedHeight);
            if (heightOffset == std::numeric_limits<std::int16_t>::min())
                throw std::runtime_error("높이를 알 수 없는 지점이 지형 자료에 있습니다.");
            height = static_cast<float>(heightOffset);
        }
    }
    return grid;
}

TerrainGrid TerrainGrid::LoadAsciiGrid(
    const std::filesystem::path& path,
    double centerLatitudeDegrees,
    double centerLongitudeDegrees,
    float referenceElevationMeters)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("ASCII 지형 격자를 열지 못했습니다: " + path.string());

    std::map<std::string, double> header;
    for (int index = 0; index < 6; ++index)
    {
        std::string line;
        std::string key;
        double value = 0.0;
        if (!std::getline(input, line))
            throw std::runtime_error("ASCII 지형 격자 머리 정보가 부족합니다.");
        std::istringstream row(line);
        if (!(row >> key >> value) || !std::isfinite(value))
            throw std::runtime_error("ASCII 지형 격자 머리 정보를 읽지 못했습니다.");
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character)
        {
            return static_cast<char>(std::tolower(character));
        });
        if (!header.emplace(std::move(key), value).second)
            throw std::runtime_error("ASCII 지형 격자 머리 정보가 중복되었습니다.");
    }

    const auto requiredValue = [&header](const char* key)
    {
        const auto value = header.find(key);
        if (value == header.end())
            throw std::runtime_error(std::string("ASCII 지형 격자에 ")
                + key + " 항목이 없습니다.");
        return value->second;
    };
    const double columnCountValue = requiredValue("ncols");
    const double rowCountValue = requiredValue("nrows");
    const double cellSize = requiredValue("cellsize");
    const double noDataValue = requiredValue("nodata_value");
    if (columnCountValue != rowCountValue || columnCountValue < 3.0
        || columnCountValue > static_cast<double>(std::numeric_limits<std::uint32_t>::max())
        || std::floor(columnCountValue) != columnCountValue
        || !std::isfinite(cellSize) || !(cellSize > 0.0)
        || cellSize > static_cast<double>(std::numeric_limits<float>::max())
        || !std::isfinite(centerLatitudeDegrees)
        || !std::isfinite(centerLongitudeDegrees)
        || !std::isfinite(referenceElevationMeters))
    {
        throw std::runtime_error(
            "지형 격자는 미터 단위의 정사각형 자료여야 하며 기준 좌표와 높이가 필요합니다.");
    }
    const bool hasXCorner = header.contains("xllcorner");
    const bool hasYCorner = header.contains("yllcorner");
    const bool hasXCenter = header.contains("xllcenter");
    const bool hasYCenter = header.contains("yllcenter");
    if (hasXCorner != hasYCorner || hasXCenter != hasYCenter
        || hasXCorner == hasXCenter)
    {
        throw std::runtime_error("ASCII 지형 격자의 시작 좌표 항목을 확인해 주세요.");
    }
    static_cast<void>(requiredValue(hasXCorner ? "xllcorner" : "xllcenter"));
    static_cast<void>(requiredValue(hasYCorner ? "yllcorner" : "yllcenter"));

    TerrainGrid grid;
    grid.sampleCount = static_cast<std::uint32_t>(columnCountValue);
    grid.spacingXMeters = static_cast<float>(cellSize);
    grid.spacingYMeters = static_cast<float>(cellSize);
    grid.referenceElevationMeters = referenceElevationMeters;
    grid.centerLatitudeDegrees = centerLatitudeDegrees;
    grid.centerLongitudeDegrees = centerLongitudeDegrees;
    grid.heightOffsetsMeters.reserve(static_cast<std::size_t>(grid.sampleCount)
        * grid.sampleCount);

    double elevation = 0.0;
    while (input >> elevation)
    {
        if (!std::isfinite(elevation) || elevation == noDataValue)
            throw std::runtime_error(
                "ASCII 지형 격자에 비어 있거나 유효하지 않은 높이가 있습니다.");
        const double heightOffset = elevation - referenceElevationMeters;
        if (!std::isfinite(heightOffset)
            || std::abs(heightOffset) > std::numeric_limits<float>::max())
        {
            throw std::runtime_error("ASCII 지형 격자의 높이가 저장 범위를 벗어났습니다.");
        }
        grid.heightOffsetsMeters.push_back(static_cast<float>(heightOffset));
    }
    if (!input.eof() || grid.heightOffsetsMeters.size()
        != static_cast<std::size_t>(grid.sampleCount) * grid.sampleCount)
    {
        throw std::runtime_error("ASCII 지형 격자의 높이 개수가 머리 정보와 다릅니다.");
    }
    return grid;
}

void TerrainGrid::Save(const std::filesystem::path& path) const
{
    const std::size_t expectedSamples = static_cast<std::size_t>(sampleCount) * sampleCount;
    if (sampleCount < 3U || heightOffsetsMeters.size() != expectedSamples
        || !std::isfinite(spacingXMeters) || !(spacingXMeters > 0.0F)
        || !std::isfinite(spacingYMeters) || !(spacingYMeters > 0.0F)
        || !std::isfinite(referenceElevationMeters)
        || !std::isfinite(centerLatitudeDegrees) || centerLatitudeDegrees < -90.0
        || centerLatitudeDegrees > 90.0 || !std::isfinite(centerLongitudeDegrees)
        || centerLongitudeDegrees < -180.0 || centerLongitudeDegrees > 180.0
        || !std::all_of(heightOffsetsMeters.begin(), heightOffsetsMeters.end(),
            [](float height) { return std::isfinite(height); }))
    {
        throw std::invalid_argument("저장할 지형 격자와 기준 정보를 확인해 주세요.");
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("지형 자료를 저장할 수 없습니다: " + path.string());
    output.write(floatTerrainMagic.data(),
        static_cast<std::streamsize>(floatTerrainMagic.size()));
    WriteLittleEndian(output, sampleCount);
    WriteLittleEndian(output, spacingXMeters);
    WriteLittleEndian(output, spacingYMeters);
    WriteLittleEndian(output, referenceElevationMeters);
    WriteLittleEndian(output, centerLatitudeDegrees);
    WriteLittleEndian(output, centerLongitudeDegrees);
    for (const float height : heightOffsetsMeters)
        WriteLittleEndian(output, height);
    if (!output)
        throw std::runtime_error("지형 자료를 끝까지 저장하지 못했습니다.");
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
    if (sampleCount < 2U || !std::isfinite(positionMeters.x)
        || !std::isfinite(positionMeters.y)
        || !std::isfinite(spacingXMeters) || !(spacingXMeters > 0.0F)
        || !std::isfinite(spacingYMeters) || !(spacingYMeters > 0.0F))
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
    const float north = std::lerp(
        HeightAt(cell.northRow, cell.westColumn), HeightAt(cell.northRow, eastColumn),
        cell.eastFraction);
    const float south = std::lerp(
        HeightAt(southRow, cell.westColumn), HeightAt(southRow, eastColumn),
        cell.eastFraction);
    return referenceElevationMeters + std::lerp(north, south, cell.southFraction);
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

    constexpr double radiansPerDegree = std::numbers::pi_v<double> / 180.0;
    const glm::dvec3 origin = Wgs84SurfacePositionEcef(
        centerLatitudeDegrees, centerLongitudeDegrees);
    const glm::dvec3 point = Wgs84SurfacePositionEcef(
        latitudeDegrees, longitudeDegrees);
    const glm::dvec3 delta = point - origin;
    const double originLatitudeRadians = centerLatitudeDegrees * radiansPerDegree;
    const double originLongitudeRadians = centerLongitudeDegrees * radiansPerDegree;
    const double sinLatitude = std::sin(originLatitudeRadians);
    const double cosLatitude = std::cos(originLatitudeRadians);
    const double sinLongitude = std::sin(originLongitudeRadians);
    const double cosLongitude = std::cos(originLongitudeRadians);
    return {
        -sinLongitude * delta.x + cosLongitude * delta.y,
        -sinLatitude * cosLongitude * delta.x
            - sinLatitude * sinLongitude * delta.y
            + cosLatitude * delta.z};
}
}
