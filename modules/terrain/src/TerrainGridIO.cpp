#include "terrain/TerrainGridIO.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace dronesim::terrain
{
namespace
{
constexpr std::array<char, 8> terrainMagic{'A', 'E', 'R', 'O', 'T', 'R', 'N', '3'};
constexpr std::uint64_t headerSizeBytes = 24;

template <typename T>
bool ReadLittleEndian(std::ifstream& input, T& value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    // 저장 형식은 리틀엔디언으로 고정한다. 빅엔디언 CPU에서는 바이트를 뒤집어 읽는다.
    std::array<unsigned char, sizeof(T)> bytes{};
    if (!input.read(reinterpret_cast<char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size())))
    {
        return false;
    }
    if constexpr (std::endian::native == std::endian::big)
        std::reverse(bytes.begin(), bytes.end());
    std::memcpy(&value, bytes.data(), sizeof(T));
    return true;
}

template <typename T>
void WriteLittleEndian(std::ofstream& output, T value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    // 어느 CPU에서 저장해도 같은 파일이 되도록 바이트 순서를 통일한다.
    std::array<unsigned char, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
    if constexpr (std::endian::native == std::endian::big)
        std::reverse(bytes.begin(), bytes.end());
    output.write(reinterpret_cast<const char*>(bytes.data()),
        static_cast<std::streamsize>(bytes.size()));
}

bool HasValidMetadata(const TerrainGrid& terrain) noexcept
{
    return terrain.sampleCount >= 3U
        && std::isfinite(terrain.spacingXMeters) && terrain.spacingXMeters > 0.0F
        && std::isfinite(terrain.spacingYMeters) && terrain.spacingYMeters > 0.0F
        && std::isfinite(terrain.referenceElevationMeters);
}

void RequireFinitePositive(float value, const char* label)
{
    if (!std::isfinite(value) || !(value > 0.0F))
        throw std::invalid_argument(std::string("지형 ") + label + " 값은 0보다 커야 합니다.");
}
}

TerrainGrid LoadTerrainGrid(const std::filesystem::path& path)
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
        || !ReadLittleEndian(input, grid.referenceElevationMeters))
    {
        throw std::runtime_error("지형 자료의 형식이나 머리 정보가 올바르지 않습니다.");
    }

    if (!HasValidMetadata(grid))
    {
        RequireFinitePositive(grid.spacingXMeters, "동서 간격");
        RequireFinitePositive(grid.spacingYMeters, "남북 간격");
        throw std::runtime_error("지형 크기와 기준 높이를 확인해 주세요.");
    }

    const std::uint64_t sampleTotal =
        static_cast<std::uint64_t>(grid.sampleCount) * grid.sampleCount;
    const std::uint64_t expectedSize = headerSizeBytes + sampleTotal * sizeof(float);
    if (expectedSize != static_cast<std::uint64_t>(fileSize)
        || sampleTotal > std::numeric_limits<std::size_t>::max())
    {
        throw std::runtime_error("지형 자료의 높이 개수가 크기 정보와 맞지 않습니다.");
    }

    grid.heightOffsetsMeters.resize(static_cast<std::size_t>(sampleTotal));
    for (float& height : grid.heightOffsetsMeters)
    {
        if (!ReadLittleEndian(input, height) || !std::isfinite(height))
            throw std::runtime_error("지형 높이 자료가 잘렸거나 숫자가 아닙니다.");
    }
    return grid;
}

TerrainGrid LoadAsciiTerrainGrid(
    const std::filesystem::path& path, float referenceElevationMeters)
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
        || !std::isfinite(referenceElevationMeters))
    {
        throw std::runtime_error("지형 격자는 미터 단위의 정사각형 자료여야 합니다.");
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

void SaveTerrainGrid(const TerrainGrid& terrain,
    const std::filesystem::path& path)
{
    const std::size_t expectedSamples =
        static_cast<std::size_t>(terrain.sampleCount) * terrain.sampleCount;
    if (!HasValidMetadata(terrain)
        || terrain.heightOffsetsMeters.size() != expectedSamples
        || !std::all_of(terrain.heightOffsetsMeters.begin(),
            terrain.heightOffsetsMeters.end(),
            [](float height) { return std::isfinite(height); }))
    {
        throw std::invalid_argument("저장할 지형 격자와 기준 정보를 확인해 주세요.");
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("지형 자료를 저장할 수 없습니다: " + path.string());
    output.write(terrainMagic.data(), static_cast<std::streamsize>(terrainMagic.size()));
    WriteLittleEndian(output, terrain.sampleCount);
    WriteLittleEndian(output, terrain.spacingXMeters);
    WriteLittleEndian(output, terrain.spacingYMeters);
    WriteLittleEndian(output, terrain.referenceElevationMeters);
    for (const float height : terrain.heightOffsetsMeters)
        WriteLittleEndian(output, height);
    if (!output)
        throw std::runtime_error("지형 자료를 끝까지 저장하지 못했습니다.");
}
}
