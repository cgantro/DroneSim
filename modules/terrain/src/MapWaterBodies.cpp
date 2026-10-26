#include "terrain/MapWaterBodies.h"

#include "MapGeometry.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <glm/geometric.hpp>

namespace aerosimlab::terrain
{
namespace
{
using Json = nlohmann::json;

std::vector<glm::dvec2> ReadRing(const Json& coordinates, const TerrainGrid& terrain)
{
    if (!coordinates.is_array() || coordinates.size() < 4U)
        return {};

    std::vector<glm::dvec2> ring;
    ring.reserve(coordinates.size());
    for (const Json& point : coordinates)
    {
        if (!point.is_array() || point.size() < 2U || !point[0].is_number() ||
            !point[1].is_number())
            return {};
        ring.push_back(terrain.ToLocalMeters(point[1].get<double>(), point[0].get<double>()));
    }

    if (glm::length(ring.front() - ring.back()) < 0.01)
        ring.pop_back();
    return ring.size() >= 3U ? ring : std::vector<glm::dvec2>{};
}

bool IntersectsTerrain(const std::vector<glm::dvec2>& ring, const TerrainGrid& terrain)
{
    if (ring.empty())
        return false;

    double minimumX = ring.front().x;
    double maximumX = ring.front().x;
    double minimumY = ring.front().y;
    double maximumY = ring.front().y;
    for (const glm::dvec2& point : ring)
    {
        minimumX = std::min(minimumX, point.x);
        maximumX = std::max(maximumX, point.x);
        minimumY = std::min(minimumY, point.y);
        maximumY = std::max(maximumY, point.y);
    }

    const double halfX = (static_cast<double>(terrain.sampleCount) - 1.0) *
                         static_cast<double>(terrain.spacingXMeters) * 0.5;
    const double halfY = (static_cast<double>(terrain.sampleCount) - 1.0) *
                         static_cast<double>(terrain.spacingYMeters) * 0.5;
    return minimumX <= halfX && maximumX >= -halfX && minimumY <= halfY && maximumY >= -halfY;
}

} // namespace

std::vector<MapWaterBody> LoadMapWaterBodies(const std::filesystem::path& path,
                                             const TerrainGrid& terrain)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("OpenStreetMap 물 영역 자료를 열지 못했습니다: " + path.string());

    Json document;
    try
    {
        input >> document;
    }
    catch (const Json::exception&)
    {
        throw std::runtime_error("OpenStreetMap 물 영역 자료의 JSON 형식을 읽지 못했습니다.");
    }

    const auto features = document.find("features");
    if (features == document.end() || !features->is_array())
        throw std::runtime_error("OpenStreetMap 자료에 물 영역 형상이 없습니다.");

    std::vector<MapWaterBody> waterBodies;
    for (const Json& feature : *features)
    {
        const auto geometry = feature.find("geometry");
        const auto properties = feature.find("properties");
        if (geometry == feature.end() || properties == feature.end() || !geometry->is_object() ||
            !properties->is_object() || geometry->value("type", std::string{}) != "Polygon")
            continue;

        const auto tags = properties->find("tags");
        if (tags == properties->end() || !tags->is_object() ||
            tags->value("natural", std::string{}) != "water")
            continue;

        const Json coordinates = geometry->value("coordinates", Json{});
        if (!coordinates.is_array() || coordinates.empty())
            continue;

        MapWaterBody water;
        water.outerBoundaryMeters = ReadRing(coordinates.front(), terrain);
        if (!IntersectsTerrain(water.outerBoundaryMeters, terrain))
            continue;

        for (std::size_t index = 1U; index < coordinates.size(); ++index)
        {
            std::vector<glm::dvec2> inner = ReadRing(coordinates[index], terrain);
            if (!inner.empty())
                water.innerBoundariesMeters.push_back(std::move(inner));
        }
        waterBodies.push_back(std::move(water));
    }
    return waterBodies;
}

bool ContainsWater(const MapWaterBody& water, const glm::dvec2& pointMeters) noexcept
{
    if (!detail::ContainsPointInRing(water.outerBoundaryMeters, pointMeters))
        return false;
    return std::none_of(water.innerBoundariesMeters.begin(), water.innerBoundariesMeters.end(),
        [&](const auto& inner)
        {
            return detail::ContainsPointInRing(inner, pointMeters);
        });
}
} // namespace aerosimlab::terrain
