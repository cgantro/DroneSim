#include "terrain/MapBuildings.h"

#include "MapGeometry.h"

#include <nlohmann/json.hpp>

#include <charconv>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace aerosimlab::terrain
{
namespace
{
using Json = nlohmann::json;
constexpr double metersPerBuildingLevel = 3.0;

double ReadPositiveNumber(const Json& value)
{
    if (value.is_number())
    {
        const double number = value.get<double>();
        return std::isfinite(number) && number > 0.0 ? number : 0.0;
    }
    if (!value.is_string())
        return 0.0;

    const std::string text = value.get<std::string>();
    const char* begin = text.data();
    double number = 0.0;
    while (begin != text.data() + text.size()
        && (*begin == ' ' || *begin == '\t'))
        ++begin;

    const auto [end, error] = std::from_chars(
        begin, text.data() + text.size(), number, std::chars_format::general);
    if (error != std::errc{} || end == begin)
        return 0.0;

    const char* remaining = end;
    const char* textEnd = text.data() + text.size();
    while (remaining != textEnd
        && (*remaining == ' ' || *remaining == '\t' || *remaining == 'm'))
        ++remaining;
    return remaining == textEnd && std::isfinite(number) && number > 0.0
        ? number : 0.0;
}

}

std::vector<MapBuilding> LoadMapBuildings(
    const std::filesystem::path& path, const TerrainGrid& terrain)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("OpenStreetMap 건물 자료를 열지 못했습니다: " + path.string());

    Json document;
    try
    {
        input >> document;
    }
    catch (const Json::exception&)
    {
        throw std::runtime_error("OpenStreetMap 건물 자료의 JSON 형식을 읽지 못했습니다.");
    }

    const auto features = document.find("features");
    if (features == document.end() || !features->is_array())
        throw std::runtime_error("OpenStreetMap 자료에 공간 형상이 없습니다.");

    std::vector<MapBuilding> buildings;
    for (const Json& feature : *features)
    {
        const auto geometry = feature.find("geometry");
        const auto properties = feature.find("properties");
        if (geometry == feature.end() || properties == feature.end()
            || !geometry->is_object() || !properties->is_object()
            || geometry->value("type", std::string{}) != "Polygon")
            continue;

        const auto tags = properties->find("tags");
        if (tags == properties->end() || !tags->is_object()
            || !tags->contains("building"))
            continue;

        MapBuilding building;
        building.osmId = properties->value("osm_id", std::string{});
        if (!detail::ReadTerrainFootprint(
            geometry->value("coordinates", Json{}), terrain,
            building.outlineMeters, building.groundElevationsMeters))
            continue;

        if (tags->contains("height"))
            building.heightMeters = ReadPositiveNumber(tags->at("height"));
        if (!(building.heightMeters > 0.0) && tags->contains("building:levels"))
        {
            building.heightMeters = ReadPositiveNumber(tags->at("building:levels"))
                * metersPerBuildingLevel;
            if (building.heightMeters > 0.0)
                building.heightSource = BuildingHeightSource::EstimatedFromLevels;
        }
        if (building.heightMeters > 0.0)
            buildings.push_back(std::move(building));
    }
    return buildings;
}

MapBuildingBoundsMeters GetMapBuildingBoundsMeters(const MapBuilding& building)
{
    glm::dvec2 minimum = building.outlineMeters.front();
    glm::dvec2 maximum = minimum;
    double groundMinimum = building.groundElevationsMeters.front();
    double groundMaximum = groundMinimum;
    for (std::size_t index = 1; index < building.outlineMeters.size(); ++index)
    {
        minimum = glm::min(minimum, building.outlineMeters[index]);
        maximum = glm::max(maximum, building.outlineMeters[index]);
        groundMinimum = std::min(groundMinimum, building.groundElevationsMeters[index]);
        groundMaximum = std::max(groundMaximum, building.groundElevationsMeters[index]);
    }

    const glm::dvec2 horizontalCenter = (minimum + maximum) * 0.5;
    const double topElevation = groundMaximum + building.heightMeters;
    return {
        {horizontalCenter.x, horizontalCenter.y, (groundMinimum + topElevation) * 0.5},
        {(maximum.x - minimum.x) * 0.5,
            (maximum.y - minimum.y) * 0.5,
            (topElevation - groundMinimum) * 0.5}};
}

MapBuildingFacade GetNearestMapBuildingFacade(
    const MapBuilding& building, const glm::dvec2& approachPositionMeters)
{
    glm::dvec2 minimum = building.outlineMeters.front();
    glm::dvec2 maximum = minimum;
    double groundMinimum = building.groundElevationsMeters.front();
    double groundMaximum = groundMinimum;
    for (std::size_t index = 1; index < building.outlineMeters.size(); ++index)
    {
        minimum = glm::min(minimum, building.outlineMeters[index]);
        maximum = glm::max(maximum, building.outlineMeters[index]);
        groundMinimum = std::min(groundMinimum, building.groundElevationsMeters[index]);
        groundMaximum = std::max(groundMaximum, building.groundElevationsMeters[index]);
    }

    const glm::dvec2 center = (minimum + maximum) * 0.5;
    const double height = building.heightMeters + groundMaximum - groundMinimum;
    const double centerHeight = groundMinimum + height * 0.5;
    const double distanceToWest = std::abs(approachPositionMeters.x - minimum.x);
    const double distanceToEast = std::abs(approachPositionMeters.x - maximum.x);
    const double distanceToSouth = std::abs(approachPositionMeters.y - minimum.y);
    const double distanceToNorth = std::abs(approachPositionMeters.y - maximum.y);

    const double nearestDistance = std::min({
        distanceToWest, distanceToEast, distanceToSouth, distanceToNorth});
    if (nearestDistance == distanceToWest)
        return {{minimum.x, center.y, centerHeight}, {-1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}, maximum.y - minimum.y, height};
    if (nearestDistance == distanceToEast)
        return {{maximum.x, center.y, centerHeight}, {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}, maximum.y - minimum.y, height};
    if (nearestDistance == distanceToSouth)
        return {{center.x, minimum.y, centerHeight}, {0.0, -1.0, 0.0},
            {1.0, 0.0, 0.0}, maximum.x - minimum.x, height};
    return {{center.x, maximum.y, centerHeight}, {0.0, 1.0, 0.0},
        {1.0, 0.0, 0.0}, maximum.x - minimum.x, height};
}
}
