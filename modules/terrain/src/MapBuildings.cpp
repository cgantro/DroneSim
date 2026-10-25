#include "terrain/MapBuildings.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

#include <glm/geometric.hpp>

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
    std::size_t consumed = 0;
    double number = 0.0;
    try
    {
        number = std::stod(text, &consumed);
    }
    catch (const std::exception&)
    {
        return 0.0;
    }
    while (consumed < text.size()
        && (text[consumed] == ' ' || text[consumed] == '\t'
            || text[consumed] == 'm'))
        ++consumed;
    return consumed == text.size() && std::isfinite(number) && number > 0.0
        ? number : 0.0;
}

bool ReadFootprint(
    const Json& coordinates,
    const TerrainGrid& terrain,
    MapBuilding& building)
{
    if (!coordinates.is_array() || coordinates.empty())
        return false;

    const Json& ring = coordinates.front();
    if (!ring.is_array() || ring.size() < 4U)
        return false;

    for (const Json& point : ring)
    {
        if (!point.is_array() || point.size() < 2U
            || !point[0].is_number() || !point[1].is_number())
            return false;

        const glm::dvec2 local = terrain.ToLocalMeters(
            point[1].get<double>(), point[0].get<double>());
        try
        {
            building.groundElevationsMeters.push_back(
                terrain.HeightAtLocalMeters(local));
        }
        catch (const std::out_of_range&)
        {
            return false;
        }
        building.outlineMeters.push_back(local);
    }

    if (glm::length(building.outlineMeters.front() - building.outlineMeters.back()) < 0.01)
    {
        building.outlineMeters.pop_back();
        building.groundElevationsMeters.pop_back();
    }
    return building.outlineMeters.size() >= 3U;
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
        if (!ReadFootprint(geometry->value("coordinates", Json{}), terrain, building))
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
}
