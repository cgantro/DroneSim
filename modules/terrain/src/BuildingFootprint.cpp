#include "terrain/BuildingFootprint.h"

#include <glm/common.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>

namespace aerosimlab::terrain
{
namespace
{
using Json = nlohmann::json;

Json ReadBuildingDocument(const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("건물 외곽선 자료를 열지 못했습니다: " + path.string());

    Json document;
    try
    {
        input >> document;
    }
    catch (const Json::exception& error)
    {
        throw std::runtime_error(std::string("건물 외곽선 자료를 읽지 못했습니다: ")
            + error.what());
    }
    return document;
}

void ValidateFeatureCollection(const Json& document)
{
    if (document.value("type", std::string{}) != "FeatureCollection"
        || !document.contains("features") || !document.at("features").is_array())
    {
        throw std::runtime_error("건물 외곽선 자료가 GeoJSON FeatureCollection이 아닙니다.");
    }
}

const Json& ReadPolygonRing(const Json& geometry)
{
    if (geometry.value("type", std::string{}) != "Polygon"
        || !geometry.contains("coordinates") || !geometry.at("coordinates").is_array()
        || geometry.at("coordinates").empty())
    {
        throw std::runtime_error("건물 외곽선은 좌표가 있는 Polygon이어야 합니다.");
    }
    const Json& ring = geometry.at("coordinates").at(0);
    if (!ring.is_array() || ring.size() < 4U)
        throw std::runtime_error("건물 외곽선에 꼭짓점이 충분하지 않습니다.");
    return ring;
}

std::vector<glm::dvec2> ReadOutlineMeters(
    const Json& ring, const TerrainGrid& terrain)
{
    std::vector<glm::dvec2> outlineMeters;
    outlineMeters.reserve(ring.size());
    for (const Json& coordinate : ring)
    {
        if (!coordinate.is_array() || coordinate.size() < 2U)
            throw std::runtime_error("건물 외곽선 좌표에 위도 또는 경도가 없습니다.");
        const double longitude = coordinate.at(0).get<double>();
        const double latitude = coordinate.at(1).get<double>();
        outlineMeters.push_back(terrain.ToLocalMeters(latitude, longitude));
    }

    const glm::dvec2 closingError = outlineMeters.front() - outlineMeters.back();
    if (std::abs(closingError.x) > 1.0e-6 || std::abs(closingError.y) > 1.0e-6)
        throw std::runtime_error("건물 외곽선의 시작점과 끝점이 닫혀 있지 않습니다.");
    return outlineMeters;
}

std::optional<float> ReadBuildingHeight(const Json& properties)
{
    const Json& height = properties.at("height_m");
    if (height.is_null())
        return std::nullopt;

    const float heightMeters = height.get<float>();
    if (!std::isfinite(heightMeters) || !(heightMeters > 0.0F))
        throw std::runtime_error("건물 높이는 0보다 큰 유한한 미터 값이어야 합니다.");
    return heightMeters;
}

BuildingFootprint ReadBuildingFeature(const Json& feature, const TerrainGrid& terrain)
{
    const Json& properties = feature.at("properties");
    const Json& ring = ReadPolygonRing(feature.at("geometry"));

    BuildingFootprint building;
    building.osmId = properties.value("osm_id", std::string{});
    building.buildingType = properties.value("building", std::string{});
    if (building.osmId.empty() || building.buildingType.empty())
        throw std::runtime_error("건물 외곽선에 OpenStreetMap ID나 종류가 없습니다.");

    building.outlineMeters = ReadOutlineMeters(ring, terrain);
    building.heightMeters = ReadBuildingHeight(properties);
    return building;
}
}

std::vector<BuildingFootprint> LoadBuildingFootprints(
    const std::filesystem::path& path, const TerrainGrid& terrain)
{
    const Json document = ReadBuildingDocument(path);
    ValidateFeatureCollection(document);

    std::vector<BuildingFootprint> buildings;
    buildings.reserve(document.at("features").size());
    for (const Json& feature : document.at("features"))
        buildings.push_back(ReadBuildingFeature(feature, terrain));

    return buildings;
}

BuildingBoundsMeters MakeBuildingBoundsMeters(
    const BuildingFootprint& building, const TerrainGrid& terrain,
    float assumedHeightMeters)
{
    glm::dvec2 minimum = building.outlineMeters.front();
    glm::dvec2 maximum = minimum;
    for (const glm::dvec2& point : building.outlineMeters)
    {
        minimum = glm::min(minimum, point);
        maximum = glm::max(maximum, point);
    }

    const glm::dvec2 footprintCenter = (minimum + maximum) * 0.5;
    const float heightMeters = building.heightMeters.value_or(assumedHeightMeters);
    const double groundElevationMeters = terrain.HeightAtLocalMeters(footprintCenter);
    return {
        {footprintCenter.x, footprintCenter.y,
            groundElevationMeters + static_cast<double>(heightMeters) * 0.5},
        {(maximum.x - minimum.x) * 0.5,
            (maximum.y - minimum.y) * 0.5,
            static_cast<double>(heightMeters) * 0.5}};
}
}
