#include "terrain/MapBuildings.h"

#include "MapGeometry.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
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

namespace
{
std::vector<MapBuilding> LoadMapBuildings(
    const std::filesystem::path& path,
    const TerrainGrid& terrain,
    bool keepFootprintsWithoutHeight)
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
        building.name = tags->value("name", std::string{});
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
        if (building.heightMeters > 0.0 || keepFootprintsWithoutHeight)
            buildings.push_back(std::move(building));
    }
    return buildings;
}
}

std::vector<MapBuilding> LoadMapBuildings(
    const std::filesystem::path& path, const TerrainGrid& terrain)
{
    return LoadMapBuildings(path, terrain, false);
}

std::vector<MapBuilding> LoadMapBuildingFootprints(
    const std::filesystem::path& path, const TerrainGrid& terrain)
{
    // 높이는 모르는 상태로 두고, 경로를 막는 평면 외곽선만 사용한다.
    return LoadMapBuildings(path, terrain, true);
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

bool ContainsMapBuildingVolume(
    const MapBuilding& building, const glm::dvec3& positionMeters) noexcept
{
    if (building.outlineMeters.size() < 3U
        || building.outlineMeters.size() != building.groundElevationsMeters.size()
        || !std::isfinite(building.heightMeters)
        || !(building.heightMeters > 0.0)
        || !std::isfinite(positionMeters.x)
        || !std::isfinite(positionMeters.y)
        || !std::isfinite(positionMeters.z))
        return false;

    double groundMinimum = building.groundElevationsMeters.front();
    double groundMaximum = groundMinimum;
    for (const double elevation : building.groundElevationsMeters)
    {
        if (!std::isfinite(elevation))
            return false;
        groundMinimum = std::min(groundMinimum, elevation);
        groundMaximum = std::max(groundMaximum, elevation);
    }
    if (positionMeters.z < groundMinimum
        || positionMeters.z > groundMaximum + building.heightMeters)
        return false;

    return detail::ContainsPointInRing(
        building.outlineMeters, {positionMeters.x, positionMeters.y});
}

std::optional<glm::dvec3> FindFirstPointOutsideMapBuildings(
    const glm::dvec3& startMeters,
    const glm::dvec3& endMeters,
    const std::vector<MapBuilding>& buildings,
    double sampleSpacingMeters)
{
    if (!std::isfinite(sampleSpacingMeters) || !(sampleSpacingMeters > 0.0))
        throw std::invalid_argument("위치 확인 간격은 0보다 커야 합니다.");
    if (!std::isfinite(startMeters.x) || !std::isfinite(startMeters.y)
        || !std::isfinite(startMeters.z) || !std::isfinite(endMeters.x)
        || !std::isfinite(endMeters.y) || !std::isfinite(endMeters.z))
        throw std::invalid_argument("건물 밖 위치를 찾을 선분을 확인해 주세요.");

    const glm::dvec3 segment = endMeters - startMeters;
    const double lengthMeters = glm::length(segment);
    const auto isInsideBuilding = [&buildings](const glm::dvec3& position)
    {
        return std::any_of(buildings.begin(), buildings.end(),
            [&position](const MapBuilding& building)
            {
                return ContainsMapBuildingVolume(building, position);
            });
    };
    if (!isInsideBuilding(startMeters))
        return startMeters;
    if (!(lengthMeters > 0.0))
        return std::nullopt;

    const std::size_t sampleCount = static_cast<std::size_t>(
        std::ceil(lengthMeters / sampleSpacingMeters));
    for (std::size_t sample = 1U; sample <= sampleCount; ++sample)
    {
        const double fraction = std::min(
            1.0, static_cast<double>(sample) * sampleSpacingMeters / lengthMeters);
        const glm::dvec3 candidate = startMeters + segment * fraction;
        if (!isInsideBuilding(candidate))
            return candidate;
    }
    return std::nullopt;
}

MapBuildingFacade GetNearestMapBuildingFacade(
    const MapBuilding& building, const glm::dvec2& approachPositionMeters)
{
    if (building.outlineMeters.size() < 3U
        || building.outlineMeters.size() != building.groundElevationsMeters.size()
        || !std::isfinite(building.heightMeters) || !(building.heightMeters > 0.0)
        || !std::isfinite(approachPositionMeters.x)
        || !std::isfinite(approachPositionMeters.y))
    {
        throw std::invalid_argument("건물 외곽선과 접근 위치를 확인해 주세요.");
    }

    for (std::size_t index = 0; index < building.outlineMeters.size(); ++index)
    {
        const glm::dvec2& point = building.outlineMeters[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y)
            || !std::isfinite(building.groundElevationsMeters[index]))
            throw std::invalid_argument("건물 외곽선의 좌표와 높이를 확인해 주세요.");
    }

    double twiceSignedArea = 0.0;
    for (std::size_t index = 0; index < building.outlineMeters.size(); ++index)
    {
        const glm::dvec2& current = building.outlineMeters[index];
        const glm::dvec2& next = building.outlineMeters[
            (index + 1U) % building.outlineMeters.size()];
        twiceSignedArea += current.x * next.y - next.x * current.y;
    }
    if (!std::isfinite(twiceSignedArea) || std::abs(twiceSignedArea) < 1.0e-9)
        throw std::invalid_argument("건물 외곽선의 면적이 없습니다.");

    const bool counterClockwise = twiceSignedArea > 0.0;
    double nearestDistanceSquared = std::numeric_limits<double>::infinity();
    MapBuildingFacade nearestFacade;
    for (std::size_t index = 0; index < building.outlineMeters.size(); ++index)
    {
        const std::size_t nextIndex =
            (index + 1U) % building.outlineMeters.size();
        const glm::dvec2& start = building.outlineMeters[index];
        const glm::dvec2 edge = building.outlineMeters[nextIndex] - start;
        const double edgeLengthSquared = glm::dot(edge, edge);
        if (!(edgeLengthSquared > 1.0e-12))
            continue;

        const double fraction = std::clamp(glm::dot(
            approachPositionMeters - start, edge) / edgeLengthSquared, 0.0, 1.0);
        const glm::dvec2 closestPoint = start + edge * fraction;
        const glm::dvec2 offset = approachPositionMeters - closestPoint;
        const double distanceSquared = glm::dot(offset, offset);
        if (distanceSquared >= nearestDistanceSquared)
            continue;

        const double edgeLength = std::sqrt(edgeLengthSquared);
        const glm::dvec2 widthAxis = edge / edgeLength;
        const glm::dvec2 outwardNormal = counterClockwise
            ? glm::dvec2{widthAxis.y, -widthAxis.x}
            : glm::dvec2{-widthAxis.y, widthAxis.x};
        const glm::dvec2 edgeCenter = (start + building.outlineMeters[nextIndex]) * 0.5;
        const double groundElevation = std::lerp(
            building.groundElevationsMeters[index],
            building.groundElevationsMeters[nextIndex], 0.5);

        nearestDistanceSquared = distanceSquared;
        nearestFacade = {
            {edgeCenter.x, edgeCenter.y,
                groundElevation + building.heightMeters * 0.5},
            {outwardNormal.x, outwardNormal.y, 0.0},
            {widthAxis.x, widthAxis.y, 0.0},
            edgeLength,
            building.heightMeters};
    }

    if (!std::isfinite(nearestDistanceSquared))
        throw std::invalid_argument("건물 외곽선에 유효한 변이 없습니다.");
    return nearestFacade;
}
}
