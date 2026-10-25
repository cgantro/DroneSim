#include "terrain/MapRoads.h"

#include <nlohmann/json.hpp>

#include <charconv>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace aerosimlab::terrain
{
namespace
{
using Json = nlohmann::json;
constexpr double metersPerLane = 3.5;
constexpr double maximumRoadWidthMeters = 100.0;

double ReadPositiveMeters(const Json& value)
{
    if (value.is_number())
    {
        const double meters = value.get<double>();
        return std::isfinite(meters) && meters > 0.0
            && meters <= maximumRoadWidthMeters ? meters : 0.0;
    }
    if (!value.is_string())
        return 0.0;

    const std::string text = value.get<std::string>();
    const char* begin = text.data();
    const char* end = begin + text.size();
    while (begin != end && (*begin == ' ' || *begin == '\t'))
        ++begin;

    double meters = 0.0;
    auto [numberEnd, error] = std::from_chars(
        begin, end, meters, std::chars_format::general);
    if (error != std::errc{} || numberEnd == begin)
        return 0.0;

    while (numberEnd != end && (*numberEnd == ' ' || *numberEnd == '\t'
        || *numberEnd == 'm'))
        ++numberEnd;
    return numberEnd == end && std::isfinite(meters) && meters > 0.0
        && meters <= maximumRoadWidthMeters
        ? meters : 0.0;
}

std::pair<double, RoadWidthSource> RoadWidth(const Json& tags)
{
    if (tags.contains("width"))
    {
        const double width = ReadPositiveMeters(tags.at("width"));
        if (width > 0.0)
            return {width, RoadWidthSource::OsmWidthTag};
    }

    if (tags.contains("lanes"))
    {
        const double lanes = ReadPositiveMeters(tags.at("lanes"));
        if (lanes > 0.0 && lanes <= 20.0)
            return {lanes * metersPerLane, RoadWidthSource::EstimatedFromLanes};
    }

    const std::string highway = tags.value("highway", std::string{});
    if (highway == "motorway" || highway == "trunk" || highway == "primary")
        return {8.0, RoadWidthSource::EstimatedFromRoadType};
    if (highway == "secondary" || highway == "tertiary")
        return {6.0, RoadWidthSource::EstimatedFromRoadType};
    if (highway == "residential" || highway == "unclassified"
        || highway == "living_street")
        return {4.0, RoadWidthSource::EstimatedFromRoadType};
    if (highway == "service")
        return {3.0, RoadWidthSource::EstimatedFromRoadType};
    return {2.0, RoadWidthSource::EstimatedFromRoadType};
}

std::string FeatureName(const Json& tags)
{
    for (const char* key : {"bridge:name", "name:ko", "name"})
    {
        const auto value = tags.find(key);
        if (value != tags.end() && value->is_string())
        {
            const std::string name = value->get<std::string>();
            if (!name.empty())
                return name;
        }
    }
    return {};
}

void AddInBoundsSegments(
    const Json& coordinates,
    const Json& properties,
    const Json& tags,
    const TerrainGrid& terrain,
    std::vector<MapRoad>& roads)
{
    if (!coordinates.is_array() || coordinates.size() < 2U)
        return;

    const auto [widthMeters, widthSource] = RoadWidth(tags);
    const bool isBridge = tags.contains("bridge")
        && tags.at("bridge") != "no" && tags.at("bridge") != false;
    const std::string osmId = properties.value("osm_id", std::string{});
    const std::string name = FeatureName(tags);
    std::vector<glm::dvec3> segment;
    segment.reserve(coordinates.size());

    const auto finishSegment = [&segment, &roads, osmId, name, widthMeters,
        widthSource, isBridge, pointCount = coordinates.size()]
    {
        if (segment.size() >= 2U)
        {
            MapRoad road;
            road.osmId = osmId;
            road.name = name;
            road.widthMeters = widthMeters;
            road.widthSource = widthSource;
            road.bridge = isBridge;
            road.centerlineMeters.swap(segment);
            roads.push_back(std::move(road));
            segment.reserve(pointCount);
        }
        else
        {
            segment.clear();
        }
    };

    for (const Json& point : coordinates)
    {
        if (!point.is_array() || point.size() < 2U
            || !point[0].is_number() || !point[1].is_number())
        {
            finishSegment();
            continue;
        }

        const glm::dvec2 local = terrain.ToLocalMeters(
            point[1].get<double>(), point[0].get<double>());
        if (!terrain.ContainsLocalPositionMeters(local))
        {
            finishSegment();
            continue;
        }

        segment.emplace_back(local.x, local.y,
            terrain.HeightAtLocalMeters(local));
    }
    finishSegment();
}
}

std::vector<MapRoad> LoadMapRoads(
    const std::filesystem::path& path, const TerrainGrid& terrain)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("OpenStreetMap 도로 자료를 열지 못했습니다: "
            + path.string());

    Json document;
    try
    {
        input >> document;
    }
    catch (const Json::exception&)
    {
        throw std::runtime_error("OpenStreetMap 도로 자료의 JSON 형식을 읽지 못했습니다.");
    }

    const auto features = document.find("features");
    if (features == document.end() || !features->is_array())
        throw std::runtime_error("OpenStreetMap 자료에 도로 형상이 없습니다.");

    std::vector<MapRoad> roads;
    for (const Json& feature : *features)
    {
        const auto geometry = feature.find("geometry");
        const auto properties = feature.find("properties");
        if (geometry == feature.end() || properties == feature.end()
            || !geometry->is_object() || !properties->is_object()
            || geometry->value("type", std::string{}) != "LineString")
            continue;

        const auto tags = properties->find("tags");
        if (tags == properties->end() || !tags->is_object()
            || !tags->contains("highway"))
            continue;

        AddInBoundsSegments(geometry->value("coordinates", Json{}),
            *properties, *tags, terrain, roads);
    }
    return roads;
}
}
