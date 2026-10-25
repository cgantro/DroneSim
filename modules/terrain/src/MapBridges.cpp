#include "terrain/MapBridges.h"

#include "MapGeometry.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace aerosimlab::terrain
{
namespace
{
using Json = nlohmann::json;

std::string BridgeName(const Json& tags)
{
    for (const char* key : {"name:ko", "name", "name:en"})
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
}

std::vector<MapBridgeFootprint> LoadMapBridgeFootprints(
    const std::filesystem::path& path, const TerrainGrid& terrain)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("OpenStreetMap 교량 자료를 열지 못했습니다: "
            + path.string());

    Json document;
    try
    {
        input >> document;
    }
    catch (const Json::exception&)
    {
        throw std::runtime_error("OpenStreetMap 교량 자료의 JSON 형식을 읽지 못했습니다.");
    }

    const auto features = document.find("features");
    if (features == document.end() || !features->is_array())
        throw std::runtime_error("OpenStreetMap 자료에 교량 형상이 없습니다.");

    std::vector<MapBridgeFootprint> bridges;
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
            || tags->value("man_made", std::string{}) != "bridge")
            continue;

        MapBridgeFootprint bridge;
        bridge.osmId = properties->value("osm_id", std::string{});
        bridge.name = BridgeName(*tags);
        if (bridge.name.empty())
            continue;
        if (!detail::ReadTerrainFootprint(
            geometry->value("coordinates", Json{}), terrain,
            bridge.outlineMeters, bridge.groundElevationsMeters, true))
            continue;
        bridges.push_back(std::move(bridge));
    }
    return bridges;
}
}
