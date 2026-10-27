#include "terrain/MapBridges.h"

#include "MapGeometry.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <glm/geometric.hpp>

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

struct BridgeAxes
{
    glm::dvec2 lengthAxis{1.0, 0.0};
    glm::dvec2 widthAxis{0.0, 1.0};
    double minimumLength = 0.0;
    double maximumLength = 0.0;
    double minimumWidth = 0.0;
    double maximumWidth = 0.0;
};

BridgeAxes FindBridgeAxes(const std::vector<glm::dvec2>& outline)
{
    BridgeAxes best;
    double smallestRectangleArea = std::numeric_limits<double>::infinity();
    for (std::size_t edgeIndex = 0; edgeIndex < outline.size(); ++edgeIndex)
    {
        const glm::dvec2 edge = outline[(edgeIndex + 1U) % outline.size()]
            - outline[edgeIndex];
        const double edgeLength = glm::length(edge);
        if (edgeLength < 1.0e-6)
            continue;

        const glm::dvec2 firstAxis = edge / edgeLength;
        const glm::dvec2 secondAxis{-firstAxis.y, firstAxis.x};
        double minimumFirst = std::numeric_limits<double>::infinity();
        double maximumFirst = -minimumFirst;
        double minimumSecond = minimumFirst;
        double maximumSecond = -minimumFirst;
        for (const glm::dvec2& point : outline)
        {
            const double first = glm::dot(point, firstAxis);
            const double second = glm::dot(point, secondAxis);
            minimumFirst = std::min(minimumFirst, first);
            maximumFirst = std::max(maximumFirst, first);
            minimumSecond = std::min(minimumSecond, second);
            maximumSecond = std::max(maximumSecond, second);
        }

        const double firstLength = maximumFirst - minimumFirst;
        const double secondLength = maximumSecond - minimumSecond;
        const double area = firstLength * secondLength;
        if (area >= smallestRectangleArea)
            continue;

        smallestRectangleArea = area;
        if (firstLength >= secondLength)
        {
            best = {firstAxis, secondAxis, minimumFirst, maximumFirst,
                minimumSecond, maximumSecond};
        }
        else
        {
            best = {secondAxis, firstAxis, minimumSecond, maximumSecond,
                minimumFirst, maximumFirst};
        }
    }
    return best;
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

std::vector<MapBridgePier> EstimateMapBridgePiers(
    const MapBridgeFootprint& bridge,
    const TerrainGrid& terrain,
    double deckBottomElevationMeters)
{
    constexpr double maximumSpanMeters = 45.0;
    constexpr double pierLengthMeters = 2.4;
    constexpr double pierWidthMeters = 1.8;
    constexpr double pierAcrossBridgeFraction = 0.22;
    constexpr double maximumPierOffsetMeters = 8.0;

    if (bridge.outlineMeters.size() < 3U
        || !std::isfinite(deckBottomElevationMeters))
        return {};

    const BridgeAxes axes = FindBridgeAxes(bridge.outlineMeters);
    const double bridgeLength = axes.maximumLength - axes.minimumLength;
    const double bridgeWidth = axes.maximumWidth - axes.minimumWidth;
    if (bridgeLength < 60.0 || bridgeWidth < 8.0)
        return {};

    const std::size_t spanCount = std::max<std::size_t>(2U,
        static_cast<std::size_t>(std::ceil(bridgeLength / maximumSpanMeters)));
    const double spanLength = bridgeLength / static_cast<double>(spanCount);
    const double acrossOffset = std::min(
        bridgeWidth * pierAcrossBridgeFraction, maximumPierOffsetMeters);
    const double middleWidth = (axes.minimumWidth + axes.maximumWidth) * 0.5;
    const std::array<double, 2> sideOffsets{-acrossOffset, acrossOffset};

    std::vector<MapBridgePier> piers;
    piers.reserve((spanCount - 1U) * sideOffsets.size());
    for (std::size_t span = 1U; span < spanCount; ++span)
    {
        const double along = axes.minimumLength + spanLength * static_cast<double>(span);
        for (const double sideOffset : sideOffsets)
        {
            const glm::dvec2 center = axes.lengthAxis * along
                + axes.widthAxis * (middleWidth + sideOffset);
            const glm::dvec2 halfLength = axes.lengthAxis * (pierLengthMeters * 0.5);
            const glm::dvec2 halfWidth = axes.widthAxis * (pierWidthMeters * 0.5);
            const std::array<glm::dvec2, 4> corners{
                center - halfLength - halfWidth,
                center + halfLength - halfWidth,
                center + halfLength + halfWidth,
                center - halfLength + halfWidth};
            if (std::any_of(corners.begin(), corners.end(), [&](const auto& corner)
                {
                    return !detail::ContainsPointInRing(bridge.outlineMeters, corner)
                        || !terrain.ContainsLocalPositionMeters(corner);
                }))
                continue;

            const double bottom = terrain.HeightAtLocalMeters(center);
            if (bottom >= deckBottomElevationMeters)
                continue;

            piers.push_back({
                .outlineMeters = std::vector<glm::dvec2>(
                    corners.begin(), corners.end()),
                .bottomElevationMeters = bottom,
                .topElevationMeters = deckBottomElevationMeters});
        }
    }
    return piers;
}
}
