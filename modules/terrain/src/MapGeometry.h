#pragma once

#include "terrain/TerrainGrid.h"

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/geometric.hpp>

namespace aerosimlab::terrain::detail
{
using PolygonPoint = std::array<double, 2>;

inline double Coordinate(const PolygonPoint& point, int edge) noexcept
{
    return edge < 2 ? point[0] : point[1];
}

inline std::vector<PolygonPoint> ClipPolygonToTerrain(
    std::vector<PolygonPoint> polygon, const TerrainGrid& terrain)
{
    if (polygon.size() < 3U)
        return {};

    const double halfX = (static_cast<double>(terrain.sampleCount) - 1.0)
        * static_cast<double>(terrain.spacingXMeters) * 0.5;
    const double halfY = (static_cast<double>(terrain.sampleCount) - 1.0)
        * static_cast<double>(terrain.spacingYMeters) * 0.5;
    const std::array<std::pair<int, double>, 4> bounds{{
        {0, -halfX}, {1, halfX}, {2, -halfY}, {3, halfY}}};

    for (const auto& [edge, bound] : bounds)
    {
        const bool keepGreater = edge == 0 || edge == 2;
        const auto inside = [&](const PolygonPoint& point)
        {
            const double coordinate = Coordinate(point, edge);
            return keepGreater ? coordinate >= bound : coordinate <= bound;
        };
        const auto intersection = [&](const PolygonPoint& first,
            const PolygonPoint& second)
        {
            const double firstCoordinate = Coordinate(first, edge);
            const double secondCoordinate = Coordinate(second, edge);
            const double amount = (bound - firstCoordinate)
                / (secondCoordinate - firstCoordinate);
            return PolygonPoint{
                first[0] + (second[0] - first[0]) * amount,
                first[1] + (second[1] - first[1]) * amount};
        };

        std::vector<PolygonPoint> clipped;
        clipped.reserve(polygon.size() + 2U);
        PolygonPoint previous = polygon.back();
        bool previousInside = inside(previous);
        for (const PolygonPoint& current : polygon)
        {
            const bool currentInside = inside(current);
            if (currentInside)
            {
                if (!previousInside)
                    clipped.push_back(intersection(previous, current));
                clipped.push_back(current);
            }
            else if (previousInside)
            {
                clipped.push_back(intersection(previous, current));
            }
            previous = current;
            previousInside = currentInside;
        }
        if (clipped.size() < 3U)
            return {};
        polygon = std::move(clipped);
    }
    return polygon;
}

/** @brief GeoJSON 다각형의 바깥 테두리를 지역 좌표와 지면 높이로 바꾼다. */
inline bool ReadTerrainFootprint(
    const nlohmann::json& coordinates,
    const TerrainGrid& terrain,
    std::vector<glm::dvec2>& outlineMeters,
    std::vector<double>& groundElevationsMeters,
    bool clipToTerrain = false)
{
    if (!coordinates.is_array() || coordinates.empty())
        return false;

    const auto& ring = coordinates.front();
    if (!ring.is_array() || ring.size() < 4U)
        return false;

    std::vector<PolygonPoint> polygon;
    polygon.reserve(ring.size());
    for (const auto& point : ring)
    {
        if (!point.is_array() || point.size() < 2U
            || !point[0].is_number() || !point[1].is_number())
            return false;

        const glm::dvec2 local = terrain.ToLocalMeters(
            point[1].get<double>(), point[0].get<double>());
        if (!clipToTerrain && !terrain.ContainsLocalPositionMeters(local))
            return false;
        polygon.push_back({local.x, local.y});
    }

    if (polygon.size() >= 2U)
    {
        const double deltaX = polygon.front()[0] - polygon.back()[0];
        const double deltaY = polygon.front()[1] - polygon.back()[1];
        if (std::hypot(deltaX, deltaY) < 0.01)
            polygon.pop_back();
    }

    if (clipToTerrain)
        polygon = ClipPolygonToTerrain(std::move(polygon), terrain);
    if (polygon.size() < 3U)
        return false;

    outlineMeters.clear();
    outlineMeters.reserve(polygon.size());
    groundElevationsMeters.clear();
    groundElevationsMeters.reserve(polygon.size());
    for (const PolygonPoint& coordinate : polygon)
    {
        const glm::dvec2 point{coordinate[0], coordinate[1]};
        if (!terrain.ContainsLocalPositionMeters(point))
            return false;
        outlineMeters.push_back(point);
        groundElevationsMeters.push_back(terrain.HeightAtLocalMeters(point));
    }
    return true;
}
}
