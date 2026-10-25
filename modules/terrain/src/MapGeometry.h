#pragma once

#include "terrain/TerrainGrid.h"

#include <nlohmann/json.hpp>

#include <array>
#include <utility>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/geometric.hpp>

namespace aerosimlab::terrain::detail
{
inline std::vector<glm::dvec2> ClipPolygonToTerrain(
    std::vector<glm::dvec2> polygon, const TerrainGrid& terrain)
{
    const double halfX = (static_cast<double>(terrain.sampleCount) - 1.0)
        * static_cast<double>(terrain.spacingXMeters) * 0.5;
    const double halfY = (static_cast<double>(terrain.sampleCount) - 1.0)
        * static_cast<double>(terrain.spacingYMeters) * 0.5;
    const std::array<std::pair<int, double>, 4> bounds{{
        {0, -halfX}, {1, halfX}, {2, -halfY}, {3, halfY}}};

    for (const auto& [edge, bound] : bounds)
    {
        if (polygon.empty())
            break;

        const auto coordinate = [edge](const glm::dvec2& point)
        {
            return edge < 2 ? point.x : point.y;
        };
        const bool keepGreater = edge == 0 || edge == 2;
        const auto inside = [&](const glm::dvec2& point)
        {
            return keepGreater ? coordinate(point) >= bound
                : coordinate(point) <= bound;
        };
        const auto intersection = [&](const glm::dvec2& first,
            const glm::dvec2& second)
        {
            const double amount = (bound - coordinate(first))
                / (coordinate(second) - coordinate(first));
            return first + (second - first) * amount;
        };

        std::vector<glm::dvec2> clipped;
        clipped.reserve(polygon.size() + 2U);
        glm::dvec2 previous = polygon.back();
        bool previousInside = inside(previous);
        for (const glm::dvec2& current : polygon)
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

    for (const auto& point : ring)
    {
        if (!point.is_array() || point.size() < 2U
            || !point[0].is_number() || !point[1].is_number())
            return false;

        const glm::dvec2 local = terrain.ToLocalMeters(
            point[1].get<double>(), point[0].get<double>());
        if (!clipToTerrain && !terrain.ContainsLocalPositionMeters(local))
            return false;
        outlineMeters.push_back(local);
    }

    if (outlineMeters.size() >= 2U
        && glm::length(outlineMeters.front() - outlineMeters.back()) < 0.01)
    {
        outlineMeters.pop_back();
    }

    if (clipToTerrain)
        outlineMeters = ClipPolygonToTerrain(std::move(outlineMeters), terrain);
    if (outlineMeters.size() < 3U)
        return false;

    groundElevationsMeters.clear();
    groundElevationsMeters.reserve(outlineMeters.size());
    for (const glm::dvec2& point : outlineMeters)
    {
        if (!terrain.ContainsLocalPositionMeters(point))
            return false;
        groundElevationsMeters.push_back(terrain.HeightAtLocalMeters(point));
    }
    return true;
}
}
