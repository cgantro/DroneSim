#include "terrain/MapFlightRoute.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include <glm/geometric.hpp>

namespace aerosimlab::terrain
{
namespace
{
constexpr double routeCellSizeMeters = 5.0;
constexpr double minimumGroundClearanceMeters = 2.0;

struct Grid
{
    std::size_t columns = 0;
    std::size_t rows = 0;
    double westMeters = 0.0;
    double southMeters = 0.0;
    double spacingXMeters = 0.0;
    double spacingYMeters = 0.0;
};

struct Cell
{
    std::size_t row = 0;
    std::size_t column = 0;
};

double DistanceToSegmentSquared(
    const glm::dvec2& point,
    const glm::dvec2& start,
    const glm::dvec2& end)
{
    const glm::dvec2 edge = end - start;
    const double lengthSquared = glm::dot(edge, edge);
    const double fraction = lengthSquared > 0.0
        ? std::clamp(glm::dot(point - start, edge) / lengthSquared, 0.0, 1.0)
        : 0.0;
    const glm::dvec2 difference = point - (start + edge * fraction);
    return glm::dot(difference, difference);
}

bool IsInsidePolygon(const glm::dvec2& point, const std::vector<glm::dvec2>& polygon)
{
    bool inside = false;
    for (std::size_t index = 0, previous = polygon.size() - 1U;
         index < polygon.size(); previous = index++)
    {
        const glm::dvec2& start = polygon[index];
        const glm::dvec2& end = polygon[previous];
        const bool crosses = (start.y > point.y) != (end.y > point.y);
        if (crosses && point.x < (end.x - start.x) * (point.y - start.y)
                / (end.y - start.y) + start.x)
            inside = !inside;
    }
    return inside;
}

bool IsBlocked(
    const glm::dvec2& point,
    const std::vector<MapBuilding>& buildings,
    double clearanceSquared)
{
    for (const MapBuilding& building : buildings)
    {
        const auto& outline = building.outlineMeters;
        if (outline.size() < 3U)
            continue;
        if (IsInsidePolygon(point, outline))
            return true;
        for (std::size_t index = 0; index < outline.size(); ++index)
        {
            if (DistanceToSegmentSquared(point, outline[index],
                    outline[(index + 1U) % outline.size()]) < clearanceSquared)
                return true;
        }
    }
    return false;
}

bool SegmentIsClear(
    const glm::dvec2& start,
    const glm::dvec2& end,
    const std::vector<MapBuilding>& buildings,
    double clearanceSquared)
{
    const double length = glm::length(end - start);
    const std::size_t sampleCount = std::max<std::size_t>(1U,
        static_cast<std::size_t>(std::ceil(length)));
    for (std::size_t index = 0; index <= sampleCount; ++index)
    {
        const double fraction = static_cast<double>(index)
            / static_cast<double>(sampleCount);
        if (IsBlocked(glm::mix(start, end, fraction), buildings, clearanceSquared))
            return false;
    }
    return true;
}

std::size_t Index(const Grid& grid, Cell cell)
{
    return cell.row * grid.columns + cell.column;
}

glm::dvec2 Position(const Grid& grid, Cell cell)
{
    return {
        grid.westMeters + static_cast<double>(cell.column) * grid.spacingXMeters,
        grid.southMeters + static_cast<double>(cell.row) * grid.spacingYMeters};
}

Cell NearestCell(const Grid& grid, const glm::dvec2& position)
{
    const double column = std::round(
        (position.x - grid.westMeters) / grid.spacingXMeters);
    const double row = std::round(
        (position.y - grid.southMeters) / grid.spacingYMeters);
    return {
        static_cast<std::size_t>(std::clamp(row, 0.0,
            static_cast<double>(grid.rows - 1U))),
        static_cast<std::size_t>(std::clamp(column, 0.0,
            static_cast<double>(grid.columns - 1U)))};
}

std::optional<std::vector<Cell>> FindGridPath(
    const Grid& grid,
    Cell start,
    Cell goal,
    const std::vector<MapBuilding>& buildings,
    double clearanceSquared)
{
    const std::size_t cellCount = grid.rows * grid.columns;
    std::vector<double> distance(cellCount, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(cellCount, cellCount);
    std::vector<bool> closed(cellCount, false);
    using Candidate = std::pair<double, std::size_t>;
    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<>> pending;
    const std::size_t startIndex = Index(grid, start);
    const std::size_t goalIndex = Index(grid, goal);
    distance[startIndex] = 0.0;
    pending.emplace(0.0, startIndex);

    constexpr std::array<std::pair<int, int>, 8> neighbors{{
        {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
        {0, 1}, {1, -1}, {1, 0}, {1, 1}}};
    while (!pending.empty())
    {
        const std::size_t currentIndex = pending.top().second;
        pending.pop();
        if (closed[currentIndex])
            continue;
        closed[currentIndex] = true;
        if (currentIndex == goalIndex)
            break;

        const Cell current{
            currentIndex / grid.columns, currentIndex % grid.columns};
        for (const auto [rowOffset, columnOffset] : neighbors)
        {
            const auto row = static_cast<std::ptrdiff_t>(current.row) + rowOffset;
            const auto column = static_cast<std::ptrdiff_t>(current.column) + columnOffset;
            if (row < 0 || column < 0
                || row >= static_cast<std::ptrdiff_t>(grid.rows)
                || column >= static_cast<std::ptrdiff_t>(grid.columns))
                continue;

            const Cell next{static_cast<std::size_t>(row),
                static_cast<std::size_t>(column)};
            const glm::dvec2 nextPosition = Position(grid, next);
            if (IsBlocked(nextPosition, buildings, clearanceSquared))
                continue;
            if (rowOffset != 0 && columnOffset != 0)
            {
                const Cell horizontal{current.row, next.column};
                const Cell vertical{next.row, current.column};
                if (IsBlocked(Position(grid, horizontal), buildings, clearanceSquared)
                    || IsBlocked(Position(grid, vertical), buildings, clearanceSquared))
                    continue;
            }

            const std::size_t nextIndex = Index(grid, next);
            const double stepCost = rowOffset != 0 && columnOffset != 0
                ? routeCellSizeMeters * std::sqrt(2.0)
                : routeCellSizeMeters;
            const double candidateDistance = distance[currentIndex] + stepCost;
            if (candidateDistance >= distance[nextIndex])
                continue;

            previous[nextIndex] = currentIndex;
            distance[nextIndex] = candidateDistance;
            const double heuristic = glm::length(
                Position(grid, next) - Position(grid, goal));
            pending.emplace(candidateDistance + heuristic, nextIndex);
        }
    }

    if (!closed[goalIndex])
        return std::nullopt;

    std::vector<Cell> path;
    for (std::size_t current = goalIndex; current != startIndex;
         current = previous[current])
    {
        if (current == cellCount)
            return std::nullopt;
        path.push_back({current / grid.columns, current % grid.columns});
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<glm::dvec2> SimplifyPath(
    const std::vector<Cell>& gridPath,
    const Grid& grid,
    const glm::dvec2& start,
    const glm::dvec2& goal,
    const std::vector<MapBuilding>& buildings,
    double clearanceSquared)
{
    std::vector<glm::dvec2> anchors;
    anchors.reserve(gridPath.size() + 2U);
    anchors.push_back(start);
    for (std::size_t index = 0; index < gridPath.size(); ++index)
        anchors.push_back(Position(grid, gridPath[index]));
    anchors.push_back(goal);

    std::vector<glm::dvec2> simplified;
    simplified.push_back(start);
    std::size_t current = 0;
    while (current + 1U < anchors.size())
    {
        std::size_t next = current + 1U;
        for (std::size_t candidate = anchors.size() - 1U;
             candidate > current + 1U; --candidate)
        {
            if (SegmentIsClear(anchors[current], anchors[candidate],
                    buildings, clearanceSquared))
            {
                next = candidate;
                break;
            }
        }
        if (!SegmentIsClear(anchors[current], anchors[next],
                buildings, clearanceSquared))
            return {};
        simplified.push_back(anchors[next]);
        current = next;
    }
    return simplified;
}
}

std::optional<std::vector<glm::dvec3>> PlanMapFlightRoute(
    const TerrainGrid& terrain,
    const std::vector<MapBuilding>& buildings,
    const glm::dvec3& startMeters,
    const glm::dvec3& goalMeters,
    double buildingClearanceMeters)
{
    if (terrain.sampleCount < 2U
        || !std::isfinite(startMeters.x) || !std::isfinite(startMeters.y)
        || !std::isfinite(startMeters.z) || !std::isfinite(goalMeters.x)
        || !std::isfinite(goalMeters.y) || !std::isfinite(goalMeters.z)
        || !std::isfinite(buildingClearanceMeters)
        || !(buildingClearanceMeters > 0.0))
        throw std::invalid_argument("경로 시작점, 목표점과 건물 간격을 확인해 주세요.");

    const double halfWidth = static_cast<double>(terrain.sampleCount - 1U)
        * terrain.spacingXMeters * 0.5;
    const double halfDepth = static_cast<double>(terrain.sampleCount - 1U)
        * terrain.spacingYMeters * 0.5;
    Grid grid;
    grid.columns = static_cast<std::size_t>(std::ceil(
        2.0 * halfWidth / routeCellSizeMeters)) + 1U;
    grid.rows = static_cast<std::size_t>(std::ceil(
        2.0 * halfDepth / routeCellSizeMeters)) + 1U;
    grid.westMeters = -halfWidth;
    grid.southMeters = -halfDepth;
    grid.spacingXMeters = 2.0 * halfWidth / static_cast<double>(grid.columns - 1U);
    grid.spacingYMeters = 2.0 * halfDepth / static_cast<double>(grid.rows - 1U);

    const glm::dvec2 start{startMeters.x, startMeters.y};
    const glm::dvec2 goal{goalMeters.x, goalMeters.y};
    const double clearanceSquared = buildingClearanceMeters * buildingClearanceMeters;
    if (!terrain.ContainsLocalPositionMeters(start)
        || !terrain.ContainsLocalPositionMeters(goal)
        || IsBlocked(start, buildings, clearanceSquared)
        || IsBlocked(goal, buildings, clearanceSquared))
        return std::nullopt;

    if (glm::length(goal - start) < 1.0e-6)
        return std::vector<glm::dvec3>{goalMeters};

    const Cell startCell = NearestCell(grid, start);
    const Cell goalCell = NearestCell(grid, goal);
    const glm::dvec2 startCellPosition = Position(grid, startCell);
    const glm::dvec2 goalCellPosition = Position(grid, goalCell);
    if (IsBlocked(startCellPosition, buildings, clearanceSquared)
        || IsBlocked(goalCellPosition, buildings, clearanceSquared))
        return std::nullopt;

    const auto gridPath = FindGridPath(
        grid, startCell, goalCell, buildings, clearanceSquared);
    if (!gridPath)
        return std::nullopt;
    const std::vector<glm::dvec2> horizontalPath = SimplifyPath(
        *gridPath, grid, start, goal, buildings, clearanceSquared);
    if (horizontalPath.size() < 2U)
        return std::nullopt;

    double totalDistance = 0.0;
    for (std::size_t index = 1; index < horizontalPath.size(); ++index)
        totalDistance += glm::length(horizontalPath[index] - horizontalPath[index - 1U]);

    std::vector<glm::dvec3> route;
    route.reserve(horizontalPath.size() - 1U);
    double traveledDistance = 0.0;
    for (std::size_t index = 1; index < horizontalPath.size(); ++index)
    {
        traveledDistance += glm::length(horizontalPath[index] - horizontalPath[index - 1U]);
        const bool isGoal = index + 1U == horizontalPath.size();
        const double fraction = totalDistance > 0.0
            ? traveledDistance / totalDistance : 1.0;
        const double plannedAltitude = std::lerp(startMeters.z, goalMeters.z, fraction);
        const double groundAltitude = terrain.HeightAtLocalMeters(horizontalPath[index]);
        const double altitude = isGoal
            ? goalMeters.z
            : std::max(plannedAltitude,
                groundAltitude + minimumGroundClearanceMeters);
        if (altitude < groundAltitude + minimumGroundClearanceMeters)
            return std::nullopt;
        route.emplace_back(horizontalPath[index].x, horizontalPath[index].y, altitude);
    }
    return route;
}
}
