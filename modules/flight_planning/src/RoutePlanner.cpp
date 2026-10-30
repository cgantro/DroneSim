#include "flight_planning/RoutePlanner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include <glm/geometric.hpp>

namespace dronesim::flight_planning
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

bool ContainsPointInRing(
    const std::vector<glm::dvec2>& outline, const glm::dvec2& point)
{
    bool inside = false;
    for (std::size_t index = 0, previous = outline.size() - 1U;
         index < outline.size(); previous = index++)
    {
        const glm::dvec2& first = outline[index];
        const glm::dvec2& second = outline[previous];
        const bool crosses = (first.y > point.y) != (second.y > point.y);
        if (crosses && point.x < (second.x - first.x) * (point.y - first.y)
                / (second.y - first.y) + first.x)
            inside = !inside;
    }
    return inside;
}

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

bool IsBlocked(
    const glm::dvec2& point,
    const std::vector<BuildingObstacle>& buildings,
    double clearanceSquared)
{
    for (const BuildingObstacle& building : buildings)
    {
        const auto& outline = building.outlineMeters;
        if (outline.size() < 3U)
            continue;
        if (ContainsPointInRing(outline, point))
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

bool IsBlockedAtAltitude(
    const glm::dvec2& point,
    double altitudeMeters,
    const std::vector<BuildingObstacle>& buildings,
    double clearanceSquared)
{
    for (const BuildingObstacle& building : buildings)
    {
        const auto& outline = building.outlineMeters;
        if (outline.size() < 3U)
            continue;
        bool horizontallyBlocked = ContainsPointInRing(outline, point);
        for (std::size_t index = 0; !horizontallyBlocked && index < outline.size(); ++index)
        {
            horizontallyBlocked = DistanceToSegmentSquared(point, outline[index],
                outline[(index + 1U) % outline.size()]) < clearanceSquared;
        }
        if (!horizontallyBlocked)
            continue;

        const double groundTop = building.groundElevationsMeters.empty()
            ? 0.0
            : *std::max_element(building.groundElevationsMeters.begin(),
                building.groundElevationsMeters.end());
        const double buildingTop = building.heightMeters > 0.0
            ? groundTop + building.heightMeters
            : std::numeric_limits<double>::infinity();
        if (altitudeMeters < buildingTop + minimumGroundClearanceMeters)
            return true;
    }
    return false;
}

bool SegmentIsClear(
    const glm::dvec2& start,
    const glm::dvec2& end,
    const std::vector<BuildingObstacle>& buildings,
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

/**
 * @brief A*로 격자에서 시작 칸부터 목표 칸까지의 수평 경로를 찾는다.
 * @details A*는 시작점부터 쌓인 실제 이동 비용 g와 목표까지의 추정 비용 h를
 *          더한 f = g + h가 가장 작은 후보를 먼저 살펴본다.
 *          이 함수에서는 distance가 g, 목표까지의 직선거리가 h,
 *          pending이 f 순서의 미방문 후보 목록, closed가 방문 완료 칸,
 *          previous가 경로 복원용 이전 칸을 맡는다. 대각선 이동은 모서리
 *          양쪽 칸도 확인해 건물 모서리를 가로질러 통과하지 못하게 한다.
 * @return 시작 칸과 목표 칸을 포함한 경로. 막혀 있으면 빈 값을 반환한다.
 */
std::optional<std::vector<Cell>> FindGridPath(
    const Grid& grid,
    Cell start,
    Cell goal,
    const std::vector<BuildingObstacle>& buildings,
    double clearanceSquared)
{
    const std::size_t cellCount = grid.rows * grid.columns;
    // g 비용, 이전 칸, 방문 완료 여부를 칸 번호로 바로 찾도록 같은 크기로 둔다.
    std::vector<double> distance(cellCount, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(cellCount, cellCount);
    std::vector<bool> closed(cellCount, false);
    // 우선순위 큐의 첫 값이 f = g + h이며, 가장 작은 예상 총비용부터 꺼낸다.
    using Candidate = std::pair<double, std::size_t>;
    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<>> pending;
    const std::size_t startIndex = Index(grid, start);
    const std::size_t goalIndex = Index(grid, goal);
    distance[startIndex] = 0.0;
    pending.emplace(0.0, startIndex);

    constexpr std::array<std::pair<int, int>, 8> neighbors{{
        {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
        {0, 1}, {1, -1}, {1, 0}, {1, 1}}};
    // 가장 낮은 f 후보를 하나씩 확정하면서 인접한 칸의 더 나은 경로를 갱신한다.
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
            // 직선 이동은 한 칸, 대각선 이동은 한 칸의 대각선 길이만큼 비용을 더한다.
            const double stepCost = rowOffset != 0 && columnOffset != 0
                ? routeCellSizeMeters * std::sqrt(2.0)
                : routeCellSizeMeters;
            const double candidateDistance = distance[currentIndex] + stepCost;
            if (candidateDistance >= distance[nextIndex])
                continue;

            previous[nextIndex] = currentIndex;
            distance[nextIndex] = candidateDistance;
            // 장애물이 없는 공간에서 목표까지 갈 때의 하한인 직선거리를 h로 쓴다.
            const double heuristic = glm::length(
                Position(grid, next) - Position(grid, goal));
            pending.emplace(candidateDistance + heuristic, nextIndex);
        }
    }

    if (!closed[goalIndex])
        return std::nullopt;

    // 목표에서 previous를 거꾸로 따라간 뒤 순서를 뒤집어 출발점부터 정렬한다.
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
    const std::vector<BuildingObstacle>& buildings,
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

std::optional<std::vector<glm::dvec3>> PlanRoute(
    const terrain::TerrainGrid& terrain,
    const std::vector<BuildingObstacle>& buildings,
    const glm::dvec3& startMeters,
    const glm::dvec3& goalMeters,
    double buildingClearanceMeters,
    double minimumTransitAltitudeMeters)
{
    if (terrain.sampleCount < 2U
        || !std::isfinite(startMeters.x) || !std::isfinite(startMeters.y)
        || !std::isfinite(startMeters.z) || !std::isfinite(goalMeters.x)
        || !std::isfinite(goalMeters.y) || !std::isfinite(goalMeters.z)
        || !std::isfinite(minimumTransitAltitudeMeters)
        || !std::isfinite(buildingClearanceMeters)
        || !(buildingClearanceMeters > 0.0))
        throw std::invalid_argument("경로 시작점, 목표점과 건물 간격을 확인해 주세요.");

    const double halfWidth = terrain.HalfWidthMeters();
    const double halfDepth = terrain.HalfDepthMeters();
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
    const double transitAltitude = minimumTransitAltitudeMeters;
    const double clearanceSquared = buildingClearanceMeters * buildingClearanceMeters;
    const double safeStartAltitude = std::max(
        transitAltitude,
        terrain.HeightAtLocalMeters(start) + minimumGroundClearanceMeters);
    std::vector<BuildingObstacle> routeObstacles;
    routeObstacles.reserve(buildings.size());
    // 출발 직후 수직으로 안전 고도까지 오른 뒤 수평 이동하므로, 그 고도보다 낮은 건물은 경로를 막지 않는다.
    const double minimumRouteAltitude = std::min(safeStartAltitude, goalMeters.z);
    for (const BuildingObstacle& building : buildings)
    {
        const double highestGround = building.groundElevationsMeters.empty()
            ? 0.0
            : *std::max_element(building.groundElevationsMeters.begin(),
                building.groundElevationsMeters.end());
        const double buildingTop = highestGround + building.heightMeters;
        if (minimumRouteAltitude < buildingTop + minimumGroundClearanceMeters)
            routeObstacles.push_back(building);
    }
    const bool startBlocked = IsBlockedAtAltitude(
        start, startMeters.z, buildings, clearanceSquared);
    const bool goalBlocked = IsBlockedAtAltitude(
        goal, goalMeters.z, buildings, clearanceSquared);
    if (!terrain.ContainsLocalPositionMeters(start)
        || !terrain.ContainsLocalPositionMeters(goal) || startBlocked || goalBlocked)
        return std::nullopt;

    if (glm::length(goal - start) < 1.0e-6)
        return std::vector<glm::dvec3>{goalMeters};

    const Cell startCell = NearestCell(grid, start);
    const Cell goalCell = NearestCell(grid, goal);
    const glm::dvec2 startCellPosition = Position(grid, startCell);
    const glm::dvec2 goalCellPosition = Position(grid, goalCell);
    if (IsBlocked(startCellPosition, routeObstacles, clearanceSquared)
        || IsBlocked(goalCellPosition, routeObstacles, clearanceSquared))
        return std::nullopt;

    const auto gridPath = FindGridPath(
        grid, startCell, goalCell, routeObstacles, clearanceSquared);
    if (!gridPath)
        return std::nullopt;
    const std::vector<glm::dvec2> horizontalPath = SimplifyPath(
        *gridPath, grid, start, goal, routeObstacles, clearanceSquared);
    if (horizontalPath.size() < 2U)
        return std::nullopt;

    double totalDistance = 0.0;
    for (std::size_t index = 1; index < horizontalPath.size(); ++index)
        totalDistance += glm::length(horizontalPath[index] - horizontalPath[index - 1U]);

    std::vector<glm::dvec3> route;
    route.reserve(horizontalPath.size() + 1U);
    if (startMeters.z + 1.0e-6 < safeStartAltitude)
        route.emplace_back(startMeters.x, startMeters.y, safeStartAltitude);

    double traveledDistance = 0.0;
    for (std::size_t index = 1; index < horizontalPath.size(); ++index)
    {
        traveledDistance += glm::length(horizontalPath[index] - horizontalPath[index - 1U]);
        const bool isGoal = index + 1U == horizontalPath.size();
        const double fraction = totalDistance > 0.0
            ? traveledDistance / totalDistance : 1.0;
        const double plannedAltitude = std::lerp(startMeters.z, goalMeters.z, fraction);
        const double groundAltitude = terrain.HeightAtLocalMeters(horizontalPath[index]);
        if (isGoal)
        {
            const double approachAltitude = std::max({goalMeters.z,
                groundAltitude + minimumGroundClearanceMeters, transitAltitude});
            if (approachAltitude > goalMeters.z + 1.0e-6)
                route.emplace_back(goalMeters.x, goalMeters.y, approachAltitude);
            route.push_back(goalMeters);
            continue;
        }

        const double altitude = std::max({plannedAltitude,
            groundAltitude + minimumGroundClearanceMeters, transitAltitude});
        if (altitude < groundAltitude + minimumGroundClearanceMeters)
            return std::nullopt;
        route.emplace_back(horizontalPath[index].x, horizontalPath[index].y, altitude);
    }
    return route;
}

}
