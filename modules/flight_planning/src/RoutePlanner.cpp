#include "flight_planning/RoutePlanner.h"
#include "geometry/Finite.h"
#include "geometry/Polygon.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <span>
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

bool IsInsideObstacleClearance(
    const glm::dvec2& point,
    const RouteObstacle& obstacle,
    double clearanceSquared)
{
    // 건물 안이거나 외곽선에서 안전거리보다 가까운 점은 비행 경로에서 제외한다.
    const auto& outline = obstacle.footprintMeters;
    if (outline.size() < 3U)
        return false;
    if (geometry::ContainsPointInPolygon(point, outline))
        return true;
    return geometry::SquaredDistanceToPolygon(point, outline) < clearanceSquared;
}

bool IsBlocked(
    const glm::dvec2& point,
    std::span<const RouteObstacle> obstacles,
    double clearanceSquared)
{
    return std::any_of(obstacles.begin(), obstacles.end(),
        [&](const RouteObstacle& obstacle)
        {
            return IsInsideObstacleClearance(point, obstacle, clearanceSquared);
        });
}

bool IsBlockedAtAltitude(
    const glm::dvec2& point,
    double altitudeMeters,
    std::span<const RouteObstacle> obstacles,
    double clearanceSquared)
{
    return std::any_of(obstacles.begin(), obstacles.end(),
        [&](const RouteObstacle& obstacle)
        {
            // 수평 외곽과 안전거리는 두 검사에서 같은 기준을 쓰고, 여기서는 높이만 추가로 본다.
            return IsInsideObstacleClearance(point, obstacle, clearanceSquared)
                && altitudeMeters
                    < obstacle.topAltitudeMeters + minimumGroundClearanceMeters;
        });
}

bool SegmentIsClear(
    const glm::dvec2& start,
    const glm::dvec2& end,
    std::span<const RouteObstacle> obstacles,
    double clearanceSquared)
{
    const double length = glm::length(end - start);
    const std::size_t sampleCount = std::max<std::size_t>(1U,
        static_cast<std::size_t>(std::ceil(length)));
    // 선분을 약 1m 간격으로 확인해 장애물 외곽을 통과하는 단순화 경로를 거른다.
    for (std::size_t index = 0; index <= sampleCount; ++index)
    {
        const double fraction = static_cast<double>(index)
            / static_cast<double>(sampleCount);
        if (IsBlocked(glm::mix(start, end, fraction), obstacles, clearanceSquared))
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

Grid CreateRouteGrid(const glm::dvec4& bounds)
{
    Grid grid;
    grid.columns = static_cast<std::size_t>(std::ceil(
        (bounds.z - bounds.x) / routeCellSizeMeters)) + 1U;
    grid.rows = static_cast<std::size_t>(std::ceil(
        (bounds.w - bounds.y) / routeCellSizeMeters)) + 1U;
    grid.westMeters = bounds.x;
    grid.southMeters = bounds.y;
    grid.spacingXMeters = (bounds.z - bounds.x)
        / static_cast<double>(grid.columns - 1U);
    grid.spacingYMeters = (bounds.w - bounds.y)
        / static_cast<double>(grid.rows - 1U);
    return grid;
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
    std::span<const RouteObstacle> obstacles,
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
            if (IsBlocked(nextPosition, obstacles, clearanceSquared))
                continue;
            if (rowOffset != 0 && columnOffset != 0)
            {
                const Cell horizontal{current.row, next.column};
                const Cell vertical{next.row, current.column};
                if (IsBlocked(Position(grid, horizontal), obstacles, clearanceSquared)
                    || IsBlocked(Position(grid, vertical), obstacles, clearanceSquared))
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
            // 직선거리는 실제 이동 비용을 넘지 않는 하한이므로 A*가 최적 경로를 보장한다.
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
    std::span<const RouteObstacle> obstacles,
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
        bool foundClearShortcut = false;
        for (std::size_t candidate = anchors.size() - 1U;
             candidate > current + 1U; --candidate)
        {
            if (SegmentIsClear(anchors[current], anchors[candidate],
                    obstacles, clearanceSquared))
            {
                next = candidate;
                foundClearShortcut = true;
                break;
            }
        }
        // 먼 후보를 찾지 못했을 때만 다음 인접 지점을 검사한다.
        // 찾은 먼 후보는 위에서 이미 장애물과의 거리를 확인했다.
        if (!foundClearShortcut
            && !SegmentIsClear(anchors[current], anchors[next],
                obstacles, clearanceSquared))
            return {};
        simplified.push_back(anchors[next]);
        current = next;
    }
    return simplified;
}

std::vector<glm::dvec3> AddFlightAltitudes(
    const std::vector<glm::dvec2>& horizontalPath,
    const RouteEnvironment& environment,
    const glm::dvec3& startMeters,
    const glm::dvec3& goalMeters,
    double transitAltitudeMeters,
    double safeStartAltitudeMeters)
{
    double totalDistanceMeters = 0.0;
    for (std::size_t index = 1; index < horizontalPath.size(); ++index)
    {
        totalDistanceMeters += glm::length(
            horizontalPath[index] - horizontalPath[index - 1U]);
    }

    std::vector<glm::dvec3> route;
    route.reserve(horizontalPath.size() + 1U);
    if (startMeters.z + 1.0e-6 < safeStartAltitudeMeters)
        route.emplace_back(startMeters.x, startMeters.y, safeStartAltitudeMeters);

    // 목표 고도까지의 변화를 수평 이동 거리에 비례해 나눈다.
    double traveledDistanceMeters = 0.0;
    for (std::size_t index = 1; index < horizontalPath.size(); ++index)
    {
        traveledDistanceMeters += glm::length(
            horizontalPath[index] - horizontalPath[index - 1U]);
        const bool isGoal = index + 1U == horizontalPath.size();
        const double fraction = totalDistanceMeters > 0.0
            ? traveledDistanceMeters / totalDistanceMeters : 1.0;
        const double plannedAltitude = std::lerp(
            startMeters.z, goalMeters.z, fraction);
        const double groundAltitude =
            environment.GroundAltitudeMeters(horizontalPath[index]);
        if (isGoal)
        {
            // 마지막 접근점은 목표 자체가 지면이나 최소 순항 고도에 묻히지 않게 한다.
            const double approachAltitude = std::max({goalMeters.z,
                groundAltitude + minimumGroundClearanceMeters,
                transitAltitudeMeters});
            if (approachAltitude > goalMeters.z + 1.0e-6)
                route.emplace_back(goalMeters.x, goalMeters.y, approachAltitude);
            route.push_back(goalMeters);
            continue;
        }

        const double altitude = std::max({plannedAltitude,
            groundAltitude + minimumGroundClearanceMeters,
            transitAltitudeMeters});
        route.emplace_back(horizontalPath[index].x,
            horizontalPath[index].y, altitude);
    }
    return route;
}
}

std::optional<std::vector<glm::dvec3>> RoutePlanner::Plan(
    const RouteEnvironment& environment,
    const glm::dvec3& startMeters,
    const glm::dvec3& goalMeters,
    const RoutePlanningOptions& options) const
{
    // 지도 구현체에서 경계·지면 높이·장애물만 받아오고, 지도 파일 형식은 알지 않는다.
    const glm::dvec4 bounds = environment.BoundsMeters();
    if (!geometry::IsFinite(bounds)
        || !(bounds.z > bounds.x) || !(bounds.w > bounds.y)
        || !geometry::IsFinite(startMeters) || !geometry::IsFinite(goalMeters)
        || !std::isfinite(options.minimumTransitAltitudeMeters)
        || !std::isfinite(options.obstacleClearanceMeters)
        || !(options.obstacleClearanceMeters > 0.0))
        throw std::invalid_argument("경로 시작점, 목표점과 장애물 간격을 확인해 주세요.");

    const Grid grid = CreateRouteGrid(bounds);

    const glm::dvec2 start{startMeters.x, startMeters.y};
    const glm::dvec2 goal{goalMeters.x, goalMeters.y};
    const double transitAltitude = options.minimumTransitAltitudeMeters;
    const double clearanceSquared = options.obstacleClearanceMeters
        * options.obstacleClearanceMeters;
    // 시작 지점에서 안전 고도까지 수직 상승한 뒤 수평 경로를 시작한다.
    const double safeStartAltitude = std::max(
        transitAltitude,
        environment.GroundAltitudeMeters(start) + minimumGroundClearanceMeters);
    std::vector<RouteObstacle> routeObstacles;
    routeObstacles.reserve(environment.Obstacles().size());
    // 출발 직후 수직으로 안전 고도까지 오른 뒤 수평 이동하므로, 그 고도보다 낮은 건물은 경로를 막지 않는다.
    const double minimumRouteAltitude = std::min(safeStartAltitude, goalMeters.z);
    for (const RouteObstacle& obstacle : environment.Obstacles())
    {
        if (minimumRouteAltitude < obstacle.topAltitudeMeters + minimumGroundClearanceMeters)
            routeObstacles.push_back(obstacle);
    }
    // 시작·목표 위치는 실제 고도에서 확인하고, 수평 탐색에는 비행 고도보다 높은 장애물만 넘긴다.
    const bool startBlocked = IsBlockedAtAltitude(
        start, startMeters.z, environment.Obstacles(), clearanceSquared);
    const bool goalBlocked = IsBlockedAtAltitude(
        goal, goalMeters.z, environment.Obstacles(), clearanceSquared);
    if (!environment.Contains(start) || !environment.Contains(goal)
        || startBlocked || goalBlocked)
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

    // 1) A*가 격자 경로를 찾고, 2) 직선으로 통과 가능한 구간을 합쳐 경유점을 줄인다.
    const auto gridPath = FindGridPath(
        grid, startCell, goalCell, routeObstacles, clearanceSquared);
    if (!gridPath)
        return std::nullopt;
    const std::vector<glm::dvec2> horizontalPath = SimplifyPath(
        *gridPath, grid, start, goal, routeObstacles, clearanceSquared);
    if (horizontalPath.size() < 2U)
        return std::nullopt;

    return AddFlightAltitudes(horizontalPath, environment,
        startMeters, goalMeters, transitAltitude, safeStartAltitude);
}

}
