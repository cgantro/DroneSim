#include "flight_planning/FormationPlanner.h"

#include "flight_planning/RoutePlanner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <glm/geometric.hpp>

namespace dronesim::flight_planning
{
namespace
{
std::vector<glm::dvec2> LocalOffsets(
    std::size_t droneCount, double spacingMeters)
{
    // 첫 드론은 리더 위치에 두고 나머지는 뒤쪽 좌우에 번갈아 놓는다.
    std::vector<glm::dvec2> offsets;
    offsets.reserve(droneCount);
    offsets.emplace_back(0.0, 0.0);

    for (std::size_t index = 1; index < droneCount; ++index)
    {
        const std::size_t followerIndex = index - 1U;
        const double row = static_cast<double>(followerIndex / 2U + 1U);
        const std::size_t followerCount = droneCount - 1U;
        const bool isLastSoloFollower = followerCount % 2U != 0U
            && followerIndex + 1U == followerCount;
        const double side = isLastSoloFollower ? 0.0
            : (followerIndex % 2U == 0U ? 1.0 : -1.0);
        offsets.emplace_back(-row * spacingMeters,
            side * row * spacingMeters * 0.5);
    }
    return offsets;
}

double RequiredClearance(
    std::size_t droneCount, double spacingMeters,
    double droneRadiusMeters, double safetyMarginMeters)
{
    // 편대의 가장 바깥 드론까지 거리와 기체 반경·안전 여유를 합친다.
    const auto offsets = LocalOffsets(droneCount, spacingMeters);
    double formationRadiusMeters = 0.0;
    for (const glm::dvec2& offset : offsets)
        formationRadiusMeters = std::max(formationRadiusMeters, glm::length(offset));
    return formationRadiusMeters + droneRadiusMeters + safetyMarginMeters;
}
}

double FormationRoutePlan::HeadingAtWaypoint(
    std::size_t waypointIndex, double fallbackHeadingRadians) const noexcept
{
    if (leaderWaypointsMeters.empty() || waypointIndex >= leaderWaypointsMeters.size())
        return fallbackHeadingRadians;

    glm::dvec2 direction{0.0};
    if (waypointIndex + 1U < leaderWaypointsMeters.size())
    {
        direction = {
            leaderWaypointsMeters[waypointIndex + 1U].x
                - leaderWaypointsMeters[waypointIndex].x,
            leaderWaypointsMeters[waypointIndex + 1U].y
                - leaderWaypointsMeters[waypointIndex].y};
    }
    else if (waypointIndex > 0U)
    {
        direction = {
            leaderWaypointsMeters[waypointIndex].x
                - leaderWaypointsMeters[waypointIndex - 1U].x,
            leaderWaypointsMeters[waypointIndex].y
                - leaderWaypointsMeters[waypointIndex - 1U].y};
    }
    else
    {
        direction = {
            leaderWaypointsMeters.front().x - leaderStartMeters.x,
            leaderWaypointsMeters.front().y - leaderStartMeters.y};
    }

    // 정지점이나 중복 경유점에서는 atan2(0, 0) 대신 기체의 현재 방향을 유지한다.
    return glm::length(direction) > 1.0e-6
        ? std::atan2(direction.y, direction.x)
        : fallbackHeadingRadians;
}

std::vector<glm::dvec3> FormationRoutePlan::TargetsAtWaypoint(
    std::size_t waypointIndex,
    std::size_t droneCount,
    const glm::dvec3& leaderPositionMeters,
    double leaderYawRadians) const
{
    if (droneCount == 0U || waypointIndex >= leaderWaypointsMeters.size())
        return {};

    const double headingRadians = HeadingAtWaypoint(
        waypointIndex, leaderYawRadians);
    const auto offsets = FormationOffsets(
        droneCount, spacingMeters, headingRadians);

    std::vector<glm::dvec3> targets;
    targets.reserve(droneCount);
    targets.push_back(leaderWaypointsMeters[waypointIndex]);

    // 리더는 다음 경유점으로 향하고, 편대원은 현재 리더 위치를 기준으로 자리를 잡는다.
    for (std::size_t index = 1U; index < droneCount; ++index)
    {
        glm::dvec3 target = leaderPositionMeters;
        target.x += offsets[index].x;
        target.y += offsets[index].y;
        targets.push_back(target);
    }
    return targets;
}

std::vector<glm::dvec2> FormationOffsets(
    std::size_t droneCount, double spacingMeters, double headingRadians)
{
    if (droneCount == 0U || !std::isfinite(spacingMeters) || spacingMeters <= 0.0
        || !std::isfinite(headingRadians))
        throw std::invalid_argument("편대 수, 간격, 진행 방향이 올바르지 않습니다.");

    const std::vector<glm::dvec2> local =
        LocalOffsets(droneCount, spacingMeters);
    // 로컬 앞·옆 축의 간격을 월드 진행 방향에 맞춰 회전한다.
    const glm::dvec2 forward{std::cos(headingRadians), std::sin(headingRadians)};
    const glm::dvec2 right{forward.y, -forward.x};
    std::vector<glm::dvec2> worldOffsets;
    worldOffsets.reserve(local.size());
    for (const glm::dvec2& offset : local)
        worldOffsets.push_back(forward * offset.x + right * offset.y);
    return worldOffsets;
}

std::optional<FormationRoutePlan> PlanFormationRoute(
    const RouteEnvironment& environment,
    const glm::dvec3& leaderStartMeters,
    const glm::dvec3& destinationMeters,
    std::size_t droneCount,
    double spacingMeters,
    double droneRadiusMeters,
    double safetyMarginMeters,
    double minimumTransitAltitudeMeters)
{
    if (droneCount == 0U || !std::isfinite(spacingMeters) || spacingMeters <= 0.0
        || !std::isfinite(droneRadiusMeters) || droneRadiusMeters < 0.0
        || !std::isfinite(safetyMarginMeters) || safetyMarginMeters < 0.0)
        throw std::invalid_argument("편대 경로의 드론 수와 간격·여유 값이 올바르지 않습니다.");

    // 편대 전체가 통과할 여유를 확보하지 못하면 대형을 바꾸지 않고 실패한다.
    const double clearance = RequiredClearance(droneCount,
        spacingMeters, droneRadiusMeters, safetyMarginMeters);
    // 편대가 통째로 지나갈 폭을 경로 탐색에 요구한다.
    const auto route = RoutePlanner{}.Plan(environment,
        leaderStartMeters, destinationMeters,
        {.obstacleClearanceMeters = clearance,
         .minimumTransitAltitudeMeters = minimumTransitAltitudeMeters});
    if (!route)
        return std::nullopt;

    return FormationRoutePlan{
        .leaderStartMeters = leaderStartMeters,
        .leaderWaypointsMeters = *route,
        .requiredClearanceMeters = clearance,
        .spacingMeters = spacingMeters};
}
}
