#include "FleetMissionPlanner.h"

#include "flight_planning/RoutePlanner.h"

#include <utility>

namespace dronesim::mission
{
FleetMissionPlanner::FleetMissionPlanner(
    const flight_planning::RouteEnvironment& environment,
    double minimumTransitAltitudeMeters) noexcept
    : environment_(environment),
      minimumTransitAltitudeMeters_(minimumTransitAltitudeMeters)
{
}

std::optional<std::vector<glm::dvec3>> FleetMissionPlanner::PlanTransitRoute(
    const glm::dvec3& startPositionMeters,
    const glm::dvec3& targetPositionMeters) const
{
    // 정찰·집결·복귀 경로는 모두 같은 건물 여유 거리와 최소 고도를 사용한다.
    return flight_planning::RoutePlanner{}.Plan(
        environment_, startPositionMeters, targetPositionMeters,
        {.obstacleClearanceMeters = 3.0,
         .minimumTransitAltitudeMeters = minimumTransitAltitudeMeters_});
}

std::optional<std::vector<glm::dvec3>> FleetMissionPlanner::PlanReturnRoute(
    const glm::dvec3& currentPositionMeters,
    const glm::dvec3& homePositionMeters) const
{
    glm::dvec3 approachPosition = homePositionMeters;
    approachPosition.z += returnApproachHeightMeters;
    return PlanTransitRoute(currentPositionMeters, approachPosition);
}

std::optional<std::vector<FleetReconnaissanceAssignment>>
FleetMissionPlanner::PlanReconnaissance(
    std::span<const FleetReconnaissanceStart> starts,
    std::span<const FleetReconnaissanceTarget> targets,
    bool takeOffDrones) const
{
    if (starts.empty() || targets.empty())
        return std::vector<FleetReconnaissanceAssignment>{};

    std::vector<FleetReconnaissanceAssignment> assignments;
    assignments.reserve(starts.size());
    for (std::size_t index = 0; index < starts.size(); ++index)
    {
        // 목표 수보다 드론이 많으면 앞의 목표부터 다시 배정해 모든 드론에 경로를 준다.
        const std::size_t targetIndex = index % targets.size();
        const FleetReconnaissanceTarget& target = targets[targetIndex];
        const FleetReconnaissanceStart& start = starts[index];
        const auto route = PlanTransitRoute(
            start.currentPositionMeters, target.approachPositionMeters);
        if (!route)
        {
            // 일부 드론만 임무에 들어가지 않도록 전체 계획을 실패로 돌린다.
            return std::nullopt;
        }

        FleetReconnaissanceState mission;
        mission.SetInitialAssignment(targetIndex, start.homePositionMeters);
        mission.SetPlannedRoute(std::move(*route));
        mission.UpdateStatus(takeOffDrones
            ? FleetDroneStatus::TakeoffReady
            : FleetDroneStatus::Reconnaissance);
        assignments.push_back({start.droneId, std::move(mission)});
    }
    return assignments;
}
}
