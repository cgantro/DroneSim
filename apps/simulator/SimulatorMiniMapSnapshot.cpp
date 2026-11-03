#include "SimulatorMiniMapSnapshot.h"

#include "FleetMissionProgress.h"

#include <cstddef>
#include <utility>

namespace dronesim::simulator
{
SimulatorMiniMapSnapshot SimulatorMiniMapSnapshot::Capture(
    const simulation::SimulationCore& simulation,
    const mission::FleetMissionState& mission)
{
    SimulatorMiniMapSnapshot snapshot;

    const auto droneIds = simulation.DroneIds();
    snapshot.drones.reserve(droneIds.size());
    for (const simulation::DroneId droneId : droneIds)
    {
        const auto& state = simulation.State(droneId);
        snapshot.drones.push_back({droneId, state.positionMeters, state.yawRadians});
    }

    for (const auto& [droneId, assignment] : mission.Assignments())
    {
        if (!mission::IsFleetMissionInProgress(assignment.Status())
            || !assignment.HasRemainingWaypoints())
            continue;

        const auto& route = assignment.RouteWaypoints();
        const auto firstRemaining = route.begin()
            + static_cast<std::ptrdiff_t>(assignment.WaypointIndex());
        std::vector<glm::dvec3> remainingWaypoints(firstRemaining, route.end());
        snapshot.routes.push_back({droneId,
            simulation.State(droneId).positionMeters,
            std::move(remainingWaypoints)});
    }

    const auto& formation = mission.FormationTransit();
    const auto& formationRoute = formation.Route().leaderWaypointsMeters;
    if (formation.IsActive() && formation.WaypointIndex() < formationRoute.size())
    {
        const auto firstRemaining = formationRoute.begin()
            + static_cast<std::ptrdiff_t>(formation.WaypointIndex());
        const auto leaderId = mission.LeaderId();
        std::vector<glm::dvec3> remainingWaypoints(firstRemaining, formationRoute.end());
        snapshot.routes.push_back({leaderId,
            simulation.State(leaderId).positionMeters,
            std::move(remainingWaypoints)});
    }

    return snapshot;
}
}
