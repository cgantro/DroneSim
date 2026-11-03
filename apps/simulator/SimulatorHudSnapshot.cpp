#include "SimulatorHudSnapshot.h"

#include "FleetMissionProgress.h"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace dronesim::simulator
{
namespace
{
const gui::DroneStatusView& FindDroneStatus(
    const std::vector<gui::DroneStatusView>& drones,
    simulation::DroneId droneId)
{
    const auto found = std::find_if(drones.begin(), drones.end(),
        [droneId](const gui::DroneStatusView& drone)
        {
            return drone.droneId == droneId;
        });
    if (found == drones.end())
        throw std::out_of_range("HUD 스냅샷에서 드론 상태를 찾을 수 없습니다.");
    return *found;
}
}

SimulatorHudSnapshot SimulatorHudSnapshot::Capture(
    const simulation::SimulationCore& simulation,
    const mission::FleetMissionState& mission)
{
    SimulatorHudSnapshot snapshot;
    snapshot.primaryDroneId = simulation.PrimaryDroneId();
    snapshot.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    snapshot.tickCount = simulation.TickCount();

    const auto droneIds = simulation.DroneIds();
    snapshot.drones.reserve(droneIds.size());
    for (const simulation::DroneId droneId : droneIds)
    {
        const auto& state = simulation.State(droneId);
        snapshot.drones.push_back({
            droneId, state.mode, state.positionMeters, state.yawRadians});
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
            FindDroneStatus(snapshot.drones, droneId).positionMeters,
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
            FindDroneStatus(snapshot.drones, leaderId).positionMeters,
            std::move(remainingWaypoints)});
    }

    return snapshot;
}
}
