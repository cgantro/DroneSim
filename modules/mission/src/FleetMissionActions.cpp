#include "FleetMissionActions.h"

#include <vector>

namespace dronesim::mission
{
bool TakeOffFleetDrones(
    simulation::SimulationCore& simulation,
    std::span<const simulation::DroneId> droneIds)
{
    std::vector<simulation::DroneId> airborneDrones;
    airborneDrones.reserve(droneIds.size());

    for (const simulation::DroneId droneId : droneIds)
    {
        const auto result = simulation.Apply(droneId,
            {simulation::DroneCommand::Kind::TakeOff});
        if (result != simulation::CommandResult::Accepted)
        {
            // 부분 출발 상태로 임무가 남지 않도록, 성공한 기체부터 안전하게 내린다.
            LandActiveFleetDrones(simulation, airborneDrones);
            return false;
        }
        airborneDrones.push_back(droneId);
    }
    return true;
}

bool TryLandDrone(
    simulation::SimulationCore& simulationCore,
    simulation::DroneId droneId)
{
    const auto mode = simulationCore.State(droneId).mode;
    if (mode == simulation::FlightMode::Grounded
        || mode == simulation::FlightMode::Landing)
        return true;

    // 지상에 있는 기체와 이미 착륙 중인 기체에는 중복 착륙 명령을 보내지 않는다.
    return simulationCore.Apply(droneId,
        {simulation::DroneCommand::Kind::Land})
        == simulation::CommandResult::Accepted;
}

void LandActiveFleetDrones(
    simulation::SimulationCore& simulationCore,
    std::span<const simulation::DroneId> droneIds)
{
    for (const simulation::DroneId droneId : droneIds)
        static_cast<void>(TryLandDrone(simulationCore, droneId));
}
}
