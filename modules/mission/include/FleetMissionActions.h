#pragma once

#include "simulation_core/SimulationCore.h"

#include <span>

namespace dronesim::mission
{
/** @brief 편대원에게 차례로 이륙을 명령하고, 중간에 실패하면 먼저 뜬 기체를 착륙시킨다. */
[[nodiscard]] bool TakeOffFleetDrones(
    simulation::SimulationCore& simulation,
    std::span<const simulation::DroneId> droneIds);

/** @brief 이미 지상에 있거나 착륙 중인 드론을 제외하고 착륙을 명령한다. */
void LandDroneIfNeeded(
    simulation::SimulationCore& simulation,
    simulation::DroneId droneId);

/** @brief 착륙 명령이 받아들여졌거나 이미 착륙 상태인지 반환한다. */
[[nodiscard]] bool TryLandDrone(
    simulation::SimulationCore& simulation,
    simulation::DroneId droneId);

/** @brief 지상에 있지 않고 착륙 중도 아닌 편대원에게 착륙을 명령한다. */
void LandActiveFleetDrones(
    simulation::SimulationCore& simulation,
    std::span<const simulation::DroneId> droneIds);
}
