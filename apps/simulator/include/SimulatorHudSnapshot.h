#pragma once

#include "FleetMission.h"
#include "gui/MiniMapPanel.h"
#include "simulation_core/SimulationCore.h"

#include <vector>

namespace dronesim::simulator
{
/** @brief 한 프레임에서 HUD 패널들이 함께 사용할 상태와 임무 경로다. */
struct SimulatorHudSnapshot
{
    std::vector<gui::DroneStatusView> drones;
    std::vector<gui::MiniMapRoute> routes;
    simulation::DroneId primaryDroneId = 0;
    double simulationTimeSeconds = 0.0;
    std::uint64_t tickCount = 0;

    /** @brief 각 드론 상태를 한 번 읽고, 그 자료로 지도 경로를 구성한다. */
    [[nodiscard]] static SimulatorHudSnapshot Capture(
        const simulation::SimulationCore& simulation,
        const mission::FleetMissionState& mission);
};
}
