#pragma once

#include "FleetMission.h"
#include "gui/MiniMapPanel.h"
#include "simulation_core/SimulationCore.h"

#include <vector>

namespace dronesim::simulator
{
/**
 * @brief 현재 시뮬레이션과 임무 상태를 한 프레임의 미니맵 자료로 묶는다.
 * @details 화면 코드는 이 자료만 그리며, 드론과 임무 경로를 찾는 규칙은
 * HUD를 조립하는 앱 코드에서 분리한다.
 */
struct SimulatorMiniMapSnapshot
{
    std::vector<gui::MiniMapDrone> drones;
    std::vector<gui::MiniMapRoute> routes;

    /** @brief 지도에 표시할 현재 위치와 아직 남은 경로를 수집한다. */
    [[nodiscard]] static SimulatorMiniMapSnapshot Capture(
        const simulation::SimulationCore& simulation,
        const mission::FleetMissionState& mission);
};
}
