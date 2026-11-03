#pragma once

#include "FleetMissionCommunication.h"
#include "FleetMission.h"
#include "simulation_core/SimulationCore.h"

#include <optional>
#include <string>

namespace dronesim::simulator
{
/**
 * @brief 활성화된 V자 편대 이동을 한 Tick씩 진행한다.
 * @details 리더 경유점, 추종 드론 상태 보고, 편대 목표 전달과 도착 여부를 다룬다.
 *          통신이 끊겨 안전 착륙이 필요하면 이유를 돌려주고, 임무 전체의 종료는
 *          호출한 조정자에게 맡긴다.
 */
class FleetFormationFlight final
{
public:
    FleetFormationFlight(simulation::SimulationCore& simulation,
        mission::FleetMissionState& missionState,
        FleetMissionCommunication& communication);

    /** @brief 편대 상태를 갱신하고, 안전 착륙이 필요하면 그 이유를 돌려준다. */
    [[nodiscard]] std::optional<std::string> Advance();

private:
    simulation::SimulationCore& simulation_;
    mission::FleetMissionState& missionState_;
    FleetMissionCommunication& communication_;
};
}
