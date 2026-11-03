#pragma once

#include "FleetMissionCommunication.h"
#include "FleetMissionEnvironment.h"
#include "FleetMission.h"
#include "simulation_core/SimulationCore.h"

namespace dronesim::simulator
{
/**
 * @brief 임무에 배정된 드론들의 개별 경로를 한 Tick씩 진행한다.
 * @details 경유점 적용, 착륙 상태 갱신, 배터리 부족 시 복귀 경로 전환을 맡는다.
 *          임무 전체의 단계 전환은 상위 조정자가 판단한다.
 */
class FleetDroneMissionExecutor final
{
public:
    FleetDroneMissionExecutor(simulation::SimulationCore& simulation,
        const FleetMissionEnvironment& environment,
        mission::FleetMissionState& missionState,
        FleetMissionCommunication& communication);

    /** @brief 현재 Tick의 드론 상태에 맞춰 각 임무 경로를 진행한다. */
    void Advance();

private:
    /** @brief 배터리 복귀를 시작했다면 일반 임무 경로 처리를 이번 Tick에 건너뛴다. */
    [[nodiscard]] bool TryStartLowBatteryReturn(
        simulation::DroneId droneId,
        mission::FleetReconnaissanceState& assignment);
    /** @brief 착륙한 드론의 현재 임무 구간에 맞는 완료 상태를 기록한다. */
    void UpdateGroundedMissionStatus(
        simulation::DroneId droneId,
        mission::FleetReconnaissanceState& assignment);
    /** @brief 현재 경유점 도착 여부를 확인하고 다음 비행 목표를 적용한다. */
    void AdvanceRoute(
        simulation::DroneId droneId,
        mission::FleetReconnaissanceState& assignment);

    simulation::SimulationCore& simulation_;
    const FleetMissionEnvironment& environment_;
    mission::FleetMissionState& missionState_;
    FleetMissionCommunication& communication_;
};
}
