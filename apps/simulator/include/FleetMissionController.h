#pragma once

#include "FleetMissionCommunication.h"
#include "FleetMissionEnvironment.h"
#include "TrainingRangeScene.h"
#include "simulation_core/SimulationCore.h"

#include <map>
#include <string>
#include <vector>

namespace dronesim::simulator
{
/**
 * @brief 편대 임무의 시작, 단계 전환, 경로 진행과 안전 종료를 맡는다.
 * @details 임무용 통신 메시지는 FleetMissionCommunication에 맡긴다. 이 객체는
 *          화면 입력 없이 시뮬레이션과 임무 상태를 갱신한다.
 */
class FleetMissionController final
{
public:
    FleetMissionController(simulation::SimulationCore& simulation,
        const terrain::TerrainGrid& terrain,
        const scene::TrainingRangeScene& trainingScene);

    /** @brief 다음 고정 Tick에서 임무·통신·편대 상태를 갱신한다. */
    void AdvanceTick();

    [[nodiscard]] bool FleetReconnaissanceIsActive() const;
    [[nodiscard]] bool CompleteFleetMissionCanStart() const;
    [[nodiscard]] bool CompleteFleetMissionIsActive() const;
    [[nodiscard]] bool StartCompleteFleetMission();
    [[nodiscard]] std::string CompleteFleetMissionStatus() const;
    [[nodiscard]] bool StartFleetReconnaissance();
    [[nodiscard]] bool FleetReturnCanStart() const;
    [[nodiscard]] bool StartFleetReturn();
    void Abort(std::string reason);

    /** @brief 화면에 필요한 임무 상태를 읽기 전용으로 제공한다. */
    [[nodiscard]] const mission::FleetMissionState& State() const noexcept;
    [[nodiscard]] const FleetMissionEnvironment& Environment() const noexcept;
    [[nodiscard]] const FleetNetworkSession& NetworkSession() const noexcept;
    void SetRadioEnabled(simulation::DroneId droneId, bool enabled);

private:
    friend class DroneSimAppTestAccess;

    [[nodiscard]] bool StartFleetReconnaissance(
        mission::FleetMissionStartMode startMode,
        const std::map<simulation::DroneId, glm::dvec3>& homePositions);
    [[nodiscard]] bool FleetIsReadyForMission() const;
    [[nodiscard]] bool FleetReconnaissanceCanStart() const;
    void AdvanceCompleteFleetMission();
    void AdvanceFleetReconnaissance();
    [[nodiscard]] bool StartFormationTransit();
    void InitializeFleetLeadership();
    [[nodiscard]] bool StartFleetRally();
    [[nodiscard]] bool StartFleetReturn(mission::FleetMissionStartMode startMode);
    void FailCompleteFleetMission(const std::string& reason);
    void AdvanceFormationTransit();

    simulation::SimulationCore& simulation_;
    const scene::TrainingRangeScene& trainingScene_;
    FleetMissionEnvironment missionEnvironment_;
    mission::FleetMissionState fleetMission_;
    FleetMissionCommunication fleetCommunication_;
};
}
