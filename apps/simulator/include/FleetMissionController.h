#pragma once

#include "FleetMissionEnvironment.h"
#include "FleetNetworkSession.h"
#include "TrainingRangeScene.h"
#include "simulation_core/SimulationCore.h"

#include <map>
#include <string>
#include <vector>

namespace dronesim::simulator
{
/**
 * @brief 편대 임무의 시작, 진행, 통신 반영과 안전 종료를 맡는다.
 * @details 화면과 게임패드에는 의존하지 않는다. 시뮬레이션 상태, 훈련장 자료,
 *          임무 상태, 통신 세션을 연결해 M01·M02·M03 흐름을 진행한다.
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
    [[nodiscard]] bool PrepareFleetNetwork(bool reset);
    void InitializeFleetLeadership();
    void AdvanceFleetNetwork();
    void RefreshFleetNetworkNodes(std::uint64_t currentTick);
    void ProcessDeliveredFleetMessages(std::uint64_t currentTick);
    void SendFleetTelemetry(std::uint64_t currentTick);
    void RetryPendingFleetRoutes(std::uint64_t currentTick);
    [[nodiscard]] bool SendFleetRouteAssignment(
        simulation::DroneId droneId,
        mission::FleetMissionLeg leg,
        std::size_t targetIndex,
        std::vector<glm::dvec3> routeWaypointsMeters);
    void ApplyFleetRouteAssignment(
        simulation::DroneId droneId,
        mission::FleetMissionLeg leg,
        std::size_t targetIndex,
        std::vector<glm::dvec3> routeWaypointsMeters,
        std::uint32_t sequence,
        std::uint64_t assignmentVersion,
        std::uint64_t assignmentLeaseExpiresAtTick);
    [[nodiscard]] bool StartFleetRally();
    [[nodiscard]] bool StartFleetReturn(mission::FleetMissionStartMode startMode);
    void FailCompleteFleetMission(const std::string& reason);
    void AdvanceFormationTransit();

    simulation::SimulationCore& simulation_;
    const scene::TrainingRangeScene& trainingScene_;
    FleetMissionEnvironment missionEnvironment_;
    mission::FleetMissionState fleetMission_;
    FleetNetworkSession fleetNetworkSession_;
    std::vector<simulation::DroneId> fleetDroneIds_;
};
}
