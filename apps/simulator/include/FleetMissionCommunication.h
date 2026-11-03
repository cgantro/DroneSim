#pragma once

#include "FleetMissionEnvironment.h"
#include "FleetNetworkSession.h"
#include "FleetMission.h"
#include "TrainingRangeScene.h"
#include "simulation_core/SimulationCore.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace dronesim::simulator
{
/**
 * @brief 편대 임무에서 오가는 메시지를 만들고, 받고, 확인한다.
 * @details 소켓과 가상 연결은 FleetNetworkSession이 처리한다. 이 객체는 그 위에서
 *          경로 배정, 편대 목표, 드론 상태 보고 같은 임무 메시지의 의미를 다룬다.
 */
class FleetMissionCommunication final
{
public:
    FleetMissionCommunication(simulation::SimulationCore& simulation,
        const scene::TrainingRangeScene& trainingScene,
        const FleetMissionEnvironment& missionEnvironment,
        mission::FleetMissionState& missionState);

    /** @brief 현재 편대 구성과 건물 가림 자료로 통신을 준비한다. */
    [[nodiscard]] bool Prepare(bool reset);
    /** @brief 임무용 메시지를 주고받고 확인이 필요한 경로를 다시 보낸다. */
    void AdvanceTick();
    /** @brief 통신 대상이 될 드론 목록을 현재 시뮬레이션에서 다시 읽는다. */
    void RefreshDroneRoster();
    /** @brief 드론에 경로를 전달하고 상태에 전달 대기 여부를 기록한다. */
    [[nodiscard]] bool SendRouteAssignment(
        simulation::DroneId droneId,
        mission::FleetMissionLeg leg,
        std::size_t targetIndex,
        std::vector<glm::dvec3> routeWaypointsMeters);
    /** @brief 편대원이 바뀐 목표를 따라가도록 필요할 때만 새 목표를 보낸다. */
    [[nodiscard]] bool SendFormationTarget(
        simulation::DroneId droneId,
        const glm::dvec3& targetMeters);

    void SetRadioEnabled(simulation::DroneId droneId, bool enabled);
    [[nodiscard]] const FleetNetworkSession& NetworkSession() const noexcept;
    [[nodiscard]] const std::vector<simulation::DroneId>& DroneIds() const noexcept;

private:
    void RefreshNodes(std::uint64_t currentTick);
    void ProcessDeliveredMessages(std::uint64_t currentTick);
    void SendTelemetry(std::uint64_t currentTick);
    void RetryPendingRoutes(std::uint64_t currentTick);
    void ApplyRouteAssignment(
        simulation::DroneId droneId,
        mission::FleetMissionLeg leg,
        std::size_t targetIndex,
        std::vector<glm::dvec3> routeWaypointsMeters,
        std::uint32_t sequence,
        std::uint64_t assignmentVersion,
        std::uint64_t assignmentLeaseExpiresAtTick);

    simulation::SimulationCore& simulation_;
    const scene::TrainingRangeScene& trainingScene_;
    const FleetMissionEnvironment& missionEnvironment_;
    mission::FleetMissionState& missionState_;
    FleetNetworkSession networkSession_;
    std::vector<simulation::DroneId> droneIds_;
};
}
