#pragma once

#include "FleetMissionProgress.h"
#include "simulation_core/SimulationCore.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <map>
#include <string>

namespace dronesim::mission
{
struct PeerTelemetry
{
    glm::dvec3 positionMeters{0.0};
    simulation::FlightMode mode = simulation::FlightMode::Grounded;
    std::uint8_t missionLeg = 0;
    FleetDroneStatus missionStatus = FleetDroneStatus::Other;
    std::uint64_t assignmentVersion = 0;
};

/** @brief 통신으로 받은 최신 기체 상태와 수신 시점을 보관한다. */
struct ReceivedPeerTelemetry
{
    PeerTelemetry state;
    std::uint64_t receivedAtTick = 0;
    std::uint32_t sequence = 0;
};

/** @brief 드론별 마지막 상태를 보관하고 일정 시간 지난 보고를 구분한다. */
class FleetTelemetryStore final
{
public:
    /** @brief 더 새 번호의 상태만 저장하고 저장 여부를 돌려준다. */
    [[nodiscard]] bool Record(simulation::DroneId droneId,
        const PeerTelemetry& state,
        std::uint64_t receivedAtTick,
        std::uint32_t sequence);

    /** @brief 통신이 끊겨도 보존하는 마지막 수신 상태를 찾는다. */
    [[nodiscard]] const ReceivedPeerTelemetry* Latest(
        simulation::DroneId droneId) const noexcept;

    /** @brief 30 Tick 안에 받은 최신 상태만 돌려준다. */
    [[nodiscard]] const ReceivedPeerTelemetry* Fresh(
        simulation::DroneId droneId,
        std::uint64_t currentTick) const noexcept;

    /** @brief 임무를 새로 시작할 때 이전 임무에서 받은 상태를 비운다. */
    void Clear() noexcept;

private:
    std::map<simulation::DroneId, ReceivedPeerTelemetry> latestByDrone_;
};

[[nodiscard]] std::string EncodePeerTelemetry(
    const glm::dvec3& positionMeters,
    simulation::FlightMode mode,
    std::uint8_t missionLeg,
    FleetDroneStatus missionStatus,
    std::uint64_t assignmentVersion);

[[nodiscard]] std::optional<PeerTelemetry> DecodePeerTelemetry(
    const std::string& payload);
}
