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

/** @brief 드론별 수신 상태와 텔레메트리 전송 순서를 관리한다. */
class FleetTelemetry final
{
public:
    /** @brief 이 시간보다 오래된 상태는 편대 제어에서 끊긴 보고로 본다. */
    static constexpr std::uint64_t FreshnessTimeoutTicks = 30U;

    /** @brief 더 새 번호의 상태만 저장하고 저장 여부를 돌려준다. */
    [[nodiscard]] bool Record(simulation::DroneId droneId,
        const PeerTelemetry& state,
        std::uint64_t receivedAtTick,
        std::uint32_t sequence);

    /** @brief 통신이 끊겨도 보존하는 마지막 수신 상태를 찾는다. */
    [[nodiscard]] const ReceivedPeerTelemetry* Latest(
        simulation::DroneId droneId) const noexcept;

    /** @brief 마지막 보고가 현재 모의 시각보다 몇 Tick 전인지 반환한다. */
    [[nodiscard]] std::optional<std::uint64_t> AgeInTicks(
        simulation::DroneId droneId,
        std::uint64_t currentTick) const noexcept;

    /** @brief 정해 둔 신선도 시간 안에 받은 상태만 돌려준다. */
    [[nodiscard]] const ReceivedPeerTelemetry* Fresh(
        simulation::DroneId droneId,
        std::uint64_t currentTick) const noexcept;

    /** @brief 드론 상태를 보낼 시각이 되었는지 확인한다. */
    [[nodiscard]] bool ShouldSend(
        simulation::DroneId droneId,
        std::uint64_t currentTick,
        std::uint64_t intervalTicks) const noexcept;

    /** @brief 성공 여부와 관계없이 드론별 전송 시도 시각을 기록한다. */
    void RecordSendAttempt(simulation::DroneId droneId, std::uint64_t currentTick);

    /** @brief 다음 텔레메트리 메시지에 사용할 순번을 받고 하나 올린다. */
    [[nodiscard]] std::uint32_t TakeNextSequence() noexcept
    {
        return nextSequence_++;
    }

    /** @brief 새 임무를 시작할 때 수신 상태와 전송 기록을 함께 비운다. */
    void Clear() noexcept;

private:
    std::map<simulation::DroneId, ReceivedPeerTelemetry> latestByDrone_;
    std::map<simulation::DroneId, std::uint64_t> lastSendAttemptByDrone_;
    std::uint32_t nextSequence_ = 1U;
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
