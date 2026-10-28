#pragma once

#include "simulation_core/SimulationCore.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace aerosimlab::simulator
{
/** @brief 통신 장애 조작을 모의 Tick에 맞춰 저장하고 다시 적용한다. */
class FaultScenarioTimeline
{
public:
    struct Event
    {
        enum class Kind
        {
            PacketLoss,
            LinkState,
            DirectedLinkState,
            RadioState
        };

        std::uint64_t tickOffset = 0;
        Kind kind = Kind::PacketLoss;
        simulation::DroneId source = 0;
        simulation::DroneId destination = 0;
        double value = 0.0;
        bool enabled = false;
    };

    /** @brief 기록을 비우고 현재 드론 구성을 기준으로 새로 시작한다. */
    void StartRecording(std::uint64_t currentTick,
        const std::vector<simulation::DroneId>& droneIds,
        std::string missionKind);
    void StopRecording() noexcept;
    /** @brief 기록한 장애와 드론 ID를 파일에 저장한다. */
    [[nodiscard]] bool SaveRecording(const std::filesystem::path& path,
        std::string& error) const;
    /** @brief 파일에서 장애 기록을 읽는다. */
    [[nodiscard]] bool LoadRecording(const std::filesystem::path& path,
        std::string& error);
    /** @brief 현재 모의 시각을 기준으로 기록 재생을 시작한다. */
    [[nodiscard]] bool StartReplay(std::uint64_t currentTick,
        const std::vector<simulation::DroneId>& droneIds,
        std::string_view missionKind, std::string& error);
    void StopReplay() noexcept;

    void RecordPacketLoss(std::uint64_t currentTick, double probability);
    void RecordLinkState(std::uint64_t currentTick, simulation::DroneId source,
        simulation::DroneId destination, bool disabled);
    void RecordDirectedLinkState(std::uint64_t currentTick,
        simulation::DroneId source, simulation::DroneId destination, bool disabled);
    void RecordRadioState(std::uint64_t currentTick,
        simulation::DroneId droneId, bool enabled);

    /** @brief 재생 시각에 도달한 이벤트를 돌려주고 다음 이벤트로 이동한다. */
    [[nodiscard]] std::vector<Event> TakeDueEvents(std::uint64_t currentTick);
    [[nodiscard]] bool IsRecording() const noexcept;
    [[nodiscard]] bool IsReplaying() const noexcept;
    [[nodiscard]] bool ReplayFinished() const noexcept;
    [[nodiscard]] std::size_t EventCount() const noexcept;

private:
    void Record(Event event, std::uint64_t currentTick);

    std::vector<simulation::DroneId> droneIds_;
    std::string missionKind_;
    std::vector<Event> events_;
    std::uint64_t recordingStartTick_ = 0;
    std::uint64_t replayStartTick_ = 0;
    std::size_t replayEventIndex_ = 0;
    bool recording_ = false;
    bool replaying_ = false;
};
}
