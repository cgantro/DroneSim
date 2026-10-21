#pragma once

#include "simulation_core/DroneModel.h"

#include <cstdint>
#include <span>
#include <vector>

namespace aerosimlab::simulation
{
/**
 * @brief 드론 명령을 모의 Tick에 맞춰 기록하고 다시 꺼낸다.
 * @details 기록 시작·종료 Tick도 모의 시간에 포함한다. 재생은 기록된 각 Tick의 명령을 한 번씩 제공한다.
 */
class CommandRecording
{
public:
    /** @brief 기록된 명령과 그 명령을 적용할 모의 Tick이다. */
    struct Entry
    {
        std::uint64_t tick = 0;
        DroneCommand command;
    };

    /** @brief 새 기록을 시작하고 이전 명령과 Tick 수를 지운다. */
    [[nodiscard]] bool StartRecording() noexcept;
    /** @brief 진행 중인 기록을 끝낸다. */
    [[nodiscard]] bool StopRecording() noexcept;
    /** @brief 저장된 기록을 처음부터 재생할 준비를 한다. */
    [[nodiscard]] bool StartReplay() noexcept;
    /** @brief 현재 재생을 끝낸다. */
    void StopReplay() noexcept;

    /** @brief 현재 기록·재생 상태를 반환한다. */
    [[nodiscard]] bool IsRecording() const noexcept;
    [[nodiscard]] bool IsReplaying() const noexcept;
    /** @brief 저장한 모의 Tick 수를 반환한다. */
    [[nodiscard]] std::uint64_t TickCount() const noexcept;
    /** @brief 현재 재생이 기록의 마지막 Tick에 도달했는지 반환한다. */
    [[nodiscard]] bool ReplayFinished() const noexcept;

    /** @brief 기록 중이면 명령을 현재 Tick에 저장한다. */
    void Record(const DroneCommand& command);
    /**
     * @brief 끝난 모의 Tick을 기록 길이에 반영한다.
     * @param wasRecordingAtTickStart 이 Tick에서 기록 종료 명령을 처리했다면 true다.
     */
    void FinishTick(bool wasRecordingAtTickStart = false) noexcept;

    /** @brief 현재 재생 Tick의 명령을 반환하고 다음 Tick 명령 위치로 이동한다. */
    [[nodiscard]] std::span<const Entry> CommandsForReplayTick() noexcept;
    /** @brief 재생 Tick을 한 단계 진행한다. */
    void AdvanceReplayTick() noexcept;

private:
    std::vector<Entry> commands_;
    std::uint64_t tickCount_ = 0;
    std::uint64_t replayTick_ = 0;
    std::size_t replayCommandIndex_ = 0;
    bool recording_ = false;
    bool replaying_ = false;
};
}
