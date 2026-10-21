#include "simulation_core/CommandRecording.h"

namespace aerosimlab::simulation
{
bool CommandRecording::StartRecording() noexcept
{
    if (recording_ || replaying_)
        return false;

    commands_.clear();
    tickCount_ = 0;
    replayTick_ = 0;
    replayCommandIndex_ = 0;
    recording_ = true;
    return true;
}

bool CommandRecording::StopRecording() noexcept
{
    if (!recording_)
        return false;

    recording_ = false;
    return true;
}

bool CommandRecording::StartReplay() noexcept
{
    if (recording_ || replaying_ || commands_.empty() || tickCount_ == 0)
        return false;

    replayTick_ = 0;
    replayCommandIndex_ = 0;
    replaying_ = true;
    return true;
}

void CommandRecording::StopReplay() noexcept
{
    replaying_ = false;
}

bool CommandRecording::IsRecording() const noexcept
{
    return recording_;
}

bool CommandRecording::IsReplaying() const noexcept
{
    return replaying_;
}

std::uint64_t CommandRecording::TickCount() const noexcept
{
    return tickCount_;
}

bool CommandRecording::ReplayFinished() const noexcept
{
    return replayTick_ >= tickCount_;
}

void CommandRecording::Record(const DroneCommand& command)
{
    if (recording_)
        commands_.push_back({tickCount_, command});
}

void CommandRecording::FinishTick(bool wasRecordingAtTickStart) noexcept
{
    if (recording_ || wasRecordingAtTickStart)
        ++tickCount_;
}

std::span<const CommandRecording::Entry> CommandRecording::CommandsForReplayTick() noexcept
{
    const std::size_t first = replayCommandIndex_;
    while (replayCommandIndex_ < commands_.size()
        && commands_[replayCommandIndex_].tick == replayTick_)
    {
        ++replayCommandIndex_;
    }

    if (first == replayCommandIndex_)
        return {};
    return {commands_.data() + first, replayCommandIndex_ - first};
}

void CommandRecording::AdvanceReplayTick() noexcept
{
    if (replaying_)
        ++replayTick_;
}
}
