#include "simulation_core/SimulationCore.h"

namespace aerosimlab::simulation
{
CommandResult SimulationCore::Apply(const DroneCommand& command) noexcept
{
    if (paused_)
    {
        return CommandResult::SimulationPaused;
    }
    return drone_.Apply(command);
}

void SimulationCore::AdvanceOneTick() noexcept
{
    if (paused_)
    {
        return;
    }
    drone_.AdvanceOneTick();
    ++tickCount_;
}

void SimulationCore::Pause() noexcept
{
    paused_ = true;
}

void SimulationCore::Resume() noexcept
{
    paused_ = false;
}

void SimulationCore::Reset() noexcept
{
    drone_.Reset();
    tickCount_ = 0;
    paused_ = false;
}

const DroneState& SimulationCore::State() const noexcept
{
    return drone_.State();
}

std::uint64_t SimulationCore::TickCount() const noexcept
{
    return tickCount_;
}

double SimulationCore::SimulationTimeSeconds() const noexcept
{
    return static_cast<double>(tickCount_) * DroneModel::fixedStepSeconds;
}

bool SimulationCore::IsPaused() const noexcept
{
    return paused_;
}
}
