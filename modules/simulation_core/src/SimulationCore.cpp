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
    camera_.AdvanceOneTick(tickCount_ + 1);
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
    camera_.Reset();
    tickCount_ = 0;
    paused_ = false;
}

const DroneState& SimulationCore::State() const noexcept
{
    return drone_.State();
}

const CameraState& SimulationCore::Camera() const noexcept
{
    return camera_.State();
}

void SimulationCore::SetCameraEnabled(bool enabled) noexcept
{
    camera_.SetEnabled(enabled);
}

void SimulationCore::SetCameraFault(bool faulted) noexcept
{
    if (faulted)
        camera_.InjectFault();
    else
        camera_.Recover();
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
