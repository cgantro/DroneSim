#include "simulation_core/SimulationCore.h"

#include <flecs.h>

#include <algorithm>
#include <memory>

namespace aerosimlab::simulation
{
struct SimulationCore::EcsStorage
{
    flecs::world world;
    flecs::entity drone;
    flecs::entity camera;
    std::uint64_t tickCount = 0;

    EcsStorage()
    {
        drone = world.entity("Drone")
            .set<DroneState>({})
            .set<DroneControlState>({});
        camera = world.entity("EoCamera").set<CameraState>({});

        world.system<DroneState, DroneControlState>("AdvanceDrone")
            .kind(flecs::OnUpdate)
            .each([](DroneState& state, DroneControlState& control)
            {
                DroneModel::AdvanceOneTick(state, control);
            });

        world.system<CameraState>("AdvanceCamera")
            .kind(flecs::OnUpdate)
            .each([this](CameraState& state)
            {
                EoCameraModel::AdvanceOneTick(state, tickCount + 1);
            });
    }
};

std::size_t FixedTickAccumulator::Consume(double elapsedSeconds) noexcept
{
    accumulatedSeconds_ += std::min(elapsedSeconds, maximumFrameSeconds);

    std::size_t tickCount = 0;
    while (accumulatedSeconds_ >= fixedStepSeconds)
    {
        accumulatedSeconds_ -= fixedStepSeconds;
        ++tickCount;
    }
    return tickCount;
}

SimulationCore::SimulationCore()
    : ecs_(std::make_unique<EcsStorage>())
{
}

SimulationCore::~SimulationCore() = default;

CommandResult SimulationCore::Apply(const DroneCommand& command) noexcept
{
    if (paused_)
    {
        return CommandResult::SimulationPaused;
    }
    return DroneModel::Apply(
        ecs_->drone.get_mut<DroneState>(),
        ecs_->drone.get_mut<DroneControlState>(),
        command);
}

void SimulationCore::AdvanceOneTick() noexcept
{
    if (paused_)
    {
        return;
    }
    ecs_->world.progress(DroneModel::fixedStepSeconds);
    ++ecs_->tickCount;
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
    DroneModel::Reset(
        ecs_->drone.get_mut<DroneState>(),
        ecs_->drone.get_mut<DroneControlState>());
    EoCameraModel::Reset(ecs_->camera.get_mut<CameraState>());
    ecs_->tickCount = 0;
    paused_ = false;
}

const DroneState& SimulationCore::State() const noexcept
{
    return ecs_->drone.get<DroneState>();
}

const CameraState& SimulationCore::Camera() const noexcept
{
    return ecs_->camera.get<CameraState>();
}

void SimulationCore::SetCameraEnabled(bool enabled) noexcept
{
    EoCameraModel::SetEnabled(ecs_->camera.get_mut<CameraState>(), enabled);
}

void SimulationCore::SetCameraFault(bool faulted) noexcept
{
    if (faulted)
        EoCameraModel::InjectFault(ecs_->camera.get_mut<CameraState>());
    else
        EoCameraModel::Recover(ecs_->camera.get_mut<CameraState>());
}

std::uint64_t SimulationCore::TickCount() const noexcept
{
    return ecs_->tickCount;
}

double SimulationCore::SimulationTimeSeconds() const noexcept
{
    return static_cast<double>(ecs_->tickCount) * DroneModel::fixedStepSeconds;
}

bool SimulationCore::IsPaused() const noexcept
{
    return paused_;
}
}
