#include "simulation_core/SimulationCore.h"
#include "simulation_core/PhysicsWorld.h"
#include "simulation_core/QuadrotorModel.h"

#include <flecs.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace aerosimlab::simulation
{
struct SimulationCore::EcsStorage
{
    struct PhysicsBody
    {
        RigidBodyHandle handle;
    };

    SimulationDynamics dynamics;
    flecs::world world;
    flecs::entity drone;
    flecs::entity camera;
    std::unique_ptr<PhysicsWorld> physics;
    std::uint64_t tickCount = 0;

    explicit EcsStorage(SimulationDynamics selectedDynamics)
        : dynamics(selectedDynamics)
    {
        drone = world.entity("Drone")
            .set<DroneState>({})
            .set<DroneControlState>({});
        camera = world.entity("EoCamera").set<CameraState>({});

        if (dynamics == SimulationDynamics::Kinematic)
        {
            world.system<DroneState, DroneControlState>("AdvanceKinematicDrone")
                .kind(flecs::OnUpdate)
                .each([](DroneState& state, DroneControlState& control)
                {
                    DroneModel::AdvanceOneTick(state, control);
                });
        }
        else
        {
            CreatePhysicsBody();
            world.system<DroneState, PhysicsBody>("AdvanceRigidBodyDrone")
                .kind(flecs::OnUpdate)
                .each([this](DroneState& state, const PhysicsBody& body)
                {
                    physics->Step(DroneModel::fixedStepSeconds);
                    CopyPhysicsState(state, body.handle);
                });
        }

        world.system<CameraState>("AdvanceCamera")
            .kind(flecs::OnUpdate)
            .each([this](CameraState& state)
            {
                EoCameraModel::AdvanceOneTick(state, tickCount + 1);
            });
    }

    void CreatePhysicsBody()
    {
        const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
        // 기체 충돌 형상이 준비되기 전까지 지면 접촉을 확인할 임시 상자다.
        constexpr glm::vec3 halfExtentsMeters{0.2F, 0.2F, 0.05F};
        physics = std::make_unique<PhysicsWorld>();
        RigidBodyDefinition definition;
        definition.halfExtentsMeters = halfExtentsMeters;
        definition.positionMeters = {0.0, 0.0, halfExtentsMeters.z};
        definition.massKilograms = parameters.massKilograms;
        definition.principalInertiaKilogramMetersSquared =
            parameters.principalInertiaKilogramMetersSquared;
        const RigidBodyHandle handle = physics->CreateBox(definition);
        drone.set<PhysicsBody>({handle});
        CopyPhysicsState(drone.get_mut<DroneState>(), handle);
    }

    void CopyPhysicsState(DroneState& target, RigidBodyHandle body) const
    {
        const RigidBodyState source = physics->State(body);
        target.positionMeters = source.positionMeters;
        target.velocityMetersPerSecond = source.linearVelocityMetersPerSecond;
        target.orientation = source.orientation;
        target.angularVelocityRadiansPerSecond = source.angularVelocityRadiansPerSecond;
        const glm::dquat& q = source.orientation;
        target.yawRadians = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
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

SimulationCore::SimulationCore(SimulationDynamics dynamics)
    : ecs_(std::make_unique<EcsStorage>(dynamics))
{
}

SimulationCore::~SimulationCore() = default;

CommandResult SimulationCore::Apply(const DroneCommand& command) noexcept
{
    if (paused_)
    {
        return CommandResult::SimulationPaused;
    }
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        return CommandResult::UnsupportedDynamics;
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
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->CreatePhysicsBody();
    }
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
