#include "simulation_core/SimulationCore.h"
#if !defined(__EMSCRIPTEN__)
#include "simulation_core/PhysicsWorld.h"
#endif
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <flecs.h>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace aerosimlab::simulation
{
struct SimulationCore::EcsStorage
{
    struct InitialPose
    {
        glm::dvec3 positionMeters{0.0};
        double yawRadians = 0.0;
        double groundElevationMeters = 0.0;
    };

#if !defined(__EMSCRIPTEN__)
    struct PhysicsBody
    {
        RigidBodyHandle handle;
    };

    struct DroneActuation
    {
        QuadrotorParameters parameters;
    };
#endif

    SimulationDynamics dynamics;
    flecs::world world;
    std::unordered_map<DroneId, flecs::entity> drones;
    DroneId primaryDroneId = 0;
    DroneId nextDroneId = 1;
    flecs::entity camera;
    bool measurePhysicsStepDuration = false;
    double lastPhysicsStepDurationMicroseconds = 0.0;
#if !defined(__EMSCRIPTEN__)
    std::unique_ptr<PhysicsWorld> physics;
#endif
    std::uint64_t tickCount = 0;

    explicit EcsStorage(SimulationDynamics selectedDynamics)
        : dynamics(selectedDynamics)
    {
        if (dynamics == SimulationDynamics::Kinematic)
        {
            primaryDroneId = CreateDrone(glm::dvec3{0.0}, 0.0);
            world.system<DroneState, DroneControlState>("AdvanceKinematicDrone")
                .kind(flecs::OnUpdate)
                .each([](DroneState& state, DroneControlState& control)
                {
                    DroneModel::AdvanceOneTick(state, control);
                });
        }
#if !defined(__EMSCRIPTEN__)
        else
        {
            physics = std::make_unique<PhysicsWorld>();
            primaryDroneId = CreateDrone(glm::dvec3{
                0.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters}, 0.0);
            world.system<DroneState, DroneControlState, PhysicsBody, DroneActuation>(
                "ApplyRigidBodyForces")
                .kind(flecs::OnUpdate)
                .each([this](
                    DroneState& state,
                    DroneControlState& control,
                    const PhysicsBody& body,
                    DroneActuation& actuation)
                {
                    ApplyRigidBodyForces(state, control, body.handle, actuation);
                });
        }
#else
        else
        {
            primaryDroneId = CreateDrone(glm::dvec3{0.0}, 0.0);
        }
#endif

        camera = world.entity("EoCamera").set<CameraState>({});
        world.system<CameraState>("AdvanceCamera")
            .kind(flecs::OnUpdate)
            .each([this](CameraState& state)
            {
                EoCameraModel::AdvanceOneTick(state, tickCount + 1);
            });
        SynchronizeCameraPose();
    }

    DroneId CreateDrone(const glm::dvec3& positionMeters, double yawRadians)
    {
        const DroneId id = nextDroneId++;
        DroneState initialState;
        initialState.positionMeters = positionMeters;
        initialState.yawRadians = yawRadians;
        initialState.orientation = glm::angleAxis(yawRadians, glm::dvec3{0.0, 0.0, 1.0});
        DroneControlState initialControl;
        initialControl.desiredYawRadians = yawRadians;
        flecs::entity drone = world.entity()
            .set<DroneState>(initialState)
            .set<DroneControlState>(initialControl)
            .set<InitialPose>({positionMeters, yawRadians, 0.0});

#if !defined(__EMSCRIPTEN__)
        if (dynamics == SimulationDynamics::RigidBody)
        {
            const QuadrotorParameters parameters = QuadrotorParameters::X500MotorTableEstimate();
            RigidBodyDefinition definition;
            definition.positionMeters = positionMeters;
            definition.orientation = initialState.orientation;
            const RigidBodyHandle body = physics->CreateQuadrotor(definition, parameters);
            drone.set<PhysicsBody>({body});
            drone.set<DroneActuation>({parameters});
            CopyPhysicsState(drone.get_mut<DroneState>(), body);
        }
#endif

        drones.emplace(id, drone);
        return id;
    }

    [[nodiscard]] flecs::entity& FindDrone(DroneId id)
    {
        const auto found = drones.find(id);
        if (found == drones.end())
            throw std::out_of_range("드론 식별값이 없습니다.");
        return found->second;
    }

    [[nodiscard]] const flecs::entity& FindDrone(DroneId id) const
    {
        const auto found = drones.find(id);
        if (found == drones.end())
            throw std::out_of_range("드론 식별값이 없습니다.");
        return found->second;
    }

    void SynchronizeCameraPose()
    {
        EoCameraModel::UpdatePose(
            camera.get_mut<CameraState>(),
            FindDrone(primaryDroneId).get<DroneState>());
    }

    bool RemoveDrone(DroneId id)
    {
        if (drones.size() <= 1)
            return false;
        const auto found = drones.find(id);
        if (found == drones.end())
            return false;

#if !defined(__EMSCRIPTEN__)
        if (dynamics == SimulationDynamics::RigidBody
            && !physics->DestroyBody(found->second.get<PhysicsBody>().handle))
            return false;
#endif

        found->second.destruct();
        const bool removedPrimary = id == primaryDroneId;
        drones.erase(found);
        if (removedPrimary)
        {
            primaryDroneId = drones.begin()->first;
            SynchronizeCameraPose();
        }
        return true;
    }

#if !defined(__EMSCRIPTEN__)
    void ResetPhysicsBody(flecs::entity& drone)
    {
        RigidBodyState initialState;
        const InitialPose& initialPose = drone.get<InitialPose>();
        initialState.positionMeters = initialPose.positionMeters;
        initialState.orientation = glm::angleAxis(
            initialPose.yawRadians, glm::dvec3{0.0, 0.0, 1.0});
        const RigidBodyHandle body = drone.get<PhysicsBody>().handle;
        physics->SetState(body, initialState);
        CopyPhysicsState(drone.get_mut<DroneState>(), body);
    }

    void ApplyRigidBodyForces(
        DroneState& state,
        DroneControlState& control,
        RigidBodyHandle body,
        DroneActuation& actuation)
    {
        const double step = DroneModel::fixedStepSeconds;
        if (state.mode != FlightMode::Grounded)
            control.desiredYawRadians += control.commandedYawRateRadiansPerSecond * step;

        const auto targetSpeeds = QuadrotorFlightController::MotorTargets(
            state, control, actuation.parameters, step);
        physics->SetMotorTargets(body, targetSpeeds);
    }

    void SynchronizeRigidBody(
        DroneState& state,
        DroneControlState& control,
        RigidBodyHandle body)
    {
        CopyPhysicsState(state, body);

        if (state.mode == FlightMode::TakingOff
            && state.positionMeters.z >= control.groundElevationMeters
                + DroneModel::takeoffHeightMeters - 0.03
            && std::abs(state.velocityMetersPerSecond.z) < 0.15)
        {
            state.mode = FlightMode::Hovering;
        }
        else if (state.mode == FlightMode::FollowingWaypoint
            && glm::length(control.waypointMeters - state.positionMeters) < 0.15
            && glm::length(state.velocityMetersPerSecond) < 0.15)
        {
            state.mode = FlightMode::Hovering;
            control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
        }
        else if (state.mode == FlightMode::Landing
            && state.positionMeters.z <= control.groundElevationMeters
                + QuadrotorFlightController::groundedModelOriginHeightMeters + 0.02
            && std::abs(state.velocityMetersPerSecond.z) < 0.15)
        {
            state.mode = FlightMode::Grounded;
            control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
            control.commandedYawRateRadiansPerSecond = 0.0;
        }
    }

    void CopyPhysicsState(DroneState& target, RigidBodyHandle body) const
    {
        const RigidBodyState source = physics->State(body);
        target.positionMeters = source.positionMeters;
        target.velocityMetersPerSecond = source.linearVelocityMetersPerSecond;
        target.orientation = source.orientation;
        target.angularVelocityRadiansPerSecond = source.angularVelocityRadiansPerSecond;
        target.motorSpeedsRadiansPerSecond = physics->MotorSpeedsRadiansPerSecond(body);
        const glm::dquat& q = source.orientation;
        target.yawRadians = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void AdvancePhysicsWorld()
    {
        if (measurePhysicsStepDuration)
            physics->Step(DroneModel::fixedStepSeconds,
                &lastPhysicsStepDurationMicroseconds);
        else
            physics->Step(DroneModel::fixedStepSeconds);

        for (auto& [id, drone] : drones)
        {
            (void)id;
            SynchronizeRigidBody(
                drone.get_mut<DroneState>(),
                drone.get_mut<DroneControlState>(),
                drone.get<PhysicsBody>().handle);
        }
    }
#endif
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
    return Apply(ecs_->primaryDroneId, command);
}

CommandResult SimulationCore::Apply(DroneId droneId, const DroneCommand& command) noexcept
{
    if (paused_)
    {
        return CommandResult::SimulationPaused;
    }
    const auto found = ecs_->drones.find(droneId);
    if (found == ecs_->drones.end())
        return CommandResult::DroneNotFound;

    flecs::entity& drone = found->second;
    DroneState& state = drone.get_mut<DroneState>();
    DroneControlState& control = drone.get_mut<DroneControlState>();
#if defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        return CommandResult::UnsupportedDynamics;
#endif
    const CommandResult result = DroneModel::Apply(state, control, command);
#if !defined(__EMSCRIPTEN__)
    if (result == CommandResult::Accepted
        && ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->CopyPhysicsState(state, drone.get<EcsStorage::PhysicsBody>().handle);
    }
#endif
    return result;
}

DroneId SimulationCore::AddDrone(const glm::dvec3& positionMeters, double yawRadians)
{
    if (!std::isfinite(positionMeters.x)
        || !std::isfinite(positionMeters.y)
        || !std::isfinite(positionMeters.z)
        || !std::isfinite(yawRadians))
        throw std::invalid_argument("드론 시작 위치와 방향은 유한한 값이어야 합니다.");
    return ecs_->CreateDrone(positionMeters, yawRadians);
}

void SimulationCore::SetInitialPose(const glm::dvec3& positionMeters, double yawRadians)
{
    if (ecs_->tickCount != 0)
        throw std::logic_error("드론 시작 자세는 첫 Tick 전에만 바꿀 수 있습니다.");
    if (!std::isfinite(positionMeters.x) || !std::isfinite(positionMeters.y)
        || !std::isfinite(positionMeters.z) || !std::isfinite(yawRadians))
        throw std::invalid_argument("드론 시작 위치와 방향은 유한한 값이어야 합니다.");

    flecs::entity& drone = ecs_->FindDrone(ecs_->primaryDroneId);
    const double groundElevationMeters = ecs_->dynamics == SimulationDynamics::RigidBody
        ? positionMeters.z - QuadrotorFlightController::groundedModelOriginHeightMeters
        : positionMeters.z;
    drone.set<EcsStorage::InitialPose>(
        {positionMeters, yawRadians, groundElevationMeters});
    DroneState& state = drone.get_mut<DroneState>();
    state.positionMeters = positionMeters;
    state.yawRadians = yawRadians;
    state.orientation = glm::angleAxis(yawRadians, glm::dvec3{0.0, 0.0, 1.0});
    DroneControlState& control = drone.get_mut<DroneControlState>();
    control = {};
    control.desiredYawRadians = yawRadians;
    control.groundElevationMeters = groundElevationMeters;
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        ecs_->ResetPhysicsBody(drone);
#endif
    ecs_->SynchronizeCameraPose();
}

bool SimulationCore::RemoveDrone(DroneId droneId)
{
    return ecs_->RemoveDrone(droneId);
}

DroneId SimulationCore::PrimaryDroneId() const noexcept
{
    return ecs_->primaryDroneId;
}

std::size_t SimulationCore::DroneCount() const noexcept
{
    return ecs_->drones.size();
}

void SimulationCore::AdvanceOneTick() noexcept
{
    if (paused_)
    {
        return;
    }
    ecs_->lastPhysicsStepDurationMicroseconds = 0.0;
    ecs_->world.progress(DroneModel::fixedStepSeconds);
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        ecs_->AdvancePhysicsWorld();
#endif
    ++ecs_->tickCount;
    ecs_->SynchronizeCameraPose();
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
    for (auto& [id, drone] : ecs_->drones)
    {
        (void)id;
        DroneModel::Reset(drone.get_mut<DroneState>(), drone.get_mut<DroneControlState>());
        const EcsStorage::InitialPose& initialPose = drone.get<EcsStorage::InitialPose>();
        DroneState& state = drone.get_mut<DroneState>();
        DroneControlState& control = drone.get_mut<DroneControlState>();
        state.positionMeters = initialPose.positionMeters;
        state.yawRadians = initialPose.yawRadians;
        state.orientation = glm::angleAxis(
            initialPose.yawRadians, glm::dvec3{0.0, 0.0, 1.0});
        control.desiredYawRadians = initialPose.yawRadians;
        control.groundElevationMeters = initialPose.groundElevationMeters;
#if !defined(__EMSCRIPTEN__)
        if (ecs_->dynamics == SimulationDynamics::RigidBody)
            ecs_->ResetPhysicsBody(drone);
#endif
    }
    EoCameraModel::Reset(ecs_->camera.get_mut<CameraState>());
    ecs_->tickCount = 0;
    paused_ = false;
    ecs_->SynchronizeCameraPose();
}

const DroneState& SimulationCore::State() const noexcept
{
    return ecs_->drones.at(ecs_->primaryDroneId).get<DroneState>();
}

const DroneState& SimulationCore::State(DroneId droneId) const
{
    return ecs_->FindDrone(droneId).get<DroneState>();
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

void SimulationCore::SetWindVelocityMetersPerSecond(const glm::dvec3& velocity)
{
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z))
        throw std::invalid_argument("바람 속도는 유한한 값이어야 합니다.");
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        ecs_->physics->SetWindVelocityMetersPerSecond(velocity);
#endif
}

void SimulationCore::SetPhysicsStepTimingEnabled(bool enabled) noexcept
{
    ecs_->measurePhysicsStepDuration = enabled;
    ecs_->lastPhysicsStepDurationMicroseconds = 0.0;
}

double SimulationCore::LastPhysicsStepDurationMicroseconds() const noexcept
{
    return ecs_->lastPhysicsStepDurationMicroseconds;
}

#ifndef __EMSCRIPTEN__
void SimulationCore::AddStaticObstacleBox(
    const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters)
{
    if (!ecs_->physics)
        throw std::logic_error("고정 장애물은 강체 모의에서만 추가할 수 있습니다.");

    ecs_->physics->AddStaticObstacleBox(centerMeters, halfExtentsMeters);
}

void SimulationCore::SetTerrain(terrain::TerrainGrid terrain)
{
    if (!ecs_->physics)
        throw std::logic_error("지형 충돌은 강체 모의에서만 설정할 수 있습니다.");

    ecs_->physics->SetTerrain(std::move(terrain));
}
#endif

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
