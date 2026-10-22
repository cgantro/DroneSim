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
namespace
{
bool HasFinitePose(const glm::dvec3& positionMeters, double yawRadians) noexcept
{
    return std::isfinite(positionMeters.x)
        && std::isfinite(positionMeters.y)
        && std::isfinite(positionMeters.z)
        && std::isfinite(yawRadians);
}
}

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
    double lastContactGenerationDurationMicroseconds = 0.0;
#if !defined(__EMSCRIPTEN__)
    std::unique_ptr<PhysicsWorld> physics;
#endif
    std::uint64_t tickCount = 0;

    static void ApplyInitialPose(
        DroneState& state, DroneControlState& control, const InitialPose& pose) noexcept
    {
        state.positionMeters = pose.positionMeters;
        state.yawRadians = pose.yawRadians;
        state.orientation = glm::angleAxis(
            pose.yawRadians, glm::dvec3{0.0, 0.0, 1.0});
        control = {};
        control.desiredYawRadians = pose.yawRadians;
        control.groundElevationMeters = pose.groundElevationMeters;
    }

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

    DroneId CreateDrone(
        const glm::dvec3& positionMeters, double yawRadians, double groundElevationMeters = 0.0)
    {
        const DroneId id = nextDroneId++;
        const InitialPose initialPose{positionMeters, yawRadians, groundElevationMeters};
        DroneState initialState;
        DroneControlState initialControl;
        ApplyInitialPose(initialState, initialControl, initialPose);
        flecs::entity drone = world.entity();
#if !defined(__EMSCRIPTEN__)
        RigidBodyHandle createdBody{};
        bool hasCreatedBody = false;
#endif

        try
        {
            drone.set<DroneState>(initialState)
                .set<DroneControlState>(initialControl)
                .set<InitialPose>(initialPose);
#if !defined(__EMSCRIPTEN__)
            if (dynamics == SimulationDynamics::RigidBody)
            {
                const QuadrotorParameters parameters =
                    QuadrotorParameters::X500MotorTableEstimate();
                RigidBodyDefinition definition;
                definition.positionMeters = positionMeters;
                definition.orientation = initialState.orientation;
                createdBody = physics->CreateQuadrotor(definition, parameters);
                hasCreatedBody = true;
                drone.set<PhysicsBody>({createdBody});
                drone.set<DroneActuation>({parameters});
                CopyPhysicsState(drone.get_mut<DroneState>(), createdBody);
            }
#endif

            drones.emplace(id, drone);
        }
        catch (...)
        {
#if !defined(__EMSCRIPTEN__)
            if (hasCreatedBody)
                static_cast<void>(physics->DestroyBody(createdBody));
#endif
            // ECS에만 남은 엔티티가 다음 시스템 실행에 섞이지 않도록 함께 지운다.
            drone.destruct();
            throw;
        }
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
            state, control, actuation.parameters);
        physics->SetMotorTargets(body, targetSpeeds);
    }

    void SynchronizeRigidBody(
        DroneState& state,
        DroneControlState& control,
        RigidBodyHandle body)
    {
        CopyPhysicsState(state, body);

        switch (state.mode)
        {
        case FlightMode::TakingOff:
            if (state.positionMeters.z >= control.groundElevationMeters
                    + DroneModel::takeoffHeightMeters - 0.03
                && std::abs(state.velocityMetersPerSecond.z) < 0.15)
            {
                state.mode = FlightMode::Hovering;
            }
            break;

        case FlightMode::FollowingWaypoint:
            if (glm::length(control.waypointMeters - state.positionMeters) < 0.15
                && glm::length(state.velocityMetersPerSecond) < 0.15)
            {
                state.mode = FlightMode::Hovering;
                control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
            }
            break;

        case FlightMode::Landing:
            if (state.positionMeters.z <= control.groundElevationMeters
                    + QuadrotorFlightController::groundedModelOriginHeightMeters + 0.02
                && std::abs(state.velocityMetersPerSecond.z) < 0.15)
            {
                state.mode = FlightMode::Grounded;
                control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
                control.commandedYawRateRadiansPerSecond = 0.0;
            }
            break;

        case FlightMode::Grounded:
        case FlightMode::Hovering:
        case FlightMode::Flying:
            break;
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
                &lastPhysicsStepDurationMicroseconds,
                &lastContactGenerationDurationMicroseconds);
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
    if (std::isnan(elapsedSeconds) || elapsedSeconds <= 0.0)
        return 0;

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
    return AddDrone(positionMeters, yawRadians, 0.0);
}

DroneId SimulationCore::AddDrone(const glm::dvec3& positionMeters,
    double yawRadians, double groundElevationMeters)
{
    if (!HasFinitePose(positionMeters, yawRadians))
        throw std::invalid_argument("드론 시작 위치와 방향은 유한한 값이어야 합니다.");
    if (!std::isfinite(groundElevationMeters))
        throw std::invalid_argument("드론 지면 높이는 유한한 값이어야 합니다.");
    return ecs_->CreateDrone(positionMeters, yawRadians, groundElevationMeters);
}

void SimulationCore::SetInitialPose(const glm::dvec3& positionMeters, double yawRadians)
{
    if (ecs_->tickCount != 0)
        throw std::logic_error("드론 시작 자세는 첫 Tick 전에만 바꿀 수 있습니다.");
    if (!HasFinitePose(positionMeters, yawRadians))
        throw std::invalid_argument("드론 시작 위치와 방향은 유한한 값이어야 합니다.");

    flecs::entity& drone = ecs_->FindDrone(ecs_->primaryDroneId);
    const double groundElevationMeters = ecs_->dynamics == SimulationDynamics::RigidBody
        ? positionMeters.z - QuadrotorFlightController::groundedModelOriginHeightMeters
        : positionMeters.z;
    const EcsStorage::InitialPose initialPose{
        positionMeters, yawRadians, groundElevationMeters};
    drone.set<EcsStorage::InitialPose>(initialPose);
    EcsStorage::ApplyInitialPose(
        drone.get_mut<DroneState>(), drone.get_mut<DroneControlState>(), initialPose);
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

std::vector<DroneId> SimulationCore::DroneIds() const
{
    std::vector<DroneId> ids;
    ids.reserve(ecs_->drones.size());
    for (const auto& [id, drone] : ecs_->drones)
    {
        (void)drone;
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

void SimulationCore::AdvanceOneTick() noexcept
{
    if (paused_)
    {
        return;
    }
    ecs_->lastPhysicsStepDurationMicroseconds = 0.0;
    ecs_->lastContactGenerationDurationMicroseconds = 0.0;
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
        EcsStorage::ApplyInitialPose(
            drone.get_mut<DroneState>(), drone.get_mut<DroneControlState>(), initialPose);
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
    ecs_->lastContactGenerationDurationMicroseconds = 0.0;
}

double SimulationCore::LastPhysicsStepDurationMicroseconds() const noexcept
{
    return ecs_->lastPhysicsStepDurationMicroseconds;
}

double SimulationCore::LastContactGenerationDurationMicroseconds() const noexcept
{
    return ecs_->lastContactGenerationDurationMicroseconds;
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
