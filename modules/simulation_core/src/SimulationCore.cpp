#include "simulation_core/SimulationCore.h"
#if !defined(__EMSCRIPTEN__)
#include "simulation_core/PhysicsWorld.h"
#endif
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/ScopedDuration.h"

#include <flecs.h>
#include <glm/common.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

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
    struct StaticObstacleBox
    {
        glm::dvec3 centerMeters{0.0};
        glm::dvec3 halfExtentsMeters{0.0};
    };

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

#endif

    SimulationDynamics dynamics;
    QuadrotorControllerParameters controllerParameters;
#if !defined(__EMSCRIPTEN__)
    QuadrotorParameters quadrotorParameters;
    /** @brief Flecs 콜백이 참조하므로 Flecs 월드보다 나중에 정리한다. */
    std::unique_ptr<PhysicsWorld> physics;
#endif
    flecs::world world;
    std::unordered_map<DroneId, flecs::entity> drones;
    std::unordered_map<DroneId, BatteryModel> batteries;
    DroneId primaryDroneId = 0;
    DroneId nextDroneId = 1;
    std::vector<StaticObstacleBox> kinematicObstacles;
    std::optional<terrain::TerrainGrid> terrainGrid;
    bool measureEcsSystemDuration = false;
    EcsSystemDurations lastEcsSystemDurations;
    double lastFlecsProgressDurationMicroseconds = 0.0;
    bool measurePhysicsStepDuration = false;
    double lastPhysicsStepDurationMicroseconds = 0.0;
    double lastContactGenerationDurationMicroseconds = 0.0;
    std::uint64_t tickCount = 0;

    static void ApplyInitialPose(
        DroneState& state, DroneControlState& control, const InitialPose& pose) noexcept
    {
        DroneModel::Reset(state, control);
        state.positionMeters = pose.positionMeters;
        state.yawRadians = pose.yawRadians;
        state.orientation = glm::angleAxis(
            pose.yawRadians, glm::dvec3{0.0, 0.0, 1.0});
        control.desiredYawRadians = pose.yawRadians;
        control.groundElevationMeters = pose.groundElevationMeters;
    }

    explicit EcsStorage(
        SimulationDynamics selectedDynamics,
        QuadrotorParameters selectedQuadrotorParameters,
        QuadrotorControllerParameters selectedControllerParameters = {})
        : dynamics(selectedDynamics)
        , controllerParameters(selectedControllerParameters)
#if !defined(__EMSCRIPTEN__)
        , quadrotorParameters(std::move(selectedQuadrotorParameters))
#endif
    {
#if defined(__EMSCRIPTEN__)
        static_cast<void>(selectedQuadrotorParameters);
#endif
        RegisterDroneSystem();
        RegisterCameraSystem();
        SynchronizeCameraPose();
    }

    void RegisterDroneSystem()
    {
        if (dynamics == SimulationDynamics::Kinematic)
        {
            RegisterKinematicSystem();
            return;
        }
#if !defined(__EMSCRIPTEN__)
        RegisterRigidBodySystem();
#else
        primaryDroneId = CreateDrone(glm::dvec3{0.0}, 0.0);
#endif
    }

    void RegisterKinematicSystem()
    {
        primaryDroneId = CreateDrone(glm::dvec3{0.0}, 0.0);
        world.system<DroneState, DroneControlState>("AdvanceKinematicDrone")
            .kind(flecs::OnUpdate)
            .run([this](flecs::iter& iterator)
            {
                ScopedDuration measureSystem(
                    measureEcsSystemDuration
                        ? &lastEcsSystemDurations.advanceKinematicDroneMicroseconds
                        : nullptr);
                while (iterator.next())
                {
                    auto states = iterator.field<DroneState>(0);
                    auto controls = iterator.field<DroneControlState>(1);
                    for (const std::size_t row : iterator)
                    {
                        const glm::dvec3 previousPosition = states[row].positionMeters;
                        DroneModel::AdvanceOneTick(states[row], controls[row]);
                        ResolveKinematicContacts(
                            states[row], controls[row], previousPosition);
                    }
                }
            });
    }

    void ResolveKinematicContacts(
        DroneState& state, DroneControlState& control,
        const glm::dvec3& previousPosition) const
    {
        if (terrainGrid
            && terrainGrid->ContainsLocalPositionMeters(
                {state.positionMeters.x, state.positionMeters.y}))
        {
            control.groundElevationMeters = terrainGrid->HeightAtLocalMeters(
                {state.positionMeters.x, state.positionMeters.y});
            if (state.positionMeters.z < control.groundElevationMeters)
            {
                state.positionMeters.z = control.groundElevationMeters;
                state.velocityMetersPerSecond.z = 0.0;
            }
        }

        constexpr glm::dvec3 droneHalfExtentsMeters{0.30, 0.30, 0.12};
        for (const StaticObstacleBox& obstacle : kinematicObstacles)
        {
            const glm::dvec3 offset = state.positionMeters - obstacle.centerMeters;
            const glm::dvec3 overlap = obstacle.halfExtentsMeters
                + droneHalfExtentsMeters - glm::abs(offset);
            if (overlap.x <= 0.0 || overlap.y <= 0.0 || overlap.z <= 0.0)
                continue;

            glm::length_t axis = 0;
            if (overlap.y < overlap[axis])
                axis = 1;
            if (overlap.z < overlap[axis])
                axis = 2;

            const double previousSide = previousPosition[axis]
                - obstacle.centerMeters[axis];
            const double currentSide = offset[axis];
            const double side = previousSide != 0.0
                ? std::copysign(1.0, previousSide)
                : (currentSide != 0.0 ? std::copysign(1.0, currentSide) : 1.0);
            state.positionMeters[axis] = obstacle.centerMeters[axis]
                + side * (obstacle.halfExtentsMeters[axis]
                    + droneHalfExtentsMeters[axis]);

            if (state.velocityMetersPerSecond[axis] * side < 0.0)
                state.velocityMetersPerSecond[axis] = 0.0;
            if (control.targetVelocityMetersPerSecond[axis] * side < 0.0)
                control.targetVelocityMetersPerSecond[axis] = 0.0;
        }
    }

#if !defined(__EMSCRIPTEN__)
    void RegisterRigidBodySystem()
    {
        physics = std::make_unique<PhysicsWorld>();
        primaryDroneId = CreateDrone(glm::dvec3{
            0.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters}, 0.0);
        world.system<DroneState, DroneControlState, PhysicsBody>(
            "ApplyRigidBodyForces")
            .kind(flecs::OnUpdate)
            .run([this](flecs::iter& iterator)
            {
                ScopedDuration measureSystem(
                    measureEcsSystemDuration
                        ? &lastEcsSystemDurations.applyRigidBodyForcesMicroseconds
                        : nullptr);
                while (iterator.next())
                {
                    auto states = iterator.field<DroneState>(0);
                    auto controls = iterator.field<DroneControlState>(1);
                    auto bodies = iterator.field<const PhysicsBody>(2);
                    for (const std::size_t row : iterator)
                    {
                        ApplyRigidBodyForces(
                            states[row], controls[row], bodies[row].handle);
                    }
                }
            });
    }
#endif

    void RegisterCameraSystem()
    {
        world.system<CameraState>("AdvanceCamera")
            .kind(flecs::OnUpdate)
            .run([this](flecs::iter& iterator)
            {
                ScopedDuration measureSystem(
                    measureEcsSystemDuration
                        ? &lastEcsSystemDurations.advanceCameraMicroseconds : nullptr);
                while (iterator.next())
                {
                    auto cameraStates = iterator.field<CameraState>(0);
                    for (const std::size_t row : iterator)
                        EoCameraModel::AdvanceOneTick(cameraStates[row], tickCount + 1);
                }
            });
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
        std::optional<RigidBodyHandle> createdBody;
#endif

        try
        {
            drone.set<DroneState>(initialState)
                .set<DroneControlState>(initialControl)
                .set<InitialPose>(initialPose)
                .set<CameraState>({});
            EoCameraModel::UpdatePose(
                drone.get_mut<CameraState>(), drone.get<DroneState>());
#if !defined(__EMSCRIPTEN__)
            if (dynamics == SimulationDynamics::RigidBody)
            {
                RigidBodyDefinition definition;
                definition.positionMeters = positionMeters;
                definition.orientation = initialState.orientation;
                createdBody = physics->CreateQuadrotor(definition, quadrotorParameters);
                drone.set<PhysicsBody>({*createdBody});
                CopyPhysicsState(drone.get_mut<DroneState>(), *createdBody);
            }
#endif

            drones.emplace(id, drone);
        }
        catch (...)
        {
#if !defined(__EMSCRIPTEN__)
            if (createdBody)
                static_cast<void>(physics->DestroyBody(*createdBody));
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
        for (auto& [id, drone] : drones)
        {
            (void)id;
            EoCameraModel::UpdatePose(
                drone.get_mut<CameraState>(), drone.get<DroneState>());
        }
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
        batteries.erase(id);
        if (removedPrimary)
        {
            const auto nextPrimary = std::min_element(
                drones.begin(), drones.end(),
                [](const auto& first, const auto& second)
                {
                    return first.first < second.first;
                });
            primaryDroneId = nextPrimary->first;
            SynchronizeCameraPose();
        }
        return true;
    }

#if !defined(__EMSCRIPTEN__)
    void ResetPhysicsBody(flecs::entity& drone)
    {
        const DroneState& droneState = drone.get<DroneState>();
        RigidBodyState initialState;
        initialState.positionMeters = droneState.positionMeters;
        initialState.orientation = droneState.orientation;
        const RigidBodyHandle body = drone.get<PhysicsBody>().handle;
        physics->SetState(body, initialState);
        CopyPhysicsState(drone.get_mut<DroneState>(), body);
    }
#endif

    void ResetDrone(flecs::entity& drone)
    {
        const InitialPose& initialPose = drone.get<InitialPose>();
        ApplyInitialPose(
            drone.get_mut<DroneState>(),
            drone.get_mut<DroneControlState>(),
            initialPose);
#if !defined(__EMSCRIPTEN__)
        if (dynamics == SimulationDynamics::RigidBody)
            ResetPhysicsBody(drone);
#endif
    }

#if !defined(__EMSCRIPTEN__)
    void ApplyRigidBodyForces(
        DroneState& state,
        DroneControlState& control,
        RigidBodyHandle body)
    {
        const double step = DroneModel::fixedStepSeconds;
        if (state.mode != FlightMode::Grounded)
            control.desiredYawRadians += control.commandedYawRateRadiansPerSecond * step;

        const auto targetSpeeds = control.commandedRotorThrustRatios
            ? RotorSpeedTargets(*control.commandedRotorThrustRatios)
            : QuadrotorFlightController::MotorTargets(
                state, control, quadrotorParameters, controllerParameters);
        physics->SetMotorTargets(body, targetSpeeds);
    }

    [[nodiscard]] std::array<double, 4> RotorSpeedTargets(
        const std::array<double, 4>& thrustRatios) const noexcept
    {
        std::array<double, 4> targets{};
        for (std::size_t index = 0; index < targets.size(); ++index)
        {
            // AirSim은 속도 제곱에 비례해 추력을 계산하므로 비율의 제곱근을 속도로 바꾼다.
            targets[index] = quadrotorParameters.maximumMotorSpeedRadiansPerSecond
                * std::sqrt(thrustRatios[index]);
        }
        return targets;
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
        PhysicsStepMeasurements measurements;
        physics->Step(DroneModel::fixedStepSeconds,
            measurePhysicsStepDuration ? &measurements : nullptr);
        lastPhysicsStepDurationMicroseconds = measurements.stepDurationMicroseconds;
        lastContactGenerationDurationMicroseconds =
            measurements.contactGenerationMicroseconds;

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
    : SimulationCore(dynamics, QuadrotorParameters::X500MotorTableEstimate(), {})
{
}

SimulationCore::SimulationCore(
    SimulationDynamics dynamics, QuadrotorParameters quadrotorParameters)
    : SimulationCore(dynamics, std::move(quadrotorParameters), {})
{
}

SimulationCore::SimulationCore(SimulationDynamics dynamics,
    QuadrotorParameters quadrotorParameters,
    const QuadrotorControllerParameters& controllerParameters)
    : ecs_(std::make_unique<EcsStorage>(
        dynamics, std::move(quadrotorParameters), controllerParameters))
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
    if (command.kind == DroneCommand::Kind::SetRotorThrusts
        && ecs_->dynamics != SimulationDynamics::RigidBody)
    {
        return CommandResult::UnsupportedDynamics;
    }
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
    ecs_->ResetDrone(drone);
    ecs_->SynchronizeCameraPose();
}

void SimulationCore::SetInitialState(DroneId droneId, const DroneState& state)
{
    if (ecs_->tickCount != 0)
        throw std::logic_error("드론 시작 상태는 첫 Tick 전에만 바꿀 수 있습니다.");
    const double orientationLength = glm::length(state.orientation);
    if (!HasFinitePose(state.positionMeters, 0.0)
        || !std::isfinite(state.velocityMetersPerSecond.x)
        || !std::isfinite(state.velocityMetersPerSecond.y)
        || !std::isfinite(state.velocityMetersPerSecond.z)
        || !std::isfinite(state.angularVelocityRadiansPerSecond.x)
        || !std::isfinite(state.angularVelocityRadiansPerSecond.y)
        || !std::isfinite(state.angularVelocityRadiansPerSecond.z)
        || !std::isfinite(orientationLength) || orientationLength < 1.0e-12
        || !std::all_of(state.motorSpeedsRadiansPerSecond.begin(),
            state.motorSpeedsRadiansPerSecond.end(),
            [](double speed) { return std::isfinite(speed) && speed >= 0.0; }))
    {
        throw std::invalid_argument("드론 시작 상태의 위치·속도·자세를 확인해 주세요.");
    }
    const bool hasInitialMotorSpeed = std::any_of(
        state.motorSpeedsRadiansPerSecond.begin(),
        state.motorSpeedsRadiansPerSecond.end(),
        [](double speed) { return speed > 0.0; });
#if !defined(__EMSCRIPTEN__)
    if (hasInitialMotorSpeed && ecs_->dynamics != SimulationDynamics::RigidBody)
        throw std::invalid_argument("초기 로터 속도는 강체 물리 모드에서만 지정할 수 있습니다.");
    if (std::any_of(state.motorSpeedsRadiansPerSecond.begin(),
        state.motorSpeedsRadiansPerSecond.end(), [&](double speed)
        {
            return speed > ecs_->quadrotorParameters.maximumMotorSpeedRadiansPerSecond;
        }))
    {
        throw std::invalid_argument("초기 로터 속도가 기체의 최대 속도를 넘었습니다.");
    }
#else
    if (hasInitialMotorSpeed)
        throw std::invalid_argument("초기 로터 속도는 강체 물리 모드에서만 지정할 수 있습니다.");
#endif

    flecs::entity& drone = ecs_->FindDrone(droneId);
    DroneState normalized = state;
    normalized.orientation = glm::normalize(normalized.orientation);
    const glm::dquat& q = normalized.orientation;
    normalized.yawRadians = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    DroneControlState& control = drone.get_mut<DroneControlState>();
    control.desiredYawRadians = normalized.yawRadians;
    control.targetVelocityMetersPerSecond = normalized.velocityMetersPerSecond;
    control.groundElevationMeters = normalized.positionMeters.z
        - QuadrotorFlightController::groundedModelOriginHeightMeters;
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        const RigidBodyState rigidBodyState{
            normalized.positionMeters,
            normalized.orientation,
            normalized.velocityMetersPerSecond,
            normalized.angularVelocityRadiansPerSecond};
        ecs_->physics->SetState(
            drone.get<EcsStorage::PhysicsBody>().handle, rigidBodyState);
        ecs_->physics->SetInitialMotorSpeeds(
            drone.get<EcsStorage::PhysicsBody>().handle,
            normalized.motorSpeedsRadiansPerSecond);
        ecs_->CopyPhysicsState(normalized, drone.get<EcsStorage::PhysicsBody>().handle);
        normalized.mode = state.mode;
    }
#endif
    drone.set<DroneState>(normalized);
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

void SimulationCore::ConfigureBattery(
    DroneId droneId, BatteryModelParameters parameters, double initialPercent)
{
    static_cast<void>(ecs_->FindDrone(droneId));
    ecs_->batteries.insert_or_assign(
        droneId, BatteryModel{parameters, initialPercent});
}

bool SimulationCore::HasBattery(DroneId droneId) const noexcept
{
    return ecs_->batteries.contains(droneId);
}

BatteryModel& SimulationCore::Battery(DroneId droneId)
{
    const auto found = ecs_->batteries.find(droneId);
    if (found == ecs_->batteries.end())
        throw std::out_of_range("드론 배터리가 설정되지 않았습니다.");
    return found->second;
}

const BatteryModel& SimulationCore::Battery(DroneId droneId) const
{
    const auto found = ecs_->batteries.find(droneId);
    if (found == ecs_->batteries.end())
        throw std::out_of_range("드론 배터리가 설정되지 않았습니다.");
    return found->second;
}

void SimulationCore::AdvanceOneTick()
{
    if (paused_)
    {
        return;
    }
    ecs_->lastPhysicsStepDurationMicroseconds = 0.0;
    ecs_->lastContactGenerationDurationMicroseconds = 0.0;
    ecs_->lastEcsSystemDurations = {};
    ecs_->lastFlecsProgressDurationMicroseconds = 0.0;
    {
        ScopedDuration measureEcsSystems(ecs_->measureEcsSystemDuration
            ? &ecs_->lastFlecsProgressDurationMicroseconds : nullptr);
        ecs_->world.progress(DroneModel::fixedStepSeconds);
    }
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        ecs_->AdvancePhysicsWorld();
#endif
    for (auto& [droneId, battery] : ecs_->batteries)
        battery.Advance(State(droneId).mode);
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

void SimulationCore::Reset()
{
    for (auto& [id, drone] : ecs_->drones)
    {
        (void)id;
        ecs_->ResetDrone(drone);
    }
    for (auto& [id, drone] : ecs_->drones)
    {
        (void)id;
        EoCameraModel::Reset(drone.get_mut<CameraState>());
    }
    for (auto& [id, battery] : ecs_->batteries)
    {
        (void)id;
        battery.Reset();
    }
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
    return ecs_->drones.at(ecs_->primaryDroneId).get<CameraState>();
}

const CameraState& SimulationCore::Camera(DroneId droneId) const
{
    return ecs_->FindDrone(droneId).get<CameraState>();
}

void SimulationCore::SetCameraModel(DroneId droneId, const CameraModel& model)
{
    if (!model.IsValid())
        throw std::invalid_argument("카메라 해상도는 양수이고 시야각은 0도보다 크고 180도보다 작아야 합니다.");
    ecs_->FindDrone(droneId).get_mut<CameraState>().model = model;
}

void SimulationCore::SetCameraEnabled(bool enabled) noexcept
{
    EoCameraModel::SetEnabled(
        ecs_->drones.at(ecs_->primaryDroneId).get_mut<CameraState>(), enabled);
}

void SimulationCore::SetCameraEnabled(DroneId droneId, bool enabled)
{
    EoCameraModel::SetEnabled(ecs_->FindDrone(droneId).get_mut<CameraState>(), enabled);
}

void SimulationCore::SetCameraFault(bool faulted) noexcept
{
    SetCameraFault(ecs_->primaryDroneId, faulted);
}

void SimulationCore::SetCameraFault(DroneId droneId, bool faulted)
{
    CameraState& camera = ecs_->FindDrone(droneId).get_mut<CameraState>();
    if (faulted)
        EoCameraModel::InjectFault(camera);
    else
        EoCameraModel::Recover(camera);
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

void SimulationCore::SetEcsSystemTimingEnabled(bool enabled) noexcept
{
    ecs_->measureEcsSystemDuration = enabled;
    ecs_->lastEcsSystemDurations = {};
    ecs_->lastFlecsProgressDurationMicroseconds = 0.0;
}

EcsSystemDurations SimulationCore::LastEcsSystemDurations() const noexcept
{
    return ecs_->lastEcsSystemDurations;
}

double SimulationCore::LastFlecsProgressDurationMicroseconds() const noexcept
{
    return ecs_->lastFlecsProgressDurationMicroseconds;
}

double SimulationCore::LastPhysicsStepDurationMicroseconds() const noexcept
{
    return ecs_->lastPhysicsStepDurationMicroseconds;
}

double SimulationCore::LastContactGenerationDurationMicroseconds() const noexcept
{
    return ecs_->lastContactGenerationDurationMicroseconds;
}

void SimulationCore::AddStaticObstacleBox(
    const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters)
{
    if (!std::isfinite(centerMeters.x) || !std::isfinite(centerMeters.y)
        || !std::isfinite(centerMeters.z)
        || !std::isfinite(halfExtentsMeters.x)
        || !std::isfinite(halfExtentsMeters.y)
        || !std::isfinite(halfExtentsMeters.z)
        || halfExtentsMeters.x <= 0.0F || halfExtentsMeters.y <= 0.0F
        || halfExtentsMeters.z <= 0.0F)
    {
        throw std::invalid_argument("장애물 위치와 크기를 확인해 주세요.");
    }

#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->physics->AddStaticObstacleBox(centerMeters, halfExtentsMeters);
        return;
    }
#endif
    ecs_->kinematicObstacles.push_back(
        {centerMeters, glm::dvec3{halfExtentsMeters}});
}

void SimulationCore::SetTerrain(terrain::TerrainGrid terrain)
{
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        ecs_->physics->SetTerrain(terrain);
#endif

    ecs_->terrainGrid = std::move(terrain);
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
