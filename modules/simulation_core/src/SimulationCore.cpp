#include "simulation_core/SimulationCore.h"
#include "simulation_core/SimulationPhysics.h"
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <flecs.h>
#include <glm/common.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace aerosimlab::simulation
{
namespace
{
bool HasFiniteVector(const glm::dvec3& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}

bool HasFinitePose(const glm::dvec3& positionMeters, double yawRadians) noexcept
{
    return HasFiniteVector(positionMeters) && std::isfinite(yawRadians);
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

    struct PhysicsBody
    {
        RigidBodyHandle handle;
    };

    QuadrotorControllerParameters controllerParameters;
    QuadrotorParameters quadrotorParameters;
    /** @brief Flecs 콜백이 참조하므로 Flecs 월드보다 나중에 정리한다. */
    std::unique_ptr<SimulationPhysics> physics;
    flecs::world world;
    std::map<DroneId, flecs::entity> drones;
    DroneId primaryDroneId = 0;
    DroneId nextDroneId = 1;
    bool measurePhysicsStepDuration = false;
    double lastPhysicsStepDurationMicroseconds = 0.0;
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
        QuadrotorParameters selectedQuadrotorParameters,
        QuadrotorControllerParameters selectedControllerParameters = {})
        : controllerParameters(selectedControllerParameters)
        , quadrotorParameters(std::move(selectedQuadrotorParameters))
    {
        RegisterRigidBodySystem();
        RegisterCameraSystem();
        SynchronizeCameraPoses();
    }

    void RegisterRigidBodySystem()
    {
        physics = std::make_unique<SimulationPhysics>();
        primaryDroneId = CreateDrone(glm::dvec3{
            0.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters}, 0.0);
        world.system<DroneState, DroneControlState, PhysicsBody>(
            "ApplyRigidBodyForces")
            .kind(flecs::OnUpdate)
            .run([this](flecs::iter& iterator)
            {
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

    void RegisterCameraSystem()
    {
        world.system<CameraState>("AdvanceCamera")
            .kind(flecs::OnUpdate)
            .run([this](flecs::iter& iterator)
            {
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
        std::optional<RigidBodyHandle> createdBody;

        try
        {
            drone.set<DroneState>(initialState)
                .set<DroneControlState>(initialControl)
                .set<InitialPose>(initialPose)
                .set<CameraState>({});
            EoCameraModel::UpdatePose(
                drone.get_mut<CameraState>(), drone.get<DroneState>());
            RigidBodyDefinition definition;
            definition.positionMeters = positionMeters;
            definition.orientation = initialState.orientation;
            createdBody = physics->CreateQuadrotor(definition, quadrotorParameters);
            drone.set<PhysicsBody>({*createdBody});
            CopyPhysicsState(drone.get_mut<DroneState>(), *createdBody);

            drones.emplace(id, drone);
        }
        catch (...)
        {
            if (createdBody)
                static_cast<void>(physics->DestroyBody(*createdBody));
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

    void SynchronizeCameraPose(flecs::entity drone)
    {
        EoCameraModel::UpdatePose(
            drone.get_mut<CameraState>(), drone.get<DroneState>());
    }

    void SynchronizeCameraPoses()
    {
        world.each(
            [](CameraState& camera, const DroneState& state)
            {
                EoCameraModel::UpdatePose(camera, state);
            });
    }

    bool RemoveDrone(DroneId id)
    {
        if (drones.size() <= 1)
            return false;
        const auto found = drones.find(id);
        if (found == drones.end())
            return false;

        if (!physics->DestroyBody(found->second.get<PhysicsBody>().handle))
            return false;

        found->second.destruct();
        const bool removedPrimary = id == primaryDroneId;
        drones.erase(found);
        if (removedPrimary)
            primaryDroneId = drones.begin()->first;
        return true;
    }

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

    void ResetDrone(flecs::entity& drone)
    {
        const InitialPose& initialPose = drone.get<InitialPose>();
        ApplyInitialPose(
            drone.get_mut<DroneState>(),
            drone.get_mut<DroneControlState>(),
            initialPose);
        ResetPhysicsBody(drone);
    }

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
            // 추력은 모터 속도의 제곱에 비례하므로 비율의 제곱근을 목표 속도로 바꾼다.
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
        target.yawRadians = DroneModel::YawRadians(target);
    }

    void AdvancePhysicsWorld()
    {
        physics->StepFixed(
            measurePhysicsStepDuration ? &lastPhysicsStepDurationMicroseconds : nullptr);

        for (auto& [id, drone] : drones)
        {
            (void)id;
            SynchronizeRigidBody(
                drone.get_mut<DroneState>(),
                drone.get_mut<DroneControlState>(),
                drone.get<PhysicsBody>().handle);
        }
    }
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

SimulationCore::SimulationCore()
    : SimulationCore(QuadrotorParameters::X500MotorTableEstimate(), {})
{
}

SimulationCore::SimulationCore(
    QuadrotorParameters quadrotorParameters)
    : SimulationCore(std::move(quadrotorParameters), {})
{
}

SimulationCore::SimulationCore(
    QuadrotorParameters quadrotorParameters,
    const QuadrotorControllerParameters& controllerParameters)
    : ecs_(std::make_unique<EcsStorage>(
        std::move(quadrotorParameters), controllerParameters))
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
    const CommandResult result = DroneModel::Apply(state, control, command);
    if (result == CommandResult::Accepted)
    {
        ecs_->CopyPhysicsState(state, drone.get<EcsStorage::PhysicsBody>().handle);
    }
    return result;
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
    const double groundElevationMeters = positionMeters.z
        - QuadrotorFlightController::groundedModelOriginHeightMeters;
    const EcsStorage::InitialPose initialPose{
        positionMeters, yawRadians, groundElevationMeters};
    drone.set<EcsStorage::InitialPose>(initialPose);
    ecs_->ResetDrone(drone);
    ecs_->SynchronizeCameraPose(drone);
}

void SimulationCore::SetInitialState(DroneId droneId, const DroneState& state)
{
    if (ecs_->tickCount != 0)
        throw std::logic_error("드론 시작 상태는 첫 Tick 전에만 바꿀 수 있습니다.");
    const double orientationLength = glm::length(state.orientation);
    if (!HasFiniteVector(state.positionMeters)
        || !HasFiniteVector(state.velocityMetersPerSecond)
        || !HasFiniteVector(state.angularVelocityRadiansPerSecond)
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
    if (std::any_of(state.motorSpeedsRadiansPerSecond.begin(),
        state.motorSpeedsRadiansPerSecond.end(), [&](double speed)
        {
            return speed > ecs_->quadrotorParameters.maximumMotorSpeedRadiansPerSecond;
        }))
    {
        throw std::invalid_argument("초기 로터 속도가 기체의 최대 속도를 넘었습니다.");
    }

    flecs::entity& drone = ecs_->FindDrone(droneId);
    DroneState normalized = state;
    normalized.orientation = glm::normalize(normalized.orientation);
    normalized.yawRadians = DroneModel::YawRadians(normalized);
    DroneControlState& control = drone.get_mut<DroneControlState>();
    control.desiredYawRadians = normalized.yawRadians;
    control.targetVelocityMetersPerSecond = normalized.velocityMetersPerSecond;
    control.groundElevationMeters = normalized.positionMeters.z
        - QuadrotorFlightController::groundedModelOriginHeightMeters;
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
    drone.set<DroneState>(normalized);
    ecs_->SynchronizeCameraPose(drone);
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
    for (const auto& entry : ecs_->drones)
        ids.push_back(entry.first);
    return ids;
}

void SimulationCore::ConfigureBattery(
    DroneId droneId, BatteryModelParameters parameters, double initialPercent)
{
    ecs_->FindDrone(droneId).set<BatteryModel>(
        BatteryModel{parameters, initialPercent});
}

bool SimulationCore::HasBattery(DroneId droneId) const noexcept
{
    const auto found = ecs_->drones.find(droneId);
    return found != ecs_->drones.end() && found->second.has<BatteryModel>();
}

BatteryModel& SimulationCore::Battery(DroneId droneId)
{
    flecs::entity& drone = ecs_->FindDrone(droneId);
    if (!drone.has<BatteryModel>())
        throw std::out_of_range("드론 배터리가 설정되지 않았습니다.");
    return drone.get_mut<BatteryModel>();
}

const BatteryModel& SimulationCore::Battery(DroneId droneId) const
{
    const flecs::entity& drone = ecs_->FindDrone(droneId);
    if (!drone.has<BatteryModel>())
        throw std::out_of_range("드론 배터리가 설정되지 않았습니다.");
    return drone.get<BatteryModel>();
}

void SimulationCore::AdvanceOneTick()
{
    if (paused_)
    {
        return;
    }
    ecs_->lastPhysicsStepDurationMicroseconds = 0.0;
    ecs_->world.progress(DroneModel::fixedStepSeconds);
    ecs_->AdvancePhysicsWorld();
    // 배터리 컴포넌트가 붙은 드론만 골라, 물리 계산 뒤의 비행 상태로 잔량을 갱신한다.
    ecs_->world.each(
        [](BatteryModel& battery, const DroneState& state)
        {
            battery.Advance(state.mode);
        });
    ++ecs_->tickCount;
    ecs_->SynchronizeCameraPoses();
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
    ecs_->world.each(
        [this](flecs::entity drone, DroneState& state,
            DroneControlState&, CameraState& camera)
        {
            ecs_->ResetDrone(drone);
            EoCameraModel::Reset(camera);
            EoCameraModel::UpdatePose(camera, state);
            if (drone.has<BatteryModel>())
                drone.get_mut<BatteryModel>().Reset();
        });
    ecs_->tickCount = 0;
    paused_ = false;
}

const DroneState& SimulationCore::State() const noexcept
{
    return State(ecs_->primaryDroneId);
}

const DroneState& SimulationCore::State(DroneId droneId) const
{
    return ecs_->FindDrone(droneId).get<DroneState>();
}

const CameraState& SimulationCore::Camera() const noexcept
{
    return Camera(ecs_->primaryDroneId);
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
    SetCameraEnabled(ecs_->primaryDroneId, enabled);
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
    if (!HasFiniteVector(velocity))
        throw std::invalid_argument("바람 속도는 유한한 값이어야 합니다.");
    ecs_->physics->SetWindVelocityMetersPerSecond(velocity);
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

void SimulationCore::AddStaticObstacleBox(
    const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters)
{
    ecs_->physics->AddStaticObstacleBox(centerMeters, halfExtentsMeters);
}

void SimulationCore::AddStaticObstaclePrism(
    const std::vector<glm::dvec2>& outlineMeters,
    double bottomMeters, double topMeters)
{
    ecs_->physics->AddStaticObstaclePrism(
        outlineMeters, bottomMeters, topMeters);
}

void SimulationCore::SetTerrain(terrain::TerrainGrid terrain)
{
    ecs_->physics->SetTerrain(std::move(terrain));
}

std::uint64_t SimulationCore::TickCount() const noexcept
{
    return ecs_->tickCount;
}

double SimulationCore::SimulationTimeSeconds() const noexcept
{
    return static_cast<double>(ecs_->tickCount) * DroneModel::fixedStepSeconds;
}

}
