#include "simulation_core/SimulationCore.h"
#include "geometry/VectorMath.h"
#include "simulation_core/JoltPhysicsWorld.h"
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <flecs.h>
#include <glm/common.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dronesim::simulation
{
namespace
{
/**
 * @brief 공개 드론 식별값과 Flecs 엔티티를 연결하는 색인이다.
 * @details 컴포넌트 상태는 Flecs가 소유하고, 이 객체는 외부에서 쓰는 안정된 식별값으로 엔티티를 찾는 일만 맡는다.
 */
class DroneRegistry
{
public:
    /** @brief 새 드론 생성이 실패해도 식별값을 재사용하지 않도록 번호를 먼저 예약한다. */
    [[nodiscard]] DroneId ReserveId() noexcept
    {
        return nextId_++;
    }

    /** @brief 완성된 엔티티를 등록하고, 첫 드론을 기본 대상으로 지정한다. */
    void Register(DroneId id, flecs::entity drone)
    {
        const auto [entry, inserted] = drones_.emplace(id, drone);
        static_cast<void>(entry);
        if (!inserted)
            throw std::logic_error("이미 등록된 드론 식별값입니다.");
        if (primaryId_ == 0)
            primaryId_ = id;
    }

    /** @brief 반드시 존재해야 하는 드론을 찾는다. */
    [[nodiscard]] flecs::entity& Find(DroneId id)
    {
        flecs::entity* drone = TryFind(id);
        if (drone == nullptr)
            ThrowMissingDrone(id);
        return *drone;
    }

    /** @brief 반드시 존재해야 하는 드론을 읽기 전용으로 찾는다. */
    [[nodiscard]] const flecs::entity& Find(DroneId id) const
    {
        const flecs::entity* drone = TryFind(id);
        if (drone == nullptr)
            ThrowMissingDrone(id);
        return *drone;
    }

    /** @brief 드론이 없어도 예외 없이 찾을 수 있도록 포인터를 반환한다. */
    [[nodiscard]] flecs::entity* TryFind(DroneId id) noexcept
    {
        const auto found = drones_.find(id);
        return found == drones_.end() ? nullptr : &found->second;
    }

    /** @brief 읽기 전용 조회에서도 같은 방식으로 엔티티를 찾는다. */
    [[nodiscard]] const flecs::entity* TryFind(DroneId id) const noexcept
    {
        const auto found = drones_.find(id);
        return found == drones_.end() ? nullptr : &found->second;
    }

    /**
     * @brief 물리 객체 제거가 끝난 드론을 등록부에서도 지운다.
     * @details 호출자는 드론이 등록되어 있고 마지막 한 대가 아님을 먼저 확인한다.
     */
    void RemoveAfterPhysicsBodyDestroyed(DroneId id) noexcept
    {
        const bool removedPrimary = id == primaryId_;
        const std::size_t removedCount = drones_.erase(id);
        assert(removedCount == 1);
        static_cast<void>(removedCount);
        if (removedPrimary)
            primaryId_ = drones_.begin()->first;
    }

    [[nodiscard]] DroneId PrimaryId() const noexcept
    {
        return primaryId_;
    }

    [[nodiscard]] std::size_t Size() const noexcept
    {
        return drones_.size();
    }

    [[nodiscard]] std::vector<DroneId> Ids() const
    {
        std::vector<DroneId> ids;
        ids.reserve(drones_.size());
        for (const auto& [id, drone] : drones_)
        {
            static_cast<void>(drone);
            ids.push_back(id);
        }
        return ids;
    }

private:
    [[noreturn]] static void ThrowMissingDrone(DroneId id)
    {
        throw std::out_of_range(
            "드론 식별값 " + std::to_string(id) + "이 없습니다.");
    }

    std::map<DroneId, flecs::entity> drones_;
    DroneId primaryId_ = 0;
    DroneId nextId_ = 1;
};

bool HasFinitePose(const glm::dvec3& positionMeters, double yawRadians) noexcept
{
    return geometry::IsFinite(positionMeters) && std::isfinite(yawRadians);
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
        JoltRigidBodyHandle handle;
    };

    QuadrotorControllerParameters controllerParameters;
    QuadrotorParameters quadrotorParameters;
    /** @brief 월드의 콜백이 물리를 사용하므로 Flecs 월드보다 나중에 정리한다. */
    JoltPhysicsWorld physics;
    std::optional<JoltRigidBodyHandle> defaultGround;
    flecs::world world;
    DroneRegistry droneRegistry;
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
        // 지형을 불러오기 전에도 드론이 떨어지지 않도록 임시 평면을 둔다.
        // 바닥 윗면은 z=0이며, 실제 지형 등록이 끝나면 이 평면을 제거한다.
        defaultGround = physics.AddStaticBox(
            {0.0, 0.0, -0.5}, {10000.0, 10000.0, 0.5});
        static_cast<void>(CreateDrone(glm::dvec3{
            0.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters}, 0.0));
        RegisterMotorTargetSystem();
        RegisterCameraSystem();
    }

    /** @brief Flecs가 각 기체의 제어 상태를 읽어 물리 모터 목표를 갱신하게 등록한다. */
    void RegisterMotorTargetSystem()
    {
        world.system<DroneState, DroneControlState, PhysicsBody>(
            "UpdateMotorTargets")
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
                        UpdateMotorTargets(
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
        const DroneId id = droneRegistry.ReserveId();
        const InitialPose initialPose{positionMeters, yawRadians, groundElevationMeters};
        DroneState initialState;
        DroneControlState initialControl;
        ApplyInitialPose(initialState, initialControl, initialPose);
        flecs::entity drone = world.entity();
        std::optional<JoltRigidBodyHandle> createdBody;

        try
        {
            drone.set<DroneState>(initialState)
                .set<DroneControlState>(initialControl)
                .set<InitialPose>(initialPose)
                .set<CameraState>({});
            JoltRigidBodyDefinition definition;
            definition.positionMeters = positionMeters;
            definition.orientation = initialState.orientation;
            createdBody = physics.CreateQuadrotor(definition, quadrotorParameters);
            drone.set<PhysicsBody>({*createdBody});
            CopyPhysicsState(drone.get_mut<DroneState>(), *createdBody);
            // 카메라는 물리 엔진이 확정한 기체 위치와 방향을 기준으로 붙인다.
            SynchronizeCameraPose(drone);

            droneRegistry.Register(id, drone);
        }
        catch (...)
        {
            if (createdBody)
                static_cast<void>(physics.DestroyBody(*createdBody));
            // ECS에만 남은 엔티티가 다음 시스템 실행에 섞이지 않도록 함께 지운다.
            drone.destruct();
            throw;
        }
        return id;
    }

    CommandResult Apply(DroneId id, const DroneCommand& command) noexcept
    {
        flecs::entity* drone = droneRegistry.TryFind(id);
        if (drone == nullptr)
            return CommandResult::DroneNotFound;

        DroneState& state = drone->get_mut<DroneState>();
        DroneControlState& control = drone->get_mut<DroneControlState>();
        const CommandResult result = DroneModel::Apply(state, control, command);
        if (result == CommandResult::Accepted)
        {
            // 명령으로 초기 속도가 바뀌면 ECS 상태를 강체에도 바로 반영한다.
            CopyPhysicsState(state, drone->get<PhysicsBody>().handle);
        }
        return result;
    }

    void SetInitialPose(const glm::dvec3& positionMeters, double yawRadians)
    {
        if (tickCount != 0)
            throw std::logic_error("드론 시작 자세는 첫 Tick 전에만 바꿀 수 있습니다.");
        if (!HasFinitePose(positionMeters, yawRadians))
            throw std::invalid_argument("드론 시작 위치와 방향은 유한한 값이어야 합니다.");

        flecs::entity& drone = droneRegistry.Find(droneRegistry.PrimaryId());
        const double groundElevationMeters = positionMeters.z
            - QuadrotorFlightController::groundedModelOriginHeightMeters;
        const InitialPose initialPose{
            positionMeters, yawRadians, groundElevationMeters};
        drone.set<InitialPose>(initialPose);
        ResetDrone(drone);
        SynchronizeCameraPose(drone);
    }

    void SetInitialState(DroneId id, const DroneState& state)
    {
        if (tickCount != 0)
            throw std::logic_error("드론 시작 상태는 첫 Tick 전에만 바꿀 수 있습니다.");

        flecs::entity& drone = droneRegistry.Find(id);
        DroneState normalized = DroneModel::PrepareInitialState(
            state, quadrotorParameters.maximumMotorSpeedRadiansPerSecond);
        DroneControlState& control = drone.get_mut<DroneControlState>();
        control.desiredYawRadians = normalized.yawRadians;
        control.targetVelocityMetersPerSecond = normalized.velocityMetersPerSecond;
        control.groundElevationMeters = normalized.positionMeters.z
            - QuadrotorFlightController::groundedModelOriginHeightMeters;

        const JoltRigidBodyHandle body = drone.get<PhysicsBody>().handle;
        const JoltRigidBodyState rigidBodyState{
            normalized.positionMeters,
            normalized.orientation,
            normalized.velocityMetersPerSecond,
            normalized.angularVelocityRadiansPerSecond};
        physics.SetState(body, rigidBodyState);
        physics.SetInitialMotorSpeeds(body,
            normalized.motorSpeedsRadiansPerSecond);
        CopyPhysicsState(normalized, body);
        normalized.mode = state.mode;
        drone.set<DroneState>(normalized);
        SynchronizeCameraPose(drone);
    }

    void SynchronizeCameraPose(
        CameraState& camera, const DroneState& droneState) const
    {
        EoCameraModel::UpdatePose(camera, droneState);
    }

    void SynchronizeCameraPose(flecs::entity drone) const
    {
        SynchronizeCameraPose(
            drone.get_mut<CameraState>(), drone.get<DroneState>());
    }

    void SynchronizeCameraPoses()
    {
        world.each([this](CameraState& camera, const DroneState& state)
        {
            SynchronizeCameraPose(camera, state);
        });
    }

    bool RemoveDrone(DroneId id)
    {
        if (droneRegistry.Size() <= 1)
            return false;
        flecs::entity* drone = droneRegistry.TryFind(id);
        if (drone == nullptr)
            return false;

        if (!physics.DestroyBody(drone->get<PhysicsBody>().handle))
            return false;

        drone->destruct();
        droneRegistry.RemoveAfterPhysicsBodyDestroyed(id);
        return true;
    }

    void ResetPhysicsBody(flecs::entity& drone)
    {
        const DroneState& droneState = drone.get<DroneState>();
        JoltRigidBodyState initialState;
        initialState.positionMeters = droneState.positionMeters;
        initialState.orientation = droneState.orientation;
        const JoltRigidBodyHandle body = drone.get<PhysicsBody>().handle;
        physics.SetState(body, initialState);
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

    void UpdateMotorTargets(
        DroneState& state,
        DroneControlState& control,
        JoltRigidBodyHandle body)
    {
        const double step = DroneModel::fixedStepSeconds;
        if (state.mode != FlightMode::Grounded)
            control.desiredYawRadians += control.commandedYawRateRadiansPerSecond * step;

        const auto targetSpeeds = QuadrotorFlightController::MotorTargets(
            state, control, quadrotorParameters, controllerParameters);
        // Jolt는 이 목표 회전수를 모터 응답과 추력으로 바꿔 물체에 힘을 적용한다.
        physics.SetMotorTargets(body, targetSpeeds);
    }

    void SynchronizeRigidBody(
        DroneState& state,
        DroneControlState& control,
        JoltRigidBodyHandle body)
    {
        CopyPhysicsState(state, body);
        DroneModel::UpdateFlightModeAfterPhysics(state, control,
            QuadrotorFlightController::groundedModelOriginHeightMeters);
    }

    void CopyPhysicsState(DroneState& target, JoltRigidBodyHandle body) const
    {
        // 비행 위치와 자세의 기준은 물리 엔진이다. 모터 시험에서 기록한 실제 회전수도 함께 복사한다.
        const JoltRigidBodyState source = physics.State(body);
        target.positionMeters = source.positionMeters;
        target.velocityMetersPerSecond = source.linearVelocityMetersPerSecond;
        target.orientation = source.orientation;
        target.angularVelocityRadiansPerSecond = source.angularVelocityRadiansPerSecond;
        target.motorSpeedsRadiansPerSecond = physics.MotorSpeedsRadiansPerSecond(body);
        target.yawRadians = DroneModel::YawRadians(target);
    }

    void AdvancePhysicsWorld()
    {
        // 물리 엔진이 한 고정 간격만큼 이동시킨 뒤, ECS 상태를 새 위치에 맞춘다.
        physics.StepFixed(
            measurePhysicsStepDuration ? &lastPhysicsStepDurationMicroseconds : nullptr);

        // 물리 객체가 연결된 상태·제어 컴포넌트만 Flecs 쿼리로 다시 동기화한다.
        world.each([this](DroneState& state,
            DroneControlState& control, const PhysicsBody& body)
        {
            SynchronizeRigidBody(
                state, control, body.handle);
        });
    }

    void SetTerrain(const terrain::TerrainGrid& terrain)
    {
        // 지형 생성이 실패하면 기존 임시 바닥을 유지해 드론이 추락하지 않게 한다.
        physics.SetTerrain(terrain);
        if (defaultGround)
        {
            static_cast<void>(physics.DestroyBody(*defaultGround));
            defaultGround.reset();
        }
    }
};

std::size_t FixedTickAccumulator::Consume(double elapsedSeconds) noexcept
{
    if (std::isnan(elapsedSeconds) || elapsedSeconds <= 0.0)
        return 0;

    // 화면이 오래 멈춘 뒤에도 최대 100ms만 따라잡아, 밀린 Tick이 계속 쌓이지 않게 한다.
    accumulatedSeconds_ += std::min(elapsedSeconds, maximumFrameSeconds);

    // 남은 시간은 버리지 않고 보관해, 다음 화면 프레임과 함께 고정 간격에 맞춘다.
    std::size_t ticksToRun = 0;
    while (accumulatedSeconds_ >= fixedStepSeconds)
    {
        accumulatedSeconds_ -= fixedStepSeconds;
        ++ticksToRun;
    }
    return ticksToRun;
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
    return Apply(ecs_->droneRegistry.PrimaryId(), command);
}

CommandResult SimulationCore::Apply(DroneId droneId, const DroneCommand& command) noexcept
{
    if (paused_)
    {
        return CommandResult::SimulationPaused;
    }
    return ecs_->Apply(droneId, command);
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
    ecs_->SetInitialPose(positionMeters, yawRadians);
}

void SimulationCore::SetInitialState(DroneId droneId, const DroneState& state)
{
    ecs_->SetInitialState(droneId, state);
}

bool SimulationCore::RemoveDrone(DroneId droneId)
{
    return ecs_->RemoveDrone(droneId);
}

DroneId SimulationCore::PrimaryDroneId() const noexcept
{
    return ecs_->droneRegistry.PrimaryId();
}

std::size_t SimulationCore::DroneCount() const noexcept
{
    return ecs_->droneRegistry.Size();
}

std::vector<DroneId> SimulationCore::DroneIds() const
{
    return ecs_->droneRegistry.Ids();
}

void SimulationCore::ConfigureBattery(
    DroneId droneId, BatteryModelParameters parameters, double initialPercent)
{
    ecs_->droneRegistry.Find(droneId).set<BatteryModel>(
        BatteryModel{parameters, initialPercent});
}

bool SimulationCore::HasBattery(DroneId droneId) const noexcept
{
    const flecs::entity* drone =
        std::as_const(ecs_->droneRegistry).TryFind(droneId);
    return drone != nullptr && drone->has<BatteryModel>();
}

const BatteryModel& SimulationCore::Battery(DroneId droneId) const
{
    const flecs::entity& drone = ecs_->droneRegistry.Find(droneId);
    if (!drone.has<BatteryModel>())
        throw std::out_of_range("드론 배터리가 설정되지 않았습니다.");
    return drone.get<BatteryModel>();
}

bool SimulationCore::TryBeginLowBatteryReturn(DroneId droneId)
{
    flecs::entity& drone = ecs_->droneRegistry.Find(droneId);
    if (!drone.has<BatteryModel>())
        throw std::out_of_range("드론 배터리가 설정되지 않았습니다.");
    return drone.get_mut<BatteryModel>().TryBeginLowBatteryReturn();
}

void SimulationCore::AdvanceOneTick()
{
    if (paused_)
    {
        return;
    }
    ecs_->lastPhysicsStepDurationMicroseconds = 0.0;
    // Flecs에서 제어 목표를 모터 명령으로 바꾸고, 이어서 Jolt가 기체를 움직인다.
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
            ecs_->SynchronizeCameraPose(camera, state);
            if (drone.has<BatteryModel>())
                drone.get_mut<BatteryModel>().Reset();
        });
    ecs_->tickCount = 0;
    paused_ = false;
}

const DroneState& SimulationCore::State() const noexcept
{
    return State(ecs_->droneRegistry.PrimaryId());
}

const DroneState& SimulationCore::State(DroneId droneId) const
{
    return ecs_->droneRegistry.Find(droneId).get<DroneState>();
}

const CameraState& SimulationCore::Camera() const noexcept
{
    return Camera(ecs_->droneRegistry.PrimaryId());
}

const CameraState& SimulationCore::Camera(DroneId droneId) const
{
    return ecs_->droneRegistry.Find(droneId).get<CameraState>();
}

void SimulationCore::SetCameraModel(DroneId droneId, const CameraModel& model)
{
    if (!model.IsValid())
        throw std::invalid_argument("카메라 해상도는 양수이고 시야각은 0도보다 크고 180도보다 작아야 합니다.");
    ecs_->droneRegistry.Find(droneId).get_mut<CameraState>().model = model;
}

void SimulationCore::SetCameraGimbalAngles(
    DroneId droneId, const CameraGimbalAngles& angles)
{
    if (!angles.IsValid())
        throw std::invalid_argument("카메라 짐벌 각도는 유한한 값이어야 합니다.");

    flecs::entity& drone = ecs_->droneRegistry.Find(droneId);
    CameraState& camera = drone.get_mut<CameraState>();
    camera.gimbalAngles = angles;
    EoCameraModel::UpdatePose(camera, drone.get<DroneState>());
}

void SimulationCore::SetCameraEnabled(DroneId droneId, bool enabled)
{
    EoCameraModel::SetEnabled(
        ecs_->droneRegistry.Find(droneId).get_mut<CameraState>(), enabled);
}

void SimulationCore::SetCameraFault(DroneId droneId, bool faulted)
{
    CameraState& camera = ecs_->droneRegistry.Find(droneId).get_mut<CameraState>();
    if (faulted)
        EoCameraModel::InjectFault(camera);
    else
        EoCameraModel::Recover(camera);
}

void SimulationCore::SetWindVelocityMetersPerSecond(const glm::dvec3& velocity)
{
    if (!geometry::IsFinite(velocity))
        throw std::invalid_argument("바람 속도는 유한한 값이어야 합니다.");
    ecs_->physics.SetWindVelocityMetersPerSecond(velocity);
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
    static_cast<void>(ecs_->physics.AddStaticBox(
        centerMeters, glm::dvec3{halfExtentsMeters}));
}

void SimulationCore::AddStaticObstacleMesh(
    std::span<const glm::dvec3> verticesMeters,
    std::span<const std::uint32_t> triangleIndices)
{
    ecs_->physics.AddStaticTriangleMesh(verticesMeters, triangleIndices);
}

void SimulationCore::AddStaticObstaclePrism(
    const std::vector<glm::dvec2>& outlineMeters,
    double bottomMeters, double topMeters)
{
    ecs_->physics.AddStaticPrism(
        outlineMeters, bottomMeters, topMeters);
}

void SimulationCore::SetTerrain(terrain::TerrainGrid terrain)
{
    ecs_->SetTerrain(terrain);
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
