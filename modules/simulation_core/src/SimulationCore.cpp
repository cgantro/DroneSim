#include "simulation_core/SimulationCore.h"
#if !defined(__EMSCRIPTEN__)
#include "simulation_core/PhysicsWorld.h"
#endif
#include "CollisionGeometry.h"
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"

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

    struct StaticObstaclePrism
    {
        std::vector<glm::dvec2> outlineMeters;
        double bottomMeters = 0.0;
        double topMeters = 0.0;
    };

    struct StaticObstacleFootprint
    {
        std::vector<glm::dvec2> outlineMeters;
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
    DroneId primaryDroneId = 0;
    DroneId nextDroneId = 1;
    std::vector<StaticObstacleBox> kinematicObstacles;
    std::vector<StaticObstaclePrism> kinematicPrisms;
    std::vector<StaticObstacleFootprint> kinematicFootprints;
    std::optional<terrain::TerrainGrid> terrainGrid;
    glm::dvec3 windVelocityMetersPerSecond{0.0};
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
                while (iterator.next())
                {
                    auto states = iterator.field<DroneState>(0);
                    auto controls = iterator.field<DroneControlState>(1);
                    for (const std::size_t row : iterator)
                    {
                        const glm::dvec3 previousPosition = states[row].positionMeters;
                        DroneModel::AdvanceOneTick(
                            states[row], controls[row], windVelocityMetersPerSecond);
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
        constexpr double droneHalfWidthMeters = 0.30;
        constexpr glm::dvec3 droneHalfExtentsMeters{
            droneHalfWidthMeters, droneHalfWidthMeters, 0.12};
        if (terrainGrid)
        {
            const double halfWidth = terrainGrid->HalfWidthMeters();
            const double halfDepth = terrainGrid->HalfDepthMeters();
            const double maximumCenterX = std::max(
                0.0, halfWidth - droneHalfWidthMeters);
            const double maximumCenterY = std::max(
                0.0, halfDepth - droneHalfWidthMeters);
            const auto keepInsideTerrain = [](
                double& position, double maximum, double& velocity, double& targetVelocity)
            {
                if (position > maximum)
                {
                    position = maximum;
                    if (velocity > 0.0)
                        velocity = 0.0;
                    if (targetVelocity > 0.0)
                        targetVelocity = 0.0;
                }
                else if (position < -maximum)
                {
                    position = -maximum;
                    if (velocity < 0.0)
                        velocity = 0.0;
                    if (targetVelocity < 0.0)
                        targetVelocity = 0.0;
                }
            };
            keepInsideTerrain(state.positionMeters.x, maximumCenterX,
                state.velocityMetersPerSecond.x,
                control.targetVelocityMetersPerSecond.x);
            keepInsideTerrain(state.positionMeters.y, maximumCenterY,
                state.velocityMetersPerSecond.y,
                control.targetVelocityMetersPerSecond.y);
        }

        if (terrainGrid
            && terrainGrid->ContainsLocalPositionMeters(
                {state.positionMeters.x, state.positionMeters.y}))
        {
            const glm::dvec2 dronePosition{state.positionMeters.x, state.positionMeters.y};
            control.groundElevationMeters = terrainGrid->HeightAtLocalMeters(dronePosition);

            double highestGroundMeters = control.groundElevationMeters;
            // 기체 네 모서리의 지면도 확인해 경사면이 동체를 뚫고 지나가지 않게 한다.
            constexpr glm::dvec2 footprintOffsets[] = {
                {-droneHalfWidthMeters, -droneHalfWidthMeters},
                {-droneHalfWidthMeters, droneHalfWidthMeters},
                {droneHalfWidthMeters, -droneHalfWidthMeters},
                {droneHalfWidthMeters, droneHalfWidthMeters}};
            for (const glm::dvec2& offset : footprintOffsets)
            {
                const glm::dvec2 sample = dronePosition + offset;
                if (terrainGrid->ContainsLocalPositionMeters(sample))
                    highestGroundMeters = std::max(
                        highestGroundMeters,
                        static_cast<double>(terrainGrid->HeightAtLocalMeters(sample)));
            }

            const double minimumCenterHeightMeters =
                highestGroundMeters + droneHalfExtentsMeters.z;
            if (state.positionMeters.z < minimumCenterHeightMeters)
            {
                state.positionMeters.z = minimumCenterHeightMeters;
                if (state.velocityMetersPerSecond.z < 0.0)
                    state.velocityMetersPerSecond.z = 0.0;
                if (control.targetVelocityMetersPerSecond.z < 0.0)
                    control.targetVelocityMetersPerSecond.z = 0.0;
                if (state.mode == FlightMode::Landing)
                {
                    // 충돌 형상이 지면에서 기체 중심을 띄우므로, 이 접촉을 착륙 완료로 본다.
                    state.velocityMetersPerSecond = glm::dvec3{0.0};
                    control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
                    control.commandedYawRateRadiansPerSecond = 0.0;
                    state.mode = FlightMode::Grounded;
                }
            }
        }

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

        for (const StaticObstaclePrism& obstacle : kinematicPrisms)
        {
            constexpr double droneRadiusMeters = 0.4242640687119285;
            const auto contact = detail::FindPrismContact(
                obstacle.outlineMeters, obstacle.bottomMeters, obstacle.topMeters,
                state.positionMeters, droneRadiusMeters,
                droneHalfExtentsMeters.z);
            if (!contact)
                continue;

            if (std::abs(contact->normal.z) > 0.5)
            {
                state.positionMeters.z += contact->normal.z
                    * contact->penetrationMeters;
                if (state.velocityMetersPerSecond.z * contact->normal.z < 0.0)
                    state.velocityMetersPerSecond.z = 0.0;
                if (control.targetVelocityMetersPerSecond.z * contact->normal.z < 0.0)
                    control.targetVelocityMetersPerSecond.z = 0.0;
            }
            else
            {
                const glm::dvec2 normal{contact->normal.x, contact->normal.y};
                const glm::dvec2 correction = normal * contact->penetrationMeters;
                state.positionMeters.x += correction.x;
                state.positionMeters.y += correction.y;
                const glm::dvec2 velocity{state.velocityMetersPerSecond.x,
                    state.velocityMetersPerSecond.y};
                const double inwardSpeed = glm::dot(velocity, normal);
                if (inwardSpeed < 0.0)
                {
                    state.velocityMetersPerSecond.x -= inwardSpeed * normal.x;
                    state.velocityMetersPerSecond.y -= inwardSpeed * normal.y;
                }
                const glm::dvec2 target{control.targetVelocityMetersPerSecond.x,
                    control.targetVelocityMetersPerSecond.y};
                const double inwardTargetSpeed = glm::dot(target, normal);
                if (inwardTargetSpeed < 0.0)
                {
                    control.targetVelocityMetersPerSecond.x -= inwardTargetSpeed * normal.x;
                    control.targetVelocityMetersPerSecond.y -= inwardTargetSpeed * normal.y;
                }
            }
        }

        for (const StaticObstacleFootprint& obstacle : kinematicFootprints)
        {
            constexpr double droneRadiusMeters = 0.4242640687119285;
            const auto contact = detail::FindFootprintContact(
                obstacle.outlineMeters,
                {state.positionMeters.x, state.positionMeters.y}, droneRadiusMeters);
            if (!contact)
                continue;

            state.positionMeters.x += contact->normal.x * contact->penetrationMeters;
            state.positionMeters.y += contact->normal.y * contact->penetrationMeters;
            const glm::dvec2 normal = contact->normal;
            const glm::dvec2 velocity{
                state.velocityMetersPerSecond.x, state.velocityMetersPerSecond.y};
            const double inwardSpeed = glm::dot(velocity, normal);
            if (inwardSpeed < 0.0)
            {
                state.velocityMetersPerSecond.x -= inwardSpeed * normal.x;
                state.velocityMetersPerSecond.y -= inwardSpeed * normal.y;
            }
            const glm::dvec2 target{
                control.targetVelocityMetersPerSecond.x,
                control.targetVelocityMetersPerSecond.y};
            const double inwardTargetSpeed = glm::dot(target, normal);
            if (inwardTargetSpeed < 0.0)
            {
                control.targetVelocityMetersPerSecond.x -= inwardTargetSpeed * normal.x;
                control.targetVelocityMetersPerSecond.y -= inwardTargetSpeed * normal.y;
            }
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
        target.yawRadians = DroneModel::YawRadians(target);
    }

    void AdvancePhysicsWorld()
    {
        physics->Step(DroneModel::fixedStepSeconds,
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
    normalized.yawRadians = DroneModel::YawRadians(normalized);
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
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        ecs_->AdvancePhysicsWorld();
#endif
    // 배터리 컴포넌트가 붙은 드론만 골라, 물리 계산 뒤의 비행 상태로 잔량을 갱신한다.
    ecs_->world.each(
        [](BatteryModel& battery, const DroneState& state)
        {
            battery.Advance(state.mode);
        });
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
    ecs_->world.each(
        [this](flecs::entity drone, DroneState&, DroneControlState&, CameraState& camera)
        {
            ecs_->ResetDrone(drone);
            EoCameraModel::Reset(camera);
            if (drone.has<BatteryModel>())
                drone.get_mut<BatteryModel>().Reset();
        });
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
    {
        ecs_->physics->SetWindVelocityMetersPerSecond(velocity);
        return;
    }
#endif
    ecs_->windVelocityMetersPerSecond = velocity;
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

void SimulationCore::AddStaticObstaclePrism(
    const std::vector<glm::dvec2>& outlineMeters,
    double bottomMeters, double topMeters)
{
    if (!detail::IsValidFootprint(outlineMeters) || !std::isfinite(bottomMeters)
        || !std::isfinite(topMeters) || !(topMeters > bottomMeters))
        throw std::invalid_argument("건물 외곽선과 높이 범위를 확인해 주세요.");

    for (const glm::dvec2& point : outlineMeters)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            throw std::invalid_argument("건물 외곽선 좌표를 확인해 주세요.");
    }

#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->physics->AddStaticObstaclePrism(
            outlineMeters, bottomMeters, topMeters);
        return;
    }
#endif
    ecs_->kinematicPrisms.push_back(
        {outlineMeters, bottomMeters, topMeters});
}

void SimulationCore::AddStaticObstacleFootprint(
    const std::vector<glm::dvec2>& outlineMeters)
{
    if (!detail::IsValidFootprint(outlineMeters))
        throw std::invalid_argument("구조물 외곽선 좌표를 확인해 주세요.");

#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->physics->AddStaticObstacleFootprint(outlineMeters);
        return;
    }
#endif
    ecs_->kinematicFootprints.push_back({outlineMeters});
}

void SimulationCore::SetTerrain(terrain::TerrainGrid terrain)
{
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->physics->SetTerrain(std::move(terrain));
        return;
    }
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

}
