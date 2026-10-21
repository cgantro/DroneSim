#include "simulation_core/SimulationCore.h"
#if !defined(__EMSCRIPTEN__)
#include "simulation_core/PhysicsWorld.h"
#endif
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <flecs.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace aerosimlab::simulation
{
struct SimulationCore::EcsStorage
{
#if !defined(__EMSCRIPTEN__)
    struct PhysicsBody
    {
        RigidBodyHandle handle;
    };

    struct DroneActuation
    {
        QuadrotorParameters parameters;
        QuadrotorState motors;
    };
#endif

    SimulationDynamics dynamics;
    flecs::world world;
    flecs::entity drone;
    flecs::entity camera;
    glm::dvec3 windVelocityMetersPerSecond{0.0};
#if !defined(__EMSCRIPTEN__)
    std::unique_ptr<PhysicsWorld> physics;
    RigidBodyHandle physicsBody;
#endif
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
#if !defined(__EMSCRIPTEN__)
        else
        {
            CreatePhysicsBody();
            world.system<DroneState, DroneControlState, PhysicsBody, DroneActuation>(
                "AdvanceRigidBodyDrone")
                .kind(flecs::OnUpdate)
                .each([this](
                    DroneState& state,
                    DroneControlState& control,
                    const PhysicsBody& body,
                    DroneActuation& actuation)
                {
                    AdvanceRigidBody(state, control, body.handle, actuation);
                });
        }
#endif

        world.system<CameraState>("AdvanceCamera")
            .kind(flecs::OnUpdate)
            .each([this](CameraState& state)
            {
                EoCameraModel::AdvanceOneTick(state, tickCount + 1);
            });
    }

#if !defined(__EMSCRIPTEN__)
    void CreatePhysicsBody()
    {
        const QuadrotorParameters parameters = QuadrotorParameters::X500MotorTableEstimate();
        physics = std::make_unique<PhysicsWorld>();
        RigidBodyDefinition definition;
        definition.positionMeters = {
            0.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters};
        definition.massKilograms = parameters.massKilograms;
        definition.principalInertiaKilogramMetersSquared =
            parameters.principalInertiaKilogramMetersSquared;
        physicsBody = physics->CreateCompoundBoxes(definition, parameters.collisionBoxes);
        drone.set<PhysicsBody>({physicsBody});
        drone.set<DroneActuation>({parameters, {}});
        CopyPhysicsState(drone.get_mut<DroneState>(), physicsBody);
    }

    void ResetPhysicsBody()
    {
        RigidBodyState initialState;
        initialState.positionMeters = {
            0.0, 0.0, QuadrotorFlightController::groundedModelOriginHeightMeters};
        physics->SetState(physicsBody, initialState);
        drone.get_mut<DroneActuation>().motors = {};
        CopyPhysicsState(drone.get_mut<DroneState>(), physicsBody);
    }

    void AdvanceRigidBody(
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
        QuadrotorModel::AdvanceMotors(
            actuation.motors, targetSpeeds, actuation.parameters, step);
        const glm::dquat worldToBody = glm::inverse(state.orientation);
        const glm::dvec3 airRelativeVelocityBody = worldToBody
            * (state.velocityMetersPerSecond - windVelocityMetersPerSecond);
        const glm::dvec3 angularVelocityBody =
            worldToBody * state.angularVelocityRadiansPerSecond;
        const QuadrotorWrench wrench = QuadrotorModel::CalculateWrench(
            actuation.motors, actuation.parameters,
            airRelativeVelocityBody, angularVelocityBody);
        physics->ApplyForce(body, state.orientation * wrench.forceNewtons);
        physics->ApplyTorque(body, state.orientation * wrench.torqueNewtonMeters);
        physics->Step(step);
        CopyPhysicsState(state, body);

        if (state.mode == FlightMode::TakingOff
            && state.positionMeters.z >= DroneModel::takeoffHeightMeters - 0.03
            && std::abs(state.velocityMetersPerSecond.z) < 0.15)
        {
            state.mode = FlightMode::Hovering;
        }
        else if (state.mode == FlightMode::FollowingWaypoint
            && glm::length(control.waypointMeters - state.positionMeters) < 0.05
            && glm::length(state.velocityMetersPerSecond) < 0.15)
        {
            state.mode = FlightMode::Hovering;
            control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
        }
        else if (state.mode == FlightMode::Landing
            && state.positionMeters.z <= QuadrotorFlightController::groundedModelOriginHeightMeters + 0.02
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
        const glm::dquat& q = source.orientation;
        target.yawRadians = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
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
    if (paused_)
    {
        return CommandResult::SimulationPaused;
    }
    DroneState& state = ecs_->drone.get_mut<DroneState>();
    DroneControlState& control = ecs_->drone.get_mut<DroneControlState>();
#if defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
        return CommandResult::UnsupportedDynamics;
#endif
    const CommandResult result = DroneModel::Apply(state, control, command);
#if !defined(__EMSCRIPTEN__)
    if (result == CommandResult::Accepted
        && ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->CopyPhysicsState(state, ecs_->physicsBody);
    }
#endif
    return result;
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
#if !defined(__EMSCRIPTEN__)
    if (ecs_->dynamics == SimulationDynamics::RigidBody)
    {
        ecs_->ResetPhysicsBody();
    }
#endif
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

void SimulationCore::SetWindVelocityMetersPerSecond(const glm::dvec3& velocity)
{
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z))
        throw std::invalid_argument("바람 속도는 유한한 값이어야 합니다.");
    ecs_->windVelocityMetersPerSecond = velocity;
}

#ifndef __EMSCRIPTEN__
void SimulationCore::SetTerrain(const terrain::TerrainGrid& terrain)
{
    ecs_->physics->SetTerrain(terrain);
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
