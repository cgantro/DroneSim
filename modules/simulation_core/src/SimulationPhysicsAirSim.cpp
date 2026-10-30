#include "simulation_core/SimulationPhysics.h"

#include "simulation_core/PhysicsWorld.h"
#include "simulation_core/DroneModel.h"

#include <utility>

namespace aerosimlab::simulation
{
struct SimulationPhysics::Impl
{
    PhysicsWorld world;
};

SimulationPhysics::SimulationPhysics() : impl_(std::make_unique<Impl>()) {}

SimulationPhysics::~SimulationPhysics() = default;

RigidBodyHandle SimulationPhysics::CreateQuadrotor(
    const RigidBodyDefinition& definition, const QuadrotorParameters& parameters)
{
    return impl_->world.CreateQuadrotor(definition, parameters);
}

bool SimulationPhysics::DestroyBody(RigidBodyHandle body)
{
    return impl_->world.DestroyBody(body);
}

void SimulationPhysics::SetMotorTargets(
    RigidBodyHandle body, const std::array<double, 4>& speeds)
{
    impl_->world.SetMotorTargets(body, speeds);
}

void SimulationPhysics::SetInitialMotorSpeeds(
    RigidBodyHandle body, const std::array<double, 4>& speeds)
{
    impl_->world.SetInitialMotorSpeeds(body, speeds);
}

std::array<double, 4> SimulationPhysics::MotorSpeedsRadiansPerSecond(
    RigidBodyHandle body) const
{
    return impl_->world.MotorSpeedsRadiansPerSecond(body);
}

void SimulationPhysics::SetState(
    RigidBodyHandle body, const RigidBodyState& state)
{
    impl_->world.SetState(body, state);
}

RigidBodyState SimulationPhysics::State(RigidBodyHandle body) const
{
    return impl_->world.State(body);
}

void SimulationPhysics::AddStaticObstacleBox(
    const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters)
{
    impl_->world.AddStaticObstacleBox(centerMeters, halfExtentsMeters);
}

void SimulationPhysics::AddStaticObstaclePrism(
    const std::vector<glm::dvec2>& outlineMeters,
    double bottomMeters, double topMeters)
{
    impl_->world.AddStaticObstaclePrism(outlineMeters, bottomMeters, topMeters);
}

void SimulationPhysics::SetTerrain(terrain::TerrainGrid terrain)
{
    impl_->world.SetTerrain(std::move(terrain));
}

void SimulationPhysics::SetWindVelocityMetersPerSecond(const glm::dvec3& velocity)
{
    impl_->world.SetWindVelocityMetersPerSecond(velocity);
}

void SimulationPhysics::StepFixed(double* stepDurationMicroseconds)
{
    impl_->world.Step(DroneModel::fixedStepSeconds, stepDurationMicroseconds);
}
}
