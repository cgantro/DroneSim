#include "simulation_core/SimulationPhysics.h"

#include "simulation_core/JoltPhysicsWorld.h"

#include <optional>
#include <utility>

namespace aerosimlab::simulation
{
// 공개 물리 API와 Jolt의 전용 타입 사이에서 변환을 맡는다.
struct SimulationPhysics::Impl
{
    JoltPhysicsWorld world;
    std::optional<JoltRigidBodyHandle> defaultGround;

    Impl()
        : defaultGround(world.AddStaticBox(
            {0.0, 0.0, -0.5}, {10000.0, 10000.0, 0.5}))
    {
        // 실제 지형을 지정하기 전에도 드론이 끝없이 떨어지지 않도록 큰 바닥을 둔다.
    }
};

namespace
{
JoltRigidBodyHandle ToJolt(RigidBodyHandle body) noexcept
{
    return {body.value};
}

RigidBodyHandle FromJolt(JoltRigidBodyHandle body) noexcept
{
    return {body.value};
}

JoltRigidBodyState ToJolt(const RigidBodyState& state) noexcept
{
    return {state.positionMeters, state.orientation,
        state.linearVelocityMetersPerSecond, state.angularVelocityRadiansPerSecond};
}

RigidBodyState FromJolt(const JoltRigidBodyState& state) noexcept
{
    return {state.positionMeters, state.orientation,
        state.linearVelocityMetersPerSecond, state.angularVelocityRadiansPerSecond};
}
}

SimulationPhysics::SimulationPhysics() : impl_(std::make_unique<Impl>()) {}

SimulationPhysics::~SimulationPhysics() = default;

RigidBodyHandle SimulationPhysics::CreateQuadrotor(
    const RigidBodyDefinition& definition, const QuadrotorParameters& parameters)
{
    JoltRigidBodyDefinition body;
    body.positionMeters = definition.positionMeters;
    body.orientation = definition.orientation;
    return FromJolt(impl_->world.CreateQuadrotor(body, parameters));
}

bool SimulationPhysics::DestroyBody(RigidBodyHandle body)
{
    return impl_->world.DestroyBody(ToJolt(body));
}

void SimulationPhysics::SetMotorTargets(
    RigidBodyHandle body, const std::array<double, 4>& speeds)
{
    impl_->world.SetMotorTargets(ToJolt(body), speeds);
}

void SimulationPhysics::SetInitialMotorSpeeds(
    RigidBodyHandle body, const std::array<double, 4>& speeds)
{
    impl_->world.SetInitialMotorSpeeds(ToJolt(body), speeds);
}

std::array<double, 4> SimulationPhysics::MotorSpeedsRadiansPerSecond(
    RigidBodyHandle body) const
{
    return impl_->world.MotorSpeedsRadiansPerSecond(ToJolt(body));
}

void SimulationPhysics::SetState(
    RigidBodyHandle body, const RigidBodyState& state)
{
    impl_->world.SetState(ToJolt(body), ToJolt(state));
}

RigidBodyState SimulationPhysics::State(RigidBodyHandle body) const
{
    return FromJolt(impl_->world.State(ToJolt(body)));
}

void SimulationPhysics::AddStaticObstacleBox(
    const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters)
{
    static_cast<void>(
        impl_->world.AddStaticBox(centerMeters, glm::dvec3{halfExtentsMeters}));
}

void SimulationPhysics::AddStaticObstacleMesh(
    std::span<const glm::dvec3> verticesMeters,
    std::span<const std::uint32_t> triangleIndices)
{
    impl_->world.AddStaticTriangleMesh(verticesMeters, triangleIndices);
}

void SimulationPhysics::AddStaticObstaclePrism(
    const std::vector<glm::dvec2>& outlineMeters,
    double bottomMeters, double topMeters)
{
    impl_->world.AddStaticPrism(outlineMeters, bottomMeters, topMeters);
}

void SimulationPhysics::SetTerrain(terrain::TerrainGrid terrain)
{
    impl_->world.SetTerrain(terrain);
    if (impl_->defaultGround)
    {
        // 높이 격자 바닥으로 바꾼 뒤에는 기본 평면이 겹치지 않도록 제거한다.
        static_cast<void>(impl_->world.DestroyBody(*impl_->defaultGround));
        impl_->defaultGround.reset();
    }
}

void SimulationPhysics::SetWindVelocityMetersPerSecond(const glm::dvec3& velocity)
{
    impl_->world.SetWindVelocityMetersPerSecond(velocity);
}

void SimulationPhysics::StepFixed(double* stepDurationMicroseconds)
{
    impl_->world.StepFixed(stepDurationMicroseconds);
}
}
