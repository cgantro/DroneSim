#include "simulation_core/SimulationPhysics.h"

#include "simulation_core/JoltPhysicsWorld.h"

#include <optional>
#include <utility>

namespace dronesim::simulation
{
// 공개 헤더에는 Jolt를 노출하지 않고, 이 구현 파일에서만 실제 물리 월드를 소유한다.
// unique_ptr가 Impl을 파괴하므로 Jolt 월드와 그 안의 물리 자원도 함께 정리된다.
struct SimulationPhysics::Impl
{
    JoltPhysicsWorld world;
    std::optional<JoltRigidBodyHandle> defaultGround;

    Impl()
        : defaultGround(world.AddStaticBox(
            {0.0, 0.0, -0.5}, {10000.0, 10000.0, 0.5}))
    {
        // 바닥 윗면을 z=0에 맞춘 20km 정사각형이다. 지형을 불러오기 전에도
        // 드론이 빈 공간으로 떨어지지 않게 임시 바닥으로 사용한다.
    }
};

namespace
{
JoltRigidBodyHandle ToJolt(RigidBodyHandle body) noexcept
{
    // 두 핸들은 같은 월드의 물체 번호를 공유한다. 변환은 이름이 다른 래퍼만 벗긴다.
    return {body.value};
}

RigidBodyHandle FromJolt(JoltRigidBodyHandle body) noexcept
{
    return {body.value};
}

JoltRigidBodyState ToJolt(const RigidBodyState& state) noexcept
{
    // 두 공개 상태 형식 모두 ENU 좌표, 미터, 초, 라디안 단위를 사용한다.
    // 축 변환이 필요하면 이 경계가 아니라 JoltPhysicsWorld가 내부 Jolt 좌표로 바꾼다.
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
    // 새 지형이 성공적으로 만들어진 뒤에만 임시 평면을 지운다.
    // 지형 생성 중 오류가 나면 기존 바닥이 남아 시뮬레이션을 받쳐 준다.
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
