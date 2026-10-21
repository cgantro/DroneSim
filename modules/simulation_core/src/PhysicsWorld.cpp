#include "simulation_core/PhysicsWorld.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <glm/geometric.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace aerosimlab::simulation
{
namespace
{
constexpr JPH::ObjectLayer staticLayer = 0;
constexpr JPH::ObjectLayer movingLayer = 1;
constexpr JPH::uint layerCount = 2;
constexpr JPH::uint maximumBodies = 1024;
constexpr JPH::uint maximumBodyPairs = 1024;
constexpr JPH::uint maximumContacts = 1024;
constexpr JPH::uint maximumPhysicsJobs = 1024;
constexpr float gravityMetersPerSecondSquared = 9.81F;

/** @brief Jolt에는 한 번만 전역 형식을 등록하고 프로그램이 끝날 때 해제한다. */
class JoltRuntime
{
public:
    JoltRuntime()
    {
        JPH::RegisterDefaultAllocator();
        factory_ = std::make_unique<JPH::Factory>();
        JPH::Factory::sInstance = factory_.get();
        JPH::RegisterTypes();
    }

    ~JoltRuntime()
    {
        JPH::UnregisterTypes();
        JPH::Factory::sInstance = nullptr;
    }

private:
    std::unique_ptr<JPH::Factory> factory_;
};

JoltRuntime& Runtime()
{
    static JoltRuntime runtime;
    return runtime;
}

/** @brief Jolt가 쓰는 float 범위를 확인한 뒤 변환합니다. */
float ToJoltFloat(double value)
{
    constexpr double maximumFloat = std::numeric_limits<float>::max();
    if (!std::isfinite(value) || value > maximumFloat || value < -maximumFloat)
        throw std::invalid_argument("물리 값이 Jolt가 표현할 수 있는 범위를 벗어났습니다.");
    return static_cast<float>(value);
}

JPH::Vec3 ToJolt(const glm::dvec3& value)
{
    return {ToJoltFloat(value.x), ToJoltFloat(value.y), ToJoltFloat(value.z)};
}

JPH::Vec3 ToJolt(const glm::vec3& value)
{
    return {value.x, value.y, value.z};
}

JPH::Quat ToJolt(const glm::dquat& value)
{
    // 큰 값을 float으로 먼저 바꾸면 길이 제곱이 넘칠 수 있어 double 범위에서 정규화한다.
    const double scale = std::max({
        std::abs(value.w), std::abs(value.x), std::abs(value.y), std::abs(value.z)});
    if (!std::isfinite(scale) || !(scale > 0.0))
        throw std::invalid_argument("강체의 방향을 나타내는 값을 확인해 주세요.");

    const double scaledW = value.w / scale;
    const double scaledX = value.x / scale;
    const double scaledY = value.y / scale;
    const double scaledZ = value.z / scale;
    const double length = std::sqrt(
        scaledW * scaledW + scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ);
    return JPH::Quat{
        static_cast<float>(scaledX / length),
        static_cast<float>(scaledY / length),
        static_cast<float>(scaledZ / length),
        static_cast<float>(scaledW / length)};
}

glm::dvec3 ToGlm(const JPH::Vec3& value)
{
    return {value.GetX(), value.GetY(), value.GetZ()};
}

glm::dquat ToGlm(const JPH::Quat& value)
{
    return {value.GetW(), value.GetX(), value.GetY(), value.GetZ()};
}

class ObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override
    {
        return first == movingLayer || second == movingLayer;
    }
};

class AppBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface
{
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return layerCount; }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
    {
        return JPH::BroadPhaseLayer{static_cast<JPH::BroadPhaseLayer::Type>(layer)};
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
    {
        switch (static_cast<JPH::BroadPhaseLayer::Type>(layer))
        {
        case staticLayer: return "static";
        case movingLayer: return "moving";
        default: return "unknown";
        }
    }
#endif
};

class AppObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer object, JPH::BroadPhaseLayer broadPhase) const override
    {
        return object == movingLayer
            || static_cast<JPH::BroadPhaseLayer::Type>(broadPhase) == movingLayer;
    }
};

bool IsFinite(const glm::dvec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
}

struct PhysicsWorld::Impl
{
    JoltRuntime& runtime;
    AppBroadPhaseLayerInterface broadPhaseLayers;
    ObjectLayerPairFilter objectLayerPairFilter;
    AppObjectVsBroadPhaseFilter objectVsBroadPhaseFilter;
    JPH::PhysicsSystem physics;
    JPH::TempAllocatorImpl tempAllocator{4 * 1024 * 1024};
    JPH::JobSystemSingleThreaded jobSystem{maximumPhysicsJobs};
    std::vector<std::optional<JPH::BodyID>> bodies;
    JPH::BodyID groundBody;
    JPH::BodyID terrainBody;
    bool hasTerrain = false;

    Impl()
        : runtime(Runtime())
    {
        physics.Init(
            maximumBodies,
            0,
            maximumBodyPairs,
            maximumContacts,
            broadPhaseLayers,
            objectVsBroadPhaseFilter,
            objectLayerPairFilter);
        physics.SetGravity(JPH::Vec3{0.0F, 0.0F, -gravityMetersPerSecondSquared});
        CreateGround();
    }

    void CreateGround()
    {
        JPH::BodyCreationSettings settings{
            new JPH::BoxShape(JPH::Vec3{100.0F, 100.0F, 0.5F}, 0.0F),
            JPH::RVec3{0.0F, 0.0F, -0.5F},
            JPH::Quat::sIdentity(),
            JPH::EMotionType::Static,
            staticLayer};
        settings.mFriction = 0.8F;
        groundBody = physics.GetBodyInterface().CreateAndAddBody(
            settings, JPH::EActivation::DontActivate);
        if (groundBody.IsInvalid())
            throw std::runtime_error("Jolt 바닥 강체를 만들지 못했습니다.");
    }

    JPH::BodyID GetBody(RigidBodyHandle body) const
    {
        if (body.value >= bodies.size() || !bodies[body.value].has_value())
            throw std::out_of_range("존재하지 않는 강체 번호입니다.");
        return *bodies[body.value];
    }
};

PhysicsWorld::PhysicsWorld()
    : impl_(std::make_unique<Impl>())
{
}

PhysicsWorld::~PhysicsWorld() = default;

RigidBodyHandle PhysicsWorld::CreateBox(const RigidBodyDefinition& definition)
{
    const RigidBodyBox box{definition.halfExtentsMeters, {}, {1.0, 0.0, 0.0, 0.0}};
    return CreateCompoundBoxes(definition, std::span{&box, 1U});
}

RigidBodyHandle PhysicsWorld::CreateCompoundBoxes(
    const RigidBodyDefinition& definition,
    std::span<const RigidBodyBox> boxes)
{
    if (boxes.empty()
        || !std::isfinite(definition.massKilograms)
        || definition.massKilograms < 0.0
        || !IsFinite(definition.positionMeters)
        || !std::isfinite(glm::length(definition.orientation))
        || !(glm::length(definition.orientation) > 0.0)
        || (definition.principalInertiaKilogramMetersSquared.has_value()
            && (!IsFinite(*definition.principalInertiaKilogramMetersSquared)
                || !(definition.principalInertiaKilogramMetersSquared->x > 0.0)
                || !(definition.principalInertiaKilogramMetersSquared->y > 0.0)
                || !(definition.principalInertiaKilogramMetersSquared->z > 0.0)))
        || (!definition.isStatic && !(definition.massKilograms > 0.0)))
    {
        throw std::invalid_argument("강체 위치와 질량을 확인해 주세요.");
    }

    JPH::StaticCompoundShapeSettings shapeSettings;
    for (const RigidBodyBox& box : boxes)
    {
        if (!(box.halfExtentsMeters.x > 0.0F)
            || !(box.halfExtentsMeters.y > 0.0F)
            || !(box.halfExtentsMeters.z > 0.0F)
            || !std::isfinite(box.halfExtentsMeters.x)
            || !std::isfinite(box.halfExtentsMeters.y)
            || !std::isfinite(box.halfExtentsMeters.z)
            || !IsFinite(box.positionMeters)
            || !std::isfinite(glm::length(box.orientation))
            || !(glm::length(box.orientation) > 0.0))
        {
            throw std::invalid_argument("강체 충돌 상자의 크기와 자세를 확인해 주세요.");
        }
        shapeSettings.AddShape(
            ToJolt(box.positionMeters),
            ToJolt(box.orientation),
            new JPH::BoxShape(ToJolt(box.halfExtentsMeters), 0.0F));
    }

    const JPH::ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(std::string("Jolt 복합 충돌 모양을 만들지 못했습니다: ")
            + shapeResult.GetError().c_str());

    const JPH::EMotionType motion = definition.isStatic
        ? JPH::EMotionType::Static
        : JPH::EMotionType::Dynamic;
    JPH::BodyCreationSettings settings{
        shapeResult.Get(),
        JPH::RVec3{ToJolt(definition.positionMeters)},
        ToJolt(definition.orientation),
        motion,
        definition.isStatic ? staticLayer : movingLayer};
    settings.mFriction = 0.8F;
    settings.mLinearDamping = 0.0F;
    settings.mAngularDamping = 0.0F;
    settings.mAllowSleeping = false;
    if (!definition.isStatic)
    {
        settings.mMassPropertiesOverride.mMass = ToJoltFloat(definition.massKilograms);
        if (definition.principalInertiaKilogramMetersSquared.has_value())
        {
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
            settings.mMassPropertiesOverride.mInertia =
                JPH::Mat44::sScale(ToJolt(*definition.principalInertiaKilogramMetersSquared));
        }
        else
        {
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        }
    }

    JPH::BodyInterface& bodyInterface = impl_->physics.GetBodyInterface();
    const JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::Activate);
    if (id.IsInvalid())
        throw std::runtime_error("Jolt 움직이는 강체를 만들지 못했습니다.");

    const auto handle = RigidBodyHandle{static_cast<std::uint32_t>(impl_->bodies.size())};
    impl_->bodies.emplace_back(id);
    return handle;
}

void PhysicsWorld::SetTerrain(const terrain::TerrainGrid& terrain)
{
    if (terrain.sampleCount < 3U
        || terrain.heightOffsetsMeters.size()
            != static_cast<std::size_t>(terrain.sampleCount) * terrain.sampleCount
        || !std::isfinite(terrain.spacingXMeters) || !(terrain.spacingXMeters > 0.0F)
        || !std::isfinite(terrain.spacingYMeters) || !(terrain.spacingYMeters > 0.0F)
        || !std::all_of(terrain.heightOffsetsMeters.begin(), terrain.heightOffsetsMeters.end(),
            [](float height) { return std::isfinite(height); }))
    {
        throw std::invalid_argument("지형 격자 크기, 간격 또는 높이 값을 확인해 주세요.");
    }

    const float halfWidth = static_cast<float>(terrain.sampleCount - 1U)
        * terrain.spacingXMeters * 0.5F;
    const float halfDepth = static_cast<float>(terrain.sampleCount - 1U)
        * terrain.spacingYMeters * 0.5F;
    JPH::HeightFieldShapeSettings shapeSettings{
        terrain.heightOffsetsMeters.data(),
        JPH::Vec3{-halfWidth, 0.0F, -halfDepth},
        JPH::Vec3{terrain.spacingXMeters, 1.0F, terrain.spacingYMeters},
        terrain.sampleCount};
    shapeSettings.mBitsPerSample = 16;
    const JPH::ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(std::string("Jolt 지형 충돌 모양을 만들지 못했습니다: ")
            + shapeResult.GetError().c_str());

    const JPH::Quat simulationUpRotation = JPH::Quat::sRotation(
        JPH::Vec3::sAxisX(), JPH::DegreesToRadians(90.0F));
    JPH::BodyCreationSettings settings{
        shapeResult.Get(), JPH::RVec3::sZero(), simulationUpRotation,
        JPH::EMotionType::Static, staticLayer};
    settings.mFriction = 0.8F;

    JPH::BodyInterface& bodyInterface = impl_->physics.GetBodyInterface();
    const JPH::BodyID newTerrainBody = bodyInterface.CreateAndAddBody(
        settings, JPH::EActivation::DontActivate);
    if (newTerrainBody.IsInvalid())
        throw std::runtime_error("Jolt 지형 충돌체를 만들지 못했습니다.");

    if (impl_->hasTerrain)
    {
        bodyInterface.RemoveBody(impl_->terrainBody);
        bodyInterface.DestroyBody(impl_->terrainBody);
    }
    else
    {
        bodyInterface.RemoveBody(impl_->groundBody);
        bodyInterface.DestroyBody(impl_->groundBody);
    }
    impl_->terrainBody = newTerrainBody;
    impl_->hasTerrain = true;
}

void PhysicsWorld::SetState(RigidBodyHandle body, const RigidBodyState& state)
{
    if (!IsFinite(state.positionMeters)
        || !IsFinite(state.linearVelocityMetersPerSecond)
        || !IsFinite(state.angularVelocityRadiansPerSecond)
        || !std::isfinite(glm::length(state.orientation))
        || !(glm::length(state.orientation) > 0.0))
    {
        throw std::invalid_argument("강체의 초기 상태를 확인해 주세요.");
    }

    impl_->physics.GetBodyInterface().SetPositionRotationAndVelocity(
        impl_->GetBody(body),
        JPH::RVec3{ToJolt(state.positionMeters)},
        ToJolt(state.orientation),
        ToJolt(state.linearVelocityMetersPerSecond),
        ToJolt(state.angularVelocityRadiansPerSecond));
}

void PhysicsWorld::ApplyForce(RigidBodyHandle body, const glm::dvec3& forceNewtons)
{
    if (!IsFinite(forceNewtons))
        throw std::invalid_argument("힘은 유한한 숫자여야 합니다.");
    impl_->physics.GetBodyInterface().AddForce(impl_->GetBody(body), ToJolt(forceNewtons));
}

void PhysicsWorld::ApplyForceAtPoint(
    RigidBodyHandle body,
    const glm::dvec3& forceNewtons,
    const glm::dvec3& worldPositionMeters)
{
    if (!IsFinite(forceNewtons) || !IsFinite(worldPositionMeters))
        throw std::invalid_argument("힘과 작용점은 유한한 숫자여야 합니다.");
    impl_->physics.GetBodyInterface().AddForce(
        impl_->GetBody(body), ToJolt(forceNewtons), JPH::RVec3{ToJolt(worldPositionMeters)});
}

void PhysicsWorld::ApplyTorque(RigidBodyHandle body, const glm::dvec3& torqueNewtonMeters)
{
    if (!IsFinite(torqueNewtonMeters))
        throw std::invalid_argument("토크는 유한한 숫자여야 합니다.");
    impl_->physics.GetBodyInterface().AddTorque(impl_->GetBody(body), ToJolt(torqueNewtonMeters));
}

void PhysicsWorld::Step(double seconds, int collisionSteps, double* durationMicroseconds)
{
    if (!std::isfinite(seconds) || !(seconds > 0.0) || collisionSteps <= 0)
        throw std::invalid_argument("물리 시간 간격과 충돌 계산 횟수를 확인해 주세요.");

    const float stepSeconds = ToJoltFloat(seconds);
    if (!(stepSeconds > 0.0F))
        throw std::invalid_argument("물리 시간 간격이 Jolt에서 0초로 줄어듭니다.");

    if (durationMicroseconds == nullptr)
    {
        impl_->physics.Update(stepSeconds, collisionSteps,
            &impl_->tempAllocator, &impl_->jobSystem);
        return;
    }

    const auto start = std::chrono::steady_clock::now();
    impl_->physics.Update(stepSeconds, collisionSteps,
        &impl_->tempAllocator, &impl_->jobSystem);
    *durationMicroseconds = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - start).count();
}

bool PhysicsWorld::DestroyBody(RigidBodyHandle body)
{
    if (body.value >= impl_->bodies.size() || !impl_->bodies[body.value].has_value())
        return false;

    const JPH::BodyID id = *impl_->bodies[body.value];
    JPH::BodyInterface& bodyInterface = impl_->physics.GetBodyInterface();
    bodyInterface.RemoveBody(id);
    bodyInterface.DestroyBody(id);
    impl_->bodies[body.value].reset();
    return true;
}

RigidBodyState PhysicsWorld::State(RigidBodyHandle body) const
{
    const JPH::BodyID id = impl_->GetBody(body);
    const JPH::BodyInterface& bodyInterface = impl_->physics.GetBodyInterface();
    JPH::RVec3 position;
    JPH::Quat orientation;
    JPH::Vec3 linearVelocity;
    JPH::Vec3 angularVelocity;
    bodyInterface.GetPositionAndRotation(id, position, orientation);
    bodyInterface.GetLinearAndAngularVelocity(id, linearVelocity, angularVelocity);
    return {
        ToGlm(position),
        ToGlm(orientation),
        ToGlm(linearVelocity),
        ToGlm(angularVelocity)};
}
}
