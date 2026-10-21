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
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <glm/geometric.hpp>

#include <cmath>
#include <stdexcept>
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

JPH::Vec3 ToJolt(const glm::dvec3& value)
{
    return {
        static_cast<float>(value.x),
        static_cast<float>(value.y),
        static_cast<float>(value.z)};
}

JPH::Vec3 ToJolt(const glm::vec3& value)
{
    return {value.x, value.y, value.z};
}

JPH::Quat ToJolt(const glm::dquat& value)
{
    return JPH::Quat{
        static_cast<float>(value.x),
        static_cast<float>(value.y),
        static_cast<float>(value.z),
        static_cast<float>(value.w)}.Normalized();
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
    std::vector<JPH::BodyID> bodies;

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
        const JPH::BodyID ground = physics.GetBodyInterface().CreateAndAddBody(
            settings, JPH::EActivation::DontActivate);
        if (ground.IsInvalid())
            throw std::runtime_error("Jolt 바닥 강체를 만들지 못했습니다.");
    }

    JPH::BodyID GetBody(RigidBodyHandle body) const
    {
        if (body.value >= bodies.size())
            throw std::out_of_range("존재하지 않는 강체 번호입니다.");
        return bodies[body.value];
    }
};

PhysicsWorld::PhysicsWorld()
    : impl_(std::make_unique<Impl>())
{
}

PhysicsWorld::~PhysicsWorld() = default;

RigidBodyHandle PhysicsWorld::CreateBox(const RigidBodyDefinition& definition)
{
    if (!(definition.halfExtentsMeters.x > 0.0F)
        || !(definition.halfExtentsMeters.y > 0.0F)
        || !(definition.halfExtentsMeters.z > 0.0F)
        || !std::isfinite(definition.halfExtentsMeters.x)
        || !std::isfinite(definition.halfExtentsMeters.y)
        || !std::isfinite(definition.halfExtentsMeters.z)
        || !std::isfinite(definition.massKilograms)
        || definition.massKilograms < 0.0
        || !IsFinite(definition.positionMeters)
        || !std::isfinite(glm::length(definition.orientation))
        || !(glm::length(definition.orientation) > 0.0)
        || (!definition.isStatic && !(definition.massKilograms > 0.0)))
    {
        throw std::invalid_argument("강체 크기, 위치, 질량을 확인해 주세요.");
    }

    const JPH::EMotionType motion = definition.isStatic
        ? JPH::EMotionType::Static
        : JPH::EMotionType::Dynamic;
    JPH::BodyCreationSettings settings{
        new JPH::BoxShape(ToJolt(definition.halfExtentsMeters), 0.0F),
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
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = static_cast<float>(definition.massKilograms);
    }

    JPH::BodyInterface& bodyInterface = impl_->physics.GetBodyInterface();
    const JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, JPH::EActivation::Activate);
    if (id.IsInvalid())
        throw std::runtime_error("Jolt 움직이는 강체를 만들지 못했습니다.");

    const auto handle = RigidBodyHandle{static_cast<std::uint32_t>(impl_->bodies.size())};
    impl_->bodies.push_back(id);
    return handle;
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

void PhysicsWorld::Step(double seconds, int collisionSteps)
{
    if (!std::isfinite(seconds) || !(seconds > 0.0) || collisionSteps <= 0)
        throw std::invalid_argument("물리 시간 간격과 충돌 계산 횟수를 확인해 주세요.");
    impl_->physics.Update(static_cast<float>(seconds), collisionSteps,
        &impl_->tempAllocator, &impl_->jobSystem);
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
