#include "physics/PhysicsWorld.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>

#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>

#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#endif

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
#include <thread>
#endif
#include <map>
#include <unordered_map>
#include <tuple>

namespace grasplink::physics
{

namespace
{

// Jolt Factory와 타입 등록은 모든 PhysicsWorld가 공유하므로 사용 World 수를 세어 관리한다.
// mutex는 여러 thread가 이 수를 동시에 바꾸지 못하게 한다.
// 첫 World를 만들 때 등록하고 마지막 World를 해제할 때 등록을 취소한다.
std::mutex g_JoltRuntimeMutex;
std::size_t g_JoltRuntimeUsers = 0;
std::unique_ptr<JPH::Factory> g_JoltFactory;

struct PhysicsWorldLimits final
{
    static constexpr JPH::ObjectLayer collisionCategoryCount =
        static_cast<JPH::ObjectLayer>(CollisionLayer::Count);
    static constexpr JPH::ObjectLayer movingLayerOffset = collisionCategoryCount;
    static constexpr JPH::ObjectLayer objectLayerCount = collisionCategoryCount * 2;
    static constexpr JPH::uint broadPhaseLayerCount = 2;

    static constexpr JPH::uint temporaryAllocatorSizeBytes = 10U * 1024U * 1024U;
    static constexpr JPH::uint maximumBodies = 4096;
    static constexpr JPH::uint bodyMutexCount = 0;
    static constexpr JPH::uint maximumBodyPairs = 65536;
    static constexpr JPH::uint maximumContactConstraints = 10240;
    static constexpr JPH::uint bodyPairCount = 2;
    static constexpr unsigned int reservedWorkerThreads = 1;
    static constexpr unsigned int maximumWorkerThreads = 4;
    static constexpr int collisionStepsPerUpdate = 1;
    static constexpr std::size_t minimumConvexHullPointCount = 4;
};

struct PhysicsWorldSettings final
{
    static constexpr float gravityYMetersPerSecondSquared = -9.81F;
    static constexpr float penetrationSlopMeters = 0.002F;
    static constexpr float maximumBoxConvexRadiusMeters = 0.002F;
    static constexpr float boxConvexRadiusExtentRatio = 0.1F;
    static constexpr float convexHullRadiusMeters = 0.0F;
    static constexpr float minimumRotationLengthSquared = 1.0e-12F;
};

// 공개 API는 double을 받지만 Jolt는 float만 받으므로 변환해도 유한한 양수인지 먼저 확인한다.
bool IsValidFixedDeltaSeconds(double seconds) noexcept
{
    return std::isfinite(seconds) && seconds > 0.0 &&
        seconds <= static_cast<double>(std::numeric_limits<float>::max());
}

struct PhysicsWorldIdentity final
{
    static constexpr std::uint64_t firstWorldToken = 1;
    static constexpr std::uint64_t firstConstraintId = 1;
};

std::atomic<std::uint64_t> g_NextWorldToken{PhysicsWorldIdentity::firstWorldToken};

struct PhysicsErrorMessages final
{
    inline static constexpr char joltFactoryAlreadyInitialized[] = "Jolt Factory is already initialized.";
    inline static constexpr char unsupportedBodyMotionType[] = "Unsupported BodyMotionType.";
    inline static constexpr char invalidPhysicsBodyHandle[] = "Invalid PhysicsBodyHandle.";
    inline static constexpr char unsupportedCollisionShapeType[] = "Unsupported CollisionShapeType";
    inline static constexpr char unsupportedCollisionLayer[] = "Unsupported CollisionLayer";
    inline static constexpr char invalidBoxHalfExtents[] = "Box half extents must be finite and > 0.";
    inline static constexpr char convexHullNeedsMorePoints[] = "Convex hull requires at least four points.";
    inline static constexpr char nonFiniteConvexHullPoint[] = "Convex hull points must be finite.";
    inline static constexpr char invalidTransform[] =
        "Physics Transform requires finite position and nonzero finite rotation";
    inline static constexpr char bodyNeedsCollisionShape[] =
        "Physics Body requires at least one collision shape.";
    inline static constexpr char failedToCreatePhysicsBody[] = "Failed to create physics body.";
    inline static constexpr char unsupportedJoltMotionType[] = "Unsupported Jolt motion type.";
    inline static constexpr char fixedGraspNeedsKinematicAndDynamic[] =
        "Fixed grasp requires a kinematic anchor and a dynamic object.";
    inline static constexpr char fixedGraspBodyMissing[] = "Fixed grasp body no longer exists.";
    inline static constexpr char invalidIgnoredEnvironmentBody[] = "Invalid ignored Environment PhysicsBodyHandle.";
    inline static constexpr char ignoredBodyMustBeEnvironment[] =
        "Only an Environment body can be ignored in an environment overlap query.";
    inline static constexpr char invalidFixedDeltaSeconds[] =
        "fixedDeltaSeconds must be a finite positive float value.";
    inline static constexpr char moveKinematicRequiresKinematicBody[] =
        "MoveKinematic requires a kinematic body.";
    inline static constexpr char kinematicStopRequiresKinematicBody[] =
        "StopKinematic requires a kinematic body.";
};

void AcquireJoltRuntime()
{
    std::lock_guard<std::mutex> lock(g_JoltRuntimeMutex);

    if (g_JoltRuntimeUsers > 0)
    {
        ++g_JoltRuntimeUsers;
        return;
    }

    JPH::RegisterDefaultAllocator();

    if (JPH::Factory::sInstance != nullptr)
        throw std::runtime_error(PhysicsErrorMessages::joltFactoryAlreadyInitialized);

    g_JoltFactory = std::make_unique<JPH::Factory>();
    JPH::Factory::sInstance = g_JoltFactory.get();
    try
    {
        JPH::RegisterTypes();
    }
    catch (...)
    {
        JPH::UnregisterTypes();
        JPH::Factory::sInstance = nullptr;
        g_JoltFactory.reset();
        throw;
    }

    g_JoltRuntimeUsers = 1;
}

void ReleaseJoltRuntime()
{
    std::lock_guard<std::mutex> lock(g_JoltRuntimeMutex);

    if (g_JoltRuntimeUsers == 0)
        return;

    --g_JoltRuntimeUsers;

    if (g_JoltRuntimeUsers > 0)
        return;

    JPH::UnregisterTypes();

    JPH::Factory::sInstance = nullptr;
    g_JoltFactory.reset();
}

class JoltRuntimeLease final
{
public:
    JoltRuntimeLease() { AcquireJoltRuntime(); }
    ~JoltRuntimeLease() { ReleaseJoltRuntime(); }

    JoltRuntimeLease(const JoltRuntimeLease&) = delete;
    JoltRuntimeLease& operator=(const JoltRuntimeLease&) = delete;
    JoltRuntimeLease(JoltRuntimeLease&&) = delete;
    JoltRuntimeLease& operator=(JoltRuntimeLease&&) = delete;
};


// Jolt에 넘기는 내부 그룹 번호는 CollisionLayer 종류와 Static인지 움직이는 Body인지를 함께 담는다. 이 번호는 충돌 필터를 구현하기 위한 값이며 공개 API의 CollisionLayer 자체에 움직임 정보가 추가되는 것은 아니다.
namespace ObjectLayers
{

constexpr JPH::ObjectLayer CategoryCount = PhysicsWorldLimits::collisionCategoryCount;
constexpr JPH::ObjectLayer MovingOffset = PhysicsWorldLimits::movingLayerOffset;
constexpr JPH::ObjectLayer Count = PhysicsWorldLimits::objectLayerCount;

JPH::ObjectLayer Make(bool moving, CollisionLayer category)
{
    const JPH::ObjectLayer offset = moving ? MovingOffset : 0;
    return static_cast<JPH::ObjectLayer>(offset + static_cast<JPH::ObjectLayer>(category));
}

bool IsMoving(JPH::ObjectLayer layer)
{
    return layer >= MovingOffset;
}

CollisionLayer Category(JPH::ObjectLayer layer)
{
    return static_cast<CollisionLayer>(layer % CategoryCount);
}

class EnvironmentLayerFilter final : public JPH::ObjectLayerFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer layer) const override
    {
        return Category(layer) == CollisionLayer::Environment;
    }
};

} // namespace ObjectLayers


// Broad Phase는 모든 물체 쌍을 자세히 검사하기 전에 공간상 충돌 가능성이 있는 후보를 빠르게 고르는 단계다. 이 구현은 여기서 물체가 움직이는 그룹인지 정지 그룹인지만 나눈다. Robot과 Environment처럼 어떤 종류끼리 검사할지는 아래 범주 조합 필터가 결정한다.
namespace BroadPhaseLayers
{

constexpr JPH::BroadPhaseLayer NonMoving{0};
constexpr JPH::BroadPhaseLayer Moving{1};
constexpr JPH::uint Count = PhysicsWorldLimits::broadPhaseLayerCount;

} // namespace BroadPhaseLayers


class BroadPhaseLayerInterfaceImpl final
    : public JPH::BroadPhaseLayerInterface
{
public:
    BroadPhaseLayerInterfaceImpl()
    {
        for (JPH::ObjectLayer category = 0; category < ObjectLayers::CategoryCount; ++category)
        {
            objectToBroadPhase_[category] = BroadPhaseLayers::NonMoving;
            objectToBroadPhase_[ObjectLayers::MovingOffset + category] = BroadPhaseLayers::Moving;
        }
    }

    JPH::uint GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::Count;
    }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
    {
        JPH_ASSERT(layer < ObjectLayers::Count);
        return objectToBroadPhase_[layer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)

    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
    {
        if (layer == BroadPhaseLayers::NonMoving)
            return "NON_MOVING";

        if (layer == BroadPhaseLayers::Moving)
            return "MOVING";

        return "INVALID";
    }

#endif

private:
    JPH::BroadPhaseLayer objectToBroadPhase_[ObjectLayers::Count];
};


// 이 함수는 두 CollisionLayer 그룹을 받아 정밀 충돌 검사까지 진행할 물체 쌍인지 결정한다. Robot과 Gripper는 Environment 및 DynamicObject와만 검사하고, 서로 또는 같은 그룹끼리는 검사하지 않는다. Environment끼리는 검사하지 않는다. DynamicObject는 네 그룹 모두와 검사한다.
class ObjectLayerPairFilterImpl final
    : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(
        JPH::ObjectLayer object1,
        JPH::ObjectLayer object2) const override
    {
        const CollisionLayer category1 = ObjectLayers::Category(object1);
        const CollisionLayer category2 = ObjectLayers::Category(object2);
        switch (category1)
        {
        case CollisionLayer::Environment:
            return category2 != CollisionLayer::Environment;
        case CollisionLayer::Robot:
        case CollisionLayer::Gripper:
            return category2 == CollisionLayer::Environment ||
                category2 == CollisionLayer::DynamicObject;
        case CollisionLayer::DynamicObject:
            return true;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};


// 정지 그룹에 속한 물체는 다른 정지 물체와 검사하지 않는다. 움직이는 Body는 정지 그룹과 움직이는 그룹 모두를 검사해 바닥과 다른 물체 양쪽에 닿을 수 있게 한다.
class ObjectVsBroadPhaseLayerFilterImpl final
    : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(
        JPH::ObjectLayer objectLayer,
        JPH::BroadPhaseLayer broadPhaseLayer) const override
    {
        return ObjectLayers::IsMoving(objectLayer) ||
            broadPhaseLayer == BroadPhaseLayers::Moving;
    }
};



JPH::RVec3 ToJoltPosition(const glm::vec3& value)
{
    return JPH::RVec3(value.x, value.y, value.z);
}

JPH::Quat ToJoltRotation(const glm::quat& value)
{
    // GLM과 Jolt는 같은 회전을 서로 다른 성분 순서로 저장하므로, 이 경계에서 (w,x,y,z)를 (x,y,z,w)로 바꾼다.
    return JPH::Quat(value.x, value.y, value.z, value.w);
}

glm::vec3 ToGlmPosition(const JPH::RVec3& value)
{
    return {
        static_cast<float>(value.GetX()),
        static_cast<float>(value.GetY()),
        static_cast<float>(value.GetZ())
    };
}

glm::quat ToGlmRotation(const JPH::Quat& value)
{
    return {
        value.GetW(),
        value.GetX(),
        value.GetY(),
        value.GetZ()
    };
}


JPH::EMotionType ToJoltMotionType(BodyMotionType motionType)
{
    switch (motionType)
    {
    case BodyMotionType::Static:
        return JPH::EMotionType::Static;

    case BodyMotionType::Kinematic:
        return JPH::EMotionType::Kinematic;

    case BodyMotionType::Dynamic:
        return JPH::EMotionType::Dynamic;
    }

    throw std::invalid_argument(PhysicsErrorMessages::unsupportedBodyMotionType);
}


JPH::ObjectLayer ToObjectLayer(BodyMotionType motionType, CollisionLayer collisionLayer)
{
    return ObjectLayers::Make(motionType != BodyMotionType::Static, collisionLayer);
}


JPH::BodyID ToBodyID(PhysicsBodyHandle handle)
{
    return JPH::BodyID{handle.value};
}

JPH::RefConst<JPH::Shape> CreateJoltShape(const CollisionShapeDescription& description)
{
    switch (description.type)
    {
    case CollisionShapeType::Box:
    {
        const glm::vec3& halfExtents = description.halfExtentsMeters;
        if (!std::isfinite(halfExtents.x) || !std::isfinite(halfExtents.y) ||
            !std::isfinite(halfExtents.z) || halfExtents.x <= 0.0F ||
            halfExtents.y <= 0.0F || halfExtents.z <= 0.0F)
            throw std::invalid_argument(PhysicsErrorMessages::invalidBoxHalfExtents);

        const float smallestExtent = std::min({halfExtents.x, halfExtents.y, halfExtents.z});
        const float convexRadius = std::min(
            PhysicsWorldSettings::maximumBoxConvexRadiusMeters,
            smallestExtent * PhysicsWorldSettings::boxConvexRadiusExtentRatio);
        JPH::RefConst<JPH::Shape> shape = new JPH::BoxShape(
            JPH::Vec3(halfExtents.x, halfExtents.y, halfExtents.z), convexRadius);
        return shape;
    }
    case CollisionShapeType::ConvexHull:
    {
        if (description.pointsMeters.size() < PhysicsWorldLimits::minimumConvexHullPointCount)
            throw std::invalid_argument(PhysicsErrorMessages::convexHullNeedsMorePoints);

        JPH::Array<JPH::Vec3> points;
        points.reserve(description.pointsMeters.size());
        for (const glm::vec3& point : description.pointsMeters)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                throw std::invalid_argument(PhysicsErrorMessages::nonFiniteConvexHullPoint);
            points.emplace_back(point.x, point.y, point.z);
        }

        JPH::ConvexHullShapeSettings hullSettings(points, PhysicsWorldSettings::convexHullRadiusMeters);
        const auto hullResult = hullSettings.Create();
        if (hullResult.HasError())
            throw std::runtime_error(hullResult.GetError().c_str());
        return hullResult.Get();
    }
    }

    throw std::invalid_argument(PhysicsErrorMessages::unsupportedCollisionShapeType);
}


} // namespace


// 접촉 listener는 Jolt가 충돌을 발견할 때 알려주는 callback 객체다.
// callback은 여러 worker에서 동시에 실행되고 Body는 이미 잠겨 있으므로 World를 조작하거나 BodyInterface로 다시 잠그지 않는다.
// 충돌 모양 쌍을 키로 삼아 현재 접촉만 유지하고, 삭제 callback에서는 Body에 접근하지 않고 저장된 번호만 사용한다.
class ContactCollector final : public JPH::ContactListener
{
public:
    struct Key
    {
        std::uint32_t firstBodyId;
        std::uint32_t firstShapeId;
        std::uint32_t secondBodyId;
        std::uint32_t secondShapeId;

        bool operator<(const Key& other) const
        {
            return std::tie(firstBodyId, firstShapeId, secondBodyId, secondShapeId) <
                std::tie(other.firstBodyId, other.firstShapeId, other.secondBodyId, other.secondShapeId);
        }
    };
    std::mutex mutex;
    std::map<Key, ContactSnapshot> contacts;
    std::map<Key, ContactSnapshot> removed;
    std::map<Key, ContactSnapshot> sleeping;
    std::uint64_t worldToken = 0;

    void Record(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold& manifold)
    {
        const Key key{first.GetID().GetIndexAndSequenceNumber(), manifold.mSubShapeID1.GetValue(),
            second.GetID().GetIndexAndSequenceNumber(), manifold.mSubShapeID2.GetValue()};
        std::lock_guard lock(mutex);
        sleeping.erase(key);
        removed.erase(key);
        contacts[key] = {{key.firstBodyId, worldToken}, {key.secondBodyId, worldToken},
            ToGlmPosition(JPH::RVec3(manifold.mWorldSpaceNormal)), manifold.mPenetrationDepth,
            ToGlmPosition(manifold.GetWorldSpaceContactPointOn1(0))};
    }

    void OnContactAdded(const JPH::Body& first, const JPH::Body& second,
        const JPH::ContactManifold& manifold, JPH::ContactSettings&) override
    {
        Record(first, second, manifold);
    }

    void OnContactPersisted(const JPH::Body& first, const JPH::Body& second,
        const JPH::ContactManifold& manifold, JPH::ContactSettings&) override
    {
        Record(first, second, manifold);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair& pair) override
    {
        std::lock_guard lock(mutex);
        const Key key{pair.GetBody1ID().GetIndexAndSequenceNumber(), pair.GetSubShapeID1().GetValue(),
            pair.GetBody2ID().GetIndexAndSequenceNumber(), pair.GetSubShapeID2().GetValue()};
        if (const auto entry = contacts.find(key); entry != contacts.end())
        {
            removed[key] = entry->second;
            contacts.erase(entry);
        }
    }

    void RemoveBody(std::uint32_t id)
    {
        std::lock_guard lock(mutex);
        auto removeBodyContacts = [id](auto& entries)
        {
            for (auto entry = entries.begin(); entry != entries.end();)
            {
                const bool belongsToBody = entry->first.firstBodyId == id ||
                    entry->first.secondBodyId == id;
                if (belongsToBody)
                    entry = entries.erase(entry);
                else
                    ++entry;
            }
        };
        removeBodyContacts(contacts);
        removeBodyContacts(removed);
        removeBodyContacts(sleeping);
    }
};

struct PhysicsWorld::Impl
{
    // Jolt 등록은 모든 필터와 PhysicsSystem보다 늦게 해제되도록 첫 번째 멤버가 소유한다.
    JoltRuntimeLease runtimeLease;

    // PhysicsSystem은 아래 필터 객체를 참조만 한다. C++은 멤버를 선언한 역순으로 파괴하므로 필터를 먼저 선언해 PhysicsSystem보다 나중에 해제한다.
    BroadPhaseLayerInterfaceImpl broadPhaseLayerInterface;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhaseLayerFilter;
    ObjectLayerPairFilterImpl objectLayerPairFilter;

    // listener는 PhysicsSystem이 빌려 쓰므로 먼저 선언해 PhysicsSystem보다 나중에 파괴한다.
    ContactCollector contactCollector;

    JPH::PhysicsSystem physicsSystem;

    JPH::TempAllocatorImpl tempAllocator;

    std::unique_ptr<JPH::JobSystem> jobSystem;
    const std::uint64_t worldToken = g_NextWorldToken.fetch_add(1, std::memory_order_relaxed);

    struct FixedBinding
    {
        PhysicsBodyHandle anchor;
        PhysicsBodyHandle object;
        JPH::Ref<JPH::TwoBodyConstraint> constraint;
    };
    std::unordered_map<std::uint64_t, FixedBinding> constraints;
    std::uint64_t nextConstraint = PhysicsWorldIdentity::firstConstraintId;

    bool IsBodyValid(PhysicsBodyHandle handle) const
    {
        return handle.IsValid() && handle.worldToken == worldToken &&
            physicsSystem.GetBodyInterface().IsAdded(ToBodyID(handle));
    }

    bool AreBodiesAsleep(const ContactSnapshot& contact) const
    {
        if (!IsBodyValid(contact.first) || !IsBodyValid(contact.second))
            return false;

        const auto& bodies = physicsSystem.GetBodyInterface();
        return !bodies.IsActive(ToBodyID(contact.first)) &&
            !bodies.IsActive(ToBodyID(contact.second));
    }

    JPH::BodyID RequireBodyID(PhysicsBodyHandle handle) const
    {
        if (!IsBodyValid(handle))
            throw std::invalid_argument(PhysicsErrorMessages::invalidPhysicsBodyHandle);
        return ToBodyID(handle);
    }

    void PrepareSleepingContactsForStep()
    {
        // 잠든 물체의 접촉은 유지한다. 물체가 깨어났다면 새 충돌 계산 결과를 기다리도록 이전 접촉을 지운다.
        std::lock_guard lock(contactCollector.mutex);
        for (auto entry = contactCollector.sleeping.begin();
            entry != contactCollector.sleeping.end();)
        {
            if (AreBodiesAsleep(entry->second))
            {
                ++entry;
                continue;
            }

            contactCollector.contacts.erase(entry->first);
            entry = contactCollector.sleeping.erase(entry);
        }
    }

    void PreserveSleepingContactsAfterStep()
    {
        // 삭제 callback에서는 Body에 접근할 수 없다. worker가 끝난 뒤에만 수면 상태를 확인한다.
        std::lock_guard lock(contactCollector.mutex);
        for (const auto& entry : contactCollector.removed)
        {
            if (!AreBodiesAsleep(entry.second))
                continue;

            contactCollector.sleeping[entry.first] = entry.second;
            contactCollector.contacts[entry.first] = entry.second;
        }
        contactCollector.removed.clear();
    }

    Impl()
        : tempAllocator(PhysicsWorldLimits::temporaryAllocatorSizeBytes)
    {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
        jobSystem = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
#else
        const unsigned int hardwareThreads =
            std::thread::hardware_concurrency();

        const int workerThreads = hardwareThreads > PhysicsWorldLimits::reservedWorkerThreads
            ? static_cast<int>(std::min(
                hardwareThreads - PhysicsWorldLimits::reservedWorkerThreads,
                PhysicsWorldLimits::maximumWorkerThreads))
            : static_cast<int>(PhysicsWorldLimits::reservedWorkerThreads);

        // Jolt Update가 물리 계산을 나눠 실행할 작업자 pool이다. 메인 스레드와 별도로 최대 4개의 worker를 만든다.
        jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs,
            JPH::cMaxPhysicsBarriers,
            workerThreads);
#endif

        physicsSystem.Init(
            PhysicsWorldLimits::maximumBodies,
            PhysicsWorldLimits::bodyMutexCount,
            PhysicsWorldLimits::maximumBodyPairs,
            PhysicsWorldLimits::maximumContactConstraints,
            broadPhaseLayerInterface,
            objectVsBroadPhaseLayerFilter,
            objectLayerPairFilter);

        contactCollector.worldToken = worldToken;
        physicsSystem.SetContactListener(&contactCollector);

        // 좌표: World의 +Y가 위쪽이다. 중력 벡터의 단위는 m/s²이며 각 Body의 가속도로 적용된다.
        physicsSystem.SetGravity(JPH::Vec3(
            0.0F, PhysicsWorldSettings::gravityYMetersPerSecondSquared, 0.0F));

        JPH::PhysicsSettings settings = physicsSystem.GetPhysicsSettings();
        settings.mPenetrationSlop = PhysicsWorldSettings::penetrationSlopMeters;
        physicsSystem.SetPhysicsSettings(settings);
    }

    ~Impl()
    {
        // constraint는 Body 참조를 보관하므로 PhysicsSystem과 Body가 살아 있을 때 먼저 제거한다.
        for (const auto& entry : constraints)
            physicsSystem.RemoveConstraint(entry.second.constraint.GetPtr());
        constraints.clear();
        physicsSystem.SetContactListener(nullptr);
    }
};


PhysicsWorld::PhysicsWorld()
{
    impl_ = std::make_unique<Impl>();
}

PhysicsWorld::~PhysicsWorld() = default;


void PhysicsWorld::Step(double fixedDeltaSeconds)
{
    if (!IsValidFixedDeltaSeconds(fixedDeltaSeconds))
    {
        return;
    }

    // 간격이 0 이하이거나 유한하지 않으면 물리 상태를 바꾸지 않는다. 호출자는 Fixed Update의 양수 시간 간격을 전달해야 한다.
    // 이 호출은 시간을 더 잘게 나누지 않고 Jolt 계산을 한 번 수행한다. Jolt 내부의 병렬 작업은 jobSystem이 맡는다.
    impl_->PrepareSleepingContactsForStep();

    impl_->physicsSystem.Update(
        static_cast<float>(fixedDeltaSeconds),
        PhysicsWorldLimits::collisionStepsPerUpdate,
        &impl_->tempAllocator,
        impl_->jobSystem.get());

    impl_->PreserveSleepingContactsAfterStep();
}


PhysicsBodyHandle PhysicsWorld::CreateBox(const BoxBodyDescription& description)
{
    CollisionShapeDescription shape;
    shape.halfExtentsMeters = description.halfExtentsMeters;

    BodyDescription body;
    body.shapes.push_back(shape);
    body.transform = description.transform;
    body.motionType = description.motionType;
    body.collisionLayer = description.collisionLayer;
    return CreateBody(body);
}

void ValidateTransform(const Transform& transform)
{
    // NaN이나 무한대가 포함된 위치, 회전을 나타내지 못하는 영 quaternion은 Jolt 계산을 망가뜨릴 수 있어 입력 경계에서 거부한다.
    // 유한한 회전 quaternion은 호출자가 이 지점에 오기 전에 길이 1로 정규화한다. 여기서는 회전 방향이 정의되지 않는 길이 0만 거부한다.
    const float rotationLengthSquared = glm::dot(transform.rotation, transform.rotation);
    if (!std::isfinite(transform.position.x) || !std::isfinite(transform.position.y) ||
        !std::isfinite(transform.position.z) || !std::isfinite(rotationLengthSquared) ||
        rotationLengthSquared <= PhysicsWorldSettings::minimumRotationLengthSquared)
        throw std::invalid_argument(PhysicsErrorMessages::invalidTransform);
}

PhysicsBodyHandle PhysicsWorld::CreateBody(const BodyDescription& description)
{
    if (description.shapes.empty())
        throw std::invalid_argument(PhysicsErrorMessages::bodyNeedsCollisionShape);
    ValidateTransform(description.transform);
    if (description.collisionLayer >= CollisionLayer::Count)
        throw std::invalid_argument(PhysicsErrorMessages::unsupportedCollisionLayer);

    JPH::MutableCompoundShapeSettings compoundSettings;
    for (const CollisionShapeDescription& part : description.shapes)
    {
        ValidateTransform(part.localTransform);
        const JPH::RefConst<JPH::Shape> shape = CreateJoltShape(part);

        // 형상 치수와 정점은 미터 단위다. GLM의 X/Y/Z 값을 같은 축의 Jolt 좌표로 전달하고, 각 형상은 모델 Body 원점에 대해 배치한다.
        compoundSettings.AddShape(ToJoltPosition(part.localTransform.position),
            ToJoltRotation(glm::normalize(part.localTransform.rotation)), shape.GetPtr());
    }

    const auto shapeResult = compoundSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());

    const JPH::EMotionType motionType = ToJoltMotionType(description.motionType);
    const JPH::ObjectLayer objectLayer = ToObjectLayer(description.motionType, description.collisionLayer);
    const glm::quat rotation = glm::normalize(description.transform.rotation);
    // 입력 자세는 모델에서 정한 Body 원점이다. 여러 형상의 배치가 비대칭이라 무게중심(COM)이 달라져도 Jolt가 내부 좌표 변환을 맡는다.
    JPH::BodyCreationSettings settings(shapeResult.Get(),
        ToJoltPosition(description.transform.position), ToJoltRotation(rotation),
        motionType, objectLayer);
    JPH::BodyInterface& bodyInterface = impl_->physicsSystem.GetBodyInterface();
    const JPH::EActivation activation = description.motionType == BodyMotionType::Static
        ? JPH::EActivation::DontActivate
        : JPH::EActivation::Activate;
    const JPH::BodyID bodyID = bodyInterface.CreateAndAddBody(settings, activation);
    if (bodyID.IsInvalid())
        throw std::runtime_error(PhysicsErrorMessages::failedToCreatePhysicsBody);

    return {bodyID.GetIndexAndSequenceNumber(), impl_->worldToken};
}


bool PhysicsWorld::IsBodyValid(PhysicsBodyHandle handle) const
{
    return impl_->IsBodyValid(handle);
}

BodyMotionType PhysicsWorld::GetBodyMotionType(PhysicsBodyHandle handle) const
{
    switch (impl_->physicsSystem.GetBodyInterface().GetMotionType(impl_->RequireBodyID(handle)))
    {
    case JPH::EMotionType::Static: return BodyMotionType::Static;
    case JPH::EMotionType::Kinematic: return BodyMotionType::Kinematic;
    case JPH::EMotionType::Dynamic: return BodyMotionType::Dynamic;
    }
    throw std::logic_error(PhysicsErrorMessages::unsupportedJoltMotionType);
}

CollisionLayer PhysicsWorld::GetCollisionLayer(PhysicsBodyHandle handle) const
{
    const JPH::ObjectLayer layer = impl_->physicsSystem.GetBodyInterface().GetObjectLayer(impl_->RequireBodyID(handle));
    return ObjectLayers::Category(layer);
}

std::vector<ContactSnapshot> PhysicsWorld::GetContacts() const
{
    std::vector<ContactSnapshot> result;
    std::lock_guard lock(impl_->contactCollector.mutex);
    for (const auto& entry : impl_->contactCollector.contacts)
        if (IsBodyValid(entry.second.first) && IsBodyValid(entry.second.second))
            result.push_back(entry.second);
    return result;
}

PhysicsConstraintHandle PhysicsWorld::CreateFixedConstraint(PhysicsBodyHandle anchor, PhysicsBodyHandle object)
{
    auto& bodyInterface = impl_->physicsSystem.GetBodyInterface();
    const JPH::BodyID anchorID = impl_->RequireBodyID(anchor);
    const JPH::BodyID objectID = impl_->RequireBodyID(object);
    if (bodyInterface.GetMotionType(anchorID) != JPH::EMotionType::Kinematic ||
        bodyInterface.GetMotionType(objectID) != JPH::EMotionType::Dynamic)
        throw std::invalid_argument(PhysicsErrorMessages::fixedGraspNeedsKinematicAndDynamic);

    JPH::Ref<JPH::TwoBodyConstraint> constraint;
    {
        const JPH::BodyID ids[]{anchorID, objectID};
        JPH::BodyLockMultiWrite lock(
            impl_->physicsSystem.GetBodyLockInterface(), ids, PhysicsWorldLimits::bodyPairCount);
        JPH::Body* first = lock.GetBody(0);
        JPH::Body* second = lock.GetBody(1);
        if (!first || !second)
            throw std::invalid_argument(PhysicsErrorMessages::fixedGraspBodyMissing);

        // WorldSpace는 장면 전체 좌표를 뜻한다. 양쪽 연결점을 물체의 현재 무게중심(COM) 위치에 같게 두면 생성 순간 물체가 뛰지 않는다.
        // 양쪽 기준 축도 같은 World 방향으로 두면 각 Body의 현재 회전 차이가 그대로 보존된다.
        // Jolt가 World 연결점을 각 Body COM 기준 좌표로 바꾸므로 모델 원점과 COM의 차이를 호출자가 더하지 않는다.
        JPH::FixedConstraintSettings settings;
        settings.mPoint1 = second->GetCenterOfMassPosition();
        settings.mPoint2 = settings.mPoint1;
        constraint = settings.Create(*first, *second);
    }
    const std::uint64_t id = impl_->nextConstraint++;
    impl_->constraints.emplace(id, Impl::FixedBinding{anchor, object, constraint});
    impl_->physicsSystem.AddConstraint(constraint.GetPtr());
    bodyInterface.ActivateBody(objectID);
    return {id, impl_->worldToken};
}

bool PhysicsWorld::IsConstraintValid(PhysicsConstraintHandle handle) const
{
    return handle.IsValid() && handle.worldToken == impl_->worldToken && impl_->constraints.find(handle.value) != impl_->constraints.end();
}

void PhysicsWorld::DestroyConstraint(PhysicsConstraintHandle handle)
{
    const auto entry = impl_->constraints.find(handle.value);
    if (!handle.IsValid() || handle.worldToken != impl_->worldToken || entry == impl_->constraints.end())
        return;
    const PhysicsBodyHandle object = entry->second.object;
    impl_->physicsSystem.RemoveConstraint(entry->second.constraint.GetPtr());
    impl_->constraints.erase(entry);
    // 해제 후에는 Dynamic 물체가 기존 속도와 중력을 이어받는다. 잠든 물체도 다음 Step에서 떨어질 수 있도록 깨운다.
    if (IsBodyValid(object))
        impl_->physicsSystem.GetBodyInterface().ActivateBody(ToBodyID(object));
}


Transform PhysicsWorld::GetBodyTransform(
    PhysicsBodyHandle handle) const
{
    const JPH::BodyID bodyID = impl_->RequireBodyID(handle);

    JPH::RVec3 position;
    JPH::Quat rotation;

    // 이 API는 Jolt가 계산에 쓰는 무게중심(COM) 자세 대신, 모델과 Entity가 공유하는 Body 원점의 World 자세를 반환한다.
    impl_->physicsSystem
        .GetBodyInterface()
        .GetPositionAndRotation(bodyID, position, rotation);

    return {ToGlmPosition(position), ToGlmRotation(rotation)};
}


bool PhysicsWorld::OverlapsEnvironmentAt(
    PhysicsBodyHandle handle,
    const Transform& targetTransform) const
{
    return OverlapsEnvironmentAtImpl(handle, targetTransform, nullptr);
}

bool PhysicsWorld::OverlapsEnvironmentAt(
    PhysicsBodyHandle handle,
    const Transform& targetTransform,
    PhysicsBodyHandle ignoredEnvironmentBody) const
{
    if (!IsBodyValid(ignoredEnvironmentBody))
        throw std::invalid_argument(PhysicsErrorMessages::invalidIgnoredEnvironmentBody);
    if (GetCollisionLayer(ignoredEnvironmentBody) != CollisionLayer::Environment)
        throw std::invalid_argument(PhysicsErrorMessages::ignoredBodyMustBeEnvironment);

    return OverlapsEnvironmentAtImpl(handle, targetTransform, &ignoredEnvironmentBody);
}

bool PhysicsWorld::OverlapsEnvironmentAtImpl(
    PhysicsBodyHandle handle,
    const Transform& targetTransform,
    const PhysicsBodyHandle* ignoredEnvironmentBody) const
{
    ZoneScopedN("JoltEnvironmentOverlap");
    const JPH::BodyID bodyID = impl_->RequireBodyID(handle);
    ValidateTransform(targetTransform);

    const JPH::Shape* shape = nullptr;
    {
        JPH::BodyLockRead lock(impl_->physicsSystem.GetBodyLockInterface(), bodyID);
        if (!lock.Succeeded())
            return false;
        shape = lock.GetBody().GetShape();
    }
    const JPH::Quat rotation = ToJoltRotation(glm::normalize(targetTransform.rotation));
    // 공개 자세는 Body 원점 기준이고 Jolt 형상은 무게중심(COM) 기준으로 검사한다. 비대칭 그리퍼 형상에서도 두 기준점 차이를 보정한다.
    const JPH::RVec3 centerOfMass = ToJoltPosition(targetTransform.position) + rotation * shape->GetCenterOfMass();
    const JPH::RMat44 centerOfMassTransform = JPH::RMat44::sRotationTranslation(rotation, centerOfMass);
    JPH::CollideShapeSettings settings;
    // 겹침 여부만 필요하므로 첫 접촉에서 끝내고 모든 접촉 지점을 저장하지 않는다.
    JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
    const ObjectLayers::EnvironmentLayerFilter environmentOnly;
    const JPH::BodyID ignoredBodyId = ignoredEnvironmentBody
        ? ToBodyID(*ignoredEnvironmentBody)
        : JPH::BodyID{};
    const JPH::IgnoreSingleBodyFilter bodyFilter(ignoredBodyId);
    impl_->physicsSystem.GetNarrowPhaseQuery().CollideShape(
        shape,
        JPH::Vec3::sOne(),
        centerOfMassTransform,
        settings,
        centerOfMass,
        collector,
        {},
        environmentOnly,
        bodyFilter);
    return collector.HadHit();
}

bool PhysicsWorld::OverlapsBodiesAt(
    PhysicsBodyHandle first,
    const Transform& firstTransform,
    PhysicsBodyHandle second,
    const Transform& secondTransform) const
{
    ZoneScopedN("JoltPairShapeOverlap");
    const JPH::BodyID firstId = impl_->RequireBodyID(first);
    const JPH::BodyID secondId = impl_->RequireBodyID(second);
    if (firstId == secondId)
        return false;
    ValidateTransform(firstTransform);
    ValidateTransform(secondTransform);

    const JPH::BodyID bodyIds[]{firstId, secondId};
    JPH::BodyLockMultiRead lock(
        impl_->physicsSystem.GetBodyLockInterface(), bodyIds, PhysicsWorldLimits::bodyPairCount);
    const JPH::Body* firstBody = lock.GetBody(0);
    const JPH::Body* secondBody = lock.GetBody(1);
    if (firstBody == nullptr || secondBody == nullptr)
        return false;

    const JPH::Shape* firstShape = firstBody->GetShape();
    const JPH::Shape* secondShape = secondBody->GetShape();
    const JPH::Quat firstRotation = ToJoltRotation(glm::normalize(firstTransform.rotation));
    const JPH::Quat secondRotation = ToJoltRotation(glm::normalize(secondTransform.rotation));
    const JPH::RVec3 firstPosition = ToJoltPosition(firstTransform.position);
    const JPH::RVec3 secondPosition = ToJoltPosition(secondTransform.position);
    const JPH::RVec3 firstCenterOfMass = firstPosition + firstRotation * firstShape->GetCenterOfMass();
    const JPH::RVec3 secondCenterOfMass = secondPosition + secondRotation * secondShape->GetCenterOfMass();

    // Jolt의 기하 검사는 Body 쌍 필터를 거치지 않고 두 shape만 직접 비교하므로, 일반 물리 접촉에서 제외한 로봇 링크도 후보 자세 기준으로 검사할 수 있다.
    const JPH::TransformedShape firstAtTarget{
        firstCenterOfMass, firstRotation, firstShape, firstId};
    const JPH::RMat44 secondAtTarget = JPH::RMat44::sRotationTranslation(
        secondRotation, secondCenterOfMass);
    JPH::CollideShapeSettings settings;
    JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
    firstAtTarget.CollideShape(
        secondShape,
        JPH::Vec3::sOne(),
        secondAtTarget,
        settings,
        firstCenterOfMass,
        collector);
    return collector.HadHit();
}


void PhysicsWorld::SetBodyTransform(
    PhysicsBodyHandle handle,
    const Transform& transform)
{
    const JPH::BodyID bodyID = impl_->RequireBodyID(handle);
    ValidateTransform(transform);

    // 순간이동은 경로 접촉을 계산하지 않는다. 잠든 이전 표면의 접촉도 즉시 버려 새 자세의 충돌 검사 결과만 남긴다.
    impl_->contactCollector.RemoveBody(handle.value);

    // 모델 Body 원점의 World 자세를 그대로 전달한다. 무게중심(COM) 차이를 여기서 다시 더하면 Jolt의 내부 보정과 겹쳐 위치가 어긋난다.
    impl_->physicsSystem.GetBodyInterface().SetPositionAndRotation(
        bodyID, ToJoltPosition(transform.position),
        ToJoltRotation(glm::normalize(transform.rotation)), JPH::EActivation::Activate);
    if (impl_->physicsSystem.GetBodyInterface().GetMotionType(bodyID) == JPH::EMotionType::Dynamic)
    {
        impl_->physicsSystem.GetBodyInterface().SetLinearAndAngularVelocity(
            bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
    }
}


void PhysicsWorld::MoveKinematic(
    PhysicsBodyHandle handle,
    const Transform& targetTransform,
    double fixedDeltaSeconds)
{
    const JPH::BodyID bodyID = impl_->RequireBodyID(handle);
    ValidateTransform(targetTransform);

    if (!IsValidFixedDeltaSeconds(fixedDeltaSeconds))
        throw std::invalid_argument(PhysicsErrorMessages::invalidFixedDeltaSeconds);

    auto& bodyInterface = impl_->physicsSystem.GetBodyInterface();
    const JPH::EMotionType motionType = bodyInterface.GetMotionType(bodyID);

    if (motionType != JPH::EMotionType::Kinematic)
    {
        throw std::logic_error(PhysicsErrorMessages::moveKinematicRequiresKinematicBody);
    }

    // 목표 자세 역시 모델 Body 원점 기준이다. Jolt가 무게중심(COM) 이동을 내부에서 처리하고 목표와 시간 간격으로 필요한 속도를 계산한다.
    bodyInterface.MoveKinematic(
        bodyID, ToJoltPosition(targetTransform.position),
        ToJoltRotation(glm::normalize(targetTransform.rotation)),
        static_cast<float>(fixedDeltaSeconds));
}


void PhysicsWorld::StopKinematic(PhysicsBodyHandle handle)
{
    const JPH::BodyID bodyID = impl_->RequireBodyID(handle);
    auto& bodyInterface = impl_->physicsSystem.GetBodyInterface();
    if (bodyInterface.GetMotionType(bodyID) != JPH::EMotionType::Kinematic)
        throw std::logic_error(PhysicsErrorMessages::kinematicStopRequiresKinematicBody);
    bodyInterface.SetLinearAndAngularVelocity(bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
}

void PhysicsWorld::DestroyBody(PhysicsBodyHandle handle)
{
    if (!IsBodyValid(handle))
        return;

    const JPH::BodyID bodyID = ToBodyID(handle);

    // 연결이 삭제된 Body 메모리를 참조하지 않도록 그 Body에 붙은 모든 constraint를 먼저 제거한다.
    std::vector<PhysicsConstraintHandle> attached;
    for (const auto& entry : impl_->constraints)
        if (entry.second.anchor.value == handle.value || entry.second.object.value == handle.value)
            attached.push_back({entry.first, impl_->worldToken});
    for (const auto constraint : attached)
        DestroyConstraint(constraint);
    impl_->contactCollector.RemoveBody(handle.value);

    JPH::BodyInterface& bodyInterface =
        impl_->physicsSystem.GetBodyInterface();

    // 먼저 Jolt 계산 목록에서 Body를 제거해 다음 Step에 참여하지 않게 한다. 이후 ID와 Body 메모리를 해제한다.
    bodyInterface.RemoveBody(bodyID);

    bodyInterface.DestroyBody(bodyID);
}

} // namespace grasplink::physics
