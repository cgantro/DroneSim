#include "simulation/systems/PhysicsSystemModule.h"

#include "physics/PhysicsWorld.h"
#include "diagnostics/Logger.h"
#include "simulation/components/PhysicsComponents.h"
#include "scene/TransformComponents.h"

#include <glm/gtx/matrix_decompose.hpp>

#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
using grasplink::physics::BodyDescription;
using grasplink::physics::BodyMotionType;
using grasplink::physics::PhysicsBodyHandle;
using grasplink::physics::Transform;
using grasplink::scene::Local;
using grasplink::scene::Position;
using grasplink::scene::Rotation;
using grasplink::scene::Scale;
using grasplink::scene::TransformMatrix;
using grasplink::scene::World;

struct PhysicsSystemTolerances final
{
    static constexpr float targetTransformComponentDifference = 1.0e-6F;
    static constexpr float parentIdentityDifference = 1.0e-6F;
    static constexpr float unitScaleDifference = 1.0e-4F;
    static constexpr float unitScaleValue = 1.0F;
};

struct PhysicsSystemMessages final
{
    inline static constexpr char bodyConfigurationLogCategory[] = "physics.body_configuration";
    inline static constexpr char unsupportedScaleOrParentHierarchy[] =
        "unsupported scale or parent hierarchy (Dynamic requires identity ancestors)";
    inline static constexpr char worldTransformUnavailable[] =
        "could not calculate Entity World Transform";
    inline static constexpr char bodyCreationFailed[] = "failed to create Body";
};

struct BodySnapshot
{
    flecs::entity entity;
    bool ready = false;
};

bool IsFinite(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool SameTarget(const Transform& left, const Transform& right)
{
    // quaternion q와 -q는 같은 회전을 나타낼 수 있다. 두 회전을 성분별로 비교하면 달라 보일 수 있으므로 목표 변경 판단은 원래 성분 비교와 별도 허용 오차로 처리한다.
    return left.position == right.position &&
        (left.rotation == right.rotation || left.rotation == -right.rotation);
}

bool ReachedTarget(const Transform& actual, const Transform& target)
{
    // Jolt가 float로 적분하면서 생기는 작은 수치 차이는 허용한다. 같은 목표에 도달했는지 보는 기준은 위치 1 μm와 quaternion 성분 1e-6이다.
    constexpr float tolerance = PhysicsSystemTolerances::targetTransformComponentDifference;
    const glm::quat signedActual = glm::dot(actual.rotation, target.rotation) < 0.0F
        ? -actual.rotation : actual.rotation;
    return glm::all(glm::lessThanEqual(glm::abs(actual.position - target.position), glm::vec3{tolerance})) &&
        std::abs(signedActual.w - target.rotation.w) <= tolerance &&
        std::abs(signedActual.x - target.rotation.x) <= tolerance &&
        std::abs(signedActual.y - target.rotation.y) <= tolerance &&
        std::abs(signedActual.z - target.rotation.z) <= tolerance;
}

bool AllowsVisualScale(const RigidBody& rigidBody)
{
    // 바닥 그림 크기를 맞추기 위한 렌더 Mesh scale은 허용한다. 충돌 형상은 그 scale을 곱하지 않고 선언된 미터 치수를 유지한다.
    return rigidBody.motionType == BodyMotionType::Static &&
        rigidBody.collisionLayer == grasplink::physics::CollisionLayer::Environment;
}

bool HasUnitScale(flecs::entity entity, bool allowEntityScale)
{
    // 현재 Entity부터 부모 조상까지 각 Local Scale component를 확인한다. 단순 계층 정리에만 쓰여 Scale component가 없는 Entity는 크기 1인 것으로 취급한다.
    bool isEntity = true;
    while (entity.id() != 0 && entity.is_alive())
    {
        if ((!isEntity || !allowEntityScale) && entity.has<Scale, Local>())
        {
            const glm::vec3 scale = entity.get<Scale, Local>();
            if (!IsFinite(scale) || glm::any(glm::greaterThan(
                glm::abs(scale - glm::vec3{PhysicsSystemTolerances::unitScaleValue}),
                glm::vec3{PhysicsSystemTolerances::unitScaleDifference})))
                return false;
        }

        const flecs::entity parent = entity.parent();
        if (parent.id() == 0 || !parent.is_alive() || parent == entity)
            break;
        entity = parent;
        isEntity = false;
    }
    return true;
}

bool HasIdentityParentPose(flecs::entity parent)
{
    constexpr float tolerance = PhysicsSystemTolerances::parentIdentityDifference;
    if (parent.has<Position, Local>())
    {
        const glm::vec3 position = parent.get<Position, Local>();
        if (!IsFinite(position) || glm::any(glm::greaterThan(glm::abs(position), glm::vec3{tolerance})))
            return false;
    }

    if (!parent.has<Rotation, Local>())
        return true;

    const glm::quat rotation = parent.get<Rotation, Local>();
    const double length = std::hypot(std::hypot(static_cast<double>(rotation.w), rotation.x),
        std::hypot(static_cast<double>(rotation.y), rotation.z));
    return std::isfinite(length) && length > 0.0 &&
        std::abs(rotation.x / length) <= tolerance &&
        std::abs(rotation.y / length) <= tolerance &&
        std::abs(rotation.z / length) <= tolerance;
}

bool HasSupportedPhysicsParents(flecs::entity entity, BodyMotionType motionType)
{
    // Dynamic 부모가 Jolt에서 이동하는 동안 자식 Body도 따로 움직이면 두 이동 결과를 하나의 World 자세로 합치는 규칙이 이 시스템에는 없다.
    for (flecs::entity parent = entity.parent(); parent.id() != 0 && parent.is_alive(); parent = parent.parent())
    {
        if (parent.has<RigidBody>() && grasplink::physics::IsPhysicsDriven(parent.get<RigidBody>().motionType))
            return false;
        // World는 장면 전체 기준이고 Local은 바로 위 부모 기준이다. Dynamic의 World 결과를 Entity Local 값에 기록하려면 각 조상 자세가 항등이어야 한다.
        // 조상 변환끼리 곱해져 우연히 상쇄되는 경우도 허용하지 않는다.
        if (grasplink::physics::IsPhysicsDriven(motionType) && !HasIdentityParentPose(parent))
            return false;
    }
    return true;
}

bool HasSupportedBodyPlacement(flecs::entity entity, const RigidBody& rigidBody)
{
    return HasUnitScale(entity, AllowsVisualScale(rigidBody)) &&
        HasSupportedPhysicsParents(entity, rigidBody.motionType);
}

bool HasRequiredLocalTransform(flecs::entity entity)
{
    return entity.has<Position, Local>() &&
        entity.has<Rotation, Local>() &&
        entity.has<Scale, Local>();
}

bool TryGetPhysicsTransform(
    flecs::entity entity,
    Transform& result)
{
    if (!entity.has<TransformMatrix, World>())
        return false;

    // TransformSystem이 갱신한 World 행렬에서 위치와 회전을 읽는다. 물리 Body의 기준점은 Entity 원점이며 내부 계산용 무게중심(COM)과 다를 수 있다.
    const glm::mat4 worldMatrix = entity.get<TransformMatrix, World>();
    glm::vec3 scale{PhysicsSystemTolerances::unitScaleValue};
    glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 translation{0.0F};
    glm::vec3 skew{0.0F};
    glm::vec4 perspective{0.0F};

    if (!glm::decompose(worldMatrix, scale, rotation, translation, skew, perspective))
        return false;

    result.position = translation;
    result.rotation = glm::normalize(rotation);
    return true;
}
}

namespace grasplink::simulation
{
namespace detail
{
struct PhysicsBodyBinding
{
    // ECS Binding은 Body를 소유하지 않는 핸들만 보관한다. Binding이 제거될 때 observer가 핸들을 받아 PhysicsWorld가 소유한 Body를 해제한다.
    grasplink::physics::PhysicsBodyHandle handle;
    // 직전 Fixed Update에서 Jolt에 보낸 Scene World 목표 자세다. 처음에는 새 Body를 만든 자세로 초기화한다.
    Transform sceneTarget;
    bool kinematicMoving = false;
};
}

struct PhysicsSystemModule::Impl
{
    grasplink::physics::PhysicsWorld& physicsWorld;
    grasplink::diagnostics::Logger* logger;
    flecs::query<const RigidBody, const Colliders> bodyQuery;
    flecs::query<const grasplink::simulation::detail::PhysicsBodyBinding> bindingQuery;
    std::vector<BodySnapshot> bodies;
    // Flecs 설정 변경이 지연 중이어도 모듈이 만든 Jolt Body를 빠짐없이 정리할 수 있도록 핸들을 별도로 추적한다.
    std::unordered_map<std::uint32_t, PhysicsBodyHandle> ownedBodies;
    // Flecs OnSet callback 중 query 구성을 바꾸면 현재 순회가 영향을 받을 수 있어 예약만 한다. 다음 Step에서 변경이 모두 끝난 설정과 최신 World 자세를 읽는다.
    std::vector<flecs::entity> pendingConfiguration;
    std::unordered_set<flecs::entity_t> pendingIds;
    // 등록된 callback은 현재 Impl(this)과 참조한 PhysicsWorld를 사용한다. 두 객체가 해제된 뒤 callback이 남지 않도록 Impl 파괴 중 observer부터 해제한다.
    flecs::observer rigidBodySetObserver;
    flecs::observer rigidBodyRemoveObserver;
    flecs::observer colliderSetObserver;
    flecs::observer colliderRemoveObserver;
    flecs::observer bodyBindingRemoveObserver;

    Impl(flecs::world& worldValue, grasplink::physics::PhysicsWorld& physicsWorldValue,
        grasplink::diagnostics::Logger* loggerValue)
        : physicsWorld(physicsWorldValue), logger(loggerValue),
          bodyQuery(worldValue.query<const RigidBody, const Colliders>()),
          bindingQuery(worldValue.query<const grasplink::simulation::detail::PhysicsBodyBinding>())
    {
        worldValue.module<PhysicsSystemModule>();

        rigidBodySetObserver = worldValue.observer<RigidBody>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const RigidBody&) { QueueConfiguration(entity); });
        rigidBodyRemoveObserver = worldValue.observer<RigidBody>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const RigidBody&) { DestroyBody(entity); });
        colliderSetObserver = worldValue.observer<Colliders>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const Colliders&) { QueueConfiguration(entity); });
        colliderRemoveObserver = worldValue.observer<Colliders>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const Colliders&) { DestroyBody(entity); });
        bodyBindingRemoveObserver = worldValue.observer<detail::PhysicsBodyBinding>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity, const grasplink::simulation::detail::PhysicsBodyBinding& binding)
            {
                DestroyOwnedBody(binding.handle);
            });

        // 생성 전에 붙어 있던 설정도 빠뜨리지 않고 연결한다. query 순회와 Binding 추가를 분리한다.
        bodyQuery.each([this](flecs::entity entity, const RigidBody&, const Colliders&)
        {
            bodies.push_back({entity});
        });
        for (const BodySnapshot& body : bodies)
            ConfigureBody(body.entity);
        bodies.clear();
    }

    ~Impl()
    {
        // Binding을 Query에서 지우기 전에 Entity를 복사한다. 모듈을 없애는 동안 ECS 변경이 지연돼도 Body는 먼저 직접 정리한다.
        std::vector<flecs::entity> entitiesToUnbind;
        bindingQuery.each([&entitiesToUnbind](flecs::entity entity, const detail::PhysicsBodyBinding&)
        {
            entitiesToUnbind.push_back(entity);
        });

        rigidBodySetObserver.destruct();
        rigidBodyRemoveObserver.destruct();
        colliderSetObserver.destruct();
        colliderRemoveObserver.destruct();
        bodyBindingRemoveObserver.destruct();

        for (const auto& owned : ownedBodies)
            physicsWorld.DestroyBody(owned.second);
        ownedBodies.clear();

        for (flecs::entity entity : entitiesToUnbind)
            if (entity.is_alive())
                entity.remove<grasplink::simulation::detail::PhysicsBodyBinding>();
    }

    // 두 Fixed Update 사이에 같은 Entity의 설정이 여러 번 바뀌면 Entity ID로 예약을 합친다. 다음 Step에서는 중간값으로 Body를 반복 생성하지 않고 마지막 설정만 적용한다.
    void QueueConfiguration(flecs::entity entity)
    {
        if (pendingIds.insert(entity.id()).second)
            pendingConfiguration.push_back(entity);
    }

    bool ConfigureBody(flecs::entity entity)
    {
        if (!entity.is_alive())
            return false;

        // 기존 Binding을 먼저 제거해 이전 설정으로 만든 Jolt Body와 자세가 새 설정에 연결된 것처럼 남지 않게 한다.
        DestroyBody(entity);
        if (!entity.has<RigidBody>() || !entity.has<Colliders>())
            return false;
        // RigidBody와 Colliders만으로는 Body의 시작 자세와 크기를 알 수 없다.
        // 해당 Entity 자신의 Local Position, Rotation, Scale component가 모두 있어야 한다.
        if (!HasRequiredLocalTransform(entity))
            return false;

        const RigidBody& rigidBody = entity.get<RigidBody>();
        const Colliders& colliders = entity.get<Colliders>();
        if (colliders.shapes.empty())
            return false;
        if (!HasSupportedBodyPlacement(entity, rigidBody))
        {
            LogBodyIssue(grasplink::diagnostics::LogLevel::Warning,
                PhysicsSystemMessages::unsupportedScaleOrParentHierarchy, entity.id());
            return false;
        }
        Transform bodyTransform;
        if (!TryGetPhysicsTransform(entity, bodyTransform))
        {
            LogBodyIssue(grasplink::diagnostics::LogLevel::Error,
                PhysicsSystemMessages::worldTransformUnavailable, entity.id());
            return false;
        }

        BodyDescription description;
        // ECS component는 선언 데이터다. Scene World pose와 충돌 필터를 합쳐 Jolt 소유 Body를 만든다.
        description.shapes = colliders.shapes;
        description.transform = bodyTransform;
        description.motionType = rigidBody.motionType;
        description.collisionLayer = rigidBody.collisionLayer;

        PhysicsBodyHandle newHandle;
        try
        {
            newHandle = physicsWorld.CreateBody(description);
            ownedBodies.emplace(newHandle.value, newHandle);
            entity.set<grasplink::simulation::detail::PhysicsBodyBinding>({newHandle, bodyTransform, false});
            return true;
        }
        catch (const std::exception& error)
        {
            if (newHandle.IsValid())
                DestroyOwnedBody(newHandle);
            LogBodyIssue(grasplink::diagnostics::LogLevel::Error,
                PhysicsSystemMessages::bodyCreationFailed, entity.id(), error.what());
            return false;
        }
    }

    void LogBodyIssue(grasplink::diagnostics::LogLevel level, const char* message,
        flecs::entity_t entityId, const char* detail = nullptr) noexcept
    {
        if (!logger)
            return;
        try
        {
            std::string entry = "PhysicsSystemModule: ";
            entry += message;
            entry += " for Entity ";
            entry += std::to_string(entityId);
            if (detail)
            {
                entry += ": ";
                entry += detail;
            }
            logger->Write(level, PhysicsSystemMessages::bodyConfigurationLogCategory, entry);
        }
        catch (...)
        {
        }
    }

    void DestroyOwnedBody(PhysicsBodyHandle handle)
    {
        physicsWorld.DestroyBody(handle);
        const auto owned = ownedBodies.find(handle.value);
        if (owned != ownedBodies.end() && owned->second.worldToken == handle.worldToken)
            ownedBodies.erase(owned);
    }

    void DestroyBody(flecs::entity entity)
    {
        if (entity.id() != 0 && entity.is_alive() &&
            entity.has<grasplink::simulation::detail::PhysicsBodyBinding>())
        {
            // OnRemove observer가 handle을 PhysicsWorld에 넘긴다. 여기서 직접 중복 해제하지 않는다.
            entity.remove<grasplink::simulation::detail::PhysicsBodyBinding>();
        }
    }

    bool EnsureBody(flecs::entity entity, bool configurationChanged)
    {
        if (!configurationChanged && entity.has<grasplink::simulation::detail::PhysicsBodyBinding>())
        {
            const auto& binding = entity.get<grasplink::simulation::detail::PhysicsBodyBinding>();
            if (physicsWorld.IsBodyValid(binding.handle))
                return true;
            DestroyBody(entity);
        }

        return ConfigureBody(entity);
    }

    void Step(double fixedDeltaSeconds)
    {
        if (!std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0)
            return;

        PrePhysicsSync(fixedDeltaSeconds);
        physicsWorld.Step(fixedDeltaSeconds);
        PostPhysicsSync();
    }

    void PrePhysicsSync(double fixedDeltaSeconds)
    {
        // 먼저 바뀐 설정으로 기존 Body를 다시 만든 뒤 Scene이 움직임을 정하는 Static/Kinematic 자세를 제출한다. 이 단계에서는 아직 Jolt 물리 계산을 실행하지 않는다.
        std::vector<flecs::entity> changedEntities;
        changedEntities.swap(pendingConfiguration);
        for (flecs::entity entity : changedEntities)
        {
            if (!entity.is_alive() || !entity.has<RigidBody>() || !entity.has<Colliders>())
                DestroyBody(entity);
        }
        // 이 Query는 RigidBody와 Colliders를 모두 가진 Entity를 찾는다.
        // Binding을 바꾸면 조회 대상도 바뀌므로 먼저 Entity 목록을 복사한 뒤 처리한다.
        bodies.clear();
        bodyQuery.each([this](flecs::entity entity, const RigidBody&, const Colliders&)
        {
            bodies.push_back({entity});
        });
        for (BodySnapshot& body : bodies)
        {
            const RigidBody& rigidBody = body.entity.get<RigidBody>();
            if (!HasRequiredLocalTransform(body.entity))
            {
                DestroyBody(body.entity);
                continue;
            }
            if (!HasSupportedBodyPlacement(body.entity, rigidBody))
            {
                DestroyBody(body.entity);
                continue;
            }

            body.ready = EnsureBody(body.entity, pendingIds.erase(body.entity.id()) != 0);
        }
        pendingIds.clear();

        for (const BodySnapshot& body : bodies)
            SyncSceneDrivenBody(body, fixedDeltaSeconds);
    }

    void SyncSceneDrivenBody(const BodySnapshot& body, double fixedDeltaSeconds)
    {
        if (!body.ready)
            return;

        const RigidBody& rigidBody = body.entity.get<RigidBody>();
        if (!grasplink::physics::IsSceneDriven(rigidBody.motionType))
            return;

        auto& binding = body.entity.get_mut<grasplink::simulation::detail::PhysicsBodyBinding>();
        Transform targetTransform;
        if (!TryGetPhysicsTransform(body.entity, targetTransform))
            return;

        if (rigidBody.motionType == BodyMotionType::Static)
            SyncStaticBody(binding, targetTransform);
        else
            SyncKinematicBody(binding, targetTransform, fixedDeltaSeconds);

        binding.sceneTarget = targetTransform;
    }

    void SyncStaticBody(
        const grasplink::simulation::detail::PhysicsBodyBinding& binding,
        const Transform& targetTransform)
    {
        if (!SameTarget(targetTransform, binding.sceneTarget))
            physicsWorld.SetBodyTransform(binding.handle, targetTransform);
    }

    void SyncKinematicBody(
        grasplink::simulation::detail::PhysicsBodyBinding& binding,
        const Transform& targetTransform,
        double fixedDeltaSeconds)
    {
        if (!SameTarget(targetTransform, binding.sceneTarget) ||
            (binding.kinematicMoving &&
                !ReachedTarget(physicsWorld.GetBodyTransform(binding.handle), targetTransform)))
        {
            physicsWorld.MoveKinematic(binding.handle, targetTransform, fixedDeltaSeconds);
            binding.kinematicMoving = true;
            return;
        }

        if (!binding.kinematicMoving)
            return;

        physicsWorld.StopKinematic(binding.handle);
        binding.kinematicMoving = false;
    }

    void PostPhysicsSync()
    {
        // Dynamic Body의 자세는 Jolt 계산 결과가 기준이다. 지원 조건상 모든 조상 변환이 항등이므로 계산된 World 위치와 회전을 Entity Local 값으로 그대로 기록할 수 있다.
        for (const BodySnapshot& body : bodies)
        {
            if (!body.ready || !body.entity.is_alive() ||
                !grasplink::physics::IsPhysicsDriven(body.entity.get<RigidBody>().motionType))
                continue;

            const auto& binding = body.entity.get<grasplink::simulation::detail::PhysicsBodyBinding>();
            // Jolt 결과에서 위치와 회전만 Entity에 반영한다. Local Scale component가 있으면 component와 현재 값 모두 유지한다.
            const Transform bodyTransform = physicsWorld.GetBodyTransform(binding.handle);
            body.entity.set<Position, Local>(Position{bodyTransform.position});
            body.entity.set<Rotation, Local>(Rotation{bodyTransform.rotation});
        }
    }
};

PhysicsSystemModule::PhysicsSystemModule(
    flecs::world& world,
    grasplink::physics::PhysicsWorld& physicsWorld,
    grasplink::diagnostics::Logger* logger)
    : m_Impl(std::make_unique<Impl>(world, physicsWorld, logger))
{
}

PhysicsSystemModule::~PhysicsSystemModule() = default;

void PhysicsSystemModule::Step(double fixedDeltaSeconds)
{
    m_Impl->Step(fixedDeltaSeconds);
}

grasplink::physics::PhysicsBodyHandle PhysicsSystemModule::GetBodyHandle(flecs::entity entity) const
{
    if (entity.id() == 0 || !entity.is_alive() || !entity.has<detail::PhysicsBodyBinding>())
        return {};
    const auto handle = entity.get<detail::PhysicsBodyBinding>().handle;
    return m_Impl->physicsWorld.IsBodyValid(handle) ? handle : grasplink::physics::PhysicsBodyHandle{};
}

}
