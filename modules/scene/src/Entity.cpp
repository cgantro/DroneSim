#include "scene/Entity.h"

#include "scene/TransformComponents.h"
#include <stdexcept>

namespace grasplink::scene
{


Entity::Entity(flecs::entity handle)
    : m_EntityHandle(handle)
{
}


// ============================================================================
// 변환
// ============================================================================

glm::vec3 Entity::GetLocalPosition() const
{
    // Entity는 장면 물체 자체가 아니라 그 물체 ID를 빌려 보관한다. 대상이 삭제된 뒤에는 Flecs 저장소를 읽지 않고 기본 위치를 반환한다.
    if (!IsValid() || !m_EntityHandle.has<Position, Local>())
        return glm::vec3(0.0f);

    return m_EntityHandle.get<Position, Local>();
}


glm::quat Entity::GetLocalRotation() const
{
    if (!IsValid() || !m_EntityHandle.has<Rotation, Local>())
        return glm::quat(1.0F, 0.0F, 0.0F, 0.0F);

    return m_EntityHandle.get<Rotation, Local>();
}


glm::vec3 Entity::GetLocalScale() const
{
    if (!IsValid() || !m_EntityHandle.has<Scale, Local>())
        return glm::vec3(1.0f);

    return m_EntityHandle.get<Scale, Local>();
}


void Entity::SetLocalPosition(const glm::vec3& position)
{
    if (!IsValid())
        return;

    // Local은 바로 위 부모 기준 값이고 World는 부모와 조상까지 더한 Scene 기준 값이다. 여기서는 새 위치만 저장하고 행렬은 다음 TransformSystemModule 호출에서 계산한다.
    m_EntityHandle.set<Position, Local>(
        Position{position}
    );
}


void Entity::SetLocalRotation(const glm::quat& rotation)
{
    if (!IsValid())
        return;

    m_EntityHandle.set<Rotation, Local>(
        Rotation{rotation}
    );
}


void Entity::SetLocalScale(const glm::vec3& scale)
{
    if (!IsValid())
        return;

    m_EntityHandle.set<Scale, Local>(
        Scale{scale}
    );
}


glm::mat4 Entity::GetWorldMatrix() const
{
    // 이 World 행렬은 저장된 Local 값과 부모 행렬에서 마지막으로 계산한 결과다. 위치를 바꾼 직후의 행렬이 필요하면 호출자가 TransformSystemModule을 실행해야 한다.
    if (!IsValid() || !m_EntityHandle.has<TransformMatrix, World>())
        return glm::mat4(1.0f);

    return m_EntityHandle.get<TransformMatrix, World>();
}


// ============================================================================
// 계층
// ============================================================================

Entity& Entity::SetParent(const Entity& parent)
{
    if (!IsValid() || !parent.IsValid())
        return *this;

    // Flecs World는 Entity와 그 값을 소유하는 저장소다. 서로 다른 저장소의 물체는 같은 부모-자식 관계에 연결할 수 없다.
    if (m_EntityHandle.world().c_ptr() != parent.GetHandle().world().c_ptr())
        throw std::invalid_argument("Entity::SetParent requires the same Flecs World");
    // SceneRootTag는 이 물체 묶음을 정리할 Scene의 경계를 표시한다. 중간 grouping Entity는 소유 경계가 아니므로 건너뛰고 Scene에 속한 물체가 다른 Scene이나 Scene 밖으로 이동하지 않게 한다.
    auto sceneRoot = [](flecs::entity handle)
    {
        while (handle.id() != 0 && handle.is_alive())
        {
            if (handle.has<SceneRootTag>()) return handle;
            handle = handle.parent();
        }
        return flecs::entity::null();
    };
    const flecs::entity owner = sceneRoot(m_EntityHandle);
    if (owner.id() != 0 && owner != sceneRoot(parent.GetHandle()))
        throw std::invalid_argument("Entity::SetParent cannot leave the owning Scene");
    // 새 부모의 위쪽 조상에 현재 물체가 있으면 자식이 다시 조상이 되는 순환이 생긴다. 부모 행렬을 따라 내려갈 수 없는 구조이므로 연결을 거부한다.
    for (flecs::entity ancestor = parent.GetHandle(); ancestor.id() != 0 && ancestor.is_alive(); ancestor = ancestor.parent())
        if (ancestor == m_EntityHandle)
            throw std::invalid_argument("Entity::SetParent cannot create a hierarchy cycle");

    // 부모를 바꿔도 저장된 Local 위치·회전·크기는 새 부모 기준으로 다시 해석된다. Scene에서 보이는 자세를 유지하려면 새 부모 기준 값을 호출자가 다시 지정해야 한다.
    m_EntityHandle.child_of(
        parent.GetHandle()
    );

    return *this;
}


Entity& Entity::AddChild(const Entity& child)
{
    // 부모 설정 한 곳에서 World 소속, Scene 경계, 순환 여부를 검사해 자식을 추가할 때도 같은 규칙을 지킨다.
    Entity childHandle = child;
    childHandle.SetParent(*this);

    return *this;
}


Entity Entity::GetParent() const
{
    if (!IsValid())
        return Entity{};

    return Entity{
        m_EntityHandle.parent()
    };
}


std::vector<Entity> Entity::GetChildren() const
{
    std::vector<Entity> children;

    if (!IsValid())
        return children;

    m_EntityHandle.children(
        [&](flecs::entity child)
        {
            children.emplace_back(child);
        }
    );

    return children;
}


Entity Entity::GetChild(const std::string& name) const
{
    if (!IsValid())
        return Entity{};

    // Flecs 저장소에서 이 물체 아래의 이름 또는 경로를 찾는다. 대상이 없으면 비어 있는 Entity 참조를 반환한다.
    flecs::entity child =
        m_EntityHandle.lookup(
            name.c_str()
        );

    if (!child.is_alive())
        return Entity{};

    return Entity{child};
}


Entity Entity::FindChildByNameRecursive(
    const std::string& targetName
) const
{
    if (!IsValid())
        return Entity{};

    // 이 물체 자신은 검사하지 않고 자식부터 각 하위 가지를 끝까지 따라가며 찾는다. 이름 없는 중간 물체 아래에 있어도 검색한다.
    flecs::entity found =
        flecs::entity::null();


    m_EntityHandle.children(
        [&](flecs::entity child)
        {
            // 첫 번째로 발견한 이름을 반환하므로 나머지 형제 가지는 더 살펴보지 않는다.
            if (found != flecs::entity::null())
                return;

            const char* childName = child.name().c_str();
            if (childName && targetName == childName)
            {
                found = child;
                return;
            }

            Entity childEntity{child};

            Entity result =
                childEntity.FindChildByNameRecursive(
                    targetName
                );

            if (result.IsValid())
            {
                found = result.GetHandle();
            }
        }
    );


    if (found == flecs::entity::null())
        return Entity{};

    return Entity{found};
}


// ============================================================================
// 보조 기능
// ============================================================================

bool Entity::IsValid() const
{
    // 이 참조는 삭제 알림을 받거나 수명을 연장하지 않는다. 호출할 때마다 Flecs World에 물어 대상 물체가 남아 있는지 확인한다.
    return m_EntityHandle.is_alive();
}


void Entity::Destroy()
{
    if (!IsValid())
        return;

    // Flecs는 이 물체와 그 아래 연결된 자손을 저장소에서 함께 삭제한다. 복사된 Entity 참조는 메모리에 남아도 모두 더는 유효하지 않다.
    m_EntityHandle.destruct();
}

}
