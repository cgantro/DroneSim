#include "scene/TransformSystemModule.h"

#include "scene/TransformComponents.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <unordered_map>
#include <vector>

namespace grasplink::scene
{

namespace
{
using WorldTransformCache = std::unordered_map<flecs::entity_t, glm::mat4>;

struct TransformSystemScratch final
{
    std::vector<flecs::entity> transformedEntities;
    WorldTransformCache worldTransforms;
};

glm::mat4 UpdateEntityWorldTransform(flecs::entity entity, WorldTransformCache& cache)
{
    if (!entity.is_alive())
        return glm::mat4(1.0F);

    const flecs::entity_t id = entity.id();
    if (const auto cached = cache.find(id); cached != cache.end())
        return cached->second;

    const bool hasTransform = entity.has<Position, Local>() &&
        entity.has<Rotation, Local>() && entity.has<Scale, Local>();
    // TRS는 위치, 회전, 크기 입력을 뜻한다. 이름만 있는 grouping 부모는 이 값이 모두 없으므로 위치를 바꾸지 않고, 일부 값만 있는 부모도 불완전한 자세로 처리하지 않는다.
    const glm::mat4 local = hasTransform ? TransformSystemModule::ComposeLocalMatrix(
        entity.get<Position, Local>(), entity.get<Rotation, Local>(), entity.get<Scale, Local>())
        : glm::mat4(1.0F);
    if (hasTransform)
        entity.set<TransformMatrix, Local>(TransformMatrix{local});

    // 자식 위치는 부모에서 잰 Local 값이다. 부모의 누적 행렬을 먼저 적용해야 자식 점이 로봇 root 배치까지 포함한 Scene 위치로 옮겨진다.
    const flecs::entity parent = entity.parent();
    const glm::mat4 parentWorld = parent.id() != 0 && parent.is_alive() && parent != entity
        ? UpdateEntityWorldTransform(parent, cache)
        : glm::mat4(1.0F);
    const glm::mat4 world = parentWorld * local;
    // 중간 grouping 부모는 자기 위치를 더하지 않지만 조상의 행렬은 자식이 Scene 위치를 얻는 데 필요하므로 그대로 저장한다.
    entity.set<TransformMatrix, World>(TransformMatrix{world});
    cache.emplace(id, world);
    return world;
}
}

glm::mat4 TransformSystemModule::ComposeLocalMatrix(
    const glm::vec3& position,
    const glm::quat& rotation,
    const glm::vec3& scale)
{
    // 점에는 먼저 축별 크기, 다음 방향 회전, 마지막으로 위치 이동을 적용한다. ECS 저장소에서 직접 바꾼 회전값도 계산 가능한 방향인지 확인한다.
    return glm::translate(glm::mat4(1.0F), position) *
        glm::mat4_cast(static_cast<const glm::quat&>(Rotation{rotation})) *
        glm::scale(glm::mat4(1.0F), scale);
}

TransformSystemModule::TransformSystemModule(flecs::world& world)
{
    world.module<TransformSystemModule>();
}

void TransformSystemModule::UpdateWorldTransforms(flecs::world& world)
{
    // 필요한 구성 요소가 모두 있는 물체만 Flecs query로 고른다. 전체 장면을 훑으며 각 물체에 구성 요소가 있는지 따로 묻지 않는다.
    // 물체 handle을 먼저 모아야 한다. 계산 중 부모에 행렬을 추가하면 Flecs가 찾는 물체 목록이 바뀔 수 있다.
    auto& scratch = world.ensure<TransformSystemScratch>();
    auto& transformedEntities = scratch.transformedEntities;
    auto& cache = scratch.worldTransforms;
    transformedEntities.clear();
    cache.clear();
    auto transformQuery = world.query_builder<const Position, const Rotation, const Scale,
        TransformMatrix, TransformMatrix>()
        .term_at(0).second<Local>()
        .term_at(1).second<Local>()
        .term_at(2).second<Local>()
        .term_at(3).second<Local>()
        .term_at(4).second<World>()
        .build();
    transformQuery.each([&](flecs::entity entity, const Position&, const Rotation&, const Scale&,
        TransformMatrix&, TransformMatrix&)
    {
        transformedEntities.push_back(entity);
    });

    cache.reserve(transformedEntities.size());
    // 같은 호출 안에서 여러 자식이 쓰는 부모 위치는 한 번만 계산해 재사용한다. 다음 물리 갱신이나 화면 그리기 전에는 바뀐 부모 기준 값을 새로 읽는다.
    for (flecs::entity entity : transformedEntities)
        UpdateEntityWorldTransform(entity, cache);
}

}
