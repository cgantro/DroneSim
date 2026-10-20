#include "scene/Scene.h"

#include "scene/TransformComponents.h"

#include <stdexcept>

namespace grasplink::scene
{

Scene::Scene(flecs::world& world)
    : m_World(world), m_SceneRoot(world.entity().add<SceneRootTag>())
{
}

Scene::~Scene()
{
    if (m_SceneRoot.is_alive())
        m_SceneRoot.destruct();
}

Entity Scene::CreateEntity(const std::string& name)
{
    if (!m_SceneRoot.is_alive())
        throw std::logic_error("Scene::CreateEntity requires a live Scene");

    flecs::entity entity = name.empty() ? m_World.entity() : m_World.entity(name.c_str());
    entity
        .set<Position, Local>(Position{0.0F, 0.0F, 0.0F})
        .set<Rotation, Local>(Rotation{})
        .set<Scale, Local>(Scale{1.0F})
        .set<TransformMatrix, Local>(TransformMatrix{})
        .set<TransformMatrix, World>(TransformMatrix{})
        .child_of(m_SceneRoot);
    return Entity{entity};
}

flecs::world& Scene::GetWorld()
{
    return m_World;
}

flecs::entity Scene::GetSceneRoot() const
{
    return m_SceneRoot;
}

}
