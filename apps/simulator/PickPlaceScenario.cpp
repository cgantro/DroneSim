#include "PickPlaceScenario.h"

#include "application/PickPlaceConfig.h"
#include "application/PickPlaceScenarioSampler.h"
#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "rendering/components/RenderComponents.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/SimulationSceneBuilder.h"

#include <glm/glm.hpp>

#include <stdexcept>

namespace grasplink::simulator::pick_place
{
namespace config = grasplink::application::pick_place::config;
using grasplink::graphics::Material;
using grasplink::graphics::Mesh;
using grasplink::graphics::Shader;
using grasplink::rendering::MeshFilter;
using grasplink::rendering::MeshRenderer;
using grasplink::scene::Entity;
using grasplink::scene::Scene;

void ConfigureFloor(Entity& floor)
{
    grasplink::simulation::ConfigureStaticFloor(
        floor, config::floorHalfExtentsMeters, config::floorColliderCenterOffsetMeters);
}

Entity CreateGraspBox(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("PickPlaceScenario: grasp box requires a shader");
    constexpr float sideMeters = config::boxSideMeters;
    Entity box = scene.CreateEntity("GraspBox");
    box.SetLocalPosition({0.0F, config::boxPositionHeightMeters, 0.0F});
    box.set<RigidBody>({grasplink::physics::BodyMotionType::Dynamic, grasplink::physics::CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box(glm::vec3(sideMeters * 0.5F))}});
    box.set<MeshFilter>({Mesh::CreateCube(sideMeters), 0U, 0U})
        .set<MeshRenderer>({shader, std::make_shared<Material>(glm::vec4{0.15F, 0.65F, 0.95F, 1.0F}, 0.0F, 0.75F), true});
    return box;
}

Entity CreatePlacementArea(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("PickPlaceScenario: placement area requires a shader");
    Entity area = scene.CreateEntity("PlacementArea");
    area.SetLocalPosition({0.0F, config::placementPositionHeightMeters, 0.0F});
    area.SetLocalScale({config::placementAreaSideMeters, config::placementAreaThicknessMeters,
        config::placementAreaSideMeters});
    area.set<MeshFilter>({Mesh::CreateCube(1.0F), 0U, 0U})
        .set<MeshRenderer>({shader,
            std::make_shared<Material>(glm::vec4{0.18F, 0.82F, 0.32F, 1.0F}, 0.0F, 0.8F), true});
    return area;
}

void RandomizePickPlaceScene(Entity& box, Entity& placement,
    grasplink::application::PickPlaceScenarioSampler& sampler)
{
    const auto boxPosition = sampler.SampleBoxPosition();
    const float boxRotation = sampler.SamplePlanarRotation();
    const auto placementPosition = sampler.SamplePlacementPosition();
    const float placementRotation = sampler.SamplePlanarRotation();
    box.SetLocalPosition({boxPosition[0], boxPosition[1], boxPosition[2]});
    box.SetLocalRotation(glm::angleAxis(boxRotation, glm::vec3{0.0F, 1.0F, 0.0F}));
    placement.SetLocalPosition({placementPosition[0], placementPosition[1], placementPosition[2]});
    placement.SetLocalRotation(glm::angleAxis(placementRotation, glm::vec3{0.0F, 1.0F, 0.0F}));
}

} // namespace grasplink::simulator::pick_place
