#include "TrainingRangeScene.h"

#include "graphics/Renderer.h"
#include "graphics/Shader.h"

#include <map>
#include <iterator>
#include <string_view>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

namespace aerosimlab::simulator
{
namespace
{
struct Placement
{
    std::string_view model;
    glm::dvec2 position;
    float scale;
    glm::dvec3 collisionHalfExtents;
};

// 시설물은 500m × 400m 시험장 안에 두고, 각 모형과 같은 위치에 충돌 상자를 둔다.
constexpr Placement placements[]{
    {"industrial/building-a.glb", {-112.0, -62.0}, 16.0F, {16.0, 16.0, 13.0}},
    {"industrial/building-c.glb", {-52.0, -62.0}, 15.0F, {15.0, 15.0, 12.0}},
    {"industrial/building-f.glb", {16.0, -70.0}, 17.0F, {17.0, 17.0, 14.0}},
    {"industrial/building-j.glb", {84.0, -62.0}, 15.0F, {15.0, 15.0, 12.0}},
    {"industrial/building-m.glb", {-116.0, 8.0}, 14.0F, {14.0, 14.0, 11.0}},
    {"industrial/building-q.glb", {-48.0, 12.0}, 16.0F, {16.0, 16.0, 13.0}},
    {"industrial/building-t.glb", {28.0, 8.0}, 13.0F, {13.0, 13.0, 10.0}},
    {"industrial/shipping-container-a.glb", {112.0, -20.0}, 8.0F, {8.0, 9.0, 5.0}},
    {"industrial/shipping-container-b.glb", {132.0, -20.0}, 8.0F, {8.0, 9.0, 5.0}},
    {"industrial/shipping-container-c.glb", {112.0, 1.0}, 8.0F, {8.0, 9.0, 5.0}},
    {"industrial/detail-tank.glb", {-20.0, 78.0}, 10.0F, {10.0, 10.0, 9.0}},
    {"industrial/detail-tank-large.glb", {15.0, 78.0}, 13.0F, {13.0, 13.0, 12.0}},
    {"industrial/chimney-large.glb", {58.0, 78.0}, 13.0F, {7.0, 7.0, 18.0}},
    {"industrial/water-tower.glb", {164.0, 128.0}, 12.0F, {12.0, 12.0, 18.0}},
    {"industrial/windmill.glb", {-178.0, 128.0}, 12.0F, {9.0, 9.0, 20.0}},
    {"nature/tree_oak.glb", {-210.0, 18.0}, 15.0F, {5.0, 5.0, 9.0}},
    {"nature/tree_pineTallA_detailed.glb", {-205.0, 52.0}, 15.0F, {4.0, 4.0, 12.0}},
    {"nature/tree_oak.glb", {-204.0, 86.0}, 16.0F, {5.0, 5.0, 10.0}},
    {"nature/tree_pineTallA_detailed.glb", {205.0, -102.0}, 15.0F, {4.0, 4.0, 12.0}},
    {"nature/tree_oak.glb", {208.0, -66.0}, 16.0F, {5.0, 5.0, 10.0}},
    {"nature/tree_pineTallA_detailed.glb", {207.0, -28.0}, 14.0F, {4.0, 4.0, 11.0}},
    {"nature/rock_largeA.glb", {-175.0, 174.0}, 12.0F, {6.0, 5.0, 6.0}},
    {"nature/rock_tallB.glb", {-145.0, 179.0}, 12.0F, {5.0, 5.0, 6.0}},
    {"nature/rock_largeC.glb", {180.0, 174.0}, 12.0F, {7.0, 5.0, 6.0}},
};

constexpr glm::dvec2 reconLocations[]{
    // 첫 네 지점은 직접 통신 범위 안에서 나눠 맡을 수 있도록 배치한다.
    {-165.0, -116.0}, {-140.0, -116.0}, {-115.0, -116.0}, {-90.0, -116.0},
    {172.0, -82.0}, {-170.0, -18.0}, {-80.0, 54.0}, {92.0, 52.0},
    {168.0, 64.0}, {-172.0, 150.0}, {70.0, 156.0}, {176.0, 148.0},
};

glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    return {static_cast<float>(positionMeters.x),
        static_cast<float>(positionMeters.z),
        static_cast<float>(-positionMeters.y)};
}
}

void TrainingRangeScene::Load(const std::filesystem::path& assetRoot,
    const aerosimlab::terrain::TerrainGrid& terrain,
    aerosimlab::simulation::SimulationCore& simulation)
{
    std::map<std::string, std::vector<glm::mat4>> instancesByModel;
    obstacles_.clear();
    reconPoints_.clear();

    for (const Placement& placement : placements)
    {
        const double groundMeters = terrain.HeightAtLocalMeters(placement.position);
        const glm::dvec3 centerMeters{placement.position,
            groundMeters + placement.collisionHalfExtents.z};
        simulation.AddStaticObstacleBox(centerMeters,
            glm::vec3{placement.collisionHalfExtents});

        aerosimlab::terrain::MapBuilding obstacle;
        obstacle.osmId = std::string{placement.model};
        obstacle.name = std::string{placement.model};
        obstacle.heightMeters = placement.collisionHalfExtents.z * 2.0;
        obstacle.heightSource =
            aerosimlab::terrain::BuildingHeightSource::EstimatedFromFootprint;
        const double halfWidth = placement.collisionHalfExtents.x;
        const double halfDepth = placement.collisionHalfExtents.y;
        obstacle.outlineMeters = {
            placement.position + glm::dvec2{-halfWidth, -halfDepth},
            placement.position + glm::dvec2{halfWidth, -halfDepth},
            placement.position + glm::dvec2{halfWidth, halfDepth},
            placement.position + glm::dvec2{-halfWidth, halfDepth}};
        for (const glm::dvec2& corner : obstacle.outlineMeters)
            obstacle.groundElevationsMeters.push_back(
                terrain.HeightAtLocalMeters(corner));
        obstacles_.push_back(std::move(obstacle));

        glm::mat4 transform = glm::translate(glm::mat4{1.0F},
            ToRenderPosition({placement.position.x, placement.position.y,
                groundMeters}));
        transform = glm::scale(transform, glm::vec3{placement.scale});
        instancesByModel[std::string{placement.model}].push_back(transform);
    }

    for (auto& [relativeModelPath, transforms] : instancesByModel)
    {
        auto model = aerosimlab::graphics::GlbModel::Load(
            (assetRoot / "models/environment" / relativeModelPath).string());
        model->SetInstances(transforms);
        models_.push_back({std::move(model)});
    }

    for (std::size_t index = 0; index < std::size(reconLocations); ++index)
    {
        reconPoints_.push_back({
            "정찰 지점 " + std::to_string(index + 1U), reconLocations[index]});
    }
}

void TrainingRangeScene::Draw(aerosimlab::graphics::Renderer& renderer,
    aerosimlab::graphics::Shader& shader, const glm::mat4& view,
    const glm::mat4& projection, const glm::vec3& cameraPosition) const
{
    for (const ModelGroup& group : models_)
        group.model->Draw(glm::mat4{1.0F}, renderer, shader,
            view, projection, cameraPosition);
}

const std::vector<aerosimlab::terrain::MapBuilding>&
TrainingRangeScene::Obstacles() const noexcept
{
    return obstacles_;
}

const std::vector<TrainingRangeScene::ReconPoint>&
TrainingRangeScene::ReconPoints() const noexcept
{
    return reconPoints_;
}

glm::dvec2 TrainingRangeScene::FormationStagingPositionMeters() const noexcept
{
    return {0.0, -20.0};
}
}
