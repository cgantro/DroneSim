#include "TrainingRangeScene.h"

#include "graphics/Renderer.h"
#include "graphics/Shader.h"

#include <algorithm>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

namespace aerosimlab::simulator
{
namespace
{
// 네 기체가 직접 통신 범위를 유지하도록 정찰 구역은 120m 안에 둔다.
constexpr glm::dvec2 reconLocations[]{
    {-0.6, -23.0}, {-0.2, -23.0}, {0.2, -23.0}, {0.6, -23.0}};

}

void TrainingRangeScene::Load(const std::filesystem::path& assetRoot,
    aerosimlab::simulation::SimulationCore& simulation)
{
    const float worldScale = static_cast<float>(mapScale);
    cityModel_ = aerosimlab::graphics::GlbModel::Load(
        (assetRoot / "maps/ruined_city.glb").string());
    buildingObstacles_.clear();
    reconPoints_.clear();

    const auto& partBounds = cityModel_->PartBounds();
    double largestGroundArea = 0.0;
    std::size_t groundSurfacePart = partBounds.size();
    aerosimlab::graphics::GlbPartBounds groundBounds{};
    for (std::size_t index = 0; index < partBounds.size(); ++index)
    {
        const auto& bounds = partBounds[index];
        const double thickness = bounds.maximum.y - bounds.minimum.y;
        const double width = bounds.maximum.x - bounds.minimum.x;
        const double depth = bounds.maximum.z - bounds.minimum.z;
        const double area = width * depth;

        // 에셋의 크기가 달라도 두께 비율로 바닥판을 고른다.
        if (thickness < std::max(width, depth) * 0.02
            && area > largestGroundArea)
        {
            largestGroundArea = area;
            groundBounds = bounds;
            groundSurfacePart = index;
        }
    }
    if (groundSurfacePart == partBounds.size())
        throw std::runtime_error("도시 지도에서 바닥판을 찾지 못했습니다.");

    // 전체 구조물 중심이 아니라 실제 바닥판 중심을 원점으로 맞춘다.
    const float groundSurfaceY = groundBounds.maximum.y;
    mapOffset_ = {
        -0.5F * (groundBounds.minimum.x + groundBounds.maximum.x),
        -groundSurfaceY,
        -0.5F * (groundBounds.minimum.z + groundBounds.maximum.z)};
    mapTransform_ = glm::scale(
        glm::translate(glm::mat4{1.0F}, mapOffset_ * worldScale),
        glm::vec3{worldScale});
    groundSizeMeters_ = {
        static_cast<double>(groundBounds.maximum.x - groundBounds.minimum.x) * mapScale,
        static_cast<double>(groundBounds.maximum.z - groundBounds.minimum.z) * mapScale};

    std::size_t obstacleNumber = 0;
    for (std::size_t index = 0; index < partBounds.size(); ++index)
    {
        if (index == groundSurfacePart)
            continue; // 바닥판은 평면 지형 충돌체가 맡는다.

        const auto& bounds = partBounds[index];
        const double minHeight = (bounds.minimum.y + mapOffset_.y) * mapScale;
        const double maxHeight = (bounds.maximum.y + mapOffset_.y) * mapScale;

        // GLB의 Z축은 시뮬레이터의 북쪽 축과 반대 방향이다.
        const glm::dvec2 minimum{
            (bounds.minimum.x + mapOffset_.x) * mapScale,
            -(bounds.maximum.z + mapOffset_.z) * mapScale};
        const glm::dvec2 maximum{
            (bounds.maximum.x + mapOffset_.x) * mapScale,
            -(bounds.minimum.z + mapOffset_.z) * mapScale};
        const glm::dvec2 center = (minimum + maximum) * 0.5;
        const double bottom = std::max(0.0, minHeight);
        const double top = std::max(bottom + 0.1, maxHeight);
        const glm::dvec3 halfExtents{
            std::max(0.05, (maximum.x - minimum.x) * 0.5),
            std::max(0.05, (maximum.y - minimum.y) * 0.5),
            (top - bottom) * 0.5};
        const glm::dvec3 centerMeters{center, (top + bottom) * 0.5};
        simulation.AddStaticObstacleBox(centerMeters, glm::vec3{halfExtents});

        aerosimlab::flight_planning::BuildingObstacle obstacle;
        obstacle.name = "도시 구조물 " + std::to_string(++obstacleNumber);
        obstacle.heightMeters = top;
        obstacle.outlineMeters = {
            {minimum.x, minimum.y}, {maximum.x, minimum.y},
            {maximum.x, maximum.y}, {minimum.x, maximum.y}};
        obstacle.groundElevationsMeters.assign(4U, 0.0F);
        buildingObstacles_.push_back(std::move(obstacle));
    }

    for (std::size_t index = 0; index < std::size(reconLocations); ++index)
        reconPoints_.push_back({
            "정찰 지점 " + std::to_string(index + 1U),
            reconLocations[index] * mapScale});
}

void TrainingRangeScene::Draw(aerosimlab::graphics::Renderer& renderer,
    aerosimlab::graphics::Shader& shader, const glm::mat4& view,
    const glm::mat4& projection, const glm::vec3& cameraPosition) const
{
    if (cityModel_)
        cityModel_->Draw(mapTransform_, renderer, shader,
            view, projection, cameraPosition);
}

const std::vector<aerosimlab::flight_planning::BuildingObstacle>&
TrainingRangeScene::BuildingObstacles() const noexcept
{
    return buildingObstacles_;
}

const std::vector<TrainingRangeScene::ReconPoint>&
TrainingRangeScene::ReconPoints() const noexcept
{
    return reconPoints_;
}

glm::dvec2 TrainingRangeScene::GroundSizeMeters() const noexcept
{
    return groundSizeMeters_;
}

glm::dvec2 TrainingRangeScene::FormationStagingPositionMeters() const noexcept
{
    return {0.0, -23.0 * mapScale};
}
}
