#include "TrainingRangeScene.h"

#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "geometry/VectorMath.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

namespace dronesim::scene
{
namespace
{
/**
 * @brief 점들을 감싸는 볼록 외곽선을 Andrew의 단조 사슬 방식으로 계산한다.
 * @details 모든 GLB 정점을 장애물 경계로 쓰면 내부 점까지 경로 장애물로 오인할 수 있다.
 *          바닥 윤곽의 바깥선만 남겨 경로 탐색에 전달한다. 오목한 건물은 실제보다 넓게
 *          막을 수 있지만, 경로 계산은 그만큼 보수적으로 된다.
 */
std::vector<glm::dvec2> MakeFootprintHull(std::vector<glm::dvec2> points)
{
    // x, y 순으로 정렬해 아래쪽 사슬과 위쪽 사슬을 같은 순서로 훑는다.
    std::sort(points.begin(), points.end(), [](const auto& left, const auto& right)
    {
        return left.x == right.x ? left.y < right.y : left.x < right.x;
    });
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() < 3U)
        return {};

    std::vector<glm::dvec2> hull;
    hull.reserve(points.size() * 2U);
    for (const glm::dvec2& point : points)
    {
        // 외적이 0 이하인 점은 오른쪽 회전이거나 일직선이므로 외곽선에서 뺀다.
        while (hull.size() >= 2U
            && geometry::Cross2D(hull[hull.size() - 2U], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lowerHullSize = hull.size();

    // 역순으로 훑어 위쪽 사슬을 만든 뒤, 아래쪽 끝점이 중복되지 않게 뺀다.
    for (auto point = points.rbegin() + 1; point != points.rend(); ++point)
    {
        while (hull.size() > lowerHullSize
            && geometry::Cross2D(hull[hull.size() - 2U], hull.back(), *point) <= 0.0)
            hull.pop_back();
        hull.push_back(*point);
    }
    hull.pop_back();
    return hull;
}

}

void TrainingRangeScene::Load(const std::filesystem::path& assetRoot)
{
    const float worldScale = static_cast<float>(mapScale);
    cityModel_ = dronesim::graphics::GlbModel::Load(
        (assetRoot / "maps/ruined_city.glb").string());
    buildingObstacles_.clear();
    collisionMesh_ = {};

    const auto& partBounds = cityModel_->PartBounds();
    const auto collisionParts = cityModel_->CollisionParts();
    if (collisionParts.size() != partBounds.size())
        throw std::logic_error("도시 지도 충돌 자료와 화면 메시 순서가 다릅니다.");
    double largestGroundArea = 0.0;
    std::size_t groundSurfacePart = partBounds.size();
    dronesim::graphics::GlbPartBounds groundBounds{};
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

    // 현재 훈련장은 평탄한 바닥으로 가정한다. 61×61 격자를 쓰면 화면 바닥과
    // 같은 외곽 범위를 경로 탐색과 물리 충돌에서도 공유할 수 있다.
    constexpr std::uint32_t terrainSampleCount = 61U;
    terrain_.sampleCount = terrainSampleCount;
    terrain_.spacingXMeters = static_cast<float>(
        groundSizeMeters_.x / static_cast<double>(terrainSampleCount - 1U));
    terrain_.spacingYMeters = static_cast<float>(
        groundSizeMeters_.y / static_cast<double>(terrainSampleCount - 1U));
    terrain_.referenceElevationMeters = 0.0F;
    terrain_.heightOffsetsMeters.assign(
        static_cast<std::size_t>(terrainSampleCount) * terrainSampleCount, 0.0F);

    for (std::size_t index = 0; index < partBounds.size(); ++index)
    {
        if (index == groundSurfacePart)
            continue; // 바닥판은 같은 크기로 만든 높이 지형이 충돌을 맡는다.

        const auto& part = collisionParts[index];
        const std::uint32_t firstVertex = static_cast<std::uint32_t>(
            collisionMesh_.verticesMeters.size());
        std::vector<glm::dvec2> footprintPoints;
        footprintPoints.reserve(part.vertices.size());
        double highestPointMeters = 0.0;
        for (const glm::vec3& vertex : part.vertices)
        {
            const glm::vec3 scenePosition = glm::vec3(
                part.nodeTransform * glm::vec4(vertex, 1.0F));
            // GLB의 Y-up 좌표를 시뮬레이터 ENU(X=동, Y=북, Z=상)로 바꾼다.
            const glm::dvec3 positionMeters{
                (scenePosition.x + mapOffset_.x) * mapScale,
                -(scenePosition.z + mapOffset_.z) * mapScale,
                (scenePosition.y + mapOffset_.y) * mapScale};
            collisionMesh_.verticesMeters.push_back(positionMeters);
            footprintPoints.emplace_back(positionMeters.x, positionMeters.y);
            highestPointMeters = std::max(highestPointMeters, positionMeters.z);
        }
        collisionMesh_.triangleIndices.reserve(
            collisionMesh_.triangleIndices.size() + part.triangleIndices.size());
        for (const std::uint32_t vertexIndex : part.triangleIndices)
            collisionMesh_.triangleIndices.push_back(firstVertex + vertexIndex);

        dronesim::flight_planning::BuildingObstacle obstacle;
        obstacle.name = partBounds[index].nodeName.empty()
            ? "이름 없는 구조물 " + std::to_string(index + 1U)
            : partBounds[index].nodeName;
        obstacle.heightMeters = highestPointMeters;
        obstacle.outlineMeters = MakeFootprintHull(std::move(footprintPoints));
        // 바닥판을 원점 높이 z=0에 맞췄으므로 장애물 외곽도 같은 높이에서 시작한다.
        obstacle.groundElevationsMeters.assign(obstacle.outlineMeters.size(), 0.0);
        if (obstacle.outlineMeters.size() >= 3U && obstacle.heightMeters > 0.0)
            buildingObstacles_.push_back(std::move(obstacle));
    }

}

void TrainingRangeScene::Draw(dronesim::graphics::Renderer& renderer,
    dronesim::graphics::Shader& shader, const glm::mat4& view,
    const glm::mat4& projection, const glm::vec3& cameraPosition) const
{
    if (cityModel_)
        cityModel_->Draw(mapTransform_, renderer, shader,
            view, projection, cameraPosition);
}

const std::vector<dronesim::flight_planning::BuildingObstacle>&
TrainingRangeScene::BuildingObstacles() const noexcept
{
    return buildingObstacles_;
}

const TrainingRangeScene::BuildingCollisionMesh&
TrainingRangeScene::CollisionMesh() const noexcept
{
    return collisionMesh_;
}

glm::dvec2 TrainingRangeScene::GroundSizeMeters() const noexcept
{
    return groundSizeMeters_;
}

const terrain::TerrainGrid& TrainingRangeScene::Terrain() const noexcept
{
    return terrain_;
}
}
