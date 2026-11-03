#include "TrainingRangeScene.h"

#include "graphics/Renderer.h"
#include "graphics/Shader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

namespace dronesim::scene
{
namespace
{
double Cross(const glm::dvec2& origin, const glm::dvec2& first,
    const glm::dvec2& second)
{
    const glm::dvec2 a = first - origin;
    const glm::dvec2 b = second - origin;
    return a.x * b.y - a.y * b.x;
}

std::vector<glm::dvec2> MakeFootprintHull(std::vector<glm::dvec2> points)
{
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
        while (hull.size() >= 2U
            && Cross(hull[hull.size() - 2U], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lowerSize = hull.size();
    for (auto point = points.rbegin() + 1; point != points.rend(); ++point)
    {
        while (hull.size() > lowerSize
            && Cross(hull[hull.size() - 2U], hull.back(), *point) <= 0.0)
            hull.pop_back();
        hull.push_back(*point);
    }
    hull.pop_back();
    return hull;
}

double DistanceToSegment(const glm::dvec2& point,
    const glm::dvec2& start, const glm::dvec2& end)
{
    const glm::dvec2 edge = end - start;
    const double edgeLengthSquared = glm::dot(edge, edge);
    const double fraction = edgeLengthSquared > 0.0
        ? std::clamp(glm::dot(point - start, edge) / edgeLengthSquared, 0.0, 1.0)
        : 0.0;
    return glm::length(point - (start + fraction * edge));
}

double DistanceToStructures(const glm::dvec2& point,
    const std::vector<dronesim::flight_planning::BuildingObstacle>& buildings)
{
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& building : buildings)
    {
        const auto& outline = building.outlineMeters;
        bool inside = false;
        for (std::size_t index = 0, previous = outline.size() - 1U;
            index < outline.size(); previous = index++)
        {
            const auto& first = outline[index];
            const auto& second = outline[previous];
            if ((first.y > point.y) != (second.y > point.y)
                && point.x < (second.x - first.x) * (point.y - first.y)
                    / (second.y - first.y) + first.x)
                inside = !inside;
            nearest = std::min(nearest, DistanceToSegment(
                point, first, second));
        }
        if (inside)
            return 0.0;
    }
    return nearest;
}

glm::dvec2 FindOpenPosition(const glm::dvec2& preferred,
    const glm::dvec2& halfSize,
    const std::vector<dronesim::flight_planning::BuildingObstacle>& buildings,
    const std::vector<glm::dvec2>& reserved, double minimumClearanceMeters,
    double minimumSeparationMeters, double maximumSearchDistanceMeters)
{
    constexpr double searchStepMeters = 5.0;
    glm::dvec2 best{0.0};
    double bestDistance = std::numeric_limits<double>::infinity();
    for (double north = -halfSize.y + searchStepMeters;
        north <= halfSize.y - searchStepMeters; north += searchStepMeters)
    {
        for (double east = -halfSize.x + searchStepMeters;
            east <= halfSize.x - searchStepMeters; east += searchStepMeters)
        {
            const glm::dvec2 candidate{east, north};
            if (DistanceToStructures(candidate, buildings) < minimumClearanceMeters)
                continue;
            if (std::any_of(reserved.begin(), reserved.end(),
                [&](const glm::dvec2& position)
                {
                    return glm::distance(candidate, position) < minimumSeparationMeters;
                }))
                continue;

            const double distance = glm::distance(candidate, preferred);
            if (distance > maximumSearchDistanceMeters)
                continue;
            if (distance < bestDistance)
            {
                best = candidate;
                bestDistance = distance;
            }
        }
    }
    if (!std::isfinite(bestDistance))
        throw std::runtime_error("도시 지도에서 드론이 모일 빈 공간을 찾지 못했습니다.");
    return best;
}

}

void TrainingRangeScene::Load(const std::filesystem::path& assetRoot,
    dronesim::simulation::SimulationCore& simulation)
{
    const float worldScale = static_cast<float>(mapScale);
    cityModel_ = dronesim::graphics::GlbModel::Load(
        (assetRoot / "maps/ruined_city.glb").string());
    buildingObstacles_.clear();
    reconPoints_.clear();

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

    std::vector<glm::dvec3> collisionVertices;
    std::vector<std::uint32_t> collisionIndices;
    std::size_t obstacleNumber = 0;
    for (std::size_t index = 0; index < partBounds.size(); ++index)
    {
        if (index == groundSurfacePart)
            continue; // 바닥판은 같은 크기로 만든 높이 지형이 충돌을 맡는다.

        const auto& part = collisionParts[index];
        const std::uint32_t firstVertex = static_cast<std::uint32_t>(collisionVertices.size());
        std::vector<glm::dvec2> footprintPoints;
        footprintPoints.reserve(part.vertices.size());
        double highestPointMeters = 0.0;
        for (const glm::vec3& vertex : part.vertices)
        {
            const glm::vec3 scenePosition = glm::vec3(
                part.nodeTransform * glm::vec4(vertex, 1.0F));
            const glm::dvec3 positionMeters{
                (scenePosition.x + mapOffset_.x) * mapScale,
                -(scenePosition.z + mapOffset_.z) * mapScale,
                (scenePosition.y + mapOffset_.y) * mapScale};
            collisionVertices.push_back(positionMeters);
            footprintPoints.emplace_back(positionMeters.x, positionMeters.y);
            highestPointMeters = std::max(highestPointMeters, positionMeters.z);
        }
        collisionIndices.reserve(collisionIndices.size() + part.triangleIndices.size());
        for (const std::uint32_t vertexIndex : part.triangleIndices)
            collisionIndices.push_back(firstVertex + vertexIndex);

        dronesim::flight_planning::BuildingObstacle obstacle;
        obstacle.name = "도시 구조물 " + std::to_string(++obstacleNumber);
        obstacle.heightMeters = highestPointMeters;
        obstacle.outlineMeters = MakeFootprintHull(std::move(footprintPoints));
        obstacle.groundElevationsMeters.assign(obstacle.outlineMeters.size(), 0.0);
        if (obstacle.outlineMeters.size() >= 3U && obstacle.heightMeters > 0.0)
            buildingObstacles_.push_back(std::move(obstacle));
    }

    // 렌더 메시의 실제 삼각형을 충돌에 사용해 빈 공간을 상자로 막지 않는다.
    simulation.AddStaticObstacleMesh(collisionVertices, collisionIndices);

    const glm::dvec2 halfSize = groundSizeMeters_ * 0.5;
    stagingPositionMeters_ = FindOpenPosition(
        {0.0, -23.0 * mapScale}, halfSize, buildingObstacles_, {},
        10.0, 0.0, std::numeric_limits<double>::infinity());
    const std::array<glm::dvec2, 4> preferredReconPositions{
        stagingPositionMeters_ + glm::dvec2{6.0, 0.0},
        stagingPositionMeters_ + glm::dvec2{-6.0, 0.0},
        stagingPositionMeters_ + glm::dvec2{0.0, 6.0},
        stagingPositionMeters_ + glm::dvec2{0.0, -6.0}};
    std::vector<glm::dvec2> reserved{stagingPositionMeters_};
    for (std::size_t index = 0; index < preferredReconPositions.size(); ++index)
    {
        const glm::dvec2 position = FindOpenPosition(
            preferredReconPositions[index], halfSize, buildingObstacles_,
            reserved, 8.0, 6.0, 25.0);
        reserved.push_back(position);
        reconPoints_.push_back({"정찰 지점 " + std::to_string(index + 1U), position});
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
    return stagingPositionMeters_;
}
}
