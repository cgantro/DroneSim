#include "CollisionGeometry.h"
#include "model/ModelResource.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace grasplink::simulation::detail
{
using grasplink::model::MeshData;
using grasplink::model::ModelResource;
using grasplink::model::NodeData;

namespace
{
struct ModelTransformErrorMessages final
{
    inline static constexpr char nodeCycle[] = "모델 노드 계층에 순환 참조가 있습니다.";
    inline static constexpr char invalidParentNode[] = "모델 노드가 존재하지 않는 부모를 가리킵니다.";
};

struct ConvexHullApproximationSettings final
{
    inline static constexpr std::size_t minimumPointCount = 4;
    inline static constexpr int samplingDirectionCount = 162;
    inline static constexpr float goldenAngleRadians = 2.39996323F;
    inline static constexpr float duplicatePointSquaredDistance = 1.0e-10F;
    inline static constexpr float minimumEdgeSquaredLength = 1.0e-10F;
    inline static constexpr float minimumTriangleSquaredArea = 1.0e-10F;
    inline static constexpr float maximumPlaneDeviation = 1.0e-7F;
};

glm::mat4 NodeLocalTransform(const NodeData& node)
{
    return glm::translate(glm::mat4(1.0F), node.translation) *
        glm::mat4_cast(glm::normalize(node.rotation)) * glm::scale(glm::mat4(1.0F), node.scale);
}
}

std::vector<glm::mat4> BuildNodeWorldTransforms(const ModelResource& model)
{
    std::vector<glm::mat4> transforms(model.nodes.size(), glm::mat4(1.0F));
    std::vector<bool> ready(model.nodes.size(), false);
    std::vector<bool> visiting(model.nodes.size(), false);
    auto build = [&](auto&& self, std::size_t index) -> const glm::mat4&
    {
        if (ready[index]) return transforms[index];
        if (visiting[index])
            throw std::invalid_argument(ModelTransformErrorMessages::nodeCycle);
        visiting[index] = true;
        const NodeData& node = model.nodes[index];
        if (node.parentIndex && *node.parentIndex >= model.nodes.size())
            throw std::invalid_argument(ModelTransformErrorMessages::invalidParentNode);
        transforms[index] = NodeLocalTransform(node);
        if (node.parentIndex)
            transforms[index] = self(self, *node.parentIndex) * transforms[index];
        ready[index] = true;
        visiting[index] = false;
        return transforms[index];
    };
    for (std::size_t i = 0; i < model.nodes.size(); ++i) build(build, i);
    return transforms;
}

std::vector<glm::vec3> BuildConvexSupportPoints(const std::vector<glm::vec3>& vertices)
{
    std::vector<glm::vec3> result;
    if (vertices.size() < ConvexHullApproximationSettings::minimumPointCount)
        return result;

    // 여러 방향에서 가장 바깥에 있는 정점만 골라 Jolt에 전달해 충돌 계산에 쓰는 점의 수를 제한한다.
    // 볼록 외피는 원본 메시의 오목한 부분을 채울 수 있으므로, 화면 모양과 충돌 모양은 서로 다를 수 있다.
    auto addExtreme = [&](const glm::vec3& direction)
    {
        const auto iterator = std::max_element(vertices.begin(), vertices.end(),
            [&](const glm::vec3& left, const glm::vec3& right)
            {
                return glm::dot(left, direction) < glm::dot(right, direction);
            });
        if (iterator == vertices.end()) return;
        if (std::none_of(result.begin(), result.end(), [&](const glm::vec3& point)
            { return glm::length2(point - *iterator) <
                ConvexHullApproximationSettings::duplicatePointSquaredDistance; }))
            result.push_back(*iterator);
    };

    for (int i = 0; i < ConvexHullApproximationSettings::samplingDirectionCount; ++i)
    {
        const float y = 1.0F - 2.0F * (static_cast<float>(i) + 0.5F) /
            ConvexHullApproximationSettings::samplingDirectionCount;
        const float radius = std::sqrt(std::max(0.0F, 1.0F - y * y));
        const float angle = ConvexHullApproximationSettings::goldenAngleRadians * i;
        addExtreme({radius * std::cos(angle), y, radius * std::sin(angle)});
    }
    addExtreme({1.0F, 0.0F, 0.0F});
    addExtreme({-1.0F, 0.0F, 0.0F});
    addExtreme({0.0F, 1.0F, 0.0F});
    addExtreme({0.0F, -1.0F, 0.0F});
    addExtreme({0.0F, 0.0F, 1.0F});
    addExtreme({0.0F, 0.0F, -1.0F});
    return result;
}

bool HasHullVolume(const std::vector<glm::vec3>& points, float coordinateScaleMeters)
{
    // 점이 네 개 미만이면 3차원 부피를 만들 수 없다. 네 개 이상이어도 한 평면에 놓일 수 있어 평면 밖의 점이 있는지 확인한다.
    if (points.size() < ConvexHullApproximationSettings::minimumPointCount)
        return false;
    glm::vec3 minimum = points.front();
    glm::vec3 maximum = points.front();
    for (const auto& point : points)
    {
        minimum = glm::min(minimum, point);
        maximum = glm::max(maximum, point);
    }
    // 기준 길이가 양수면 그대로 쓰고, 그렇지 않으면 점 집합의 크기를 기준으로 삼는다.
    const float extent = coordinateScaleMeters > 0.0F ? coordinateScaleMeters : glm::length(maximum - minimum);
    if (!std::isfinite(extent) || extent <= 0.0F) return false;
    std::vector<glm::vec3> normalized;
    normalized.reserve(points.size());
    for (const auto& point : points) normalized.push_back(point / extent);
    const glm::vec3 origin = normalized.front();
    std::size_t edgeIndex = 1;
    while (edgeIndex < normalized.size() && glm::length2(normalized[edgeIndex] - origin) <
        ConvexHullApproximationSettings::minimumEdgeSquaredLength)
        ++edgeIndex;
    if (edgeIndex == normalized.size()) return false;

    const glm::vec3 edge = normalized[edgeIndex] - origin;
    glm::vec3 normal{0.0F};
    bool foundPlane = false;
    for (std::size_t i = 1; i < normalized.size(); ++i)
    {
        normal = glm::cross(edge, normalized[i] - origin);
        if (glm::length2(normal) > ConvexHullApproximationSettings::minimumTriangleSquaredArea)
        {
            foundPlane = true;
            break;
        }
    }
    if (!foundPlane) return false;
    for (const glm::vec3& point : normalized)
        if (std::abs(glm::dot(normal, point - origin)) >
            ConvexHullApproximationSettings::maximumPlaneDeviation)
            return true;
    return false;
}

}
