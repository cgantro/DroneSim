#include "geometry/Polygon.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/geometric.hpp>

namespace dronesim::geometry
{
bool ContainsPointInPolygon(
    const glm::dvec2& point, std::span<const glm::dvec2> outline) noexcept
{
    if (outline.size() < 3U)
        return false;

    // 수평선을 오른쪽으로 쏜 뒤 변을 지나는 횟수가 홀수인지 센다.
    bool inside = false;
    for (std::size_t index = 0, previous = outline.size() - 1U;
         index < outline.size(); previous = index++)
    {
        const glm::dvec2& first = outline[index];
        const glm::dvec2& second = outline[previous];
        const bool crossesRay = (first.y > point.y) != (second.y > point.y);
        if (crossesRay
            && point.x < (second.x - first.x) * (point.y - first.y)
                / (second.y - first.y) + first.x)
        {
            inside = !inside;
        }
    }
    return inside;
}

double SquaredDistanceToSegment(
    const glm::dvec2& point,
    const glm::dvec2& start,
    const glm::dvec2& end) noexcept
{
    const glm::dvec2 edge = end - start;
    const double edgeLengthSquared = glm::dot(edge, edge);
    // 내적으로 점을 선분 방향에 투영한다. 비율을 0~1로 제한하면 선분 양 끝을 벗어나지 않는다.
    // 길이가 0인 선분은 시작점 하나로 보고 투영 비율을 0으로 둔다.
    const double fraction = edgeLengthSquared > 0.0
        ? std::clamp(glm::dot(point - start, edge) / edgeLengthSquared, 0.0, 1.0)
        : 0.0;
    const glm::dvec2 difference = point - (start + edge * fraction);
    return glm::dot(difference, difference);
}

double SquaredDistanceToPolygon(
    const glm::dvec2& point, std::span<const glm::dvec2> outline) noexcept
{
    if (outline.size() < 3U)
        return std::numeric_limits<double>::infinity();
    if (ContainsPointInPolygon(point, outline))
        return 0.0;

    // 모든 변의 거리 제곱을 비교한다. 안전거리 비교만 필요한 호출자는 제곱근을 구하지 않아도 된다.
    double nearestDistanceSquared = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < outline.size(); ++index)
    {
        nearestDistanceSquared = std::min(nearestDistanceSquared,
            SquaredDistanceToSegment(point, outline[index],
                outline[(index + 1U) % outline.size()]));
    }
    return nearestDistanceSquared;
}

double DistanceToPolygon(
    const glm::dvec2& point, std::span<const glm::dvec2> outline) noexcept
{
    return std::sqrt(SquaredDistanceToPolygon(point, outline));
}
}
