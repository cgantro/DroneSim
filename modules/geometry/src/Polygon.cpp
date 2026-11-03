#include "geometry/Polygon.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <glm/geometric.hpp>

namespace dronesim::geometry
{
namespace
{
double Cross2D(const glm::dvec2& first, const glm::dvec2& second,
    const glm::dvec2& third) noexcept
{
    const glm::dvec2 firstEdge = second - first;
    const glm::dvec2 secondEdge = third - first;
    return firstEdge.x * secondEdge.y - firstEdge.y * secondEdge.x;
}

bool PointInTriangle(const glm::dvec2& point, const glm::dvec2& first,
    const glm::dvec2& second, const glm::dvec2& third, double winding) noexcept
{
    constexpr double tolerance = 1.0e-10;
    return winding * Cross2D(first, second, point) >= -tolerance
        && winding * Cross2D(second, third, point) >= -tolerance
        && winding * Cross2D(third, first, point) >= -tolerance;
}
}

double SignedArea(std::span<const glm::dvec2> outline) noexcept
{
    double twiceArea = 0.0;
    for (std::size_t index = 0; index < outline.size(); ++index)
    {
        const glm::dvec2& first = outline[index];
        const glm::dvec2& second = outline[(index + 1U) % outline.size()];
        twiceArea += first.x * second.y - second.x * first.y;
    }
    return twiceArea * 0.5;
}

std::vector<std::array<std::size_t, 3>> TriangulateSimplePolygon(
    std::span<const glm::dvec2> outline)
{
    if (outline.size() < 3U || SignedArea(outline) == 0.0)
        throw std::invalid_argument("삼각형으로 나눌 다각형의 꼭짓점이 부족합니다.");

    std::vector<std::size_t> remaining(outline.size());
    for (std::size_t index = 0; index < remaining.size(); ++index)
        remaining[index] = index;

    // 꼭짓점 순서에 따라 볼록한 귀의 외적 부호가 달라지므로 방향을 먼저 고정한다.
    const double winding = SignedArea(outline) > 0.0 ? 1.0 : -1.0;
    std::vector<std::array<std::size_t, 3>> triangles;
    triangles.reserve(outline.size() - 2U);

    // 매 반복에서 볼록하고 내부에 다른 꼭짓점이 없는 귀를 잘라 삼각형으로 확정한다.
    while (remaining.size() > 3U)
    {
        bool clippedEar = false;
        for (std::size_t index = 0; index < remaining.size(); ++index)
        {
            const std::size_t previous =
                remaining[(index + remaining.size() - 1U) % remaining.size()];
            const std::size_t current = remaining[index];
            const std::size_t next = remaining[(index + 1U) % remaining.size()];
            if (winding * Cross2D(outline[previous], outline[current],
                    outline[next]) <= 1.0e-10)
                continue;

            bool containsVertex = false;
            for (const std::size_t candidate : remaining)
            {
                if (candidate == previous || candidate == current || candidate == next)
                    continue;
                if (PointInTriangle(outline[candidate], outline[previous],
                        outline[current], outline[next], winding))
                {
                    containsVertex = true;
                    break;
                }
            }
            if (containsVertex)
                continue;

            triangles.push_back({previous, current, next});
            remaining.erase(remaining.begin()
                + static_cast<std::ptrdiff_t>(index));
            clippedEar = true;
            break;
        }

        if (!clippedEar)
            throw std::invalid_argument("다각형을 삼각형으로 나누지 못했습니다.");
    }

    triangles.push_back({remaining[0], remaining[1], remaining[2]});
    return triangles;
}

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
