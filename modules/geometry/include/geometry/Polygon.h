#pragma once

#include <span>

#include <glm/vec2.hpp>

namespace dronesim::geometry
{
/**
 * @brief 점이 다각형 내부에 있는지 홀짝 교차법으로 판정한다.
 * @details 광선을 다각형 변과 교차시켜 교차 횟수가 홀수면 내부로 본다.
 */
[[nodiscard]] bool ContainsPointInPolygon(
    const glm::dvec2& point, std::span<const glm::dvec2> outline) noexcept;

/** @brief 점과 선분 사이의 최단 거리 제곱을 반환한다. */
[[nodiscard]] double SquaredDistanceToSegment(
    const glm::dvec2& point,
    const glm::dvec2& start,
    const glm::dvec2& end) noexcept;

/**
 * @brief 다각형 내부는 0, 바깥은 가장 가까운 변까지의 거리를 반환한다.
 * @details 꼭짓점이 세 개보다 적으면 다각형이 아니므로 무한대를 반환한다.
 */
[[nodiscard]] double DistanceToPolygon(
    const glm::dvec2& point, std::span<const glm::dvec2> outline) noexcept;
}
