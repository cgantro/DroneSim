#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include <glm/vec2.hpp>

namespace dronesim::geometry
{
/** @brief 다각형의 부호 있는 넓이를 구한다. 반시계 방향 꼭짓점이면 양수다. */
[[nodiscard]] double SignedArea(std::span<const glm::dvec2> outline) noexcept;

/**
 * @brief 구멍이 없는 단순 다각형을 삼각형 꼭짓점 번호로 나눈다.
 * @details 귀를 하나씩 잘라내는 방식이다. 꼭짓점 순서는 입력 순서로 유지하며,
 * 자기 교차 다각형이나 면적이 0인 다각형은 지원하지 않는다.
 */
[[nodiscard]] std::vector<std::array<std::size_t, 3>> TriangulateSimplePolygon(
    std::span<const glm::dvec2> outline);

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
 * @brief 다각형 내부는 0, 바깥은 가장 가까운 변까지 거리의 제곱을 반환한다.
 * @details 꼭짓점이 세 개보다 적으면 다각형이 아니므로 무한대를 반환한다.
 */
[[nodiscard]] double SquaredDistanceToPolygon(
    const glm::dvec2& point, std::span<const glm::dvec2> outline) noexcept;

/** @brief 점과 다각형 사이의 거리를 미터 단위로 반환한다. */
[[nodiscard]] double DistanceToPolygon(
    const glm::dvec2& point, std::span<const glm::dvec2> outline) noexcept;
}
