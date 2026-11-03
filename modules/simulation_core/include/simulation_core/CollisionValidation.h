#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <span>

namespace dronesim::simulation::collision
{
/** @brief 고정 상자의 위치와 반 크기가 유한하고 양수인지 확인한다. */
[[nodiscard]] bool IsValidObstacleBox(
    const glm::dvec3& centerMeters,
    const glm::dvec3& halfExtentsMeters) noexcept;

/** @brief 건물 외곽선과 바닥·꼭대기 높이가 유효한 프리즘인지 확인한다. */
[[nodiscard]] bool IsValidPrism(
    std::span<const glm::dvec2> outlineMeters,
    double bottomMeters,
    double topMeters) noexcept;
}
