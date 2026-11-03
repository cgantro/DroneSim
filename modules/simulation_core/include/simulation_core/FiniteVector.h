#pragma once

#include <glm/vec3.hpp>

#include <cmath>

namespace dronesim::simulation
{
/** @brief 3차원 벡터의 모든 좌표가 NaN이나 무한대가 아닌지 확인한다. */
[[nodiscard]] inline bool IsFiniteVector(const glm::dvec3& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}
}
