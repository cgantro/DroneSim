#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cmath>

namespace dronesim::geometry
{
/** @brief 벡터의 모든 좌표가 유한한 수인지 확인한다. */
[[nodiscard]] inline bool IsFinite(const glm::dvec2& value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y);
}

/** @brief 벡터의 모든 좌표가 유한한 수인지 확인한다. */
[[nodiscard]] inline bool IsFinite(const glm::dvec3& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}

/** @brief 벡터의 모든 좌표가 유한한 수인지 확인한다. */
[[nodiscard]] inline bool IsFinite(const glm::vec3& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}

/** @brief 벡터의 모든 좌표가 유한한 수인지 확인한다. */
[[nodiscard]] inline bool IsFinite(const glm::dvec4& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z)
        && std::isfinite(value.w);
}
}
