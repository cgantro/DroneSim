#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <cmath>

namespace dronesim::geometry
{
/** @brief 2차원 외적의 z 성분을 계산한다. 부호로 두 방향의 회전 순서를 알 수 있다. */
[[nodiscard]] inline double Cross2D(
    const glm::dvec2& first, const glm::dvec2& second) noexcept
{
    return first.x * second.y - first.y * second.x;
}

/** @brief 세 점이 이루는 방향 있는 면적의 두 배를 계산한다. */
[[nodiscard]] inline double Cross2D(
    const glm::dvec2& origin,
    const glm::dvec2& first,
    const glm::dvec2& second) noexcept
{
    return Cross2D(first - origin, second - origin);
}

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

/**
 * @brief 벡터의 방향은 유지하면서 길이를 지정한 최댓값 이하로 제한한다.
 * @details 세 좌표를 제곱해 더하면 큰 입력에서 중간 계산이 먼저 넘칠 수 있다.
 *          std::hypot은 이를 줄여 벡터 길이를 구하고, 제한을 넘을 때만 비율을 곱한다.
 * @param value 제한할 벡터
 * @param maximumMagnitude 허용할 최대 길이. 0 이상의 값을 전달한다.
 */
[[nodiscard]] inline glm::dvec3 ClampMagnitude(
    const glm::dvec3& value, double maximumMagnitude) noexcept
{
    const double magnitude = std::hypot(value.x, value.y, value.z);
    if (magnitude <= maximumMagnitude || magnitude == 0.0)
        return value;

    return value * (maximumMagnitude / magnitude);
}
}
