#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

namespace dronesim::graphics
{
/** @brief ENU 위치를 Y축이 위인 렌더 좌표로 바꾼다. */
inline glm::vec3 EnuPositionToRender(const glm::dvec3& eastNorthUpMeters) noexcept
{
    // 장면의 동쪽은 화면 X, 위쪽은 Y, 북쪽은 화면의 -Z가 되도록 축을 맞춘다.
    return {static_cast<float>(eastNorthUpMeters.x),
        static_cast<float>(eastNorthUpMeters.z),
        static_cast<float>(-eastNorthUpMeters.y)};
}

/** @brief ENU 방향 벡터를 Y축이 위인 렌더 좌표로 바꾼다. */
inline glm::vec3 EnuDirectionToRender(const glm::dvec3& eastNorthUp) noexcept
{
    return {static_cast<float>(eastNorthUp.x),
        static_cast<float>(eastNorthUp.z), static_cast<float>(-eastNorthUp.y)};
}

/** @brief ENU 기체 자세를 Y축이 위인 렌더 좌표의 회전으로 바꾼다. */
inline glm::quat EnuOrientationToRender(const glm::dquat& orientation) noexcept
{
    constexpr float halfPi = 1.57079632679F;
    const glm::quat enuOrientation{
        static_cast<float>(orientation.w), static_cast<float>(orientation.x),
        static_cast<float>(orientation.y), static_cast<float>(orientation.z)};
    const glm::quat enuToRender = glm::angleAxis(
        -halfPi, glm::vec3{1.0F, 0.0F, 0.0F});

    // 축 기준을 바꾸는 회전은 자세 양쪽에 적용해야 결과가 같은 물리 회전을 가리킨다.
    return glm::normalize(
        enuToRender * enuOrientation * glm::inverse(enuToRender));
}
}
