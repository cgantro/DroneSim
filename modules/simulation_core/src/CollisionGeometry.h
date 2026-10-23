#pragma once

#include "simulation_core/PhysicsWorld.h"

#include <array>
#include <optional>
#include <span>

namespace aerosimlab::simulation::detail
{
/** @brief 월드 좌표에 놓인 회전 상자와 빠른 후보 검사에 필요한 경계다. */
struct OrientedBox
{
    glm::dvec3 centerMeters;
    glm::dvec3 halfExtentsMeters;
    std::array<glm::dvec3, 3> axes;
    glm::dvec3 axisAlignedHalfExtentsMeters;
    double boundingRadiusMeters = 0.0;
};

/** @brief 첫 번째 상자 바깥을 향하는 접촉 방향과 가장 깊은 접촉점이다. */
struct BoxContact
{
    glm::dvec3 normal;
    glm::dvec3 impactPointMeters;
    double penetrationMeters = 0.0;
};

/** @brief 기체 기준 충돌 모양의 바깥 반경을 구한다. */
double CollisionRadius(std::span<const RigidBodyBox> boxes);

/** @brief 상자 표면에서 지정한 방향으로 가장 멀리 나온 꼭짓점을 구한다. */
glm::dvec3 SupportPoint(const OrientedBox& box, const glm::dvec3& direction);

/** @brief 강체 기준 상자를 위치와 자세를 적용해 월드 좌표로 바꾼다. */
OrientedBox ToWorldBox(
    const glm::dvec3& bodyPosition,
    const glm::dquat& bodyOrientation,
    const RigidBodyBox& box);

/**
 * @brief 두 회전 상자의 겹침, 바깥쪽 방향과 기체 쪽 접촉점을 찾는다.
 * @details 분리축 정리는 상자 면 방향과 두 상자의 모서리 방향을 확인한다. 첫 번째 상자의 법선은 두 번째 상자에서 바깥을 향한다.
 */
std::optional<BoxContact> FindBoxContact(
    const OrientedBox& firstBox,
    const OrientedBox& secondBox);
}
