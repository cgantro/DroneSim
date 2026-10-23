#include "CollisionGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aerosimlab::simulation::detail
{
namespace
{
/** @brief 회전 상자를 둘러싸는 월드 축 정렬 상자의 반크기를 구한다. */
glm::dvec3 AxisAlignedHalfExtents(
    const std::array<glm::dvec3, 3>& axes, const glm::dvec3& halfExtents)
{
    return {
        std::abs(axes[0].x) * halfExtents.x
            + std::abs(axes[1].x) * halfExtents.y
            + std::abs(axes[2].x) * halfExtents.z,
        std::abs(axes[0].y) * halfExtents.x
            + std::abs(axes[1].y) * halfExtents.y
            + std::abs(axes[2].y) * halfExtents.z,
        std::abs(axes[0].z) * halfExtents.x
            + std::abs(axes[1].z) * halfExtents.y
            + std::abs(axes[2].z) * halfExtents.z};
}
}

double CollisionRadius(std::span<const RigidBodyBox> boxes)
{
    double radius = 0.0;
    for (const RigidBodyBox& box : boxes)
    {
        radius = std::max(radius, glm::length(box.positionMeters)
            + glm::length(glm::dvec3{box.halfExtentsMeters}));
    }
    return radius;
}

bool BroadphaseMayOverlap(
    const glm::dvec3& firstCenter,
    double firstRadius,
    const glm::dvec3& secondCenter,
    double secondRadius) noexcept
{
    const double combinedRadius = firstRadius + secondRadius;
    const glm::dvec3 centerOffset = firstCenter - secondCenter;
    return glm::dot(centerOffset, centerOffset) <= combinedRadius * combinedRadius;
}

glm::dvec3 SupportPoint(const OrientedBox& box, const glm::dvec3& direction)
{
    glm::dvec3 point = box.centerMeters;
    for (glm::length_t axis = 0; axis < 3; ++axis)
    {
        const double side = glm::dot(direction, box.axes[axis]) >= 0.0 ? 1.0 : -1.0;
        point += box.axes[axis] * (side * box.halfExtentsMeters[axis]);
    }
    return point;
}

OrientedBox ToWorldBox(
    const glm::dvec3& bodyPosition,
    const glm::dquat& bodyOrientation,
    const RigidBodyBox& box)
{
    const glm::dquat orientation = glm::normalize(bodyOrientation * box.orientation);
    const glm::dvec3 halfExtents{box.halfExtentsMeters};
    const std::array<glm::dvec3, 3> axes{
        orientation * glm::dvec3{1.0, 0.0, 0.0},
        orientation * glm::dvec3{0.0, 1.0, 0.0},
        orientation * glm::dvec3{0.0, 0.0, 1.0}};
    return {
        bodyPosition + bodyOrientation * box.positionMeters,
        halfExtents,
        axes,
        AxisAlignedHalfExtents(axes, halfExtents),
        glm::length(halfExtents)};
}

std::optional<BoxContact> FindBoxContact(
    const OrientedBox& firstBox,
    const OrientedBox& secondBox)
{
    const glm::dvec3 centerOffset = secondBox.centerMeters - firstBox.centerMeters;
    // 외접구와 바깥 AABB로 먼저 거르면 서로 떨어진 상자의 세부 검사를 건너뛴다.
    const double combinedBoundingRadius =
        firstBox.boundingRadiusMeters + secondBox.boundingRadiusMeters;
    if (glm::dot(centerOffset, centerOffset)
        > combinedBoundingRadius * combinedBoundingRadius)
    {
        return std::nullopt;
    }

    for (glm::length_t axis = 0; axis < 3; ++axis)
    {
        if (std::abs(centerOffset[axis])
            > firstBox.axisAlignedHalfExtentsMeters[axis]
                + secondBox.axisAlignedHalfExtentsMeters[axis])
        {
            return std::nullopt;
        }
    }

    // 상자의 면 6개와 서로 다른 상자의 모서리 방향 최대 9개를 모두 검사한다.
    std::array<glm::dvec3, 15> candidateAxes{};
    std::size_t axisCount = 0;
    for (const glm::dvec3& axis : firstBox.axes)
        candidateAxes[axisCount++] = axis;
    for (const glm::dvec3& axis : secondBox.axes)
        candidateAxes[axisCount++] = axis;
    for (const glm::dvec3& firstAxis : firstBox.axes)
    {
        for (const glm::dvec3& secondAxis : secondBox.axes)
            candidateAxes[axisCount++] = glm::cross(firstAxis, secondAxis);
    }

    double smallestPenetration = std::numeric_limits<double>::infinity();
    glm::dvec3 collisionNormal{0.0};
    for (std::size_t axisIndex = 0; axisIndex < axisCount; ++axisIndex)
    {
        const glm::dvec3& candidate = candidateAxes[axisIndex];
        const double axisLengthSquared = glm::dot(candidate, candidate);
        if (axisLengthSquared < 1.0e-12)
            continue;

        const glm::dvec3 axis = candidate / std::sqrt(axisLengthSquared);
        const auto projectedRadius = [&axis](const OrientedBox& box)
        {
            return std::abs(glm::dot(axis, box.axes[0])) * box.halfExtentsMeters.x
                + std::abs(glm::dot(axis, box.axes[1])) * box.halfExtentsMeters.y
                + std::abs(glm::dot(axis, box.axes[2])) * box.halfExtentsMeters.z;
        };
        const double centerDistance = glm::dot(centerOffset, axis);
        const double penetration = projectedRadius(firstBox)
            + projectedRadius(secondBox) - std::abs(centerDistance);
        if (penetration < 0.0)
            return std::nullopt;

        if (penetration < smallestPenetration)
        {
            smallestPenetration = penetration;
            collisionNormal = axis * (centerDistance >= 0.0 ? -1.0 : 1.0);
        }
    }

    if (!std::isfinite(smallestPenetration))
        return std::nullopt;

    return BoxContact{
        collisionNormal,
        SupportPoint(firstBox, -collisionNormal),
        smallestPenetration};
}
}
