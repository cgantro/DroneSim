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

bool IsValidFootprint(std::span<const glm::dvec2> outlineMeters) noexcept
{
    if (outlineMeters.size() < 3U)
        return false;
    double twiceArea = 0.0;
    for (std::size_t i = 0; i < outlineMeters.size(); ++i)
    {
        const glm::dvec2& current = outlineMeters[i];
        const glm::dvec2& next = outlineMeters[(i + 1U) % outlineMeters.size()];
        if (!std::isfinite(current.x) || !std::isfinite(current.y))
            return false;
        twiceArea += current.x * next.y - next.x * current.y;
    }
    return std::isfinite(twiceArea) && std::abs(twiceArea) > 1.0e-9;
}

std::optional<PrismContact> FindPrismContact(
    std::span<const glm::dvec2> outlineMeters,
    double bottomMeters, double topMeters,
    const glm::dvec3& centerMeters,
    double horizontalRadiusMeters, double verticalRadiusMeters)
{
    if (!IsValidFootprint(outlineMeters) || !std::isfinite(bottomMeters)
        || !std::isfinite(topMeters) || !std::isfinite(centerMeters.x)
        || !std::isfinite(centerMeters.y) || !std::isfinite(centerMeters.z)
        || !(topMeters > bottomMeters) || !(horizontalRadiusMeters > 0.0)
        || !(verticalRadiusMeters > 0.0))
        return std::nullopt;

    const double bodyBottom = centerMeters.z - verticalRadiusMeters;
    const double bodyTop = centerMeters.z + verticalRadiusMeters;
    const double verticalOverlap = std::min(bodyTop, topMeters)
        - std::max(bodyBottom, bottomMeters);
    if (verticalOverlap <= 0.0)
        return std::nullopt;

    const double distanceToMoveDown = bodyTop - bottomMeters;
    const double distanceToMoveUp = topMeters - bodyBottom;
    const bool moveUp = distanceToMoveUp < distanceToMoveDown;
    const double verticalPenetration = centerMeters.z < bottomMeters
        || centerMeters.z > topMeters
        ? verticalOverlap
        : std::min(distanceToMoveDown, distanceToMoveUp);

    const glm::dvec2 center{centerMeters.x, centerMeters.y};
    bool inside = false;
    double twiceArea = 0.0;
    for (std::size_t i = 0; i < outlineMeters.size(); ++i)
    {
        const glm::dvec2& a = outlineMeters[i];
        const glm::dvec2& b = outlineMeters[(i + 1U) % outlineMeters.size()];
        twiceArea += a.x * b.y - b.x * a.y;
        const glm::dvec2& previous = outlineMeters[
            (i + outlineMeters.size() - 1U) % outlineMeters.size()];
        if ((a.y > center.y) != (previous.y > center.y)
            && center.x < (previous.x - a.x) * (center.y - a.y)
                / (previous.y - a.y) + a.x)
            inside = !inside;
    }

    double nearestDistanceSquared = std::numeric_limits<double>::infinity();
    glm::dvec2 nearestPoint{0.0};
    glm::dvec2 nearestNormal{0.0};
    for (std::size_t i = 0; i < outlineMeters.size(); ++i)
    {
        const glm::dvec2& a = outlineMeters[i];
        const glm::dvec2& b = outlineMeters[(i + 1U) % outlineMeters.size()];
        const glm::dvec2 edge = b - a;
        const double edgeLengthSquared = glm::dot(edge, edge);
        if (edgeLengthSquared <= 1.0e-12)
            continue;
        const double fraction = std::clamp(
            glm::dot(center - a, edge) / edgeLengthSquared, 0.0, 1.0);
        const glm::dvec2 point = a + edge * fraction;
        const glm::dvec2 offset = center - point;
        const double distanceSquared = glm::dot(offset, offset);
        if (distanceSquared >= nearestDistanceSquared)
            continue;
        nearestDistanceSquared = distanceSquared;
        nearestPoint = point;
        const double edgeLength = std::sqrt(edgeLengthSquared);
        nearestNormal = twiceArea >= 0.0
            ? glm::dvec2{edge.y, -edge.x} / edgeLength
            : glm::dvec2{-edge.y, edge.x} / edgeLength;
        if (distanceSquared > 1.0e-12 && !inside)
            nearestNormal = offset / std::sqrt(distanceSquared);
    }
    if (!std::isfinite(nearestDistanceSquared))
        return std::nullopt;

    const double distance = std::sqrt(nearestDistanceSquared);
    const double horizontalPenetration = inside
        ? horizontalRadiusMeters + distance
        : horizontalRadiusMeters - distance;
    if (horizontalPenetration <= 0.0)
        return std::nullopt;

    PrismContact result;
    if (verticalPenetration < horizontalPenetration)
    {
        result.normal = {0.0, 0.0, moveUp ? 1.0 : -1.0};
        result.penetrationMeters = verticalPenetration;
        result.impactPointMeters = {center.x, center.y,
            result.normal.z > 0.0 ? topMeters : bottomMeters};
    }
    else
    {
        result.normal = {nearestNormal.x, nearestNormal.y, 0.0};
        result.penetrationMeters = horizontalPenetration;
        result.impactPointMeters = {nearestPoint.x, nearestPoint.y,
            std::clamp(centerMeters.z, bottomMeters, topMeters)};
    }
    return result;
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
