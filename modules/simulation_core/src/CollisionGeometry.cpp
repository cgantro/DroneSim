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

namespace
{
struct FootprintProjection
{
    bool inside = false;
    glm::dvec2 nearestPoint{0.0};
    glm::dvec2 outwardNormal{0.0};
    double distanceMeters = 0.0;
};

std::optional<FootprintProjection> ProjectToFootprint(
    std::span<const glm::dvec2> outlineMeters,
    const glm::dvec2& centerMeters)
{
    bool inside = false;
    double twiceArea = 0.0;
    double nearestDistanceSquared = std::numeric_limits<double>::infinity();
    glm::dvec2 nearestPoint{0.0};
    glm::dvec2 outwardNormal{0.0};

    for (std::size_t index = 0; index < outlineMeters.size(); ++index)
    {
        const glm::dvec2& first = outlineMeters[index];
        const glm::dvec2& second = outlineMeters[(index + 1U) % outlineMeters.size()];
        twiceArea += first.x * second.y - second.x * first.y;
    }

    for (std::size_t index = 0; index < outlineMeters.size(); ++index)
    {
        const glm::dvec2& first = outlineMeters[index];
        const glm::dvec2& second = outlineMeters[(index + 1U) % outlineMeters.size()];
        const glm::dvec2 edge = second - first;
        const double edgeLengthSquared = glm::dot(edge, edge);

        const glm::dvec2& previous = outlineMeters[
            (index + outlineMeters.size() - 1U) % outlineMeters.size()];
        if ((first.y > centerMeters.y) != (previous.y > centerMeters.y)
            && centerMeters.x < (previous.x - first.x)
                * (centerMeters.y - first.y) / (previous.y - first.y) + first.x)
        {
            inside = !inside;
        }

        if (edgeLengthSquared <= 1.0e-12)
            continue;

        const double fraction = std::clamp(
            glm::dot(centerMeters - first, edge) / edgeLengthSquared, 0.0, 1.0);
        const glm::dvec2 point = first + edge * fraction;
        const glm::dvec2 offset = centerMeters - point;
        const double distanceSquared = glm::dot(offset, offset);
        if (distanceSquared >= nearestDistanceSquared)
            continue;

        nearestDistanceSquared = distanceSquared;
        nearestPoint = point;
        const double edgeLength = std::sqrt(edgeLengthSquared);
        outwardNormal = twiceArea >= 0.0
            ? glm::dvec2{edge.y, -edge.x} / edgeLength
            : glm::dvec2{-edge.y, edge.x} / edgeLength;
    }

    if (!std::isfinite(nearestDistanceSquared))
        return std::nullopt;

    const double distance = std::sqrt(nearestDistanceSquared);
    if (!inside && distance > 1.0e-12)
        outwardNormal = (centerMeters - nearestPoint) / distance;
    return FootprintProjection{inside, nearestPoint, outwardNormal, distance};
}
}

std::optional<FootprintContact> FindFootprintContact(
    std::span<const glm::dvec2> outlineMeters,
    const glm::dvec2& centerMeters,
    double horizontalRadiusMeters)
{
    if (!IsValidFootprint(outlineMeters)
        || !std::isfinite(centerMeters.x) || !std::isfinite(centerMeters.y)
        || !std::isfinite(horizontalRadiusMeters) || !(horizontalRadiusMeters > 0.0))
        return std::nullopt;

    const auto projection = ProjectToFootprint(outlineMeters, centerMeters);
    if (!projection)
        return std::nullopt;

    const double penetrationMeters = projection->inside
        ? horizontalRadiusMeters + projection->distanceMeters
        : horizontalRadiusMeters - projection->distanceMeters;
    if (penetrationMeters <= 0.0)
        return std::nullopt;

    return FootprintContact{
        projection->outwardNormal, projection->nearestPoint, penetrationMeters};
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

    const auto projection = ProjectToFootprint(
        outlineMeters, {centerMeters.x, centerMeters.y});
    if (!projection)
        return std::nullopt;

    const double horizontalPenetration = projection->inside
        ? horizontalRadiusMeters + projection->distanceMeters
        : horizontalRadiusMeters - projection->distanceMeters;
    if (horizontalPenetration <= 0.0)
        return std::nullopt;

    PrismContact result;
    if (verticalPenetration < horizontalPenetration)
    {
        result.normal = {0.0, 0.0, moveUp ? 1.0 : -1.0};
        result.penetrationMeters = verticalPenetration;
        result.impactPointMeters = {centerMeters.x, centerMeters.y,
            result.normal.z > 0.0 ? topMeters : bottomMeters};
    }
    else
    {
        result.normal = {
            projection->outwardNormal.x, projection->outwardNormal.y, 0.0};
        result.penetrationMeters = horizontalPenetration;
        result.impactPointMeters = {
            projection->nearestPoint.x, projection->nearestPoint.y,
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
