#include "simulation_core/CollisionValidation.h"

#include "geometry/Polygon.h"
#include "geometry/VectorMath.h"

#include <algorithm>
#include <cmath>

namespace dronesim::simulation::collision
{
namespace
{
bool IsValidFootprint(std::span<const glm::dvec2> outlineMeters) noexcept
{
    if (outlineMeters.size() < 3U
        || !std::all_of(outlineMeters.begin(), outlineMeters.end(),
            [](const glm::dvec2& point) { return geometry::IsFinite(point); }))
    {
        return false;
    }

    // 면적이 0인 외곽선은 건물 바닥이나 충돌 면을 만들 수 없다.
    const double area = geometry::SignedArea(outlineMeters);
    return std::isfinite(area) && std::abs(area) > 1.0e-9;
}
}

bool IsValidObstacleBox(
    const glm::dvec3& centerMeters,
    const glm::dvec3& halfExtentsMeters) noexcept
{
    return geometry::IsFinite(centerMeters)
        && geometry::IsFinite(halfExtentsMeters)
        && halfExtentsMeters.x > 0.0
        && halfExtentsMeters.y > 0.0
        && halfExtentsMeters.z > 0.0;
}

bool IsValidPrism(
    std::span<const glm::dvec2> outlineMeters,
    double bottomMeters,
    double topMeters) noexcept
{
    return IsValidFootprint(outlineMeters)
        && std::isfinite(bottomMeters)
        && std::isfinite(topMeters)
        && topMeters > bottomMeters;
}
}
