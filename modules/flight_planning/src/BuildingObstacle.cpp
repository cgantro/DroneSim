#include "flight_planning/BuildingObstacle.h"
#include "geometry/Polygon.h"

#include <algorithm>
#include <cmath>

namespace dronesim::flight_planning
{
bool ContainsBuildingVolume(
    const BuildingObstacle& building, const glm::dvec3& positionMeters) noexcept
{
    // 높이와 평면 외곽선을 모두 만족해야 건물 충돌 부피 안으로 본다.
    if (building.outlineMeters.size() < 3U
        || !std::isfinite(positionMeters.x) || !std::isfinite(positionMeters.y)
        || !std::isfinite(positionMeters.z))
        return false;

    const double ground = building.groundElevationsMeters.empty()
        ? 0.0 : *std::min_element(building.groundElevationsMeters.begin(),
            building.groundElevationsMeters.end());
    return positionMeters.z >= ground
        && positionMeters.z <= ground + building.heightMeters
        && geometry::ContainsPointInPolygon(
            {positionMeters.x, positionMeters.y}, building.outlineMeters);
}
}
