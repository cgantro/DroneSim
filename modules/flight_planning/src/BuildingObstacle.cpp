#include "flight_planning/BuildingObstacle.h"

#include <algorithm>
#include <cmath>

namespace dronesim::flight_planning
{
namespace
{
bool ContainsPoint(const std::vector<glm::dvec2>& outline,
    const glm::dvec2& point) noexcept
{
    // 다각형 변을 가로지르는 횟수의 홀짝으로 점이 내부인지 판정한다.
    bool inside = false;
    for (std::size_t first = 0, previous = outline.size() - 1U;
         first < outline.size(); previous = first++)
    {
        const glm::dvec2& a = outline[first];
        const glm::dvec2& b = outline[previous];
        const bool crosses = (a.y > point.y) != (b.y > point.y);
        if (crosses && point.x < (b.x - a.x) * (point.y - a.y)
                / (b.y - a.y) + a.x)
            inside = !inside;
    }
    return inside;
}
}

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
        && ContainsPoint(building.outlineMeters,
            {positionMeters.x, positionMeters.y});
}
}
