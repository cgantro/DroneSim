#pragma once

#include "flight_planning/BuildingObstacle.h"
#include "terrain/TerrainGrid.h"

#include <optional>
#include <vector>

namespace dronesim::flight_planning
{
/**
 * @brief 지형과 건물 배치를 고려해 출발점에서 목표점까지의 경로를 찾는다.
 * @details 경로는 시작점 뒤의 수직 상승 위치부터 목표점까지 차례로 담는다.
 * @return 시작점은 제외하고 목표점을 포함한 경로. 통로가 없으면 빈 값을 반환한다.
 */
[[nodiscard]] std::optional<std::vector<glm::dvec3>> PlanRoute(
    const terrain::TerrainGrid& terrain,
    const std::vector<BuildingObstacle>& buildings,
    const glm::dvec3& startMeters,
    const glm::dvec3& goalMeters,
    double buildingClearanceMeters = 3.0,
    double minimumTransitAltitudeMeters = 0.0);

}
