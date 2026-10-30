#pragma once

#include "flight_planning/BuildingObstacle.h"
#include "terrain/TerrainGrid.h"

#include <optional>
#include <vector>

namespace dronesim::flight_planning
{
/**
 * @brief 지형과 건물 배치를 고려해 출발점에서 목표점까지의 경로를 찾는다.
 * @details 수평 경로는 5m 격자에서 8방향 A* 탐색으로 찾고, 장애물 없이
 *          직선으로 연결할 수 있는 구간을 합쳐 경유점을 줄인다. A*는
 *          지금까지 이동한 비용 g와 목표까지의 예상 비용 h를 더한
 *          f = g + h가 가장 작은 칸부터 살펴보는 탐색 방법이다.
 *          이 함수는 수직 이륙 위치와 지형·건물 높이를 반영한 비행 고도도
 *          경로에 포함한다.
 * @param terrain 높이와 탐색 영역을 제공하는 지형 격자
 * @param buildings 수평 경로를 막는 건물 외곽선과 높이
 * @param startMeters 출발 위치(미터)
 * @param goalMeters 목표 위치(미터)
 * @param buildingClearanceMeters 건물 외곽에서 유지할 수평 안전거리(미터)
 * @param minimumTransitAltitudeMeters 이동 중 유지할 최소 고도(미터)
 * @return 출발점은 뺀 경유점 목록. 경로를 찾지 못하면 빈 값을 반환한다.
 */
[[nodiscard]] std::optional<std::vector<glm::dvec3>> PlanRoute(
    const terrain::TerrainGrid& terrain,
    const std::vector<BuildingObstacle>& buildings,
    const glm::dvec3& startMeters,
    const glm::dvec3& goalMeters,
    double buildingClearanceMeters = 3.0,
    double minimumTransitAltitudeMeters = 0.0);

}
