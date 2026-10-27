#pragma once

#include "terrain/MapBuildings.h"

#include <optional>
#include <vector>

namespace aerosimlab::terrain
{
/** @brief 출발 위치에서 도달할 수 있는 외벽과 촬영 접근 위치다. */
struct MapBuildingFacadeRoute
{
    MapBuildingFacade facade;
    glm::dvec3 approachPositionMeters{0.0};
    std::vector<glm::dvec3> waypointsMeters;
};

/**
 * @brief 현재 위치에서 목표까지 안전하게 지나갈 경유점을 만든다.
 * @details 확인된 높이의 건물은 우회한다. 외곽선만 있는 건물은 6m 임시 높이로 보고,
 *          그보다 2m 높게 통과한다. 중간 경유점은 5m 격자에서 찾고,
 *          시작 위치의 수직 상승 지점과 마지막 목표 위치를 경로에 포함한다.
 * @return 시작점은 제외하고 목표점을 포함한 경로. 통로가 없으면 빈 값을 반환한다.
 */
[[nodiscard]] std::optional<std::vector<glm::dvec3>> PlanMapFlightRoute(
    const TerrainGrid& terrain,
    const std::vector<MapBuilding>& buildings,
    const glm::dvec3& startMeters,
    const glm::dvec3& goalMeters,
    double buildingClearanceMeters = 3.0,
    double minimumTransitAltitudeMeters = 0.0);

/** @brief 건물 둘레를 살펴 출발 위치에서 닿을 수 있는 외벽 경로를 찾는다. */
[[nodiscard]] std::optional<MapBuildingFacadeRoute> PlanMapFlightRouteToFacade(
    const TerrainGrid& terrain,
    const std::vector<MapBuilding>& buildings,
    const MapBuilding& targetBuilding,
    const glm::dvec3& startMeters,
    double preferredDistanceMeters,
    double maximumDistanceMeters,
    double approachStepMeters = 2.0,
    double buildingClearanceMeters = 3.0,
    double minimumTransitAltitudeMeters = 0.0);
}
