#pragma once

#include "flight_planning/BuildingObstacle.h"
#include "terrain/TerrainGrid.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace dronesim::flight_planning
{
/** @brief 리더가 따라갈 경로와 경로에 맞춰 고른 대형·드론 간격이다. */
struct FormationRoutePlan
{
    glm::dvec3 leaderStartMeters{0.0};
    std::vector<glm::dvec3> leaderWaypointsMeters;
    double requiredClearanceMeters = 0.0;
    double spacingMeters = 8.0;
};

/**
 * @brief 현재 이동 방향에 맞춘 리더 기준 편대 위치를 계산한다.
 * @param droneCount 편대 드론 수. 첫 번째 위치는 리더다.
 * @param spacingMeters 드론 사이 목표 간격 [m]
 * @param headingRadians 동쪽을 0으로 하고 반시계 방향으로 잰 진행 방향 [rad]
 */
[[nodiscard]] std::vector<glm::dvec2> FormationOffsets(
    std::size_t droneCount, double spacingMeters, double headingRadians);

/**
 * @brief 편대 전체가 통과할 수 있도록 장애물 여유를 넓혀 리더의 2.5D 경로를 찾는다.
 * @details 요청한 대형이 지나갈 경로가 없으면 빈 결과를 반환한다.
 *          반환 경로는 시작점을 빼고 목적지를 포함한다.
 */
[[nodiscard]] std::optional<FormationRoutePlan> PlanFormationRoute(
    const terrain::TerrainGrid& terrain,
    const std::vector<BuildingObstacle>& obstacles,
    const glm::dvec3& leaderStartMeters,
    const glm::dvec3& destinationMeters,
    std::size_t droneCount,
    double spacingMeters,
    double droneRadiusMeters,
    double safetyMarginMeters,
    double minimumTransitAltitudeMeters = 0.0);
}
