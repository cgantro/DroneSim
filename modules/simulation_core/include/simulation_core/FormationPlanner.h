#pragma once

#include "terrain/MapBuildings.h"
#include "terrain/TerrainGrid.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace aerosimlab::simulation
{
/** @brief 편대 이동 중 사용할 드론 배치 방식이다. */
enum class FormationShape
{
    Vee,
    Column,
    Line
};

/** @brief 리더가 따라갈 장애물 회피 경로와 그때 선택한 편대 대형이다. */
struct FormationRoutePlan
{
    FormationShape shape = FormationShape::Vee;
    std::vector<glm::dvec3> leaderWaypointsMeters;
    double requiredClearanceMeters = 0.0;
};

/**
 * @brief 현재 이동 방향에 맞춘 리더 기준 편대 위치를 계산한다.
 * @param shape 편대 배치 방식
 * @param droneCount 편대 드론 수. 첫 번째 위치는 리더다.
 * @param spacingMeters 드론 사이 목표 간격 [m]
 * @param headingRadians 동쪽을 0으로 하고 반시계 방향으로 잰 진행 방향 [rad]
 */
[[nodiscard]] std::vector<glm::dvec2> FormationOffsets(
    FormationShape shape, std::size_t droneCount,
    double spacingMeters, double headingRadians);

/**
 * @brief 편대 전체가 통과할 수 있도록 장애물 여유를 넓혀 리더의 2.5D 경로를 찾는다.
 * @details 요청한 V자 경로를 찾지 못하면 폭이 좁은 일렬 대형으로 다시 찾는다.
 *          반환 경로에는 시작 위치와 목적지가 모두 들어간다.
 */
[[nodiscard]] std::optional<FormationRoutePlan> PlanFormationRoute(
    const terrain::TerrainGrid& terrain,
    const std::vector<terrain::MapBuilding>& obstacles,
    const glm::dvec3& leaderStartMeters,
    const glm::dvec3& destinationMeters,
    std::size_t droneCount,
    FormationShape requestedShape,
    double spacingMeters,
    double droneRadiusMeters,
    double safetyMarginMeters,
    double minimumTransitAltitudeMeters = 0.0);
}
