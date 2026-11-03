#pragma once

#include "flight_planning/RouteEnvironment.h"

#include <optional>
#include <vector>

#include <glm/vec3.hpp>

namespace dronesim::flight_planning
{
/** @brief 경로를 찾을 때 적용할 안전 여유와 이동 고도다. */
struct RoutePlanningOptions
{
    /** @brief 장애물 외곽에서 유지할 수평 거리 [m] */
    double obstacleClearanceMeters = 3.0;
    /** @brief 경로 중간에서 유지할 최소 고도 [m] */
    double minimumTransitAltitudeMeters = 0.0;
};

/** @brief 지도 정보 제공자와 분리된 격자 기반 경로 탐색기다. */
class RoutePlanner
{
public:
    /** @brief 지도 구현체가 제공한 경계·지면·장애물 정보로 비행 경로를 찾는다.
     *  @details 수평 경로는 8방향 A*로 찾고, 장애물에 닿지 않는 직선 구간을
     *           이어 경유점을 줄인다. 마지막에 지면 높이와 장애물 높이를 반영해
     *           비행 고도를 정한다. 지도 자료는 RouteEnvironment 구현체가 제공한다.
     *  @param environment 탐색 영역, 지면 높이, 장애물 외곽선을 제공하는 지도 정보
     *  @param startMeters 출발점의 ENU 위치 [m]
     *  @param goalMeters 목표점의 ENU 위치 [m]
     *  @param options 장애물 간격과 최소 비행 고도
     *  @return 출발점은 제외하고 목표점을 포함한 경유점. 경로가 없으면 빈 값을 반환한다.
     */
    [[nodiscard]] std::optional<std::vector<glm::dvec3>> Plan(
        const RouteEnvironment& environment,
        const glm::dvec3& startMeters,
        const glm::dvec3& goalMeters,
        const RoutePlanningOptions& options = {}) const;
};
}
