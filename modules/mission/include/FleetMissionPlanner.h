#pragma once

#include "FleetMission.h"

#include <span>
#include <string>
#include <vector>

namespace dronesim::mission
{
/** @brief 정찰 임무를 지상에서 시작할지 비행 중 이어서 시작할지 정한다. */
enum class FleetMissionStartMode
{
    TakeOffFromGround,
    ContinueInFlight
};

/** @brief 지도에서 정찰 대상으로 쓸 이름과 촬영 접근 위치다. */
struct FleetReconnaissanceTarget
{
    std::string displayName;
    glm::dvec3 approachPositionMeters{0.0};
};

/** @brief 드론의 현재 위치와 임무가 끝난 뒤 돌아갈 출발 위치다. */
struct FleetReconnaissanceStart
{
    simulation::DroneId droneId = 0;
    glm::dvec3 currentPositionMeters{0.0};
    glm::dvec3 homePositionMeters{0.0};
};

/** @brief 정찰 대상과 경로를 배정받은 드론별 임무 자료다. */
struct FleetReconnaissanceAssignment
{
    simulation::DroneId droneId = 0;
    FleetReconnaissanceState mission;
};

/**
 * @brief 지도 탐색 결과를 드론별 정찰 경로와 임무 상태로 준비한다.
 * @details 지형·건물 자료는 RouteEnvironment가 제공하며, 이 객체는 대상 분배,
 *          안전 여유, 최소 비행 고도를 정찰 임무 규칙으로 적용한다.
 */
class FleetMissionPlanner
{
public:
    FleetMissionPlanner(
        const flight_planning::RouteEnvironment& environment,
        double minimumTransitAltitudeMeters) noexcept;

    /** @brief 편대 이동 규칙을 적용해 한 구간의 안전 경로를 찾는다. */
    [[nodiscard]] std::optional<std::vector<glm::dvec3>> PlanTransitRoute(
        const glm::dvec3& startPositionMeters,
        const glm::dvec3& targetPositionMeters) const;

    /** @brief 출발 위치보다 5m 높게 접근한 뒤 기지로 내려오는 복귀 경로를 만든다. */
    [[nodiscard]] std::optional<std::vector<glm::dvec3>> PlanReturnRoute(
        const glm::dvec3& currentPositionMeters,
        const glm::dvec3& homePositionMeters) const;

    /**
     * @brief 드론마다 정찰 대상을 순환 배정하고 경로를 만든다.
     * @return 입력이 비었으면 빈 목록, 경로를 하나라도 만들지 못하면 빈 값이다.
     */
    [[nodiscard]] std::optional<std::vector<FleetReconnaissanceAssignment>>
    PlanReconnaissance(
        std::span<const FleetReconnaissanceStart> starts,
        std::span<const FleetReconnaissanceTarget> targets,
        FleetMissionStartMode startMode) const;

private:
    static constexpr double returnApproachHeightMeters = 5.0;

    const flight_planning::RouteEnvironment& environment_;
    double minimumTransitAltitudeMeters_;
};
}
