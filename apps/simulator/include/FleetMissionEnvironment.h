#pragma once

#include "FleetMissionPlanner.h"
#include "TrainingRangeScene.h"
#include "scene/TrainingRangeRouteEnvironment.h"
#include "terrain/TerrainGrid.h"

#include <glm/vec2.hpp>

#include <vector>

namespace dronesim::simulator
{
/** @brief 장면 지형·건물 자료를 임무 경로와 정찰 대상으로 준비한다. */
class FleetMissionEnvironment final
{
public:
    FleetMissionEnvironment(const terrain::TerrainGrid& terrain,
        const scene::TrainingRangeScene& trainingScene);

    [[nodiscard]] const scene::TrainingRangeRouteEnvironment&
    RouteEnvironment() const noexcept;
    /** @brief 이 장면의 장애물과 고도 기준으로 경로를 계산하는 계획기를 돌려준다. */
    [[nodiscard]] const mission::FleetMissionPlanner& Planner() const noexcept;
    [[nodiscard]] const std::vector<mission::FleetReconnaissanceTarget>&
    Targets() const noexcept;
    /** @brief 편대가 정찰 임무를 시작하기 전에 모일 지점을 반환한다. */
    [[nodiscard]] const glm::dvec2& StagingPositionMeters() const noexcept;
    [[nodiscard]] double MinimumTransitAltitudeMeters() const noexcept;

private:
    /** @brief 경로에서 유지할 최소 순항 고도 [m]. */
    static constexpr double minimumTransitAltitudeMeters = 25.0;
    /** @brief 정찰 지점에 접근할 때 지면에서 띄울 높이 [m]. */
    static constexpr double targetApproachHeightMeters = 25.0;

    scene::TrainingRangeRouteEnvironment routeEnvironment_;
    mission::FleetMissionPlanner planner_;
    glm::dvec2 stagingPositionMeters_{0.0};
    std::vector<mission::FleetReconnaissanceTarget> targets_;
};
}
