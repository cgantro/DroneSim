#pragma once

#include "FleetMissionPlanner.h"
#include "TrainingRangeScene.h"
#include "scene/TrainingRangeRouteEnvironment.h"
#include "terrain/TerrainGrid.h"

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
    [[nodiscard]] const std::vector<mission::FleetReconnaissanceTarget>&
    Targets() const noexcept;
    [[nodiscard]] double MinimumTransitAltitudeMeters() const noexcept;

private:
    /** @brief 경로에서 유지할 최소 순항 고도 [m]. */
    static constexpr double minimumTransitAltitudeMeters = 25.0;
    /** @brief 정찰 지점에 접근할 때 지면에서 띄울 높이 [m]. */
    static constexpr double targetApproachHeightMeters = 25.0;

    scene::TrainingRangeRouteEnvironment routeEnvironment_;
    std::vector<mission::FleetReconnaissanceTarget> targets_;
};
}
