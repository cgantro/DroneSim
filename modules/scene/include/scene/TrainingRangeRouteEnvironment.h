#pragma once

#include "flight_planning/RouteEnvironment.h"
#include "flight_planning/BuildingObstacle.h"
#include "terrain/TerrainGrid.h"

#include <vector>

namespace dronesim::scene
{
/** @brief 훈련장 지형·건물 자료를 경로 탐색용 지도 정보로 바꿔 제공한다. */
class TrainingRangeRouteEnvironment final
    : public flight_planning::RouteEnvironment
{
public:
    TrainingRangeRouteEnvironment(
        const terrain::TerrainGrid& terrain,
        const std::vector<flight_planning::BuildingObstacle>& buildings);

    [[nodiscard]] glm::dvec4 BoundsMeters() const noexcept override;
    [[nodiscard]] bool Contains(glm::dvec2 positionMeters) const noexcept override;
    [[nodiscard]] double GroundAltitudeMeters(
        glm::dvec2 positionMeters) const override;
    [[nodiscard]] std::span<const flight_planning::RouteObstacle>
    Obstacles() const noexcept override;

private:
    const terrain::TerrainGrid& terrain_;
    std::vector<flight_planning::RouteObstacle> obstacles_;
};
}
