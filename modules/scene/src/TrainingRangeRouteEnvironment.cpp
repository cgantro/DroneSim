#include "scene/TrainingRangeRouteEnvironment.h"

#include <algorithm>
#include <limits>

namespace dronesim::scene
{
TrainingRangeRouteEnvironment::TrainingRangeRouteEnvironment(
    const terrain::TerrainGrid& terrain,
    const std::vector<flight_planning::BuildingObstacle>& buildings)
    : terrain_(terrain)
{
    obstacles_.reserve(buildings.size());
    for (const auto& building : buildings)
    {
        if (building.outlineMeters.size() < 3U)
            continue;

        const double groundAltitude = building.groundElevationsMeters.empty()
            ? 0.0
            : *std::max_element(building.groundElevationsMeters.begin(),
                building.groundElevationsMeters.end());
        const double topAltitude = building.heightMeters > 0.0
            ? groundAltitude + building.heightMeters
            : std::numeric_limits<double>::infinity();
        obstacles_.push_back({building.outlineMeters, topAltitude});
    }
}

glm::dvec4 TrainingRangeRouteEnvironment::BoundsMeters() const noexcept
{
    const double halfWidth = terrain_.HalfWidthMeters();
    const double halfDepth = terrain_.HalfDepthMeters();
    return {-halfWidth, -halfDepth, halfWidth, halfDepth};
}

bool TrainingRangeRouteEnvironment::Contains(glm::dvec2 positionMeters) const noexcept
{
    return terrain_.ContainsLocalPositionMeters(positionMeters);
}

double TrainingRangeRouteEnvironment::GroundAltitudeMeters(
    glm::dvec2 positionMeters) const
{
    return terrain_.HeightAtLocalMeters(positionMeters);
}

std::span<const flight_planning::RouteObstacle>
TrainingRangeRouteEnvironment::Obstacles() const noexcept
{
    return obstacles_;
}
}
