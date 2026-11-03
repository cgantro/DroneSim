#include "FleetMissionEnvironment.h"

namespace dronesim::simulator
{
FleetMissionEnvironment::FleetMissionEnvironment(
    const terrain::TerrainGrid& terrain,
    const scene::TrainingRangeScene& trainingScene)
    : routeEnvironment_(terrain, trainingScene.BuildingObstacles())
{
    // 경로와 정찰 지점이 같은 장면 자료를 기준으로 삼도록 한 번에 준비한다.
    const auto& reconPoints = trainingScene.ReconPoints();
    targets_.reserve(reconPoints.size());
    for (const auto& point : reconPoints)
    {
        const double groundMeters = terrain.HeightAtLocalMeters(point.positionMeters);
        // 드론이 대상 지점에 닿지 않고도 위에서 관찰할 수 있게 지면보다 높게 둔다.
        targets_.push_back({point.id,
            {point.positionMeters, groundMeters + targetApproachHeightMeters}});
    }
}

const scene::TrainingRangeRouteEnvironment& FleetMissionEnvironment::RouteEnvironment() const noexcept
{
    return routeEnvironment_;
}

const std::vector<mission::FleetReconnaissanceTarget>&
FleetMissionEnvironment::Targets() const noexcept
{
    return targets_;
}

double FleetMissionEnvironment::MinimumTransitAltitudeMeters() const noexcept
{
    return minimumTransitAltitudeMeters;
}
}
