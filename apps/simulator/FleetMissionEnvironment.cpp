#include "FleetMissionEnvironment.h"

#include "geometry/Polygon.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace dronesim::simulator
{
namespace
{
double DistanceToBuildings(
    const glm::dvec2& positionMeters,
    const std::vector<flight_planning::BuildingObstacle>& buildings)
{
    double nearestDistanceMeters = std::numeric_limits<double>::infinity();
    for (const auto& building : buildings)
    {
        const double distanceMeters = geometry::DistanceToPolygon(
            positionMeters, building.outlineMeters);
        if (distanceMeters == 0.0)
            return 0.0;
        nearestDistanceMeters = std::min(nearestDistanceMeters, distanceMeters);
    }
    return nearestDistanceMeters;
}

/** @brief 건물과 다른 지점에서 떨어진 위치 중 기준점과 가장 가까운 곳을 찾는다. */
glm::dvec2 FindOpenPosition(
    const glm::dvec2& preferredPositionMeters,
    const glm::dvec2& halfMapSizeMeters,
    const std::vector<flight_planning::BuildingObstacle>& buildings,
    const std::vector<glm::dvec2>& reservedPositionsMeters,
    double minimumBuildingClearanceMeters,
    double minimumPointSeparationMeters,
    double maximumSearchDistanceMeters)
{
    // 지도를 5m 간격으로 훑어 장애물과 다른 기준점에서 떨어진 후보만 남긴다.
    constexpr double searchStepMeters = 5.0;
    glm::dvec2 closestPositionMeters{0.0};
    double closestDistanceMeters = std::numeric_limits<double>::infinity();

    for (double north = -halfMapSizeMeters.y + searchStepMeters;
        north <= halfMapSizeMeters.y - searchStepMeters;
        north += searchStepMeters)
    {
        for (double east = -halfMapSizeMeters.x + searchStepMeters;
            east <= halfMapSizeMeters.x - searchStepMeters;
            east += searchStepMeters)
        {
            const glm::dvec2 candidateMeters{east, north};
            if (DistanceToBuildings(candidateMeters, buildings)
                < minimumBuildingClearanceMeters)
            {
                continue;
            }

            const bool tooCloseToReservedPoint = std::any_of(
                reservedPositionsMeters.begin(), reservedPositionsMeters.end(),
                [&](const glm::dvec2& reservedPositionMeters)
                {
                    return glm::distance(candidateMeters, reservedPositionMeters)
                        < minimumPointSeparationMeters;
                });
            if (tooCloseToReservedPoint)
                continue;

            const double distanceFromPreferredMeters = glm::distance(
                candidateMeters, preferredPositionMeters);
            if (distanceFromPreferredMeters > maximumSearchDistanceMeters
                || distanceFromPreferredMeters >= closestDistanceMeters)
            {
                continue;
            }

            closestPositionMeters = candidateMeters;
            closestDistanceMeters = distanceFromPreferredMeters;
        }
    }

    if (!std::isfinite(closestDistanceMeters))
    {
        throw std::runtime_error(
            "도시 지도에서 비행 가능한 위치를 찾지 못했습니다. 기준 위치 ("
            + std::to_string(preferredPositionMeters.x) + ", "
            + std::to_string(preferredPositionMeters.y) + "), 탐색 반경 "
            + std::to_string(maximumSearchDistanceMeters) + "m, 최소 간격 "
            + std::to_string(minimumPointSeparationMeters) + "m");
    }
    return closestPositionMeters;
}
}

FleetMissionEnvironment::FleetMissionEnvironment(
    const terrain::TerrainGrid& terrain,
    const scene::TrainingRangeScene& trainingScene)
    : routeEnvironment_(terrain, trainingScene.BuildingObstacles()),
      planner_(routeEnvironment_, minimumTransitAltitudeMeters)
{
    const glm::dvec2 halfMapSizeMeters = trainingScene.GroundSizeMeters() * 0.5;
    stagingPositionMeters_ = FindOpenPosition(
        {0.0, -23.0 * scene::TrainingRangeScene::mapScale},
        halfMapSizeMeters, trainingScene.BuildingObstacles(), {},
        10.0, 0.0, std::numeric_limits<double>::infinity());

    // 정찰 지점을 출발 지점 주변 네 방향에 나눠 임무 범위를 넓힌다.
    const double reconRadiusMeters = std::min(
        std::min(halfMapSizeMeters.x, halfMapSizeMeters.y) * 0.2, 30.0);
    const auto keepInsideMap = [&](glm::dvec2 positionMeters)
    {
        positionMeters.x = std::clamp(
            positionMeters.x, -halfMapSizeMeters.x + 10.0, halfMapSizeMeters.x - 10.0);
        positionMeters.y = std::clamp(
            positionMeters.y, -halfMapSizeMeters.y + 10.0, halfMapSizeMeters.y - 10.0);
        return positionMeters;
    };
    const std::array<glm::dvec2, 4> preferredReconPositionsMeters{
        keepInsideMap(stagingPositionMeters_ + glm::dvec2{reconRadiusMeters, 0.0}),
        keepInsideMap(stagingPositionMeters_ + glm::dvec2{0.0, reconRadiusMeters}),
        keepInsideMap(stagingPositionMeters_ + glm::dvec2{-reconRadiusMeters, 0.0}),
        keepInsideMap(stagingPositionMeters_ + glm::dvec2{0.0, -reconRadiusMeters})};
    std::vector<glm::dvec2> reservedPositionsMeters{stagingPositionMeters_};
    constexpr double maximumTargetSearchDistanceMeters = 50.0;
    targets_.reserve(preferredReconPositionsMeters.size());
    for (std::size_t index = 0; index < preferredReconPositionsMeters.size(); ++index)
    {
        const glm::dvec2 targetPositionMeters = FindOpenPosition(
            preferredReconPositionsMeters[index], halfMapSizeMeters,
            trainingScene.BuildingObstacles(), reservedPositionsMeters,
            8.0, 15.0, maximumTargetSearchDistanceMeters);
        reservedPositionsMeters.push_back(targetPositionMeters);
        const double groundMeters = terrain.HeightAtLocalMeters(targetPositionMeters);
        // 드론이 대상 지점에 닿지 않고도 위에서 관찰할 수 있게 지면보다 높게 둔다.
        targets_.push_back({"정찰 지점 " + std::to_string(index + 1U),
            {targetPositionMeters, groundMeters + targetApproachHeightMeters}});
    }
}

const scene::TrainingRangeRouteEnvironment& FleetMissionEnvironment::RouteEnvironment() const noexcept
{
    return routeEnvironment_;
}

const mission::FleetMissionPlanner& FleetMissionEnvironment::Planner() const noexcept
{
    return planner_;
}

const std::vector<mission::FleetReconnaissanceTarget>&
FleetMissionEnvironment::Targets() const noexcept
{
    return targets_;
}

const glm::dvec2& FleetMissionEnvironment::StagingPositionMeters() const noexcept
{
    return stagingPositionMeters_;
}

double FleetMissionEnvironment::MinimumTransitAltitudeMeters() const noexcept
{
    return minimumTransitAltitudeMeters;
}
}
