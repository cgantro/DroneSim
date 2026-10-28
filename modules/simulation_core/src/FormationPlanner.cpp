#include "simulation_core/FormationPlanner.h"

#include "terrain/MapFlightRoute.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <glm/geometric.hpp>

namespace aerosimlab::simulation
{
namespace
{
std::vector<glm::dvec2> LocalOffsets(
    FormationShape shape, std::size_t droneCount, double spacingMeters)
{
    std::vector<glm::dvec2> offsets;
    offsets.reserve(droneCount);
    offsets.emplace_back(0.0, 0.0);

    for (std::size_t index = 1; index < droneCount; ++index)
    {
        const std::size_t followerIndex = index - 1U;
        switch (shape)
        {
        case FormationShape::Vee:
        {
            const double row = static_cast<double>(followerIndex / 2U + 1U);
            const std::size_t followerCount = droneCount - 1U;
            const bool isLastSoloFollower = followerCount % 2U != 0U
                && followerIndex + 1U == followerCount;
            const double side = isLastSoloFollower ? 0.0
                : (followerIndex % 2U == 0U ? 1.0 : -1.0);
            offsets.emplace_back(-row * spacingMeters,
                side * row * spacingMeters * 0.5);
            break;
        }
        case FormationShape::Column:
            offsets.emplace_back(-static_cast<double>(index) * spacingMeters, 0.0);
            break;
        case FormationShape::Line:
        {
            const double middle = static_cast<double>(droneCount - 1U) * 0.5;
            offsets.emplace_back(0.0,
                (middle - static_cast<double>(index)) * spacingMeters);
            break;
        }
        }
    }
    return offsets;
}

double RequiredClearance(
    FormationShape shape, std::size_t droneCount,
    double spacingMeters, double droneRadiusMeters, double safetyMarginMeters)
{
    const auto offsets = LocalOffsets(shape, droneCount, spacingMeters);
    double formationRadiusMeters = 0.0;
    for (const glm::dvec2& offset : offsets)
        formationRadiusMeters = std::max(formationRadiusMeters, glm::length(offset));
    return formationRadiusMeters + droneRadiusMeters + safetyMarginMeters;
}
}

std::vector<glm::dvec2> FormationOffsets(
    FormationShape shape, std::size_t droneCount,
    double spacingMeters, double headingRadians)
{
    if (droneCount == 0U || !std::isfinite(spacingMeters) || spacingMeters <= 0.0
        || !std::isfinite(headingRadians))
        throw std::invalid_argument("편대 수, 간격, 진행 방향이 올바르지 않습니다.");

    const std::vector<glm::dvec2> local =
        LocalOffsets(shape, droneCount, spacingMeters);
    const glm::dvec2 forward{std::cos(headingRadians), std::sin(headingRadians)};
    const glm::dvec2 right{forward.y, -forward.x};
    std::vector<glm::dvec2> worldOffsets;
    worldOffsets.reserve(local.size());
    for (const glm::dvec2& offset : local)
        worldOffsets.push_back(forward * offset.x + right * offset.y);
    return worldOffsets;
}

std::optional<FormationRoutePlan> PlanFormationRoute(
    const terrain::TerrainGrid& terrain,
    const std::vector<terrain::MapBuilding>& obstacles,
    const glm::dvec3& leaderStartMeters,
    const glm::dvec3& destinationMeters,
    std::size_t droneCount,
    FormationShape requestedShape,
    double spacingMeters,
    double droneRadiusMeters,
    double safetyMarginMeters,
    double minimumTransitAltitudeMeters)
{
    if (droneCount == 0U || !std::isfinite(spacingMeters) || spacingMeters <= 0.0
        || !std::isfinite(droneRadiusMeters) || droneRadiusMeters < 0.0
        || !std::isfinite(safetyMarginMeters) || safetyMarginMeters < 0.0)
        throw std::invalid_argument("편대 경로의 드론 수와 간격·여유 값이 올바르지 않습니다.");

    const auto tryShape = [&](FormationShape shape)
        -> std::optional<FormationRoutePlan>
    {
        const double clearance = RequiredClearance(shape, droneCount,
            spacingMeters, droneRadiusMeters, safetyMarginMeters);
        const auto route = terrain::PlanMapFlightRoute(terrain, obstacles,
            leaderStartMeters, destinationMeters, clearance,
            minimumTransitAltitudeMeters);
        if (!route)
            return std::nullopt;
        return FormationRoutePlan{shape, *route, clearance};
    };

    if (auto route = tryShape(requestedShape))
        return route;
    if (requestedShape != FormationShape::Column)
        return tryShape(FormationShape::Column);
    return std::nullopt;
}
}
