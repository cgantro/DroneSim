#include "simulation_core/FormationPlanner.h"

#include "terrain/MapFlightRoute.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>
#include <stdexcept>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

namespace aerosimlab::simulation
{
namespace
{
constexpr double compactColumnSpacingMeters = 4.0;
constexpr double formationProfileSampleStepMeters = 1.0;

double RequiredClearance(
    FormationShape shape, std::size_t droneCount,
    double spacingMeters, double droneRadiusMeters, double safetyMarginMeters);

struct RouteSample
{
    glm::dvec3 positionMeters{0.0};
    double distanceMeters = 0.0;
    bool requestedFormationFits = false;
};

double RouteLength(
    const glm::dvec3& startMeters,
    const std::vector<glm::dvec3>& route)
{
    double lengthMeters = 0.0;
    glm::dvec3 previous = startMeters;
    for (const glm::dvec3& waypoint : route)
    {
        lengthMeters += glm::distance(previous, waypoint);
        previous = waypoint;
    }
    return lengthMeters;
}

std::vector<RouteSample> SampleRoute(
    const glm::dvec3& startMeters,
    const std::vector<glm::dvec3>& route,
    const terrain::TerrainGrid& terrain,
    const std::vector<terrain::MapBuilding>& obstacles,
    double requestedClearanceMeters)
{
    std::vector<RouteSample> samples;
    double distanceMeters = 0.0;
    samples.push_back({startMeters, 0.0,
        terrain::IsMapFlightPositionClear(terrain, obstacles, startMeters,
            requestedClearanceMeters)});
    glm::dvec3 segmentStartMeters = startMeters;
    for (const glm::dvec3& segmentEndMeters : route)
    {
        const double segmentLength = glm::distance(
            segmentStartMeters, segmentEndMeters);
        const auto sampleCount = std::max<std::size_t>(1U,
            static_cast<std::size_t>(std::ceil(
                segmentLength / formationProfileSampleStepMeters)));
        for (std::size_t sampleIndex = 1U;
             sampleIndex <= sampleCount; ++sampleIndex)
        {
            const double fraction = static_cast<double>(sampleIndex)
                / static_cast<double>(sampleCount);
            const glm::dvec3 position = glm::mix(
                segmentStartMeters, segmentEndMeters, fraction);
            samples.push_back({position,
                distanceMeters + segmentLength * fraction,
                terrain::IsMapFlightPositionClear(terrain, obstacles, position,
                    requestedClearanceMeters)});
        }
        distanceMeters += segmentLength;
        segmentStartMeters = segmentEndMeters;
    }
    return samples;
}

void AddProfilePoint(
    std::vector<FormationRoutePlan::ProfilePoint>& profile,
    double distanceMeters,
    FormationShape shape,
    double spacingMeters)
{
    if (!profile.empty()
        && std::abs(profile.back().distanceMeters - distanceMeters) < 1.0e-6)
    {
        profile.back() = {distanceMeters, shape, spacingMeters};
        return;
    }
    profile.push_back({distanceMeters, shape, spacingMeters});
}

void BuildLocalFormationProfile(
    FormationRoutePlan& plan,
    const terrain::TerrainGrid& terrain,
    const std::vector<terrain::MapBuilding>& obstacles,
    std::size_t droneCount,
    double droneRadiusMeters,
    double safetyMarginMeters)
{
    const double requestedClearanceMeters = RequiredClearance(plan.requestedShape,
        droneCount, plan.requestedSpacingMeters, droneRadiusMeters, safetyMarginMeters);
    const double transitionLengthMeters = std::max(10.0,
        plan.requestedSpacingMeters);
    const auto samples = SampleRoute(plan.leaderStartMeters,
        plan.leaderWaypointsMeters,
        terrain, obstacles, requestedClearanceMeters);
    if (samples.empty())
        return;

    struct SafeRun
    {
        std::size_t first = 0U;
        std::size_t last = 0U;
    };
    std::vector<SafeRun> safeRuns;
    for (std::size_t index = 0U; index < samples.size();)
    {
        if (!samples[index].requestedFormationFits)
        {
            ++index;
            continue;
        }
        const std::size_t first = index;
        while (index + 1U < samples.size()
            && samples[index + 1U].requestedFormationFits)
            ++index;
        safeRuns.push_back({first, index});
        ++index;
    }

    std::vector<bool> useRequestedFormation(safeRuns.size(), false);
    for (std::size_t runIndex = 0U; runIndex < safeRuns.size(); ++runIndex)
    {
        const SafeRun run = safeRuns[runIndex];
        const double runLengthMeters = samples[run.last].distanceMeters
            - samples[run.first].distanceMeters;
        const bool hasNarrowSectionBefore = run.first > 0U;
        const bool hasNarrowSectionAfter = run.last + 1U < samples.size();
        const double requiredTransitionLength = hasNarrowSectionBefore
                && hasNarrowSectionAfter
            ? transitionLengthMeters * 3.0
            : transitionLengthMeters;
        useRequestedFormation[runIndex] =
            runLengthMeters + 1.0e-6 >= requiredTransitionLength;
    }

    const FormationShape compactShape = FormationShape::Column;
    const double compactSpacingMeters = std::min(
        plan.requestedSpacingMeters, compactColumnSpacingMeters);
    FormationShape currentShape = compactShape;
    double currentSpacingMeters = compactSpacingMeters;
    if (!safeRuns.empty() && safeRuns.front().first == 0U
        && useRequestedFormation.front())
    {
        currentShape = plan.requestedShape;
        currentSpacingMeters = plan.requestedSpacingMeters;
    }

    plan.profile.clear();
    AddProfilePoint(plan.profile, 0.0, currentShape, currentSpacingMeters);
    for (std::size_t runIndex = 0U; runIndex < safeRuns.size(); ++runIndex)
    {
        if (!useRequestedFormation[runIndex])
            continue;

        const SafeRun run = safeRuns[runIndex];
        const bool beginsAfterNarrowSection = run.first > 0U;
        const bool endsBeforeNarrowSection = run.last + 1U < samples.size();
        if (beginsAfterNarrowSection)
        {
            AddProfilePoint(plan.profile, samples[run.first].distanceMeters,
                currentShape, currentSpacingMeters);
            AddProfilePoint(plan.profile,
                samples[run.first].distanceMeters + transitionLengthMeters,
                plan.requestedShape, plan.requestedSpacingMeters);
            currentShape = plan.requestedShape;
            currentSpacingMeters = plan.requestedSpacingMeters;
        }
        if (endsBeforeNarrowSection)
        {
            const double transitionEndMeters = samples[run.last].distanceMeters;
            const double transitionStartMeters = transitionEndMeters
                - transitionLengthMeters;
            AddProfilePoint(plan.profile, transitionStartMeters,
                currentShape, currentSpacingMeters);
            AddProfilePoint(plan.profile, transitionEndMeters,
                compactShape, compactSpacingMeters);
            currentShape = compactShape;
            currentSpacingMeters = compactSpacingMeters;
        }
    }
    AddProfilePoint(plan.profile, RouteLength(plan.leaderStartMeters,
        plan.leaderWaypointsMeters),
        currentShape, currentSpacingMeters);

    plan.shape = plan.profile.front().shape;
    plan.spacingMeters = plan.profile.front().spacingMeters;
    plan.requiredClearanceMeters = RequiredClearance(plan.shape,
        droneCount, plan.spacingMeters, droneRadiusMeters, safetyMarginMeters);
}

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

std::vector<glm::dvec2> FormationOffsetsAlongRoute(
    const FormationRoutePlan& route,
    std::size_t droneCount,
    double headingRadians,
    double distanceAlongRouteMeters)
{
    if (!std::isfinite(distanceAlongRouteMeters))
        throw std::invalid_argument("편대 경로 진행 거리는 유한한 값이어야 합니다.");
    if (route.profile.size() < 2U)
        return FormationOffsets(route.shape, droneCount,
            route.spacingMeters, headingRadians);

    const double distance = std::max(0.0, distanceAlongRouteMeters);
    const auto next = std::upper_bound(route.profile.begin(), route.profile.end(),
        distance, [](double value, const FormationRoutePlan::ProfilePoint& point)
        {
            return value < point.distanceMeters;
        });
    if (next == route.profile.begin())
        return FormationOffsets(next->shape, droneCount,
            next->spacingMeters, headingRadians);
    if (next == route.profile.end())
    {
        const auto& last = route.profile.back();
        return FormationOffsets(last.shape, droneCount,
            last.spacingMeters, headingRadians);
    }

    const auto& from = *std::prev(next);
    const auto& to = *next;
    const double transitionLength = to.distanceMeters - from.distanceMeters;
    const double blend = transitionLength > 0.0
        ? std::clamp((distance - from.distanceMeters) / transitionLength, 0.0, 1.0)
        : 1.0;
    const auto fromOffsets = FormationOffsets(from.shape, droneCount,
        from.spacingMeters, headingRadians);
    const auto toOffsets = FormationOffsets(to.shape, droneCount,
        to.spacingMeters, headingRadians);
    std::vector<glm::dvec2> offsets;
    offsets.reserve(droneCount);
    for (std::size_t index = 0U; index < droneCount; ++index)
        offsets.push_back(glm::mix(fromOffsets[index], toOffsets[index], blend));
    return offsets;
}

bool IsFormationTransitionAtDistance(
    const FormationRoutePlan& route,
    double distanceAlongRouteMeters)
{
    if (!std::isfinite(distanceAlongRouteMeters) || route.profile.size() < 2U)
        return false;

    const double distance = std::max(0.0, distanceAlongRouteMeters);
    const auto next = std::upper_bound(route.profile.begin(), route.profile.end(),
        distance, [](double value, const FormationRoutePlan::ProfilePoint& point)
        {
            return value < point.distanceMeters;
        });
    if (next == route.profile.begin() || next == route.profile.end())
        return false;
    const auto& from = *std::prev(next);
    const auto& to = *next;
    return from.shape != to.shape
        || std::abs(from.spacingMeters - to.spacingMeters) > 1.0e-6;
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

    const auto tryShape = [&](FormationShape shape, double formationSpacingMeters)
        -> std::optional<FormationRoutePlan>
    {
        const double clearance = RequiredClearance(shape, droneCount,
            formationSpacingMeters, droneRadiusMeters, safetyMarginMeters);
        const auto route = terrain::PlanMapFlightRoute(terrain, obstacles,
            leaderStartMeters, destinationMeters, clearance,
            minimumTransitAltitudeMeters);
        if (!route)
            return std::nullopt;
        FormationRoutePlan plan;
        plan.shape = shape;
        plan.leaderStartMeters = leaderStartMeters;
        plan.leaderWaypointsMeters = *route;
        plan.requiredClearanceMeters = clearance;
        plan.spacingMeters = formationSpacingMeters;
        plan.requestedShape = requestedShape;
        plan.requestedSpacingMeters = spacingMeters;
        const double routeLengthMeters = RouteLength(leaderStartMeters, *route);
        plan.profile = {
            {0.0, shape, formationSpacingMeters},
            {routeLengthMeters, shape, formationSpacingMeters}};
        return plan;
    };

    if (auto route = tryShape(requestedShape, spacingMeters))
        return route;
    if (requestedShape != FormationShape::Column)
    {
        const double compactSpacingMeters = std::min(
            spacingMeters, compactColumnSpacingMeters);
        const double minimumSafeSpacingMeters = droneRadiusMeters * 2.0
            + safetyMarginMeters * 0.5;
        const double requestedClearance = RequiredClearance(requestedShape,
            droneCount, spacingMeters, droneRadiusMeters, safetyMarginMeters);
        const double compactClearance = RequiredClearance(FormationShape::Column,
            droneCount, compactSpacingMeters, droneRadiusMeters, safetyMarginMeters);
        if (compactSpacingMeters >= minimumSafeSpacingMeters
            && compactClearance < requestedClearance)
        {
            auto compactRoute = tryShape(FormationShape::Column,
                compactSpacingMeters);
            if (!compactRoute)
                return std::nullopt;
            BuildLocalFormationProfile(*compactRoute, terrain, obstacles,
                droneCount, droneRadiusMeters, safetyMarginMeters);
            return compactRoute;
        }
    }
    return std::nullopt;
}
}
