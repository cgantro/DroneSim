#include "FleetMissionMessages.h"

#include <cmath>
#include <glm/geometric.hpp>
#include <iomanip>
#include <sstream>

namespace dronesim::mission
{
namespace
{
constexpr std::uint64_t leaseGraceTicks = 1800U;
constexpr double estimatedPlanningSpeedMetersPerSecond = 1.0;
constexpr double simulationTicksPerSecond = 60.0;
}

std::string EncodeWaypoint(const glm::dvec3& waypointMeters)
{
    std::ostringstream payload;
    payload << std::setprecision(17)
        << waypointMeters.x << ' ' << waypointMeters.y << ' ' << waypointMeters.z;
    return payload.str();
}

std::optional<glm::dvec3> DecodeWaypoint(const std::string& payload)
{
    std::istringstream values(payload);
    glm::dvec3 waypointMeters{0.0};
    if (!(values >> waypointMeters.x >> waypointMeters.y >> waypointMeters.z))
        return std::nullopt;

    values >> std::ws;
    if (!values.eof()
        || !std::isfinite(waypointMeters.x)
        || !std::isfinite(waypointMeters.y)
        || !std::isfinite(waypointMeters.z))
        return std::nullopt;
    return waypointMeters;
}

std::string EncodeFleetRoute(
    FleetMissionLeg leg,
    std::size_t targetIndex,
    std::uint64_t assignmentVersion,
    std::uint64_t assignmentLeaseExpiresAtTick,
    std::span<const glm::dvec3> waypointsMeters)
{
    std::ostringstream payload;
    payload << static_cast<unsigned int>(leg) << ' '

        // 만료 시각도 함께 보내면 수신 드론은 리더의 모의 시간 기준으로 임무 유효성을 판단한다.
        << targetIndex << ' ' << assignmentVersion << ' '
        << assignmentLeaseExpiresAtTick << ' ' << waypointsMeters.size();
    for (const glm::dvec3& waypointMeters : waypointsMeters)
    {
        payload << ' ' << std::setprecision(17)
            << waypointMeters.x << ' ' << waypointMeters.y << ' ' << waypointMeters.z;
    }
    return payload.str();
}

std::optional<FleetRouteMessage> DecodeFleetRoute(const std::string& payload)
{
    std::istringstream values(payload);
    unsigned int legValue = 0;
    FleetRouteMessage route;
    std::size_t waypointCount = 0;
    if (!(values >> legValue >> route.targetIndex >> route.assignmentVersion
            >> route.assignmentLeaseExpiresAtTick >> waypointCount)
        || legValue > static_cast<unsigned int>(FleetMissionLeg::Returning))
        return std::nullopt;

    route.leg = static_cast<FleetMissionLeg>(legValue);
    route.waypointsMeters.reserve(waypointCount);
    for (std::size_t index = 0; index < waypointCount; ++index)
    {
        glm::dvec3 waypointMeters{0.0};
        if (!(values >> waypointMeters.x >> waypointMeters.y >> waypointMeters.z)
            || !std::isfinite(waypointMeters.x)
            || !std::isfinite(waypointMeters.y)
            || !std::isfinite(waypointMeters.z))
            return std::nullopt;
        route.waypointsMeters.push_back(waypointMeters);
    }

    values >> std::ws;
    if (!values.eof())
        return std::nullopt;
    return route;
}

std::uint64_t EstimateMissionLeaseExpiry(
    std::uint64_t currentTick,
    const glm::dvec3& startPositionMeters,
    std::span<const glm::dvec3> routeWaypointsMeters)
{
    double routeLengthMeters = 0.0;
    glm::dvec3 previousPositionMeters = startPositionMeters;
    for (const glm::dvec3& waypointMeters : routeWaypointsMeters)
    {
        routeLengthMeters += glm::distance(previousPositionMeters, waypointMeters);
        previousPositionMeters = waypointMeters;
    }

    // 예상 비행 시간에 30초 여유를 더해, 느린 기체가 정상 임무 중 만료되지 않게 한다.
    const double estimatedTravelTicks = std::ceil(routeLengthMeters
        / estimatedPlanningSpeedMetersPerSecond * simulationTicksPerSecond);
    return currentTick + static_cast<std::uint64_t>(estimatedTravelTicks)
        + leaseGraceTicks;
}
}
