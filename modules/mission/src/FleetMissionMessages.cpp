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

void WriteWaypoint(std::ostream& payload, const glm::dvec3& waypointMeters)
{
    // 17자리 유효 숫자는 double 값을 문자열로 바꿨다가 읽어도 원래 값으로 복원한다.
    payload << std::setprecision(17)
        << waypointMeters.x << ' ' << waypointMeters.y << ' ' << waypointMeters.z;
}

bool ReadWaypoint(std::istream& values, glm::dvec3& waypointMeters)
{
    return static_cast<bool>(values >> waypointMeters.x
        >> waypointMeters.y >> waypointMeters.z)
        && std::isfinite(waypointMeters.x)
        && std::isfinite(waypointMeters.y)
        && std::isfinite(waypointMeters.z);
}

bool HasNoTrailingData(std::istream& values)
{
    values >> std::ws;
    return values.eof();
}
}

std::string EncodeWaypoint(const glm::dvec3& waypointMeters)
{
    std::ostringstream payload;
    WriteWaypoint(payload, waypointMeters);
    return payload.str();
}

std::optional<glm::dvec3> DecodeWaypoint(const std::string& payload)
{
    std::istringstream values(payload);
    glm::dvec3 waypointMeters{0.0};
    if (!ReadWaypoint(values, waypointMeters) || !HasNoTrailingData(values))
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
    // 만료 시각도 보내 수신 드론이 리더와 같은 모의 시간 기준으로 임무를 판단하게 한다.
    payload << static_cast<unsigned int>(leg) << ' '
        << targetIndex << ' ' << assignmentVersion << ' '
        << assignmentLeaseExpiresAtTick << ' ' << waypointsMeters.size();
    for (const glm::dvec3& waypointMeters : waypointsMeters)
    {
        payload << ' ';
        WriteWaypoint(payload, waypointMeters);
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
    // 경유점 개수는 받은 문자열에 적혀 있으므로, 미리 큰 메모리를 예약하지 않는다.
    // 입력 좌표가 부족하면 아래 읽기에서 곧바로 실패해 잘못된 메시지를 거른다.
    for (std::size_t index = 0; index < waypointCount; ++index)
    {
        glm::dvec3 waypointMeters{0.0};
        if (!ReadWaypoint(values, waypointMeters))
            return std::nullopt;
        route.waypointsMeters.push_back(waypointMeters);
    }

    if (!HasNoTrailingData(values))
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

    // 계획 단계에서는 실제 비행 제어기의 속도와 다를 수 있어 보수적으로 1m/s를 쓴다.
    // 여기에 30초를 더해 경로 배정이 정상 이동 중 먼저 만료되지 않게 한다.
    const double estimatedTravelTicks = std::ceil(routeLengthMeters
        / estimatedPlanningSpeedMetersPerSecond * simulationTicksPerSecond);
    return currentTick + static_cast<std::uint64_t>(estimatedTravelTicks)
        + leaseGraceTicks;
}
}
