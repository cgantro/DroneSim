#pragma once

#include "FleetMissionProgress.h"

#include <glm/vec3.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace dronesim::mission
{
/** @brief 통신으로 전달하는 드론 한 대의 경로 배정이다. */
struct FleetRouteMessage
{
    FleetMissionLeg leg = FleetMissionLeg::Reconnaissance;
    std::size_t targetIndex = 0;
    std::uint64_t assignmentVersion = 0;
    std::uint64_t assignmentLeaseExpiresAtTick = 0;
    std::vector<glm::dvec3> waypointsMeters;
};

/** @brief 위치 하나를 기존 공백 구분 통신 형식으로 바꾼다. */
[[nodiscard]] std::string EncodeWaypoint(const glm::dvec3& waypointMeters);

/** @brief 좌표 세 개가 모두 유효한 위치 문자열인지 확인해 읽는다. */
[[nodiscard]] std::optional<glm::dvec3> DecodeWaypoint(const std::string& payload);

/** @brief 경로 배정을 드론 간 통신에 보낼 문자열로 만든다. */
[[nodiscard]] std::string EncodeFleetRoute(
    FleetMissionLeg leg,
    std::size_t targetIndex,
    std::uint64_t assignmentVersion,
    std::uint64_t assignmentLeaseExpiresAtTick,
    std::span<const glm::dvec3> waypointsMeters);

/** @brief 통신으로 받은 경로 문자열을 확인해 경로 배정으로 읽는다. */
[[nodiscard]] std::optional<FleetRouteMessage> DecodeFleetRoute(
    const std::string& payload);

/**
 * @brief 경로 길이와 여유 시간을 바탕으로 배정 만료 Tick을 계산한다.
 * @details 계획 속도는 1m/s로 잡고, 60Hz 모의 시간으로 환산한 뒤 30초 여유를 더한다.
 *          실제 기체의 예상 도착 시각이 아니라, 통신 지연으로 배정이 일찍 만료되는 것을 막는 기준이다.
 */
[[nodiscard]] std::uint64_t EstimateMissionLeaseExpiry(
    std::uint64_t currentTick,
    const glm::dvec3& startPositionMeters,
    std::span<const glm::dvec3> routeWaypointsMeters);
}
