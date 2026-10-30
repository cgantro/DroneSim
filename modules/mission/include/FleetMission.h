#pragma once

#include "FleetMissionProgress.h"
#include "flight_planning/FormationPlanner.h"
#include "simulation_core/SimulationCore.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

namespace dronesim::mission
{
/** @brief 드론 한 대에 배정된 정찰·집결·복귀 경로와 진행 상태다. */
struct FleetReconnaissanceState
{
    std::size_t targetIndex = 0;
    glm::dvec3 startPositionMeters{0.0};
    std::vector<glm::dvec3> routeWaypointsMeters;
    std::string pendingRoutePayload;
    std::uint32_t routeSequence = 0;
    std::uint32_t pendingRouteSequence = 0;
    std::uint64_t assignmentVersion = 0;
    std::uint64_t assignmentLeaseExpiresAtTick = 0;
    std::uint32_t routeTransmissionCount = 0;
    std::uint64_t routeSentAtTick = 0;
    std::uint64_t lastRouteTransmissionTick = 0;
    std::size_t nextWaypoint = 0;
    FleetDroneStatus status = FleetDroneStatus::Other;
    bool routeReceived = false;
    bool waypointCommanded = false;
    bool lowBatteryReturn = false;
    FleetMissionLeg leg = FleetMissionLeg::Reconnaissance;
};

/** @brief 편대가 목표 지점으로 이동할 때 필요한 진행 자료다. */
struct FormationTransitState
{
    enum class Phase
    {
        Idle,
        InProgress,
        Arrived,
        Failed
    };

    Phase phase = Phase::Idle;
    [[nodiscard]] bool IsActive() const noexcept
    {
        return phase == Phase::InProgress;
    }

    flight_planning::FormationRoutePlan route;
    std::size_t waypointIndex = 0;
    std::uint32_t nextMessageSequence = 1;
    std::map<simulation::DroneId, glm::dvec3> sentFollowerTargets;
    std::map<simulation::DroneId, std::uint64_t> lastTargetSentAtTick;
    std::map<simulation::DroneId, std::uint32_t> lastAppliedSequence;
    std::optional<std::uint64_t> lastNetworkNodeUpdateTick;
    std::string status = "대기";
};

/** @brief M01 편대 이동부터 M03 복귀까지 전체 임무의 진행 자료다. */
struct CompleteFleetMissionState
{
    enum class Stage
    {
        Idle,
        FormationTransit,
        Reconnaissance,
        Regrouping,
        Returning,
        Completed,
        Failed
    };

    Stage stage = Stage::Idle;
    std::map<simulation::DroneId, glm::dvec3> homePositionsMeters;
    std::string status = "대기";
};
}
