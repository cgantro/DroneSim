#include "FleetMission.h"

#include <algorithm>
#include <utility>

#include <glm/geometric.hpp>

namespace dronesim::mission
{

void FormationTransitState::SetRoute(flight_planning::FormationRoutePlan route)
{
    // 새 경로는 이전 경로의 경유점 진행을 이어받지 않는다.
    route_ = std::move(route);
    waypointIndex_ = 0U;
}
void FormationTransitState::AdvanceWaypointIfReached(
    const glm::dvec3& positionMeters,
    double toleranceMeters)
{
    const auto& waypoints = route_.leaderWaypointsMeters;
    if (waypoints.empty())
        return;

    waypointIndex_ = std::min(waypointIndex_, waypoints.size() - 1U);
    const bool reachedCurrentWaypoint =
        glm::distance(positionMeters, waypoints[waypointIndex_]) <= toleranceMeters;
    if (reachedCurrentWaypoint && waypointIndex_ + 1U < waypoints.size())
        ++waypointIndex_;
}
void FleetMissionState::BeginFormationTransit(
    flight_planning::FormationRoutePlan route, std::string status)
{
    formationNetwork_.Reset();
    formationTransit_.SetRoute(std::move(route));
    formationTransit_.Start(std::move(status));
}

bool FleetReconnaissanceState::IsLegFinished() const noexcept
{
    switch (leg_)
    {
    case FleetMissionLeg::Reconnaissance:
        return status_ == FleetDroneStatus::TargetReached
            || status_ == FleetDroneStatus::RouteFailed
            || status_ == FleetDroneStatus::OutsideTargetLanding;
    case FleetMissionLeg::Regrouping:
        return status_ == FleetDroneStatus::RegroupComplete
            || status_ == FleetDroneStatus::RegroupFailed;
    case FleetMissionLeg::Returning:
        return status_ == FleetDroneStatus::ReturnComplete
            || status_ == FleetDroneStatus::ReturnFailed
            || status_ == FleetDroneStatus::LowBatteryReturnComplete;
    }
    return false;
}

void FleetReconnaissanceState::BeginLeg(FleetMissionLeg nextLeg) noexcept
{
    leg_ = nextLeg;
    status_ = FleetDroneStatus::RoutePending;
    routeWaypointsMeters_.clear();
    ResetRouteProgress();
}

void FleetReconnaissanceState::ResetRouteProgress() noexcept
{
    nextWaypoint_ = 0U;
    routeReceived_ = false;
    waypointCommanded_ = false;
}

void FleetReconnaissanceState::SetPlannedRoute(
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    routeWaypointsMeters_ = std::move(routeWaypointsMeters);
    ResetRouteProgress();
}

std::vector<glm::dvec3> FleetReconnaissanceState::TakeRouteForDelivery()
{
    std::vector<glm::dvec3> route = std::move(routeWaypointsMeters_);
    routeWaypointsMeters_.clear();
    ResetRouteProgress();
    return route;
}

void FleetReconnaissanceState::ReceiveRoute(
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    routeWaypointsMeters_ = std::move(routeWaypointsMeters);
    ResetRouteProgress();
    routeReceived_ = true;
}

void FleetReconnaissanceState::WaitForRoute() noexcept
{
    routeWaypointsMeters_.clear();
    ResetRouteProgress();
}

void FleetReconnaissanceState::BeginLowBatteryReturn() noexcept
{
    lowBatteryReturn_ = true;
    BeginLeg(FleetMissionLeg::Returning);
}

void FleetReconnaissanceState::BeginWaypointCommand() noexcept
{
    waypointCommanded_ = true;
    switch (leg_)
    {
    case FleetMissionLeg::Returning:
        status_ = lowBatteryReturn_
            ? FleetDroneStatus::LowBatteryReturning
            : FleetDroneStatus::Returning;
        break;
    case FleetMissionLeg::Regrouping:
        status_ = FleetDroneStatus::Regrouping;
        break;
    case FleetMissionLeg::Reconnaissance:
        status_ = FleetDroneStatus::Reconnaissance;
        break;
    }
}

bool FleetReconnaissanceState::AdvanceWaypointIfReached(
    const glm::dvec3& positionMeters,
    double toleranceMeters) noexcept
{
    if (nextWaypoint_ >= routeWaypointsMeters_.size())
        return false;

    const double distanceMeters = glm::length(
        routeWaypointsMeters_[nextWaypoint_] - positionMeters);
    if (distanceMeters > toleranceMeters)
        return false;

    ++nextWaypoint_;
    return true;
}

const glm::dvec3* FleetReconnaissanceState::CurrentWaypoint() const noexcept
{
    if (nextWaypoint_ >= routeWaypointsMeters_.size())
        return nullptr;
    return &routeWaypointsMeters_[nextWaypoint_];
}

FleetReconnaissanceState* FleetMissionState::FindAssignment(
    simulation::DroneId droneId) noexcept
{
    const auto found = assignments_.find(droneId);
    // 반복자는 이 함수 안에서만 쓰고, 호출부에는 배정 값이나 없음만 전달한다.
    return found == assignments_.end() ? nullptr : &found->second;
}

const FleetReconnaissanceState* FleetMissionState::FindAssignment(
    simulation::DroneId droneId) const noexcept
{
    const auto found = assignments_.find(droneId);
    // 읽기 전용 호출도 같은 검색 규칙을 사용하고 수정 권한만 제한한다.
    return found == assignments_.end() ? nullptr : &found->second;
}

void FleetMissionState::SetAssignment(
    simulation::DroneId droneId,
    FleetReconnaissanceState assignment)
{
    assignments_.insert_or_assign(droneId, std::move(assignment));
}

void FleetMissionState::ResetTelemetry()
{
    telemetryAtLeader_.Clear();
}

std::optional<PeerTelemetry> FleetMissionState::CurrentDroneObservation(
    simulation::DroneId droneId,
    const simulation::SimulationCore& simulation,
    MissionStateSource source) const
{
    if (source == MissionStateSource::LocalSimulation || droneId == leaderId_)
    {
        const simulation::DroneState& drone = simulation.State(droneId);
        PeerTelemetry observation{
            .positionMeters = drone.positionMeters,
            .mode = drone.mode};

        const FleetReconnaissanceState* assignment = FindAssignment(droneId);
        if (assignment != nullptr)
        {
            observation.missionLeg = static_cast<std::uint8_t>(assignment->Leg());
            observation.missionStatus = assignment->Status();
            observation.assignmentVersion = assignment->AssignmentVersion();
        }
        return observation;
    }

    const ReceivedPeerTelemetry* report = telemetryAtLeader_.Fresh(
        droneId, simulation.TickCount());
    return report == nullptr
        ? std::nullopt
        : std::optional<PeerTelemetry>{report->state};
}

bool FleetMissionState::HasActiveReconnaissance() const noexcept
{
    for (const auto& [droneId, reconnaissance] : assignments_)
    {
        (void)droneId;
        if (IsFleetMissionInProgress(reconnaissance.Status()))
            return true;
    }
    return false;
}

bool FleetReconnaissanceState::CanBeginLowBatteryReturn() const noexcept
{
    return leg_ == FleetMissionLeg::Reconnaissance
        && (IsFleetMissionInProgress(status_)
            || status_ == FleetDroneStatus::TargetReached);
}

FleetDroneStatus FleetReconnaissanceState::StatusAfterLanding(
    const glm::dvec3& positionMeters,
    const glm::dvec3& destinationMeters) const noexcept
{
    if (leg_ == FleetMissionLeg::Regrouping)
        return FleetDroneStatus::RegroupFailed;

    // 착륙 판정은 수평 오차만 본다. 고도는 착륙 중 지면 영향으로 변할 수 있다.
    const glm::dvec2 horizontalError{
        positionMeters.x - destinationMeters.x,
        positionMeters.y - destinationMeters.y};
    const bool reachedDestination = glm::length(horizontalError) <= 5.0;

    if (leg_ == FleetMissionLeg::Returning)
    {
        if (!reachedDestination)
            return FleetDroneStatus::ReturnFailed;
        return lowBatteryReturn_
            ? FleetDroneStatus::LowBatteryReturnComplete
            : FleetDroneStatus::ReturnComplete;
    }

    return reachedDestination
        ? FleetDroneStatus::TargetReached
        : FleetDroneStatus::OutsideTargetLanding;
}

std::optional<FleetMissionReport> FleetMissionState::FreshReport(
    simulation::DroneId droneId,
    const FleetReconnaissanceState& assignment,
    const simulation::SimulationCore& simulation) const
{
    const auto observation = CurrentDroneObservation(
        droneId, simulation, MissionStateSource::FreshTelemetry);
    if (!observation)
        return std::nullopt;

    if (droneId == leaderId_)
    {
        return FleetMissionReport{
            observation->mode, assignment.Leg(), assignment.Status()};
    }
    return FleetMissionReport{
        observation->mode,
        static_cast<FleetMissionLeg>(observation->missionLeg),
        observation->missionStatus};
}

std::optional<FleetMissionReport> FleetMissionState::CurrentReport(
    simulation::DroneId droneId,
    const FleetReconnaissanceState& assignment,
    const simulation::SimulationCore& simulation,
    CompleteFleetMissionState::Stage currentStage) const
{
    if (const auto report = FreshReport(droneId, assignment, simulation))
        return report;

    if (droneId == leaderId_)
        return std::nullopt;

    const ReceivedPeerTelemetry* cached = telemetryAtLeader_.Latest(droneId);
    if (cached == nullptr
        || cached->state.assignmentVersion != assignment.AssignmentVersion()
        || cached->state.mode != simulation::FlightMode::Grounded)
        return std::nullopt;

    // 통신이 끊겨도 이 임무에서 이미 착륙을 마친 기록은 최종 결과로 쓸 수 있다.
    const bool returnedAfterBatteryReserve = assignment.IsLowBatteryReturnComplete()
        && cached->state.missionLeg
            == static_cast<std::uint8_t>(FleetMissionLeg::Returning)
        && cached->state.missionStatus == FleetDroneStatus::LowBatteryReturnComplete;
    const bool returnedToBase = currentStage == CompleteFleetMissionState::Stage::Returning
        && cached->state.missionLeg
            == static_cast<std::uint8_t>(FleetMissionLeg::Returning)
        && cached->state.missionStatus == FleetDroneStatus::ReturnComplete;
    if (!returnedAfterBatteryReserve && !returnedToBase)
        return std::nullopt;

    return FleetMissionReport{
        cached->state.mode,
        static_cast<FleetMissionLeg>(cached->state.missionLeg),
        cached->state.missionStatus};
}

bool FleetMissionState::AllDronesReadyToReturn(
    std::span<const simulation::DroneId> droneIds,
    const simulation::SimulationCore& simulation) const
{
    if (droneIds.empty())
        return false;

    return std::all_of(droneIds.begin(), droneIds.end(), [this, &simulation](auto droneId)
    {
        const FleetReconnaissanceState* assignment = FindAssignment(droneId);
        if (assignment == nullptr)
            return false;

        const auto report = FreshReport(droneId, *assignment, simulation);
        if (!report || report->mode != simulation::FlightMode::Grounded)
            return false;

        const bool landedAfterLowBatteryReserve =
            assignment->IsLowBatteryReturnComplete()
            && report->status == FleetDroneStatus::LowBatteryReturnComplete;
        if (landedAfterLowBatteryReserve)
            return true;

        return report->leg == FleetMissionLeg::Reconnaissance
            && !IsFleetMissionInProgress(report->status);
    });
}

}
