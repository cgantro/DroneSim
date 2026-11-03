#include "FleetMission.h"

#include <algorithm>
#include <utility>

#include <glm/geometric.hpp>

namespace dronesim::mission
{
namespace
{
/**
 * @brief 시작 Tick 이후 지정한 간격이 흘렀는지 확인한다.
 * @details 시뮬레이션을 초기화하면 현재 Tick이 과거 기록보다 작아질 수 있다.
 * 이때 부호 없는 정수를 먼저 빼면 큰 값으로 넘치므로, 순서를 확인한 뒤 차이를 구한다.
 */
bool HasTickIntervalElapsed(
    std::uint64_t currentTick,
    std::uint64_t previousTick,
    std::uint64_t intervalTicks) noexcept
{
    return currentTick >= previousTick
        && currentTick - previousTick >= intervalTicks;
}
}

std::uint32_t FleetRouteDeliveryState::NextSequence() const noexcept
{
    // 새 명령은 이미 적용된 경로와 아직 확인을 기다리는 경로보다 앞선 번호를 쓴다.
    return std::max(appliedSequence_, pendingSequence_) + 1U;
}

void FleetRouteDeliveryState::Queue(
    std::uint32_t sequence,
    std::string payload,
    std::uint64_t currentTick)
{
    pendingSequence_ = sequence;
    pendingPayload_ = std::move(payload);
    transmissionCount_ = 1U;
    sentAtTick_ = currentTick;
    lastTransmissionTick_ = currentTick;
}

void FleetRouteDeliveryState::RecordApplied(std::uint32_t sequence) noexcept
{
    appliedSequence_ = sequence;
}

bool FleetRouteDeliveryState::Acknowledge(std::uint32_t sequence) noexcept
{
    // 예전 경로나 아직 적용되지 않은 경로의 응답으로 현재 재전송 대기를 지우지 않는다.
    if (pendingSequence_ != sequence || appliedSequence_ != sequence)
        return false;

    ClearPending();
    return true;
}

void FleetRouteDeliveryState::ClearPending() noexcept
{
    pendingSequence_ = 0U;
    pendingPayload_.clear();
    transmissionCount_ = 0U;
}

bool FleetRouteDeliveryState::CanRetry(
    std::uint64_t currentTick,
    std::uint64_t retryIntervalTicks,
    std::uint32_t maximumTransmissions) const noexcept
{
    // 모의 시간이 초기화됐거나 Tick이 아직 간격만큼 흐르지 않았다면 재전송하지 않는다.
    return HasPendingTransmission()
        && transmissionCount_ < maximumTransmissions
        && HasTickIntervalElapsed(
            currentTick, lastTransmissionTick_, retryIntervalTicks);
}

void FleetRouteDeliveryState::RecordRetry(std::uint64_t currentTick) noexcept
{
    lastTransmissionTick_ = currentTick;
    ++transmissionCount_;
}

void FormationTransitState::SetRoute(flight_planning::FormationRoutePlan route)
{
    // 새 구간은 이전 구간과 다른 메시지 흐름이므로 진행 번호와 전송 기록을 함께 비운다.
    route_ = std::move(route);
    waypointIndex_ = 0U;
    nextMessageSequence_ = 1U;
    sentFollowerTargets_.clear();
    lastTargetSentAtTick_.clear();
    lastAppliedSequence_.clear();
    lastNetworkNodeUpdateTick_.reset();
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

bool FormationTransitState::IsNewAppliedSequence(
    simulation::DroneId droneId,
    std::uint32_t sequence) const noexcept
{
    const auto previous = lastAppliedSequence_.find(droneId);
    return previous == lastAppliedSequence_.end() || sequence > previous->second;
}

void FormationTransitState::RecordAppliedSequence(
    simulation::DroneId droneId,
    std::uint32_t sequence)
{
    lastAppliedSequence_[droneId] = sequence;
}

bool FormationTransitState::ShouldSendFollowerTarget(
    simulation::DroneId droneId,
    const glm::dvec3& targetMeters,
    std::uint64_t currentTick,
    double changeThresholdMeters,
    std::uint64_t refreshIntervalTicks) const noexcept
{
    // 목표가 충분히 바뀌면 즉시 보내고, 그대로라면 일정 Tick마다 현재 목표를 다시 알린다.
    const auto previousTarget = sentFollowerTargets_.find(droneId);
    if (previousTarget == sentFollowerTargets_.end()
        || glm::distance(previousTarget->second, targetMeters) >= changeThresholdMeters)
        return true;

    const auto lastSent = lastTargetSentAtTick_.find(droneId);
    return lastSent == lastTargetSentAtTick_.end()
        || HasTickIntervalElapsed(currentTick, lastSent->second,
            refreshIntervalTicks);
}

void FormationTransitState::RecordFollowerTargetSent(
    simulation::DroneId droneId,
    const glm::dvec3& targetMeters,
    std::uint64_t currentTick)
{
    sentFollowerTargets_[droneId] = targetMeters;
    lastTargetSentAtTick_[droneId] = currentTick;
}

bool FormationTransitState::ShouldRefreshNetworkNodes(
    std::uint64_t currentTick,
    std::uint64_t refreshIntervalTicks) const noexcept
{
    // 모의 Tick이 초기화되면 이전 시각과 비교할 수 없으므로 새 기준 시각이 올 때까지 기다린다.
    return !lastNetworkNodeUpdateTick_
        || HasTickIntervalElapsed(
            currentTick, *lastNetworkNodeUpdateTick_, refreshIntervalTicks);
}

void FormationTransitState::RecordNetworkNodesUpdated(std::uint64_t currentTick) noexcept
{
    lastNetworkNodeUpdateTick_ = currentTick;
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
    nextWaypoint_ = 0U;
    routeReceived_ = false;
    waypointCommanded_ = false;
}

void FleetReconnaissanceState::SetPlannedRoute(
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    routeWaypointsMeters_ = std::move(routeWaypointsMeters);
    nextWaypoint_ = 0U;
    routeReceived_ = false;
    waypointCommanded_ = false;
}

std::vector<glm::dvec3> FleetReconnaissanceState::TakeRouteForDelivery()
{
    std::vector<glm::dvec3> route = std::move(routeWaypointsMeters_);
    routeWaypointsMeters_.clear();
    nextWaypoint_ = 0U;
    routeReceived_ = false;
    waypointCommanded_ = false;
    return route;
}

void FleetReconnaissanceState::ReceiveRoute(
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    routeWaypointsMeters_ = std::move(routeWaypointsMeters);
    nextWaypoint_ = 0U;
    routeReceived_ = true;
    waypointCommanded_ = false;
}

void FleetReconnaissanceState::WaitForRoute() noexcept
{
    routeWaypointsMeters_.clear();
    nextWaypoint_ = 0U;
    routeReceived_ = false;
    waypointCommanded_ = false;
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

FleetMissionState::AssignmentIterator FleetMissionState::FindAssignment(
    simulation::DroneId droneId) noexcept
{
    return assignments_.find(droneId);
}

FleetMissionState::ConstAssignmentIterator FleetMissionState::FindAssignment(
    simulation::DroneId droneId) const noexcept
{
    return assignments_.find(droneId);
}

void FleetMissionState::SetAssignment(
    simulation::DroneId droneId,
    FleetReconnaissanceState assignment)
{
    assignments_.insert_or_assign(droneId, std::move(assignment));
}

bool FleetMissionState::ShouldSendTelemetry(
    simulation::DroneId droneId,
    std::uint64_t currentTick,
    std::uint64_t intervalTicks) const noexcept
{
    const auto lastSent = lastTelemetrySentAtTick_.find(droneId);
    return lastSent == lastTelemetrySentAtTick_.end()
        || HasTickIntervalElapsed(currentTick, lastSent->second, intervalTicks);
}

void FleetMissionState::RecordTelemetryAttempt(
    simulation::DroneId droneId,
    std::uint64_t currentTick)
{
    lastTelemetrySentAtTick_[droneId] = currentTick;
}

void FleetMissionState::ResetTelemetry()
{
    telemetryAtLeader_.Clear();
    nextTelemetrySequence_ = 1U;
    lastTelemetrySentAtTick_.clear();
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

/** @brief 내부 임무 상태를 HUD에서 읽을 수 있는 한국어 문구로 바꾼다. */
std::string_view FleetDroneStatusLabel(FleetDroneStatus status) noexcept
{
    switch (status)
    {
    case FleetDroneStatus::RoutePending: return "경로 전달 대기";
    case FleetDroneStatus::TakeoffReady: return "출발 준비";
    case FleetDroneStatus::Reconnaissance: return "정찰 이동 중";
    case FleetDroneStatus::RegroupPreparing: return "재집결 준비";
    case FleetDroneStatus::Regrouping: return "집결 이동 중";
    case FleetDroneStatus::ReturnPreparing: return "복귀 준비";
    case FleetDroneStatus::Returning: return "복귀 중";
    case FleetDroneStatus::LowBatteryReturning: return "저전력 복귀 중";
    case FleetDroneStatus::LowBatteryLanding: return "저전력 착륙 중";
    case FleetDroneStatus::Landing: return "착륙 중";
    case FleetDroneStatus::ReturnLanding: return "복귀 착륙 중";
    case FleetDroneStatus::TargetReached: return "정찰 위치 도착";
    case FleetDroneStatus::RegroupComplete: return "재집결 완료";
    case FleetDroneStatus::ReturnComplete: return "복귀 완료";
    case FleetDroneStatus::LowBatteryReturnComplete: return "저전력 복귀 완료";
    case FleetDroneStatus::RouteFailed: return "경로 오류";
    case FleetDroneStatus::OutsideTargetLanding: return "목표 외 착륙";
    case FleetDroneStatus::BatteryReturnRouteFailed:
        return "배터리 부족 · 복귀 경로 오류";
    case FleetDroneStatus::AssignmentExpiredLanding:
        return "임무 권한 만료 · 안전 착륙";
    case FleetDroneStatus::RegroupFailed: return "집결 실패";
    case FleetDroneStatus::ReturnFailed: return "복귀 실패";
    case FleetDroneStatus::Other: return {};
    }
    return {};
}

/** @brief 드론 상태가 아직 임무 수행 중인지 판정한다. */
bool IsFleetMissionInProgress(FleetDroneStatus status) noexcept
{
    switch (status)
    {
    case FleetDroneStatus::RoutePending:
    case FleetDroneStatus::TakeoffReady:
    case FleetDroneStatus::Reconnaissance:
    case FleetDroneStatus::RegroupPreparing:
    case FleetDroneStatus::Regrouping:
    case FleetDroneStatus::ReturnPreparing:
    case FleetDroneStatus::Returning:
    case FleetDroneStatus::LowBatteryReturning:
    case FleetDroneStatus::LowBatteryLanding:
    case FleetDroneStatus::Landing:
    case FleetDroneStatus::ReturnLanding:
        return true;
    default:
        return false;
    }
}

std::optional<FleetMissionReport> FleetMissionState::FreshReport(
    simulation::DroneId droneId,
    const FleetReconnaissanceState& assignment,
    const simulation::SimulationCore& simulation) const
{
    if (droneId == leaderId_)
    {
        return FleetMissionReport{
            simulation.State(droneId).mode,
            assignment.Leg(),
            assignment.Status()};
    }

    const ReceivedPeerTelemetry* telemetry = telemetryAtLeader_.Fresh(
        droneId, simulation.TickCount());
    if (telemetry == nullptr)
        return std::nullopt;

    return FleetMissionReport{
        telemetry->state.mode,
        static_cast<FleetMissionLeg>(telemetry->state.missionLeg),
        telemetry->state.missionStatus};
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
        const auto assignment = assignments_.find(droneId);
        if (assignment == assignments_.end())
            return false;

        const auto report = FreshReport(droneId, assignment->second, simulation);
        if (!report || report->mode != simulation::FlightMode::Grounded)
            return false;

        const bool landedAfterLowBatteryReserve =
            assignment->second.IsLowBatteryReturnComplete()
            && report->status == FleetDroneStatus::LowBatteryReturnComplete;
        if (landedAfterLowBatteryReserve)
            return true;

        return report->leg == FleetMissionLeg::Reconnaissance
            && !IsFleetMissionInProgress(report->status);
    });
}

FleetMissionProgressDecision EvaluateFleetMissionProgress(
    FleetMissionLeg leg,
    std::span<const std::optional<FleetMissionReport>> reports) noexcept
{
    // 임무 단계는 편대원 전원의 보고로 판정한다. 한 대라도 보고가 없으면
    // 아직 실패나 완료를 확정할 정보가 없으므로 다음 Tick까지 기다린다.
    if (reports.empty())
        return {FleetMissionProgress::Waiting, {}};

    for (const auto& report : reports)
    {
        if (!report)
            return {FleetMissionProgress::Waiting, {}};

        const bool returnedForLowBattery =
            report->status == FleetDroneStatus::LowBatteryReturnComplete
            && report->mode == simulation::FlightMode::Grounded;
        if (returnedForLowBattery)
        {
            // 예비 배터리에 도달해 먼저 착륙한 기체는 정찰을 마친 것으로
            // 볼 수 없다. 복귀 단계에서는 이미 안전하게 착륙한 것으로 인정한다.
            if (leg == FleetMissionLeg::Reconnaissance)
                return {FleetMissionProgress::Failed,
                    "배터리 예비량에 도달해 정찰을 중단했습니다."};
            if (leg == FleetMissionLeg::Returning)
                continue;
        }

        // 이전 구간의 늦은 보고가 새 구간 완료로 오인되지 않게 구간도 맞춘다.
        if (report->leg != leg)
            return {FleetMissionProgress::Waiting, {}};

        switch (leg)
        {
        case FleetMissionLeg::Reconnaissance:
            // 도착 상태만으로는 충분하지 않다. 목표 지점에서 착륙 상태까지
            // 확인해야 해당 기체의 정찰 구간이 끝난 것으로 친다.
            if (report->status == FleetDroneStatus::RouteFailed
                || report->status == FleetDroneStatus::OutsideTargetLanding)
                return {FleetMissionProgress::Failed,
                    "정찰 목표에 도착하지 못해 임무를 중단했습니다."};
            if (report->mode != simulation::FlightMode::Grounded
                || report->status != FleetDroneStatus::TargetReached)
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionLeg::Regrouping:
            // 집결 구간은 모든 드론이 집결 완료를 보고해야 다음 구간으로 간다.
            if (report->status == FleetDroneStatus::RegroupFailed)
                return {FleetMissionProgress::Failed,
                    "모든 기체가 집결하지 못해 임무를 중단했습니다."};
            if (report->status != FleetDroneStatus::RegroupComplete)
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionLeg::Returning:
            // 복귀 완료는 기지 도착뿐 아니라 착륙 상태까지 포함한다.
            if (report->status == FleetDroneStatus::ReturnFailed)
                return {FleetMissionProgress::Failed,
                    "복귀에 실패한 드론이 있습니다."};
            if (report->mode != simulation::FlightMode::Grounded
                || report->status != FleetDroneStatus::ReturnComplete)
                return {FleetMissionProgress::Waiting, {}};
            break;
        }
    }

    return {leg == FleetMissionLeg::Returning
            ? FleetMissionProgress::Complete : FleetMissionProgress::Advance,
        {}};
}
}
