#pragma once

#include "FleetMissionProgress.h"
#include "FleetTelemetry.h"
#include "flight_planning/FormationPlanner.h"
#include "simulation_core/SimulationCore.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

namespace dronesim::mission
{
/** @brief 경로 명령의 전송 확인과 재전송 상태를 관리한다. */
class FleetRouteDeliveryState
{
private:
    std::string pendingPayload_;
    std::uint32_t appliedSequence_ = 0U;
    std::uint32_t pendingSequence_ = 0U;
    std::uint32_t transmissionCount_ = 0U;
    std::uint64_t sentAtTick_ = 0U;
    std::uint64_t lastTransmissionTick_ = 0U;

public:
    /** @brief 현재·대기 순번보다 큰 다음 경로 순번을 계산한다. */
    [[nodiscard]] std::uint32_t NextSequence() const noexcept;

    /** @brief 현재 드론에 적용된 경로 순번을 돌려준다. */
    [[nodiscard]] std::uint32_t AppliedSequence() const noexcept
    {
        return appliedSequence_;
    }

    /** @brief 확인 응답을 기다리는 경로 순번을 돌려준다. */
    [[nodiscard]] std::uint32_t PendingSequence() const noexcept
    {
        return pendingSequence_;
    }

    /** @brief 재전송할 메시지가 있는지 확인한다. */
    [[nodiscard]] bool HasPendingTransmission() const noexcept
    {
        return pendingSequence_ != 0U;
    }

    /** @brief 재전송할 경로 내용을 읽기 전용으로 돌려준다. */
    [[nodiscard]] const std::string& PendingPayload() const noexcept
    {
        return pendingPayload_;
    }

    /** @brief 처음 경로를 보낸 Tick을 돌려준다. */
    [[nodiscard]] std::uint64_t SentAtTick() const noexcept { return sentAtTick_; }

    /** @brief 새 전송을 기록하고 확인 대기 상태로 바꾼다. */
    void Queue(
        std::uint32_t sequence, std::string payload, std::uint64_t currentTick);

    /** @brief 드론에 적용된 순번과 경로 적용 시각을 기록한다. */
    void RecordApplied(std::uint32_t sequence, std::uint64_t currentTick) noexcept;

    /** @brief 현재 경로를 확인한 응답이면 대기 메시지를 비운다. */
    [[nodiscard]] bool Acknowledge(std::uint32_t sequence) noexcept;

    /** @brief 새 직접 경로 적용 전에 이전 전송 대기를 지운다. */
    void ClearPending() noexcept;

    /** @brief 재전송 횟수와 간격을 만족해 다시 보낼 수 있는지 확인한다. */
    [[nodiscard]] bool CanRetry(
        std::uint64_t currentTick,
        std::uint64_t retryIntervalTicks,
        std::uint32_t maximumTransmissions) const noexcept;

    /** @brief 전송에 성공한 재시도 시각과 횟수를 기록한다. */
    void RecordRetry(std::uint64_t currentTick) noexcept;
};

/** @brief 드론 한 대에 배정된 정찰·집결·복귀 경로와 진행 상태다. */
class FleetReconnaissanceState final
{
private:
    std::vector<glm::dvec3> routeWaypointsMeters_;
    std::size_t nextWaypoint_ = 0U;
    bool routeReceived_ = false;
    bool waypointCommanded_ = false;
    bool lowBatteryReturn_ = false;
    FleetMissionLeg leg_ = FleetMissionLeg::Reconnaissance;
    FleetDroneStatus status_ = FleetDroneStatus::Other;
    FleetRouteDeliveryState routeDelivery_;
    std::size_t targetIndex_ = 0;
    glm::dvec3 startPositionMeters_{0.0};
    std::uint64_t assignmentVersion_ = 0;

public:
    /** @brief 드론에 배정된 정찰 목표 번호를 돌려준다. */
    [[nodiscard]] std::size_t TargetIndex() const noexcept { return targetIndex_; }

    /** @brief 임무 종료 후 돌아갈 출발 위치를 미터 단위로 돌려준다. */
    [[nodiscard]] const glm::dvec3& StartPositionMeters() const noexcept
    {
        return startPositionMeters_;
    }

    /** @brief 마지막으로 받은 임무 배정 버전을 돌려준다. */
    [[nodiscard]] std::uint64_t AssignmentVersion() const noexcept
    {
        return assignmentVersion_;
    }

    /** @brief 새 배정에서 정찰 목표와 복귀할 출발 위치를 함께 기록한다. */
    void SetInitialAssignment(
        std::size_t targetIndex, const glm::dvec3& startPositionMeters) noexcept
    {
        targetIndex_ = targetIndex;
        startPositionMeters_ = startPositionMeters;
    }

    /** @brief 통신으로 보낸 임무 배정의 버전을 기록한다. */
    void SetAssignmentVersion(std::uint64_t assignmentVersion) noexcept
    {
        assignmentVersion_ = assignmentVersion;
    }

    /** @brief 새 경로가 적용될 때 목표와 배정 버전을 함께 갱신한다. */
    void ApplyAssignment(
        std::size_t targetIndex,
        std::uint64_t assignmentVersion) noexcept
    {
        targetIndex_ = targetIndex;
        SetAssignmentVersion(assignmentVersion);
    }

    /** @brief 현재 구간의 진행 상태를 읽는다. */
    [[nodiscard]] FleetDroneStatus Status() const noexcept { return status_; }

    /** @brief 통신·비행 처리 결과를 현재 구간 상태로 기록한다. */
    void UpdateStatus(FleetDroneStatus nextStatus) noexcept { status_ = nextStatus; }

    /** @brief 현재 정찰·집결·복귀 구간을 읽는다. */
    [[nodiscard]] FleetMissionLeg Leg() const noexcept { return leg_; }

    /** @brief 배터리 부족으로 시작한 복귀인지 확인한다. */
    [[nodiscard]] bool IsLowBatteryReturn() const noexcept
    {
        return lowBatteryReturn_;
    }

    /** @brief 배터리 부족 복귀를 마치고 착륙 상태까지 기록했는지 확인한다. */
    [[nodiscard]] bool IsLowBatteryReturnComplete() const noexcept
    {
        return lowBatteryReturn_
            && status_ == FleetDroneStatus::LowBatteryReturnComplete;
    }

    /** @brief 경로 메시지의 확인·재전송을 맡는 전송 상태를 제공한다. */
    [[nodiscard]] FleetRouteDeliveryState& RouteDelivery() noexcept
    {
        return routeDelivery_;
    }
    [[nodiscard]] const FleetRouteDeliveryState& RouteDelivery() const noexcept
    {
        return routeDelivery_;
    }

    /** @brief 경로 계획 결과를 저장하고 진행 번호를 첫 경유점으로 맞춘다. */
    void SetPlannedRoute(std::vector<glm::dvec3> routeWaypointsMeters);

    /** @brief 통신으로 전달할 경로를 꺼내고 보관 중인 경로를 비운다. */
    [[nodiscard]] std::vector<glm::dvec3> TakeRouteForDelivery();

    /** @brief 새 구간의 경로 수신·진행 상태를 초기화한다. */
    void BeginLeg(FleetMissionLeg nextLeg) noexcept;

    /** @brief 드론에 실제 적용된 경로와 목표 상태를 기록한다. */
    void ReceiveRoute(std::vector<glm::dvec3> routeWaypointsMeters);

    /** @brief 경로가 통신으로 도착할 때까지 이전 경로를 숨긴다. */
    void WaitForRoute() noexcept;

    /** @brief 계획·수신된 경로를 읽기 전용으로 제공한다. */
    [[nodiscard]] const std::vector<glm::dvec3>& RouteWaypoints() const noexcept
    {
        return routeWaypointsMeters_;
    }

    /** @brief 현재 다음 경유점 번호를 돌려준다. */
    [[nodiscard]] std::size_t WaypointIndex() const noexcept { return nextWaypoint_; }

    /** @brief 통신으로 현재 구간의 경로를 받았는지 확인한다. */
    [[nodiscard]] bool HasReceivedRoute() const noexcept { return routeReceived_; }

    /** @brief 비행 조종기에 현재 구간의 첫 경유점을 전달했는지 확인한다. */
    [[nodiscard]] bool HasCommandedWaypoint() const noexcept { return waypointCommanded_; }

    /** @brief 아직 전달할 경유점이 남아 있는지 확인한다. */
    [[nodiscard]] bool HasRemainingWaypoints() const noexcept
    {
        return nextWaypoint_ < routeWaypointsMeters_.size();
    }

    /** @brief 현재 임무 구간이 완료 또는 실패 상태인지 확인한다. */
    [[nodiscard]] bool IsLegFinished() const noexcept;

    /** @brief 배터리 부족으로 복귀를 시작했음을 임무 상태에 기록한다. */
    void BeginLowBatteryReturn() noexcept;

    /** @brief 첫 경유점을 조종기에 전달하고 현재 구간에 맞는 비행 상태를 정한다. */
    void BeginWaypointCommand() noexcept;

    /** @brief 현재 경유점에 도착했으면 다음 점으로 이동한다. */
    [[nodiscard]] bool AdvanceWaypointIfReached(
        const glm::dvec3& positionMeters, double toleranceMeters) noexcept;

    /** @brief 아직 전달할 경유점이 있으면 그 위치를 돌려준다. */
    [[nodiscard]] const glm::dvec3* CurrentWaypoint() const noexcept;

    /** @brief 정찰 중이거나 목표에 도착한 상태에서 저전력 복귀를 시작할 수 있는지 확인한다. */
    [[nodiscard]] bool CanBeginLowBatteryReturn() const noexcept;

    /** @brief 지상에 도착한 위치를 바탕으로 현재 구간의 최종 상태를 정한다. */
    [[nodiscard]] FleetDroneStatus StatusAfterLanding(
        const glm::dvec3& positionMeters,
        const glm::dvec3& destinationMeters) const noexcept;
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

private:
    Phase phase_ = Phase::Idle;
    std::string status_ = "대기";
    flight_planning::FormationRoutePlan route_;
    std::size_t waypointIndex_ = 0;
    std::uint32_t nextMessageSequence_ = 1;
    std::map<simulation::DroneId, glm::dvec3> sentFollowerTargets_;
    std::map<simulation::DroneId, std::uint64_t> lastTargetSentAtTick_;
    std::map<simulation::DroneId, std::uint32_t> lastAppliedSequence_;
    std::optional<std::uint64_t> lastNetworkNodeUpdateTick_;

public:
    /** @brief 편대 이동이 현재 어느 단계인지 반환한다. */
    [[nodiscard]] Phase CurrentPhase() const noexcept { return phase_; }

    /** @brief 화면에 보여줄 현재 이동 안내를 읽기 전용으로 제공한다. */
    [[nodiscard]] std::string_view StatusText() const noexcept { return status_; }

    /** @brief 이동 단계가 실제로 진행 중인지 확인한다. */
    [[nodiscard]] bool IsActive() const noexcept
    {
        return phase_ == Phase::InProgress;
    }

    /** @brief 편대 경로를 교체하고 이전 경로의 진행·통신 기록을 초기화한다. */
    void SetRoute(flight_planning::FormationRoutePlan route);

    /** @brief 편대 경로를 읽기 전용으로 돌려준다. */
    [[nodiscard]] const flight_planning::FormationRoutePlan& Route() const noexcept
    {
        return route_;
    }

    /** @brief 현재 따라가는 경유점 번호를 돌려준다. */
    [[nodiscard]] std::size_t WaypointIndex() const noexcept { return waypointIndex_; }

    /** @brief 리더가 현재 경유점에 도착했으면 다음 점으로 이동한다. */
    void AdvanceWaypointIfReached(const glm::dvec3& positionMeters, double toleranceMeters);

    /** @brief 다음 편대 목표 메시지에 쓸 순번을 돌려주고 1 증가시킨다. */
    [[nodiscard]] std::uint32_t TakeNextMessageSequence() noexcept
    {
        return nextMessageSequence_++;
    }

    /** @brief 드론에 이미 적용한 순번보다 새 메시지 순번이 큰지 확인한다. */
    [[nodiscard]] bool IsNewAppliedSequence(
        simulation::DroneId droneId, std::uint32_t sequence) const noexcept;

    /** @brief 드론에 적용한 마지막 메시지 순번을 기록한다. */
    void RecordAppliedSequence(simulation::DroneId droneId, std::uint32_t sequence);

    /** @brief 편대원 목표가 달라졌거나 정기 갱신 시점인지 확인한다. */
    [[nodiscard]] bool ShouldSendFollowerTarget(
        simulation::DroneId droneId,
        const glm::dvec3& targetMeters,
        std::uint64_t currentTick,
        double changeThresholdMeters,
        std::uint64_t refreshIntervalTicks) const noexcept;

    /** @brief 전송에 성공한 편대원 목표와 Tick을 기억한다. */
    void RecordFollowerTargetSent(
        simulation::DroneId droneId,
        const glm::dvec3& targetMeters,
        std::uint64_t currentTick);

    /** @brief 링크 위치를 다시 계산할 시점인지 확인한다. */
    [[nodiscard]] bool ShouldRefreshNetworkNodes(
        std::uint64_t currentTick, std::uint64_t refreshIntervalTicks) const noexcept;

    /** @brief 링크 위치를 갱신한 Tick을 기록한다. */
    void RecordNetworkNodesUpdated(std::uint64_t currentTick) noexcept;

    /** @brief 새 편대 이동을 시작하고 출발 안내를 저장한다. */
    void Start(std::string status)
    {
        phase_ = Phase::InProgress;
        status_ = std::move(status);
    }

    /** @brief 편대 이동이 끝났음을 기록한다. */
    void Arrive(std::string status)
    {
        phase_ = Phase::Arrived;
        status_ = std::move(status);
    }

    /** @brief 실패 상태와 실패 안내를 함께 기록한다. */
    void Fail(std::string status)
    {
        phase_ = Phase::Failed;
        status_ = std::move(status);
    }

    /** @brief 전체 임무 실패에 따라 단계만 멈추고 기존 편대 안내는 보존한다. */
    void Stop() noexcept { phase_ = Phase::Failed; }

    /** @brief 진행 단계는 유지하고 화면에 표시할 안내만 갱신한다. */
    void UpdateStatus(std::string status) { status_ = std::move(status); }

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

private:
    Stage stage_ = Stage::Idle;
    std::map<simulation::DroneId, glm::dvec3> homePositionsMeters_;
    std::string failureReason_;
    bool waitingForReports_ = false;

public:
    /** @brief 전체 임무가 현재 어느 단계에 있는지 돌려준다. */
    [[nodiscard]] Stage CurrentStage() const noexcept { return stage_; }

    /** @brief 각 드론의 출발 위치를 임무 기준 위치로 저장한다. */
    void RecordHomePosition(simulation::DroneId droneId, const glm::dvec3& positionMeters)
    {
        homePositionsMeters_[droneId] = positionMeters;
    }

    /** @brief 직전 임무의 출발 위치를 비운다. */
    void ClearHomePositions() noexcept { homePositionsMeters_.clear(); }

    /** @brief 저장한 출발 위치 목록을 읽기 전용으로 제공한다. */
    [[nodiscard]] const std::map<simulation::DroneId, glm::dvec3>& HomePositions() const noexcept
    {
        return homePositionsMeters_;
    }

    /** @brief 지정한 드론의 출발 위치가 저장돼 있는지 확인한다. */
    [[nodiscard]] bool HasHomePosition(simulation::DroneId droneId) const noexcept
    {
        return homePositionsMeters_.contains(droneId);
    }

    /** @brief 지정한 드론의 저장된 출발 위치를 읽는다. */
    [[nodiscard]] const glm::dvec3& HomePositionMeters(simulation::DroneId droneId) const
    {
        return homePositionsMeters_.at(droneId);
    }

    /** @brief 단계가 바뀌면 이전 단계의 대기·실패 정보를 함께 지운다. */
    void SetStage(Stage nextStage)
    {
        stage_ = nextStage;
        waitingForReports_ = false;
        failureReason_.clear();
    }

    /** @brief 전체 임무가 실패한 이유를 보관하고 실패 단계로 바꾼다. */
    void Fail(std::string reason)
    {
        stage_ = Stage::Failed;
        waitingForReports_ = false;
        failureReason_ = std::move(reason);
    }

    /** @brief 다음 상태 보고가 도착할 때까지 기다린다. */
    void WaitForReports() noexcept
    {
        waitingForReports_ = true;
    }

    /** @brief 화면 문구를 별도 상태로 저장하지 않고 임무 단계에서 만든다. */
    [[nodiscard]] std::string_view StatusText() const noexcept
    {
        if (stage_ == Stage::Failed)
            return failureReason_.empty() ? std::string_view{"전체 임무 실패"} : failureReason_;
        if (waitingForReports_)
            return "드론 상태 보고를 기다리는 중";

        switch (stage_)
        {
        case Stage::Idle: return "대기";
        case Stage::FormationTransit: return "M01 편대 이동 중";
        case Stage::Reconnaissance: return "M02 분산 정찰 중";
        case Stage::Regrouping: return "M03 재집결 중";
        case Stage::Returning: return "M03 기지로 복귀 중";
        case Stage::Completed: return "전체 임무 완료";
        case Stage::Failed: return "전체 임무 실패";
        }
        return "대기";
    }

    /** @brief 전체 임무가 실제로 진행 중인 단계인지 확인한다. */
    [[nodiscard]] bool IsActive() const noexcept
    {
        switch (stage_)
        {
        case Stage::FormationTransit:
        case Stage::Reconnaissance:
        case Stage::Regrouping:
        case Stage::Returning:
            return true;
        case Stage::Idle:
        case Stage::Completed:
        case Stage::Failed:
            return false;
        }
        return false;
    }
};

/** @brief 편대 임무의 드론 배정과 통신 상태를 소유한다. */
class FleetMissionState
{
public:
    using AssignmentMap =
        std::unordered_map<simulation::DroneId, FleetReconnaissanceState>;
    using AssignmentIterator = AssignmentMap::iterator;
    using ConstAssignmentIterator = AssignmentMap::const_iterator;

private:
    AssignmentMap assignments_;
    simulation::DroneId leaderId_ = 0;
    std::uint64_t nextAssignmentVersion_ = 1;
    FleetTelemetryStore telemetryAtLeader_;
    std::uint32_t nextTelemetrySequence_ = 1;
    std::map<simulation::DroneId, std::uint64_t> lastTelemetrySentAtTick_;
    FormationTransitState formationTransit_;
    CompleteFleetMissionState completeMission_;

public:
    /** @brief 편대원 배정을 읽기 전용으로 순회한다. */
    [[nodiscard]] const AssignmentMap& Assignments() const noexcept
    {
        return assignments_;
    }

    /** @brief 지정한 드론의 배정을 수정할 수 있도록 찾는다. */
    [[nodiscard]] AssignmentIterator FindAssignment(
        simulation::DroneId droneId) noexcept;

    /** @brief 지정한 드론의 배정을 읽기 전용으로 찾는다. */
    [[nodiscard]] ConstAssignmentIterator FindAssignment(
        simulation::DroneId droneId) const noexcept;

    /** @brief 배정 목록의 끝을 반환해 찾기 결과와 비교할 수 있게 한다. */
    [[nodiscard]] ConstAssignmentIterator AssignmentsEnd() const noexcept
    {
        return assignments_.end();
    }

    /** @brief 지정한 드론의 배정을 참조로 가져온다. 배정이 없으면 표준 범위 오류가 난다. */
    [[nodiscard]] FleetReconnaissanceState& Assignment(simulation::DroneId droneId)
    {
        return assignments_.at(droneId);
    }
    [[nodiscard]] const FleetReconnaissanceState& Assignment(
        simulation::DroneId droneId) const
    {
        return assignments_.at(droneId);
    }

    /** @brief 배정이 하나 이상 있는지 확인한다. */
    [[nodiscard]] bool HasAssignments() const noexcept { return !assignments_.empty(); }

    /** @brief 배정을 추가하거나 해당 드론의 기존 배정을 교체한다. */
    void SetAssignment(
        simulation::DroneId droneId, FleetReconnaissanceState assignment);

    /** @brief 모든 드론 배정을 지우고 이전 임무를 비운다. */
    void ClearAssignments() noexcept { assignments_.clear(); }

    /** @brief 편대장 식별값을 읽거나 새 임무의 편대장을 지정한다. */
    [[nodiscard]] simulation::DroneId LeaderId() const noexcept { return leaderId_; }
    void SetLeaderId(simulation::DroneId droneId) noexcept { leaderId_ = droneId; }

    /** @brief 다음 임무 배정에 쓸 버전을 받고 번호를 하나 올린다. */
    [[nodiscard]] std::uint64_t TakeNextAssignmentVersion() noexcept
    {
        return nextAssignmentVersion_++;
    }

    /** @brief 편대장이 받은 원격 드론 상태 기록에 접근한다. */
    [[nodiscard]] FleetTelemetryStore& TelemetryAtLeader() noexcept
    {
        return telemetryAtLeader_;
    }
    [[nodiscard]] const FleetTelemetryStore& TelemetryAtLeader() const noexcept
    {
        return telemetryAtLeader_;
    }

    /** @brief 다음 텔레메트리 메시지 순번을 받고 번호를 하나 올린다. */
    [[nodiscard]] std::uint32_t TakeNextTelemetrySequence() noexcept
    {
        return nextTelemetrySequence_++;
    }

    /** @brief 새 임무를 위해 텔레메트리 순번과 수신·전송 기록을 초기화한다. */
    void ResetTelemetry();

    /** @brief 지정한 드론의 상태를 보낼 시간이 되었는지 확인한다. */
    [[nodiscard]] bool ShouldSendTelemetry(
        simulation::DroneId droneId,
        std::uint64_t currentTick,
        std::uint64_t intervalTicks) const noexcept;

    /** @brief 성공 여부와 관계없이 메시지 전달 시도 시각을 기록한다. */
    void RecordTelemetryAttempt(
        simulation::DroneId droneId, std::uint64_t currentTick);

    /** @brief 편대 이동과 전체 임무 진행 객체를 읽고 갱신한다. */
    [[nodiscard]] FormationTransitState& FormationTransit() noexcept
    {
        return formationTransit_;
    }
    [[nodiscard]] const FormationTransitState& FormationTransit() const noexcept
    {
        return formationTransit_;
    }
    [[nodiscard]] CompleteFleetMissionState& CompleteMission() noexcept
    {
        return completeMission_;
    }
    [[nodiscard]] const CompleteFleetMissionState& CompleteMission() const noexcept
    {
        return completeMission_;
    }

    /** @brief 드론 하나라도 편대 정찰·집결·복귀 임무를 진행 중인지 확인한다. */
    [[nodiscard]] bool HasActiveReconnaissance() const noexcept;

    /** @brief 리더의 현재 상태나 일정 시간 안에 받은 편대원 보고를 읽는다. */
    [[nodiscard]] std::optional<FleetMissionReport> FreshReport(
        simulation::DroneId droneId,
        const FleetReconnaissanceState& assignment,
        const simulation::SimulationCore& simulation) const;

    /**
     * @brief 한 드론의 현재 임무 보고를 모의 상태나 리더의 통신 기록에서 만든다.
     * @details 리더는 로컬 상태를 읽고, 편대원은 최신 통신 상태를 읽는다. 통신이
     * 끊긴 뒤에도 임무 단계에 맞는 착륙 완료 기록은 안전한 최종 상태로 인정한다.
     */
    [[nodiscard]] std::optional<FleetMissionReport> CurrentReport(
        simulation::DroneId droneId,
        const FleetReconnaissanceState& assignment,
        const simulation::SimulationCore& simulation,
        CompleteFleetMissionState::Stage currentStage) const;

    /** @brief 모든 편대원이 정찰을 마치고 복귀 명령을 받을 수 있는지 확인한다. */
    [[nodiscard]] bool AllDronesReadyToReturn(
        std::span<const simulation::DroneId> droneIds,
        const simulation::SimulationCore& simulation) const;
};
}
