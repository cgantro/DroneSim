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

    /** @brief 현재 임무 구간이 완료 또는 실패 상태인지 확인한다. */
    [[nodiscard]] bool IsLegFinished() const noexcept;

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

/** @brief 편대 임무 실행에 필요한 드론별 상태를 한곳에 모은다. */
struct FleetMissionState
{
    std::unordered_map<simulation::DroneId, FleetReconnaissanceState>
        reconnaissanceByDrone;
    simulation::DroneId leaderId = 0;
    std::uint64_t nextAssignmentVersion = 1;

    FleetTelemetryStore telemetryAtLeader;
    std::uint32_t nextTelemetrySequence = 1;
    std::map<simulation::DroneId, std::uint64_t> lastTelemetrySentAtTick;

    FormationTransitState formationTransit;
    CompleteFleetMissionState completeMission;

    /** @brief 드론 하나라도 편대 정찰·집결·복귀 임무를 진행 중인지 확인한다. */
    [[nodiscard]] bool HasActiveReconnaissance() const noexcept
    {
        for (const auto& [droneId, reconnaissance] : reconnaissanceByDrone)
        {
            (void)droneId;
            if (IsFleetMissionInProgress(reconnaissance.status))
                return true;
        }
        return false;
    }

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
