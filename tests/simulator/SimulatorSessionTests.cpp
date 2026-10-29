#include "AeroSimLabApp.h"
#include "FleetMissionProgress.h"
#include "FleetTelemetry.h"
#include "TrainingRangeScene.h"

#include "simulation_core/SimulationCore.h"
#include "flight_planning/RoutePlanner.h"
#include "terrain/TerrainGrid.h"

#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"
#include "graphics/Camera.h"

#include <glm/glm.hpp>
#include <glm/geometric.hpp>
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace aerosimlab;

namespace aerosimlab::simulator
{
class AeroSimLabAppTestAccess
{
public:
    static void VerifyCompleteMissionRequiresFourDrones()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_EQ(app.simulation_.DroneIds().size(), 4U);
        EXPECT_TRUE(app.CompleteFleetMissionCanStart());

        app.fleetReconnaissance_[app.simulation_.PrimaryDroneId()].status =
            detail::FleetDroneStatus::Reconnaissance;
        EXPECT_FALSE(app.FleetReconnaissanceCanStart());
        EXPECT_FALSE(app.CompleteFleetMissionCanStart());
        app.fleetReconnaissance_.clear();
        EXPECT_TRUE(app.CompleteFleetMissionCanStart());

        const auto extraDrone = app.simulation_.AddDrone({0.0, 0.0, 0.0});
        EXPECT_FALSE(app.CompleteFleetMissionCanStart());
        ASSERT_TRUE(app.simulation_.RemoveDrone(extraDrone));
        const auto lastDrone = app.simulation_.DroneIds().back();
        ASSERT_TRUE(app.simulation_.RemoveDrone(lastDrone));
        EXPECT_FALSE(app.CompleteFleetMissionCanStart());
    }

    static void VerifyReconnaissanceKeepsTheFirstWaypoint()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        const auto droneId = app.simulation_.PrimaryDroneId();
        const auto expectedRoute = flight_planning::PlanRoute(
            app.terrainGrid_, app.trainingRangeScene_->BuildingObstacles(),
            app.simulation_.State(droneId).positionMeters,
            app.reconTargets_.front().approachPositionMeters,
            3.0, app.cameraTransitAltitudeMeters_);
        ASSERT_TRUE(expectedRoute);
        ASSERT_FALSE(expectedRoute->empty());
        ASSERT_TRUE(app.StartFleetReconnaissance());

        // 경로 탐색 결과는 출발점을 제외하므로 첫 항목도 임무에 필요하다.
        EXPECT_EQ(app.fleetReconnaissance_.at(droneId).routeWaypointsMeters,
            *expectedRoute);
    }

    static void UseConnectedNetworkForTest(AeroSimLabApp& app)
    {
        communication::VirtualMeshSettings settings;
        settings.maximumLinkRangeMeters = 1'000.0;
        app.formationNetwork_ = std::make_unique<
            communication::VirtualMeshNetwork>(settings);
        app.formationRadioObstacles_.clear();
        app.formationTransit_.lastNetworkNodeUpdateTick.reset();
    }

    static void VerifyObserverCameraKeepsAUsableFollowDistance()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        app.UpdateObserverCamera(1280, 720);

        const auto distance = glm::distance(
            app.observerCamera_->GetPosition(), app.observerCamera_->GetTarget());
        EXPECT_GT(distance, 5.0F);
    }

    static void VerifyObserverCameraCanFollowEachDrone()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        const auto droneIds = app.simulation_.DroneIds();
        ASSERT_EQ(droneIds.size(), 4U);

        app.UpdateObserverCamera(1280, 720);
        const glm::vec3 primaryTarget = app.observerCamera_->GetTarget();

        app.observerDroneId_ = droneIds[2U];
        app.UpdateObserverCamera(1280, 720);
        const glm::vec3 selectedTarget = app.observerCamera_->GetTarget();
        EXPECT_NE(primaryTarget, selectedTarget);
        EXPECT_GT(glm::distance(
            app.observerCamera_->GetPosition(), app.observerCamera_->GetTarget()), 5.0F);
    }

    static void VerifyGroundControlShowsOnlyDeliveredPeerPositions()
    {
        AeroSimLabApp app({
            .smokeTest = true,
            .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));
        UseConnectedNetworkForTest(app);
        app.formationTransit_.phase = AeroSimLabApp::FormationTransitState::Phase::InProgress;

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        const auto disconnectedDroneId = droneIds[2U];
        ASSERT_EQ(app.simulation_.Apply(disconnectedDroneId,
            {simulation::DroneCommand::Kind::TakeOff}),
            simulation::CommandResult::Accepted);
        for (int tick = 0; tick < 360; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }
        ASSERT_EQ(app.telemetryAtGroundControl_.size(), droneIds.size());

        const auto mode = app.simulation_.State(disconnectedDroneId).mode;
        ASSERT_TRUE(mode == simulation::FlightMode::Hovering
            || mode == simulation::FlightMode::Flying
            || mode == simulation::FlightMode::FollowingWaypoint);
        const auto originalReport = app.telemetryAtGroundControl_.at(
            disconnectedDroneId).state.positionMeters;
        for (const auto droneId : droneIds)
            app.formationNetwork_->SetNodeRadioEnabled(droneId, false);

        simulation::DroneCommand waypoint;
        waypoint.kind = simulation::DroneCommand::Kind::SetWaypoint;
        waypoint.vector = app.simulation_.State(disconnectedDroneId).positionMeters
            + glm::dvec3{35.0, 0.0, 0.0};
        ASSERT_EQ(app.simulation_.Apply(disconnectedDroneId, waypoint),
            simulation::CommandResult::Accepted);
        for (int tick = 0; tick < 300; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }

        const auto& received = app.telemetryAtGroundControl_.at(disconnectedDroneId);
        EXPECT_DOUBLE_EQ(received.state.positionMeters.x, originalReport.x);
        EXPECT_DOUBLE_EQ(received.state.positionMeters.y, originalReport.y);
        EXPECT_DOUBLE_EQ(received.state.positionMeters.z, originalReport.z);
        EXPECT_GT(glm::distance(
            app.simulation_.State(disconnectedDroneId).positionMeters,
            received.state.positionMeters), 1.0);
        EXPECT_GT(app.simulation_.TickCount() - received.receivedAtTick, 0U);
        const auto lastReceivedTick = received.receivedAtTick;
        for (const auto droneId : droneIds)
            app.formationNetwork_->SetNodeRadioEnabled(droneId, true);
        for (int tick = 0; tick < 360; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }
        EXPECT_GT(app.telemetryAtGroundControl_.at(disconnectedDroneId).receivedAtTick,
            lastReceivedTick);
    }

    static void VerifyGroundControlReceivesEachDroneDirectly()
    {
        AeroSimLabApp app({
            .smokeTest = true,
            .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));
        UseConnectedNetworkForTest(app);
        app.formationTransit_.phase = AeroSimLabApp::FormationTransitState::Phase::InProgress;

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        // 시험용 통신망은 연결 설정만 있고 노드 목록은 첫 갱신 때 만들어진다.
        // 한 번 상태를 주고받은 뒤 차단해야 차단 직전 전송분이 결과에 섞이지 않는다.
        app.AdvanceFleetNetwork();
        app.simulation_.AdvanceOneTick();
        app.AdvanceFleetNetwork();
        app.telemetryAtGroundControl_.clear();
        app.formationNetwork_->SetNodeRadioEnabled(droneIds.front(), false);
        for (int tick = 0; tick < 240; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }

        EXPECT_FALSE(app.telemetryAtGroundControl_.contains(droneIds.front()));
        for (std::size_t i = 1; i < droneIds.size(); ++i)
            ASSERT_TRUE(app.telemetryAtGroundControl_.contains(droneIds[i]));

        const auto blockedDroneId = droneIds[2];
        app.formationNetwork_->SetNodeRadioEnabled(blockedDroneId, false);
        // 이미 전송 중인 메시지가 정리된 뒤 마지막 수신 시각을 확인한다.
        for (int tick = 0; tick < 30; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }
        const auto lastReceivedTick =
            app.telemetryAtGroundControl_.at(blockedDroneId).receivedAtTick;
        for (int tick = 0; tick < 120; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }
        EXPECT_EQ(app.telemetryAtGroundControl_.at(blockedDroneId).receivedAtTick,
            lastReceivedTick);
        EXPECT_GT(app.telemetryAtGroundControl_.at(droneIds[1]).receivedAtTick,
            lastReceivedTick);

        app.formationNetwork_->SetNodeRadioEnabled(blockedDroneId, true);
        for (int tick = 0; tick < 480; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }
        EXPECT_GT(app.telemetryAtGroundControl_.at(blockedDroneId).receivedAtTick,
            lastReceivedTick);
    }

    static void VerifyFormationTransitSafelyLandsWhenFollowerTelemetryIsMissing()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        for (const auto droneId : droneIds)
        {
            auto state = app.simulation_.State(droneId);
            state.positionMeters.z = 10.0;
            state.mode = simulation::FlightMode::Hovering;
            app.simulation_.SetInitialState(droneId, state);
        }

        app.formationTransit_.phase = AeroSimLabApp::FormationTransitState::Phase::InProgress;
        app.formationTransit_.route.leaderStartMeters =
            app.simulation_.State(droneIds.front()).positionMeters;
        app.formationTransit_.route.leaderWaypointsMeters.push_back(
            app.formationTransit_.route.leaderStartMeters + glm::dvec3{10.0, 0.0, 0.0});
        app.completeFleetMission_.stage =
            AeroSimLabApp::CompleteFleetMissionState::Stage::FormationTransit;

        app.AdvanceFormationTransit();

        EXPECT_EQ(app.formationTransit_.phase,
            AeroSimLabApp::FormationTransitState::Phase::Failed);
        EXPECT_EQ(app.completeFleetMission_.stage,
            AeroSimLabApp::CompleteFleetMissionState::Stage::Failed);
        EXPECT_EQ(app.completeFleetMission_.status,
            "추종 드론 상태 보고가 끊겨 편대를 안전 착륙시킵니다.");
        for (const auto droneId : droneIds)
            EXPECT_EQ(app.simulation_.State(droneId).mode,
                simulation::FlightMode::Landing);
    }

    static void VerifyFormationResultDoesNotDependOnStatusText()
    {
        using FormationPhase = AeroSimLabApp::FormationTransitState::Phase;
        using MissionStage = AeroSimLabApp::CompleteFleetMissionState::Stage;
        AeroSimLabApp app({.smokeTest = true,
            .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));

        for (const auto droneId : app.fleetDroneIds_)
        {
            auto state = app.simulation_.State(droneId);
            state.positionMeters.z += 15.0;
            state.mode = simulation::FlightMode::Hovering;
            app.simulation_.SetInitialState(droneId, state);
            app.completeFleetMission_.homePositionsMeters[droneId] = state.positionMeters;
            if (droneId != app.fleetLeaderId_)
            {
                AeroSimLabApp::ReceivedPeerTelemetry report;
                report.state.positionMeters = state.positionMeters;
                report.state.mode = state.mode;
                report.receivedAtTick = app.simulation_.TickCount();
                app.peerTelemetryAtLeader_[droneId] = report;
            }
        }

        // 도착 상태라면 안내 문구가 바뀌어도 다음 정찰 단계를 시작한다.
        app.completeFleetMission_.stage = MissionStage::FormationTransit;
        app.formationTransit_.phase = FormationPhase::Arrived;
        app.formationTransit_.status = "다른 도착 안내";
        app.AdvanceCompleteFleetMission();
        ASSERT_EQ(app.completeFleetMission_.stage, MissionStage::Reconnaissance)
            << app.completeFleetMission_.status;
        app.FailCompleteFleetMission("정찰 단계 중단");
        EXPECT_EQ(app.formationTransit_.phase, FormationPhase::Arrived);

        // 실패 상태는 도착 문구가 남아 있어도 성공으로 처리하지 않는다.
        app.completeFleetMission_.stage = MissionStage::FormationTransit;
        app.formationTransit_.phase = FormationPhase::Failed;
        app.formationTransit_.status = "목적지 도착 · 편대 유지 중";
        app.AdvanceCompleteFleetMission();
        EXPECT_EQ(app.completeFleetMission_.stage, MissionStage::Failed);
        EXPECT_EQ(app.completeFleetMission_.status,
            "M01 편대 이동이 끝나지 않아 전체 임무를 중단했습니다.");
    }

    static void VerifyCompleteFleetMissionFinishesAndLands()
    {
        using Stage = AeroSimLabApp::CompleteFleetMissionState::Stage;
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        const auto droneIds = app.simulation_.DroneIds();

        ASSERT_TRUE(app.StartCompleteFleetMission()) << app.formationTransit_.status;
        bool sawReconnaissance = false;
        bool sawRegrouping = false;
        bool sawReturning = false;
        constexpr std::uint64_t maximumMissionTicks = 60000U;
        std::map<simulation::DroneId, std::uint64_t> lastAssignmentVersions;
        std::vector<std::string> assignmentHistory;
        simulation::DroneId previousLeaderId = app.fleetLeaderId_;
        for (std::uint64_t tick = 0U; tick < maximumMissionTicks; ++tick)
        {
            app.AdvanceSimulationTick();
            if (previousLeaderId != app.fleetLeaderId_)
            {
                assignmentHistory.push_back("Tick "
                    + std::to_string(app.simulation_.TickCount())
                    + ": 리더 " + std::to_string(previousLeaderId) + " → "
                    + std::to_string(app.fleetLeaderId_));
                previousLeaderId = app.fleetLeaderId_;
            }
            for (const auto& [droneId, mission] : app.fleetReconnaissance_)
            {
                auto& lastVersion = lastAssignmentVersions[droneId];
                if (mission.assignmentVersion == lastVersion)
                    continue;
                assignmentHistory.push_back("Tick "
                    + std::to_string(app.simulation_.TickCount()) + ": 드론 "
                    + std::to_string(droneId) + " 목표 "
                    + std::to_string(mission.targetIndex) + " 버전 "
                    + std::to_string(mission.assignmentVersion) + " 상태 "
                    + std::string(simulator::detail::FleetDroneStatusLabel(
                        mission.status)));
                lastVersion = mission.assignmentVersion;
            }
            sawReconnaissance = sawReconnaissance
                || app.completeFleetMission_.stage == Stage::Reconnaissance;
            sawRegrouping = sawRegrouping
                || app.completeFleetMission_.stage == Stage::Regrouping;
            sawReturning = sawReturning
                || app.completeFleetMission_.stage == Stage::Returning;
            if (app.completeFleetMission_.stage == Stage::Completed
                || app.completeFleetMission_.stage == Stage::Failed)
                break;
        }

        std::ostringstream failureContext;
        failureContext << "전체 임무가 끝나지 않았습니다. 상태: "
            << app.completeFleetMission_.status << ", Tick: "
            << app.simulation_.TickCount();
        if (app.completeFleetMission_.stage != Stage::Completed)
        {
            const auto& leader = app.simulation_.State(app.fleetLeaderId_);
            failureContext << ", 리더 ID " << app.fleetLeaderId_ << " 위치: ("
                << leader.positionMeters.x << ", "
                << leader.positionMeters.y << ", " << leader.positionMeters.z
                << "), GCS 위치: (" << app.groundControlPositionMeters_.x << ", "
                << app.groundControlPositionMeters_.y << ", "
                << app.groundControlPositionMeters_.z << "), 임무 단계 "
                << static_cast<int>(app.completeFleetMission_.stage);
            for (const auto droneId : droneIds)
            {
                const auto& drone = app.simulation_.State(droneId);
                failureContext << "; 드론 " << droneId << " 위치 ("
                    << drone.positionMeters.x << ", " << drone.positionMeters.y
                    << ", " << drone.positionMeters.z << ")";
                if (droneId != app.fleetLeaderId_)
                {
                    const auto report = app.peerTelemetryAtLeader_.find(droneId);
                    failureContext << ", 리더와 거리 "
                        << glm::distance(drone.positionMeters,
                            leader.positionMeters) << "m, 보고 경과 ";
                    if (report == app.peerTelemetryAtLeader_.end())
                        failureContext << "없음";
                    else
                        failureContext << app.simulation_.TickCount()
                            - report->second.receivedAtTick << " Tick";
                }
                const auto mission = app.fleetReconnaissance_.find(droneId);
                if (mission != app.fleetReconnaissance_.end())
                    failureContext << ", 목표 " << mission->second.targetIndex
                        << ", 상태 " << simulator::detail::FleetDroneStatusLabel(
                            mission->second.status);
            }
            const auto& metrics = app.formationNetwork_->Metrics();
            failureContext << "; 통신 통계 전달 " << metrics.deliveredMessages
                << ", 폐기 " << metrics.droppedMessages;
            const auto events = app.formationNetwork_->Events();
            const std::size_t firstEvent = events.size() > 12U
                ? events.size() - 12U : 0U;
            for (std::size_t index = firstEvent; index < events.size(); ++index)
            {
                const auto& event = events[index];
                failureContext << "; 사건 " << static_cast<int>(event.kind) << ' '
                    << event.source << "->" << event.destination << ' '
                    << event.detail;
            }
            const std::size_t firstAssignment = assignmentHistory.size() > 24U
                ? assignmentHistory.size() - 24U : 0U;
            for (std::size_t index = firstAssignment;
                index < assignmentHistory.size(); ++index)
                failureContext << "; 배정 이력 " << assignmentHistory[index];
        }

        EXPECT_EQ(app.completeFleetMission_.stage, Stage::Completed)
            << failureContext.str();
        EXPECT_TRUE(sawReconnaissance) << "M02 정찰 단계에 도달하지 못했습니다.";
        EXPECT_TRUE(sawRegrouping) << "M03 재집결 단계에 도달하지 못했습니다.";
        EXPECT_TRUE(sawReturning) << "기지 복귀 단계에 도달하지 못했습니다.";
        for (const auto droneId : app.simulation_.DroneIds())
            EXPECT_EQ(app.simulation_.State(droneId).mode,
                simulation::FlightMode::Grounded)
                << "전체 임무가 끝난 뒤 드론 " << droneId << "이 지상에 있지 않습니다.";
    }

    static void VerifyFleetDroneReturnsWhenBatteryReachesReserve()
    {
        AeroSimLabApp app({
            .smokeTest = true,
            .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.StartFleetReconnaissance());
        const auto lowBatteryDroneId = app.fleetDroneIds_.at(1U);
        const std::size_t unfinishedTargetIndex =
            app.fleetReconnaissance_.at(lowBatteryDroneId).targetIndex;
        app.simulation_.ConfigureBattery(lowBatteryDroneId,
            {10.0, 40.0, 120.0, 99.9});

        for (std::uint64_t tick = 0U; tick < 30000U; ++tick)
        {
            app.AdvanceSimulationTick();
            const auto& mission = app.fleetReconnaissance_.at(lowBatteryDroneId);
            if (mission.lowBatteryReturn
                && mission.status
                    == simulator::detail::FleetDroneStatus::LowBatteryReturnComplete)
                break;
        }

        const auto& mission = app.fleetReconnaissance_.at(lowBatteryDroneId);
        EXPECT_TRUE(app.simulation_.Battery(lowBatteryDroneId)
            .IsReturningForLowBattery());
        EXPECT_TRUE(mission.lowBatteryReturn);
        EXPECT_EQ(mission.leg, simulator::FleetMissionLeg::Returning);
        EXPECT_EQ(mission.status,
            simulator::detail::FleetDroneStatus::LowBatteryReturnComplete);
        EXPECT_LE(glm::distance(app.simulation_.State(lowBatteryDroneId).positionMeters,
            mission.startPositionMeters), 5.0);
        const bool targetWasReassigned = std::any_of(app.fleetDroneIds_.begin(),
            app.fleetDroneIds_.end(), [&app, lowBatteryDroneId, unfinishedTargetIndex](auto id)
            {
                return id != lowBatteryDroneId
                    && app.fleetReconnaissance_.at(id).targetIndex
                        == unfinishedTargetIndex;
            });
        EXPECT_FALSE(targetWasReassigned);
    }

};
}

void Verify_PeerTelemetryPreservesStateAndRejectsInvalidValues()
{
    const glm::dvec3 positionMeters{12.5, -7.25, 31.0};
    using simulator::detail::FleetDroneStatus;
    const std::string payload = simulator::detail::EncodePeerTelemetry(
        positionMeters, simulation::FlightMode::Hovering, 2U,
        FleetDroneStatus::ReturnLanding,
        17U);
    const auto telemetry = simulator::detail::DecodePeerTelemetry(payload);
    ASSERT_TRUE(telemetry.has_value());
    EXPECT_EQ(telemetry->positionMeters, positionMeters);
    EXPECT_EQ(telemetry->mode, simulation::FlightMode::Hovering);
    EXPECT_EQ(telemetry->missionLeg, 2U);
    EXPECT_EQ(telemetry->missionStatus, FleetDroneStatus::ReturnLanding);
    EXPECT_EQ(telemetry->assignmentVersion, 17U);
    EXPECT_FALSE(simulator::detail::DecodePeerTelemetry(payload + " 2400").has_value());

    EXPECT_FALSE(simulator::detail::DecodePeerTelemetry(
        simulator::detail::EncodePeerTelemetry(positionMeters,
            simulation::FlightMode::Hovering, 3U, FleetDroneStatus::Other,
            17U)).has_value());
    EXPECT_FALSE(simulator::detail::DecodePeerTelemetry(
        simulator::detail::EncodePeerTelemetry(positionMeters,
            simulation::FlightMode::Hovering, 2U,
            static_cast<FleetDroneStatus>(255U), 17U)).has_value());
}

void Verify_FleetMissionProgressWaitsForEveryCurrentReport()
{
    using simulator::FleetMissionLeg;
    using simulator::detail::FleetMissionProgress;
    using simulator::detail::FleetMissionReport;
    using simulator::detail::FleetDroneStatus;
    using simulation::FlightMode;

    EXPECT_TRUE(simulator::detail::IsFleetMissionInProgress(
        FleetDroneStatus::Reconnaissance));
    EXPECT_EQ(simulator::detail::FleetDroneStatusLabel(
        FleetDroneStatus::TargetReached), "정찰 위치 도착");
    EXPECT_FALSE(simulator::detail::IsFleetMissionInProgress(
        FleetDroneStatus::TargetReached));

    std::vector<std::optional<FleetMissionReport>> reports{
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Reconnaissance, FleetDroneStatus::TargetReached},
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Reconnaissance, FleetDroneStatus::TargetReached},
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Reconnaissance, FleetDroneStatus::TargetReached},
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Reconnaissance, FleetDroneStatus::TargetReached}};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Advance);

    reports.back().reset();
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Waiting);
    reports.back() = FleetMissionReport{
        FlightMode::Grounded, FleetMissionLeg::Regrouping,
        FleetDroneStatus::RegroupComplete};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Waiting);

    reports.back() = FleetMissionReport{FlightMode::Grounded,
        FleetMissionLeg::Reconnaissance, FleetDroneStatus::RouteFailed};
    const auto failed = simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports);
    EXPECT_EQ(failed.progress, FleetMissionProgress::Failed);
    EXPECT_FALSE(failed.failureReason.empty());

    reports.assign(4U, FleetMissionReport{
        FlightMode::Hovering, FleetMissionLeg::Regrouping,
        FleetDroneStatus::RegroupComplete});
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Regrouping, reports).progress,
        FleetMissionProgress::Advance);
    reports.back() = FleetMissionReport{
        FlightMode::Hovering, FleetMissionLeg::Regrouping,
        FleetDroneStatus::RegroupFailed};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Regrouping, reports).progress,
        FleetMissionProgress::Failed);
}

void Verify_FleetMissionProgressRequiresLandedReturnReports()
{
    using simulator::FleetMissionLeg;
    using simulator::detail::FleetMissionProgress;
    using simulator::detail::FleetMissionReport;
    using simulator::detail::FleetDroneStatus;
    using simulation::FlightMode;

    std::vector<std::optional<FleetMissionReport>> reports{
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Returning, FleetDroneStatus::ReturnComplete},
        FleetMissionReport{FlightMode::Hovering,
            FleetMissionLeg::Returning, FleetDroneStatus::ReturnComplete}};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Returning, reports).progress,
        FleetMissionProgress::Waiting);
    reports.back() = FleetMissionReport{
        FlightMode::Grounded, FleetMissionLeg::Returning,
        FleetDroneStatus::ReturnComplete};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Returning, reports).progress,
        FleetMissionProgress::Complete);

    reports = {
        FleetMissionReport{
            FlightMode::Grounded, FleetMissionLeg::Returning,
            FleetDroneStatus::LowBatteryReturnComplete},
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Reconnaissance, FleetDroneStatus::TargetReached}};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Failed);
    reports.back() = FleetMissionReport{
        FlightMode::Grounded, FleetMissionLeg::Returning,
        FleetDroneStatus::LowBatteryReturnComplete};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionLeg::Returning, reports).progress,
        FleetMissionProgress::Complete);
}

void Verify_SimulatorSession_FlightCommandsOverUdp()
{
    std::uint16_t simulatorPort = 0;
    {
        communication::UdpSocket availablePort(0);
        simulatorPort = availablePort.LocalPort();
    }

    communication::UdpSocket clientSocket(0);
    simulator::AeroSimLabApp app({
        .smokeTest = true,
        .udpPort = simulatorPort,
        .demoMode = false,
        .networkEnabled = true,
        .benchmarkMode = false,
        .groundControlCommandTimeoutTicks = 10
    });

    std::uint32_t sequence = 0;
    std::exception_ptr commandFailure;
    std::jthread groundControlThread([&]
    {
        const auto send = [&](const char* operation, const auto& setCommand,
            std::chrono::milliseconds timeout = std::chrono::seconds(5))
        {
            communication::CommandPacket packet;
            packet.commandId = ++sequence;
            packet.sequence = sequence;
            setCommand(packet);
            try
            {
                return ground_control::SendCommand(
                    "127.0.0.1", simulatorPort, packet, clientSocket, timeout).reply;
            }
            catch (const std::exception& error)
            {
                throw std::runtime_error(std::string{operation} + ": " + error.what());
            }
        };
        const auto require = [](bool condition, const char* message)
        {
            if (!condition)
                throw std::runtime_error(message);
        };

        try
        {
            // 초기 그래픽 준비가 끝나고 응답할 때까지 상태 조회를 기다린다.
            const auto beforeStatus = send("초기 상태 조회",
                [](communication::CommandPacket& packet)
            {
                packet.command = communication::FleetCommand::GetStatus;
            }, std::chrono::seconds(15));
            require(beforeStatus.accepted, "초기 상태 조회가 거부되었습니다.");
            require(beforeStatus.telemetry.drones.size() == 4U,
                "훈련장 시작 편대에 드론 네 대가 있어야 합니다.");
            const auto primaryDrone = std::find_if(
                beforeStatus.telemetry.drones.begin(),
                beforeStatus.telemetry.drones.end(),
                [&beforeStatus](const communication::DroneTelemetry& drone)
                {
                    return drone.id == beforeStatus.telemetry.primaryDroneId;
                });
            require(primaryDrone != beforeStatus.telemetry.drones.end(),
                "주 드론 식별값이 편대 목록과 일치해야 합니다.");
            require(beforeStatus.appliedTick == beforeStatus.telemetry.tick,
                "초기 상태 응답의 Tick 값이 서로 다릅니다.");

            const auto primaryTakeoff = send("주 드론 이륙",
                [](communication::CommandPacket& packet)
            {
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(primaryTakeoff.accepted && primaryTakeoff.status == "applied",
                "주 드론 이륙 명령을 적용하지 못했습니다.");

            const auto secondary = std::find_if(
                beforeStatus.telemetry.drones.begin(),
                beforeStatus.telemetry.drones.end(),
                [&beforeStatus](const communication::DroneTelemetry& drone)
            {
                return drone.id != beforeStatus.telemetry.primaryDroneId;
            });
            require(secondary != beforeStatus.telemetry.drones.end(),
                "주 드론 외에 시험할 드론이 없습니다.");
            const auto secondaryId = secondary->id;
            const auto secondaryTakeoff = send("보조 드론 이륙",
                [secondaryId](communication::CommandPacket& packet)
            {
                packet.droneId = secondaryId;
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(secondaryTakeoff.accepted,
                "선택한 보조 드론에 이륙 명령을 보내지 못했습니다.");
            const auto selectedDrone = std::find_if(
                secondaryTakeoff.telemetry.drones.begin(),
                secondaryTakeoff.telemetry.drones.end(),
                [secondaryId](const communication::DroneTelemetry& drone)
                {
                    return drone.id == secondaryId;
                });
            require(selectedDrone != secondaryTakeoff.telemetry.drones.end()
                    && selectedDrone->mode
                    == simulation::FlightMode::TakingOff,
                "선택한 드론이 이륙 상태로 바뀌지 않았습니다.");

            const auto landPrimary = send("주 드론 착륙",
                [](communication::CommandPacket& packet)
            {
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::Land};
            });
            require(landPrimary.accepted && landPrimary.status == "applied",
                "주 드론 착륙 명령을 적용하지 못했습니다.");
            const auto landSecondary = send("보조 드론 착륙",
                [secondaryId](communication::CommandPacket& packet)
            {
                packet.droneId = secondaryId;
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::Land};
            });
            require(landSecondary.accepted && landSecondary.status == "applied",
                "보조 드론 착륙 명령을 적용하지 못했습니다.");

            // 스모크 앱은 시작된 모든 드론이 지상에 돌아오면 스스로 종료한다.
            // 종료 직전 상태 조회를 보내면 앱이 응답 전에 닫힐 수 있어 여기서 기다리지 않는다.
        }
        catch (...)
        {
            commandFailure = std::current_exception();
        }
    });

    int simulatorExitCode = -1;
    std::exception_ptr simulatorFailure;
    try
    {
        // GLFW 창과 이벤트는 GLFW가 요구하는 메인 스레드에서만 다룬다.
        simulatorExitCode = app.Run();
    }
    catch (...)
    {
        simulatorFailure = std::current_exception();
    }

    groundControlThread.join();
    EXPECT_EQ(simulatorExitCode, 0)
        << "두 드론의 착륙이 끝나면 숨김 시뮬레이터가 정상 종료되어야 합니다.";

    const auto reportFailure = [](const char* context, std::exception_ptr failure)
    {
        if (!failure)
            return;
        try
        {
            std::rethrow_exception(failure);
        }
        catch (const std::exception& error)
        {
            ADD_FAILURE() << context << error.what();
        }
    };
    reportFailure("시뮬레이터 실행 실패: ", simulatorFailure);
    reportFailure("UDP 비행 명령 실패: ", commandFailure);
}

TEST(SimulatorSessionTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyGroundControlShowsOnlyDeliveredPeerPositions());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyGroundControlReceivesEachDroneDirectly());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyObserverCameraKeepsAUsableFollowDistance());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyObserverCameraCanFollowEachDrone());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyFormationTransitSafelyLandsWhenFollowerTelemetryIsMissing());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyFormationResultDoesNotDependOnStatusText());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyCompleteFleetMissionFinishesAndLands());
    EXPECT_NO_FATAL_FAILURE(Verify_PeerTelemetryPreservesStateAndRejectsInvalidValues());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressWaitsForEveryCurrentReport());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressRequiresLandedReturnReports());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorSession_FlightCommandsOverUdp());
}

TEST(SimulatorSessionTests, FleetDroneReturnsWhenBatteryReachesReserve)
{
    simulator::AeroSimLabAppTestAccess::VerifyFleetDroneReturnsWhenBatteryReachesReserve();
}

TEST(SimulatorSessionTests, ReconnaissanceKeepsTheFirstWaypoint)
{
    simulator::AeroSimLabAppTestAccess::VerifyReconnaissanceKeepsTheFirstWaypoint();
}

TEST(SimulatorSessionTests, CompleteMissionRequiresFourDrones)
{
    simulator::AeroSimLabAppTestAccess::VerifyCompleteMissionRequiresFourDrones();
}
