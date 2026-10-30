#include "DroneSimApp.h"
#include "gui/SimulatorView.h"
#include "FleetMissionProgress.h"
#include "FleetTelemetry.h"

#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "flight_planning/RoutePlanner.h"
#include "terrain/TerrainGrid.h"

#include "graphics/Camera.h"

#include <glm/glm.hpp>
#include <glm/geometric.hpp>
#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace dronesim;

namespace dronesim::simulator
{
class DroneSimAppTestAccess
{
public:
    static void VerifyCompleteMissionRequiresFourDrones()
    {
        DroneSimApp app({.smokeTest = true});
        app.InitializeGraphics();
        ASSERT_EQ(app.simulation_.DroneIds().size(), 4U);
        ASSERT_FALSE(app.trainingRangeScene_->BuildingObstacles().empty());
        const glm::dvec2 mapGroundSize =
            app.trainingRangeScene_->GroundSizeMeters();
        EXPECT_NEAR(app.terrainGrid_.HalfWidthMeters() * 2.0,
            mapGroundSize.x, 0.02);
        EXPECT_NEAR(app.terrainGrid_.HalfDepthMeters() * 2.0,
            mapGroundSize.y, 0.02);
        for (const auto droneId : app.simulation_.DroneIds())
        {
            const auto& state = app.simulation_.State(droneId);
            EXPECT_EQ(state.mode, simulation::FlightMode::Grounded);
            const glm::dvec2 groundPosition{
                state.positionMeters.x, state.positionMeters.y};
            EXPECT_TRUE(app.terrainGrid_.ContainsLocalPositionMeters(groundPosition));
            EXPECT_NEAR(state.positionMeters.z,
                app.terrainGrid_.HeightAtLocalMeters(groundPosition)
                    + simulation::QuadrotorFlightController::groundedModelOriginHeightMeters,
                0.02);
            EXPECT_NEAR(state.positionMeters.z,
                simulation::QuadrotorFlightController::groundedModelOriginHeightMeters,
                0.02);
        }
        EXPECT_TRUE(app.CompleteFleetMissionCanStart());

        app.fleetReconnaissance_[app.simulation_.PrimaryDroneId()].status =
            mission::FleetDroneStatus::Reconnaissance;
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

    static void VerifyFallingDroneLandsOnLoadedMapFloor()
    {
        DroneSimApp app({.smokeTest = true});
        app.InitializeGraphics();
        const glm::dvec2 openGround =
            app.trainingRangeScene_->FormationStagingPositionMeters();
        const auto fallingDrone = app.simulation_.AddDrone(
            {openGround, 8.0}, 0.0, 0.0);

        for (int tick = 0; tick < 180; ++tick)
            app.simulation_.AdvanceOneTick();

        const auto& landed = app.simulation_.State(fallingDrone);
        EXPECT_GT(landed.positionMeters.z, 0.2);
        EXPECT_LT(landed.positionMeters.z, 0.8);
        EXPECT_LT(std::abs(landed.velocityMetersPerSecond.z), 0.5);
    }

    static void VerifyReconnaissanceKeepsTheFirstWaypoint()
    {
        DroneSimApp app({.smokeTest = true});
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

    static void UseConnectedNetworkForTest(DroneSimApp& app)
    {
        communication::VirtualMeshSettings settings;
        settings.maximumLinkRangeMeters = 1'000.0;
        app.formationNetwork_ = std::make_unique<
            communication::VirtualMeshNetwork>(app.droneSocketTransport_, settings);
        app.formationRadioObstacles_.clear();
        app.formationTransit_.lastNetworkNodeUpdateTick.reset();
    }

    static void VerifyObserverCameraKeepsAUsableFollowDistance()
    {
        DroneSimApp app({.smokeTest = true});
        app.InitializeGraphics();
        app.view_->UpdateObserverCamera(app.simulation_, app.observerDroneId_,
            app.trainingRangeScene_->BuildingObstacles(), 1280, 720);

        const auto distance = glm::distance(
            app.view_->observerCamera_->GetPosition(),
            app.view_->observerCamera_->GetTarget());
        EXPECT_GT(distance, 5.0F);
    }

    static void VerifyObserverCameraCanFollowEachDrone()
    {
        DroneSimApp app({.smokeTest = true});
        app.InitializeGraphics();
        const auto droneIds = app.simulation_.DroneIds();
        ASSERT_EQ(droneIds.size(), 4U);

        app.view_->UpdateObserverCamera(app.simulation_, app.observerDroneId_,
            app.trainingRangeScene_->BuildingObstacles(), 1280, 720);
        const glm::vec3 primaryTarget = app.view_->observerCamera_->GetTarget();

        app.observerDroneId_ = droneIds[2U];
        app.view_->UpdateObserverCamera(app.simulation_, app.observerDroneId_,
            app.trainingRangeScene_->BuildingObstacles(), 1280, 720);
        const glm::vec3 selectedTarget = app.view_->observerCamera_->GetTarget();
        EXPECT_NE(primaryTarget, selectedTarget);
        EXPECT_GT(glm::distance(
            app.view_->observerCamera_->GetPosition(),
            app.view_->observerCamera_->GetTarget()), 5.0F);
    }

    static void VerifyEveryDroneHasAnUdpSocket()
    {
        DroneSimApp app({.smokeTest = true});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));
        std::vector<communication::SocketPort> ports;
        for (const auto droneId : app.simulation_.DroneIds())
        {
            const auto port = app.droneSocketTransport_.PortFor(droneId);
            ASSERT_TRUE(port.has_value());
            EXPECT_NE(*port, 0U);
            ports.push_back(*port);
        }
        std::sort(ports.begin(), ports.end());
        EXPECT_EQ(std::adjacent_find(ports.begin(), ports.end()), ports.end());
    }
    static void VerifyFormationTransitSafelyLandsWhenFollowerTelemetryIsMissing()
    {
        DroneSimApp app({.smokeTest = true});
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

        app.formationTransit_.phase = mission::FormationTransitState::Phase::InProgress;
        app.formationTransit_.route.leaderStartMeters =
            app.simulation_.State(droneIds.front()).positionMeters;
        app.formationTransit_.route.leaderWaypointsMeters.push_back(
            app.formationTransit_.route.leaderStartMeters + glm::dvec3{10.0, 0.0, 0.0});
        app.completeFleetMission_.stage =
            mission::CompleteFleetMissionState::Stage::FormationTransit;

        app.AdvanceFormationTransit();

        EXPECT_EQ(app.formationTransit_.phase,
            mission::FormationTransitState::Phase::Failed);
        EXPECT_EQ(app.completeFleetMission_.stage,
            mission::CompleteFleetMissionState::Stage::Failed);
        EXPECT_EQ(app.completeFleetMission_.status,
            "추종 드론 상태 보고가 끊겨 편대를 안전 착륙시킵니다.");
        for (const auto droneId : droneIds)
            EXPECT_EQ(app.simulation_.State(droneId).mode,
                simulation::FlightMode::Landing);
    }

    static void VerifyFormationResultDoesNotDependOnStatusText()
    {
        using FormationPhase = mission::FormationTransitState::Phase;
        using MissionStage = mission::CompleteFleetMissionState::Stage;
        DroneSimApp app({.smokeTest = true});
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
                mission::PeerTelemetry report;
                report.positionMeters = state.positionMeters;
                report.mode = state.mode;
                ASSERT_TRUE(app.peerTelemetryAtLeader_.Record(droneId, report,
                    app.simulation_.TickCount(), 1U));
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
        using Stage = mission::CompleteFleetMissionState::Stage;
        DroneSimApp app({.smokeTest = true});
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
                    + std::string(mission::FleetDroneStatusLabel(
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
                << "), 임무 단계 "
                << static_cast<int>(app.completeFleetMission_.stage);
            for (const auto droneId : droneIds)
            {
                const auto& drone = app.simulation_.State(droneId);
                failureContext << "; 드론 " << droneId << " 위치 ("
                    << drone.positionMeters.x << ", " << drone.positionMeters.y
                    << ", " << drone.positionMeters.z << ")";
                if (droneId != app.fleetLeaderId_)
                {
                    const auto* report = app.peerTelemetryAtLeader_.Latest(droneId);
                    failureContext << ", 리더와 거리 "
                        << glm::distance(drone.positionMeters,
                            leader.positionMeters) << "m, 보고 경과 ";
                    if (report == nullptr)
                        failureContext << "없음";
                    else
                        failureContext << app.simulation_.TickCount()
                            - report->receivedAtTick << " Tick";
                }
                const auto mission = app.fleetReconnaissance_.find(droneId);
                if (mission != app.fleetReconnaissance_.end())
                    failureContext << ", 목표 " << mission->second.targetIndex
                        << ", 상태 " << mission::FleetDroneStatusLabel(
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
        DroneSimApp app({
            .smokeTest = true});
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
                    == mission::FleetDroneStatus::LowBatteryReturnComplete)
                break;
        }

        const auto& mission = app.fleetReconnaissance_.at(lowBatteryDroneId);
        EXPECT_TRUE(app.simulation_.Battery(lowBatteryDroneId)
            .IsReturningForLowBattery());
        EXPECT_TRUE(mission.lowBatteryReturn);
        EXPECT_EQ(mission.leg, mission::FleetMissionLeg::Returning);
        EXPECT_EQ(mission.status,
            mission::FleetDroneStatus::LowBatteryReturnComplete);
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
    using mission::FleetDroneStatus;
    const std::string payload = mission::EncodePeerTelemetry(
        positionMeters, simulation::FlightMode::Hovering, 2U,
        FleetDroneStatus::ReturnLanding,
        17U);
    const auto telemetry = mission::DecodePeerTelemetry(payload);
    ASSERT_TRUE(telemetry.has_value());
    EXPECT_EQ(telemetry->positionMeters, positionMeters);
    EXPECT_EQ(telemetry->mode, simulation::FlightMode::Hovering);
    EXPECT_EQ(telemetry->missionLeg, 2U);
    EXPECT_EQ(telemetry->missionStatus, FleetDroneStatus::ReturnLanding);
    EXPECT_EQ(telemetry->assignmentVersion, 17U);
    EXPECT_FALSE(mission::DecodePeerTelemetry(payload + " 2400").has_value());

    mission::FleetTelemetryStore receivedStates;
    EXPECT_TRUE(receivedStates.Record(4U, *telemetry, 100U, 8U));
    EXPECT_FALSE(receivedStates.Record(4U, *telemetry, 101U, 7U));
    ASSERT_NE(receivedStates.Fresh(4U, 130U), nullptr);
    EXPECT_EQ(receivedStates.Fresh(4U, 130U)->state.positionMeters, positionMeters);
    EXPECT_EQ(receivedStates.Fresh(4U, 131U), nullptr);
    EXPECT_NE(receivedStates.Latest(4U), nullptr);
    receivedStates.Clear();
    EXPECT_EQ(receivedStates.Latest(4U), nullptr);

    EXPECT_FALSE(mission::DecodePeerTelemetry(
        mission::EncodePeerTelemetry(positionMeters,
            simulation::FlightMode::Hovering, 3U, FleetDroneStatus::Other,
            17U)).has_value());
    EXPECT_FALSE(mission::DecodePeerTelemetry(
        mission::EncodePeerTelemetry(positionMeters,
            simulation::FlightMode::Hovering, 2U,
            static_cast<FleetDroneStatus>(255U), 17U)).has_value());
}

void Verify_FleetMissionProgressWaitsForEveryCurrentReport()
{
    using mission::FleetMissionLeg;
    using mission::FleetMissionProgress;
    using mission::FleetMissionReport;
    using mission::FleetDroneStatus;
    using simulation::FlightMode;

    EXPECT_TRUE(mission::IsFleetMissionInProgress(
        FleetDroneStatus::Reconnaissance));
    EXPECT_EQ(mission::FleetDroneStatusLabel(
        FleetDroneStatus::TargetReached), "정찰 위치 도착");
    EXPECT_FALSE(mission::IsFleetMissionInProgress(
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
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Advance);

    reports.back().reset();
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Waiting);
    reports.back() = FleetMissionReport{
        FlightMode::Grounded, FleetMissionLeg::Regrouping,
        FleetDroneStatus::RegroupComplete};
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Waiting);

    reports.back() = FleetMissionReport{FlightMode::Grounded,
        FleetMissionLeg::Reconnaissance, FleetDroneStatus::RouteFailed};
    const auto failed = mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports);
    EXPECT_EQ(failed.progress, FleetMissionProgress::Failed);
    EXPECT_FALSE(failed.failureReason.empty());

    reports.assign(4U, FleetMissionReport{
        FlightMode::Hovering, FleetMissionLeg::Regrouping,
        FleetDroneStatus::RegroupComplete});
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Regrouping, reports).progress,
        FleetMissionProgress::Advance);
    reports.back() = FleetMissionReport{
        FlightMode::Hovering, FleetMissionLeg::Regrouping,
        FleetDroneStatus::RegroupFailed};
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Regrouping, reports).progress,
        FleetMissionProgress::Failed);
}

void Verify_FleetMissionProgressRequiresLandedReturnReports()
{
    using mission::FleetMissionLeg;
    using mission::FleetMissionProgress;
    using mission::FleetMissionReport;
    using mission::FleetDroneStatus;
    using simulation::FlightMode;

    std::vector<std::optional<FleetMissionReport>> reports{
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Returning, FleetDroneStatus::ReturnComplete},
        FleetMissionReport{FlightMode::Hovering,
            FleetMissionLeg::Returning, FleetDroneStatus::ReturnComplete}};
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Returning, reports).progress,
        FleetMissionProgress::Waiting);
    reports.back() = FleetMissionReport{
        FlightMode::Grounded, FleetMissionLeg::Returning,
        FleetDroneStatus::ReturnComplete};
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Returning, reports).progress,
        FleetMissionProgress::Complete);

    reports = {
        FleetMissionReport{
            FlightMode::Grounded, FleetMissionLeg::Returning,
            FleetDroneStatus::LowBatteryReturnComplete},
        FleetMissionReport{FlightMode::Grounded,
            FleetMissionLeg::Reconnaissance, FleetDroneStatus::TargetReached}};
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Reconnaissance, reports).progress,
        FleetMissionProgress::Failed);
    reports.back() = FleetMissionReport{
        FlightMode::Grounded, FleetMissionLeg::Returning,
        FleetDroneStatus::LowBatteryReturnComplete};
    EXPECT_EQ(mission::EvaluateFleetMissionProgress(
        FleetMissionLeg::Returning, reports).progress,
        FleetMissionProgress::Complete);
}

TEST(SimulatorSessionTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
    simulator::DroneSimAppTestAccess::VerifyEveryDroneHasAnUdpSocket());
    EXPECT_NO_FATAL_FAILURE(
        simulator::DroneSimAppTestAccess::VerifyObserverCameraKeepsAUsableFollowDistance());
    EXPECT_NO_FATAL_FAILURE(
        simulator::DroneSimAppTestAccess::VerifyObserverCameraCanFollowEachDrone());
    EXPECT_NO_FATAL_FAILURE(
        simulator::DroneSimAppTestAccess::VerifyFormationTransitSafelyLandsWhenFollowerTelemetryIsMissing());
    EXPECT_NO_FATAL_FAILURE(
        simulator::DroneSimAppTestAccess::VerifyFormationResultDoesNotDependOnStatusText());
    EXPECT_NO_FATAL_FAILURE(
        simulator::DroneSimAppTestAccess::VerifyCompleteFleetMissionFinishesAndLands());
    EXPECT_NO_FATAL_FAILURE(Verify_PeerTelemetryPreservesStateAndRejectsInvalidValues());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressWaitsForEveryCurrentReport());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressRequiresLandedReturnReports());
}

TEST(SimulatorSessionTests, FleetDroneReturnsWhenBatteryReachesReserve)
{
    simulator::DroneSimAppTestAccess::VerifyFleetDroneReturnsWhenBatteryReachesReserve();
}

TEST(SimulatorSessionTests, ReconnaissanceKeepsTheFirstWaypoint)
{
    simulator::DroneSimAppTestAccess::VerifyReconnaissanceKeepsTheFirstWaypoint();
}

TEST(SimulatorSessionTests, CompleteMissionRequiresFourDrones)
{
    simulator::DroneSimAppTestAccess::VerifyCompleteMissionRequiresFourDrones();
}

TEST(SimulatorSessionTests, FallingDroneLandsOnLoadedMapFloor)
{
    simulator::DroneSimAppTestAccess::VerifyFallingDroneLandsOnLoadedMapFloor();
}
