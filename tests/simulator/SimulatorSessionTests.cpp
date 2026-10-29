#include "AeroSimLabApp.h"
#include "FleetMissionProgress.h"
#include "EoVideoRecorder.h"
#include "FleetTelemetry.h"
#include "FleetDroneSpawn.h"

#include "simulation_core/SimulationCore.h"
#include "terrain/MapBuildings.h"
#include "terrain/MapFlightRoute.h"
#include "terrain/TerrainGrid.h"

#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"
#include "graphics/Camera.h"

#include <glm/glm.hpp>
#include <glm/geometric.hpp>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <opencv2/videoio.hpp>

#include <array>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
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
        const auto expectedRoute = terrain::PlanMapFlightRoute(
            app.terrainGrid_, app.routeBuildings_,
            app.simulation_.State(droneId).positionMeters,
            app.inspectionTargets_.front().approachPositionMeters,
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

    static void VerifyGroundControlShowsOnlyDeliveredPeerPositions()
    {
        AeroSimLabApp app({
            .smokeTest = true,
            .simulationDynamics = simulation::SimulationDynamics::Kinematic,
            .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));
        UseConnectedNetworkForTest(app);
        app.formationTransit_.active = true;

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        const auto disconnectedDroneId = droneIds[2U];
        ASSERT_EQ(app.simulation_.Apply(disconnectedDroneId,
            {simulation::DroneCommand::Kind::TakeOff}),
            simulation::CommandResult::Accepted);
        for (int tick = 0; tick < 240; ++tick)
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
        const auto groundControlNodeId = std::numeric_limits<
            communication::NetworkNodeId>::max();
        for (const auto droneId : droneIds)
            app.formationNetwork_->SetLinkDisabled(
                droneId, groundControlNodeId, true);

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
            app.formationNetwork_->SetLinkDisabled(
                droneId, groundControlNodeId, false);
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
            .simulationDynamics = simulation::SimulationDynamics::Kinematic,
            .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));
        UseConnectedNetworkForTest(app);
        app.formationTransit_.active = true;

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        const auto groundControlNodeId = std::numeric_limits<
            communication::NetworkNodeId>::max();
        app.formationNetwork_->SetLinkDisabled(
            droneIds.front(), groundControlNodeId, true);
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

        app.formationTransit_.active = true;
        app.formationTransit_.route.leaderStartMeters =
            app.simulation_.State(droneIds.front()).positionMeters;
        app.formationTransit_.route.leaderWaypointsMeters.push_back(
            app.formationTransit_.route.leaderStartMeters + glm::dvec3{10.0, 0.0, 0.0});
        app.completeFleetMission_.stage =
            AeroSimLabApp::CompleteFleetMissionState::Stage::FormationTransit;

        app.AdvanceFormationTransit();

        EXPECT_FALSE(app.formationTransit_.active);
        EXPECT_EQ(app.completeFleetMission_.stage,
            AeroSimLabApp::CompleteFleetMissionState::Stage::Failed);
        EXPECT_EQ(app.completeFleetMission_.status,
            "추종 드론 상태 보고가 끊겨 편대를 안전 착륙시킵니다.");
        for (const auto droneId : droneIds)
            EXPECT_EQ(app.simulation_.State(droneId).mode,
                simulation::FlightMode::Landing);
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
                    + mission.status);
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
                        << ", 상태 " << mission->second.status;
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
            .simulationDynamics = simulation::SimulationDynamics::Kinematic,
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
                && mission.status == "저전력 복귀 완료")
                break;
        }

        const auto& mission = app.fleetReconnaissance_.at(lowBatteryDroneId);
        EXPECT_TRUE(app.simulation_.Battery(lowBatteryDroneId)
            .IsReturningForLowBattery());
        EXPECT_TRUE(mission.lowBatteryReturn);
        EXPECT_EQ(mission.leg, simulator::FleetMissionLeg::Returning);
        EXPECT_EQ(mission.status, "저전력 복귀 완료");
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

void Verify_PeerTelemetryPreservesMissionStateAndRejectsInvalidLeg()
{
    const glm::dvec3 positionMeters{12.5, -7.25, 31.0};
    const std::string payload = simulator::detail::EncodePeerTelemetry(
        positionMeters, simulation::FlightMode::Hovering, 2U, "복귀 착륙 중",
        17U, 2400U);
    const auto telemetry = simulator::detail::DecodePeerTelemetry(payload);
    ASSERT_TRUE(telemetry.has_value());
    EXPECT_EQ(telemetry->positionMeters, positionMeters);
    EXPECT_EQ(telemetry->mode, simulation::FlightMode::Hovering);
    EXPECT_EQ(telemetry->missionLeg, 2U);
    EXPECT_EQ(telemetry->missionStatus, "복귀 착륙 중");
    EXPECT_EQ(telemetry->assignmentVersion, 17U);
    EXPECT_EQ(telemetry->assignmentLeaseExpiresAtTick, 2400U);

    EXPECT_FALSE(simulator::detail::DecodePeerTelemetry(
        simulator::detail::EncodePeerTelemetry(positionMeters,
            simulation::FlightMode::Hovering, 3U, "잘못된 단계", 17U, 2400U))
        .has_value());
}

void Verify_FleetMissionProgressWaitsForEveryCurrentReport()
{
    using simulator::FleetMissionLeg;
    using simulator::detail::FleetMissionProgress;
    using simulator::detail::FleetMissionReport;
    using simulator::detail::FleetDroneStatus;
    using simulation::FlightMode;

    EXPECT_TRUE(simulator::detail::IsFleetMissionInProgress("정찰 이동 중"));
    EXPECT_TRUE(simulator::detail::IsFleetMissionInProgress("저전력 복귀 중"));
    EXPECT_FALSE(simulator::detail::IsFleetMissionInProgress("정찰 위치 도착"));
    EXPECT_FALSE(simulator::detail::IsFleetMissionInProgress("전체 임무 완료"));
    EXPECT_EQ(simulator::detail::ParseFleetDroneStatus("경로 오류"),
        FleetDroneStatus::RouteFailed);

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

void Verify_EoVideoRecorderStoresH264AndFrameMetadata()
{
    const auto outputDirectory = std::filesystem::temp_directory_path()
        / "dronesim-video-recorder-test";
    std::filesystem::remove_all(outputDirectory);
    const auto videoPath = outputDirectory / "inspection.mp4";
    simulator::EoVideoRecorder recorder;
    std::string error;
    const simulation::CameraModel cameraModel{};
    ASSERT_TRUE(recorder.Start(videoPath, cameraModel.imageWidthPixels,
        cameraModel.imageHeightPixels, 60.0, "osm-building-1", "facade-test-area",
        error)) << error;
    recorder.SetMissionOutcome(simulator::EoVideoRecorder::MissionOutcome::Completed);

    sensor::RgbFrame frame;
    frame.width = cameraModel.imageWidthPixels;
    frame.height = cameraModel.imageHeightPixels;
    frame.rowOrder = sensor::RgbFrameRowOrder::BottomToTop;
    frame.pixels.resize(static_cast<std::size_t>(frame.width * frame.height * 3), 128U);
    simulation::CameraState camera;
    camera.positionMeters = {1.0, 2.0, 3.0};
    for (std::uint64_t tick = 1; tick <= 4; ++tick)
    {
        frame.simulationTick = tick;
        ASSERT_TRUE(recorder.WriteFrame(frame, camera,
            static_cast<double>(tick) / 60.0,
            tick == 2 ? simulation::CaptureAssessment::Occluded
                : simulation::CaptureAssessment::Ready,
            error)) << error;
    }
    std::fill(frame.pixels.begin(), frame.pixels.end(), 0U);
    auto metadataPath = videoPath;
    metadataPath.replace_extension(".jsonl");
    EXPECT_FALSE(std::filesystem::exists(metadataPath));
    auto partialMetadataPath = metadataPath;
    partialMetadataPath += ".partial";
    EXPECT_TRUE(std::filesystem::exists(partialMetadataPath));

    std::uint64_t savedFrameCount = 0;
    ASSERT_TRUE(recorder.Stop(savedFrameCount, error)) << error;
    EXPECT_EQ(savedFrameCount, 4U);
    EXPECT_TRUE(std::filesystem::exists(metadataPath));
    EXPECT_FALSE(std::filesystem::exists(partialMetadataPath));
    ASSERT_TRUE(std::filesystem::exists(videoPath));
    EXPECT_GT(std::filesystem::file_size(videoPath), 0U);
    cv::VideoCapture decodedVideo(videoPath.string());
    ASSERT_TRUE(decodedVideo.isOpened());
    cv::Mat decodedFrame;
    ASSERT_TRUE(decodedVideo.read(decodedFrame));
    EXPECT_EQ(decodedFrame.cols, cameraModel.imageWidthPixels);
    EXPECT_EQ(decodedFrame.rows, cameraModel.imageHeightPixels);
    EXPECT_GT(cv::mean(decodedFrame)[0], 100.0);
    decodedVideo.release();

    std::ifstream metadata(metadataPath);
    ASSERT_TRUE(metadata.good());
    const std::string contents{
        std::istreambuf_iterator<char>{metadata}, std::istreambuf_iterator<char>{}};
    EXPECT_EQ(std::count(contents.begin(), contents.end(), '\n'), 4);
    EXPECT_NE(contents.find("\"capture_state\":\"CAPTURED\""), std::string::npos);
    EXPECT_NE(contents.find("\"facility_id\":\"osm-building-1\""), std::string::npos);
    EXPECT_NE(contents.find("\"inspection_zone_id\":\"facade-test-area\""),
        std::string::npos);
    EXPECT_NE(contents.find("\"capture_assessment\":\"occluded\""),
        std::string::npos);
    EXPECT_NE(contents.find("\"simulation_tick\":4"), std::string::npos);
    metadata.close();

    auto manifestPath = videoPath;
    manifestPath.replace_extension(".manifest.json");
    std::ifstream manifest(manifestPath);
    ASSERT_TRUE(manifest.good());
    const std::string manifestText{
        std::istreambuf_iterator<char>{manifest}, std::istreambuf_iterator<char>{}};
    EXPECT_NE(manifestText.find("\"collection_state\": \"SAVED\""),
        std::string::npos);
    EXPECT_NE(manifestText.find("\"video_bytes\""), std::string::npos);
    manifest.close();
    const auto manifestData = nlohmann::json::parse(manifestText);
    EXPECT_EQ(manifestData.at("checksum_algorithm").get<std::string>(),
        "CRC-32/ISO-HDLC");
    EXPECT_EQ(manifestData.at("accepted_frame_count").get<std::uint64_t>(), 4U);
    EXPECT_EQ(manifestData.at("unwritten_frame_count").get<std::uint64_t>(), 0U);
    EXPECT_EQ(manifestData.at("facility_id").get<std::string>(), "osm-building-1");
    EXPECT_EQ(manifestData.at("inspection_zone_id").get<std::string>(),
        "facade-test-area");
    EXPECT_EQ(manifestData.at("mission_outcome").get<std::string>(), "COMPLETED");
    EXPECT_EQ(manifestData.at("geometry_ready_frame_count").get<std::uint64_t>(), 3U);
    std::string calculatedChecksum;
    ASSERT_TRUE(simulator::EoVideoRecorder::CalculateFileChecksum(
        videoPath, calculatedChecksum, error)) << error;
    EXPECT_EQ(calculatedChecksum,
        manifestData.at("video_checksum").get<std::string>());
    ASSERT_TRUE(simulator::EoVideoRecorder::CalculateFileChecksum(
        metadataPath, calculatedChecksum, error)) << error;
    EXPECT_EQ(calculatedChecksum,
        manifestData.at("metadata_checksum").get<std::string>());
    ASSERT_TRUE(simulator::EoVideoRecorder::MarkCaptureDelivered(
        manifestPath, error)) << error;
    std::ifstream deliveredManifestFile(manifestPath);
    ASSERT_TRUE(deliveredManifestFile.good());
    const auto deliveredManifest = nlohmann::json::parse(
        std::istreambuf_iterator<char>{deliveredManifestFile},
        std::istreambuf_iterator<char>{});
    EXPECT_EQ(deliveredManifest.at("collection_state").get<std::string>(),
        "DELIVERED");
    deliveredManifestFile.close();

    const auto failedVideoPath = outputDirectory / "failed-inspection.mp4";
    simulator::EoVideoRecorder failedRecorder;
    ASSERT_TRUE(failedRecorder.Start(failedVideoPath,
        cameraModel.imageWidthPixels, cameraModel.imageHeightPixels, 60.0,
        "osm-building-1", "facade-test-area", error)) << error;
    failedRecorder.SetMissionOutcome(
        simulator::EoVideoRecorder::MissionOutcome::Failed,
        "촬영 조건을 만족하지 못했습니다.");
    frame.simulationTick = 5;
    ASSERT_TRUE(failedRecorder.WriteFrame(frame, camera, 5.0 / 60.0,
        simulation::CaptureAssessment::TooFar, error)) << error;
    ASSERT_TRUE(failedRecorder.Stop(savedFrameCount, error)) << error;
    auto failedManifestPath = failedVideoPath;
    failedManifestPath.replace_extension(".manifest.json");
    std::ifstream failedManifestFile(failedManifestPath);
    ASSERT_TRUE(failedManifestFile.good());
    const auto failedManifest = nlohmann::json::parse(
        std::istreambuf_iterator<char>{failedManifestFile},
        std::istreambuf_iterator<char>{});
    EXPECT_EQ(failedManifest.at("collection_state").get<std::string>(), "SAVED");
    EXPECT_EQ(failedManifest.at("mission_outcome").get<std::string>(), "FAILED");
    EXPECT_EQ(failedManifest.at("mission_failure_reason").get<std::string>(),
        "촬영 조건을 만족하지 못했습니다.");
    failedManifestFile.close();
    std::filesystem::remove_all(outputDirectory);
}

void Verify_FleetDroneSpawnRejectsAPositionOutsideTheTerrain()
{
    terrain::TerrainGrid terrainGrid;
    terrainGrid.sampleCount = 3;
    terrainGrid.spacingXMeters = 2.0F;
    terrainGrid.spacingYMeters = 2.0F;
    terrainGrid.heightOffsetsMeters.resize(9U);

    std::optional<simulator::detail::FleetDroneSpawnPose> spawn;
    EXPECT_NO_THROW(spawn = simulator::detail::FindFleetDroneSpawnPose(
        terrainGrid, glm::dvec3{0.0}, 0U));
    EXPECT_FALSE(spawn.has_value());
}

void Verify_SimulatorSession_RecordingReplayOverUdp()
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

            const auto start = send("기록 시작", [](communication::CommandPacket& packet)
            {
                packet.command = communication::SessionCommand::StartRecording;
            });
            require(start.accepted && start.status == "recording_started",
                "기록을 시작하지 못했습니다.");

            const auto takeoff = send("이륙", [](communication::CommandPacket& packet)
            {
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(takeoff.accepted && takeoff.status == "applied",
                "이륙 명령을 적용하지 못했습니다.");

            const auto land = send("착륙", [](communication::CommandPacket& packet)
            {
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::Land};
            });
            require(land.accepted && land.status == "applied",
                "착륙 명령을 적용하지 못했습니다.");

            const auto added = send("드론 추가", [](communication::CommandPacket& packet)
            {
                packet.command = communication::FleetCommand::AddDrone;
            });
            require(added.accepted && added.status == "drone_added",
                "다섯 번째 드론을 추가하지 못했습니다.");
            require(added.telemetry.drones.size() == 5U,
                "드론 추가 응답에 다섯 대가 있어야 합니다.");
            const communication::DroneTelemetry& addedDrone =
                added.telemetry.drones.back();
            const auto addedDroneId = addedDrone.id;
            require(addedDrone.name == "드론 5",
                "추가된 드론 이름이 예상과 다릅니다.");
            require(addedDrone.mode == simulation::FlightMode::Grounded,
                "추가된 드론이 지상 상태가 아닙니다.");
            require(addedDrone.positionMeters.x
                    > added.telemetry.drones[0].positionMeters.x,
                "추가된 드론이 기존 드론과 겹친 위치에 있습니다.");
            const auto unrecordedSecondaryTakeoff = send("기록 중 두 번째 드론 이륙",
                [&addedDroneId](communication::CommandPacket& packet)
            {
                packet.droneId = addedDroneId;
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(!unrecordedSecondaryTakeoff.accepted
                    && unrecordedSecondaryTakeoff.status == "recording_secondary_unsupported",
                "기록에 남지 않는 추가 드론 명령을 허용했습니다.");

            const auto stop = send("기록 종료", [](communication::CommandPacket& packet)
            {
                packet.command = communication::SessionCommand::StopRecording;
            });
            require(stop.accepted && stop.status == "recording_stopped",
                "기록을 종료하지 못했습니다.");

            const auto secondaryTakeoff = send("두 번째 드론 이륙",
                [&addedDroneId](communication::CommandPacket& packet)
            {
                packet.droneId = addedDroneId;
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(secondaryTakeoff.accepted,
                "선택한 추가 드론에 이륙 명령을 보내지 못했습니다.");
            const auto selectedDrone = std::find_if(
                secondaryTakeoff.telemetry.drones.begin(),
                secondaryTakeoff.telemetry.drones.end(),
                [&addedDroneId](const communication::DroneTelemetry& drone)
                {
                    return drone.id == addedDroneId;
                });
            require(selectedDrone != secondaryTakeoff.telemetry.drones.end()
                    && selectedDrone->mode
                    == simulation::FlightMode::TakingOff,
                "선택한 드론이 이륙 상태로 바뀌지 않았습니다.");
            require(secondaryTakeoff.telemetry.drone.mode
                    != simulation::FlightMode::TakingOff,
                "두 번째 드론 명령이 주 드론 상태를 바꿨습니다.");

            const auto replay = send("기록 재생", [](communication::CommandPacket& packet)
            {
                packet.command = communication::SessionCommand::ReplayRecording;
            });
            require(replay.accepted && replay.status == "recording_replay_started",
                "기록을 재생하지 못했습니다.");
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
        << "기록 재생이 끝나고 드론이 지상에 도달하면 숨김 시뮬레이터가 종료되어야 합니다.";

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
    reportFailure("UDP 기록·재생 명령 실패: ", commandFailure);
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
        simulator::AeroSimLabAppTestAccess::VerifyFormationTransitSafelyLandsWhenFollowerTelemetryIsMissing());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyCompleteFleetMissionFinishesAndLands());
    EXPECT_NO_FATAL_FAILURE(Verify_PeerTelemetryPreservesMissionStateAndRejectsInvalidLeg());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressWaitsForEveryCurrentReport());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressRequiresLandedReturnReports());
    EXPECT_NO_FATAL_FAILURE(Verify_EoVideoRecorderStoresH264AndFrameMetadata());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetDroneSpawnRejectsAPositionOutsideTheTerrain());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorSession_RecordingReplayOverUdp());
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
