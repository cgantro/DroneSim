#include "AeroSimLabApp.h"
#include "FleetMissionProgress.h"
#include "EoVideoRecorder.h"
#include "FleetTelemetry.h"
#include "FleetDroneSpawn.h"
#include "FaultScenarioTimeline.h"

#include "simulation_core/SimulationCore.h"
#include "terrain/MapBuildings.h"
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
    enum class RotorFailureScenario
    {
        Follower,
        LeaderWithoutGroundControlRoute,
        LeaderWithGroundControlRoute
    };

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
            app.formationNetwork_->SetDirectedLinkDisabled(
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
    }

    static void UseConnectedNetworkForTest(AeroSimLabApp& app)
    {
        // 실제 전파 범위와 가림 대신 연결을 보장해 승계 절차를 따로 확인한다.
        aerosimlab::communication::VirtualMeshSettings settings;
        settings.mode = app.fleetNetworkMode_;
        settings.leaderNodeId = app.fleetLeaderId_;
        settings.maximumLinkRangeMeters = 1'000.0;
        settings.helloIntervalTicks = 15U;
        settings.neighborTimeoutTicks = 90U;
        settings.routeLifetimeTicks = 300U;
        settings.maximumHops = 5U;
        settings.packetLossProbability =
            static_cast<double>(app.fleetPacketLossPercent_) / 100.0;
        settings.linkLatencyJitterTicks = app.fleetLatencyJitterTicks_;
        app.formationNetwork_ = std::make_unique<
            aerosimlab::communication::VirtualMeshNetwork>(settings);
        app.formationRadioObstacles_.clear();
        app.formationTransit_.lastNetworkNodeUpdateTick.reset();
    }

    static void VerifyGroundedRelayDroneTakesOffForAssignment()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        const auto relayDroneId = droneIds.back();
        auto& mission = app.fleetReconnaissance_[relayDroneId];
        mission.targetIndex = 0U;
        mission.leg = AeroSimLabApp::FleetMissionLeg::Reconnaissance;
        mission.status = "정찰 위치 도착";
        mission.routeReceived = true;

        const auto currentPosition = app.simulation_.State(relayDroneId).positionMeters;
        app.ApplyFleetRouteAssignment(relayDroneId,
            AeroSimLabApp::FleetMissionLeg::Relaying, 0U,
            {currentPosition + glm::dvec3{8.0, 0.0, 10.0}},
            1U, 1U, 1000U);

        EXPECT_EQ(app.simulation_.State(relayDroneId).mode,
            simulation::FlightMode::TakingOff);
        EXPECT_EQ(mission.status, "중계 준비");
        EXPECT_EQ(mission.leg, AeroSimLabApp::FleetMissionLeg::Relaying);
    }

    static void VerifyRelayRecoveryUsesTheConfiguredRadioRange()
    {
        AeroSimLabApp app({
            .smokeTest = true,
            .simulationDynamics = simulation::SimulationDynamics::RigidBody,
            .networkEnabled = false});
        app.InitializeGraphics();
        app.fleetNetworkMode_ = communication::VirtualMeshMode::AdaptiveMesh;
        ASSERT_TRUE(app.PrepareFleetNetwork(true));

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        app.routeBuildings_.clear();
        app.formationRadioObstacles_.clear();

        const auto setState = [&app](simulation::DroneId droneId,
            glm::dvec3 position, simulation::FlightMode mode)
        {
            simulation::DroneState state = app.simulation_.State(droneId);
            state.positionMeters = position;
            state.mode = mode;
            app.simulation_.SetInitialState(droneId, state);
        };
        const std::size_t disconnectedTargetIndex = 4U;
        const glm::dvec3 disconnectedTarget = app.inspectionTargets_.at(
            disconnectedTargetIndex).approachPositionMeters;
        const double leaderGround = app.terrainGrid_.HeightAtRenderedSurfaceMeters({0.0, 0.0});
        setState(droneIds[0], {0.0, 0.0, leaderGround + 10.0},
            simulation::FlightMode::Grounded);
        setState(droneIds[1], {90.0, 0.0, leaderGround + 0.5},
            simulation::FlightMode::Grounded);
        setState(droneIds[2], disconnectedTarget - glm::dvec3{8.0, 0.0, 0.0},
            simulation::FlightMode::FollowingWaypoint);
        setState(droneIds[3], {20.0, 0.0, leaderGround + 10.0},
            simulation::FlightMode::Grounded);

        for (std::size_t index = 0; index < droneIds.size(); ++index)
        {
            AeroSimLabApp::FleetReconnaissanceState mission;
            mission.targetIndex = index == 2U ? disconnectedTargetIndex : index;
            mission.leg = AeroSimLabApp::FleetMissionLeg::Reconnaissance;
            mission.status = index == 2U ? "정찰 이동 중" : "정찰 위치 도착";
            mission.routeReceived = true;
            if (index == 2U)
                mission.routeWaypointsMeters = {disconnectedTarget};
            app.fleetReconnaissance_.emplace(droneIds[index], std::move(mission));
        }

        for (std::uint64_t tick = 0U; tick < 200U; ++tick)
            app.simulation_.AdvanceOneTick();
        app.formationTransit_.lastNetworkNodeUpdateTick.reset();
        const auto leaderPosition = app.simulation_.State(droneIds[0]).positionMeters;
        const auto candidatePosition = app.simulation_.State(droneIds[1]).positionMeters;
        const auto disconnectedPosition = app.simulation_.State(droneIds[2]).positionMeters;
        app.formationNetwork_->UpdateNodes(
            std::array{
                communication::NetworkNodeSnapshot{droneIds[0], leaderPosition},
                communication::NetworkNodeSnapshot{droneIds[1], candidatePosition},
                communication::NetworkNodeSnapshot{droneIds[2], disconnectedPosition},
                communication::NetworkNodeSnapshot{droneIds[3],
                    app.simulation_.State(droneIds[3]).positionMeters}}, {});

        const auto makeTelemetry = [](glm::dvec3 position,
            simulation::FlightMode mode, std::string status)
        {
            return simulator::detail::PeerTelemetry{
                .positionMeters = position,
                .mode = mode,
                .missionLeg = 0U,
                .missionStatus = std::move(status),
                .assignmentVersion = 1U,
                .assignmentLeaseExpiresAtTick = 1000U};
        };
        app.peerTelemetryAtLeader_[droneIds[1]] = {
            makeTelemetry(candidatePosition, simulation::FlightMode::Grounded,
                "정찰 위치 도착"), app.simulation_.TickCount(), 1U};
        app.peerTelemetryAtLeader_[droneIds[2]] = {
            makeTelemetry(disconnectedPosition,
                simulation::FlightMode::FollowingWaypoint, "정찰 이동 중"),
            1U, 1U};
        EXPECT_TRUE(app.TryStartFleetRelayRecovery());
        const auto relayDroneId = app.fleetRelayRecovery_.relayDroneId;
        EXPECT_NE(relayDroneId, droneIds[0]);
        EXPECT_NE(relayDroneId, droneIds[2]);
        EXPECT_EQ(app.fleetRelayRecovery_.disconnectedDroneId, droneIds[2]);
        EXPECT_GT(app.fleetReconnaissance_.at(relayDroneId).pendingRouteSequence, 0U);
        EXPECT_FALSE(app.formationNetwork_->CanExchangeAtPositions(
            droneIds[0], droneIds[2], leaderPosition, disconnectedPosition));
        EXPECT_TRUE(app.formationNetwork_->CanExchangeAtPositions(
            droneIds[0], relayDroneId, leaderPosition,
            app.fleetRelayRecovery_.holdPositionMeters));
        EXPECT_TRUE(app.formationNetwork_->CanExchangeAtPositions(
            relayDroneId, droneIds[2], app.fleetRelayRecovery_.holdPositionMeters,
            disconnectedPosition));

        const auto recoveryStartedAtTick = app.fleetRelayRecovery_.startedAtTick;
        bool telemetryRecovered = false;
        bool telemetryRoutedViaRelay = false;
        constexpr std::uint64_t maximumRecoveryTicks = 5000U;
        for (std::uint64_t tick = 0U; tick < maximumRecoveryTicks; ++tick)
        {
            app.AdvanceSimulationTick();
            const auto* report = app.FreshPeerTelemetry(droneIds[2]);
            for (const auto& delivered : app.formationNetwork_->DeliveredMessages())
            {
                const auto& message = delivered.message;
                if (message.kind == "drone_telemetry"
                    && message.source == droneIds[2]
                    && message.destination == droneIds[0]
                    && std::find(message.route.begin(), message.route.end(), relayDroneId)
                        != message.route.end())
                    telemetryRoutedViaRelay = true;
            }
            telemetryRecovered = telemetryRecovered
                || (report != nullptr
                    && report->receivedAtTick > recoveryStartedAtTick
                    && telemetryRoutedViaRelay
                    && !app.fleetRelayRecovery_.waitingForTelemetry);
            if (telemetryRecovered)
                break;
        }
        EXPECT_TRUE(telemetryRecovered)
            << "시작 후 " << app.simulation_.TickCount() - recoveryStartedAtTick
            << " Tick, 정찰 텔레메트리 중계 경로=" << telemetryRoutedViaRelay
            << ", 위치 링크(리더-중계/중계-정찰)="
            << app.formationNetwork_->CanExchangeAtPositions(
                droneIds[0], relayDroneId,
                app.simulation_.State(droneIds[0]).positionMeters,
                app.simulation_.State(relayDroneId).positionMeters)
            << '/'
            << app.formationNetwork_->CanExchangeAtPositions(
                relayDroneId, droneIds[2],
                app.simulation_.State(relayDroneId).positionMeters,
                app.simulation_.State(droneIds[2]).positionMeters)
            << ", 무전 활성(중계/정찰)="
            << app.formationNetwork_->IsNodeRadioEnabled(relayDroneId)
            << '/'
            << app.formationNetwork_->IsNodeRadioEnabled(droneIds[2])
            << ", 경로 요청/응답="
            << app.formationNetwork_->Metrics().routeRequests << '/'
            << app.formationNetwork_->Metrics().routeReplies
            << ", 대기=" << app.formationNetwork_->Metrics().queuedMessages
            << ", 버림=" << app.formationNetwork_->Metrics().droppedMessages;
        EXPECT_TRUE(app.fleetReconnaissance_.at(droneIds[2]).waypointCommanded)
            << "통신이 끊긴 정찰 기체가 기존 경로를 실행하지 않았습니다.";
    }

    static void VerifyLeaderRadioLossPromotesCandidateAndReassignsTarget()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        for (std::size_t index = 0; index < droneIds.size(); ++index)
        {
            AeroSimLabApp::FleetReconnaissanceState mission;
            mission.targetIndex = index;
            mission.leg = AeroSimLabApp::FleetMissionLeg::Reconnaissance;
            mission.status = index == 0U ? "정찰 이동 중" : "정찰 위치 도착";
            if (index == 0U)
            {
                mission.assignmentVersion = 5U;
                mission.assignmentLeaseExpiresAtTick = 300U;
                app.nextMissionAssignmentVersion_ = 6U;
            }
            app.fleetReconnaissance_.emplace(droneIds[index], std::move(mission));
        }
        auto failedLeaderState = app.simulation_.State(droneIds.front());
        failedLeaderState.mode = simulation::FlightMode::Hovering;
        app.simulation_.SetInitialState(droneIds.front(), failedLeaderState);

        for (std::uint64_t tick = 0; tick < 60U; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }
        ASSERT_GT(app.lastLeaderHeartbeatTick_, 0U);

        app.formationNetwork_->SetNodeRadioEnabled(droneIds.front(), false);
        bool verifiedNoEarlyTransfer = false;
        for (std::uint64_t tick = 0; tick < 420U; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.AdvanceFleetReconnaissance();
            app.ReassignLostLeaderTarget();
            app.simulation_.AdvanceOneTick();

            if (!verifiedNoEarlyTransfer && app.fleetLeaderId_ != droneIds.front()
                && !app.leaderStateCollectionPending_)
            {
                verifiedNoEarlyTransfer = true;
                EXPECT_LT(app.simulation_.TickCount(), 300U);
                EXPECT_TRUE(app.pendingReassignedTargetIndices_.empty());
                EXPECT_EQ(app.simulation_.State(droneIds.front()).mode,
                    simulation::FlightMode::Hovering);
                for (std::size_t index = 1U; index < droneIds.size(); ++index)
                    EXPECT_NE(app.fleetReconnaissance_.at(droneIds[index]).targetIndex, 0U);
            }

            const bool targetWasReassigned = std::any_of(droneIds.begin() + 1,
                droneIds.end(), [&app](simulation::DroneId droneId)
                {
                    return app.fleetReconnaissance_.at(droneId).targetIndex == 0U;
                });
            if (targetWasReassigned)
                break;
        }

        EXPECT_TRUE(verifiedNoEarlyTransfer);
        EXPECT_EQ(app.fleetLeaderId_, droneIds[1]);
        EXPECT_EQ(app.fleetDroneIds_.front(), droneIds[1]);
        EXPECT_EQ(app.excludedFleetDroneIds_,
            (std::vector<simulation::DroneId>{droneIds.front()}));
        EXPECT_FALSE(app.leaderStateCollectionPending_);
        EXPECT_TRUE(app.pendingReassignedTargetIndices_.empty());
        EXPECT_TRUE(app.fleetLeaderId_ == droneIds[1]);
        EXPECT_EQ(app.fleetReconnaissance_.at(droneIds.front()).status,
            "임무 권한 만료 · 재배정 대기");
        EXPECT_TRUE(app.fleetReconnaissance_.at(droneIds[1]).targetIndex == 0U
            || app.fleetReconnaissance_.at(droneIds[2]).targetIndex == 0U
            || app.fleetReconnaissance_.at(droneIds[3]).targetIndex == 0U);
    }

    static void VerifyLeaderLossWithoutCandidateSafelyLandsFleet()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        for (std::size_t index = 0; index < droneIds.size(); ++index)
        {
            AeroSimLabApp::FleetReconnaissanceState mission;
            mission.targetIndex = index;
            mission.leg = AeroSimLabApp::FleetMissionLeg::Reconnaissance;
            mission.status = "정찰 이동 중";
            app.fleetReconnaissance_.emplace(droneIds[index], std::move(mission));

            EXPECT_EQ(app.simulation_.Apply(droneIds[index],
                {simulation::DroneCommand::Kind::TakeOff}),
                simulation::CommandResult::Accepted);
        }

        for (std::uint64_t tick = 0; tick < 120U; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
        }
        for (const auto droneId : droneIds)
            ASSERT_NE(app.simulation_.State(droneId).mode,
                simulation::FlightMode::Grounded);

        for (const auto droneId : droneIds)
            app.formationNetwork_->SetNodeRadioEnabled(droneId, false);

        for (std::uint64_t tick = 0; tick < 300U; ++tick)
        {
            app.AdvanceFleetNetwork();
            app.simulation_.AdvanceOneTick();
            if (app.fleetLeadershipStatus_.find("안전 착륙") != std::string::npos)
                break;
        }

        EXPECT_EQ(app.fleetLeadershipStatus_,
            "통신 가능한 새 리더를 찾지 못해 안전 착륙합니다");
        EXPECT_FALSE(app.leaderReplacementPending_);
        EXPECT_FALSE(app.leaderStateCollectionPending_);
        for (const auto droneId : droneIds)
        {
            const auto mode = app.simulation_.State(droneId).mode;
            EXPECT_TRUE(mode == simulation::FlightMode::Landing
                || mode == simulation::FlightMode::Grounded)
                << "드론 " << droneId << "이 안전 착륙 상태가 아닙니다.";
        }
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

        ASSERT_TRUE(app.StartCompleteFleetMission(
            simulation::FormationShape::Vee)) << app.formationTransit_.status;
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
                << ", 중계 " << metrics.forwardedMessages << ", 경로 탐색 "
                << metrics.routeRequests << ", 경로 오류 " << metrics.routeErrors
                << ", 폐기 " << metrics.droppedMessages;
            const auto leaderHop = app.formationNetwork_->NextHop(
                droneIds.back(), app.fleetLeaderId_);
            failureContext << "; 리더 경로 다음 홉 "
                << (leaderHop ? std::to_string(*leaderHop) : "없음");
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

    static void VerifyCompleteFleetMissionReassignsAfterRotorFailure(
        RotorFailureScenario scenario)
    {
        using Stage = AeroSimLabApp::CompleteFleetMissionState::Stage;
        const bool leaderFailed = scenario != RotorFailureScenario::Follower;
        const bool connectedNetwork = scenario
            == RotorFailureScenario::LeaderWithGroundControlRoute;
        const bool expectMissionRecovery = scenario
            != RotorFailureScenario::LeaderWithoutGroundControlRoute;
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        const auto droneIds = app.simulation_.DroneIds();
        ASSERT_EQ(droneIds.size(), 4U);
        ASSERT_TRUE(app.StartCompleteFleetMission(
            simulation::FormationShape::Vee)) << app.formationTransit_.status;
        if (connectedNetwork)
            UseConnectedNetworkForTest(app);

        simulation::DroneId failedDroneId = 0U;
        std::size_t failedTargetIndex = 0U;
        std::uint64_t originalAssignmentVersion = 0U;
        std::uint64_t lastReassignedAssignmentVersion = 0U;
        std::size_t reassignedTargetAssignmentCount = 0U;
        bool faultInjected = false;
        bool targetReassigned = false;
        bool sawRegrouping = false;
        bool sawReturning = false;
        double nearestDroneToGcsMeters = std::numeric_limits<double>::infinity();
        constexpr std::uint64_t maximumMissionTicks = 60000U;
        const simulation::DroneId initialLeaderId = app.fleetLeaderId_;
        for (std::uint64_t tick = 0U; tick < maximumMissionTicks; ++tick)
        {
            app.AdvanceSimulationTick();
            if (!faultInjected
                && app.completeFleetMission_.stage == Stage::Reconnaissance)
            {
                if (leaderFailed)
                    failedDroneId = app.fleetLeaderId_;
                else
                {
                    const auto follower = std::find_if(app.fleetDroneIds_.begin(),
                        app.fleetDroneIds_.end(), [&app](simulation::DroneId droneId)
                        {
                            return droneId != app.fleetLeaderId_;
                        });
                    ASSERT_NE(follower, app.fleetDroneIds_.end());
                    failedDroneId = *follower;
                }
                const auto& failedMission = app.fleetReconnaissance_.at(failedDroneId);
                failedTargetIndex = failedMission.targetIndex;
                originalAssignmentVersion = failedMission.assignmentVersion;
                nearestDroneToGcsMeters = std::numeric_limits<double>::infinity();
                for (const auto droneId : droneIds)
                    nearestDroneToGcsMeters = std::min(nearestDroneToGcsMeters,
                        glm::distance(app.groundControlPositionMeters_,
                            app.simulation_.State(droneId).positionMeters));
                app.simulation_.SetRotorMotorFault(failedDroneId, 0U, true);
                faultInjected = true;
            }

            const auto failedMission = app.fleetReconnaissance_.find(failedDroneId);
            if (faultInjected && failedMission != app.fleetReconnaissance_.end())
            {
                for (const auto droneId : app.fleetDroneIds_)
                {
                    const auto& mission = app.fleetReconnaissance_.at(droneId);
                    if (mission.leg != AeroSimLabApp::FleetMissionLeg::Reconnaissance
                        || mission.targetIndex != failedTargetIndex
                        || mission.assignmentVersion <= originalAssignmentVersion
                        || !mission.routeReceived
                        || mission.assignmentVersion == lastReassignedAssignmentVersion)
                        continue;
                    lastReassignedAssignmentVersion = mission.assignmentVersion;
                    ++reassignedTargetAssignmentCount;
                }
                targetReassigned = targetReassigned
                    || std::any_of(app.fleetDroneIds_.begin(), app.fleetDroneIds_.end(),
                        [&app, originalAssignmentVersion, failedTargetIndex](
                            simulation::DroneId droneId)
                        {
                            const auto& mission = app.fleetReconnaissance_.at(droneId);
                            return mission.targetIndex == failedTargetIndex
                                && mission.assignmentVersion > originalAssignmentVersion
                                && mission.routeReceived;
                        });
            }

            sawRegrouping = sawRegrouping
                || app.completeFleetMission_.stage == Stage::Regrouping;
            sawReturning = sawReturning
                || app.completeFleetMission_.stage == Stage::Returning;
            if (app.completeFleetMission_.stage == Stage::Completed
                || app.completeFleetMission_.stage == Stage::Failed)
                break;
        }

        ASSERT_TRUE(faultInjected);
        SCOPED_TRACE("시나리오=" + std::to_string(static_cast<int>(scenario))
            + ", 임무 상태=" + app.completeFleetMission_.status
            + ", 리더=" + std::to_string(app.fleetLeaderId_)
            + ", 재배정 횟수=" + std::to_string(reassignedTargetAssignmentCount)
            + ", Tick=" + std::to_string(app.simulation_.TickCount()));
        if (scenario == RotorFailureScenario::LeaderWithoutGroundControlRoute)
        {
            EXPECT_GT(nearestDroneToGcsMeters, 130.0)
                << "이 시험은 GCS와 모든 드론이 직접 통신 범위 안에 있습니다.";
            EXPECT_FALSE(targetReassigned)
                << "GCS 연결이 없는 상태에서 정찰 지점을 재배정했습니다.";
            EXPECT_EQ(reassignedTargetAssignmentCount, 0U);
            EXPECT_EQ(app.fleetLeaderId_, initialLeaderId);
            EXPECT_EQ(app.fleetLeadershipStatus_,
                "통신 가능한 새 리더를 찾지 못해 안전 착륙합니다");
            EXPECT_FALSE(app.leaderReplacementPending_);
            EXPECT_FALSE(app.leaderStateCollectionPending_);
        }
        else
        {
            if (leaderFailed)
                EXPECT_NE(app.fleetLeaderId_, initialLeaderId);
            EXPECT_TRUE(targetReassigned)
                << "고장 드론의 정찰 지점을 다른 기체에 넘기지 못했습니다.";
            EXPECT_EQ(reassignedTargetAssignmentCount, 1U)
                << "같은 정찰 지점이 반복 배정됐습니다.";
            EXPECT_TRUE(sawRegrouping);
            EXPECT_TRUE(sawReturning);
        }
        if (scenario == RotorFailureScenario::LeaderWithoutGroundControlRoute)
        {
            EXPECT_EQ(app.completeFleetMission_.stage, Stage::Failed);
            EXPECT_EQ(app.fleetLeaderId_, initialLeaderId)
                << "GCS와 끊긴 상태에서 확인되지 않은 리더를 승격했습니다.";
        }
        EXPECT_EQ(app.completeFleetMission_.stage,
            expectMissionRecovery ? Stage::Completed : Stage::Failed)
            << app.completeFleetMission_.status;
        EXPECT_EQ(app.fleetDroneIds_.size(), expectMissionRecovery ? 3U : 4U);
        if (expectMissionRecovery)
            EXPECT_NE(std::find(app.excludedFleetDroneIds_.begin(),
                app.excludedFleetDroneIds_.end(), failedDroneId),
                app.excludedFleetDroneIds_.end());
        for (const auto droneId : droneIds)
        {
            const auto mode = app.simulation_.State(droneId).mode;
            if (!expectMissionRecovery)
                EXPECT_TRUE(mode == simulation::FlightMode::Landing
                    || mode == simulation::FlightMode::Grounded)
                    << "GCS 연결이 끊긴 뒤 드론 " << droneId
                    << "에 안전 착륙 명령을 보내지 않았습니다.";
            else
                EXPECT_EQ(mode, simulation::FlightMode::Grounded)
                    << "고장 포함 임무가 끝난 뒤 드론 " << droneId
                    << "이 착륙하지 않았습니다.";
        }
        EXPECT_EQ(app.fleetReconnaissance_.at(failedDroneId).status,
            scenario == RotorFailureScenario::LeaderWithGroundControlRoute
                ? "리더 통신 장애 · 기체 안전 착륙"
                : "기체 고장 · 안전 착륙");
    }

    static void VerifyLostReconAssignmentWaitsForLeaseAndReassignsOnce()
    {
        AeroSimLabApp app({.smokeTest = true, .networkEnabled = false});
        app.InitializeGraphics();
        ASSERT_TRUE(app.PrepareFleetNetwork(true));

        const auto droneIds = app.fleetDroneIds_;
        ASSERT_EQ(droneIds.size(), 4U);
        const auto lostDroneId = droneIds[1];
        app.nextMissionAssignmentVersion_ = 8U;
        for (std::size_t index = 0; index < droneIds.size(); ++index)
        {
            AeroSimLabApp::FleetReconnaissanceState mission;
            mission.targetIndex = index;
            mission.leg = AeroSimLabApp::FleetMissionLeg::Reconnaissance;
            mission.status = index == 1U ? "정찰 이동 중" : "정찰 위치 도착";
            mission.routeReceived = index == 1U;
            if (index == 1U)
            {
                mission.assignmentVersion = 7U;
                mission.assignmentLeaseExpiresAtTick = 40U;
                mission.routeWaypointsMeters.push_back(
                    app.simulation_.State(lostDroneId).positionMeters
                        + glm::dvec3{100.0, 0.0, 10.0});
            }
            app.fleetReconnaissance_.emplace(droneIds[index], std::move(mission));
        }

        auto lostState = app.simulation_.State(lostDroneId);
        lostState.mode = simulation::FlightMode::Hovering;
        app.simulation_.SetInitialState(lostDroneId, lostState);
        app.peerTelemetryAtLeader_[lostDroneId] = {
            {.positionMeters = lostState.positionMeters,
                .mode = simulation::FlightMode::Hovering,
                .missionLeg = 0U,
                .missionStatus = "정찰 이동 중",
                .assignmentVersion = 7U,
                .assignmentLeaseExpiresAtTick = 40U},
            0U, 1U};
        app.formationNetwork_->SetNodeRadioEnabled(lostDroneId, false);

        const auto advanceTick = [&app]
        {
            app.AdvanceFleetNetwork();
            app.AdvanceFleetReconnaissance();
            app.ReassignLostLeaderTarget();
            app.simulation_.AdvanceOneTick();
        };
        for (std::uint64_t tick = 0U; tick < 40U; ++tick)
            advanceTick();

        EXPECT_TRUE(app.pendingReassignedTargetIndices_.empty());
        EXPECT_EQ(app.fleetReconnaissance_.at(lostDroneId).status, "정찰 이동 중");
        app.AdvanceFleetReconnaissance();
        EXPECT_EQ(app.fleetReconnaissance_.at(lostDroneId).status,
            "임무 권한 만료 · 안전 착륙");
        EXPECT_EQ(app.simulation_.State(lostDroneId).mode,
            simulation::FlightMode::Landing);
        app.ReassignLostLeaderTarget();
        EXPECT_TRUE(app.pendingReassignedTargetIndices_.empty());

        for (std::uint64_t tick = 0U; tick < 200U
                && app.fleetReconnaissance_.at(lostDroneId).status
                    != "임무 권한 만료 · 재배정 대기";
            ++tick)
            advanceTick();

        EXPECT_EQ(app.fleetReconnaissance_.at(lostDroneId).status,
            "임무 권한 만료 · 재배정 대기");
        EXPECT_TRUE(app.pendingReassignedTargetIndices_.empty());
        const auto targetWasReassigned = [&app, lostDroneId, &droneIds]
        {
            return std::any_of(droneIds.begin(), droneIds.end(),
                [&app, lostDroneId](simulation::DroneId droneId)
            {
                if (droneId == lostDroneId)
                    return false;
                const auto& mission = app.fleetReconnaissance_.at(droneId);
                return mission.targetIndex == 1U
                    && mission.assignmentVersion == 8U
                    && mission.routeReceived
                    && mission.assignmentLeaseExpiresAtTick
                        > app.simulation_.TickCount();
            });
        };
        for (std::uint64_t tick = 0U; tick < 200U && !targetWasReassigned(); ++tick)
            advanceTick();

        ASSERT_TRUE(targetWasReassigned())
            << "재배정 경로를 받지 못했습니다. 리더 상태: "
            << app.fleetLeadershipStatus_ << ", Tick: "
            << app.simulation_.TickCount();
        const auto reassignedDrone = std::find_if(droneIds.begin(), droneIds.end(),
            [&app, lostDroneId](simulation::DroneId droneId)
            {
                if (droneId == lostDroneId)
                    return false;
                const auto& mission = app.fleetReconnaissance_.at(droneId);
                return mission.targetIndex == 1U
                    && mission.assignmentVersion == 8U
                    && mission.routeReceived;
            });
        ASSERT_NE(reassignedDrone, droneIds.end());
        EXPECT_EQ(app.simulation_.State(*reassignedDrone).mode,
            simulation::FlightMode::TakingOff);
        EXPECT_EQ(app.nextMissionAssignmentVersion_, 9U);
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

    const auto relayTelemetry = simulator::detail::DecodePeerTelemetry(
        simulator::detail::EncodePeerTelemetry(positionMeters,
            simulation::FlightMode::Flying, 3U, "중계 위치 확보", 18U, 2500U));
    ASSERT_TRUE(relayTelemetry.has_value());
    EXPECT_EQ(relayTelemetry->missionLeg, 3U);
    EXPECT_EQ(relayTelemetry->missionStatus, "중계 위치 확보");

    EXPECT_FALSE(simulator::detail::DecodePeerTelemetry(
        "12.5 -7.25 31 Hovering 4 \"잘못된 단계\"").has_value());
}

void Verify_FleetMissionProgressWaitsForEveryCurrentReport()
{
    using simulator::detail::FleetMissionPhase;
    using simulator::detail::FleetMissionProgress;
    using simulator::detail::FleetMissionReport;
    using simulation::FlightMode;

    std::vector<std::optional<FleetMissionReport>> reports{
        FleetMissionReport{FlightMode::Grounded, 0U, "정찰 위치 도착"},
        FleetMissionReport{FlightMode::Grounded, 0U, "정찰 위치 도착"},
        FleetMissionReport{FlightMode::Grounded, 0U, "정찰 위치 도착"},
        FleetMissionReport{FlightMode::Grounded, 0U, "정찰 위치 도착"}};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Reconnaissance, reports).progress,
        FleetMissionProgress::Advance);

    reports.back() = FleetMissionReport{FlightMode::Flying, 3U, "중계 위치 확보"};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Reconnaissance, reports).progress,
        FleetMissionProgress::Advance);
    reports.back() = FleetMissionReport{FlightMode::Grounded, 3U, "중계 위치 확보"};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Reconnaissance, reports).progress,
        FleetMissionProgress::Waiting);
    reports.back() = FleetMissionReport{
        FlightMode::Landing, 3U, "중계 실패 · 안전 착륙"};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Reconnaissance, reports).progress,
        FleetMissionProgress::Failed);
    reports.back() = FleetMissionReport{FlightMode::Grounded, 0U, "정찰 위치 도착"};

    reports.back().reset();
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Reconnaissance, reports).progress,
        FleetMissionProgress::Waiting);
    reports.back() = FleetMissionReport{FlightMode::Grounded, 1U, "재집결 완료"};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Reconnaissance, reports).progress,
        FleetMissionProgress::Waiting);

    reports.back() = FleetMissionReport{FlightMode::Grounded, 0U, "경로 오류"};
    const auto failed = simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Reconnaissance, reports);
    EXPECT_EQ(failed.progress, FleetMissionProgress::Failed);
    EXPECT_FALSE(failed.failureReason.empty());

    reports.assign(4U, FleetMissionReport{
        FlightMode::Hovering, 1U, "재집결 완료"});
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Regrouping, reports).progress,
        FleetMissionProgress::Advance);
    reports.back() = FleetMissionReport{FlightMode::Hovering, 1U, "집결 실패"};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Regrouping, reports).progress,
        FleetMissionProgress::Failed);
}

void Verify_FleetMissionProgressRequiresLandedReturnReports()
{
    using simulator::detail::FleetMissionPhase;
    using simulator::detail::FleetMissionProgress;
    using simulator::detail::FleetMissionReport;
    using simulation::FlightMode;

    std::vector<std::optional<FleetMissionReport>> reports{
        FleetMissionReport{FlightMode::Grounded, 2U, "복귀 완료"},
        FleetMissionReport{FlightMode::Hovering, 2U, "복귀 완료"}};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Returning, reports).progress,
        FleetMissionProgress::Waiting);
    reports.back() = FleetMissionReport{FlightMode::Grounded, 2U, "복귀 완료"};
    EXPECT_EQ(simulator::detail::EvaluateFleetMissionProgress(
        FleetMissionPhase::Returning, reports).progress,
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
    {
        const auto path = std::filesystem::temp_directory_path()
            / "dronesim-fault-timeline-test.json";
        std::error_code filesystemError;
        std::filesystem::remove(path, filesystemError);

        simulator::FaultScenarioTimeline recording;
        recording.StartRecording(100U, {1U, 2U}, "complete");
        recording.RecordPacketLoss(100U, 0.25);
        recording.RecordLinkLatencyJitter(102U, 7U);
        recording.RecordCameraFault(103U, 2U, true);
        recording.RecordRotorMotorFault(103U, 2U, 3U, true);
        recording.RecordLinkState(103U, 1U, 2U, true);
        recording.RecordDirectedLinkState(103U, 2U, 1U, true);
        recording.RecordRadioState(103U, 2U, false);
        recording.StopRecording();

        std::string error;
        ASSERT_TRUE(recording.SaveRecording(path, error)) << error;

        simulator::FaultScenarioTimeline replay;
        ASSERT_TRUE(replay.LoadRecording(path, error)) << error;
        EXPECT_FALSE(replay.StartReplay(500U, {1U, 2U}, "formation", error));
        ASSERT_TRUE(replay.StartReplay(500U, {1U, 2U}, "complete", error)) << error;
        const auto firstEvents = replay.TakeDueEvents(500U);
        ASSERT_EQ(firstEvents.size(), 1U);
        EXPECT_EQ(firstEvents.front().kind,
            simulator::FaultScenarioTimeline::Event::Kind::PacketLoss);
        EXPECT_DOUBLE_EQ(firstEvents.front().value, 0.25);
        const auto jitterEvents = replay.TakeDueEvents(502U);
        ASSERT_EQ(jitterEvents.size(), 1U);
        EXPECT_EQ(jitterEvents.front().kind,
            simulator::FaultScenarioTimeline::Event::Kind::LinkLatencyJitter);
        EXPECT_EQ(jitterEvents.front().valueTicks, 7U);
        EXPECT_TRUE(replay.TakeDueEvents(502U).empty());
        const auto laterEvents = replay.TakeDueEvents(503U);
        ASSERT_EQ(laterEvents.size(), 5U);
        EXPECT_EQ(laterEvents[0].kind,
            simulator::FaultScenarioTimeline::Event::Kind::CameraFault);
        EXPECT_EQ(laterEvents[0].source, 2U);
        EXPECT_TRUE(laterEvents[0].enabled);
        EXPECT_EQ(laterEvents[1].kind,
            simulator::FaultScenarioTimeline::Event::Kind::RotorMotorFault);
        EXPECT_EQ(laterEvents[1].source, 2U);
        EXPECT_EQ(laterEvents[1].rotorIndex, 3U);
        EXPECT_TRUE(laterEvents[1].enabled);
        EXPECT_EQ(laterEvents[2].kind,
            simulator::FaultScenarioTimeline::Event::Kind::LinkState);
        EXPECT_TRUE(laterEvents[2].enabled);
        EXPECT_EQ(laterEvents[3].kind,
            simulator::FaultScenarioTimeline::Event::Kind::DirectedLinkState);
        EXPECT_EQ(laterEvents[3].source, 2U);
        EXPECT_EQ(laterEvents[3].destination, 1U);
        EXPECT_TRUE(laterEvents[3].enabled);
        EXPECT_EQ(laterEvents[4].kind,
            simulator::FaultScenarioTimeline::Event::Kind::RadioState);
        EXPECT_FALSE(laterEvents[4].enabled);
        EXPECT_TRUE(replay.ReplayFinished());

        std::filesystem::remove(path, filesystemError);
    }
    {
        simulator::FaultScenarioTimeline scheduled;
        simulator::FaultScenarioTimeline::Event linkFault;
        linkFault.kind = simulator::FaultScenarioTimeline::Event::Kind::LinkState;
        linkFault.source = 1U;
        linkFault.destination = 2U;
        linkFault.enabled = true;
        std::string error;
        EXPECT_FALSE(scheduled.ScheduleEvent(
            1000U, {1U, 2U}, "complete", 60U, 0U, linkFault, error));
        EXPECT_FALSE(scheduled.IsReplaying());

        ASSERT_TRUE(scheduled.ScheduleEvent(
            1000U, {1U, 2U}, "complete", 60U, 120U, linkFault, error))
            << error;
        simulator::FaultScenarioTimeline::Event cameraFault;
        cameraFault.kind = simulator::FaultScenarioTimeline::Event::Kind::CameraFault;
        cameraFault.source = 2U;
        cameraFault.enabled = true;
        ASSERT_TRUE(scheduled.ScheduleEvent(
            1000U, {1U, 2U}, "complete", 60U, 60U, cameraFault, error))
            << error;
        EXPECT_EQ(scheduled.ScheduledFaultCount(), 2U);
        const auto pending = scheduled.PendingEvents(1000U);
        ASSERT_EQ(pending.size(), 4U);
        EXPECT_EQ(pending.front().tickOffset, 60U);
        EXPECT_TRUE(scheduled.TakeDueEvents(1059U).empty());

        const auto starts = scheduled.TakeDueEvents(1060U);
        ASSERT_EQ(starts.size(), 2U);
        EXPECT_TRUE(starts[0].enabled);
        EXPECT_TRUE(starts[1].enabled);
        const auto cameraRestore = scheduled.TakeDueEvents(1120U);
        ASSERT_EQ(cameraRestore.size(), 1U);
        EXPECT_EQ(cameraRestore.front().kind,
            simulator::FaultScenarioTimeline::Event::Kind::CameraFault);
        EXPECT_FALSE(cameraRestore.front().enabled);
        const auto linkRestore = scheduled.TakeDueEvents(1180U);
        ASSERT_EQ(linkRestore.size(), 1U);
        EXPECT_EQ(linkRestore.front().kind,
            simulator::FaultScenarioTimeline::Event::Kind::LinkState);
        EXPECT_FALSE(linkRestore.front().enabled);
        EXPECT_TRUE(scheduled.ReplayFinished());

        const auto scheduledPath = std::filesystem::temp_directory_path()
            / "dronesim-scheduled-faults-test.json";
        ASSERT_TRUE(scheduled.SaveRecording(scheduledPath, error)) << error;
        simulator::FaultScenarioTimeline reloaded;
        ASSERT_TRUE(reloaded.LoadRecording(scheduledPath, error)) << error;
        EXPECT_EQ(reloaded.ScheduledFaultCount(), 2U);
        EXPECT_EQ(reloaded.EventCount(), 4U);
        ASSERT_TRUE(reloaded.StartReplay(2000U, {1U, 2U}, "complete", error))
            << error;
        EXPECT_TRUE(reloaded.TakeDueEvents(2059U).empty());
        EXPECT_EQ(reloaded.TakeDueEvents(2060U).size(), 2U);
        std::error_code removeError;
        std::filesystem::remove(scheduledPath, removeError);
    }
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyLeaderRadioLossPromotesCandidateAndReassignsTarget());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyGroundedRelayDroneTakesOffForAssignment());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyRelayRecoveryUsesTheConfiguredRadioRange());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyGroundControlShowsOnlyDeliveredPeerPositions());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyObserverCameraKeepsAUsableFollowDistance());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyLeaderLossWithoutCandidateSafelyLandsFleet());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyFormationTransitSafelyLandsWhenFollowerTelemetryIsMissing());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyCompleteFleetMissionFinishesAndLands());
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyCompleteFleetMissionReassignsAfterRotorFailure(
            simulator::AeroSimLabAppTestAccess::RotorFailureScenario::Follower));
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyCompleteFleetMissionReassignsAfterRotorFailure(
            simulator::AeroSimLabAppTestAccess::RotorFailureScenario::LeaderWithoutGroundControlRoute));
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyCompleteFleetMissionReassignsAfterRotorFailure(
            simulator::AeroSimLabAppTestAccess::RotorFailureScenario::LeaderWithGroundControlRoute));
    EXPECT_NO_FATAL_FAILURE(
        simulator::AeroSimLabAppTestAccess::VerifyLostReconAssignmentWaitsForLeaseAndReassignsOnce());
    EXPECT_NO_FATAL_FAILURE(Verify_PeerTelemetryPreservesMissionStateAndRejectsInvalidLeg());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressWaitsForEveryCurrentReport());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetMissionProgressRequiresLandedReturnReports());
    EXPECT_NO_FATAL_FAILURE(Verify_EoVideoRecorderStoresH264AndFrameMetadata());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetDroneSpawnRejectsAPositionOutsideTheTerrain());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorSession_RecordingReplayOverUdp());
}
