#include "DroneSimApp.h"
#include "gui/SimulatorView.h"
#include "FleetMissionProgress.h"
#include "FleetMissionMessages.h"
#include "FleetTelemetry.h"

#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "flight_planning/FormationPlanner.h"
#include "flight_planning/RoutePlanner.h"
#include "scene/TrainingRangeRouteEnvironment.h"
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
#include <iostream>
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
        ASSERT_EQ(app.trainingRangeScene_->ReconPoints().size(), 4U);
        double farthestTargetMeters = 0.0;
        double closestTargetSpacingMeters = std::numeric_limits<double>::infinity();
        for (const auto& target : app.trainingRangeScene_->ReconPoints())
        {
            farthestTargetMeters = std::max(farthestTargetMeters,
                glm::distance(target.positionMeters,
                    app.trainingRangeScene_->FormationStagingPositionMeters()));
            for (const auto& other : app.trainingRangeScene_->ReconPoints())
                if (&target != &other)
                    closestTargetSpacingMeters = std::min(closestTargetSpacingMeters,
                        glm::distance(target.positionMeters, other.positionMeters));
        }
        EXPECT_GT(farthestTargetMeters, 25.0);
        EXPECT_GE(closestTargetSpacingMeters, 15.0);
        std::cout << "M02 정찰 지점 분포: 출발지에서 가장 먼 지점 "
            << farthestTargetMeters << " m, 가까운 두 지점 간격 "
            << closestTargetSpacingMeters << " m\n";
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

        app.fleetMission_.SetAssignment(app.simulation_.PrimaryDroneId(), {});
        app.fleetMission_.Assignment(app.simulation_.PrimaryDroneId()).UpdateStatus(
            mission::FleetDroneStatus::Reconnaissance);
        EXPECT_FALSE(app.FleetReconnaissanceCanStart());
        EXPECT_FALSE(app.CompleteFleetMissionCanStart());
        app.fleetMission_.ClearAssignments();
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
        const scene::TrainingRangeRouteEnvironment environment(
            app.terrainGrid_, app.trainingRangeScene_->BuildingObstacles());
        const auto expectedRoute = flight_planning::RoutePlanner{}.Plan(
            environment,
            app.simulation_.State(droneId).positionMeters,
            app.reconTargets_.front().approachPositionMeters,
            {.obstacleClearanceMeters = 3.0,
             .minimumTransitAltitudeMeters = app.cameraTransitAltitudeMeters_});
        ASSERT_TRUE(expectedRoute);
        ASSERT_FALSE(expectedRoute->empty());
        ASSERT_TRUE(app.StartFleetReconnaissance());

        // 경로 탐색 결과는 출발점을 제외하므로 첫 항목도 임무에 필요하다.
        EXPECT_EQ(app.fleetMission_.Assignment(droneId).RouteWaypoints(),
            *expectedRoute);
    }

    static void UseConnectedNetworkForTest(DroneSimApp& app)
    {
        communication::VirtualMeshSettings settings;
        settings.maximumLinkRangeMeters = 1'000.0;
        app.formationNetwork_ = std::make_unique<
            communication::VirtualMeshNetwork>(app.droneSocketTransport_, settings);
        app.formationRadioObstacles_.clear();
        app.fleetMission_.FormationTransit().SetRoute({});
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

        app.fleetMission_.FormationTransit().Start("시험 중 편대 이동");
        flight_planning::FormationRoutePlan route;
        route.leaderStartMeters = app.simulation_.State(droneIds.front()).positionMeters;
        route.leaderWaypointsMeters.push_back(
            route.leaderStartMeters + glm::dvec3{10.0, 0.0, 0.0});
        app.fleetMission_.FormationTransit().SetRoute(std::move(route));
        app.fleetMission_.CompleteMission().SetStage(
            mission::CompleteFleetMissionState::Stage::FormationTransit);

        app.AdvanceFormationTransit();

        EXPECT_EQ(app.fleetMission_.FormationTransit().CurrentPhase(),
            mission::FormationTransitState::Phase::Failed);
        EXPECT_EQ(app.fleetMission_.CompleteMission().CurrentStage(),
            mission::CompleteFleetMissionState::Stage::Failed);
        EXPECT_EQ(app.fleetMission_.CompleteMission().StatusText(),
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
            app.fleetMission_.CompleteMission().RecordHomePosition(droneId, state.positionMeters);
            if (droneId != app.fleetMission_.LeaderId())
            {
                mission::PeerTelemetry report;
                report.positionMeters = state.positionMeters;
                report.mode = state.mode;
                ASSERT_TRUE(app.fleetMission_.TelemetryAtLeader().Record(droneId, report,
                    app.simulation_.TickCount(), 1U));
            }
        }

        // 도착 상태라면 안내 문구가 바뀌어도 다음 정찰 단계를 시작한다.
        app.fleetMission_.CompleteMission().SetStage(MissionStage::FormationTransit);
        app.fleetMission_.FormationTransit().Arrive("다른 도착 안내");
        app.AdvanceCompleteFleetMission();
        ASSERT_EQ(app.fleetMission_.CompleteMission().CurrentStage(), MissionStage::Reconnaissance)
            << app.fleetMission_.CompleteMission().StatusText();
        app.FailCompleteFleetMission("정찰 단계 중단");
        EXPECT_EQ(app.fleetMission_.FormationTransit().CurrentPhase(), FormationPhase::Arrived);

        // 실패 상태는 도착 문구가 남아 있어도 성공으로 처리하지 않는다.
        app.fleetMission_.CompleteMission().SetStage(MissionStage::FormationTransit);
        app.fleetMission_.FormationTransit().Fail("목적지 도착 · 편대 유지 중");
        app.AdvanceCompleteFleetMission();
        EXPECT_EQ(app.fleetMission_.CompleteMission().CurrentStage(), MissionStage::Failed);
        EXPECT_EQ(app.fleetMission_.CompleteMission().StatusText(),
            "M01 편대 이동이 끝나지 않아 전체 임무를 중단했습니다.");
    }

    static void VerifyCompleteFleetMissionFinishesAndLands()
    {
        using Stage = mission::CompleteFleetMissionState::Stage;
        DroneSimApp app({.smokeTest = true});
        app.InitializeGraphics();
        const auto droneIds = app.simulation_.DroneIds();

        ASSERT_TRUE(app.StartCompleteFleetMission())
            << app.fleetMission_.FormationTransit().StatusText();
        bool sawReconnaissance = false;
        bool sawRegrouping = false;
        bool sawReturning = false;
        constexpr std::uint64_t maximumMissionTicks = 60000U;
        std::map<simulation::DroneId, std::uint64_t> lastAssignmentVersions;
        std::vector<std::string> assignmentHistory;
        std::vector<double> formationErrorsMeters;
        std::vector<double> horizontalFormationErrorsMeters;
        std::vector<double> verticalFormationErrorsMeters;
        simulation::DroneId previousLeaderId = app.fleetMission_.LeaderId();
        for (std::uint64_t tick = 0U; tick < maximumMissionTicks; ++tick)
        {
            app.AdvanceSimulationTick();
            const auto& formation = app.fleetMission_.FormationTransit();
            const auto& formationRoute = formation.Route().leaderWaypointsMeters;
            if (formation.CurrentPhase() == mission::FormationTransitState::Phase::InProgress
                && !formationRoute.empty())
            {
                const auto& leader = app.simulation_.State(app.fleetMission_.LeaderId());
                const std::size_t waypointIndex = formation.WaypointIndex();
                glm::dvec2 direction{0.0};
                if (waypointIndex + 1U < formationRoute.size())
                    direction = {formationRoute[waypointIndex + 1U].x
                            - formationRoute[waypointIndex].x,
                        formationRoute[waypointIndex + 1U].y
                            - formationRoute[waypointIndex].y};
                else if (waypointIndex > 0U)
                    direction = {formationRoute[waypointIndex].x
                            - formationRoute[waypointIndex - 1U].x,
                        formationRoute[waypointIndex].y
                            - formationRoute[waypointIndex - 1U].y};
                const double heading = glm::length(direction) > 1.0e-6
                    ? std::atan2(direction.y, direction.x) : leader.yawRadians;
                const auto offsets = flight_planning::FormationOffsets(
                    droneIds.size(), app.fleetMission_.FormationTransit().Route().spacingMeters, heading);
                for (std::size_t index = 1U; index < droneIds.size(); ++index)
                {
                    const auto& follower = app.simulation_.State(droneIds[index]);
                    const glm::dvec3 target{
                        leader.positionMeters.x + offsets[index].x,
                        leader.positionMeters.y + offsets[index].y,
                        leader.positionMeters.z};
                    const glm::dvec3 positionError =
                        follower.positionMeters - target;
                    formationErrorsMeters.push_back(glm::length(positionError));
                    horizontalFormationErrorsMeters.push_back(glm::length(
                        glm::dvec2{positionError.x, positionError.y}));
                    verticalFormationErrorsMeters.push_back(
                        std::abs(positionError.z));
                }
            }
            if (previousLeaderId != app.fleetMission_.LeaderId())
            {
                assignmentHistory.push_back("Tick "
                    + std::to_string(app.simulation_.TickCount())
                    + ": 리더 " + std::to_string(previousLeaderId) + " → "
                    + std::to_string(app.fleetMission_.LeaderId()));
                previousLeaderId = app.fleetMission_.LeaderId();
            }
            for (const auto& [droneId, mission] : app.fleetMission_.Assignments())
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
                        mission.Status())));
                lastVersion = mission.assignmentVersion;
            }
            sawReconnaissance = sawReconnaissance
                || app.fleetMission_.CompleteMission().CurrentStage() == Stage::Reconnaissance;
            sawRegrouping = sawRegrouping
                || app.fleetMission_.CompleteMission().CurrentStage() == Stage::Regrouping;
            sawReturning = sawReturning
                || app.fleetMission_.CompleteMission().CurrentStage() == Stage::Returning;
            if (app.fleetMission_.CompleteMission().CurrentStage() == Stage::Completed
                || app.fleetMission_.CompleteMission().CurrentStage() == Stage::Failed)
                break;
        }

        std::ostringstream failureContext;
        failureContext << "전체 임무가 끝나지 않았습니다. 상태: "
            << app.fleetMission_.CompleteMission().StatusText() << ", Tick: "
            << app.simulation_.TickCount();
        if (app.fleetMission_.CompleteMission().CurrentStage() != Stage::Completed)
        {
            const auto& leader = app.simulation_.State(app.fleetMission_.LeaderId());
            failureContext << ", 리더 ID " << app.fleetMission_.LeaderId() << " 위치: ("
                << leader.positionMeters.x << ", "
                << leader.positionMeters.y << ", " << leader.positionMeters.z
                << "), 임무 단계 "
                << static_cast<int>(app.fleetMission_.CompleteMission().CurrentStage());
            for (const auto droneId : droneIds)
            {
                const auto& drone = app.simulation_.State(droneId);
                failureContext << "; 드론 " << droneId << " 위치 ("
                    << drone.positionMeters.x << ", " << drone.positionMeters.y
                    << ", " << drone.positionMeters.z << ")";
                if (droneId != app.fleetMission_.LeaderId())
                {
                    const auto* report = app.fleetMission_.TelemetryAtLeader().Latest(droneId);
                    failureContext << ", 리더와 거리 "
                        << glm::distance(drone.positionMeters,
                            leader.positionMeters) << "m, 보고 경과 ";
                    if (report == nullptr)
                        failureContext << "없음";
                    else
                        failureContext << app.simulation_.TickCount()
                            - report->receivedAtTick << " Tick";
                }
                const auto mission = app.fleetMission_.FindAssignment(droneId);
                if (mission != app.fleetMission_.AssignmentsEnd())
                    failureContext << ", 목표 " << mission->second.targetIndex
                        << ", 상태 " << mission::FleetDroneStatusLabel(
                            mission->second.Status());
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

        EXPECT_EQ(app.fleetMission_.CompleteMission().CurrentStage(), Stage::Completed)
            << failureContext.str();
        EXPECT_TRUE(sawReconnaissance) << "M02 정찰 단계에 도달하지 못했습니다.";
        EXPECT_TRUE(sawRegrouping) << "M03 재집결 단계에 도달하지 못했습니다.";
        EXPECT_TRUE(sawReturning) << "기지 복귀 단계에 도달하지 못했습니다.";
        ASSERT_FALSE(formationErrorsMeters.empty())
            << "M01 비행 중 편대 오차 표본을 수집하지 못했습니다.";
        std::sort(formationErrorsMeters.begin(), formationErrorsMeters.end());
        std::sort(horizontalFormationErrorsMeters.begin(),
            horizontalFormationErrorsMeters.end());
        std::sort(verticalFormationErrorsMeters.begin(),
            verticalFormationErrorsMeters.end());
        const std::size_t percentile95Index = static_cast<std::size_t>(
            std::ceil(0.95 * static_cast<double>(formationErrorsMeters.size()))) - 1U;
        const double percentile95Meters = formationErrorsMeters[percentile95Index];
        const double maximumFormationErrorMeters = formationErrorsMeters.back();
        const double horizontalPercentile95Meters =
            horizontalFormationErrorsMeters[percentile95Index];
        const double verticalPercentile95Meters =
            verticalFormationErrorsMeters[percentile95Index];
        std::cout << "M01 편대 오차: 표본 " << formationErrorsMeters.size()
            << ", p95 " << percentile95Meters << " m, 최대 "
            << maximumFormationErrorMeters << " m, 수평 p95 "
            << horizontalPercentile95Meters << " m, 수직 p95 "
            << verticalPercentile95Meters << " m\n";
        EXPECT_LE(percentile95Meters, 6.0)
            << "편대 오차 p95가 6m를 넘습니다. 최대 오차: "
            << maximumFormationErrorMeters << " m";
        EXPECT_LE(maximumFormationErrorMeters, 8.0)
            << "순간 편대 오차가 8m를 넘습니다.";
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
            app.fleetMission_.Assignment(lowBatteryDroneId).targetIndex;
        app.simulation_.ConfigureBattery(lowBatteryDroneId,
            {10.0, 40.0, 120.0, 99.9});

        for (std::uint64_t tick = 0U; tick < 30000U; ++tick)
        {
            app.AdvanceSimulationTick();
            const auto& mission = app.fleetMission_.Assignment(lowBatteryDroneId);
            if (mission.IsLowBatteryReturn()
                && mission.Status()
                    == mission::FleetDroneStatus::LowBatteryReturnComplete)
                break;
        }

        const auto& mission = app.fleetMission_.Assignment(lowBatteryDroneId);
        EXPECT_TRUE(app.simulation_.Battery(lowBatteryDroneId)
            .IsReturningForLowBattery());
        EXPECT_TRUE(mission.IsLowBatteryReturn());
        EXPECT_EQ(mission.Leg(), mission::FleetMissionLeg::Returning);
        EXPECT_EQ(mission.Status(),
            mission::FleetDroneStatus::LowBatteryReturnComplete);
        EXPECT_LE(glm::distance(app.simulation_.State(lowBatteryDroneId).positionMeters,
            mission.startPositionMeters), 5.0);
        const bool targetWasReassigned = std::any_of(app.fleetDroneIds_.begin(),
            app.fleetDroneIds_.end(), [&app, lowBatteryDroneId, unfinishedTargetIndex](auto id)
            {
                return id != lowBatteryDroneId
                    && app.fleetMission_.Assignment(id).targetIndex
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
    const auto reportAgeTicks = receivedStates.AgeInTicks(4U, 130U);
    ASSERT_TRUE(reportAgeTicks.has_value());
    EXPECT_EQ(*reportAgeTicks, 30U);
    const auto ageAfterTickReset = receivedStates.AgeInTicks(4U, 99U);
    ASSERT_TRUE(ageAfterTickReset.has_value());
    EXPECT_EQ(*ageAfterTickReset, 0U);
    EXPECT_FALSE(receivedStates.AgeInTicks(5U, 130U).has_value());
    ASSERT_NE(receivedStates.Fresh(4U, 130U), nullptr);
    EXPECT_EQ(receivedStates.Fresh(4U, 130U)->state.positionMeters, positionMeters);
    EXPECT_EQ(receivedStates.Fresh(4U, 131U), nullptr);
    EXPECT_NE(receivedStates.Latest(4U), nullptr);

    mission::FleetTelemetryStore nearTickLimit;
    const auto highestTick = std::numeric_limits<std::uint64_t>::max();
    EXPECT_TRUE(nearTickLimit.Record(4U, *telemetry, highestTick - 10U, 1U));
    const auto ageNearTickLimit = nearTickLimit.AgeInTicks(4U, highestTick);
    ASSERT_TRUE(ageNearTickLimit.has_value());
    EXPECT_EQ(*ageNearTickLimit, 10U);
    EXPECT_NE(nearTickLimit.Fresh(4U, highestTick), nullptr);

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

void Verify_FleetRouteMessagesRoundTripAndRejectInvalidValues()
{
    const std::vector<glm::dvec3> waypoints{
        {12.5, -7.25, 31.0}, {18.0, 2.0, 36.0}};
    const auto decodedWaypoint = mission::DecodeWaypoint(
        mission::EncodeWaypoint(waypoints.front()));
    ASSERT_TRUE(decodedWaypoint.has_value());
    EXPECT_EQ(*decodedWaypoint, waypoints.front());
    const std::string payload = mission::EncodeFleetRoute(
        mission::FleetMissionLeg::Regrouping, 3U, 17U, 2500U, waypoints);
    const auto decoded = mission::DecodeFleetRoute(payload);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->leg, mission::FleetMissionLeg::Regrouping);
    EXPECT_EQ(decoded->targetIndex, 3U);
    EXPECT_EQ(decoded->assignmentVersion, 17U);
    EXPECT_EQ(decoded->assignmentLeaseExpiresAtTick, 2500U);
    ASSERT_EQ(decoded->waypointsMeters.size(), waypoints.size());
    EXPECT_EQ(decoded->waypointsMeters, waypoints);
    EXPECT_FALSE(mission::DecodeFleetRoute("99 0 1 0 0").has_value());
    EXPECT_FALSE(mission::DecodeFleetRoute(payload + " trailing").has_value());
    EXPECT_FALSE(mission::DecodeFleetRoute(
        "0 0 1 0 18446744073709551615").has_value());
    EXPECT_FALSE(mission::DecodeWaypoint("1 2 nan").has_value());

    const std::vector<glm::dvec3> leaseRoute{{10.0, 0.0, 0.0}};
    EXPECT_EQ(mission::EstimateMissionLeaseExpiry(100U, glm::dvec3{0.0}, leaseRoute),
        2500U);
}

void Verify_FleetMissionProgressWaitsForEveryCurrentReport()
{
    using mission::FleetMissionLeg;
    using mission::FleetMissionProgress;
    using mission::FleetMissionReport;
    using mission::FleetDroneStatus;
    using simulation::FlightMode;

    mission::FleetMissionState fleetState;
    using CompleteStage = mission::CompleteFleetMissionState::Stage;
    EXPECT_FALSE(fleetState.CompleteMission().IsActive());
    fleetState.CompleteMission().SetStage(CompleteStage::FormationTransit);
    EXPECT_TRUE(fleetState.CompleteMission().IsActive());
    fleetState.CompleteMission().SetStage(CompleteStage::Completed);
    EXPECT_FALSE(fleetState.CompleteMission().IsActive());
    EXPECT_EQ(fleetState.CompleteMission().StatusText(), "전체 임무 완료");
    fleetState.CompleteMission().SetStage(CompleteStage::Returning);
    EXPECT_EQ(fleetState.CompleteMission().StatusText(), "M03 기지로 복귀 중");
    fleetState.CompleteMission().WaitForReports();
    EXPECT_EQ(fleetState.CompleteMission().StatusText(), "드론 상태 보고를 기다리는 중");
    fleetState.CompleteMission().Fail("시험 실패 사유");
    EXPECT_EQ(fleetState.CompleteMission().StatusText(), "시험 실패 사유");
    EXPECT_FALSE(fleetState.HasActiveReconnaissance());
    fleetState.SetAssignment(1, {});
    fleetState.Assignment(1).UpdateStatus(FleetDroneStatus::Reconnaissance);
    EXPECT_TRUE(fleetState.HasActiveReconnaissance());
    fleetState.Assignment(1).UpdateStatus(FleetDroneStatus::TargetReached);
    EXPECT_FALSE(fleetState.HasActiveReconnaissance());

    mission::FleetReconnaissanceState routeProgress;
    routeProgress.BeginLeg(FleetMissionLeg::Reconnaissance);
    routeProgress.ReceiveRoute({{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}});
    EXPECT_EQ(routeProgress.Status(), FleetDroneStatus::RoutePending);
    EXPECT_TRUE(routeProgress.HasReceivedRoute());
    routeProgress.BeginWaypointCommand();
    EXPECT_EQ(routeProgress.Status(), FleetDroneStatus::Reconnaissance);
    EXPECT_TRUE(routeProgress.HasCommandedWaypoint());
    ASSERT_NE(routeProgress.CurrentWaypoint(), nullptr);
    EXPECT_EQ(*routeProgress.CurrentWaypoint(), glm::dvec3(0.0, 0.0, 5.0));
    EXPECT_TRUE(routeProgress.AdvanceWaypointIfReached({1.0, 0.0, 5.0}, 2.0));
    EXPECT_EQ(*routeProgress.CurrentWaypoint(), glm::dvec3(10.0, 0.0, 5.0));
    EXPECT_FALSE(routeProgress.AdvanceWaypointIfReached({1.0, 0.0, 5.0}, 2.0));
    routeProgress.BeginLowBatteryReturn();
    EXPECT_TRUE(routeProgress.IsLowBatteryReturn());
    EXPECT_FALSE(routeProgress.IsLowBatteryReturnComplete());
    EXPECT_EQ(routeProgress.Leg(), FleetMissionLeg::Returning);
    EXPECT_EQ(routeProgress.Status(), FleetDroneStatus::RoutePending);
    EXPECT_FALSE(routeProgress.HasReceivedRoute());
    routeProgress.UpdateStatus(FleetDroneStatus::LowBatteryReturnComplete);
    EXPECT_TRUE(routeProgress.IsLowBatteryReturnComplete());

    mission::FleetRouteDeliveryState routeDelivery;
    EXPECT_EQ(routeDelivery.NextSequence(), 1U);
    routeDelivery.Queue(1U, "경로 내용", 10U);
    EXPECT_TRUE(routeDelivery.HasPendingTransmission());
    EXPECT_EQ(routeDelivery.PendingPayload(), "경로 내용");
    EXPECT_FALSE(routeDelivery.Acknowledge(1U));
    routeDelivery.RecordApplied(1U, 10U);
    EXPECT_TRUE(routeDelivery.Acknowledge(1U));
    EXPECT_FALSE(routeDelivery.HasPendingTransmission());

    routeDelivery.Queue(2U, "다음 경로", 20U);
    EXPECT_EQ(routeDelivery.NextSequence(), 3U);
    EXPECT_FALSE(routeDelivery.CanRetry(24U, 5U, 3U));
    EXPECT_TRUE(routeDelivery.CanRetry(25U, 5U, 3U));
    routeDelivery.RecordRetry(25U);
    // 시뮬레이션을 초기화해 Tick이 과거 기록보다 작아져도 바로 재전송하지 않는다.
    EXPECT_FALSE(routeDelivery.CanRetry(3U, 5U, 3U));
    EXPECT_FALSE(routeDelivery.CanRetry(29U, 5U, 3U));
    EXPECT_TRUE(routeDelivery.CanRetry(30U, 5U, 3U));

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
    simulator::DroneSimAppTestAccess::VerifyCompleteFleetMissionFinishesAndLands();
    EXPECT_NO_FATAL_FAILURE(Verify_PeerTelemetryPreservesStateAndRejectsInvalidValues());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetRouteMessagesRoundTripAndRejectInvalidValues());
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
