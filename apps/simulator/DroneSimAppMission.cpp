#include "DroneSimApp.h"
#include "FleetMissionActions.h"
#include "FleetMissionMessages.h"
#include "graphics/Window.h"
#include "simulation_core/DroneModel.h"
#include <glm/geometric.hpp>
#include <algorithm>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace dronesim::simulator
{
namespace
{
}

bool DroneSimApp::FleetIsReadyForMission() const
{
    if (CompleteFleetMissionIsActive() || fleetMission_.FormationTransit().IsActive()
        || reconTargets_.empty() || FleetReconnaissanceIsActive())
        return false;

    const auto droneIds = simulation_.DroneIds();
    return !droneIds.empty() && std::all_of(droneIds.begin(), droneIds.end(),
        [this](dronesim::simulation::DroneId id)
        {
            return simulation_.State(id).mode
                == dronesim::simulation::FlightMode::Grounded;
        });
}

bool DroneSimApp::FleetReconnaissanceCanStart() const
{
    return FleetIsReadyForMission();
}

bool DroneSimApp::CompleteFleetMissionCanStart() const
{
    return simulation_.DroneCount() == 4U && FleetIsReadyForMission();
}

bool DroneSimApp::CompleteFleetMissionIsActive() const
{
    return fleetMission_.CompleteMission().IsActive();
}

bool DroneSimApp::FleetReconnaissanceIsActive() const
{
    return fleetMission_.HasActiveReconnaissance();
}

bool DroneSimApp::StartCompleteFleetMission()
{
    if (!CompleteFleetMissionCanStart())
        return false;

    fleetMission_.CompleteMission().ClearHomePositions();
    for (const auto droneId : simulation_.DroneIds())
        fleetMission_.CompleteMission().RecordHomePosition(
            droneId, simulation_.State(droneId).positionMeters);

    if (!StartFormationTransit())
    {
        fleetMission_.CompleteMission().ClearHomePositions();
        return false;
    }

    fleetMission_.CompleteMission().SetStage(
        dronesim::mission::CompleteFleetMissionState::Stage::FormationTransit);
    return true;
}

std::string DroneSimApp::CompleteFleetMissionStatus() const
{
    return std::string{fleetMission_.CompleteMission().StatusText()};
}

bool DroneSimApp::StartFleetReconnaissance()
{
    if (!FleetReconnaissanceCanStart())
        return false;

    std::map<dronesim::simulation::DroneId, glm::dvec3> homePositions;
    for (const auto droneId : simulation_.DroneIds())
        homePositions.emplace(droneId, simulation_.State(droneId).positionMeters);
    return StartFleetReconnaissance(true, homePositions);
}

bool DroneSimApp::StartFleetReconnaissance(
    bool takeOffDrones,
    const std::map<dronesim::simulation::DroneId, glm::dvec3>& homePositions)
{
    using namespace dronesim::simulation;
    if (takeOffDrones)
        InitializeFleetLeadership();
    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    const DroneId leaderId = fleetMission_.LeaderId();
    if (droneIds.empty() || reconTargets_.empty()
        || (takeOffDrones && CompleteFleetMissionIsActive()))
        return false;

    std::vector<dronesim::mission::FleetReconnaissanceStart> starts;
    starts.reserve(droneIds.size());
    for (const DroneId droneId : droneIds)
    {
        const DroneState* localState = nullptr;
        const dronesim::mission::ReceivedPeerTelemetry* peerReport = nullptr;
        if (!takeOffDrones && droneId != leaderId)
        {
            peerReport = fleetMission_.TelemetryAtLeader().Fresh(
                droneId, simulation_.TickCount());
            if (peerReport == nullptr)
                return false;
        }
        else
        {
            localState = &simulation_.State(droneId);
        }

        const FlightMode mode = localState != nullptr
            ? localState->mode : peerReport->state.mode;
        const bool ready = takeOffDrones
            ? mode == FlightMode::Grounded
            : DroneModel::CanAcceptInFlightCommands(mode);
        if (!ready)
            return false;

        const glm::dvec3& startPosition = localState != nullptr
            ? localState->positionMeters : peerReport->state.positionMeters;
        const auto home = homePositions.find(droneId);
        starts.push_back({droneId, startPosition,
            home != homePositions.end() ? home->second : startPosition});
    }

    const dronesim::mission::FleetMissionPlanner planner{
        *routeEnvironment_, cameraTransitAltitudeMeters_};
    auto assignments = planner.PlanReconnaissance(
        starts, reconTargets_, takeOffDrones);
    if (!assignments)
        return false;

    if (!PrepareFleetNetwork(takeOffDrones))
        return false;

    if (takeOffDrones)
    {
        if (!dronesim::mission::TakeOffFleetDrones(simulation_, droneIds))
            return false;
    }
    for (auto& assignment : *assignments)
    {
        const DroneId droneId = assignment.droneId;
        auto& mission = assignment.mission;
        const dronesim::mission::FleetMissionLeg leg = mission.Leg();
        const std::size_t targetIndex = mission.targetIndex;
        std::vector<glm::dvec3> route = mission.TakeRouteForDelivery();
        mission.UpdateStatus(dronesim::mission::FleetDroneStatus::RoutePending);
        fleetMission_.SetAssignment(droneId, std::move(mission));
        if (!SendFleetRouteAssignment(
                droneId, leg, targetIndex, std::move(route)))
        {
            fleetMission_.ClearAssignments();
            dronesim::mission::LandActiveFleetDrones(simulation_, droneIds);
            return false;
        }
    }
    externalControl_ = true;
    missionControlOwnsInput_ = true;
    window_->SetTitle("DroneSim | 편대 정찰 임무 진행 중");
    return true;
}

bool DroneSimApp::FleetReturnCanStart() const
{
    if (!fleetMission_.HasAssignments() || CompleteFleetMissionIsActive()
        || fleetMission_.FormationTransit().IsActive())
        return false;

    return fleetMission_.AllDronesReadyToReturn(fleetDroneIds_, simulation_);
}

bool DroneSimApp::StartFleetReturn()
{
    if (!FleetReturnCanStart())
        return false;

    return StartFleetReturn(true);
}

bool DroneSimApp::StartFleetReturn(bool takeOffDrones)
{
    using namespace dronesim::simulation;

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    const DroneId leaderId = fleetMission_.LeaderId();
    if (droneIds.empty() || leaderId == 0U)
        return false;
    const dronesim::mission::FleetMissionPlanner planner{
        *routeEnvironment_, cameraTransitAltitudeMeters_};
    std::vector<std::vector<glm::dvec3>> returnRoutes;
    returnRoutes.reserve(droneIds.size());
    for (const auto droneId : droneIds)
    {
        const auto& mission = fleetMission_.Assignment(droneId);
        if (mission.IsLowBatteryReturn()
            && mission.Status()
                == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete)
        {
            returnRoutes.emplace_back();
            continue;
        }
        glm::dvec3 currentPosition{0.0};
        FlightMode mode = FlightMode::Grounded;
        if (takeOffDrones || droneId == leaderId)
        {
            const auto& state = simulation_.State(droneId);
            currentPosition = state.positionMeters;
            mode = state.mode;
        }
        else
        {
            const auto* report = fleetMission_.TelemetryAtLeader().Fresh(
                droneId, simulation_.TickCount());
            if (report == nullptr)
                return false;
            currentPosition = report->state.positionMeters;
            mode = report->state.mode;
        }
        const bool ready = takeOffDrones
            ? mode == FlightMode::Grounded
            : DroneModel::CanAcceptInFlightCommands(mode);
        if (!ready || mission.Leg() == dronesim::mission::FleetMissionLeg::Returning)
            return false;
        const auto route = planner.PlanReturnRoute(
            currentPosition, mission.startPositionMeters);
        if (!route)
            return false;
        returnRoutes.emplace_back(route->begin(), route->end());
    }

    if (takeOffDrones)
    {
        std::vector<DroneId> dronesToTakeOff;
        dronesToTakeOff.reserve(droneIds.size());
        for (const DroneId droneId : droneIds)
        {
            const auto& mission = fleetMission_.Assignment(droneId);
            if (mission.IsLowBatteryReturn()
                && mission.Status()
                    == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete)
                continue;
            if (simulation_.State(droneId).mode == FlightMode::Grounded)
                dronesToTakeOff.push_back(droneId);
        }
        if (!dronesim::mission::TakeOffFleetDrones(simulation_, dronesToTakeOff))
            return false;
    }

    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        auto& mission = fleetMission_.Assignment(droneIds[index]);
        if (mission.IsLowBatteryReturn()
            && mission.Status()
                == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete)
            continue;
        mission.BeginLeg(dronesim::mission::FleetMissionLeg::Returning);
        if (!SendFleetRouteAssignment(droneIds[index],
                dronesim::mission::FleetMissionLeg::Returning, mission.targetIndex,
                std::move(returnRoutes[index])))
            return false;
    }
    externalControl_ = true;
    missionControlOwnsInput_ = true;
    window_->SetTitle("DroneSim | 출발 지점으로 복귀 중");
    return true;
}

bool DroneSimApp::StartFleetRally()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;
    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.empty() || fleetMission_.LeaderId() == 0U
        || std::any_of(droneIds.begin(), droneIds.end(), [this](DroneId droneId)
            {
                return !fleetMission_.CompleteMission().HasHomePosition(droneId);
            }))
        return false;
    const dronesim::mission::FleetMissionPlanner planner{
        *routeEnvironment_, cameraTransitAltitudeMeters_};
    const DroneId leaderId = fleetMission_.LeaderId();
    std::vector<DroneId> rallyDroneIds;
    rallyDroneIds.reserve(droneIds.size());
    for (const DroneId droneId : droneIds)
    {
        const auto& mission = fleetMission_.Assignment(droneId);
        const bool alreadyReturnedForLowBattery = mission.IsLowBatteryReturn()
            && mission.Status()
                == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete;
        if (!alreadyReturnedForLowBattery)
            rallyDroneIds.push_back(droneId);
    }
    if (rallyDroneIds.empty())
    {
        if (!StartFleetReturn(false))
            return false;
        fleetMission_.CompleteMission().SetStage(
            dronesim::mission::CompleteFleetMissionState::Stage::Returning);
        return true;
    }

    glm::dvec3 centerMeters{0.0};
    for (const DroneId droneId : droneIds)
        centerMeters += fleetMission_.CompleteMission().HomePositionMeters(droneId);
    centerMeters /= static_cast<double>(droneIds.size());
    centerMeters.z += 12.0;
    const auto offsets = FormationOffsets(rallyDroneIds.size(), 3.0, 0.0);

    std::vector<std::vector<glm::dvec3>> rallyRoutes;
    rallyRoutes.reserve(rallyDroneIds.size());
    for (std::size_t index = 0; index < rallyDroneIds.size(); ++index)
    {
        const DroneId droneId = rallyDroneIds[index];
        glm::dvec3 currentPosition{0.0};
        FlightMode mode = FlightMode::Grounded;
        if (droneId == leaderId)
        {
            const DroneState& state = simulation_.State(droneId);
            currentPosition = state.positionMeters;
            mode = state.mode;
        }
        else
        {
            const auto* report = fleetMission_.TelemetryAtLeader().Fresh(
                droneId, simulation_.TickCount());
            if (report == nullptr
                || report->state.missionStatus
                    != dronesim::mission::FleetDroneStatus::TargetReached)
                return false;
            currentPosition = report->state.positionMeters;
            mode = report->state.mode;
        }
        if (mode != FlightMode::Grounded)
            return false;
        const glm::dvec3 rallyTarget{
            centerMeters.x + offsets[index].x,
            centerMeters.y + offsets[index].y,
            centerMeters.z};
        const auto route = planner.PlanTransitRoute(currentPosition, rallyTarget);
        if (!route)
            return false;
        rallyRoutes.emplace_back(route->begin(), route->end());
    }

    if (!dronesim::mission::TakeOffFleetDrones(simulation_, rallyDroneIds))
        return false;

    for (std::size_t index = 0; index < rallyDroneIds.size(); ++index)
    {
        const DroneId droneId = rallyDroneIds[index];
        auto& mission = fleetMission_.Assignment(droneId);
        mission.BeginLeg(dronesim::mission::FleetMissionLeg::Regrouping);
        if (!SendFleetRouteAssignment(droneId,
                dronesim::mission::FleetMissionLeg::Regrouping, mission.targetIndex,
                std::move(rallyRoutes[index])))
            return false;
    }
    fleetMission_.CompleteMission().SetStage(
        dronesim::mission::CompleteFleetMissionState::Stage::Regrouping);
    return true;
}

void DroneSimApp::AdvanceFleetReconnaissance()
{
    using DroneId = dronesim::simulation::DroneId;
    using Command = dronesim::simulation::DroneCommand;
    using FlightMode = dronesim::simulation::FlightMode;
    constexpr std::uint64_t routeDeliveryTimeoutTicks = 600U;
    const dronesim::mission::FleetMissionPlanner planner{
        *routeEnvironment_, cameraTransitAltitudeMeters_};

    for (const DroneId droneId : fleetDroneIds_)
    {
        auto assignment = fleetMission_.FindAssignment(droneId);
        if (assignment == fleetMission_.AssignmentsEnd())
            continue;
        auto& mission = assignment->second;
        if (mission.CanBeginLowBatteryReturn() && simulation_.HasBattery(droneId))
        {
            if (simulation_.TryBeginLowBatteryReturn(droneId))
            {
                const auto route = planner.PlanReturnRoute(
                    simulation_.State(droneId).positionMeters,
                    mission.startPositionMeters);
                bool returnStarted = route.has_value();
                if (returnStarted
                    && simulation_.State(droneId).mode == FlightMode::Grounded)
                    returnStarted = simulation_.Apply(droneId,
                        {Command::Kind::TakeOff})
                        == dronesim::simulation::CommandResult::Accepted;
                if (returnStarted)
                {
                    mission.BeginLowBatteryReturn();
                    returnStarted = SendFleetRouteAssignment(droneId,
                        dronesim::mission::FleetMissionLeg::Returning, mission.targetIndex,
                        std::vector<glm::dvec3>(route->begin(), route->end()));
                }
                if (!returnStarted)
                {
                    mission.UpdateStatus(
                        dronesim::mission::FleetDroneStatus::BatteryReturnRouteFailed);
                    mission.assignmentLeaseExpiresAtTick = simulation_.TickCount();
                    dronesim::mission::LandDroneIfNeeded(simulation_, droneId);
                }
                continue;
            }
        }

        if (mission.IsLegFinished())
            continue;

        if (!mission.HasReceivedRoute())
        {
            if (simulation_.TickCount() >= mission.RouteDelivery().SentAtTick()
                    + routeDeliveryTimeoutTicks)
                mission.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
            continue;
        }

        const auto& state = simulation_.State(droneId);
        if (state.mode == FlightMode::Grounded)
        {
            const auto groundedStatus = mission.Status();
            if (groundedStatus == dronesim::mission::FleetDroneStatus::TakeoffReady
                || groundedStatus == dronesim::mission::FleetDroneStatus::ReturnPreparing
                || groundedStatus == dronesim::mission::FleetDroneStatus::RegroupPreparing)
                continue;

            if (mission.Leg() == dronesim::mission::FleetMissionLeg::Regrouping)
            {
                mission.UpdateStatus(mission.StatusAfterLanding(
                    state.positionMeters, state.positionMeters));
                continue;
            }
            const glm::dvec3& destination = mission.Leg() == dronesim::mission::FleetMissionLeg::Returning
                ? mission.startPositionMeters
                : reconTargets_[mission.targetIndex].approachPositionMeters;
            mission.UpdateStatus(mission.StatusAfterLanding(
                state.positionMeters, destination));
            continue;
        }
        if (state.mode == FlightMode::TakingOff || state.mode == FlightMode::Landing)
            continue;

        if (!mission.HasCommandedWaypoint())
        {
            mission.BeginWaypointCommand();
        }
        else if (!mission.HasRemainingWaypoints())
        {
            if (mission.Leg() == dronesim::mission::FleetMissionLeg::Regrouping)
            {
                mission.UpdateStatus(dronesim::mission::FleetDroneStatus::RegroupComplete);
                continue;
            }
            if (dronesim::mission::TryLandDrone(simulation_, droneId))
            {
                if (mission.Leg() == dronesim::mission::FleetMissionLeg::Returning)
                    mission.UpdateStatus(mission.IsLowBatteryReturn()
                        ? dronesim::mission::FleetDroneStatus::LowBatteryLanding
                        : dronesim::mission::FleetDroneStatus::ReturnLanding);
                else
                    mission.UpdateStatus(dronesim::mission::FleetDroneStatus::Landing);
            }
            else
                mission.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
            continue;
        }
        else if (mission.AdvanceWaypointIfReached(state.positionMeters, 2.0))
        {
            if (!mission.HasRemainingWaypoints())
                continue;
        }

        if (const auto* nextWaypoint = mission.CurrentWaypoint())
        {
            Command waypoint;
            waypoint.kind = Command::Kind::SetWaypoint;
            waypoint.vector = *nextWaypoint;
            if (simulation_.Apply(droneId, waypoint)
                != dronesim::simulation::CommandResult::Accepted)
                mission.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
        }
    }
}

void DroneSimApp::AdvanceCompleteFleetMission()
{
    using Stage = dronesim::mission::CompleteFleetMissionState::Stage;
    using DroneId = dronesim::simulation::DroneId;
    const Stage currentStage = fleetMission_.CompleteMission().CurrentStage();
    if (currentStage == Stage::Idle
        || currentStage == Stage::Completed
        || currentStage == Stage::Failed)
        return;

    if (currentStage == Stage::FormationTransit)
    {
        if (fleetMission_.FormationTransit().IsActive())
            return;
        if (fleetMission_.FormationTransit().CurrentPhase()
            != dronesim::mission::FormationTransitState::Phase::Arrived)
        {
            FailCompleteFleetMission("M01 편대 이동이 끝나지 않아 전체 임무를 중단했습니다.");
            return;
        }
        if (!StartFleetReconnaissance(false, fleetMission_.CompleteMission().HomePositions()))
        {
            FailCompleteFleetMission("M02 정찰 경로를 만들지 못해 기체를 착륙시킵니다.");
            return;
        }
        fleetMission_.CompleteMission().SetStage(Stage::Reconnaissance);
        return;
    }

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.empty())
        return;
    std::vector<std::optional<dronesim::mission::FleetMissionReport>> reports;
    reports.reserve(droneIds.size());
    for (const auto droneId : droneIds)
    {
        const auto assignment = fleetMission_.FindAssignment(droneId);
        if (assignment == fleetMission_.AssignmentsEnd())
        {
            if (currentStage == Stage::Returning)
            {
                fleetMission_.CompleteMission().Fail("복귀에 실패한 드론이 있습니다.");
            }
            else
            {
                FailCompleteFleetMission(currentStage == Stage::Reconnaissance
                    ? "드론별 정찰 상태를 찾지 못해 임무를 중단했습니다."
                    : "모든 기체가 집결하지 못해 임무를 중단했습니다.");
            }
            return;
        }

        const auto report = fleetMission_.CurrentReport(
            droneId, assignment->second, simulation_, currentStage);
        reports.push_back(report);
    }

    const auto missionLeg = currentStage == Stage::Reconnaissance
        ? dronesim::mission::FleetMissionLeg::Reconnaissance
        : currentStage == Stage::Regrouping
            ? dronesim::mission::FleetMissionLeg::Regrouping
            : dronesim::mission::FleetMissionLeg::Returning;
    const auto decision = dronesim::mission::EvaluateFleetMissionProgress(missionLeg, reports);
    if (decision.progress == dronesim::mission::FleetMissionProgress::Failed)
    {
        if (currentStage == Stage::Returning)
        {
            fleetMission_.CompleteMission().Fail(std::string{decision.failureReason});
        }
        else
        {
            FailCompleteFleetMission(std::string{decision.failureReason});
        }
        return;
    }
    if (decision.progress == dronesim::mission::FleetMissionProgress::Waiting)
    {
        fleetMission_.CompleteMission().WaitForReports();
        return;
    }

    if (currentStage == Stage::Reconnaissance)
    {
        if (!StartFleetRally())
        {
            FailCompleteFleetMission("공통 집결 경로를 만들지 못해 임무를 중단했습니다.");
            return;
        }
        return;
    }

    if (currentStage == Stage::Regrouping)
    {
        if (!StartFleetReturn(false))
        {
            FailCompleteFleetMission("집결지에서 기지까지 복귀 경로를 만들지 못했습니다.");
            return;
        }
        fleetMission_.CompleteMission().SetStage(Stage::Returning);
        return;
    }

    if (currentStage == Stage::Returning)
    {
        fleetMission_.CompleteMission().SetStage(Stage::Completed);
        return;
    }
}

void DroneSimApp::FailCompleteFleetMission(const std::string& reason)
{
    fleetMission_.CompleteMission().Fail(reason);
    if (fleetMission_.FormationTransit().IsActive())
        fleetMission_.FormationTransit().Stop();
    const auto droneIds = simulation_.DroneIds();
    dronesim::mission::LandActiveFleetDrones(simulation_, droneIds);
}

bool DroneSimApp::StartFormationTransit()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;

    if (fleetMission_.FormationTransit().IsActive() || reconTargets_.empty())
        return false;

    InitializeFleetLeadership();

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.empty())
        return false;
    if (droneIds.size() != 4U || std::any_of(droneIds.begin(), droneIds.end(),
        [this](DroneId droneId)
        {
            return simulation_.State(droneId).mode != FlightMode::Grounded;
        }))
        return false;

    const glm::dvec2 stagingPosition =
        trainingRangeScene_->FormationStagingPositionMeters();
    const double stagingAltitude = std::max(
        terrainGrid_.HeightAtLocalMeters(stagingPosition) + 15.0,
        cameraTransitAltitudeMeters_);
    const glm::dvec3 destination{stagingPosition, stagingAltitude};
    const auto route = flight_planning::PlanFormationRoute(*routeEnvironment_,
        simulation_.State(droneIds.front()).positionMeters,
        destination, droneIds.size(), 1.5, 0.75, 1.0,
        cameraTransitAltitudeMeters_);
    if (!route)
    {
        fleetMission_.FormationTransit().Fail("목적지까지 편대 경로를 찾지 못했습니다.");
        return false;
    }

    if (!PrepareFleetNetwork(true))
    {
        fleetMission_.FormationTransit().Fail("가상 통신망을 준비하지 못했습니다.");
        return false;
    }

    if (!dronesim::mission::TakeOffFleetDrones(simulation_, droneIds))
    {
        fleetMission_.FormationTransit().Fail("드론 이륙 명령을 적용하지 못했습니다.");
        return false;
    }

    fleetMission_.FormationTransit().SetRoute(*route);
    fleetMission_.FormationTransit().Start("출발 준비 · V자 대형 · "
        + reconTargets_.back().displayName);
    externalControl_ = true;
    missionControlOwnsInput_ = true;
    pendingControllerAction_.reset();
    pendingPilotCommand_.reset();
#ifdef __EMSCRIPTEN__
    webPilotAxes_.reset();
#endif
    if (window_)
        window_->SetTitle("DroneSim | 편대 이동 임무 진행 중");
    return true;
}

bool DroneSimApp::PrepareFleetNetwork(bool reset)
{
    if (formationNetwork_ && !reset)
        return true;

    InitializeFleetLeadership();

    fleetMission_.ResetTelemetry();

    communication::VirtualMeshSettings settings;
    settings.maximumLinkRangeMeters = 130.0;
    const auto droneIds = simulation_.DroneIds();
    formationNetwork_ = std::make_unique<
        communication::VirtualMeshNetwork>(droneSocketTransport_, settings);

    formationRadioObstacles_.clear();
    const auto& buildings = trainingRangeScene_->BuildingObstacles();
    formationRadioObstacles_.reserve(buildings.size());
    for (const dronesim::flight_planning::BuildingObstacle& building : buildings)
    {
        if (building.outlineMeters.size() < 3U || building.heightMeters <= 0.0)
            continue;
        const double groundHeight = building.groundElevationsMeters.empty()
            ? 0.0
            : *std::min_element(building.groundElevationsMeters.begin(),
                building.groundElevationsMeters.end());
        formationRadioObstacles_.push_back({
            building.outlineMeters, groundHeight, groundHeight + building.heightMeters});
    }

    std::vector<communication::NetworkNodeSnapshot> nodes;
    nodes.reserve(droneIds.size());
    for (const auto droneId : droneIds)
    {
        nodes.push_back({droneId, simulation_.State(droneId).positionMeters});
    }
    formationNetwork_->UpdateNodes(nodes, formationRadioObstacles_);
    fleetMission_.FormationTransit().RecordNetworkNodesUpdated(simulation_.TickCount());
    return !nodes.empty();
}

void DroneSimApp::InitializeFleetLeadership()
{
    fleetDroneIds_ = simulation_.DroneIds();
    fleetMission_.ClearAssignments();
    fleetMission_.SetLeaderId(fleetDroneIds_.empty() ? 0U : fleetDroneIds_.front());
}

bool DroneSimApp::SendFleetRouteAssignment(
    dronesim::simulation::DroneId droneId,
    dronesim::mission::FleetMissionLeg leg,
    std::size_t targetIndex,
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    if (!formationNetwork_ || reconTargets_.empty()
        || targetIndex >= reconTargets_.size())
        return false;

    auto mission = fleetMission_.FindAssignment(droneId);
    if (mission == fleetMission_.AssignmentsEnd() || fleetDroneIds_.empty())
        return false;

    const std::uint64_t assignmentVersion = fleetMission_.TakeNextAssignmentVersion();
    std::uint64_t leaseExpiresAtTick = 0U;
    if (leg == dronesim::mission::FleetMissionLeg::Reconnaissance)
    {
        glm::dvec3 startPositionMeters = simulation_.State(droneId).positionMeters;
        if (droneId != fleetMission_.LeaderId())
        {
            if (const auto* report = fleetMission_.TelemetryAtLeader().Fresh(
                    droneId, simulation_.TickCount()))
                startPositionMeters = report->state.positionMeters;
        }
        leaseExpiresAtTick = dronesim::mission::EstimateMissionLeaseExpiry(
            simulation_.TickCount(), startPositionMeters, routeWaypointsMeters);
    }
    const std::uint32_t sequence = mission->second.RouteDelivery().NextSequence();
    const std::string payload = dronesim::mission::EncodeFleetRoute(
        leg, targetIndex, assignmentVersion, leaseExpiresAtTick, routeWaypointsMeters);
    if (droneId == fleetMission_.LeaderId())
    {
        ApplyFleetRouteAssignment(droneId, leg, targetIndex,
            std::move(routeWaypointsMeters), sequence, assignmentVersion,
            leaseExpiresAtTick);
        mission->second.RouteDelivery().ClearPending();
        return true;
    }

    const auto messageId = formationNetwork_->Send(
        fleetMission_.LeaderId(), droneId,
        sequence, "fleet_route", payload);
    if (!messageId)
        return false;

    mission->second.assignmentVersion = assignmentVersion;
    mission->second.assignmentLeaseExpiresAtTick = leaseExpiresAtTick;
    mission->second.RouteDelivery().Queue(
        sequence, payload, simulation_.TickCount());
    mission->second.WaitForRoute();
    mission->second.UpdateStatus(dronesim::mission::FleetDroneStatus::RoutePending);
    return true;
}

void DroneSimApp::ApplyFleetRouteAssignment(
    dronesim::simulation::DroneId droneId,
    dronesim::mission::FleetMissionLeg leg,
    std::size_t targetIndex,
    std::vector<glm::dvec3> routeWaypointsMeters,
    std::uint32_t sequence,
    std::uint64_t assignmentVersion,
    std::uint64_t assignmentLeaseExpiresAtTick)
{
    auto mission = fleetMission_.FindAssignment(droneId);
    if (mission == fleetMission_.AssignmentsEnd())
        return;

    mission->second.BeginLeg(leg);
    mission->second.targetIndex = targetIndex;
    mission->second.RouteDelivery().RecordApplied(sequence, simulation_.TickCount());
    mission->second.assignmentVersion = assignmentVersion;
    mission->second.assignmentLeaseExpiresAtTick = assignmentLeaseExpiresAtTick;
    mission->second.ReceiveRoute(std::move(routeWaypointsMeters));

    auto mode = simulation_.State(droneId).mode;
    if (leg == dronesim::mission::FleetMissionLeg::Reconnaissance
        && mode == dronesim::simulation::FlightMode::Grounded)
    {
        if (simulation_.Apply(droneId,
                {dronesim::simulation::DroneCommand::Kind::TakeOff})
            != dronesim::simulation::CommandResult::Accepted)
        {
            mission->second.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
            return;
        }
        mode = simulation_.State(droneId).mode;
    }
    const bool waitingForTakeoff = mode == dronesim::simulation::FlightMode::Grounded
        || mode == dronesim::simulation::FlightMode::TakingOff;
    switch (leg)
    {
    case dronesim::mission::FleetMissionLeg::Reconnaissance:
        mission->second.UpdateStatus(waitingForTakeoff
            ? dronesim::mission::FleetDroneStatus::TakeoffReady
            : dronesim::mission::FleetDroneStatus::Reconnaissance);
        break;
    case dronesim::mission::FleetMissionLeg::Regrouping:
        mission->second.UpdateStatus(waitingForTakeoff
            ? dronesim::mission::FleetDroneStatus::RegroupPreparing
            : dronesim::mission::FleetDroneStatus::Regrouping);
        break;
    case dronesim::mission::FleetMissionLeg::Returning:
        mission->second.UpdateStatus(waitingForTakeoff
            ? dronesim::mission::FleetDroneStatus::ReturnPreparing
            : dronesim::mission::FleetDroneStatus::Returning);
        break;
    }
}

void DroneSimApp::AdvanceFleetNetwork()
{
    if (!formationNetwork_ || (!fleetMission_.FormationTransit().IsActive()
        && !CompleteFleetMissionIsActive() && !FleetReconnaissanceIsActive()))
        return;

    const std::uint64_t currentTick = simulation_.TickCount();
    RefreshFleetNetworkNodes(currentTick);
    // 먼저 지연 시간이 지난 패킷을 꺼내 처리한 뒤, 새 상태와 경로를 전송한다.
    formationNetwork_->Advance(currentTick);
    ProcessDeliveredFleetMessages(currentTick);
    SendFleetTelemetry(currentTick);
    RetryPendingFleetRoutes(currentTick);
}

void DroneSimApp::RefreshFleetNetworkNodes(std::uint64_t currentTick)
{
    if (!formationNetwork_)
        return;

    using dronesim::simulation::DroneId;
    constexpr std::uint64_t networkPositionRefreshTicks = 6U;
    // 링크 판정은 매 6 Tick마다 갱신한다. 60Hz 기준 0.1초마다 실제 드론 위치를 반영한다.
    const bool refreshNetworkPositions =
        fleetMission_.FormationTransit().ShouldRefreshNetworkNodes(
            currentTick, networkPositionRefreshTicks);
    if (refreshNetworkPositions)
    {
        std::vector<communication::NetworkNodeSnapshot> nodes;
        const auto allDroneIds = simulation_.DroneIds();
        nodes.reserve(allDroneIds.size());
        for (const DroneId droneId : allDroneIds)
        {
            const auto& drone = simulation_.State(droneId);
            nodes.push_back({droneId, drone.positionMeters});
        }
        formationNetwork_->UpdateNodes(nodes, formationRadioObstacles_);
        fleetMission_.FormationTransit().RecordNetworkNodesUpdated(currentTick);
    }

}

void DroneSimApp::ProcessDeliveredFleetMessages(std::uint64_t currentTick)
{
    using dronesim::simulation::DroneCommand;
    using dronesim::simulation::DroneId;
    const DroneId leaderId = fleetMission_.LeaderId();

    for (const auto& delivered : formationNetwork_->DeliveredMessages())
    {
        const auto& message = delivered.message;
        if (message.kind == "drone_telemetry"
            && message.destination == leaderId && message.source != leaderId)
        {
            if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.source)
                == fleetDroneIds_.end())
                continue;
            const auto telemetry = dronesim::mission::DecodePeerTelemetry(message.payload);
            if (telemetry)
                static_cast<void>(fleetMission_.TelemetryAtLeader().Record(message.source,
                    *telemetry, delivered.deliveredAtTick, message.sequence));
            continue;
        }

        if (message.kind == "fleet_route_ack"
            && message.destination == leaderId)
        {
            const auto mission = fleetMission_.FindAssignment(message.source);
            if (mission != fleetMission_.AssignmentsEnd())
                static_cast<void>(mission->second.RouteDelivery().Acknowledge(
                    message.sequence));
            continue;
        }

        if (message.source != leaderId)
            continue;

        if (message.kind == "formation_waypoint" && fleetMission_.FormationTransit().IsActive())
        {
            if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.destination)
                == fleetDroneIds_.end())
                continue;
            const auto waypoint = dronesim::mission::DecodeWaypoint(message.payload);
            if (!waypoint)
                continue;
            if (!fleetMission_.FormationTransit().IsNewAppliedSequence(
                    message.destination, message.sequence))
                continue;

            DroneCommand command;
            command.kind = DroneCommand::Kind::SetWaypoint;
            command.vector = *waypoint;
            if (simulation_.Apply(message.destination, command)
                != dronesim::simulation::CommandResult::Accepted)
            {
                fleetMission_.FormationTransit().Fail("통신으로 받은 편대 목표를 적용하지 못했습니다.");
                return;
            }
            fleetMission_.FormationTransit().RecordAppliedSequence(
                message.destination, message.sequence);
            continue;
        }

        if (message.kind != "fleet_route")
            continue;

        if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.destination)
            == fleetDroneIds_.end())
            continue;

        auto mission = fleetMission_.FindAssignment(message.destination);
        const auto route = dronesim::mission::DecodeFleetRoute(message.payload);
        if (mission == fleetMission_.AssignmentsEnd() || !route
            || route->targetIndex >= reconTargets_.size())
        {
            if (mission != fleetMission_.AssignmentsEnd())
                mission->second.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
            continue;
        }

        if (route->assignmentLeaseExpiresAtTick != 0U
            && currentTick >= route->assignmentLeaseExpiresAtTick)
        {
            dronesim::mission::LandDroneIfNeeded(simulation_, message.destination);
            mission->second.UpdateStatus(
                dronesim::mission::FleetDroneStatus::AssignmentExpiredLanding);
            continue;
        }

        if (message.sequence == mission->second.RouteDelivery().AppliedSequence())
        {
            // 같은 경로가 다시 오면 비행 상태를 초기화하지 않고 확인 응답만 다시 보낸다.
            static_cast<void>(formationNetwork_->Send(
                message.destination, leaderId,
                message.sequence, "fleet_route_ack", {}));
            continue;
        }
        if (message.sequence != mission->second.RouteDelivery().PendingSequence()
            || message.sequence < mission->second.RouteDelivery().AppliedSequence())
            continue;

        ApplyFleetRouteAssignment(message.destination,
            route->leg, route->targetIndex,
            route->waypointsMeters, message.sequence, route->assignmentVersion,
            route->assignmentLeaseExpiresAtTick);
        static_cast<void>(formationNetwork_->Send(
            message.destination, leaderId,
            message.sequence, "fleet_route_ack", {}));
    }
}

void DroneSimApp::SendFleetTelemetry(std::uint64_t currentTick)
{
    using dronesim::simulation::DroneId;
    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    const DroneId leaderId = fleetMission_.LeaderId();

    constexpr std::uint64_t telemetryIntervalTicks = 6U;

    // 리더를 제외한 드론의 상태만 보내고, 직전 전송 후 6 Tick이 지나야 다음 상태를 보낸다.
    for (const DroneId droneId : droneIds)
    {
        if (droneId == leaderId)
            continue;
        if (!fleetMission_.ShouldSendTelemetry(
                droneId, currentTick, telemetryIntervalTicks))
            continue;

        const auto mission = fleetMission_.FindAssignment(droneId);
        const std::uint8_t missionLeg = mission == fleetMission_.AssignmentsEnd()
            ? 0U : static_cast<std::uint8_t>(mission->second.Leg());
        const dronesim::mission::FleetDroneStatus missionStatus =
            mission == fleetMission_.AssignmentsEnd()
            ? dronesim::mission::FleetDroneStatus::Other : mission->second.Status();
        const std::uint64_t assignmentVersion = mission == fleetMission_.AssignmentsEnd()
            ? 0U : mission->second.assignmentVersion;
        const auto& state = simulation_.State(droneId);
        static_cast<void>(formationNetwork_->Send(
            droneId, leaderId,
            fleetMission_.TakeNextTelemetrySequence(), "drone_telemetry",
            dronesim::mission::EncodePeerTelemetry(state.positionMeters, state.mode,
                missionLeg, missionStatus,
                assignmentVersion)));
        fleetMission_.RecordTelemetryAttempt(droneId, currentTick);
    }
}

void DroneSimApp::RetryPendingFleetRoutes(std::uint64_t currentTick)
{
    using dronesim::simulation::DroneId;
    const DroneId leaderId = fleetMission_.LeaderId();

    constexpr std::uint64_t routeRetryIntervalTicks = 60U;
    constexpr std::uint32_t maximumRouteTransmissions = 10U;
    // 확인 응답이 없는 경로만 1초 간격으로 다시 보내며, 10회를 넘기지 않는다.
    for (const DroneId droneId : fleetDroneIds_)
    {
        auto assignment = fleetMission_.FindAssignment(droneId);
        if (assignment == fleetMission_.AssignmentsEnd())
            continue;
        auto& mission = assignment->second;
        if (droneId == leaderId
            || !mission.RouteDelivery().CanRetry(
                currentTick, routeRetryIntervalTicks, maximumRouteTransmissions))
            continue;

        if (formationNetwork_->Send(leaderId, droneId,
                mission.RouteDelivery().PendingSequence(), "fleet_route",
                mission.RouteDelivery().PendingPayload()))
            mission.RouteDelivery().RecordRetry(currentTick);
    }
}

void DroneSimApp::AdvanceFormationTransit()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;
    if (!fleetMission_.FormationTransit().IsActive())
        return;

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.size() < 2U
        || fleetMission_.FormationTransit().Route().leaderWaypointsMeters.empty())
    {
        fleetMission_.FormationTransit().Fail("편대 구성을 유지할 수 없어 임무를 멈췄습니다.");
        return;
    }

    const DroneId leaderId = fleetMission_.LeaderId();
    const DroneState& leader = simulation_.State(leaderId);
    if (leader.mode == FlightMode::Grounded || leader.mode == FlightMode::TakingOff)
        return;
    if (leader.mode == FlightMode::Landing)
    {
        fleetMission_.FormationTransit().Fail("리더가 착륙해 편대 이동을 멈췄습니다.");
        return;
    }

    constexpr std::uint64_t peerTelemetryTimeoutTicks = 30U;
    constexpr std::uint64_t peerTelemetryRecoveryTimeoutTicks = 180U;
    for (std::size_t index = 1U; index < droneIds.size(); ++index)
    {
        const auto* peer = fleetMission_.TelemetryAtLeader().Latest(droneIds[index]);
        if (peer == nullptr)
        {
            fleetMission_.FormationTransit().Fail("통신 끊김 · 편대 안전 착륙");
            if (fleetMission_.CompleteMission().CurrentStage()
                == dronesim::mission::CompleteFleetMissionState::Stage::FormationTransit)
            {
                FailCompleteFleetMission(
                    "추종 드론 상태 보고가 끊겨 편대를 안전 착륙시킵니다.");
            }
            else
            {
                dronesim::mission::LandActiveFleetDrones(simulation_, droneIds);
            }
            return;
        }

        const std::uint64_t reportAgeTicks = simulation_.TickCount()
            - peer->receivedAtTick;
        if (reportAgeTicks <= peerTelemetryTimeoutTicks)
            continue;

        if (reportAgeTicks <= peerTelemetryTimeoutTicks
                + peerTelemetryRecoveryTimeoutTicks)
        {
            // 편대원 보고가 잠깐 끊기면 리더를 멈춰 연결 회복을 기다린다.
            DroneCommand holdLeader;
            holdLeader.kind = DroneCommand::Kind::SetWaypoint;
            holdLeader.vector = leader.positionMeters;
            static_cast<void>(simulation_.Apply(leaderId, holdLeader));
            fleetMission_.FormationTransit().UpdateStatus(
                "추종 드론 통신 회복 대기 · 편대 이동 일시 정지");
            return;
        }

        fleetMission_.FormationTransit().Fail("통신 끊김 · 편대 안전 착륙");
        if (fleetMission_.CompleteMission().CurrentStage()
            == dronesim::mission::CompleteFleetMissionState::Stage::FormationTransit)
        {
            FailCompleteFleetMission(
                "추종 드론과 통신이 복구되지 않아 편대를 안전 착륙시킵니다.");
        }
        else
        {
            dronesim::mission::LandActiveFleetDrones(simulation_, droneIds);
        }
        return;
    }

    const auto& formationRoute = fleetMission_.FormationTransit().Route();
    const auto& route = formationRoute.leaderWaypointsMeters;
    fleetMission_.FormationTransit().AdvanceWaypointIfReached(leader.positionMeters, 3.0);

    const std::size_t nextIndex = fleetMission_.FormationTransit().WaypointIndex();
    glm::dvec2 direction{0.0, 0.0};
    if (nextIndex + 1U < route.size())
        direction = {route[nextIndex + 1U].x - route[nextIndex].x,
            route[nextIndex + 1U].y - route[nextIndex].y};
    else if (nextIndex > 0U)
        direction = {route[nextIndex].x - route[nextIndex - 1U].x,
            route[nextIndex].y - route[nextIndex - 1U].y};
    else if (!route.empty())
        direction = {route[nextIndex].x - formationRoute.leaderStartMeters.x,
            route[nextIndex].y - formationRoute.leaderStartMeters.y};
    const double heading = glm::length(direction) > 1.0e-6
        ? std::atan2(direction.y, direction.x) : leader.yawRadians;
    const auto offsets = FormationOffsets(
        droneIds.size(), formationRoute.spacingMeters, heading);

    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const DroneId droneId = droneIds[index];
        const auto* peerState = index == 0U
            ? nullptr
            : fleetMission_.TelemetryAtLeader().Latest(droneId);
        if (index != 0U && peerState == nullptr)
        {
            fleetMission_.FormationTransit().Fail("추종 드론 상태 보고를 기다리는 중입니다.");
            return;
        }
        const FlightMode mode = index == 0U ? leader.mode : peerState->state.mode;
        if (mode == FlightMode::Grounded || mode == FlightMode::TakingOff)
            continue;
        if (mode == FlightMode::Landing)
        {
            fleetMission_.FormationTransit().Fail("추종 드론이 착륙해 편대 이동을 멈췄습니다.");
            return;
        }

        glm::dvec3 target = leader.positionMeters;
        if (index == 0U)
            target = route[nextIndex];
        else
        {
            target.x += offsets[index].x;
            target.y += offsets[index].y;
        }
        DroneCommand command;
        command.kind = DroneCommand::Kind::SetWaypoint;
        command.vector = target;
        if (index == 0U)
        {
            if (simulation_.Apply(droneId, command) != CommandResult::Accepted)
            {
                fleetMission_.FormationTransit().Fail("리더 목표를 적용하지 못해 임무를 멈췄습니다.");
                return;
            }
            continue;
        }

        constexpr std::uint64_t targetRefreshTicks = 6U;
        if (fleetMission_.FormationTransit().ShouldSendFollowerTarget(
                droneId, target, simulation_.TickCount(), 0.75, targetRefreshTicks))
        {
            const auto messageId = formationNetwork_->Send(
                leaderId, droneId,
                fleetMission_.FormationTransit().TakeNextMessageSequence(), "formation_waypoint",
                dronesim::mission::EncodeWaypoint(target));
            if (!messageId)
            {
                fleetMission_.FormationTransit().Fail("추종 드론에게 편대 목표를 보내지 못했습니다.");
                return;
            }
            fleetMission_.FormationTransit().RecordFollowerTargetSent(
                droneId, target, simulation_.TickCount());
        }
    }

    const bool leaderArrived = fleetMission_.FormationTransit().WaypointIndex() + 1U == route.size()
        && glm::distance(leader.positionMeters, route.back()) <= 3.0;
    bool followersArrived = true;
    for (std::size_t index = 1U; index < droneIds.size(); ++index)
    {
        const glm::dvec3 target{leader.positionMeters.x + offsets[index].x,
            leader.positionMeters.y + offsets[index].y, leader.positionMeters.z};
        const auto* peer = fleetMission_.TelemetryAtLeader().Latest(droneIds[index]);
        if (peer == nullptr)
        {
            followersArrived = false;
            break;
        }
        if (glm::distance(peer->state.positionMeters, target) > 4.0)
        {
            followersArrived = false;
            break;
        }
    }
    if (leaderArrived && followersArrived)
    {
        fleetMission_.FormationTransit().Arrive("목적지 도착 · 편대 유지 중");
        return;
    }
    fleetMission_.FormationTransit().UpdateStatus("이동 중 · 경유점 "
        + std::to_string(fleetMission_.FormationTransit().WaypointIndex() + 1U)
        + "/" + std::to_string(route.size()));
}

}
