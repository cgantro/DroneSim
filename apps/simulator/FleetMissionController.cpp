#include "FleetMissionController.h"
#include "FleetMissionActions.h"
#include "FleetMissionMessages.h"
#include "simulation_core/DroneModel.h"
#include <glm/geometric.hpp>
#include <algorithm>

namespace dronesim::simulator
{
namespace
{
struct PlannedFleetRoute
{
    dronesim::simulation::DroneId droneId = 0;
    std::vector<glm::dvec3> waypointsMeters;
};
}

FleetMissionController::FleetMissionController(
    simulation::SimulationCore& simulation,
    const terrain::TerrainGrid& terrain,
    const scene::TrainingRangeScene& trainingScene)
    : simulation_(simulation),
      trainingScene_(trainingScene),
      missionEnvironment_(terrain, trainingScene),
      fleetCommunication_(simulation_, trainingScene_, missionEnvironment_, fleetMission_),
      formationFlight_(simulation_, fleetMission_, fleetCommunication_),
      droneMissionExecutor_(simulation_, missionEnvironment_, fleetMission_, fleetCommunication_)
{
}

void FleetMissionController::AdvanceTick()
{
    // 먼저 통신으로 도착한 경로와 상태를 반영해야 같은 Tick의 비행 단계가 새 목표를 사용한다.
    fleetCommunication_.AdvanceTick();
    droneMissionExecutor_.Advance();
    AdvanceFormationTransit();
    // 개별 단계가 갱신된 뒤 전체 임무가 다음 단계로 넘어갈지 판단한다.
    AdvanceCompleteFleetMission();
}

const mission::FleetMissionState& FleetMissionController::State() const noexcept
{
    return fleetMission_;
}

const FleetMissionEnvironment& FleetMissionController::Environment() const noexcept
{
    return missionEnvironment_;
}

const FleetNetworkSession& FleetMissionController::NetworkSession() const noexcept
{
    return fleetCommunication_.NetworkSession();
}

void FleetMissionController::SetRadioEnabled(
    simulation::DroneId droneId, bool enabled)
{
    fleetCommunication_.SetRadioEnabled(droneId, enabled);
}

void FleetMissionController::InitializeFleetLeadership()
{
    fleetCommunication_.RefreshDroneRoster();
    fleetMission_.ClearAssignments();
    const auto& droneIds = fleetCommunication_.DroneIds();
    fleetMission_.SetLeaderId(droneIds.empty() ? 0U : droneIds.front());
}

void FleetMissionController::Abort(std::string reason)
{
    FailCompleteFleetMission(reason);
}

bool FleetMissionController::FleetIsReadyForMission() const
{
    if (CompleteFleetMissionIsActive() || fleetMission_.FormationTransit().IsActive()
        || missionEnvironment_.Targets().empty() || FleetReconnaissanceIsActive())
        return false;

    const auto droneIds = simulation_.DroneIds();
    return !droneIds.empty() && std::all_of(droneIds.begin(), droneIds.end(),
        [this](dronesim::simulation::DroneId id)
        {
            return simulation_.State(id).mode
                == dronesim::simulation::FlightMode::Grounded;
        });
}

bool FleetMissionController::FleetReconnaissanceCanStart() const
{
    return FleetIsReadyForMission();
}

bool FleetMissionController::CompleteFleetMissionCanStart() const
{
    return simulation_.DroneCount() == 4U && FleetIsReadyForMission();
}

bool FleetMissionController::CompleteFleetMissionIsActive() const
{
    return fleetMission_.CompleteMission().IsActive();
}

bool FleetMissionController::FleetReconnaissanceIsActive() const
{
    return fleetMission_.HasActiveReconnaissance();
}

bool FleetMissionController::StartCompleteFleetMission()
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

std::string FleetMissionController::CompleteFleetMissionStatus() const
{
    return std::string{fleetMission_.CompleteMission().StatusText()};
}

bool FleetMissionController::StartFleetReconnaissance()
{
    if (!FleetReconnaissanceCanStart())
        return false;

    std::map<dronesim::simulation::DroneId, glm::dvec3> homePositions;
    for (const auto droneId : simulation_.DroneIds())
        homePositions.emplace(droneId, simulation_.State(droneId).positionMeters);
    return StartFleetReconnaissance(
        dronesim::mission::FleetMissionStartMode::TakeOffFromGround,
        homePositions);
}

bool FleetMissionController::StartFleetReconnaissance(
    dronesim::mission::FleetMissionStartMode startMode,
    const std::map<dronesim::simulation::DroneId, glm::dvec3>& homePositions)
{
    using namespace dronesim::simulation;
    const bool startsOnGround = startMode
        == dronesim::mission::FleetMissionStartMode::TakeOffFromGround;
    if (startsOnGround)
        InitializeFleetLeadership();
    const std::vector<DroneId>& droneIds = fleetCommunication_.DroneIds();
    if (droneIds.empty() || missionEnvironment_.Targets().empty()
        || (startsOnGround && CompleteFleetMissionIsActive()))
        return false;

    std::vector<dronesim::mission::FleetReconnaissanceStart> starts;
    starts.reserve(droneIds.size());
    const auto observationSource = startsOnGround
        ? dronesim::mission::MissionStateSource::LocalSimulation
        : dronesim::mission::MissionStateSource::FreshTelemetry;
    for (const DroneId droneId : droneIds)
    {
        const auto observation = fleetMission_.CurrentDroneObservation(
            droneId, simulation_, observationSource);
        if (!observation)
            return false;

        const FlightMode mode = observation->mode;
        const bool ready = startsOnGround
            ? mode == FlightMode::Grounded
            : DroneModel::CanAcceptInFlightCommands(mode);
        if (!ready)
            return false;

        const auto home = homePositions.find(droneId);
        starts.push_back({droneId, observation->positionMeters,
            home != homePositions.end() ? home->second : observation->positionMeters});
    }

    const auto& planner = missionEnvironment_.Planner();
    auto assignments = planner.PlanReconnaissance(
        starts, missionEnvironment_.Targets(), startMode);
    if (!assignments)
        return false;

    if (!fleetCommunication_.Prepare(startsOnGround))
        return false;

    if (startsOnGround)
    {
        if (!dronesim::mission::TakeOffFleetDrones(simulation_, droneIds))
            return false;
    }
    for (auto& assignment : *assignments)
    {
        const DroneId droneId = assignment.droneId;
        auto& mission = assignment.mission;
        const dronesim::mission::FleetMissionLeg leg = mission.Leg();
        const std::size_t targetIndex = mission.TargetIndex();
        std::vector<glm::dvec3> route = mission.TakeRouteForDelivery();
        mission.UpdateStatus(dronesim::mission::FleetDroneStatus::RoutePending);
        fleetMission_.SetAssignment(droneId, std::move(mission));
        if (!fleetCommunication_.SendRouteAssignment(
                droneId, leg, targetIndex, std::move(route)))
        {
            fleetMission_.ClearAssignments();
            dronesim::mission::LandActiveFleetDrones(simulation_, droneIds);
            return false;
        }
    }
    return true;
}

bool FleetMissionController::FleetReturnCanStart() const
{
    if (!fleetMission_.HasAssignments() || CompleteFleetMissionIsActive()
        || fleetMission_.FormationTransit().IsActive())
        return false;

    return fleetMission_.AllDronesReadyToReturn(fleetCommunication_.DroneIds(), simulation_);
}

bool FleetMissionController::StartFleetReturn()
{
    if (!FleetReturnCanStart())
        return false;

    return StartFleetReturn(
        dronesim::mission::FleetMissionStartMode::TakeOffFromGround);
}

bool FleetMissionController::StartFleetReturn(
    dronesim::mission::FleetMissionStartMode startMode)
{
    using namespace dronesim::simulation;

    const bool startsOnGround = startMode
        == dronesim::mission::FleetMissionStartMode::TakeOffFromGround;
    const std::vector<DroneId>& droneIds = fleetCommunication_.DroneIds();
    const DroneId leaderId = fleetMission_.LeaderId();
    if (droneIds.empty() || leaderId == 0U)
        return false;
    const auto& planner = missionEnvironment_.Planner();
    // 경로와 대상 드론을 한 항목으로 묶어, 배정 순서가 달라져도 서로 어긋나지 않게 한다.
    std::vector<PlannedFleetRoute> returnPlans;
    returnPlans.reserve(droneIds.size());
    const auto observationSource = startsOnGround
        ? dronesim::mission::MissionStateSource::LocalSimulation
        : dronesim::mission::MissionStateSource::FreshTelemetry;
    for (const auto droneId : droneIds)
    {
        const auto& mission = fleetMission_.Assignment(droneId);
        if (mission.IsLowBatteryReturnComplete())
            continue;

        const auto observation = fleetMission_.CurrentDroneObservation(
            droneId, simulation_, observationSource);
        if (!observation)
            return false;
        const bool ready = startsOnGround
            ? observation->mode == FlightMode::Grounded
            : DroneModel::CanAcceptInFlightCommands(observation->mode);
        if (!ready || mission.Leg() == dronesim::mission::FleetMissionLeg::Returning)
            return false;
        const auto route = planner.PlanReturnRoute(
            observation->positionMeters, mission.StartPositionMeters());
        if (!route)
            return false;
        returnPlans.push_back({droneId, std::move(*route)});
    }

    if (startsOnGround)
    {
        std::vector<DroneId> dronesToTakeOff;
        dronesToTakeOff.reserve(returnPlans.size());
        for (const PlannedFleetRoute& plan : returnPlans)
        {
            if (simulation_.State(plan.droneId).mode == FlightMode::Grounded)
                dronesToTakeOff.push_back(plan.droneId);
        }
        if (!dronesim::mission::TakeOffFleetDrones(simulation_, dronesToTakeOff))
            return false;
    }

    // 경로를 모두 구한 뒤에만 임무 상태와 드론의 경로를 바꾼다.
    for (PlannedFleetRoute& plan : returnPlans)
    {
        auto& mission = fleetMission_.Assignment(plan.droneId);
        mission.BeginLeg(dronesim::mission::FleetMissionLeg::Returning);
        if (!fleetCommunication_.SendRouteAssignment(plan.droneId,
                dronesim::mission::FleetMissionLeg::Returning, mission.TargetIndex(),
                std::move(plan.waypointsMeters)))
            return false;
    }
    return true;
}

bool FleetMissionController::StartFleetRally()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;
    const std::vector<DroneId>& droneIds = fleetCommunication_.DroneIds();
    if (droneIds.empty() || fleetMission_.LeaderId() == 0U
        || std::any_of(droneIds.begin(), droneIds.end(), [this](DroneId droneId)
            {
                return !fleetMission_.CompleteMission().HasHomePosition(droneId);
            }))
        return false;
    const auto& planner = missionEnvironment_.Planner();
    const DroneId leaderId = fleetMission_.LeaderId();
    std::vector<DroneId> rallyDroneIds;
    rallyDroneIds.reserve(droneIds.size());
    for (const DroneId droneId : droneIds)
    {
        const auto& mission = fleetMission_.Assignment(droneId);
        const bool alreadyReturnedForLowBattery =
            mission.IsLowBatteryReturnComplete();
        if (!alreadyReturnedForLowBattery)
            rallyDroneIds.push_back(droneId);
    }
    if (rallyDroneIds.empty())
    {
        if (!StartFleetReturn(
                dronesim::mission::FleetMissionStartMode::ContinueInFlight))
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

    std::vector<PlannedFleetRoute> rallyPlans;
    rallyPlans.reserve(rallyDroneIds.size());
    for (std::size_t index = 0; index < rallyDroneIds.size(); ++index)
    {
        const DroneId droneId = rallyDroneIds[index];
        const auto observation = fleetMission_.CurrentDroneObservation(
            droneId, simulation_,
            dronesim::mission::MissionStateSource::FreshTelemetry);
        if (!observation
            || (droneId != leaderId
                && observation->missionStatus
                    != dronesim::mission::FleetDroneStatus::TargetReached)
            || observation->mode != FlightMode::Grounded)
            return false;
        const glm::dvec3 rallyTarget{
            centerMeters.x + offsets[index].x,
            centerMeters.y + offsets[index].y,
            centerMeters.z};
        const auto route = planner.PlanTransitRoute(
            observation->positionMeters, rallyTarget);
        if (!route)
            return false;
        rallyPlans.push_back({droneId, std::move(*route)});
    }

    // 먼저 모든 이동 경로를 확인해 일부 드론만 출발하는 상황을 막는다.
    if (!dronesim::mission::TakeOffFleetDrones(simulation_, rallyDroneIds))
        return false;

    for (PlannedFleetRoute& plan : rallyPlans)
    {
        auto& mission = fleetMission_.Assignment(plan.droneId);
        mission.BeginLeg(dronesim::mission::FleetMissionLeg::Regrouping);
        if (!fleetCommunication_.SendRouteAssignment(plan.droneId,
                dronesim::mission::FleetMissionLeg::Regrouping, mission.TargetIndex(),
                std::move(plan.waypointsMeters)))
            return false;
    }
    fleetMission_.CompleteMission().SetStage(
        dronesim::mission::CompleteFleetMissionState::Stage::Regrouping);
    return true;
}



void FleetMissionController::AdvanceCompleteFleetMission()
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
        if (!StartFleetReconnaissance(
                dronesim::mission::FleetMissionStartMode::ContinueInFlight,
                fleetMission_.CompleteMission().HomePositions()))
        {
            FailCompleteFleetMission("M02 정찰 경로를 만들지 못해 기체를 착륙시킵니다.");
            return;
        }
        fleetMission_.CompleteMission().SetStage(Stage::Reconnaissance);
        return;
    }

    const std::vector<DroneId>& droneIds = fleetCommunication_.DroneIds();
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
        if (!StartFleetReturn(
                dronesim::mission::FleetMissionStartMode::ContinueInFlight))
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

void FleetMissionController::FailCompleteFleetMission(const std::string& reason)
{
    fleetMission_.CompleteMission().Fail(reason);
    if (fleetMission_.FormationTransit().IsActive())
        fleetMission_.FormationTransit().Stop();
    const auto droneIds = simulation_.DroneIds();
    dronesim::mission::LandActiveFleetDrones(simulation_, droneIds);
}

void FleetMissionController::AdvanceFormationTransit()
{
    const auto safetyFailure = formationFlight_.Advance();
    if (!safetyFailure)
        return;

    const auto& droneIds = fleetCommunication_.DroneIds();
    if (fleetMission_.CompleteMission().CurrentStage()
        == mission::CompleteFleetMissionState::Stage::FormationTransit)
    {
        FailCompleteFleetMission(*safetyFailure);
        return;
    }

    mission::LandActiveFleetDrones(simulation_, droneIds);
}

bool FleetMissionController::StartFormationTransit()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;

    if (fleetMission_.FormationTransit().IsActive()
        || missionEnvironment_.Targets().empty())
        return false;

    InitializeFleetLeadership();

    const std::vector<DroneId>& droneIds = fleetCommunication_.DroneIds();
    if (droneIds.empty())
        return false;
    if (droneIds.size() != 4U || std::any_of(droneIds.begin(), droneIds.end(),
        [this](DroneId droneId)
        {
            return simulation_.State(droneId).mode != FlightMode::Grounded;
        }))
        return false;

    const glm::dvec2 stagingPosition =
        trainingScene_.FormationStagingPositionMeters();
    const double stagingAltitude = std::max(
        missionEnvironment_.RouteEnvironment().GroundAltitudeMeters(stagingPosition) + 15.0,
        missionEnvironment_.MinimumTransitAltitudeMeters());
    const glm::dvec3 destination{stagingPosition, stagingAltitude};
    const auto route = flight_planning::PlanFormationRoute(
        missionEnvironment_.RouteEnvironment(),
        simulation_.State(droneIds.front()).positionMeters,
        destination, droneIds.size(), 1.5, 0.75, 1.0,
        missionEnvironment_.MinimumTransitAltitudeMeters());
    if (!route)
    {
        fleetMission_.FormationTransit().Fail("목적지까지 편대 경로를 찾지 못했습니다.");
        return false;
    }

    if (!fleetCommunication_.Prepare(true))
    {
        fleetMission_.FormationTransit().Fail("가상 통신망을 준비하지 못했습니다.");
        return false;
    }

    if (!dronesim::mission::TakeOffFleetDrones(simulation_, droneIds))
    {
        fleetMission_.FormationTransit().Fail("드론 이륙 명령을 적용하지 못했습니다.");
        return false;
    }

    fleetMission_.BeginFormationTransit(*route,
        "출발 준비 · V자 대형 · "
            + missionEnvironment_.Targets().back().displayName);
    return true;
}





















}
