#include "FleetDroneMissionExecutor.h"
#include "FleetMissionActions.h"

namespace dronesim::simulator
{
FleetDroneMissionExecutor::FleetDroneMissionExecutor(
    simulation::SimulationCore& simulation,
    const FleetMissionEnvironment& environment,
    mission::FleetMissionState& missionState,
    FleetMissionCommunication& communication)
    : simulation_(simulation),
      environment_(environment),
      missionState_(missionState),
      communication_(communication)
{
}

void FleetDroneMissionExecutor::Advance()
{
    using DroneId = dronesim::simulation::DroneId;
    using FlightMode = dronesim::simulation::FlightMode;
    // 600 Tick은 60Hz 기준 10초다. 이 시간 동안 경로가 도착하지 않으면 임무를 실패 처리한다.
    constexpr std::uint64_t routeDeliveryTimeoutTicks = 600U;

    // 드론마다 독립된 배정 상태를 따라가므로 한 기체의 경로 진행이 다른 기체를 막지 않는다.
    for (const DroneId droneId : communication_.DroneIds())
    {
        auto assignment = missionState_.FindAssignment(droneId);
        if (assignment == nullptr)
            continue;
        auto& assignmentState = *assignment;
        if (HandleLowBatteryReturn(droneId, assignmentState))
            continue;

        if (assignmentState.IsLegFinished())
            continue;

        if (!assignmentState.HasReceivedRoute())
        {
            // 다른 드론의 경로 확인 응답을 기다리느라 이 드론까지 멈추지 않도록 만료만 기록한다.
            if (simulation_.TickCount() >= assignmentState.RouteDelivery().SentAtTick()
                    + routeDeliveryTimeoutTicks)
                assignmentState.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
            continue;
        }

        const auto& state = simulation_.State(droneId);
        if (state.mode == FlightMode::Grounded)
        {
            UpdateGroundedMissionStatus(droneId, assignmentState);
            continue;
        }
        if (state.mode == FlightMode::TakingOff || state.mode == FlightMode::Landing)
            continue;

        AdvanceRoute(droneId, assignmentState);
    }
}

bool FleetDroneMissionExecutor::HandleLowBatteryReturn(
    simulation::DroneId droneId,
    mission::FleetReconnaissanceState& assignment)
{
    if (!assignment.CanBeginLowBatteryReturn() || !simulation_.HasBattery(droneId))
        return false;
    if (!simulation_.TryBeginLowBatteryReturn(droneId))
        return false;

    const auto route = environment_.Planner().PlanReturnRoute(
        simulation_.State(droneId).positionMeters,
        assignment.StartPositionMeters());
    bool returnStarted = route.has_value();
    if (returnStarted
        && simulation_.State(droneId).mode == simulation::FlightMode::Grounded)
    {
        const simulation::DroneCommand takeoff{
            simulation::DroneCommand::Kind::TakeOff};
        returnStarted = simulation_.Apply(droneId, takeoff)
            == simulation::CommandResult::Accepted;
    }

    if (returnStarted)
    {
        assignment.BeginLowBatteryReturn();
        returnStarted = communication_.SendRouteAssignment(droneId,
            mission::FleetMissionLeg::Returning, assignment.TargetIndex(),
            std::vector<glm::dvec3>(route->begin(), route->end()));
    }

    if (!returnStarted)
    {
        assignment.UpdateStatus(mission::FleetDroneStatus::BatteryReturnRouteFailed);
        static_cast<void>(mission::TryLandDrone(simulation_, droneId));
    }

    return true;
}

void FleetDroneMissionExecutor::UpdateGroundedMissionStatus(
    simulation::DroneId droneId,
    mission::FleetReconnaissanceState& assignment)
{
    // 준비 중인 드론은 지상에서 경로 메시지를 받을 수 있도록 다음 Tick까지 대기한다.
    const auto currentStatus = assignment.Status();
    if (currentStatus == mission::FleetDroneStatus::TakeoffReady
        || currentStatus == mission::FleetDroneStatus::ReturnPreparing
        || currentStatus == mission::FleetDroneStatus::RegroupPreparing)
        return;

    const auto& currentPosition = simulation_.State(droneId).positionMeters;
    if (assignment.Leg() == mission::FleetMissionLeg::Regrouping)
    {
        assignment.UpdateStatus(assignment.StatusAfterLanding(
            currentPosition, currentPosition));
        return;
    }

    const glm::dvec3& destination = assignment.Leg() == mission::FleetMissionLeg::Returning
        ? assignment.StartPositionMeters()
        : environment_.Targets()[assignment.TargetIndex()].approachPositionMeters;
    assignment.UpdateStatus(assignment.StatusAfterLanding(currentPosition, destination));
}

void FleetDroneMissionExecutor::AdvanceRoute(
    simulation::DroneId droneId,
    mission::FleetReconnaissanceState& assignment)
{
    using Command = simulation::DroneCommand;

    if (!assignment.HasCommandedWaypoint())
    {
        assignment.BeginWaypointCommand();
    }
    else if (!assignment.HasRemainingWaypoints())
    {
        if (assignment.Leg() == mission::FleetMissionLeg::Regrouping)
        {
            assignment.UpdateStatus(mission::FleetDroneStatus::RegroupComplete);
            return;
        }

        if (mission::TryLandDrone(simulation_, droneId))
        {
            if (assignment.Leg() == mission::FleetMissionLeg::Returning)
            {
                const auto landingStatus = assignment.IsLowBatteryReturn()
                    ? mission::FleetDroneStatus::LowBatteryLanding
                    : mission::FleetDroneStatus::ReturnLanding;
                assignment.UpdateStatus(landingStatus);
            }
            else
            {
                assignment.UpdateStatus(mission::FleetDroneStatus::Landing);
            }
        }
        else
        {
            assignment.UpdateStatus(mission::FleetDroneStatus::RouteFailed);
        }
        return;
    }
    // 경유점 2m 안에 들어오면 다음 목표로 넘어가 같은 명령을 반복하지 않는다.
    else if (assignment.AdvanceWaypointIfReached(
        simulation_.State(droneId).positionMeters, 2.0)
        && !assignment.HasRemainingWaypoints())
    {
        return;
    }

    if (const auto* nextWaypoint = assignment.CurrentWaypoint())
    {
        Command waypoint;
        waypoint.kind = Command::Kind::SetWaypoint;
        waypoint.vector = *nextWaypoint;
        if (simulation_.Apply(droneId, waypoint) != simulation::CommandResult::Accepted)
            assignment.UpdateStatus(mission::FleetDroneStatus::RouteFailed);
    }
}
}
