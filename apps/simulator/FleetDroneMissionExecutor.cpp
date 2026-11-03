#include "FleetDroneMissionExecutor.h"
#include "FleetMissionActions.h"
#include "simulation_core/DroneModel.h"

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
    using Command = dronesim::simulation::DroneCommand;
    using FlightMode = dronesim::simulation::FlightMode;
    // 600 Tick은 60Hz 기준 10초다. 이 시간 동안 경로가 도착하지 않으면 임무를 실패 처리한다.
    constexpr std::uint64_t routeDeliveryTimeoutTicks = 600U;
    const auto& planner = environment_.Planner();

    // 드론마다 독립된 배정 상태를 따라가므로 한 기체의 경로 진행이 다른 기체를 막지 않는다.
    for (const DroneId droneId : communication_.DroneIds())
    {
        auto assignment = missionState_.FindAssignment(droneId);
        if (assignment == missionState_.AssignmentsEnd())
            continue;
        auto& mission = assignment->second;
        // 배터리 복귀가 시작되면 일반 정찰 경로를 멈추고, 현재 위치에서 출발점까지 새 경로를 보낸다.
        if (mission.CanBeginLowBatteryReturn() && simulation_.HasBattery(droneId))
        {
            if (simulation_.TryBeginLowBatteryReturn(droneId))
            {
                const auto route = planner.PlanReturnRoute(
                    simulation_.State(droneId).positionMeters,
                    mission.StartPositionMeters());
                bool returnStarted = route.has_value();
                if (returnStarted
                    && simulation_.State(droneId).mode == FlightMode::Grounded)
                    returnStarted = simulation_.Apply(droneId,
                        {Command::Kind::TakeOff})
                        == dronesim::simulation::CommandResult::Accepted;
                if (returnStarted)
                {
                    mission.BeginLowBatteryReturn();
                    returnStarted = communication_.SendRouteAssignment(droneId,
                        dronesim::mission::FleetMissionLeg::Returning, mission.TargetIndex(),
                        std::vector<glm::dvec3>(route->begin(), route->end()));
                }
                if (!returnStarted)
                {
                    mission.UpdateStatus(
                        dronesim::mission::FleetDroneStatus::BatteryReturnRouteFailed);
                    static_cast<void>(dronesim::mission::TryLandDrone(
                        simulation_, droneId));
                }
                continue;
            }
        }

        if (mission.IsLegFinished())
            continue;

        if (!mission.HasReceivedRoute())
        {
            // 다른 드론의 경로 확인 응답을 기다리느라 이 드론까지 멈추지 않도록 만료만 기록한다.
            if (simulation_.TickCount() >= mission.RouteDelivery().SentAtTick()
                    + routeDeliveryTimeoutTicks)
                mission.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
            continue;
        }

        const auto& state = simulation_.State(droneId);
        if (state.mode == FlightMode::Grounded)
        {
            // 착륙 뒤에는 이동 명령을 내리지 않고, 임무 구간과 현재 위치로 상태를 확정한다.
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
                ? mission.StartPositionMeters()
                : environment_.Targets()[mission.TargetIndex()].approachPositionMeters;
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
        // 경유점 2m 안에 들어오면 다음 목표로 넘어가 매 Tick 같은 명령을 반복하지 않는다.
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
}
