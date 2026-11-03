#include "FleetMissionCommunication.h"
#include "FleetMissionActions.h"
#include "FleetMissionMessages.h"
#include "simulation_core/DroneModel.h"

#include <algorithm>
#include <utility>

namespace dronesim::simulator
{
FleetMissionCommunication::FleetMissionCommunication(
    simulation::SimulationCore& simulation,
    const scene::TrainingRangeScene& trainingScene,
    const FleetMissionEnvironment& missionEnvironment,
    mission::FleetMissionState& missionState)
    : simulation_(simulation),
      trainingScene_(trainingScene),
      missionEnvironment_(missionEnvironment),
      missionState_(missionState)
{
}

const FleetNetworkSession& FleetMissionCommunication::NetworkSession() const noexcept
{
    return networkSession_;
}

const std::vector<simulation::DroneId>& FleetMissionCommunication::DroneIds() const noexcept
{
    return droneIds_;
}

void FleetMissionCommunication::SetRadioEnabled(
    simulation::DroneId droneId, bool enabled)
{
    networkSession_.SetRadioEnabled(droneId, enabled);
}

void FleetMissionCommunication::RefreshDroneRoster()
{
    droneIds_ = simulation_.DroneIds();
}

bool FleetMissionCommunication::Prepare(bool reset)
{
    if (networkSession_.IsReady() && !reset)
        return true;

    RefreshDroneRoster();

    missionState_.ResetTelemetry();

    communication::VirtualMeshSettings settings;
    settings.maximumLinkRangeMeters = 130.0;
    const auto droneIds = simulation_.DroneIds();
    std::vector<communication::NetworkNodeSnapshot> nodes;
    nodes.reserve(droneIds.size());
    for (const auto droneId : droneIds)
    {
        nodes.push_back({droneId, simulation_.State(droneId).positionMeters});
    }
    if (!networkSession_.Prepare(nodes,
            trainingScene_.BuildingObstacles(), settings, reset))
        return false;
    missionState_.FormationNetwork().RecordNetworkNodesUpdated(simulation_.TickCount());
    return true;
}

bool FleetMissionCommunication::SendRouteAssignment(
    dronesim::simulation::DroneId droneId,
    dronesim::mission::FleetMissionLeg leg,
    std::size_t targetIndex,
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    const auto& targets = missionEnvironment_.Targets();
    if (!networkSession_.IsReady()
        || targets.empty() || targetIndex >= targets.size())
        return false;

    auto mission = missionState_.FindAssignment(droneId);
    if (mission == missionState_.AssignmentsEnd() || droneIds_.empty())
        return false;

    const std::uint64_t assignmentVersion = missionState_.TakeNextAssignmentVersion();
    std::uint64_t leaseExpiresAtTick = 0U;
    if (leg == dronesim::mission::FleetMissionLeg::Reconnaissance)
    {
        glm::dvec3 startPositionMeters = simulation_.State(droneId).positionMeters;
        if (droneId != missionState_.LeaderId())
        {
            if (const auto* report = missionState_.TelemetryAtLeader().Fresh(
                    droneId, simulation_.TickCount()))
                startPositionMeters = report->state.positionMeters;
        }
        leaseExpiresAtTick = dronesim::mission::EstimateMissionLeaseExpiry(
            simulation_.TickCount(), startPositionMeters, routeWaypointsMeters);
    }
    const std::uint32_t sequence = mission->second.RouteDelivery().NextSequence();
    const std::string payload = dronesim::mission::EncodeFleetRoute(
        leg, targetIndex, assignmentVersion, leaseExpiresAtTick, routeWaypointsMeters);
    if (droneId == missionState_.LeaderId())
    {
        ApplyRouteAssignment(droneId, leg, targetIndex,
            std::move(routeWaypointsMeters), sequence, assignmentVersion,
            leaseExpiresAtTick);
        mission->second.RouteDelivery().ClearPending();
        return true;
    }

    const auto messageId = networkSession_.Send(
        missionState_.LeaderId(), droneId,
        sequence, "fleet_route", payload);
    if (!messageId)
        return false;

    mission->second.SetAssignmentVersion(assignmentVersion);
    mission->second.RouteDelivery().Queue(
        sequence, payload, simulation_.TickCount());
    mission->second.WaitForRoute();
    mission->second.UpdateStatus(dronesim::mission::FleetDroneStatus::RoutePending);
    return true;
}

bool FleetMissionCommunication::SendFormationTarget(
    simulation::DroneId droneId,
    const glm::dvec3& targetMeters)
{
    constexpr std::uint64_t targetRefreshTicks = 6U;
    const std::uint64_t currentTick = simulation_.TickCount();
    if (!missionState_.FormationNetwork().ShouldSendFollowerTarget(
            droneId, targetMeters, currentTick, 0.75, targetRefreshTicks))
        return true;

    const auto messageId = networkSession_.Send(
        missionState_.LeaderId(), droneId,
        missionState_.FormationNetwork().TakeNextMessageSequence(),
        "formation_waypoint", dronesim::mission::EncodeWaypoint(targetMeters));
    if (!messageId)
        return false;

    missionState_.FormationNetwork().RecordFollowerTargetSent(
        droneId, targetMeters, currentTick);
    return true;
}

void FleetMissionCommunication::ApplyRouteAssignment(
    dronesim::simulation::DroneId droneId,
    dronesim::mission::FleetMissionLeg leg,
    std::size_t targetIndex,
    std::vector<glm::dvec3> routeWaypointsMeters,
    std::uint32_t sequence,
    std::uint64_t assignmentVersion,
    std::uint64_t assignmentLeaseExpiresAtTick)
{
    auto mission = missionState_.FindAssignment(droneId);
    if (mission == missionState_.AssignmentsEnd())
        return;

    mission->second.BeginLeg(leg);
    mission->second.ApplyAssignment(targetIndex, assignmentVersion);
    mission->second.RouteDelivery().RecordApplied(sequence);
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

void FleetMissionCommunication::AdvanceTick()
{
    if (!networkSession_.IsReady() || (!missionState_.FormationTransit().IsActive()
        && !missionState_.CompleteMission().IsActive()
        && !missionState_.HasActiveReconnaissance()))
        return;

    const std::uint64_t currentTick = simulation_.TickCount();
    RefreshNodes(currentTick);
    // 먼저 지연 시간이 지난 패킷을 꺼내 처리한 뒤, 새 상태와 경로를 전송한다.
    networkSession_.Advance(currentTick);
    ProcessDeliveredMessages(currentTick);
    SendTelemetry(currentTick);
    RetryPendingRoutes(currentTick);
}

void FleetMissionCommunication::RefreshNodes(std::uint64_t currentTick)
{
    if (!networkSession_.IsReady())
        return;

    using dronesim::simulation::DroneId;
    constexpr std::uint64_t networkPositionRefreshTicks = 6U;
    // 링크 판정은 매 6 Tick마다 갱신한다. 60Hz 기준 0.1초마다 실제 드론 위치를 반영한다.
    const bool refreshNetworkPositions =
        missionState_.FormationNetwork().ShouldRefreshNetworkNodes(
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
        networkSession_.UpdateNodes(nodes);
        missionState_.FormationNetwork().RecordNetworkNodesUpdated(currentTick);
    }

}

void FleetMissionCommunication::ProcessDeliveredMessages(std::uint64_t currentTick)
{
    using dronesim::simulation::DroneCommand;
    using dronesim::simulation::DroneId;
    const DroneId leaderId = missionState_.LeaderId();

    for (const auto& delivered : networkSession_.DeliveredMessages())
    {
        const auto& message = delivered.message;
        if (message.kind == "drone_telemetry"
            && message.destination == leaderId && message.source != leaderId)
        {
            if (std::find(droneIds_.begin(), droneIds_.end(), message.source)
                == droneIds_.end())
                continue;
            const auto telemetry = dronesim::mission::DecodePeerTelemetry(message.payload);
            if (telemetry)
                static_cast<void>(missionState_.TelemetryAtLeader().Record(message.source,
                    *telemetry, delivered.deliveredAtTick, message.sequence));
            continue;
        }

        if (message.kind == "fleet_route_ack"
            && message.destination == leaderId)
        {
            const auto mission = missionState_.FindAssignment(message.source);
            if (mission != missionState_.AssignmentsEnd())
                static_cast<void>(mission->second.RouteDelivery().Acknowledge(
                    message.sequence));
            continue;
        }

        if (message.source != leaderId)
            continue;

        if (message.kind == "formation_waypoint" && missionState_.FormationTransit().IsActive())
        {
            if (std::find(droneIds_.begin(), droneIds_.end(), message.destination)
                == droneIds_.end())
                continue;
            const auto waypoint = dronesim::mission::DecodeWaypoint(message.payload);
            if (!waypoint)
                continue;
            if (!missionState_.FormationNetwork().IsNewAppliedSequence(
                    message.destination, message.sequence))
                continue;

            DroneCommand command;
            command.kind = DroneCommand::Kind::SetWaypoint;
            command.vector = *waypoint;
            if (simulation_.Apply(message.destination, command)
                != dronesim::simulation::CommandResult::Accepted)
            {
                missionState_.FormationTransit().Fail("통신으로 받은 편대 목표를 적용하지 못했습니다.");
                return;
            }
            missionState_.FormationNetwork().RecordAppliedSequence(
                message.destination, message.sequence);
            continue;
        }

        if (message.kind != "fleet_route")
            continue;

        if (std::find(droneIds_.begin(), droneIds_.end(), message.destination)
            == droneIds_.end())
            continue;

        auto mission = missionState_.FindAssignment(message.destination);
        const auto route = dronesim::mission::DecodeFleetRoute(message.payload);
        if (mission == missionState_.AssignmentsEnd() || !route
            || route->targetIndex >= missionEnvironment_.Targets().size())
        {
            if (mission != missionState_.AssignmentsEnd())
                mission->second.UpdateStatus(dronesim::mission::FleetDroneStatus::RouteFailed);
            continue;
        }

        if (route->assignmentLeaseExpiresAtTick != 0U
            && currentTick >= route->assignmentLeaseExpiresAtTick)
        {
            static_cast<void>(dronesim::mission::TryLandDrone(
                simulation_, message.destination));
            mission->second.UpdateStatus(
                dronesim::mission::FleetDroneStatus::AssignmentExpiredLanding);
            continue;
        }

        if (message.sequence == mission->second.RouteDelivery().AppliedSequence())
        {
            // 같은 경로가 다시 오면 비행 상태를 초기화하지 않고 확인 응답만 다시 보낸다.
            static_cast<void>(networkSession_.Send(
                message.destination, leaderId,
                message.sequence, "fleet_route_ack", {}));
            continue;
        }
        if (message.sequence != mission->second.RouteDelivery().PendingSequence()
            || message.sequence < mission->second.RouteDelivery().AppliedSequence())
            continue;

        ApplyRouteAssignment(message.destination,
            route->leg, route->targetIndex,
            route->waypointsMeters, message.sequence, route->assignmentVersion,
            route->assignmentLeaseExpiresAtTick);
        static_cast<void>(networkSession_.Send(
            message.destination, leaderId,
            message.sequence, "fleet_route_ack", {}));
    }
}

void FleetMissionCommunication::SendTelemetry(std::uint64_t currentTick)
{
    using dronesim::simulation::DroneId;
    const std::vector<DroneId>& droneIds = droneIds_;
    const DroneId leaderId = missionState_.LeaderId();

    constexpr std::uint64_t telemetryIntervalTicks = 6U;

    // 리더를 제외한 드론의 상태만 보내고, 직전 전송 후 6 Tick이 지나야 다음 상태를 보낸다.
    for (const DroneId droneId : droneIds)
    {
        if (droneId == leaderId)
            continue;
        if (!missionState_.TelemetryAtLeader().ShouldSend(
                droneId, currentTick, telemetryIntervalTicks))
            continue;

        const auto mission = missionState_.FindAssignment(droneId);
        const std::uint8_t missionLeg = mission == missionState_.AssignmentsEnd()
            ? 0U : static_cast<std::uint8_t>(mission->second.Leg());
        const dronesim::mission::FleetDroneStatus missionStatus =
            mission == missionState_.AssignmentsEnd()
            ? dronesim::mission::FleetDroneStatus::Other : mission->second.Status();
        const std::uint64_t assignmentVersion = mission == missionState_.AssignmentsEnd()
            ? 0U : mission->second.AssignmentVersion();
        const auto& state = simulation_.State(droneId);
        static_cast<void>(networkSession_.Send(
            droneId, leaderId,
            missionState_.TelemetryAtLeader().TakeNextSequence(), "drone_telemetry",
            dronesim::mission::EncodePeerTelemetry(state.positionMeters, state.mode,
                missionLeg, missionStatus,
                assignmentVersion)));
        missionState_.TelemetryAtLeader().RecordSendAttempt(droneId, currentTick);
    }
}

void FleetMissionCommunication::RetryPendingRoutes(std::uint64_t currentTick)
{
    using dronesim::simulation::DroneId;
    const DroneId leaderId = missionState_.LeaderId();

    constexpr std::uint64_t routeRetryIntervalTicks = 60U;
    constexpr std::uint32_t maximumRouteTransmissions = 10U;
    // 확인 응답이 없는 경로만 1초 간격으로 다시 보내며, 10회를 넘기지 않는다.
    for (const DroneId droneId : droneIds_)
    {
        auto assignment = missionState_.FindAssignment(droneId);
        if (assignment == missionState_.AssignmentsEnd())
            continue;
        auto& mission = assignment->second;
        if (droneId == leaderId
            || !mission.RouteDelivery().CanRetry(
                currentTick, routeRetryIntervalTicks, maximumRouteTransmissions))
            continue;

        if (networkSession_.Send(leaderId, droneId,
                mission.RouteDelivery().PendingSequence(), "fleet_route",
                mission.RouteDelivery().PendingPayload()))
            mission.RouteDelivery().RecordRetry(currentTick);
    }
}
}
