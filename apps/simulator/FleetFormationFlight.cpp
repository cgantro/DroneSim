#include "FleetFormationFlight.h"

#include <glm/geometric.hpp>

namespace dronesim::simulator
{
FleetFormationFlight::FleetFormationFlight(
    simulation::SimulationCore& simulation,
    mission::FleetMissionState& missionState,
    FleetMissionCommunication& communication)
    : simulation_(simulation),
      missionState_(missionState),
      communication_(communication)
{
}

std::optional<std::string> FleetFormationFlight::Advance()
{
    using namespace dronesim::simulation;
    if (!missionState_.FormationTransit().IsActive())
        return std::nullopt;

    const std::vector<DroneId>& droneIds = communication_.DroneIds();
    if (droneIds.size() < 2U
        || missionState_.FormationTransit().Route().leaderWaypointsMeters.empty())
    {
        missionState_.FormationTransit().Fail("편대 구성을 유지할 수 없어 임무를 멈췄습니다.");
        return std::nullopt;
    }

    const DroneId leaderId = missionState_.LeaderId();
    const DroneState& leader = simulation_.State(leaderId);
    if (leader.mode == FlightMode::Grounded || leader.mode == FlightMode::TakingOff)
        return std::nullopt;
    if (leader.mode == FlightMode::Landing)
    {
        missionState_.FormationTransit().Fail("리더가 착륙해 편대 이동을 멈췄습니다.");
        return std::nullopt;
    }

    // 60Hz 기준 180 Tick은 3초다. 기존 상태 만료 시간에 이 유예 시간을 더해
    // 잠깐의 패킷 지연은 기다리고, 오래 끊긴 경우에만 안전 착륙을 요청한다.
    constexpr std::uint64_t peerTelemetryRecoveryTimeoutTicks = 180U;
    for (std::size_t index = 1U; index < droneIds.size(); ++index)
    {
        const auto& telemetry = missionState_.TelemetryAtLeader();
        const auto reportAgeTicks = telemetry.AgeInTicks(
            droneIds[index], simulation_.TickCount());
        if (!reportAgeTicks)
        {
            missionState_.FormationTransit().Fail("통신 끊김 · 편대 안전 착륙");
            return std::string{"추종 드론 상태 보고가 끊겨 편대를 안전 착륙시킵니다."};
        }

        if (telemetry.Fresh(droneIds[index], simulation_.TickCount()) != nullptr)
            continue;

        constexpr std::uint64_t recoveryWindowTicks =
            dronesim::mission::FleetTelemetry::FreshnessTimeoutTicks
            + peerTelemetryRecoveryTimeoutTicks;
        if (*reportAgeTicks <= recoveryWindowTicks)
        {
            // 편대원 보고가 잠깐 끊기면 리더를 멈춰 연결 회복을 기다린다.
            DroneCommand holdLeader;
            holdLeader.kind = DroneCommand::Kind::SetWaypoint;
            holdLeader.vector = leader.positionMeters;
            static_cast<void>(simulation_.Apply(leaderId, holdLeader));
            missionState_.FormationTransit().UpdateStatus(
                "추종 드론 통신 회복 대기 · 편대 이동 일시 정지");
            return std::nullopt;
        }

        missionState_.FormationTransit().Fail("통신 끊김 · 편대 안전 착륙");
        return std::string{"추종 드론과 통신이 복구되지 않아 편대를 안전 착륙시킵니다."};
    }

    const auto& formationRoute = missionState_.FormationTransit().Route();
    const auto& route = formationRoute.leaderWaypointsMeters;
    missionState_.FormationTransit().AdvanceWaypointIfReached(leader.positionMeters, 3.0);

    const std::size_t nextIndex = missionState_.FormationTransit().WaypointIndex();
    const auto targets = formationRoute.TargetsAtWaypoint(
        nextIndex, droneIds.size(), leader.positionMeters, leader.yawRadians);

    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const DroneId droneId = droneIds[index];
        const auto* peerState = index == 0U
            ? nullptr
            : missionState_.TelemetryAtLeader().Latest(droneId);
        if (index != 0U && peerState == nullptr)
        {
            missionState_.FormationTransit().Fail("추종 드론 상태 보고를 기다리는 중입니다.");
            return std::nullopt;
        }
        const FlightMode mode = index == 0U ? leader.mode : peerState->state.mode;
        if (mode == FlightMode::Grounded || mode == FlightMode::TakingOff)
            continue;
        if (mode == FlightMode::Landing)
        {
            missionState_.FormationTransit().Fail("추종 드론이 착륙해 편대 이동을 멈췄습니다.");
            return std::nullopt;
        }

        const glm::dvec3 target = targets[index];
        DroneCommand command;
        command.kind = DroneCommand::Kind::SetWaypoint;
        command.vector = target;
        if (index == 0U)
        {
            if (simulation_.Apply(droneId, command) != CommandResult::Accepted)
            {
                missionState_.FormationTransit().Fail("리더 목표를 적용하지 못해 임무를 멈췄습니다.");
                return std::nullopt;
            }
            continue;
        }

        if (!communication_.SendFormationTarget(droneId, target))
        {
            missionState_.FormationTransit().Fail("추종 드론에게 편대 목표를 보내지 못했습니다.");
            return std::nullopt;
        }
    }

    // 리더는 마지막 경유점 3m 이내, 추종기는 각 대형 목표 4m 이내일 때 도착으로 본다.
    const bool leaderArrived = missionState_.FormationTransit().WaypointIndex() + 1U == route.size()
        && glm::distance(leader.positionMeters, route.back()) <= 3.0;
    bool followersArrived = true;
    for (std::size_t index = 1U; index < droneIds.size(); ++index)
    {
        const glm::dvec3& target = targets[index];
        const auto* peer = missionState_.TelemetryAtLeader().Latest(droneIds[index]);
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
        missionState_.FormationTransit().Arrive("목적지 도착 · 편대 유지 중");
        return std::nullopt;
    }
    missionState_.FormationTransit().UpdateStatus("이동 중 · 경유점 "
        + std::to_string(missionState_.FormationTransit().WaypointIndex() + 1U)
        + "/" + std::to_string(route.size()));
    return std::nullopt;
}
}
