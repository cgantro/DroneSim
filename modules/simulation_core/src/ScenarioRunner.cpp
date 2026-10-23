#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"
#include "ScenarioDefinition.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aerosimlab::simulation
{
namespace
{
using detail::ExpectedDrone;
using detail::FormationLink;
using detail::FormationOutage;
using detail::ScenarioDefinition;
using detail::ScenarioDrone;
using detail::ScheduledCameraEvent;
using detail::ScheduledCommand;
using detail::DroneCommunicationOutage;
using DroneIds = std::unordered_map<std::string, DroneId>;
using DroneCommunicationStatus = ScenarioResult::Drone::CommunicationStatus;

struct DroneObservation
{
    DroneCommunicationStatus status = DroneCommunicationStatus::Connected;
    std::uint64_t lastHeartbeatTick = 0;
    DroneState lastKnownState;
};

using DroneObservations = std::unordered_map<std::string, DroneObservation>;

bool CanReceiveWaypoint(FlightMode mode) noexcept
{
    return mode == FlightMode::Hovering
        || mode == FlightMode::Flying
        || mode == FlightMode::FollowingWaypoint;
}

bool IsConnected(const DroneObservations& observations, const std::string& droneName)
{
    return observations.at(droneName).status == DroneCommunicationStatus::Connected;
}

bool IsUnknown(const DroneObservations& observations, const std::string& droneName)
{
    return observations.at(droneName).status == DroneCommunicationStatus::Unknown;
}

const DroneCommunicationOutage* FindActiveCommunicationOutage(
    std::uint64_t tick,
    const std::string& droneName,
    const std::vector<DroneCommunicationOutage>& outages)
{
    const auto found = std::find_if(outages.begin(), outages.end(),
        [tick, &droneName](const DroneCommunicationOutage& outage)
        {
            return outage.droneName == droneName
                && tick >= outage.startTick && tick < outage.endTick;
        });
    return found == outages.end() ? nullptr : &*found;
}

void UpdateDroneObservations(
    std::uint64_t tick,
    const ScenarioDefinition& definition,
    const SimulationCore& simulation,
    const DroneIds& droneIds,
    DroneObservations& observations,
    ScenarioResult& result)
{
    const auto updateDrone = [&](const std::string& droneName)
    {
        const DroneId droneId = droneIds.at(droneName);
        DroneObservation& observation = observations.at(droneName);
        const DroneCommunicationOutage* outage = FindActiveCommunicationOutage(
            tick, droneName, definition.communicationOutages);
        if (outage == nullptr)
        {
            if (observation.status != DroneCommunicationStatus::Connected)
            {
                result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                    + droneName + " 드론의 통신이 복구되어 새 상태를 받았습니다.");
            }
            observation.status = DroneCommunicationStatus::Connected;
            observation.lastHeartbeatTick = tick;
            observation.lastKnownState = simulation.State(droneId);
            return;
        }

        const std::uint64_t ticksSinceHeartbeat = tick - observation.lastHeartbeatTick;
        const DroneCommunicationStatus newStatus =
            ticksSinceHeartbeat >= outage->heartbeatTimeoutTicks
                ? DroneCommunicationStatus::Unknown
                : DroneCommunicationStatus::Stale;
        if (observation.status == DroneCommunicationStatus::Connected)
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론의 하트비트가 끊겼습니다.");
        }
        if (newStatus == DroneCommunicationStatus::Unknown
            && observation.status != DroneCommunicationStatus::Unknown)
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론의 상태를 확인할 수 없어 임무 정보를 미확정으로 바꿨습니다.");
        }
        observation.status = newStatus;
    };

    // 이벤트 로그 순서도 시나리오 입력 순서에 맞춰 매번 같게 만든다.
    if (definition.drones.empty())
    {
        updateDrone(definition.primaryDroneName);
        return;
    }

    for (const ScenarioDrone& drone : definition.drones)
        updateDrone(drone.name);
}

/**
 * @brief 시나리오에서 사용할 이름과 실제 드론 번호를 연결한다.
 * @details 드론 목록이 있으면 SimulationCore가 미리 만든 기본 드론을 지우고 목록을 구성한다.
 */
DroneIds CreateScenarioDrones(
    const ScenarioDefinition& definition,
    SimulationCore& simulation)
{
    DroneIds droneIds;
    if (definition.drones.empty())
    {
        droneIds.emplace(definition.primaryDroneName, simulation.PrimaryDroneId());
        return droneIds;
    }

    const DroneId unusedDefaultDrone = simulation.PrimaryDroneId();
    for (const ScenarioDrone& drone : definition.drones)
    {
        droneIds.emplace(drone.name, simulation.AddDrone(
            drone.initialPositionMeters, drone.initialYawRadians));
    }

    if (!simulation.RemoveDrone(unusedDefaultDrone))
        throw std::logic_error("시나리오용 기본 드론을 교체하지 못했습니다.");
    return droneIds;
}

/** @brief 카메라 동작을 적용하고 실행 기록에 쓸 이름을 돌려준다. */
std::string_view ApplyCameraEvent(
    SimulationCore& simulation, detail::CameraEventAction action)
{
    using Action = detail::CameraEventAction;
    switch (action)
    {
    case Action::Enable:
        simulation.SetCameraEnabled(true);
        return "enable";
    case Action::Disable:
        simulation.SetCameraEnabled(false);
        return "disable";
    case Action::Fault:
        simulation.SetCameraFault(true);
        return "fault";
    case Action::Recover:
        simulation.SetCameraFault(false);
        return "recover";
    }
    return "unknown";
}

double OrientationErrorRadians(
    const glm::dquat& actual, const glm::dquat& expected) noexcept
{
    const double dot = std::abs(
        actual.w * expected.w + actual.x * expected.x
        + actual.y * expected.y + actual.z * expected.z);
    return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

void RecordMinimumDroneSeparation(
    const ScenarioDefinition& definition,
    const SimulationCore& simulation,
    const DroneIds& droneIds,
    std::vector<glm::dvec3>& positionsMeters,
    ScenarioResult& result)
{
    if (!definition.minimumDroneSeparationMeters)
        return;

    positionsMeters.clear();
    for (const ScenarioDrone& drone : definition.drones)
    {
        positionsMeters.push_back(
            simulation.State(droneIds.at(drone.name)).positionMeters);
    }

    std::optional<double> minimumDistanceSquaredMeters;
    if (result.minimumDroneSeparationMeters)
    {
        const double previousMinimum = *result.minimumDroneSeparationMeters;
        minimumDistanceSquaredMeters = previousMinimum * previousMinimum;
    }

    bool minimumDistanceChanged = false;
    for (std::size_t first = 0; first < positionsMeters.size(); ++first)
    {
        for (std::size_t second = first + 1; second < positionsMeters.size(); ++second)
        {
            const glm::dvec3 offset = positionsMeters[first] - positionsMeters[second];
            const double distanceSquaredMeters = glm::dot(offset, offset);
            if (!minimumDistanceSquaredMeters
                || distanceSquaredMeters < *minimumDistanceSquaredMeters)
            {
                minimumDistanceSquaredMeters = distanceSquaredMeters;
                minimumDistanceChanged = true;
            }
        }
    }

    if (minimumDistanceChanged)
        result.minimumDroneSeparationMeters = std::sqrt(*minimumDistanceSquaredMeters);
}

/**
 * @brief 통신 장애와 명령 순서를 반영해 이번 명령을 전달할지 결정한다.
 * @details 유실된 명령은 순서 번호를 갱신하지 않는다. 전달 대상으로 정해지면
 *          시뮬레이터가 명령을 받아들이는지와 별개로 마지막 전달 순서를 갱신한다.
 */
bool PrepareScheduledCommandDelivery(
    std::uint64_t tick,
    const ScheduledCommand& scheduled,
    const DroneObservations& observations,
    std::unordered_map<std::string, std::uint64_t>& lastDeliveredSequence,
    ScenarioResult& result)
{
    if (!IsConnected(observations, scheduled.droneName))
    {
        result.eventLog.push_back("tick " + std::to_string(tick) + ": "
            + scheduled.droneName + " 드론에 명령을 전달하지 못했습니다. 마지막 위치는 이전 보고값입니다.");
        return false;
    }
    if (scheduled.dropped)
    {
        result.eventLog.push_back("tick " + std::to_string(tick) + ": "
            + scheduled.droneName + " 드론 명령을 통신 장애로 버렸습니다.");
        return false;
    }

    std::uint64_t& lastSequence = lastDeliveredSequence[scheduled.droneName];
    if (scheduled.sequence <= lastSequence)
    {
        result.issues.push_back("tick " + std::to_string(tick) + "에서 "
            + scheduled.droneName + " 드론의 역순 명령을 거부했습니다.");
        return false;
    }
    lastSequence = scheduled.sequence;

    if (scheduled.delayTicks > 0)
    {
        result.eventLog.push_back(scheduled.droneName + " 드론에 tick "
            + std::to_string(scheduled.sendTick)
            + "에 보낸 명령을 " + std::to_string(scheduled.delayTicks)
            + " Tick 늦게 전달했습니다.");
    }
    return true;
}

/** @brief 전달이 결정된 명령을 한 번 적용하고 결과를 기록한다. */
void ApplyDeliveredCommand(
    std::uint64_t tick,
    const ScheduledCommand& scheduled,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    ScenarioResult& result)
{
    const CommandResult commandResult = simulation.Apply(
        droneIds.at(scheduled.droneName), scheduled.command);
    if (commandResult != CommandResult::Accepted)
    {
        result.issues.push_back("tick " + std::to_string(tick) + "에서 "
            + scheduled.droneName + " 드론 명령을 적용하지 못했습니다.");
    }
    else
    {
        result.eventLog.push_back("tick " + std::to_string(tick) + ": "
            + scheduled.droneName + " 드론 명령을 적용했습니다.");
    }
    if (scheduled.duplicated)
    {
        result.eventLog.push_back("tick " + std::to_string(tick) + ": "
            + scheduled.droneName + " 드론의 중복 재전송 명령을 한 번만 처리했습니다.");
    }
}

void ApplyScheduledCommands(
    std::uint64_t tick,
    const std::vector<ScheduledCommand>& commands,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    std::size_t& nextCommand,
    std::unordered_map<std::string, std::uint64_t>& lastDeliveredSequence,
    ScenarioResult& result)
{
    while (nextCommand < commands.size() && commands[nextCommand].tick == tick)
    {
        const ScheduledCommand& scheduled = commands[nextCommand++];
        if (!PrepareScheduledCommandDelivery(
                tick, scheduled, observations, lastDeliveredSequence, result))
        {
            continue;
        }
        ApplyDeliveredCommand(tick, scheduled, simulation, droneIds, result);
    }
}

void UpdateMissionCommands(
    std::uint64_t tick,
    std::optional<ScenarioResult::Mission>& mission,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    if (!mission)
        return;

    const auto findNearestDrone = [&](
        const glm::dvec3& target, bool allowBusy) -> std::optional<std::string>
    {
        std::optional<std::string> nearest;
        double nearestDistanceSquared = std::numeric_limits<double>::infinity();
        const auto considerDrone = [&](const std::string& droneName)
        {
            if (!IsConnected(observations, droneName))
                return;
            const bool isFormationFollower = std::any_of(
                definition.formationLinks.begin(), definition.formationLinks.end(),
                [&droneName](const FormationLink& link)
                {
                    return link.followerName == droneName;
                });
            if (isFormationFollower)
                return;
            const auto activeTask = std::find_if(
                mission->tasks.begin(), mission->tasks.end(),
                [&droneName](const ScenarioResult::MissionTask& candidate)
                {
                    return candidate.droneName == droneName
                        && !candidate.completed && !candidate.failed;
                });
            if (!allowBusy && activeTask != mission->tasks.end())
                return;

            const DroneState& state = simulation.State(droneIds.at(droneName));
            if (!CanReceiveWaypoint(state.mode))
                return;
            const glm::dvec3 offset = state.positionMeters - target;
            const double distanceSquared = glm::dot(offset, offset);
            if (distanceSquared < nearestDistanceSquared)
            {
                nearest = droneName;
                nearestDistanceSquared = distanceSquared;
            }
        };

        if (definition.drones.empty())
            considerDrone(definition.primaryDroneName);
        else
            for (const ScenarioDrone& drone : definition.drones)
                considerDrone(drone.name);
        return nearest;
    };

    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (task.completed || task.failed || task.droneName.empty()
            || !IsUnknown(observations, task.droneName))
            continue;

        const std::string unreachableDrone = task.droneName;
        task.droneName.clear();
        task.started = false;
        result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
            + task.name + "' 임무는 " + unreachableDrone
            + " 드론의 완료 보고를 받지 못해 미확정으로 남았습니다.");

        const std::optional<std::string> replacement =
            findNearestDrone(task.targetPositionMeters, true);
        if (!replacement)
            continue;

        const auto displacedTask = std::find_if(
            mission->tasks.begin(), mission->tasks.end(),
            [&replacement](const ScenarioResult::MissionTask& candidate)
            {
                return candidate.droneName == *replacement
                    && !candidate.completed && !candidate.failed;
            });
        if (displacedTask != mission->tasks.end())
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
                + displacedTask->name + "' 임무는 " + *replacement
                + " 드론이 다른 구역을 맡아 대기열로 돌아갔습니다.");
            displacedTask->droneName.clear();
            displacedTask->started = false;
        }

        task.droneName = *replacement;
        task.assignmentHistory.push_back(*replacement);
        ++task.reassignmentCount;
        result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
            + task.name + "' 미완료 구역을 " + *replacement
            + " 드론에 다시 배정했습니다.");
    }

    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (task.completed || task.failed || !task.droneName.empty()
            || tick < task.startTick)
            continue;
        const std::optional<std::string> availableDrone =
            findNearestDrone(task.targetPositionMeters, false);
        if (!availableDrone)
            continue;
        task.droneName = *availableDrone;
        task.assignmentHistory.push_back(*availableDrone);
        ++task.reassignmentCount;
        result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
            + task.name + "' 대기 중인 구역을 " + *availableDrone
            + " 드론에 배정했습니다.");
    }

    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (task.completed || task.failed || tick < task.startTick
            || task.droneName.empty() || !IsConnected(observations, task.droneName))
            continue;
        const DroneId droneId = droneIds.at(task.droneName);
        if (!CanReceiveWaypoint(simulation.State(droneId).mode))
        {
            task.failed = true;
            result.issues.push_back(task.started
                ? task.droneName + " 드론이 비행을 멈춰 임무를 이어 가지 못했습니다."
                : task.droneName + " 드론이 비행 중이어서 임무를 시작하지 못했습니다.");
            continue;
        }
        if (!task.started)
        {
            task.started = true;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.role + "' 임무를 시작했습니다.");
        }

        const CommandResult moveResult = simulation.Apply(droneId,
            {DroneCommand::Kind::SetWaypoint, task.targetPositionMeters});
        if (moveResult != CommandResult::Accepted)
        {
            task.failed = true;
            result.issues.push_back(task.droneName + " 드론에 임무 목표를 전달하지 못했습니다.");
        }
    }
}

/** @brief 연결 단절·복구 시점을 기록하고 해당 Tick의 위치 공유 가능 여부를 반환한다. */
bool UpdateFormationOutageState(
    std::uint64_t tick,
    const FormationLink& link,
    ScenarioResult& result)
{
    bool linkAvailable = true;
    for (const FormationOutage& outage : link.outages)
    {
        if (tick == outage.startTick)
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + link.leaderName + " 드론과 " + link.followerName
                + " 드론 사이의 위치 공유가 끊겼습니다.");
        }
        if (tick == outage.endTick)
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + link.leaderName + " 드론과 " + link.followerName
                + " 드론 사이의 위치 공유가 복구됐습니다.");
        }
        if (tick >= outage.startTick && tick < outage.endTick)
            linkAvailable = false;
    }
    return linkAvailable;
}

void UpdateFormationLinks(
    std::uint64_t tick,
    const std::vector<FormationLink>& formationLinks,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    ScenarioResult& result)
{
    for (const FormationLink& link : formationLinks)
    {
        if (!UpdateFormationOutageState(tick, link, result))
            continue;
        if (!IsConnected(observations, link.leaderName)
            || !IsConnected(observations, link.followerName))
            continue;
        const DroneState& leaderState = simulation.State(droneIds.at(link.leaderName));
        const DroneState& followerState = simulation.State(droneIds.at(link.followerName));
        if (!CanReceiveWaypoint(leaderState.mode)
            || !CanReceiveWaypoint(followerState.mode))
            continue;

        const CommandResult followResult = simulation.Apply(
            droneIds.at(link.followerName),
            {DroneCommand::Kind::SetWaypoint, leaderState.positionMeters + link.offsetMeters});
        if (followResult != CommandResult::Accepted)
            result.issues.push_back(link.followerName
                + " 드론이 공유 위치를 따라가지 못했습니다.");
    }
}

void ApplyCameraEvents(
    std::uint64_t tick,
    const std::vector<ScheduledCameraEvent>& cameraEvents,
    SimulationCore& simulation,
    std::size_t& nextCameraEvent,
    ScenarioResult& result)
{
    while (nextCameraEvent < cameraEvents.size()
        && cameraEvents[nextCameraEvent].tick == tick)
    {
        const ScheduledCameraEvent& event = cameraEvents[nextCameraEvent++];
        const std::string_view actionName = ApplyCameraEvent(simulation, event.action);
        result.eventLog.push_back("tick " + std::to_string(tick) + ": 카메라 "
            + std::string(actionName));
    }
}

void UpdateMissionResults(
    std::uint64_t tick,
    std::optional<ScenarioResult::Mission>& mission,
    const SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    ScenarioResult& result)
{
    if (!mission)
        return;

    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (!task.started || task.completed || task.failed
            || task.droneName.empty() || !IsConnected(observations, task.droneName))
            continue;

        const glm::dvec3 distance = simulation.State(droneIds.at(task.droneName))
            .positionMeters - task.targetPositionMeters;
        if (glm::length(distance) <= task.toleranceMeters)
        {
            task.completed = true;
            task.completedTick = tick;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.role + "' 임무를 마쳤습니다.");
        }
        else if (tick >= task.deadlineTick)
        {
            task.failed = true;
            result.issues.push_back(task.droneName + " 드론이 임무 기한 안에 목표에 도달하지 못했습니다.");
        }
    }
}

void PopulateFinalSnapshot(
    const ScenarioDefinition& definition,
    const SimulationCore& simulation,
    const DroneObservations& observations,
    ScenarioResult& result)
{
    result.tickCount = simulation.TickCount();
    result.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    if (definition.drones.empty())
    {
        const DroneObservation& observation = observations.at(definition.primaryDroneName);
        result.drones.push_back({definition.primaryDroneName, observation.lastKnownState,
            observation.status, observation.lastHeartbeatTick});
    }
    else
    {
        for (const ScenarioDrone& drone : definition.drones)
        {
            const DroneObservation& observation = observations.at(drone.name);
            result.drones.push_back({drone.name, observation.lastKnownState,
                observation.status, observation.lastHeartbeatTick});
        }
    }
    result.finalState = result.drones.front().finalState;
    result.finalCamera = simulation.Camera();
}

/** @brief 단일 드론 형식과 이름별 형식에 맞는 비행 상태 오류 문구를 만든다. */
std::string FlightModeMismatchMessage(
    const ExpectedDrone& expectedDrone,
    const ScenarioResult::Drone& actualDrone,
    bool hasNamedDroneExpectations)
{
    if (hasNamedDroneExpectations)
        return expectedDrone.name + " 드론의 최종 비행 상태가 예상과 다릅니다.";

    return "예상 비행 상태는 " + std::string(ToString(expectedDrone.mode))
        + "이지만 실제 상태는 "
        + std::string(ToString(actualDrone.finalState.mode)) + "입니다.";
}

void CheckExpectedDroneResults(
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    std::unordered_map<std::string_view, std::size_t> resultIndexByName;
    if (!definition.expectedDrones.empty())
    {
        // 결과 이름을 복사하지 않고 빌려 쓰는 색인으로 반복 검색을 줄인다.
        resultIndexByName.reserve(result.drones.size());
        for (std::size_t index = 0; index < result.drones.size(); ++index)
            resultIndexByName.emplace(result.drones[index].name, index);
    }

    for (const ExpectedDrone& expectedDrone : definition.expectedDrones)
    {
        const auto found = resultIndexByName.find(expectedDrone.name);
        if (found == resultIndexByName.end())
        {
            result.issues.push_back("예상 결과에 지정한 드론을 찾을 수 없습니다: "
                + expectedDrone.name);
            continue;
        }
        const ScenarioResult::Drone& drone = result.drones[found->second];
        if (drone.communicationStatus != DroneCommunicationStatus::Connected)
            continue;
        const std::string dronePrefix = definition.hasExpectedDrones
            ? expectedDrone.name + " 드론의 "
            : "";
        if (drone.finalState.mode != expectedDrone.mode)
            result.issues.push_back(FlightModeMismatchMessage(
                expectedDrone, drone, definition.hasExpectedDrones));
        if (glm::length(drone.finalState.positionMeters - expectedDrone.positionMeters)
            > definition.toleranceMeters)
        {
            result.issues.push_back(dronePrefix + "최종 위치가 허용 오차를 벗어났습니다.");
        }
        if (expectedDrone.orientation
            && OrientationErrorRadians(drone.finalState.orientation, *expectedDrone.orientation)
                > definition.orientationToleranceRadians)
        {
            result.issues.push_back(dronePrefix + "최종 자세가 허용 각도 오차를 벗어났습니다.");
        }
    }
    if (definition.hasExpectedDrones
        && definition.expectedDrones.size() != result.drones.size())
    {
        result.issues.push_back("expected.drones 개수가 시나리오 드론 수와 다릅니다.");
    }
}

void CheckExpectedSeparation(
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    if (definition.minimumDroneSeparationMeters
        && result.minimumDroneSeparationMeters
        && *result.minimumDroneSeparationMeters
            < *definition.minimumDroneSeparationMeters)
    {
        result.issues.push_back("드론 사이 최소 거리가 설정한 안전 간격보다 짧았습니다.");
    }
}

void CheckExpectedCamera(
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    if (definition.expectedCameraStatus
        && result.finalCamera.status != *definition.expectedCameraStatus)
    {
        result.issues.push_back("최종 카메라 상태가 예상값과 다릅니다.");
    }
    if (definition.expectedFrameCount
        && result.finalCamera.frameCount != *definition.expectedFrameCount)
    {
        result.issues.push_back("카메라 프레임 수가 예상값과 다릅니다.");
    }
}

void CheckExpectedResults(
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    if (result.mission)
    {
        result.mission->passed = std::all_of(
            result.mission->tasks.begin(), result.mission->tasks.end(),
            [](const ScenarioResult::MissionTask& task) { return task.completed; });
    }

    CheckExpectedDroneResults(definition, result);
    CheckExpectedSeparation(definition, result);
    CheckExpectedCamera(definition, result);
    result.passed = result.issues.empty();
}
}

ScenarioResult RunScenario(std::string_view jsonText)
{
    ScenarioDefinition definition = detail::ReadScenarioDefinition(jsonText);
    const std::uint64_t tickCount = definition.tickCount;
    SimulationCore simulation{definition.dynamics};
    if (definition.windVelocity)
        simulation.SetWindVelocityMetersPerSecond(*definition.windVelocity);

    DroneIds droneIds = CreateScenarioDrones(definition, simulation);
    DroneObservations observations;
    for (const auto& [droneName, droneId] : droneIds)
        observations.emplace(droneName, DroneObservation{
            DroneCommunicationStatus::Connected, 0, simulation.State(droneId)});

    ScenarioResult result;
    result.mission = std::move(definition.mission);
    std::size_t nextCommand = 0;
    std::size_t nextCameraEvent = 0;
    std::unordered_map<std::string, std::uint64_t> lastDeliveredSequence;
    std::vector<glm::dvec3> dronePositionsMeters;
    if (definition.minimumDroneSeparationMeters)
        dronePositionsMeters.reserve(definition.drones.size());

    RecordMinimumDroneSeparation(
        definition, simulation, droneIds, dronePositionsMeters, result);
    for (std::uint64_t tick = 0; tick < tickCount; ++tick)
    {
        UpdateDroneObservations(tick, definition, simulation, droneIds,
            observations, result);
        ApplyScheduledCommands(tick, definition.commands, simulation, droneIds,
            observations, nextCommand, lastDeliveredSequence, result);
        UpdateMissionCommands(tick, result.mission, simulation, droneIds,
            observations, definition, result);
        UpdateFormationLinks(tick, definition.formationLinks, simulation, droneIds,
            observations, result);
        ApplyCameraEvents(tick, definition.cameraEvents, simulation, nextCameraEvent, result);
        simulation.AdvanceOneTick();
        RecordMinimumDroneSeparation(
            definition, simulation, droneIds, dronePositionsMeters, result);
        UpdateMissionResults(tick, result.mission, simulation, droneIds,
            observations, result);
    }

    for (const auto& [droneName, droneId] : droneIds)
    {
        DroneObservation& observation = observations.at(droneName);
        if (observation.status == DroneCommunicationStatus::Connected)
        {
            observation.lastKnownState = simulation.State(droneId);
            observation.lastHeartbeatTick = tickCount;
        }
    }
    PopulateFinalSnapshot(definition, simulation, observations, result);
    CheckExpectedResults(definition, result);
    return result;
}

}
