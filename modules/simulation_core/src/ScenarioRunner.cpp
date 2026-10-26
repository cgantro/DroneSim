#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/BatteryModel.h"
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
#include <unordered_set>
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
using detail::CollisionAvoidanceSettings;
using detail::EnergyModelSettings;
using detail::ScheduledCameraEvent;
using detail::ScheduledCommand;
using detail::DroneCommunicationOutage;
using DroneIds = std::unordered_map<std::string, DroneId>;
using DroneCommunicationStatus = ScenarioResult::Drone::CommunicationStatus;

void RecordTrajectorySample(const ScenarioDefinition& definition,
    const SimulationCore& simulation, const DroneIds& droneIds,
    std::uint64_t tick, ScenarioResult& result)
{
    ScenarioResult::TrajectorySample sample;
    sample.tick = tick;
    sample.timeSeconds = static_cast<double>(tick) * DroneModel::fixedStepSeconds;
    sample.drones.reserve(droneIds.size());

    const auto addDrone = [&](const std::string& name)
    {
        sample.drones.emplace_back(name, simulation.State(droneIds.at(name)));
    };
    if (definition.drones.empty())
        addDrone(definition.primaryDroneName);
    else
        for (const ScenarioDrone& drone : definition.drones)
            addDrone(drone.name);

    result.trajectory.push_back(std::move(sample));
}

struct DroneObservation
{
    DroneCommunicationStatus status = DroneCommunicationStatus::Connected;
    std::uint64_t lastHeartbeatTick = 0;
    DroneState lastKnownState;
    std::optional<double> lastKnownBatteryPercent;
    bool holdPositionApplied = false;
};

using DroneObservations = std::unordered_map<std::string, DroneObservation>;
using DroneNameSet = std::unordered_set<std::string>;

bool CanReceiveWaypoint(FlightMode mode) noexcept
{
    return mode == FlightMode::Hovering
        || mode == FlightMode::Flying
        || mode == FlightMode::FollowingWaypoint;
}

void UpdateEnergyPolicy(
    std::uint64_t tick,
    std::optional<ScenarioResult::Mission>& mission,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    const auto handleDrone = [&](const std::string& droneName)
    {
        BatteryModel& energy = simulation.Battery(droneIds.at(droneName));
        if (energy.IsReturningForLowBattery()
            || !energy.ShouldReturnForLowBattery())
            return;

        energy.MarkReturningForLowBattery();
        result.eventLog.push_back("tick " + std::to_string(tick) + ": "
            + droneName + " 드론이 배터리 예비량에 도달해 임무를 넘기고 착륙합니다.");

        if (mission)
        {
            for (ScenarioResult::MissionTask& task : mission->tasks)
            {
                if (task.droneName != droneName || task.completed || task.failed)
                    continue;
                task.droneName.clear();
                task.started = false;
                result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
                    + task.name + "' 임무를 대기열에 돌려놓았습니다.");
            }
        }

        const FlightMode mode = simulation.State(droneIds.at(droneName)).mode;
        if (mode != FlightMode::Grounded && mode != FlightMode::Landing)
        {
            const CommandResult landResult = simulation.Apply(droneIds.at(droneName),
                {DroneCommand::Kind::Land});
            if (landResult != CommandResult::Accepted)
                result.issues.push_back(droneName + " 드론을 낮은 배터리로 착륙시키지 못했습니다.");
        }
    };

    if (definition.drones.empty())
    {
        handleDrone(definition.primaryDroneName);
        return;
    }
    for (const ScenarioDrone& drone : definition.drones)
        handleDrone(drone.name);
}

bool IsConnected(const DroneObservations& observations, const std::string& droneName)
{
    return observations.at(droneName).status == DroneCommunicationStatus::Connected;
}

bool IsUnknown(const DroneObservations& observations, const std::string& droneName)
{
    return observations.at(droneName).status == DroneCommunicationStatus::Unknown;
}

const glm::dvec3& CurrentWaypoint(const ScenarioResult::MissionTask& task)
{
    return task.waypointsMeters[task.currentWaypointIndex];
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
    SimulationCore& simulation,
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
            observation.holdPositionApplied = false;
            observation.lastKnownBatteryPercent = simulation.HasBattery(droneId)
                ? std::optional<double>{simulation.Battery(droneId).RemainingPercent()}
                : std::nullopt;
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
        if (newStatus == DroneCommunicationStatus::Unknown
            && outage->onTimeout == DroneCommunicationOutage::OnTimeout::HoldPosition
            && !observation.holdPositionApplied)
        {
            // 이륙 중에는 위치 명령을 받을 수 없으므로, 가능한 비행 상태가 될 때까지 재시도한다.
            const DroneState& state = simulation.State(droneId);
            if (CanReceiveWaypoint(state.mode))
            {
                const CommandResult holdResult = simulation.Apply(droneId,
                    {DroneCommand::Kind::SetWaypoint, state.positionMeters});
                if (holdResult == CommandResult::Accepted)
                {
                    observation.holdPositionApplied = true;
                    result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                        + droneName + " 드론이 통신 단절 안전 동작으로 현재 위치를 유지합니다.");
                }
                else
                {
                    result.issues.push_back(droneName
                        + " 드론에 통신 단절 안전 동작을 적용하지 못했습니다.");
                }
            }
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
        const DroneId id = simulation.PrimaryDroneId();
        simulation.SetCameraModel(id, definition.defaultCameraModel);
        droneIds.emplace(definition.primaryDroneName, id);
        return droneIds;
    }

    const DroneId unusedDefaultDrone = simulation.PrimaryDroneId();
    for (const ScenarioDrone& drone : definition.drones)
    {
        const DroneId id = simulation.AddDrone(
            drone.initialPositionMeters, drone.initialYawRadians);
        simulation.SetCameraModel(
            id, drone.cameraModel.value_or(definition.defaultCameraModel));
        droneIds.emplace(drone.name, id);
        if (drone.hasFullInitialState)
        {
            DroneState initialState;
            initialState.positionMeters = drone.initialPositionMeters;
            initialState.velocityMetersPerSecond = drone.initialVelocityMetersPerSecond;
            initialState.orientation = drone.initialOrientation.value_or(
                glm::angleAxis(drone.initialYawRadians, glm::dvec3{0.0, 0.0, 1.0}));
            initialState.angularVelocityRadiansPerSecond =
                drone.initialAngularVelocityRadiansPerSecond;
            initialState.motorSpeedsRadiansPerSecond =
                drone.initialMotorSpeedsRadiansPerSecond;
            initialState.mode = drone.initialMode;
            simulation.SetInitialState(id, initialState);
        }
    }

    if (!simulation.RemoveDrone(unusedDefaultDrone))
        throw std::logic_error("시나리오용 기본 드론을 교체하지 못했습니다.");
    return droneIds;
}

/** @brief 카메라 동작을 적용하고 실행 기록에 쓸 이름을 돌려준다. */
std::string_view ApplyCameraEvent(
    SimulationCore& simulation, DroneId droneId, detail::CameraEventAction action)
{
    using Action = detail::CameraEventAction;
    switch (action)
    {
    case Action::Enable:
        simulation.SetCameraEnabled(droneId, true);
        return "enable";
    case Action::Disable:
        simulation.SetCameraEnabled(droneId, false);
        return "disable";
    case Action::Fault:
        simulation.SetCameraFault(droneId, true);
        return "fault";
    case Action::Recover:
        simulation.SetCameraFault(droneId, false);
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
            if (simulation.HasBattery(droneIds.at(droneName))
                && simulation.Battery(droneIds.at(droneName)).IsReturningForLowBattery())
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
            findNearestDrone(CurrentWaypoint(task), true);
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
            findNearestDrone(CurrentWaypoint(task), false);
        if (!availableDrone)
            continue;
        task.droneName = *availableDrone;
        task.assignmentHistory.push_back(*availableDrone);
        ++task.reassignmentCount;
        result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
            + task.name + "' 대기 중인 구역을 " + *availableDrone
            + " 드론에 배정했습니다.");
    }

    for (auto taskIterator = mission->tasks.begin();
         taskIterator != mission->tasks.end(); ++taskIterator)
    {
        ScenarioResult::MissionTask& task = *taskIterator;
        if (task.completed || task.failed || tick < task.startTick
            || task.droneName.empty() || !IsConnected(observations, task.droneName))
            continue;
        const bool earlierTaskIsActive = std::any_of(mission->tasks.begin(),
            taskIterator, [&task](const ScenarioResult::MissionTask& previous)
            {
                return previous.droneName == task.droneName
                    && !previous.completed && !previous.failed;
            });
        if (earlierTaskIsActive)
            continue;
        const DroneId droneId = droneIds.at(task.droneName);
        const FlightMode mode = simulation.State(droneId).mode;
        if (!CanReceiveWaypoint(mode))
        {
            // 이륙 중이면 정해 둔 기한까지 기다린 뒤 시작 가능 여부를 판정한다.
            if (!task.started && mode == FlightMode::TakingOff
                && tick < task.deadlineTick)
                continue;

            task.failed = true;
            if (task.started)
                result.issues.push_back(task.droneName
                    + " 드론이 비행을 멈춰 임무를 이어 가지 못했습니다.");
            else if (tick >= task.deadlineTick)
                result.issues.push_back(task.droneName
                    + " 드론이 임무 시작 기한까지 이륙하지 못했습니다.");
            else
                result.issues.push_back(task.droneName
                    + " 드론이 비행 중이어서 임무를 시작하지 못했습니다.");
            continue;
        }
        if (!task.started)
        {
            task.started = true;
            if (!mission->startedTick)
                mission->startedTick = tick;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.role + "' 임무를 시작했습니다.");
        }

        const CommandResult moveResult = simulation.Apply(droneId,
            {DroneCommand::Kind::SetWaypoint, CurrentWaypoint(task)});
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

void UpdateCollisionAvoidance(
    std::uint64_t tick,
    const ScenarioDefinition& definition,
    std::optional<ScenarioResult::Mission>& mission,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    DroneNameSet& dronesAvoiding,
    std::unordered_map<std::string, glm::dvec3>& activeAvoidanceDirections,
    std::unordered_map<std::string, glm::dvec3>& activeAvoidanceTargets,
    ScenarioResult& result)
{
    if (!definition.collisionAvoidance || !mission)
        return;

    const CollisionAvoidanceSettings& settings = *definition.collisionAvoidance;
    std::unordered_map<std::string, glm::dvec3> avoidanceDirections;
    DroneNameSet currentlyAvoiding;
    for (std::size_t firstIndex = 0; firstIndex < definition.drones.size(); ++firstIndex)
    {
        const std::string& firstName = definition.drones[firstIndex].name;
        const bool firstConnected = IsConnected(observations, firstName);
        const auto firstTask = std::find_if(mission->tasks.begin(), mission->tasks.end(),
            [firstConnected, &firstName](const ScenarioResult::MissionTask& task)
            {
                return firstConnected && task.droneName == firstName && task.started
                    && !task.completed && !task.failed;
            });
        const bool firstHasTask = firstTask != mission->tasks.end()
            && CanReceiveWaypoint(simulation.State(droneIds.at(firstName)).mode);
        for (std::size_t secondIndex = firstIndex + 1;
            secondIndex < definition.drones.size(); ++secondIndex)
        {
            const std::string& secondName = definition.drones[secondIndex].name;
            const bool secondConnected = IsConnected(observations, secondName);
            const auto secondTask = std::find_if(mission->tasks.begin(), mission->tasks.end(),
                [secondConnected, &secondName](const ScenarioResult::MissionTask& task)
                {
                    return secondConnected && task.droneName == secondName && task.started
                        && !task.completed && !task.failed;
                });
            const bool secondHasTask = secondTask != mission->tasks.end()
                && CanReceiveWaypoint(simulation.State(droneIds.at(secondName)).mode);
            const std::string* yieldingDrone = nullptr;
            if (firstConnected && secondConnected)
            {
                if (!firstHasTask && !secondHasTask)
                    continue;
                // 임무가 없는 연결 드론도 장애물로 본다. 둘 다 임무 중이면
                // 목록에서 뒤에 있는 드론이 양보해 결과를 반복할 수 있게 한다.
                yieldingDrone = firstHasTask && !secondHasTask
                    ? &firstName : &secondName;
            }
            else if (firstConnected && firstHasTask)
            {
                // 통신이 끊긴 드론은 마지막으로 받은 위치와 속도를 장애물처럼 취급한다.
                yieldingDrone = &firstName;
            }
            else if (secondConnected && secondHasTask)
            {
                yieldingDrone = &secondName;
            }
            else
            {
                continue;
            }

            const auto stateForAvoidance = [&](const std::string& droneName,
                                                bool connected)
            {
                if (connected)
                    return simulation.State(droneIds.at(droneName));
                DroneState state = observations.at(droneName).lastKnownState;
                // 통신이 오래 끊긴 뒤에는 예전 속도로 현재 위치를 외삽할 수 없다.
                // 마지막으로 확인한 위치를 정지 장애물로 보고 회피 경로를 계산한다.
                state.velocityMetersPerSecond = glm::dvec3{0.0};
                return state;
            };
            const DroneState firstState = stateForAvoidance(firstName, firstConnected);
            const DroneState secondState = stateForAvoidance(secondName, secondConnected);

            const glm::dvec3 relativePosition =
                secondState.positionMeters - firstState.positionMeters;
            const glm::dvec3 relativeVelocity =
                secondState.velocityMetersPerSecond - firstState.velocityMetersPerSecond;
            const double relativeSpeedSquared = glm::dot(relativeVelocity, relativeVelocity);
            const double closestTime = relativeSpeedSquared > 1e-9
                ? std::clamp(-glm::dot(relativePosition, relativeVelocity)
                    / relativeSpeedSquared, 0.0, settings.predictionHorizonSeconds)
                : 0.0;
            const double predictedDistance = glm::length(
                relativePosition + relativeVelocity * closestTime);
            if (!result.minimumPredictedDroneSeparationMeters
                || predictedDistance < *result.minimumPredictedDroneSeparationMeters)
            {
                result.minimumPredictedDroneSeparationMeters = predictedDistance;
            }
            const bool alreadyAvoiding = activeAvoidanceDirections.contains(*yieldingDrone);
            // 두 드론이 모두 임무 중일 때는 충분히 멀어진 뒤 회피를 풀어 떨림을 막는다.
            // 한쪽이 정지 장애물이면 안전거리까지만 확보해도 회피를 풀어야
            // 두 목표점 간격이 안전거리보다 좁은 경우 임무가 영원히 멈추지 않는다.
            const double releaseDistance = settings.safetyDistanceMeters
                * (firstHasTask && secondHasTask ? 1.5 : 1.0);
            const double avoidanceThreshold = alreadyAvoiding
                ? releaseDistance : settings.safetyDistanceMeters;
            if (predictedDistance > avoidanceThreshold)
                continue;

            // 입력 순서상 뒤에 있는 드론이 비켜서 결과를 반복 실행해도 같은 선택을 한다.
            glm::dvec3 side(-relativeVelocity.y, relativeVelocity.x, 0.0);
            if (glm::length(side) < 1e-6)
                side = {-relativePosition.y, relativePosition.x, 0.0};
            if (glm::length(side) < 1e-6)
                side = {0.0, 1.0, 0.0};
            side = glm::normalize(side);
            if (*yieldingDrone == firstName)
                side = -side;

            const auto [activeDirection, directionInserted] =
                activeAvoidanceDirections.try_emplace(*yieldingDrone, side);
            if (!directionInserted)
                side = activeDirection->second;
            auto [direction, inserted] = avoidanceDirections.try_emplace(*yieldingDrone, side);
            if (!inserted)
                direction->second += side;
            currentlyAvoiding.insert(*yieldingDrone);
        }
    }

    for (const std::string& droneName : currentlyAvoiding)
    {
        const auto task = std::find_if(mission->tasks.begin(), mission->tasks.end(),
            [&droneName](const ScenarioResult::MissionTask& candidate)
            {
                return candidate.droneName == droneName && candidate.started
                    && !candidate.completed && !candidate.failed;
            });
        if (task == mission->tasks.end())
            continue;

        glm::dvec3 direction = avoidanceDirections.at(droneName);
        if (glm::length(direction) < 1e-6)
            direction = {0.0, 1.0, 0.0};
        direction = glm::normalize(direction);
        const glm::dvec3 currentPosition =
            simulation.State(droneIds.at(droneName)).positionMeters;
        // 피난 목표는 한 번 정한 뒤 위험 구간을 벗어날 때까지 유지한다.
        const auto avoidanceTarget = activeAvoidanceTargets.try_emplace(
            droneName, currentPosition + direction * settings.lateralOffsetMeters).first;
        const CommandResult commandResult = simulation.Apply(droneIds.at(droneName),
            {DroneCommand::Kind::SetWaypoint, avoidanceTarget->second});
        if (commandResult != CommandResult::Accepted)
        {
            result.issues.push_back(droneName + " 드론에 충돌 회피 경로를 전달하지 못했습니다.");
            continue;
        }
        if (!dronesAvoiding.contains(droneName))
        {
            ++result.collisionAvoidanceEvents;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론이 예상 충돌을 피해 옆 경로로 이동합니다.");
        }
    }

    for (const std::string& droneName : dronesAvoiding)
    {
        if (!currentlyAvoiding.contains(droneName))
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론이 위험 구간을 벗어나 조사 경로로 돌아갑니다.");
        }
    }
    std::erase_if(activeAvoidanceDirections,
        [&currentlyAvoiding](const auto& entry)
        {
            return !currentlyAvoiding.contains(entry.first);
        });
    std::erase_if(activeAvoidanceTargets,
        [&currentlyAvoiding](const auto& entry)
        {
            return !currentlyAvoiding.contains(entry.first);
        });
    dronesAvoiding = std::move(currentlyAvoiding);
}

void ApplyCameraEvents(
    std::uint64_t tick,
    const std::vector<ScheduledCameraEvent>& cameraEvents,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    std::string_view primaryDroneName,
    std::size_t& nextCameraEvent,
    ScenarioResult& result)
{
    while (nextCameraEvent < cameraEvents.size()
        && cameraEvents[nextCameraEvent].tick == tick)
    {
        const ScheduledCameraEvent& event = cameraEvents[nextCameraEvent++];
        const std::string droneName = event.droneName.empty()
            ? std::string(primaryDroneName) : event.droneName;
        const std::string_view actionName = ApplyCameraEvent(
            simulation, droneIds.at(droneName), event.action);
        result.eventLog.push_back("tick " + std::to_string(tick) + ": " + droneName
            + " 카메라 " + std::string(actionName));
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

        const DroneId droneId = droneIds.at(task.droneName);
        if (task.inspectionSurface && !task.firstGeometryCaptureTick)
        {
            const CameraState& camera = simulation.Camera(droneId);
            if (camera.lastFrameTick == simulation.TickCount())
            {
                task.lastCaptureAssessment = EoCameraModel::AssessCapture(
                    camera, *task.inspectionSurface, task.inspectionOccluders);
                if (task.lastCaptureAssessment == CaptureAssessment::Ready)
                {
                    task.firstGeometryCaptureTick = camera.lastFrameTick;
                    result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                        + task.droneName + " 드론의 '" + task.name
                        + "' 점검면이 카메라 프레임 조건을 만족했습니다.");
                }
            }
            else
            {
                task.lastCaptureAssessment = camera.status == CameraStatus::Active
                    ? CaptureAssessment::NoNewFrame
                    : CaptureAssessment::CameraUnavailable;
            }
        }

        const glm::dvec3 distance = simulation.State(droneId).positionMeters
            - CurrentWaypoint(task);
        const bool reachedWaypoint = glm::length(distance) <= task.toleranceMeters;
        if (reachedWaypoint && task.currentWaypointIndex + 1 < task.waypointsMeters.size())
        {
            ++task.currentWaypointIndex;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.name + "' 경유점 "
                + std::to_string(task.currentWaypointIndex + 1) + "/"
                + std::to_string(task.waypointsMeters.size()) + "에 도착했습니다.");
        }
        else if (reachedWaypoint
            && (!task.inspectionSurface || task.firstGeometryCaptureTick))
        {
            task.completed = true;
            task.completedTick = tick;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.role + "' 임무를 마쳤습니다.");
        }
        else if (tick >= task.deadlineTick)
        {
            task.failed = true;
            if (task.inspectionSurface && !task.firstGeometryCaptureTick)
            {
                result.issues.push_back(task.droneName
                    + " 드론이 임무 기한 안에 점검면 촬영 조건을 만족하지 못했습니다.");
            }
            else
            {
                result.issues.push_back(task.droneName
                    + " 드론이 임무 기한 안에 목표에 도달하지 못했습니다.");
            }
        }
    }
}

void PopulateFinalSnapshot(
    const ScenarioDefinition& definition,
    const SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    const std::unordered_map<std::string, double>& flightDistancesMeters,
    const std::unordered_map<std::string, double>& flightTimesSeconds,
    ScenarioResult& result)
{
    result.tickCount = simulation.TickCount();
    result.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    result.windVelocityMetersPerSecond = definition.windVelocity;
    if (definition.dynamics == SimulationDynamics::RigidBody)
    {
        const QuadrotorParameters& model = definition.quadrotorParameters;
        result.physicalModel = ScenarioResult::PhysicalModel{
            model.massKilograms,
            model.principalInertiaKilogramMetersSquared,
            model.thrustCoefficientNewtonsPerSpeedSquared,
            model.momentToThrustRatio,
            model.motorTimeConstantUpSeconds,
            model.motorTimeConstantDownSeconds,
            model.maximumMotorSpeedRadiansPerSecond,
            definition.quadrotorParametersCustomized};
        const QuadrotorControllerParameters& controller = definition.controllerParameters;
        result.controllerSettings = ScenarioResult::ControllerSettings{
            controller.positionGain,
            controller.velocityGain,
            controller.attitudeGain,
            controller.angularRateGain,
            definition.controllerParametersCustomized};
    }
    const auto appendDrone = [&](const std::string& droneName)
    {
        const DroneObservation& observation = observations.at(droneName);
        ScenarioResult::Drone drone{
            droneName, observation.lastKnownState,
            observation.status, observation.lastHeartbeatTick};
        drone.finalCamera = simulation.Camera(droneIds.at(droneName));
        drone.flightDistanceMeters = flightDistancesMeters.at(droneName);
        drone.flightTimeSeconds = flightTimesSeconds.at(droneName);
        drone.reportedBatteryPercent = observation.lastKnownBatteryPercent;
        if (simulation.HasBattery(droneIds.at(droneName)))
        {
            const BatteryModel& battery = simulation.Battery(droneIds.at(droneName));
            drone.batteryCapacityWattHours = battery.CapacityWattHours();
            drone.energyConsumedWattHours = battery.ConsumedWattHours();
            drone.batteryRemainingWattHours = battery.RemainingWattHours();
            drone.batteryPercent = battery.RemainingPercent();
            drone.returnedForLowBattery = battery.IsReturningForLowBattery();
        }
        result.drones.push_back(std::move(drone));
    };

    if (definition.drones.empty())
    {
        appendDrone(definition.primaryDroneName);
    }
    else
        for (const ScenarioDrone& drone : definition.drones)
            appendDrone(drone.name);

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
        std::size_t totalWaypointCount = 0;
        std::size_t visitedWaypointCount = 0;
        for (const ScenarioResult::MissionTask& task : result.mission->tasks)
        {
            totalWaypointCount += task.waypointsMeters.size();
            visitedWaypointCount += task.completed
                ? task.waypointsMeters.size() : task.currentWaypointIndex;
            if (task.completed)
            {
                ++result.mission->completedTaskCount;
                if (task.completedTick
                    && (!result.mission->completedTick
                        || *task.completedTick > *result.mission->completedTick))
                    result.mission->completedTick = task.completedTick;
            }
        }
        result.mission->completionPercent = 100.0
            * static_cast<double>(result.mission->completedTaskCount)
            / static_cast<double>(result.mission->tasks.size());
        result.mission->coveragePercent = totalWaypointCount == 0 ? 0.0
            : 100.0 * static_cast<double>(visitedWaypointCount)
                / static_cast<double>(totalWaypointCount);
        result.mission->passed = std::all_of(
            result.mission->tasks.begin(), result.mission->tasks.end(),
            [](const ScenarioResult::MissionTask& task) { return task.completed; });
        const bool hasFailedTask = std::any_of(
            result.mission->tasks.begin(), result.mission->tasks.end(),
            [](const ScenarioResult::MissionTask& task) { return task.failed; });
        result.mission->outcome = hasFailedTask
            ? ScenarioResult::Outcome::Failed
            : result.mission->passed
                ? ScenarioResult::Outcome::Passed
                : ScenarioResult::Outcome::Inconclusive;
    }

    CheckExpectedDroneResults(definition, result);
    CheckExpectedSeparation(definition, result);
    CheckExpectedCamera(definition, result);
    if (!result.issues.empty())
        result.outcome = ScenarioResult::Outcome::Failed;
    else if (result.mission)
        result.outcome = result.mission->outcome;
    else
        result.outcome = ScenarioResult::Outcome::Passed;
    result.passed = result.outcome == ScenarioResult::Outcome::Passed;
}
}

std::string_view ToString(ScenarioResult::Outcome outcome) noexcept
{
    switch (outcome)
    {
    case ScenarioResult::Outcome::Passed: return "passed";
    case ScenarioResult::Outcome::Failed: return "failed";
    case ScenarioResult::Outcome::Inconclusive: return "inconclusive";
    }
    return "inconclusive";
}

ScenarioResult RunScenario(std::string_view jsonText)
{
    ScenarioDefinition definition = detail::ReadScenarioDefinition(jsonText);
    const std::uint64_t tickCount = definition.tickCount;
    SimulationCore simulation{definition.dynamics,
        definition.quadrotorParameters, definition.controllerParameters};
    if (definition.windVelocity)
        simulation.SetWindVelocityMetersPerSecond(*definition.windVelocity);

    DroneIds droneIds = CreateScenarioDrones(definition, simulation);
    DroneObservations observations;
    std::unordered_map<std::string, glm::dvec3> previousPositions;
    std::unordered_map<std::string, double> flightDistancesMeters;
    std::unordered_map<std::string, double> flightTimesSeconds;
    for (const auto& [droneName, droneId] : droneIds)
    {
        observations.emplace(droneName, DroneObservation{
            DroneCommunicationStatus::Connected, 0, simulation.State(droneId)});
        previousPositions.emplace(droneName, simulation.State(droneId).positionMeters);
        flightDistancesMeters.emplace(droneName, 0.0);
        flightTimesSeconds.emplace(droneName, 0.0);
    }
    if (definition.energyModel)
    {
        const auto addEnergy = [&](const std::string& droneName,
            const ScenarioDrone* scenarioDrone)
        {
            const double initialPercent = scenarioDrone
                ? scenarioDrone->initialBatteryPercent : 100.0;
            BatteryModelParameters parameters = *definition.energyModel;
            if (scenarioDrone && scenarioDrone->batteryCapacityWattHours)
                parameters.capacityWattHours = *scenarioDrone->batteryCapacityWattHours;
            const DroneId droneId = droneIds.at(droneName);
            simulation.ConfigureBattery(droneId, parameters, initialPercent);
            observations.at(droneName).lastKnownBatteryPercent =
                simulation.Battery(droneId).RemainingPercent();
        };
        if (definition.drones.empty())
        {
            addEnergy(definition.primaryDroneName, nullptr);
        }
        else
        {
            for (const ScenarioDrone& drone : definition.drones)
                addEnergy(drone.name, &drone);
        }
    }

    ScenarioResult result;
    result.mission = std::move(definition.mission);
    if (definition.trajectorySampleEveryTicks)
    {
        const std::uint64_t periodicSamples =
            tickCount / *definition.trajectorySampleEveryTicks + 1;
        const std::uint64_t sampleCount = periodicSamples
            + (tickCount % *definition.trajectorySampleEveryTicks != 0 ? 1 : 0);
        result.trajectory.reserve(static_cast<std::size_t>(sampleCount));
    }
    std::size_t nextCommand = 0;
    std::size_t nextCameraEvent = 0;
    std::unordered_map<std::string, std::uint64_t> lastDeliveredSequence;
    DroneNameSet dronesAvoiding;
    std::unordered_map<std::string, glm::dvec3> activeAvoidanceDirections;
    std::unordered_map<std::string, glm::dvec3> activeAvoidanceTargets;
    std::vector<glm::dvec3> dronePositionsMeters;
    if (definition.minimumDroneSeparationMeters)
        dronePositionsMeters.reserve(definition.drones.size());

    RecordMinimumDroneSeparation(
        definition, simulation, droneIds, dronePositionsMeters, result);
    if (definition.trajectorySampleEveryTicks)
        RecordTrajectorySample(definition, simulation, droneIds, 0, result);
    for (std::uint64_t tick = 0; tick < tickCount; ++tick)
    {
        UpdateDroneObservations(tick, definition, simulation, droneIds,
            observations, result);
        ApplyScheduledCommands(tick, definition.commands, simulation, droneIds,
            observations, nextCommand, lastDeliveredSequence, result);
        if (definition.energyModel)
        {
            UpdateEnergyPolicy(tick, result.mission,
                simulation, droneIds, definition, result);
        }
        UpdateMissionCommands(tick, result.mission, simulation, droneIds,
            observations, definition, result);
        UpdateFormationLinks(tick, definition.formationLinks, simulation, droneIds,
            observations, result);
        UpdateCollisionAvoidance(tick, definition, result.mission, simulation,
            droneIds, observations, dronesAvoiding, activeAvoidanceDirections,
            activeAvoidanceTargets, result);
        ApplyCameraEvents(tick, definition.cameraEvents, simulation, droneIds,
            definition.primaryDroneName, nextCameraEvent, result);
        for (const auto& [droneName, droneId] : droneIds)
        {
            if (simulation.State(droneId).mode != FlightMode::Grounded)
                flightTimesSeconds.at(droneName) += DroneModel::fixedStepSeconds;
        }
        simulation.AdvanceOneTick();
        for (const auto& [droneName, droneId] : droneIds)
        {
            const glm::dvec3& currentPosition =
                simulation.State(droneId).positionMeters;
            flightDistancesMeters.at(droneName) += glm::distance(
                previousPositions.at(droneName), currentPosition);
            previousPositions.at(droneName) = currentPosition;
        }
        RecordMinimumDroneSeparation(
            definition, simulation, droneIds, dronePositionsMeters, result);
        UpdateMissionResults(tick, result.mission, simulation, droneIds,
            observations, result);
        const std::uint64_t completedTick = tick + 1;
        if (definition.trajectorySampleEveryTicks
            && (completedTick % *definition.trajectorySampleEveryTicks == 0
                || completedTick == tickCount))
        {
            RecordTrajectorySample(
                definition, simulation, droneIds, completedTick, result);
        }
    }

    for (const auto& [droneName, droneId] : droneIds)
    {
        DroneObservation& observation = observations.at(droneName);
        if (observation.status == DroneCommunicationStatus::Connected)
        {
            observation.lastKnownState = simulation.State(droneId);
            observation.lastHeartbeatTick = tickCount;
            if (simulation.HasBattery(droneId))
                observation.lastKnownBatteryPercent =
                    simulation.Battery(droneId).RemainingPercent();
        }
    }
    PopulateFinalSnapshot(definition, simulation, droneIds, observations,
        flightDistancesMeters, flightTimesSeconds, result);
    CheckExpectedResults(definition, result);
    return result;
}

}
