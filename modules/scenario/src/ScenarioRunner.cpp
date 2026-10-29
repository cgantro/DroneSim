#include "scenario/ScenarioRunner.h"
#include "simulation_core/BatteryModel.h"
#include "simulation_core/QuadrotorFlightController.h"
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

namespace aerosimlab::scenario
{
using namespace aerosimlab::simulation;

namespace
{
using detail::ExpectedDrone;
using detail::ScenarioDefinition;
using detail::ScenarioDrone;
using detail::CollisionAvoidanceSettings;
using detail::EnergyModelSettings;
using detail::ScheduledCameraEvent;
using detail::TickCommand;
using detail::DroneCommunicationOutage;
using DroneIds = std::unordered_map<std::string, DroneId>;
using DroneHomePositions = std::unordered_map<std::string, glm::dvec3>;
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
    const DroneHomePositions& homePositions,
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    const auto handleDrone = [&](const std::string& droneName)
    {
        const DroneId droneId = droneIds.at(droneName);
        BatteryModel& energy = simulation.Battery(droneId);
        if (!energy.IsReturningForLowBattery())
        {
            if (!energy.ShouldReturnForLowBattery())
                return;

            energy.MarkReturningForLowBattery();
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론이 배터리 예비량에 도달해 출발 지점으로 돌아갑니다.");

            if (mission)
            {
                for (ScenarioResult::MissionTask& task : mission->tasks)
                {
                    if (task.droneName != droneName || task.IsCompleted() || task.failed)
                        continue;
                    task.failed = true;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
                + task.name + "' 임무는 미완료로 남습니다.");
                }
            }
        }

        const DroneState& state = simulation.State(droneId);
        if (!CanReceiveWaypoint(state.mode))
            return;

        const glm::dvec3& returnPosition = homePositions.at(droneName);
        if (glm::distance(state.positionMeters, returnPosition) <= 0.2)
        {
            const CommandResult landResult = simulation.Apply(droneId,
                {DroneCommand::Kind::Land});
            if (landResult != CommandResult::Accepted)
                result.issues.push_back(droneName
                    + " 드론이 귀환 지점에서 착륙하지 못했습니다.");
            return;
        }

        const CommandResult returnResult = simulation.Apply(droneId,
            {DroneCommand::Kind::SetWaypoint, returnPosition});
        if (returnResult != CommandResult::Accepted)
            result.issues.push_back(droneName + " 드론이 배터리 귀환 경로를 받지 못했습니다.");
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

void ApplyScheduledCommands(
    std::uint64_t tick,
    const std::vector<TickCommand>& commands,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    std::size_t& nextCommand,
    ScenarioResult& result)
{
    while (nextCommand < commands.size() && commands[nextCommand].tick == tick)
    {
        const TickCommand& command = commands[nextCommand++];
        if (!IsConnected(observations, command.droneName))
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + command.droneName
                + " 드론에 명령을 전달하지 못했습니다. 마지막 위치는 이전 보고값입니다.");
            continue;
        }

        if (simulation.Apply(droneIds.at(command.droneName), command.command)
            != CommandResult::Accepted)
        {
            result.issues.push_back("tick " + std::to_string(tick) + "에서 "
                + command.droneName + " 드론 명령을 적용하지 못했습니다.");
        }
    }
}

void UpdateMissionCommands(
    std::uint64_t tick,
    std::optional<ScenarioResult::Mission>& mission,
    SimulationCore& simulation,
    const DroneIds& droneIds,
    const DroneObservations& observations,
    ScenarioResult& result)
{
    if (!mission)
        return;

    for (auto taskIterator = mission->tasks.begin();
         taskIterator != mission->tasks.end(); ++taskIterator)
    {
        ScenarioResult::MissionTask& task = *taskIterator;
        if (task.IsCompleted() || task.failed || tick < task.startTick
            || !IsConnected(observations, task.droneName))
            continue;
        const bool earlierTaskIsActive = std::any_of(mission->tasks.begin(),
            taskIterator, [&task](const ScenarioResult::MissionTask& previous)
            {
                return previous.droneName == task.droneName
                    && !previous.IsCompleted() && !previous.failed;
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

        const DroneCommand moveCommand{
            DroneCommand::Kind::SetWaypoint, CurrentWaypoint(task)};
        const CommandResult moveResult = simulation.Apply(droneId, moveCommand);
        if (moveResult != CommandResult::Accepted)
        {
            task.failed = true;
            result.issues.push_back(task.droneName + " 드론에 임무 목표를 전달하지 못했습니다.");
        }
    }
}

glm::dvec3 MissionWaypoint(
    const DroneState& state,
    const ScenarioResult::MissionTask* task,
    const std::unordered_map<std::string, glm::dvec3>& activeAvoidanceTargets,
    bool includeAvoidanceTarget)
{
    if (!task)
        return state.positionMeters;

    const auto avoidanceTarget = activeAvoidanceTargets.find(task->droneName);
    if (includeAvoidanceTarget && avoidanceTarget != activeAvoidanceTargets.end())
        return avoidanceTarget->second;
    return CurrentWaypoint(*task);
}

/** @brief 현재 임무 경유점을 향하는 목표 속도를 계산한다. */
glm::dvec3 MissionVelocityIntent(
    const DroneState& state,
    const ScenarioResult::MissionTask* task,
    const ScenarioDefinition& definition,
    const std::unordered_map<std::string, glm::dvec3>& activeAvoidanceTargets,
    bool includeAvoidanceTarget)
{
    if (!task)
        return state.velocityMetersPerSecond;

    const glm::dvec3 waypoint = MissionWaypoint(
        state, task, activeAvoidanceTargets, includeAvoidanceTarget);

    glm::dvec3 targetVelocity{0.0};
    if (definition.dynamics == SimulationDynamics::RigidBody)
    {
        targetVelocity = QuadrotorFlightController::WaypointVelocityTarget(
            state, waypoint, definition.controllerParameters);
    }
    else
    {
        const glm::dvec3 offset = waypoint - state.positionMeters;
        const double distance = glm::length(offset);
        if (distance > 1.0e-9)
            targetVelocity = offset
                * (DroneModel::maximumSpeedMetersPerSecond / distance);
    }

    if (definition.windVelocity)
        targetVelocity += *definition.windVelocity;
    return targetVelocity;
}

/** @brief 경유점까지 남은 거리를 목표 속도로 나눠 도착 시각을 예상한다. */
double TimeToMissionWaypoint(
    const DroneState& state,
    const ScenarioResult::MissionTask* task,
    const glm::dvec3& velocity,
    const ScenarioDefinition& definition,
    const std::unordered_map<std::string, glm::dvec3>& activeAvoidanceTargets,
    bool includeAvoidanceTarget)
{
    const double horizon = definition.collisionAvoidance->predictionHorizonSeconds;
    if (!task)
        return horizon;

    const double speed = glm::length(velocity);
    if (speed <= 1.0e-9)
        return 0.0;

    const glm::dvec3 waypoint = MissionWaypoint(
        state, task, activeAvoidanceTargets, includeAvoidanceTarget);
    const double distance = glm::distance(state.positionMeters, waypoint);
    return std::min(distance / speed, horizon);
}

struct PredictedApproach
{
    double distanceMeters;
    double timeSeconds;
};

/** @brief 각 드론이 경유점에 도착하면 멈춘다고 보고 가장 가까운 거리를 구한다. */
PredictedApproach ClosestApproachWithStops(
    const glm::dvec3& firstPosition,
    const glm::dvec3& firstVelocity,
    double firstTravelSeconds,
    const glm::dvec3& secondPosition,
    const glm::dvec3& secondVelocity,
    double secondTravelSeconds,
    double predictionHorizonSeconds)
{
    std::array breakpoints{
        0.0,
        std::clamp(firstTravelSeconds, 0.0, predictionHorizonSeconds),
        std::clamp(secondTravelSeconds, 0.0, predictionHorizonSeconds),
        predictionHorizonSeconds};
    std::sort(breakpoints.begin(), breakpoints.end());

    PredictedApproach closest{
        std::numeric_limits<double>::infinity(), 0.0};
    for (std::size_t index = 0; index + 1 < breakpoints.size(); ++index)
    {
        const double intervalStart = breakpoints[index];
        const double intervalEnd = breakpoints[index + 1];
        if (intervalEnd <= intervalStart)
            continue;

        const glm::dvec3 firstAtStart = firstPosition + firstVelocity
            * std::min(intervalStart, firstTravelSeconds);
        const glm::dvec3 secondAtStart = secondPosition + secondVelocity
            * std::min(intervalStart, secondTravelSeconds);
        const glm::dvec3 firstIntervalVelocity = intervalStart < firstTravelSeconds
            ? firstVelocity : glm::dvec3{0.0};
        const glm::dvec3 secondIntervalVelocity = intervalStart < secondTravelSeconds
            ? secondVelocity : glm::dvec3{0.0};
        const glm::dvec3 relativePosition = secondAtStart - firstAtStart;
        const glm::dvec3 relativeVelocity =
            secondIntervalVelocity - firstIntervalVelocity;
        const double intervalDuration = intervalEnd - intervalStart;
        const double relativeSpeedSquared = glm::dot(
            relativeVelocity, relativeVelocity);
        const double timeWithinInterval = relativeSpeedSquared > 1.0e-9
            ? std::clamp(-glm::dot(relativePosition, relativeVelocity)
                / relativeSpeedSquared, 0.0, intervalDuration)
            : 0.0;
        const double distance = glm::length(
            relativePosition + relativeVelocity * timeWithinInterval);
        if (distance < closest.distanceMeters)
            closest = {distance, intervalStart + timeWithinInterval};
    }
    return closest;
}

glm::dvec3 PositionAtPredictedTime(
    const glm::dvec3& position,
    const glm::dvec3& velocity,
    double travelSeconds,
    double timeSeconds)
{
    return position + velocity * std::clamp(timeSeconds, 0.0, travelSeconds);
}

glm::dvec3 PositionBeyondConflict(
    const glm::dvec3& conflictPosition,
    const glm::dvec3& currentPosition,
    const glm::dvec3& velocityIntent,
    double clearanceMeters)
{
    glm::dvec3 forward{velocityIntent.x, velocityIntent.y, 0.0};
    if (glm::length(forward) < 1.0e-6)
        forward = {conflictPosition.x - currentPosition.x,
            conflictPosition.y - currentPosition.y, 0.0};
    if (glm::length(forward) < 1.0e-6)
        return conflictPosition;
    return conflictPosition + glm::normalize(forward) * clearanceMeters;
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
    std::unordered_map<std::string, glm::dvec3> avoidanceTargets;
    DroneNameSet currentlyAvoiding;
    for (std::size_t firstIndex = 0; firstIndex < definition.drones.size(); ++firstIndex)
    {
        const std::string& firstName = definition.drones[firstIndex].name;
        const bool firstConnected = IsConnected(observations, firstName);
        const auto firstTask = std::find_if(mission->tasks.begin(), mission->tasks.end(),
            [firstConnected, &firstName](const ScenarioResult::MissionTask& task)
            {
                return firstConnected && task.droneName == firstName && task.started
                    && !task.IsCompleted() && !task.failed;
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
                        && !task.IsCompleted() && !task.failed;
                });
            const bool secondHasTask = secondTask != mission->tasks.end()
                && CanReceiveWaypoint(simulation.State(droneIds.at(secondName)).mode);
            const bool bothDronesHaveTasks = firstConnected && secondConnected
                && firstHasTask && secondHasTask;
            const std::string* yieldingDrone = nullptr;
            if (firstConnected && secondConnected)
            {
                if (!firstHasTask && !secondHasTask)
                    continue;
                // 임무가 없는 연결 드론도 장애물로 본다. 둘 다 임무 중이면
                // 서로 반대 방향으로 비켜서 충돌 지점을 함께 통과한다.
                if (!bothDronesHaveTasks)
                    yieldingDrone = firstHasTask ? &firstName : &secondName;
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
            const glm::dvec3 firstVelocityIntent = MissionVelocityIntent(
                firstState, firstHasTask ? &*firstTask : nullptr, definition,
                activeAvoidanceTargets, true);
            const glm::dvec3 secondVelocityIntent = MissionVelocityIntent(
                secondState, secondHasTask ? &*secondTask : nullptr, definition,
                activeAvoidanceTargets, true);
            const double firstIntentTravelSeconds = TimeToMissionWaypoint(
                firstState, firstHasTask ? &*firstTask : nullptr,
                firstVelocityIntent, definition, activeAvoidanceTargets, true);
            const double secondIntentTravelSeconds = TimeToMissionWaypoint(
                secondState, secondHasTask ? &*secondTask : nullptr,
                secondVelocityIntent, definition, activeAvoidanceTargets, true);
            const PredictedApproach intendedPath = ClosestApproachWithStops(
                firstState.positionMeters, firstVelocityIntent,
                firstIntentTravelSeconds, secondState.positionMeters,
                secondVelocityIntent, secondIntentTravelSeconds,
                settings.predictionHorizonSeconds);
            const glm::dvec3 threatVelocity =
                secondVelocityIntent - firstVelocityIntent;
            const double predictedDistance = intendedPath.distanceMeters;
            const glm::dvec3 firstRouteVelocity = MissionVelocityIntent(
                firstState, firstHasTask ? &*firstTask : nullptr, definition,
                activeAvoidanceTargets, false);
            const glm::dvec3 secondRouteVelocity = MissionVelocityIntent(
                secondState, secondHasTask ? &*secondTask : nullptr, definition,
                activeAvoidanceTargets, false);
            const double firstRouteTravelSeconds = TimeToMissionWaypoint(
                firstState, firstHasTask ? &*firstTask : nullptr,
                firstRouteVelocity, definition, activeAvoidanceTargets, false);
            const double secondRouteTravelSeconds = TimeToMissionWaypoint(
                secondState, secondHasTask ? &*secondTask : nullptr,
                secondRouteVelocity, definition, activeAvoidanceTargets, false);
            const PredictedApproach routeContinuation = ClosestApproachWithStops(
                firstState.positionMeters, firstRouteVelocity,
                firstRouteTravelSeconds, secondState.positionMeters,
                secondRouteVelocity, secondRouteTravelSeconds,
                settings.predictionHorizonSeconds);
            const double routeContinuationDistance = routeContinuation.distanceMeters;
            if (!result.minimumPredictedDroneSeparationMeters
                || predictedDistance < *result.minimumPredictedDroneSeparationMeters)
            {
                result.minimumPredictedDroneSeparationMeters = predictedDistance;
            }
            const bool alreadyAvoiding = bothDronesHaveTasks
                ? activeAvoidanceDirections.contains(firstName)
                    || activeAvoidanceDirections.contains(secondName)
                : activeAvoidanceDirections.contains(*yieldingDrone);
            // 회피 목표에 도착했더라도 원래 임무 경로가 다시 교차하면 회피를 유지한다.
            // 한쪽이 정지 장애물이면 안전거리까지만 확보해도 회피를 풀 수 있다.
            const double releaseDistance = settings.safetyDistanceMeters
                * (bothDronesHaveTasks ? 1.5 : 1.0);
            const double entryDistance = settings.safetyDistanceMeters;
            const double lateralClearance = settings.lateralOffsetMeters
                + settings.safetyDistanceMeters;
            const bool shouldContinueAvoiding = alreadyAvoiding
                ? routeContinuationDistance <= releaseDistance
                : predictedDistance <= entryDistance;
            if (!shouldContinueAvoiding)
                continue;

            glm::dvec3 side(-threatVelocity.y, threatVelocity.x, 0.0);
            if (glm::length(side) < 1e-6)
                side = {-relativePosition.y, relativePosition.x, 0.0};
            if (glm::length(side) < 1e-6)
                side = {0.0, 1.0, 0.0};
            side = glm::normalize(side);
            if (bothDronesHaveTasks)
            {
                const auto [firstDirection, firstInserted] =
                    activeAvoidanceDirections.try_emplace(firstName, side);
                const glm::dvec3 firstSide = firstInserted
                    ? side : firstDirection->second;
                const auto [secondDirection, secondInserted] =
                    activeAvoidanceDirections.try_emplace(secondName, -firstSide);
                const glm::dvec3 secondSide = secondInserted
                    ? -firstSide : secondDirection->second;
                const glm::dvec3 firstConflictPosition = PositionAtPredictedTime(
                    firstState.positionMeters, firstVelocityIntent,
                    firstIntentTravelSeconds, intendedPath.timeSeconds);
                const glm::dvec3 secondConflictPosition = PositionAtPredictedTime(
                    secondState.positionMeters, secondVelocityIntent,
                    secondIntentTravelSeconds, intendedPath.timeSeconds);
                const double clearance = releaseDistance;
                avoidanceTargets.try_emplace(firstName,
                    PositionBeyondConflict(firstConflictPosition,
                        firstState.positionMeters, firstVelocityIntent, clearance)
                    + firstSide * lateralClearance);
                avoidanceTargets.try_emplace(secondName,
                    PositionBeyondConflict(secondConflictPosition,
                        secondState.positionMeters, secondVelocityIntent, clearance)
                    + secondSide * lateralClearance);
                currentlyAvoiding.insert(firstName);
                currentlyAvoiding.insert(secondName);
            }
            else
            {
                if (*yieldingDrone == firstName)
                    side = -side;
                const auto [activeDirection, directionInserted] =
                    activeAvoidanceDirections.try_emplace(*yieldingDrone, side);
                if (!directionInserted)
                    side = activeDirection->second;
                const glm::dvec3 conflictPosition = PositionAtPredictedTime(
                    *yieldingDrone == firstName
                        ? firstState.positionMeters : secondState.positionMeters,
                    *yieldingDrone == firstName
                        ? firstVelocityIntent : secondVelocityIntent,
                    *yieldingDrone == firstName
                        ? firstIntentTravelSeconds : secondIntentTravelSeconds,
                    intendedPath.timeSeconds);
                const glm::dvec3 currentPosition = *yieldingDrone == firstName
                    ? firstState.positionMeters : secondState.positionMeters;
                const glm::dvec3 velocityIntent = *yieldingDrone == firstName
                    ? firstVelocityIntent : secondVelocityIntent;
                avoidanceTargets.try_emplace(*yieldingDrone,
                    PositionBeyondConflict(conflictPosition, currentPosition,
                        velocityIntent, releaseDistance)
                    + side * lateralClearance);
                currentlyAvoiding.insert(*yieldingDrone);
            }
        }
    }

    for (const std::string& droneName : currentlyAvoiding)
    {
        const auto task = std::find_if(mission->tasks.begin(), mission->tasks.end(),
            [&droneName](const ScenarioResult::MissionTask& candidate)
            {
                return candidate.droneName == droneName && candidate.started
                    && !candidate.IsCompleted() && !candidate.failed;
            });
        if (task == mission->tasks.end())
            continue;

        // 충돌 지점을 지나서 비켜서도록 목표를 잡고, 위험이 끝날 때까지 유지한다.
        const auto avoidanceTarget = activeAvoidanceTargets.try_emplace(
            droneName, avoidanceTargets.at(droneName)).first;
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
        if (!task.started || task.IsCompleted() || task.failed
            || task.droneName.empty() || !IsConnected(observations, task.droneName))
            continue;

        const DroneId droneId = droneIds.at(task.droneName);
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
        else if (reachedWaypoint)
        {
            task.completedTick = tick;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.role + "' 임무를 마쳤습니다.");
        }
        else if (tick >= task.deadlineTick)
        {
            task.failed = true;
            result.issues.push_back(task.droneName
                + " 드론이 임무 기한 안에 목표에 도달하지 못했습니다.");
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
    const simulation::CameraState& primaryCamera = result.drones.front().finalCamera;
    if (definition.expectedCameraStatus
        && primaryCamera.status != *definition.expectedCameraStatus)
    {
        result.issues.push_back("최종 카메라 상태가 예상값과 다릅니다.");
    }
    if (definition.expectedFrameCount
        && primaryCamera.frameCount != *definition.expectedFrameCount)
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
        bool allTasksCompleted = true;
        bool hasFailedTask = false;
        std::optional<std::uint64_t> latestTaskCompletionTick;
        for (const ScenarioResult::MissionTask& task : result.mission->tasks)
        {
            totalWaypointCount += task.waypointsMeters.size();
            if (task.IsCompleted())
            {
                visitedWaypointCount += task.waypointsMeters.size();
                ++result.mission->completedTaskCount;
                if (!latestTaskCompletionTick
                    || *task.completedTick > *latestTaskCompletionTick)
                    latestTaskCompletionTick = task.completedTick;
            }
            else
            {
                allTasksCompleted = false;
                visitedWaypointCount += task.currentWaypointIndex;
            }
            hasFailedTask = hasFailedTask || task.failed;
        }
        result.mission->completionPercent = 100.0
            * static_cast<double>(result.mission->completedTaskCount)
            / static_cast<double>(result.mission->tasks.size());
        result.mission->coveragePercent = totalWaypointCount == 0 ? 0.0
            : 100.0 * static_cast<double>(visitedWaypointCount)
                / static_cast<double>(totalWaypointCount);
        result.mission->completedTick = allTasksCompleted
            ? latestTaskCompletionTick : std::nullopt;
        result.mission->outcome = hasFailedTask
            ? ScenarioResult::Outcome::Failed
            : allTasksCompleted
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
    DroneHomePositions homePositions;
    DroneObservations observations;
    std::unordered_map<std::string, glm::dvec3> previousPositions;
    std::unordered_map<std::string, double> flightDistancesMeters;
    std::unordered_map<std::string, double> flightTimesSeconds;
    for (const auto& [droneName, droneId] : droneIds)
    {
        glm::dvec3 homePosition = simulation.State(droneId).positionMeters;
        if (simulation.State(droneId).mode == FlightMode::Grounded)
            homePosition.z += DroneModel::takeoffHeightMeters;
        homePositions.emplace(droneName, homePosition);
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
            observations, nextCommand, result);
        if (definition.energyModel)
        {
            UpdateEnergyPolicy(tick, result.mission,
                simulation, droneIds, homePositions, definition, result);
        }
        UpdateMissionCommands(tick, result.mission, simulation, droneIds,
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
        UpdateMissionResults(tick, result.mission,
            simulation, droneIds, observations, result);
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
