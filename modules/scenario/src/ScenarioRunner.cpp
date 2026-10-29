#include "scenario/ScenarioRunner.h"
#include "simulation_core/BatteryModel.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/SimulationCore.h"
#include "scenario/ScenarioDefinition.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
using detail::ScenarioMission;
using detail::ScenarioMissionTask;
using detail::TickCommand;
using detail::DroneCommunicationOutage;
using DroneCommunicationStatus = ScenarioResult::Drone::CommunicationStatus;

struct DroneObservation
{
    DroneCommunicationStatus status = DroneCommunicationStatus::Connected;
    std::uint64_t lastHeartbeatTick = 0;
    DroneState lastKnownState;
    std::optional<double> lastKnownBatteryPercent;
};

struct ScenarioDroneRuntime
{
    DroneId id{};
    glm::dvec3 homePositionMeters{0.0};
    DroneObservation observation;
    glm::dvec3 previousPositionMeters{0.0};
    double flightDistanceMeters = 0.0;
    double flightTimeSeconds = 0.0;
};

using ScenarioDrones = std::unordered_map<std::string, ScenarioDroneRuntime>;

void RecordTrajectorySample(const ScenarioDefinition& definition,
    const SimulationCore& simulation, const ScenarioDrones& drones,
    std::uint64_t tick, ScenarioResult& result)
{
    ScenarioResult::TrajectorySample sample;
    sample.tick = tick;
    sample.timeSeconds = static_cast<double>(tick) * DroneModel::fixedStepSeconds;
    sample.drones.reserve(drones.size());

    const auto addDrone = [&](const std::string& name)
    {
        sample.drones.emplace_back(name, simulation.State(drones.at(name).id));
    };
    for (const ScenarioDrone& drone : definition.drones)
        addDrone(drone.name);

    result.trajectory.push_back(std::move(sample));
}

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
    const ScenarioDrones& drones,
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    const auto handleDrone = [&](const std::string& droneName)
    {
        const ScenarioDroneRuntime& drone = drones.at(droneName);
        const DroneId droneId = drone.id;
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

        const glm::dvec3& returnPosition = drone.homePositionMeters;
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

    for (const ScenarioDrone& drone : definition.drones)
        handleDrone(drone.name);
}

bool IsConnected(const ScenarioDrones& drones, const std::string& droneName)
{
    return drones.at(droneName).observation.status
        == DroneCommunicationStatus::Connected;
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
    ScenarioDrones& drones,
    ScenarioResult& result)
{
    const auto updateDrone = [&](const std::string& droneName)
    {
        ScenarioDroneRuntime& drone = drones.at(droneName);
        const DroneId droneId = drone.id;
        DroneObservation& observation = drone.observation;
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
        observation.status = newStatus;
    };

    // 이벤트 로그 순서도 시나리오 입력 순서에 맞춰 매번 같게 만든다.
    for (const ScenarioDrone& drone : definition.drones)
        updateDrone(drone.name);
}

/**
 * @brief 시나리오에서 사용할 이름과 실제 드론 번호를 연결한다.
 * @details 목록을 생략한 경우에는 SimulationCore의 기본 드론을 그대로 사용한다.
 */
ScenarioDrones CreateScenarioDrones(
    const ScenarioDefinition& definition,
    SimulationCore& simulation)
{
    ScenarioDrones drones;
    const DroneId defaultDrone = simulation.PrimaryDroneId();
    for (const ScenarioDrone& drone : definition.drones)
    {
        DroneId id = defaultDrone;
        if (!definition.usesExistingPrimaryDrone)
        {
            id = simulation.AddDrone(
                drone.initialState.positionMeters, drone.initialState.yawRadians);
        }
        simulation.SetCameraModel(
            id, drone.cameraModel.value_or(definition.defaultCameraModel));
        drones.emplace(drone.name, ScenarioDroneRuntime{id});
        if (!definition.usesExistingPrimaryDrone && drone.hasExtendedInitialState)
            simulation.SetInitialState(id, drone.initialState);
    }

    if (!definition.usesExistingPrimaryDrone && !simulation.RemoveDrone(defaultDrone))
        throw std::logic_error("시나리오용 기본 드론을 교체하지 못했습니다.");
    return drones;
}

ScenarioResult::Mission CreateMissionResult(ScenarioMission definition)
{
    ScenarioResult::Mission result;
    result.name = definition.name;
    result.tasks.reserve(definition.tasks.size());
    for (ScenarioMissionTask& task : definition.tasks)
    {
        ScenarioResult::MissionTask runtimeTask;
        runtimeTask.name = std::move(task.name);
        runtimeTask.droneName = std::move(task.droneName);
        runtimeTask.waypointsMeters = std::move(task.waypointsMeters);
        runtimeTask.startTick = task.startTick;
        runtimeTask.deadlineTick = task.deadlineTick;
        runtimeTask.toleranceMeters = task.toleranceMeters;
        result.tasks.push_back(std::move(runtimeTask));
    }
    return result;
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
    const ScenarioDrones& drones,
    std::vector<glm::dvec3>& positionsMeters,
    ScenarioResult& result)
{
    if (!definition.minimumDroneSeparationMeters)
        return;

    positionsMeters.clear();
    for (const ScenarioDrone& drone : definition.drones)
    {
        positionsMeters.push_back(
            simulation.State(drones.at(drone.name).id).positionMeters);
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
    const ScenarioDrones& drones,
    std::size_t& nextCommand,
    ScenarioResult& result)
{
    while (nextCommand < commands.size() && commands[nextCommand].tick == tick)
    {
        const TickCommand& command = commands[nextCommand++];
        if (!IsConnected(drones, command.droneName))
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + command.droneName
                + " 드론에 명령을 전달하지 못했습니다. 마지막 위치는 이전 보고값입니다.");
            continue;
        }

        if (simulation.Apply(drones.at(command.droneName).id, command.command)
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
    const ScenarioDrones& drones,
    ScenarioResult& result)
{
    if (!mission)
        return;

    // 목록 순서대로 훑으면서 드론마다 첫 미완료 작업 하나만 처리한다.
    DroneNameSet dronesWithPendingTask;
    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (task.IsCompleted() || task.failed)
            continue;
        if (!dronesWithPendingTask.insert(task.droneName).second)
            continue;
        if (tick < task.startTick || !IsConnected(drones, task.droneName))
            continue;

        const DroneId droneId = drones.at(task.droneName).id;
        const FlightMode mode = simulation.State(droneId).mode;
        if (!CanReceiveWaypoint(mode))
        {
            // 이륙 중이면 정해 둔 기한까지 기다린 뒤 시작 가능 여부를 판정한다.
            if (!task.started && mode == FlightMode::TakingOff
                && tick < task.deadlineTick)
                continue;

            task.failed = true;
            dronesWithPendingTask.erase(task.droneName);
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
                + task.droneName + " 드론이 '" + task.name + "' 임무를 시작했습니다.");
        }

        const DroneCommand moveCommand{
            DroneCommand::Kind::SetWaypoint, CurrentWaypoint(task)};
        const CommandResult moveResult = simulation.Apply(droneId, moveCommand);
        if (moveResult != CommandResult::Accepted)
        {
            task.failed = true;
            dronesWithPendingTask.erase(task.droneName);
            result.issues.push_back(task.droneName + " 드론에 임무 목표를 전달하지 못했습니다.");
        }
    }
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

    const auto avoidanceTarget = activeAvoidanceTargets.find(task->droneName);
    const glm::dvec3 waypoint = includeAvoidanceTarget
        && avoidanceTarget != activeAvoidanceTargets.end()
        ? avoidanceTarget->second
        : CurrentWaypoint(*task);

    glm::dvec3 targetVelocity = QuadrotorFlightController::WaypointVelocityTarget(
        state, waypoint, definition.controllerParameters);

    if (definition.windVelocity)
        targetVelocity += *definition.windVelocity;
    return targetVelocity;
}

struct PredictedApproach
{
    double distanceMeters;
    double timeSeconds;
};

glm::dvec3 PositionAtPredictedTime(
    const glm::dvec3& position,
    const glm::dvec3& velocity,
    double timeSeconds)
{
    return position + velocity * timeSeconds;
}

/** @brief 현재 목표 속도가 유지된다고 보고 일정 시간 안의 가장 가까운 거리를 구한다.
 * @details 경유점 도착 뒤 감속·정지는 계산하지 않는 간단한 충돌 위험 추정이다.
 */
PredictedApproach PredictClosestApproach(
    const glm::dvec3& firstPosition,
    const glm::dvec3& firstVelocity,
    const glm::dvec3& secondPosition,
    const glm::dvec3& secondVelocity,
    double predictionHorizonSeconds)
{
    const glm::dvec3 relativePosition = secondPosition - firstPosition;
    const glm::dvec3 relativeVelocity = secondVelocity - firstVelocity;
    const double relativeSpeedSquared = glm::dot(relativeVelocity, relativeVelocity);
    const double time = relativeSpeedSquared > 1.0e-9
        ? std::clamp(-glm::dot(relativePosition, relativeVelocity)
            / relativeSpeedSquared, 0.0, predictionHorizonSeconds)
        : 0.0;
    return {glm::length(relativePosition + relativeVelocity * time), time};
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

/** @brief 드론이 맡아 진행 중인 첫 임무 작업을 찾는다. */
const ScenarioResult::MissionTask* FindActiveMissionTask(
    const ScenarioResult::Mission& mission,
    std::string_view droneName)
{
    const auto task = std::find_if(mission.tasks.begin(), mission.tasks.end(),
        [droneName](const ScenarioResult::MissionTask& candidate)
        {
            return candidate.droneName == droneName && candidate.started
                && !candidate.IsCompleted() && !candidate.failed;
        });
    return task == mission.tasks.end() ? nullptr : &*task;
}

void UpdateCollisionAvoidance(
    std::uint64_t tick,
    const ScenarioDefinition& definition,
    std::optional<ScenarioResult::Mission>& mission,
    SimulationCore& simulation,
    const ScenarioDrones& drones,
    std::unordered_map<std::string, glm::dvec3>& activeAvoidanceTargets,
    ScenarioResult& result)
{
    if (!definition.collisionAvoidance || !mission)
        return;

    const CollisionAvoidanceSettings& settings = *definition.collisionAvoidance;
    std::unordered_map<std::string, glm::dvec3> avoidanceTargets;
    DroneNameSet currentlyAvoiding;
    std::vector<bool> droneConnected;
    std::vector<const ScenarioResult::MissionTask*> activeTasks;
    droneConnected.reserve(definition.drones.size());
    activeTasks.reserve(definition.drones.size());
    for (const ScenarioDrone& drone : definition.drones)
    {
        const bool connected = IsConnected(drones, drone.name);
        droneConnected.push_back(connected);
        const ScenarioResult::MissionTask* task = connected
            ? FindActiveMissionTask(*mission, drone.name) : nullptr;
        if (task && !CanReceiveWaypoint(simulation.State(drones.at(drone.name).id).mode))
            task = nullptr;
        activeTasks.push_back(task);
    }

    for (std::size_t firstIndex = 0; firstIndex < definition.drones.size(); ++firstIndex)
    {
        const std::string& firstName = definition.drones[firstIndex].name;
        const bool firstConnected = droneConnected[firstIndex];
        const ScenarioResult::MissionTask* firstTask = activeTasks[firstIndex];
        const bool firstHasTask = firstTask != nullptr;
        for (std::size_t secondIndex = firstIndex + 1;
            secondIndex < definition.drones.size(); ++secondIndex)
        {
            const std::string& secondName = definition.drones[secondIndex].name;
            const bool secondConnected = droneConnected[secondIndex];
            const ScenarioResult::MissionTask* secondTask = activeTasks[secondIndex];
            const bool secondHasTask = secondTask != nullptr;
            const bool bothDronesHaveTasks = firstConnected && secondConnected
                && firstHasTask && secondHasTask;
            const std::string* yieldingDrone = nullptr;
            if (firstConnected && secondConnected)
            {
                if (!firstHasTask && !secondHasTask)
                    continue;
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
                    return simulation.State(drones.at(droneName).id);
                DroneState state = drones.at(droneName).observation.lastKnownState;
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
                firstState, firstTask, definition,
                activeAvoidanceTargets, true);
            const glm::dvec3 secondVelocityIntent = MissionVelocityIntent(
                secondState, secondTask, definition,
                activeAvoidanceTargets, true);
            const PredictedApproach intendedPath = PredictClosestApproach(
                firstState.positionMeters, firstVelocityIntent,
                secondState.positionMeters, secondVelocityIntent,
                settings.predictionHorizonSeconds);
            const glm::dvec3 threatVelocity =
                secondVelocityIntent - firstVelocityIntent;
            const double predictedDistance = intendedPath.distanceMeters;
            const glm::dvec3 firstRouteVelocity = MissionVelocityIntent(
                firstState, firstTask, definition,
                activeAvoidanceTargets, false);
            const glm::dvec3 secondRouteVelocity = MissionVelocityIntent(
                secondState, secondTask, definition,
                activeAvoidanceTargets, false);
            const PredictedApproach routeContinuation = PredictClosestApproach(
                firstState.positionMeters, firstRouteVelocity,
                secondState.positionMeters, secondRouteVelocity,
                settings.predictionHorizonSeconds);
            const double routeContinuationDistance = routeContinuation.distanceMeters;
            if (!result.minimumPredictedDroneSeparationMeters
                || predictedDistance < *result.minimumPredictedDroneSeparationMeters)
            {
                result.minimumPredictedDroneSeparationMeters = predictedDistance;
            }
            const bool alreadyAvoiding = bothDronesHaveTasks
                ? activeAvoidanceTargets.contains(firstName)
                    || activeAvoidanceTargets.contains(secondName)
                : activeAvoidanceTargets.contains(*yieldingDrone);
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
            const auto makeTarget = [&](const DroneState& state,
                                        const glm::dvec3& velocity,
                                        const glm::dvec3& offset)
            {
                const glm::dvec3 conflictPosition = PositionAtPredictedTime(
                    state.positionMeters, velocity, intendedPath.timeSeconds);
                return PositionBeyondConflict(conflictPosition,
                    state.positionMeters, velocity, releaseDistance)
                    + offset * lateralClearance;
            };
            if (bothDronesHaveTasks)
            {
                avoidanceTargets.try_emplace(firstName,
                    makeTarget(firstState, firstVelocityIntent, side));
                avoidanceTargets.try_emplace(secondName,
                    makeTarget(secondState, secondVelocityIntent, -side));
                currentlyAvoiding.insert(firstName);
                currentlyAvoiding.insert(secondName);
            }
            else
            {
                const bool yieldingIsFirst = *yieldingDrone == firstName;
                const DroneState& yieldingState = yieldingIsFirst
                    ? firstState : secondState;
                const glm::dvec3& yieldingVelocity = yieldingIsFirst
                    ? firstVelocityIntent : secondVelocityIntent;
                avoidanceTargets.try_emplace(*yieldingDrone,
                    makeTarget(yieldingState, yieldingVelocity,
                        yieldingIsFirst ? -side : side));
                currentlyAvoiding.insert(*yieldingDrone);
            }
        }
    }

    for (std::size_t droneIndex = 0; droneIndex < definition.drones.size(); ++droneIndex)
    {
        const std::string& droneName = definition.drones[droneIndex].name;
        if (!currentlyAvoiding.contains(droneName) || activeTasks[droneIndex] == nullptr)
            continue;

        // 충돌 지점을 지나서 비켜서도록 목표를 잡고, 위험이 끝날 때까지 유지한다.
        const auto [avoidanceTarget, newlyAvoiding] =
            activeAvoidanceTargets.try_emplace(
                droneName, avoidanceTargets.at(droneName));
        const CommandResult commandResult = simulation.Apply(drones.at(droneName).id,
            {DroneCommand::Kind::SetWaypoint, avoidanceTarget->second});
        if (commandResult != CommandResult::Accepted)
        {
            result.issues.push_back(droneName + " 드론에 충돌 회피 경로를 전달하지 못했습니다.");
            continue;
        }
        if (newlyAvoiding)
        {
            ++result.collisionAvoidanceEvents;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론이 예상 충돌을 피해 옆 경로로 이동합니다.");
        }
    }

    for (const auto& entry : activeAvoidanceTargets)
    {
        const std::string& droneName = entry.first;
        if (!currentlyAvoiding.contains(droneName))
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론이 위험 구간을 벗어나 조사 경로로 돌아갑니다.");
        }
    }
    std::erase_if(activeAvoidanceTargets,
        [&currentlyAvoiding](const auto& entry)
        {
            return !currentlyAvoiding.contains(entry.first);
        });
}

void UpdateMissionResults(
    std::uint64_t tick,
    std::optional<ScenarioResult::Mission>& mission,
    const SimulationCore& simulation,
    const ScenarioDrones& drones,
    ScenarioResult& result)
{
    if (!mission)
        return;

    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (!task.started || task.IsCompleted() || task.failed
            || !IsConnected(drones, task.droneName))
            continue;

        const DroneId droneId = drones.at(task.droneName).id;
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
                + task.droneName + " 드론이 '" + task.name + "' 임무를 마쳤습니다.");
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
    const ScenarioDrones& drones,
    ScenarioResult& result)
{
    result.tickCount = simulation.TickCount();
    result.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    result.windVelocityMetersPerSecond = definition.windVelocity;
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
    const auto appendDrone = [&](const std::string& droneName)
    {
        const ScenarioDroneRuntime& runtime = drones.at(droneName);
        const DroneObservation& observation = runtime.observation;
        ScenarioResult::Drone drone{
            droneName, observation.lastKnownState,
            observation.status, observation.lastHeartbeatTick};
        drone.finalCamera = simulation.Camera(runtime.id);
        drone.flightDistanceMeters = runtime.flightDistanceMeters;
        drone.flightTimeSeconds = runtime.flightTimeSeconds;
        drone.reportedBatteryPercent = observation.lastKnownBatteryPercent;
        if (simulation.HasBattery(runtime.id))
        {
            const BatteryModel& battery = simulation.Battery(runtime.id);
            drone.batteryCapacityWattHours = battery.CapacityWattHours();
            drone.energyConsumedWattHours = battery.ConsumedWattHours();
            drone.batteryRemainingWattHours = battery.RemainingWattHours();
            drone.batteryPercent = battery.RemainingPercent();
            drone.returnedForLowBattery = battery.IsReturningForLowBattery();
        }
        result.drones.push_back(std::move(drone));
    };

    for (const ScenarioDrone& drone : definition.drones)
        appendDrone(drone.name);

}

/** @brief 단일 드론 형식과 이름별 형식에 맞는 비행 상태 오류 문구를 만든다. */
std::string FlightModeMismatchMessage(const ExpectedDrone& expectedDrone)
{
    return expectedDrone.name + " 드론의 최종 비행 상태가 예상과 다릅니다.";
}

void CheckExpectedDroneResults(
    const ScenarioDefinition& definition,
    ScenarioResult& result)
{
    // 결과 이름을 복사하지 않고 빌려 쓰는 색인으로 반복 검색을 줄인다.
    std::unordered_map<std::string_view, std::size_t> resultIndexByName;
    resultIndexByName.reserve(result.drones.size());
    for (std::size_t index = 0; index < result.drones.size(); ++index)
        resultIndexByName.emplace(result.drones[index].name, index);

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
        const std::string dronePrefix = expectedDrone.name + " 드론의 ";
        if (drone.finalState.mode != expectedDrone.mode)
            result.issues.push_back(FlightModeMismatchMessage(expectedDrone));
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
    if (!definition.expectedDrones.empty()
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
    CheckExpectedDroneResults(definition, result);
    CheckExpectedSeparation(definition, result);
    CheckExpectedCamera(definition, result);
    if (!result.issues.empty())
        result.outcome = ScenarioResult::Outcome::Failed;
    else if (result.mission)
        result.outcome = result.mission->GetOutcome();
    else
        result.outcome = ScenarioResult::Outcome::Passed;
}
}

ScenarioResult::Outcome ScenarioResult::Mission::GetOutcome() const noexcept
{
    bool allTasksCompleted = true;
    for (const MissionTask& task : tasks)
    {
        if (task.failed)
            return Outcome::Failed;
        if (!task.IsCompleted())
            allTasksCompleted = false;
    }
    return allTasksCompleted ? Outcome::Passed : Outcome::Inconclusive;
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
    // 입력을 먼저 자료형으로 읽고 규칙을 검사한다. 잘못된 설정으로
    // 시뮬레이션 객체를 만들거나 Tick을 시작하지 않는다.
    ScenarioDefinition definition = detail::ParseScenarioDefinition(jsonText);
    detail::ValidateScenarioDefinition(definition);
    const std::uint64_t tickCount = definition.tickCount;
    SimulationCore simulation{
        definition.quadrotorParameters, definition.controllerParameters};
    if (definition.windVelocity)
        simulation.SetWindVelocityMetersPerSecond(*definition.windVelocity);

    ScenarioDrones drones = CreateScenarioDrones(definition, simulation);
    for (auto& [droneName, drone] : drones)
    {
        const DroneState& initialState = simulation.State(drone.id);
        drone.homePositionMeters = initialState.positionMeters;
        if (initialState.mode == FlightMode::Grounded)
            drone.homePositionMeters.z += DroneModel::takeoffHeightMeters;
        drone.observation = DroneObservation{
            DroneCommunicationStatus::Connected, 0, initialState};
        drone.previousPositionMeters = initialState.positionMeters;
    }
    if (definition.energyModel)
    {
        for (const ScenarioDrone& scenarioDrone : definition.drones)
        {
            BatteryModelParameters parameters = *definition.energyModel;
            if (scenarioDrone.batteryCapacityWattHours)
                parameters.capacityWattHours = *scenarioDrone.batteryCapacityWattHours;
            ScenarioDroneRuntime& runtime = drones.at(scenarioDrone.name);
            const DroneId droneId = runtime.id;
            simulation.ConfigureBattery(
                droneId, parameters, scenarioDrone.initialBatteryPercent);
            runtime.observation.lastKnownBatteryPercent =
                simulation.Battery(droneId).RemainingPercent();
        }
    }

    ScenarioResult result;
    if (definition.mission)
        result.mission = CreateMissionResult(std::move(*definition.mission));
    if (definition.trajectorySampleEveryTicks)
    {
        const std::uint64_t periodicSamples =
            tickCount / *definition.trajectorySampleEveryTicks + 1;
        const std::uint64_t sampleCount = periodicSamples
            + (tickCount % *definition.trajectorySampleEveryTicks != 0 ? 1 : 0);
        result.trajectory.reserve(static_cast<std::size_t>(sampleCount));
    }
    std::size_t nextCommand = 0;
    std::unordered_map<std::string, glm::dvec3> activeAvoidanceTargets;
    std::vector<glm::dvec3> dronePositionsMeters;
    if (definition.minimumDroneSeparationMeters)
        dronePositionsMeters.reserve(definition.drones.size());

    RecordMinimumDroneSeparation(
        definition, simulation, drones, dronePositionsMeters, result);
    if (definition.trajectorySampleEveryTicks)
        RecordTrajectorySample(definition, simulation, drones, 0, result);
    for (std::uint64_t tick = 0; tick < tickCount; ++tick)
    {
        // 먼저 관제소의 연결 상태를 갱신하고, 통신이 살아 있는 드론에만
        // 외부 명령을 전달한다. 드론은 통신이 끊겨도 이미 받은 경로를 계속 따른다.
        UpdateDroneObservations(tick, definition, simulation, drones, result);
        ApplyScheduledCommands(tick, definition.commands, simulation, drones,
            nextCommand, result);

        // 배터리 귀환과 임무 명령은 같은 Tick의 물리 이동 전에 적용한다.
        if (definition.energyModel)
        {
            UpdateEnergyPolicy(tick, result.mission,
                simulation, drones, definition, result);
        }
        UpdateMissionCommands(tick, result.mission, simulation, drones, result);
        UpdateCollisionAvoidance(tick, definition, result.mission, simulation,
            drones, activeAvoidanceTargets, result);
        for (auto& [droneName, drone] : drones)
        {
            if (simulation.State(drone.id).mode != FlightMode::Grounded)
                drone.flightTimeSeconds += DroneModel::fixedStepSeconds;
        }
        simulation.AdvanceOneTick();
        for (auto& [droneName, drone] : drones)
        {
            const glm::dvec3& currentPosition =
                simulation.State(drone.id).positionMeters;
            drone.flightDistanceMeters += glm::distance(
                drone.previousPositionMeters, currentPosition);
            drone.previousPositionMeters = currentPosition;
        }

        // 물리 상태를 한 Tick 진행한 뒤 실제 이동 거리, 최소 간격,
        // 임무 도착 여부와 요청된 궤적 표본을 기록한다.
        RecordMinimumDroneSeparation(
            definition, simulation, drones, dronePositionsMeters, result);
        UpdateMissionResults(tick, result.mission,
            simulation, drones, result);
        const std::uint64_t completedTick = tick + 1;
        if (definition.trajectorySampleEveryTicks
            && (completedTick % *definition.trajectorySampleEveryTicks == 0
                || completedTick == tickCount))
        {
            RecordTrajectorySample(
                definition, simulation, drones, completedTick, result);
        }
    }

    for (auto& [droneName, drone] : drones)
    {
        DroneObservation& observation = drone.observation;
        if (observation.status == DroneCommunicationStatus::Connected)
        {
            observation.lastKnownState = simulation.State(drone.id);
            observation.lastHeartbeatTick = tickCount;
            if (simulation.HasBattery(drone.id))
                observation.lastKnownBatteryPercent =
                    simulation.Battery(drone.id).RemainingPercent();
        }
    }
    PopulateFinalSnapshot(definition, simulation, drones, result);
    CheckExpectedResults(definition, result);
    return result;
}

}
