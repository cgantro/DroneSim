#include "scenario/ScenarioRunner.h"
#include "scenario/ScenarioResultEvaluator.h"
#include "simulation_core/BatteryModel.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/SimulationCore.h"
#include "scenario/ScenarioDefinition.h"
#include "flight_planning/CollisionAvoidancePlanner.h"
#include "simulation_core/DroneModel.h"

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

namespace dronesim::scenario
{
using namespace dronesim::simulation;

namespace
{
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

    /** @brief 통신으로 받은 상태와 배터리 값을 관제소의 마지막 기록으로 저장한다. */
    void RecordHeartbeat(std::uint64_t tick, const DroneState& state,
        std::optional<double> batteryPercent) noexcept
    {
        status = DroneCommunicationStatus::Connected;
        lastHeartbeatTick = tick;
        lastKnownState = state;
        lastKnownBatteryPercent = batteryPercent;
    }

    /** @brief 마지막 하트비트 이후 지난 Tick 수로 통신 상태를 판단한다. */
    [[nodiscard]] DroneCommunicationStatus StatusAfterMissedHeartbeat(
        std::uint64_t tick, std::uint64_t timeoutTicks) const noexcept
    {
        return tick - lastHeartbeatTick >= timeoutTicks
            ? DroneCommunicationStatus::Unknown
            : DroneCommunicationStatus::Stale;
    }

    /** @brief 관제소가 현재 시각에도 새 하트비트를 받는 중인지 알려준다. */
    [[nodiscard]] bool IsConnected() const noexcept
    {
        return status == DroneCommunicationStatus::Connected;
    }
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
        const bool beganLowBatteryReturn =
            simulation.TryBeginLowBatteryReturn(droneId);
        if (beganLowBatteryReturn)
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론이 배터리 예비량에 도달해 출발 지점으로 돌아갑니다.");

            if (mission)
            {
                for (ScenarioResult::MissionTask& task : mission->tasks)
                {
                    if (task.droneName != droneName || task.IsCompleted() || task.IsFailed())
                        continue;
                    task.Fail();
            result.eventLog.push_back("tick " + std::to_string(tick) + ": '"
                + task.name + "' 임무는 미완료로 남습니다.");
                }
            }
        }
        else if (!simulation.Battery(droneId).IsReturningForLowBattery())
        {
            return;
        }

        const DroneState& state = simulation.State(droneId);
        if (!DroneModel::CanAcceptInFlightCommands(state.mode))
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
    return drones.at(droneName).observation.IsConnected();
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
            const std::optional<double> batteryPercent = simulation.HasBattery(droneId)
                ? std::optional<double>{simulation.Battery(droneId).RemainingPercent()}
                : std::nullopt;
            observation.RecordHeartbeat(tick, simulation.State(droneId), batteryPercent);
            return;
        }

        const DroneCommunicationStatus newStatus = observation.StatusAfterMissedHeartbeat(
            tick, outage->heartbeatTimeoutTicks);
        if (observation.IsConnected())
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
        if (task.IsCompleted() || task.IsFailed())
            continue;
        if (!dronesWithPendingTask.insert(task.droneName).second)
            continue;
        if (tick < task.startTick || !IsConnected(drones, task.droneName))
            continue;

        const DroneId droneId = drones.at(task.droneName).id;
        const FlightMode mode = simulation.State(droneId).mode;
        if (!DroneModel::CanAcceptInFlightCommands(mode))
        {
            // 이륙 중이면 정해 둔 기한까지 기다린 뒤 시작 가능 여부를 판정한다.
            if (!task.HasStarted() && mode == FlightMode::TakingOff
                && tick < task.deadlineTick)
                continue;

            task.Fail();
            dronesWithPendingTask.erase(task.droneName);
            if (task.HasStarted())
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
        if (!task.HasStarted())
        {
            if (!task.Start())
                continue;
            if (!mission->startedTick)
                mission->startedTick = tick;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.name + "' 임무를 시작했습니다.");
        }

        const DroneCommand moveCommand{
            DroneCommand::Kind::SetWaypoint, task.CurrentWaypoint()};
        const CommandResult moveResult = simulation.Apply(droneId, moveCommand);
        if (moveResult != CommandResult::Accepted)
        {
            task.Fail();
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
        : task->CurrentWaypoint();

    glm::dvec3 targetVelocity = QuadrotorFlightController::WaypointVelocityTarget(
        state, waypoint, definition.controllerParameters);

    if (definition.windVelocity)
        targetVelocity += *definition.windVelocity;
    return targetVelocity;
}

/** @brief 드론이 맡아 진행 중인 첫 임무 작업을 찾는다. */
const ScenarioResult::MissionTask* FindActiveMissionTask(
    const ScenarioResult::Mission& mission,
    std::string_view droneName)
{
    const auto task = std::find_if(mission.tasks.begin(), mission.tasks.end(),
        [droneName](const ScenarioResult::MissionTask& candidate)
        {
            return candidate.droneName == droneName && candidate.IsActive();
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

    std::vector<flight_planning::DroneMotionIntent> motionIntents;
    motionIntents.reserve(definition.drones.size());
    for (const ScenarioDrone& drone : definition.drones)
    {
        const bool connected = IsConnected(drones, drone.name);
        DroneState state = connected
            ? simulation.State(drones.at(drone.name).id)
            : drones.at(drone.name).observation.lastKnownState;

        // 통신이 끊긴 드론은 마지막 위치에 멈춰 있는 장애물처럼 다룬다.
        if (!connected)
            state.velocityMetersPerSecond = glm::dvec3{0.0};

        const ScenarioResult::MissionTask* task = connected
            ? FindActiveMissionTask(*mission, drone.name)
            : nullptr;
        if (task != nullptr
            && !DroneModel::CanAcceptInFlightCommands(state.mode))
            task = nullptr;

        motionIntents.push_back({
            drone.name,
            state.positionMeters,
            MissionVelocityIntent(state, task, definition,
                activeAvoidanceTargets, true),
            MissionVelocityIntent(state, task, definition,
                activeAvoidanceTargets, false),
            connected,
            task != nullptr});
    }

    const flight_planning::CollisionAvoidancePlan plan =
        flight_planning::PlanCollisionAvoidance(motionIntents,
            *definition.collisionAvoidance, activeAvoidanceTargets);
    if (plan.minimumPredictedSeparationMeters
        && (!result.minimumPredictedDroneSeparationMeters
            || *plan.minimumPredictedSeparationMeters
                < *result.minimumPredictedDroneSeparationMeters))
    {
        result.minimumPredictedDroneSeparationMeters =
            plan.minimumPredictedSeparationMeters;
    }

    for (const ScenarioDrone& drone : definition.drones)
    {
        if (!plan.activeDroneNames.contains(drone.name))
            continue;

        const auto plannedTarget = plan.targetsByDrone.find(drone.name);
        if (plannedTarget == plan.targetsByDrone.end())
            continue;

        // 처음 회피할 때만 목표를 저장한다. 다음 Tick에도 같은 목표를 유지해
        // 매번 경로가 바뀌며 기체가 좌우로 흔들리는 현상을 막는다.
        const auto [activeTarget, newlyAvoiding] =
            activeAvoidanceTargets.try_emplace(drone.name, plannedTarget->second);
        const CommandResult commandResult = simulation.Apply(drones.at(drone.name).id,
            {DroneCommand::Kind::SetWaypoint, activeTarget->second});
        if (commandResult != CommandResult::Accepted)
        {
            result.issues.push_back(drone.name
                + " 드론에 충돌 회피 경로를 전달하지 못했습니다.");
            continue;
        }

        if (newlyAvoiding)
        {
            ++result.collisionAvoidanceEvents;
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + drone.name + " 드론이 예상 충돌을 피해 옆 경로로 이동합니다.");
        }
    }

    for (const auto& [droneName, target] : activeAvoidanceTargets)
    {
        static_cast<void>(target);
        if (!plan.activeDroneNames.contains(droneName))
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + droneName + " 드론이 위험 구간을 벗어나 조사 경로로 돌아갑니다.");
        }
    }
    std::erase_if(activeAvoidanceTargets,
        [&plan](const auto& entry)
        {
            return !plan.activeDroneNames.contains(entry.first);
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
        // 임무 단계 변경은 작업 객체가 맡아 시작·실패·완료 상태가 서로 겹치지 않게 한다.
        if (!task.IsActive()
            || !IsConnected(drones, task.droneName))
            continue;

        const DroneId droneId = drones.at(task.droneName).id;
        const glm::dvec3 distance = simulation.State(droneId).positionMeters
            - task.CurrentWaypoint();
        const bool reachedWaypoint = glm::length(distance) <= task.toleranceMeters;
        // 도착한 지점 뒤에 경유점이 남아 있으면 다음 지점으로, 마지막이면 완료로 전환한다.
        if (reachedWaypoint && task.AdvanceWaypoint())
        {
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.name + "' 경유점 "
                + std::to_string(task.CurrentWaypointIndex() + 1U) + "/"
                + std::to_string(task.waypointsMeters.size()) + "에 도착했습니다.");
        }
        else if (reachedWaypoint)
        {
            task.Complete(tick);
            result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                + task.droneName + " 드론이 '" + task.name + "' 임무를 마쳤습니다.");
        }
        else if (tick >= task.deadlineTick)
        {
            task.Fail();
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
        drone.observation.RecordHeartbeat(0, initialState, std::nullopt);
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
        if (observation.IsConnected())
        {
            const std::optional<double> batteryPercent = simulation.HasBattery(drone.id)
                ? std::optional<double>{simulation.Battery(drone.id).RemainingPercent()}
                : observation.lastKnownBatteryPercent;
            observation.RecordHeartbeat(
                tickCount, simulation.State(drone.id), batteryPercent);
        }
    }
    PopulateFinalSnapshot(definition, simulation, drones, result);
    detail::EvaluateExpectedResults(definition, result);
    return result;
}

}
