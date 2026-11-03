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

/** @brief 속도 예측에 현재 회피 목표와 원래 임무 경로 중 무엇을 쓸지 정한다. */
enum class VelocityTargetSource
{
    ActiveAvoidanceTarget,
    AssignedWaypoint
};

/** @brief 시나리오 관제소가 드론별로 보관하는 마지막 통신 상태다. */
class DroneObservation final
{
public:
    /** @brief 통신으로 받은 상태와 배터리 값을 관제소의 마지막 기록으로 저장한다. */
    void RecordHeartbeat(std::uint64_t tick, const DroneState& state,
        std::optional<double> batteryPercent) noexcept
    {
        status_ = DroneCommunicationStatus::Connected;
        lastHeartbeatTick_ = tick;
        lastKnownState_ = state;
        lastKnownBatteryPercent_ = batteryPercent;
    }

    /** @brief 놓친 하트비트의 경과 시간으로 연결 상태를 갱신한다. */
    [[nodiscard]] DroneCommunicationStatus RecordMissedHeartbeat(
        std::uint64_t tick, std::uint64_t timeoutTicks) noexcept
    {
        status_ = tick - lastHeartbeatTick_ >= timeoutTicks
            ? DroneCommunicationStatus::Unknown
            : DroneCommunicationStatus::Stale;
        return status_;
    }

    /** @brief 마지막 하트비트 이후 받은 상태가 있는지 알려준다. */
    [[nodiscard]] DroneCommunicationStatus Status() const noexcept
    {
        return status_;
    }

    /** @brief 마지막으로 통신이 연결됐을 때 저장한 기체 상태를 돌려준다. */
    [[nodiscard]] const DroneState& LastKnownState() const noexcept
    {
        return lastKnownState_;
    }

    /** @brief 마지막 하트비트를 받은 시뮬레이션 Tick을 돌려준다. */
    [[nodiscard]] std::uint64_t LastHeartbeatTick() const noexcept
    {
        return lastHeartbeatTick_;
    }

    /** @brief 마지막으로 보고된 배터리 잔량을 돌려준다. */
    [[nodiscard]] std::optional<double> LastKnownBatteryPercent() const noexcept
    {
        return lastKnownBatteryPercent_;
    }

    /** @brief 관제소가 현재 시각에도 새 하트비트를 받는 중인지 알려준다. */
    [[nodiscard]] bool IsConnected() const noexcept
    {
        return status_ == DroneCommunicationStatus::Connected;
    }

private:
    DroneCommunicationStatus status_ = DroneCommunicationStatus::Connected;
    std::uint64_t lastHeartbeatTick_ = 0;
    DroneState lastKnownState_;
    std::optional<double> lastKnownBatteryPercent_;
};

/** @brief 한 드론의 비행 시간과 이동 거리를 누적한다. */
class FlightMetrics final
{
public:
    /** @brief 시나리오 시작 위치를 거리 계산의 기준점으로 저장한다. */
    void Reset(const glm::dvec3& initialPositionMeters) noexcept
    {
        previousPositionMeters_ = initialPositionMeters;
        flightDistanceMeters_ = 0.0;
        flightTimeSeconds_ = 0.0;
    }

    /** @brief 물리 계산 직전의 비행 상태를 기준으로 한 Tick 시간을 더한다. */
    void RecordTimeBeforePhysics(FlightMode mode) noexcept
    {
        if (mode != FlightMode::Grounded)
            flightTimeSeconds_ += DroneModel::fixedStepSeconds;
    }

    /** @brief 물리 계산이 끝난 위치까지의 직선 이동 거리를 누적한다. */
    void RecordPositionAfterPhysics(const glm::dvec3& positionMeters) noexcept
    {
        flightDistanceMeters_ += glm::distance(
            previousPositionMeters_, positionMeters);
        previousPositionMeters_ = positionMeters;
    }

    [[nodiscard]] double DistanceMeters() const noexcept
    {
        return flightDistanceMeters_;
    }

    [[nodiscard]] double TimeSeconds() const noexcept
    {
        return flightTimeSeconds_;
    }

private:
    glm::dvec3 previousPositionMeters_{0.0};
    double flightDistanceMeters_ = 0.0;
    double flightTimeSeconds_ = 0.0;
};

/** @brief 한 드론의 시나리오 실행 중 상태와 누적값을 함께 관리한다. */
class ScenarioDroneRuntime final
{
public:
    /** @brief 완성된 드론 상태로 실행 정보를 초기화해 유효한 객체를 만든다. */
    ScenarioDroneRuntime(DroneId id, const DroneState& initialState) noexcept
        : id_(id),
          homePositionMeters_(initialState.positionMeters)
    {
        if (initialState.mode == FlightMode::Grounded)
            homePositionMeters_.z += DroneModel::takeoffHeightMeters;

        observation_.RecordHeartbeat(0U, initialState, std::nullopt);
        flightMetrics_.Reset(initialState.positionMeters);
    }

    [[nodiscard]] DroneId Id() const noexcept { return id_; }
    [[nodiscard]] const glm::dvec3& HomePositionMeters() const noexcept
    {
        return homePositionMeters_;
    }
    [[nodiscard]] DroneCommunicationStatus CommunicationStatus() const noexcept
    {
        return observation_.Status();
    }
    [[nodiscard]] bool IsConnected() const noexcept
    {
        return observation_.IsConnected();
    }
    [[nodiscard]] const DroneState& LastKnownState() const noexcept
    {
        return observation_.LastKnownState();
    }
    [[nodiscard]] std::uint64_t LastHeartbeatTick() const noexcept
    {
        return observation_.LastHeartbeatTick();
    }
    [[nodiscard]] std::optional<double> LastKnownBatteryPercent() const noexcept
    {
        return observation_.LastKnownBatteryPercent();
    }
    [[nodiscard]] DroneCommunicationStatus RecordMissedHeartbeat(
        std::uint64_t tick, std::uint64_t timeoutTicks) noexcept
    {
        return observation_.RecordMissedHeartbeat(tick, timeoutTicks);
    }
    void RecordHeartbeat(std::uint64_t tick, const DroneState& state,
        std::optional<double> batteryPercent) noexcept
    {
        observation_.RecordHeartbeat(tick, state, batteryPercent);
    }
    void RecordTimeBeforePhysics(FlightMode mode) noexcept
    {
        flightMetrics_.RecordTimeBeforePhysics(mode);
    }
    void RecordPositionAfterPhysics(const glm::dvec3& positionMeters) noexcept
    {
        flightMetrics_.RecordPositionAfterPhysics(positionMeters);
    }
    [[nodiscard]] double FlightDistanceMeters() const noexcept
    {
        return flightMetrics_.DistanceMeters();
    }
    [[nodiscard]] double FlightTimeSeconds() const noexcept
    {
        return flightMetrics_.TimeSeconds();
    }

private:
    DroneId id_{};
    glm::dvec3 homePositionMeters_{0.0};
    DroneObservation observation_;
    FlightMetrics flightMetrics_;
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
        sample.drones.emplace_back(name, simulation.State(drones.at(name).Id()));
    };
    for (const ScenarioDrone& drone : definition.drones)
        addDrone(drone.name);

    result.trajectory.push_back(std::move(sample));
}

using DroneNameSet = std::unordered_set<std::string>;

/** @brief 모든 실행 기록에 같은 형식으로 Tick 번호를 붙인다. */
void RecordTickEvent(
    ScenarioResult& result, std::uint64_t tick, std::string message)
{
    result.eventLog.push_back(
        "tick " + std::to_string(tick) + ": " + std::move(message));
}

void FailTasksForLowBatteryReturn(
    std::uint64_t tick,
    std::string_view droneName,
    std::optional<ScenarioResult::Mission>& mission,
    ScenarioResult& result)
{
    if (!mission)
        return;

    // 배터리 귀환이 시작되면 드론이 맡은 미완료 작업은 더 진행할 수 없다.
    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (task.droneName != droneName || task.IsCompleted() || task.IsFailed())
            continue;

        task.Fail();
        RecordTickEvent(result, tick,
            "'" + task.name + "' 임무는 미완료로 남습니다.");
    }
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
        const DroneId droneId = drone.Id();
        const bool beganLowBatteryReturn =
            simulation.TryBeginLowBatteryReturn(droneId);
        if (beganLowBatteryReturn)
        {
            RecordTickEvent(result, tick,
                droneName + " 드론이 배터리 예비량에 도달해 출발 지점으로 돌아갑니다.");
            FailTasksForLowBatteryReturn(tick, droneName, mission, result);
        }
        else if (!simulation.Battery(droneId).IsReturningForLowBattery())
        {
            return;
        }

        const DroneState& state = simulation.State(droneId);
        if (!DroneModel::CanAcceptInFlightCommands(state.mode))
            return;

        const glm::dvec3& returnPosition = drone.HomePositionMeters();
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
    return drones.at(droneName).IsConnected();
}

const DroneCommunicationOutage* FindActiveCommunicationOutage(
    std::uint64_t tick,
    const std::string& droneName,
    const std::vector<DroneCommunicationOutage>& outages)
{
    // 장애 구간은 시작 Tick을 포함하고 종료 Tick은 포함하지 않는다.
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
        const DroneId droneId = drone.Id();
        const DroneCommunicationOutage* outage = FindActiveCommunicationOutage(
            tick, droneName, definition.communicationOutages);
        if (outage == nullptr)
        {
            // 연결 중에는 실제 모의 상태를 관제 측 관측값으로 갱신한다.
            // 장애 중에도 비행 계산은 계속되지만 관측값은 이 마지막 보고에 머문다.
            if (drone.CommunicationStatus() != DroneCommunicationStatus::Connected)
            {
                RecordTickEvent(result, tick,
                    droneName + " 드론의 통신이 복구되어 새 상태를 받았습니다.");
            }
            const std::optional<double> batteryPercent = simulation.HasBattery(droneId)
                ? std::optional<double>{simulation.Battery(droneId).RemainingPercent()}
                : std::nullopt;
            drone.RecordHeartbeat(tick, simulation.State(droneId), batteryPercent);
            return;
        }

        const DroneCommunicationStatus previousStatus = drone.CommunicationStatus();
        // 짧은 누락은 Stale, 설정한 제한 시간을 넘긴 누락은 Unknown으로 표시한다.
        const DroneCommunicationStatus newStatus = drone.RecordMissedHeartbeat(
            tick, outage->heartbeatTimeoutTicks);
        // 상태를 Stale로 바꾸기 전에 연결 중이었는지 확인해 첫 끊김만 기록한다.
        if (previousStatus == DroneCommunicationStatus::Connected)
        {
            RecordTickEvent(result, tick,
                droneName + " 드론의 하트비트가 끊겼습니다.");
        }
        if (newStatus == DroneCommunicationStatus::Unknown
            && previousStatus != DroneCommunicationStatus::Unknown)
        {
            RecordTickEvent(result, tick,
                droneName + " 드론의 상태를 확인할 수 없어 임무 정보를 미확정으로 바꿨습니다.");
        }
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
        if (!definition.usesExistingPrimaryDrone && drone.hasExtendedInitialState)
            simulation.SetInitialState(id, drone.initialState);
        drones.emplace(drone.name,
            ScenarioDroneRuntime{id, simulation.State(id)});
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

    // 이 버퍼는 실행 중 재사용한다. clear()는 저장 공간을 유지하므로 매 Tick마다
    // 드론 수만큼 메모리를 새로 할당하지 않는다.
    positionsMeters.clear();
    for (const ScenarioDrone& drone : definition.drones)
    {
        positionsMeters.push_back(
            simulation.State(drones.at(drone.name).Id()).positionMeters);
    }

    // 거리 제곱끼리 비교하면 드론 쌍마다 제곱근을 계산할 필요가 없다.
    // 이전 최솟값도 제곱해 같은 단위로 비교하고, 새 최솟값이 생긴 경우에만 마지막에 제곱근을 구한다.
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
            RecordTickEvent(result, tick, command.droneName
                + " 드론에 명령을 전달하지 못했습니다. 마지막 위치는 이전 보고값입니다.");
            continue;
        }

        if (simulation.Apply(drones.at(command.droneName).Id(), command.command)
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

        const DroneId droneId = drones.at(task.droneName).Id();
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
            RecordTickEvent(result, tick,
                task.droneName + " 드론이 '" + task.name + "' 임무를 시작했습니다.");
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
    VelocityTargetSource targetSource)
{
    if (!task)
        return state.velocityMetersPerSecond;

    const auto avoidanceTarget = activeAvoidanceTargets.find(task->droneName);
    const bool useActiveAvoidanceTarget =
        targetSource == VelocityTargetSource::ActiveAvoidanceTarget;
    const glm::dvec3 waypoint = useActiveAvoidanceTarget
        && avoidanceTarget != activeAvoidanceTargets.end()
        ? avoidanceTarget->second
        : task->CurrentWaypoint();

    glm::dvec3 targetVelocity = QuadrotorFlightController::PositionTargetVelocity(
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
            ? simulation.State(drones.at(drone.name).Id())
            : drones.at(drone.name).LastKnownState();

        // 통신이 끊긴 드론은 마지막 위치에 멈춰 있는 장애물처럼 다룬다.
        if (!connected)
            state.velocityMetersPerSecond = glm::dvec3{0.0};

        const ScenarioResult::MissionTask* task = connected
            ? FindActiveMissionTask(*mission, drone.name)
            : nullptr;
        if (task != nullptr
            && !DroneModel::CanAcceptInFlightCommands(state.mode))
            task = nullptr;

        // 회피 여부를 판단할 때는 실제 진행 목표와 원래 임무 경로를 모두 예측한다.
        // 원래 경로가 안전해지면 임시 회피 목표를 지우고 임무 경로로 돌아갈 수 있다.
        motionIntents.push_back({
            drone.name,
            state.positionMeters,
            MissionVelocityIntent(state, task, definition,
                activeAvoidanceTargets,
                VelocityTargetSource::ActiveAvoidanceTarget),
            MissionVelocityIntent(state, task, definition,
                activeAvoidanceTargets,
                VelocityTargetSource::AssignedWaypoint),
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
        const CommandResult commandResult = simulation.Apply(drones.at(drone.name).Id(),
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
            RecordTickEvent(result, tick,
                drone.name + " 드론이 예상 충돌을 피해 옆 경로로 이동합니다.");
        }
    }

    for (const auto& [droneName, target] : activeAvoidanceTargets)
    {
        static_cast<void>(target);
        if (!plan.activeDroneNames.contains(droneName))
        {
            RecordTickEvent(result, tick,
                droneName + " 드론이 위험 구간을 벗어나 조사 경로로 돌아갑니다.");
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

        const DroneId droneId = drones.at(task.droneName).Id();
        const glm::dvec3 distance = simulation.State(droneId).positionMeters
            - task.CurrentWaypoint();
        const bool reachedWaypoint = glm::length(distance) <= task.toleranceMeters;
        // 도착한 지점 뒤에 경유점이 남아 있으면 다음 지점으로, 마지막이면 완료로 전환한다.
        if (reachedWaypoint && task.AdvanceWaypoint())
        {
            RecordTickEvent(result, tick,
                task.droneName + " 드론이 '" + task.name + "' 경유점 "
                + std::to_string(task.CurrentWaypointIndex() + 1U) + "/"
                + std::to_string(task.waypointsMeters.size()) + "에 도착했습니다.");
        }
        else if (reachedWaypoint)
        {
            task.Complete(tick);
            RecordTickEvent(result, tick,
                task.droneName + " 드론이 '" + task.name + "' 임무를 마쳤습니다.");
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
        ScenarioResult::Drone drone{
            droneName, runtime.LastKnownState(),
            runtime.CommunicationStatus(), runtime.LastHeartbeatTick()};
        drone.finalCamera = simulation.Camera(runtime.Id());
        drone.flightDistanceMeters = runtime.FlightDistanceMeters();
        drone.flightTimeSeconds = runtime.FlightTimeSeconds();
        drone.reportedBatteryPercent = runtime.LastKnownBatteryPercent();
        if (simulation.HasBattery(runtime.Id()))
        {
            const BatteryModel& battery = simulation.Battery(runtime.Id());
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

/**
 * @brief 한 시나리오 실행에 필요한 모의 상태와 실행 순서를 소유한다.
 * @details JSON 해석과 설정 검사는 이 객체를 만들기 전에 끝난다. 객체는 검증된
 * 설정으로 기체를 준비하고, Tick마다 정해진 처리 순서를 실행한 뒤 결과를 만든다.
 */
class ScenarioExecution final
{
public:
    explicit ScenarioExecution(ScenarioDefinition definition)
        : definition_(std::move(definition)),
          simulation_(definition_.quadrotorParameters,
              definition_.controllerParameters)
    {
        if (definition_.windVelocity)
            simulation_.SetWindVelocityMetersPerSecond(*definition_.windVelocity);

        drones_ = CreateScenarioDrones(definition_, simulation_);
        ConfigureDroneBatteries();
        InitializeResult();
    }

    /** @brief 남은 Tick을 실행하고 마지막 상태를 포함한 결과를 돌려준다. */
    [[nodiscard]] ScenarioResult Run()
    {
        for (std::uint64_t tick = 0; tick < definition_.tickCount; ++tick)
            AdvanceOneTick(tick);

        FinalizeResult();
        return std::move(result_);
    }

private:
    void ConfigureDroneBatteries()
    {
        if (!definition_.energyModel)
            return;

        for (const ScenarioDrone& scenarioDrone : definition_.drones)
        {
            BatteryModelParameters parameters = *definition_.energyModel;
            if (scenarioDrone.batteryCapacityWattHours)
                parameters.capacityWattHours = *scenarioDrone.batteryCapacityWattHours;

            ScenarioDroneRuntime& runtime = drones_.at(scenarioDrone.name);
            simulation_.ConfigureBattery(runtime.Id(), parameters,
                scenarioDrone.initialBatteryPercent);
            runtime.RecordHeartbeat(0U, simulation_.State(runtime.Id()),
                simulation_.Battery(runtime.Id()).RemainingPercent());
        }
    }

    void InitializeResult()
    {
        if (definition_.mission)
            result_.mission = CreateMissionResult(std::move(*definition_.mission));

        if (definition_.trajectorySampleEveryTicks)
        {
            const std::uint64_t interval = *definition_.trajectorySampleEveryTicks;
            const std::uint64_t periodicSamples = definition_.tickCount / interval + 1U;
            const std::uint64_t finalSampleNeeded = definition_.tickCount % interval != 0U;
            result_.trajectory.reserve(static_cast<std::size_t>(
                periodicSamples + finalSampleNeeded));
        }

        if (definition_.minimumDroneSeparationMeters)
            dronePositionsMeters_.reserve(definition_.drones.size());

        RecordMinimumDroneSeparation(
            definition_, simulation_, drones_, dronePositionsMeters_, result_);
        if (definition_.trajectorySampleEveryTicks)
            RecordTrajectorySample(definition_, simulation_, drones_, 0U, result_);
    }

    void AdvanceOneTick(std::uint64_t tick)
    {
        // 관제 상태와 입력부터 갱신한다. 통신이 끊긴 동안에는 새 명령을 받지 않는다.
        UpdateDroneObservations(tick, definition_, simulation_, drones_, result_);
        ApplyScheduledCommands(tick, definition_.commands, simulation_, drones_,
            nextCommand_, result_);

        // 배터리 귀환, 임무 목표, 충돌 회피 순으로 물리 이동 전 명령을 정한다.
        if (definition_.energyModel)
        {
            UpdateEnergyPolicy(tick, result_.mission,
                simulation_, drones_, definition_, result_);
        }
        UpdateMissionCommands(tick, result_.mission,
            simulation_, drones_, result_);
        UpdateCollisionAvoidance(tick, definition_, result_.mission,
            simulation_, drones_, activeAvoidanceTargets_, result_);

        for (auto& [droneName, drone] : drones_)
        {
            static_cast<void>(droneName);
            drone.RecordTimeBeforePhysics(
                simulation_.State(drone.Id()).mode);
        }
        simulation_.AdvanceOneTick();
        for (auto& [droneName, drone] : drones_)
        {
            static_cast<void>(droneName);
            drone.RecordPositionAfterPhysics(
                simulation_.State(drone.Id()).positionMeters);
        }

        // 물리 계산 뒤 실제 이동과 임무 진행을 기록하고, 요청한 간격에 맞춰 표본을 남긴다.
        RecordMinimumDroneSeparation(
            definition_, simulation_, drones_, dronePositionsMeters_, result_);
        UpdateMissionResults(tick, result_.mission,
            simulation_, drones_, result_);

        const std::uint64_t completedTick = tick + 1U;
        if (definition_.trajectorySampleEveryTicks)
        {
            const std::uint64_t interval = *definition_.trajectorySampleEveryTicks;
            if (completedTick % interval == 0U
                || completedTick == definition_.tickCount)
            {
                RecordTrajectorySample(
                    definition_, simulation_, drones_, completedTick, result_);
            }
        }
    }

    void FinalizeResult()
    {
        for (auto& [droneName, drone] : drones_)
        {
            static_cast<void>(droneName);
            if (!drone.IsConnected())
                continue;

            const std::optional<double> batteryPercent = simulation_.HasBattery(drone.Id())
                ? std::optional<double>{simulation_.Battery(drone.Id()).RemainingPercent()}
                : drone.LastKnownBatteryPercent();
            drone.RecordHeartbeat(
                definition_.tickCount, simulation_.State(drone.Id()), batteryPercent);
        }

        PopulateFinalSnapshot(definition_, simulation_, drones_, result_);
        detail::EvaluateExpectedResults(definition_, result_);
    }

    ScenarioDefinition definition_;
    SimulationCore simulation_;
    ScenarioDrones drones_;
    ScenarioResult result_;
    std::size_t nextCommand_ = 0U;
    std::unordered_map<std::string, glm::dvec3> activeAvoidanceTargets_;
    std::vector<glm::dvec3> dronePositionsMeters_;
};
}

ScenarioResult RunScenario(std::string_view jsonText)
{
    // 입력을 먼저 자료형으로 읽고 규칙을 검사한다. 잘못된 설정으로
    // 시뮬레이션 객체를 만들거나 Tick을 시작하지 않는다.
    ScenarioDefinition definition = detail::ParseScenarioDefinition(jsonText);
    detail::ValidateScenarioDefinition(definition);
    return ScenarioExecution{std::move(definition)}.Run();
}

}
