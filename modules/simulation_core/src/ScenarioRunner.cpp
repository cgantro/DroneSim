#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"
#include "ScenarioDefinition.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
using DroneIds = std::unordered_map<std::string, DroneId>;

bool CanReceiveWaypoint(FlightMode mode) noexcept
{
    return mode == FlightMode::Hovering
        || mode == FlightMode::Flying
        || mode == FlightMode::FollowingWaypoint;
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
                result.minimumDroneSeparationMeters = std::sqrt(distanceSquaredMeters);
            }
        }
    }
}

bool PrepareScheduledCommandDelivery(
    std::uint64_t tick,
    const ScheduledCommand& scheduled,
    std::unordered_map<std::string, std::uint64_t>& lastDeliveredSequence,
    ScenarioResult& result)
{
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
    std::size_t& nextCommand,
    std::unordered_map<std::string, std::uint64_t>& lastDeliveredSequence,
    ScenarioResult& result)
{
    while (nextCommand < commands.size() && commands[nextCommand].tick == tick)
    {
        const ScheduledCommand& scheduled = commands[nextCommand++];
        if (!PrepareScheduledCommandDelivery(
                tick, scheduled, lastDeliveredSequence, result))
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
    ScenarioResult& result)
{
    if (!mission)
        return;

    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (task.completed || task.failed || tick < task.startTick)
            continue;

        const DroneId droneId = droneIds.at(task.droneName);
        if (!CanReceiveWaypoint(simulation.State(droneId).mode))
        {
            task.failed = true;
            result.issues.push_back(task.started
                ? task.droneName + " 드론이 비행을 멈춰 임무를 이어 가지 못했습니다."
                : task.droneName + " 드론이 비행 중이 아니어서 임무를 시작하지 못했습니다.");
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
    ScenarioResult& result)
{
    for (const FormationLink& link : formationLinks)
    {
        if (!UpdateFormationOutageState(tick, link, result))
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
    ScenarioResult& result)
{
    if (!mission)
        return;

    for (ScenarioResult::MissionTask& task : mission->tasks)
    {
        if (!task.started || task.completed || task.failed)
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
    const DroneIds& droneIds,
    ScenarioResult& result)
{
    result.tickCount = simulation.TickCount();
    result.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    if (definition.drones.empty())
    {
        result.drones.push_back({definition.primaryDroneName,
            simulation.State(droneIds.at(definition.primaryDroneName))});
    }
    else
    {
        for (const ScenarioDrone& drone : definition.drones)
            result.drones.push_back({drone.name, simulation.State(droneIds.at(drone.name))});
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
        ApplyScheduledCommands(tick, definition.commands, simulation, droneIds,
            nextCommand, lastDeliveredSequence, result);
        UpdateMissionCommands(tick, result.mission, simulation, droneIds, result);
        UpdateFormationLinks(tick, definition.formationLinks, simulation, droneIds, result);
        ApplyCameraEvents(tick, definition.cameraEvents, simulation, nextCameraEvent, result);
        simulation.AdvanceOneTick();
        RecordMinimumDroneSeparation(
            definition, simulation, droneIds, dronePositionsMeters, result);
        UpdateMissionResults(tick, result.mission, simulation, droneIds, result);
    }

    PopulateFinalSnapshot(definition, simulation, droneIds, result);
    CheckExpectedResults(definition, result);
    return result;
}

}
