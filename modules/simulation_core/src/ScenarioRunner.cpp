#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"
#include "ScenarioDefinition.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

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

bool CanReceiveWaypoint(FlightMode mode) noexcept
{
    return mode == FlightMode::Hovering
        || mode == FlightMode::Flying
        || mode == FlightMode::FollowingWaypoint;
}

double OrientationErrorRadians(
    const glm::dquat& actual, const glm::dquat& expected) noexcept
{
    const double dot = std::abs(
        actual.w * expected.w + actual.x * expected.x
        + actual.y * expected.y + actual.z * expected.z);
    return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}
}

ScenarioResult RunScenario(std::string_view jsonText)
{
    ScenarioDefinition definition = detail::ReadScenarioDefinition(jsonText);
    const std::uint64_t tickCount = definition.tickCount;
    const auto& scenarioDrones = definition.drones;
    const auto& primaryDroneName = definition.primaryDroneName;
    const auto& commands = definition.commands;
    const auto& formationLinks = definition.formationLinks;
    const auto& cameraEvents = definition.cameraEvents;
    const auto& expectedDrones = definition.expectedDrones;
    const bool hasExpectedDrones = definition.hasExpectedDrones;
    const double toleranceMeters = definition.toleranceMeters;
    const double orientationToleranceRadians = definition.orientationToleranceRadians;
    const auto& expectedMinimumDroneSeparationMeters = definition.minimumDroneSeparationMeters;
    const auto& expectedCameraStatus = definition.expectedCameraStatus;
    const auto& expectedFrameCount = definition.expectedFrameCount;

    SimulationCore simulation{definition.dynamics};
    if (definition.windVelocity)
        simulation.SetWindVelocityMetersPerSecond(*definition.windVelocity);

    std::unordered_map<std::string, DroneId> droneIds;
    if (scenarioDrones.empty())
        droneIds.emplace(primaryDroneName, simulation.PrimaryDroneId());
    else
    {
        const DroneId unusedDefaultDrone = simulation.PrimaryDroneId();
        for (const ScenarioDrone& drone : scenarioDrones)
            droneIds.emplace(drone.name, simulation.AddDrone(
                drone.initialPositionMeters, drone.initialYawRadians));
        if (!simulation.RemoveDrone(unusedDefaultDrone))
            throw std::logic_error("시나리오용 기본 드론을 교체하지 못했습니다.");
    }

    ScenarioResult result;
    result.mission = std::move(definition.mission);
    const auto recordMinimumDroneSeparation = [&]()
    {
        if (!expectedMinimumDroneSeparationMeters)
            return;
        for (std::size_t first = 0; first < scenarioDrones.size(); ++first)
        {
            for (std::size_t second = first + 1; second < scenarioDrones.size(); ++second)
            {
                const glm::dvec3 offset = simulation.State(droneIds.at(scenarioDrones[first].name))
                    .positionMeters
                    - simulation.State(droneIds.at(scenarioDrones[second].name)).positionMeters;
                const double distanceMeters = glm::length(offset);
                if (!result.minimumDroneSeparationMeters
                    || distanceMeters < *result.minimumDroneSeparationMeters)
                {
                    result.minimumDroneSeparationMeters = distanceMeters;
                }
            }
        }
    };
    recordMinimumDroneSeparation();
    std::size_t nextCommand = 0;
    std::size_t nextCameraEvent = 0;
    std::unordered_map<std::string, std::uint64_t> lastDeliveredSequence;
    for (std::uint64_t tick = 0; tick < tickCount; ++tick)
    {
        while (nextCommand < commands.size() && commands[nextCommand].tick == tick)
        {
            const ScheduledCommand& scheduled = commands[nextCommand];
            if (scheduled.dropped)
            {
                result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                    + scheduled.droneName + " 드론 명령을 통신 장애로 버렸습니다.");
                ++nextCommand;
                continue;
            }

            std::uint64_t& lastSequence = lastDeliveredSequence[scheduled.droneName];
            if (scheduled.sequence <= lastSequence)
            {
                result.issues.push_back("tick " + std::to_string(tick) + "에서 "
                    + scheduled.droneName + " 드론의 역순 명령을 거부했습니다.");
                ++nextCommand;
                continue;
            }
            lastSequence = scheduled.sequence;
            if (scheduled.delayTicks > 0)
            {
                result.eventLog.push_back(scheduled.droneName + " 드론에 tick "
                    + std::to_string(scheduled.sendTick)
                    + "에 보낸 명령을 " + std::to_string(scheduled.delayTicks)
                    + " Tick 늦게 전달했습니다.");
            }
            const CommandResult commandResult = simulation.Apply(
                droneIds.at(scheduled.droneName), scheduled.command);
            if (commandResult != CommandResult::Accepted)
                result.issues.push_back("tick " + std::to_string(tick) + "에서 "
                    + scheduled.droneName + " 드론 명령을 적용하지 못했습니다.");
            else
                result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                    + scheduled.droneName + " 드론 명령을 적용했습니다.");
            if (scheduled.duplicated)
                result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                    + scheduled.droneName + " 드론의 중복 재전송 명령을 한 번만 처리했습니다.");
            ++nextCommand;
        }

        if (result.mission)
        {
            for (ScenarioResult::MissionTask& task : result.mission->tasks)
            {
                if (task.completed || task.failed || tick < task.startTick)
                    continue;

                const DroneId droneId = droneIds.at(task.droneName);
                if (!task.started)
                {
                    if (!CanReceiveWaypoint(simulation.State(droneId).mode))
                    {
                        task.failed = true;
                        result.issues.push_back(task.droneName
                            + " 드론이 비행 중이 아니어서 임무를 시작하지 못했습니다.");
                        continue;
                    }
                    task.started = true;
                    result.eventLog.push_back("tick " + std::to_string(tick) + ": "
                        + task.droneName + " 드론이 '" + task.role + "' 임무를 시작했습니다.");
                }
                else if (!CanReceiveWaypoint(simulation.State(droneId).mode))
                {
                    task.failed = true;
                    result.issues.push_back(task.droneName
                        + " 드론이 비행을 멈춰 임무를 이어 가지 못했습니다.");
                    continue;
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

        for (const FormationLink& link : formationLinks)
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

            if (!linkAvailable)
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
            {
                result.issues.push_back(link.followerName
                    + " 드론이 공유 위치를 따라가지 못했습니다.");
            }
        }

        while (nextCameraEvent < cameraEvents.size() && cameraEvents[nextCameraEvent].tick == tick)
        {
            const auto& event = cameraEvents[nextCameraEvent++];
            if (event.action == "enable") simulation.SetCameraEnabled(true);
            else if (event.action == "disable") simulation.SetCameraEnabled(false);
            else if (event.action == "fault") simulation.SetCameraFault(true);
            else simulation.SetCameraFault(false);
            result.eventLog.push_back("tick " + std::to_string(tick) + ": 카메라 " + event.action);
        }
        simulation.AdvanceOneTick();
        recordMinimumDroneSeparation();

        if (result.mission)
        {
            for (ScenarioResult::MissionTask& task : result.mission->tasks)
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
    }

    result.tickCount = simulation.TickCount();
    result.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    if (scenarioDrones.empty())
    {
        result.drones.push_back({primaryDroneName,
            simulation.State(droneIds.at(primaryDroneName))});
    }
    else
    {
        for (const ScenarioDrone& drone : scenarioDrones)
            result.drones.push_back({drone.name, simulation.State(droneIds.at(drone.name))});
    }
    result.finalState = result.drones.front().finalState;
    result.finalCamera = simulation.Camera();
    if (result.mission)
    {
        result.mission->passed = std::all_of(
            result.mission->tasks.begin(), result.mission->tasks.end(),
            [](const ScenarioResult::MissionTask& task) { return task.completed; });
    }
    for (const ExpectedDrone& expectedDrone : expectedDrones)
    {
        const auto found = std::find_if(result.drones.begin(), result.drones.end(),
            [&expectedDrone](const ScenarioResult::Drone& drone)
            {
                return drone.name == expectedDrone.name;
            });
        if (found == result.drones.end())
        {
            result.issues.push_back("예상 결과에 지정한 드론을 찾을 수 없습니다: "
                + expectedDrone.name);
            continue;
        }
        const std::string dronePrefix = hasExpectedDrones
            ? expectedDrone.name + " 드론의 "
            : "";
        if (found->finalState.mode != expectedDrone.mode)
        {
            if (hasExpectedDrones)
                result.issues.push_back(dronePrefix + "최종 비행 상태가 예상과 다릅니다.");
            else
                result.issues.push_back(
                    "예상 비행 상태는 " + std::string(ToString(expectedDrone.mode))
                    + "이지만 실제 상태는 "
                    + std::string(ToString(found->finalState.mode)) + "입니다.");
        }
        if (glm::length(found->finalState.positionMeters - expectedDrone.positionMeters)
            > toleranceMeters)
        {
            result.issues.push_back(dronePrefix + "최종 위치가 허용 오차를 벗어났습니다.");
        }
        if (expectedDrone.orientation
            && OrientationErrorRadians(found->finalState.orientation, *expectedDrone.orientation)
                > orientationToleranceRadians)
        {
            result.issues.push_back(dronePrefix + "최종 자세가 허용 각도 오차를 벗어났습니다.");
        }
    }
    if (hasExpectedDrones && expectedDrones.size() != result.drones.size())
        result.issues.push_back("expected.drones 개수가 시나리오 드론 수와 다릅니다.");
    if (expectedMinimumDroneSeparationMeters
        && result.minimumDroneSeparationMeters
        && *result.minimumDroneSeparationMeters < *expectedMinimumDroneSeparationMeters)
    {
        result.issues.push_back("드론 사이 최소 거리가 설정한 안전 간격보다 짧았습니다.");
    }
    if (expectedCameraStatus && result.finalCamera.status != *expectedCameraStatus)
        result.issues.push_back("최종 카메라 상태가 예상값과 다릅니다.");
    if (expectedFrameCount && result.finalCamera.frameCount != *expectedFrameCount)
        result.issues.push_back("카메라 프레임 수가 예상값과 다릅니다.");
    result.passed = result.issues.empty();
    return result;
}

std::string_view ToString(FlightMode mode) noexcept
{
    switch (mode)
    {
    case FlightMode::Grounded: return "grounded";
    case FlightMode::TakingOff: return "taking_off";
    case FlightMode::Hovering: return "hovering";
    case FlightMode::Flying: return "flying";
    case FlightMode::FollowingWaypoint: return "following_waypoint";
    case FlightMode::Landing: return "landing";
    }
    return "unknown";
}
}
