#include "ScenarioDefinition.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace aerosimlab::scenario::detail
{
using namespace aerosimlab::simulation;

namespace
{
constexpr std::uint64_t maximumTrajectoryDroneSamples = 100'000;

std::unordered_set<std::string> GetDroneNames(const ScenarioDefinition& definition)
{
    std::unordered_set<std::string> names;
    for (const ScenarioDrone& drone : definition.drones)
        names.insert(drone.name);
    return names;
}

void ValidateCommands(const ScenarioDefinition& definition,
    const std::unordered_set<std::string>& droneNames)
{
    std::uint64_t previousTick = 0;
    bool hasPreviousTick = false;
    for (const TickCommand& command : definition.commands)
    {
        if (!droneNames.contains(command.droneName))
            throw std::invalid_argument(
                "명령이 가리키는 드론을 찾을 수 없습니다: " + command.droneName);
        if (command.tick >= definition.tickCount)
            throw std::invalid_argument("명령 tick은 시나리오 실행 범위 안에 있어야 합니다.");
        if (hasPreviousTick && command.tick < previousTick)
            throw std::invalid_argument("명령은 tick 순서대로 작성해야 합니다.");
        previousTick = command.tick;
        hasPreviousTick = true;
    }
}

void ValidateCommunicationOutages(const ScenarioDefinition& definition,
    const std::unordered_set<std::string>& droneNames)
{
    std::unordered_map<std::string, std::uint64_t> previousEndByDrone;
    for (const DroneCommunicationOutage& outage : definition.communicationOutages)
    {
        const auto previousEnd = previousEndByDrone.find(outage.droneName);
        if (!droneNames.contains(outage.droneName)
            || outage.startTick >= outage.endTick
            || outage.endTick > definition.tickCount
            || outage.heartbeatTimeoutTicks == 0
            || (previousEnd != previousEndByDrone.end()
                && outage.startTick < previousEnd->second))
        {
            throw std::invalid_argument(
                "통신 장애는 등록된 드론의 겹치지 않는 Tick 구간과 양수 하트비트 제한을 지정해야 합니다.");
        }
        previousEndByDrone[outage.droneName] = outage.endTick;
    }
}

void ValidateMission(const ScenarioDefinition& definition,
    const std::unordered_set<std::string>& droneNames)
{
    if (!definition.mission)
        return;

    std::unordered_set<std::string> taskNames;
    std::unordered_set<std::string> zoneIds;
    for (const ScenarioMissionTask& task : definition.mission->tasks)
    {
        const bool hasNegativeAltitude = std::any_of(task.waypointsMeters.begin(),
            task.waypointsMeters.end(),
            [](const glm::dvec3& point) { return point.z < 0.0; });
        if (task.name.empty() || task.role.empty()
            || !taskNames.insert(task.name).second
            || !droneNames.contains(task.droneName))
            throw std::invalid_argument(
                "임무 이름은 고유해야 하며 등록된 드론을 지정해야 합니다.");
        if (!task.inspectionZoneId.empty()
            && !zoneIds.insert(task.inspectionZoneId).second)
            throw std::invalid_argument("inspection_zone_id는 임무 안에서 고유해야 합니다.");
        if (hasNegativeAltitude || task.toleranceMeters < 0.0
            || task.startTick > task.deadlineTick
            || task.deadlineTick >= definition.tickCount)
        {
            throw std::invalid_argument(
                "임무 목표 고도와 허용 오차는 0 이상이고, 시작·기한 Tick은 시나리오 범위 안이어야 합니다.");
        }
    }
}

void ValidateRigidBodySettings(const ScenarioDefinition& definition)
{
    if (definition.windVelocity
        && definition.dynamics != SimulationDynamics::RigidBody)
        throw std::invalid_argument("바람은 rigid_body 시나리오에서만 설정할 수 있습니다.");

    for (const ScenarioDrone& drone : definition.drones)
    {
        if (drone.hasExtendedInitialState
            && definition.dynamics != SimulationDynamics::RigidBody)
            throw std::invalid_argument(
                "세부 초기 상태는 rigid_body 시나리오에서만 사용할 수 있습니다.");
    }

    const bool hasRotorCommand = std::any_of(
        definition.commands.begin(), definition.commands.end(),
        [](const TickCommand& command)
        {
            return command.command.kind == DroneCommand::Kind::SetRotorThrusts;
        });
    if (hasRotorCommand && definition.dynamics != SimulationDynamics::RigidBody)
        throw std::invalid_argument(
            "로터 추력 재생 명령은 rigid_body 시나리오에서만 사용할 수 있습니다.");

    if (definition.quadrotorParametersCustomized
        && definition.dynamics != SimulationDynamics::RigidBody)
        throw std::invalid_argument(
            "quadrotor_model은 rigid_body 시나리오의 JSON 객체여야 합니다.");

    if (definition.controllerParametersCustomized
        && definition.dynamics != SimulationDynamics::RigidBody)
        throw std::invalid_argument(
            "controller_gains는 rigid_body 시나리오의 JSON 객체여야 합니다.");
}

void ValidateTrajectorySize(const ScenarioDefinition& definition)
{
    if (!definition.trajectorySampleEveryTicks)
        return;

    const std::uint64_t interval = *definition.trajectorySampleEveryTicks;
    const std::uint64_t sampleCount = definition.tickCount / interval + 1
        + (definition.tickCount % interval != 0 ? 1 : 0);
    const std::uint64_t droneCount =
        static_cast<std::uint64_t>(definition.drones.size());
    if (sampleCount > maximumTrajectoryDroneSamples / droneCount)
        throw std::invalid_argument(
            "궤적 기록이 너무 큽니다. sample_every_ticks를 늘려 주세요.");
}

void ValidateMultiDroneSettings(const ScenarioDefinition& definition)
{
    if (definition.collisionAvoidance && definition.drones.size() < 2)
        throw std::invalid_argument(
            "collision_avoidance는 드론 두 대 이상인 시나리오에서 설정해야 합니다.");

    if (definition.minimumDroneSeparationMeters
        && definition.drones.size() < 2)
        throw std::invalid_argument(
            "minimum_drone_separation_meters는 여러 드론이 있는 시나리오에서만 설정할 수 있습니다.");
}

void ValidateEnergySettings(const ScenarioDefinition& definition)
{
    if (!definition.energyModel)
    {
        const bool hasDroneEnergyOverrides = std::any_of(
            definition.drones.begin(), definition.drones.end(),
            [](const ScenarioDrone& drone)
            {
                return drone.batteryCapacityWattHours.has_value()
                    || drone.initialBatteryPercent != 100.0;
            });
        if (hasDroneEnergyOverrides)
            throw std::invalid_argument("드론 배터리 설정을 쓰려면 energy_model을 지정해야 합니다.");
        return;
    }

    const EnergyModelSettings& settings = *definition.energyModel;
    constexpr double secondsPerHour = 3'600.0;
    const double landingEnergyWattHours = settings.flightPowerWatts
        * DroneModel::takeoffHeightMeters
        / DroneModel::climbSpeedMetersPerSecond / secondsPerHour
        + settings.flightPowerWatts * DroneModel::fixedStepSeconds / secondsPerHour;
    for (const ScenarioDrone& drone : definition.drones)
    {
        const double capacityWattHours = drone.batteryCapacityWattHours.value_or(
            settings.capacityWattHours);
        const double reserveWattHours = capacityWattHours
            * (settings.reservePercent / 100.0);
        const double initialWattHours = capacityWattHours
            * (drone.initialBatteryPercent / 100.0);
        if (reserveWattHours < landingEnergyWattHours
            || initialWattHours < landingEnergyWattHours)
            throw std::invalid_argument(drone.name
                + " 드론의 배터리 예비량은 기본 착륙에 필요한 에너지보다 커야 합니다.");
    }
}
}

/** @brief 다 읽은 설정 사이에 함께 지켜야 할 조건을 확인한다. */
void ValidateScenarioDefinition(const ScenarioDefinition& definition)
{
    const std::unordered_set<std::string> droneNames = GetDroneNames(definition);
    ValidateCommands(definition, droneNames);
    ValidateCommunicationOutages(definition, droneNames);
    ValidateMission(definition, droneNames);
    ValidateRigidBodySettings(definition);
    ValidateTrajectorySize(definition);
    ValidateMultiDroneSettings(definition);
    ValidateEnergySettings(definition);
}
}
