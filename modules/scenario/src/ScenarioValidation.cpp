#include "scenario/ScenarioDefinition.h"

#include <algorithm>
#include <cmath>
#include <numbers>
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

/** @brief 정의에 등록된 드론 이름을 명령·임무 참조 검사에 쓸 집합으로 만든다. */
std::unordered_set<std::string> GetDroneNames(const ScenarioDefinition& definition)
{
    std::unordered_set<std::string> names;
    for (const ScenarioDrone& drone : definition.drones)
        names.insert(drone.name);
    return names;
}

/** @brief 명령의 대상 드론이 있고 모든 명령 Tick이 순서대로 실행 범위에 있는지 확인한다. */
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
        // 같은 Tick의 명령은 입력 순서대로 처리할 수 있지만, 시간을 거슬러
        // 정렬되지 않은 명령은 실행 중 빠뜨릴 수 있으므로 거부한다.
        if (hasPreviousTick && command.tick < previousTick)
            throw std::invalid_argument("명령은 tick 순서대로 작성해야 합니다.");
        previousTick = command.tick;
        hasPreviousTick = true;
    }
}

/** @brief 통신 장애가 등록된 드론에 대해 유효하고 서로 겹치지 않는지 확인한다. */
void ValidateCommunicationOutages(const ScenarioDefinition& definition,
    const std::unordered_set<std::string>& droneNames)
{
    std::unordered_map<std::string, std::uint64_t> previousEndByDrone;
    for (const DroneCommunicationOutage& outage : definition.communicationOutages)
    {
        // 같은 드론의 장애 구간은 입력 순서대로 적고 서로 겹치지 않아야 한다.
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

/** @brief 임무 이름·담당 드론·고도·기한·도착 허용 오차를 확인한다. */
void ValidateMission(const ScenarioDefinition& definition,
    const std::unordered_set<std::string>& droneNames)
{
    if (!definition.mission)
        return;

    std::unordered_set<std::string> taskNames;
    for (const ScenarioMissionTask& task : definition.mission->tasks)
    {
        // 이 M&S에서는 모든 임무 지점을 지면 이상 고도로 제한한다.
        const bool hasNegativeAltitude = std::any_of(task.waypointsMeters.begin(),
            task.waypointsMeters.end(),
            [](const glm::dvec3& point) { return point.z < 0.0; });
        if (task.name.empty() || !taskNames.insert(task.name).second
            || !droneNames.contains(task.droneName))
            throw std::invalid_argument(
                "임무 이름은 고유해야 하며 등록된 드론을 지정해야 합니다.");
        if (hasNegativeAltitude || task.toleranceMeters < 0.0
            || task.startTick > task.deadlineTick
            || task.deadlineTick >= definition.tickCount)
        {
            throw std::invalid_argument(
                "임무 목표 고도와 허용 오차는 0 이상이고, 시작·기한 Tick은 시나리오 범위 안이어야 합니다.");
        }
    }
}

/** @brief 기록할 모든 드론 궤적 표본 수가 메모리 한도 안에 있는지 확인한다. */
void ValidateTrajectorySize(const ScenarioDefinition& definition)
{
    if (!definition.trajectorySampleEveryTicks)
        return;

    const std::uint64_t interval = *definition.trajectorySampleEveryTicks;
    // 궤적은 매 샘플마다 모든 드론 상태를 저장하므로, 예상 자료량을
    // 상한과 나누어 비교해 곱셈 오버플로도 피한다.
    const std::uint64_t sampleCount = definition.tickCount / interval + 1
        + (definition.tickCount % interval != 0 ? 1 : 0);
    const std::uint64_t droneCount =
        static_cast<std::uint64_t>(definition.drones.size());
    if (sampleCount > maximumTrajectoryDroneSamples / droneCount)
        throw std::invalid_argument(
            "궤적 기록이 너무 큽니다. sample_every_ticks를 늘려 주세요.");
}

/** @brief 둘 이상의 기체가 있어야 의미가 있는 충돌 설정인지 확인한다. */
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

/** @brief 드론별 카메라·배터리·모터 초기값이 실제 모델 범위에 맞는지 확인한다. */
void ValidateDroneValues(const ScenarioDefinition& definition)
{
    if (!definition.defaultCameraModel.IsValid())
        throw std::invalid_argument(
            "카메라 해상도는 양수이고 세로 시야각은 0도보다 크고 180도보다 작아야 합니다.");

    for (const ScenarioDrone& drone : definition.drones)
    {
        if (drone.cameraModel && !drone.cameraModel->IsValid())
            throw std::invalid_argument(
                "카메라 해상도는 양수이고 세로 시야각은 0도보다 크고 180도보다 작아야 합니다.");
        if (drone.batteryCapacityWattHours
            && *drone.batteryCapacityWattHours <= 0.0)
            throw std::invalid_argument(
                "battery_capacity_watt_hours는 양수여야 합니다.");
        if (drone.initialBatteryPercent < 0.0
            || drone.initialBatteryPercent > 100.0)
            throw std::invalid_argument(
                "initial_battery_percent는 0부터 100 사이여야 합니다.");
        if (std::any_of(drone.initialState.motorSpeedsRadiansPerSecond.begin(),
                drone.initialState.motorSpeedsRadiansPerSecond.end(),
                [](double speed) { return speed < 0.0; }))
            throw std::invalid_argument("초기 로터 속도는 0 이상이어야 합니다.");
    }
}

/** @brief 기체·제어기·충돌 회피 설정에 물리적으로 쓸 수 있는 값을 요구한다. */
void ValidatePhysicsSettings(const ScenarioDefinition& definition)
{
    const QuadrotorParameters& model = definition.quadrotorParameters;
    const glm::dvec3& inertia = model.principalInertiaKilogramMetersSquared;
    if (definition.quadrotorParametersCustomized
        && (model.massKilograms <= 0.0 || inertia.x <= 0.0 || inertia.y <= 0.0
            || inertia.z <= 0.0
            || model.thrustCoefficientNewtonsPerSpeedSquared <= 0.0
            || model.momentToThrustRatio <= 0.0
            || model.motorTimeConstantUpSeconds <= 0.0
            || model.motorTimeConstantDownSeconds <= 0.0
            || model.maximumMotorSpeedRadiansPerSecond <= 0.0))
        throw std::invalid_argument(
            "기체 질량·관성·추력·토크·모터 시간·최대 속도는 모두 양수여야 합니다.");

    const QuadrotorControllerParameters& controller =
        definition.controllerParameters;
    if (definition.controllerParametersCustomized
        && (controller.positionGain <= 0.0 || controller.velocityGain <= 0.0
            || controller.attitudeGain <= 0.0 || controller.angularRateGain <= 0.0))
        throw std::invalid_argument("controller_gains 값은 모두 양수여야 합니다.");

    if (definition.collisionAvoidance)
    {
        const CollisionAvoidanceSettings& settings = *definition.collisionAvoidance;
        if (settings.safetyDistanceMeters <= 0.0
            || settings.predictionHorizonSeconds <= 0.0
            || settings.lateralOffsetMeters < settings.safetyDistanceMeters)
            throw std::invalid_argument(
                "충돌 회피 거리는 양수여야 하고 옆 이동 거리는 안전거리 이상이어야 합니다.");
    }
}

/** @brief 배터리 용량·전력과 예비 비율이 소모 계산에 쓸 수 있는지 확인한다. */
void ValidateEnergyModelValues(const ScenarioDefinition& definition)
{
    if (!definition.energyModel)
        return;

    const EnergyModelSettings& settings = *definition.energyModel;
    if (settings.capacityWattHours <= 0.0
        || settings.hoverPowerWatts <= 0.0
        || settings.flightPowerWatts <= 0.0
        || settings.reservePercent < 0.0 || settings.reservePercent >= 100.0)
        throw std::invalid_argument(
            "배터리 용량과 소비 전력은 양수이고 reserve_percent는 0 이상 100 미만이어야 합니다.");
}

/** @brief 기대 결과를 비교할 때 쓸 위치·자세 오차와 간격 기준을 확인한다. */
void ValidateExpectedValues(const ScenarioDefinition& definition)
{
    if (definition.toleranceMeters < 0.0)
        throw std::invalid_argument("tolerance_meters는 0 이상이어야 합니다.");
    if (definition.orientationToleranceRadians < 0.0
        || definition.orientationToleranceRadians > std::numbers::pi)
        throw std::invalid_argument(
            "orientation_tolerance_radians는 0부터 π 사이여야 합니다.");
    if (definition.minimumDroneSeparationMeters
        && *definition.minimumDroneSeparationMeters < 0.0)
        throw std::invalid_argument(
            "minimum_drone_separation_meters는 0 이상이어야 합니다.");
}

/** @brief 낮은 배터리로 시작하거나 착륙 예비량이 부족한 설정을 거른다. */
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
    // 귀환 지점은 이륙 높이에서 시작한다. 수직 하강 시간 동안의 비행 전력과
    // 착륙 명령이 다음 Tick에 반영될 때까지의 한 Tick 전력을 예비량으로 잡는다.
    constexpr double assumedDescentSpeedMetersPerSecond = 1.0;
    const double descentTimeSeconds = DroneModel::takeoffHeightMeters
        / assumedDescentSpeedMetersPerSecond;
    const double landingEnergyWattHours = settings.flightPowerWatts
        * (descentTimeSeconds + DroneModel::fixedStepSeconds) / secondsPerHour;
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

/**
 * @brief 실행 설정의 값 범위와 항목 사이의 조건을 확인한다.
 * @details 개별 값의 허용 범위, 드론 이름 참조, 비행 방식별 설정,
 * 장애 구간과 임무 시각 순서, 배터리 귀환 가능성을 차례로 검사한다.
 */
void ValidateScenarioDefinition(const ScenarioDefinition& definition)
{
    const std::unordered_set<std::string> droneNames = GetDroneNames(definition);
    ValidateDroneValues(definition);
    ValidatePhysicsSettings(definition);
    ValidateEnergyModelValues(definition);
    ValidateExpectedValues(definition);
    ValidateCommands(definition, droneNames);
    ValidateCommunicationOutages(definition, droneNames);
    ValidateMission(definition, droneNames);
    ValidateTrajectorySize(definition);
    ValidateMultiDroneSettings(definition);
    ValidateEnergySettings(definition);
}
}
