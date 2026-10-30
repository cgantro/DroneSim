#include "scenario/ScenarioDefinition.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dronesim::scenario::detail
{
using namespace dronesim::simulation;

namespace
{
using Json = nlohmann::json;

constexpr std::uint64_t maximumScenarioTicks = 5'184'000;

/** @brief 객체에서 필수 항목을 찾고, 빠졌으면 이름을 담아 오류를 알린다. */
const Json& Required(const Json& object, const char* key)
{
    const auto entry = object.find(key);
    if (entry == object.end())
    {
        throw std::invalid_argument(std::string("필수 항목이 없습니다: ") + key);
    }
    return *entry;
}

/** @brief Tick·횟수처럼 음수나 소수가 될 수 없는 값을 읽는다. */
std::uint64_t ReadUnsigned(const Json& value, const char* name)
{
    if (!value.is_number_unsigned() && !value.is_number_integer())
    {
        throw std::invalid_argument(std::string("0 이상의 정수가 필요합니다: ") + name);
    }
    if (value.is_number_integer() && value.get<std::int64_t>() < 0)
    {
        throw std::invalid_argument(std::string("음수는 사용할 수 없습니다: ") + name);
    }
    return value.get<std::uint64_t>();
}

/** @brief 물리 계산에 사용할 숫자를 읽고 무한대와 NaN을 거른다. */
double ReadFiniteNumber(const Json& value, const char* name)
{
    if (!value.is_number())
    {
        throw std::invalid_argument(std::string("숫자가 필요합니다: ") + name);
    }
    const double number = value.get<double>();
    if (!std::isfinite(number))
    {
        throw std::invalid_argument(std::string("유한한 숫자가 필요합니다: ") + name);
    }
    return number;
}

/**
 * @brief 카메라 해상도와 세로 시야각을 카메라 설정으로 옮긴다.
 * @details 해상도는 int로 바꾸기 전에 상한을 확인해 변환 범위를 보장한다.
 * 실제로 사용할 수 있는 값인지는 설정 검증 단계에서 확인한다.
 */
CameraModel ReadCameraModel(const Json& value)
{
    if (!value.is_object())
        throw std::invalid_argument("camera_model은 객체여야 합니다.");

    CameraModel model;
    if (const auto width = value.find("image_width_pixels"); width != value.end())
    {
        const std::uint64_t parsed = ReadUnsigned(*width, "image_width_pixels");
        if (parsed > static_cast<std::uint64_t>(
                CameraModel::maximumImageDimensionPixels))
            throw std::invalid_argument("image_width_pixels는 1부터 8192 사이여야 합니다.");
        model.imageWidthPixels = static_cast<int>(parsed);
    }
    if (const auto height = value.find("image_height_pixels"); height != value.end())
    {
        const std::uint64_t parsed = ReadUnsigned(*height, "image_height_pixels");
        if (parsed > static_cast<std::uint64_t>(
                CameraModel::maximumImageDimensionPixels))
            throw std::invalid_argument("image_height_pixels는 1부터 8192 사이여야 합니다.");
        model.imageHeightPixels = static_cast<int>(parsed);
    }
    if (const auto fov = value.find("vertical_field_of_view_degrees");
        fov != value.end())
        model.verticalFieldOfViewDegrees = ReadFiniteNumber(
            *fov, "vertical_field_of_view_degrees");

    return model;
}

glm::dvec3 ReadVector(const Json& value, const char* name)
{
    if (!value.is_array() || value.size() != 3)
    {
        throw std::invalid_argument(std::string("숫자 3개로 된 배열이 필요합니다: ") + name);
    }
    return glm::dvec3{
        ReadFiniteNumber(value[0], name),
        ReadFiniteNumber(value[1], name),
        ReadFiniteNumber(value[2], name)
    };
}

glm::dquat ReadOrientationQuaternion(const Json& value)
{
    constexpr const char* fieldName = "orientation_quaternion_wxyz";
    if (!value.is_array() || value.size() != 4)
        throw std::invalid_argument(std::string(fieldName) + "에는 숫자 4개가 필요합니다.");

    const glm::dquat input{
        ReadFiniteNumber(value[0], fieldName),
        ReadFiniteNumber(value[1], fieldName),
        ReadFiniteNumber(value[2], fieldName),
        ReadFiniteNumber(value[3], fieldName)};
    // 큰 입력값도 제곱 과정에서 넘치지 않도록 최댓값으로 나눈 뒤 정규화한다.
    const double scale = std::max({
        std::abs(input.w), std::abs(input.x), std::abs(input.y), std::abs(input.z)});
    if (scale == 0.0)
        throw std::invalid_argument(std::string(fieldName) + "은 0이 아닌 사원수여야 합니다.");

    const glm::dquat scaled{
        input.w / scale, input.x / scale, input.y / scale, input.z / scale};
    const double length = std::sqrt(
        scaled.w * scaled.w + scaled.x * scaled.x
        + scaled.y * scaled.y + scaled.z * scaled.z);
    return {
        scaled.w / length, scaled.x / length, scaled.y / length, scaled.z / length};
}

DroneCommand ReadCommand(const Json& value)
{
    if (!value.is_object())
    {
        throw std::invalid_argument("각 명령은 JSON 객체여야 합니다.");
    }

    const Json& typeValue = Required(value, "type");
    if (!typeValue.is_string())
    {
        throw std::invalid_argument("명령 type은 문자열이어야 합니다.");
    }
    const std::string type = typeValue.get<std::string>();
    if (type == "takeoff")
    {
        return {DroneCommand::Kind::TakeOff};
    }
    if (type == "land")
    {
        return {DroneCommand::Kind::Land};
    }
    if (type == "stop")
    {
        return {DroneCommand::Kind::Stop};
    }
    if (type == "set_velocity")
    {
        return {
            DroneCommand::Kind::SetVelocity,
            ReadVector(Required(value, "velocity_meters_per_second"), "velocity_meters_per_second")
        };
    }
    if (type == "set_waypoint")
    {
        return {
            DroneCommand::Kind::SetWaypoint,
            ReadVector(Required(value, "position_meters"), "position_meters")
        };
    }
    if (type == "set_yaw_rate")
    {
        return {
            DroneCommand::Kind::SetYawRate,
            glm::dvec3{0.0},
            ReadFiniteNumber(Required(value, "yaw_rate_radians_per_second"), "yaw_rate_radians_per_second")
        };
    }
    if (type == "set_rotor_thrusts")
    {
        const Json& values = Required(value, "normalized_thrust");
        if (!values.is_array() || values.size() != 4)
            throw std::invalid_argument("normalized_thrust에는 로터별 값 4개가 필요합니다.");
        DroneCommand command;
        command.kind = DroneCommand::Kind::SetRotorThrusts;
        for (std::size_t index = 0; index < command.rotorThrustRatios.size(); ++index)
        {
            command.rotorThrustRatios[index] = ReadFiniteNumber(
                values[index], "normalized_thrust");
            if (command.rotorThrustRatios[index] < 0.0
                || command.rotorThrustRatios[index] > 1.0)
                throw std::invalid_argument("normalized_thrust 값은 0부터 1 사이여야 합니다.");
        }
        return command;
    }
    throw std::invalid_argument(std::string("알 수 없는 명령입니다: ") + type);
}

FlightMode ReadFlightMode(const Json& value)
{
    if (!value.is_string())
    {
        throw std::invalid_argument("expected.mode는 문자열이어야 합니다.");
    }
    const std::string name = value.get<std::string>();
    if (const auto mode = TryParseFlightMode(name))
        return *mode;
    throw std::invalid_argument(std::string("알 수 없는 비행 상태입니다: ") + name);
}

std::string ReadUniqueDroneName(
    const Json& item,
    std::unordered_set<std::string>& names,
    std::string_view droneKind)
{
    const Json& nameValue = Required(item, "name");
    if (!nameValue.is_string())
        throw std::invalid_argument(std::string(droneKind) + " name은 문자열이어야 합니다.");

    std::string name = nameValue.get<std::string>();
    if (name.empty() || !names.insert(name).second)
        throw std::invalid_argument(
            std::string(droneKind) + " name은 비어 있거나 중복될 수 없습니다.");
    return name;
}

std::vector<ExpectedDrone> ReadExpectedDrones(const Json& value)
{
    if (!value.is_array() || value.empty())
        throw std::invalid_argument("expected.drones는 한 대 이상을 적은 배열이어야 합니다.");

    std::vector<ExpectedDrone> drones;
    std::unordered_set<std::string> names;
    drones.reserve(value.size());
    for (const Json& item : value)
    {
        if (!item.is_object())
            throw std::invalid_argument("예상 드론 상태는 JSON 객체여야 합니다.");
        std::string name = ReadUniqueDroneName(item, names, "예상 드론");
        const auto orientation = item.find("orientation_quaternion_wxyz");
        drones.push_back({
            std::move(name),
            ReadFlightMode(Required(item, "mode")),
            ReadVector(Required(item, "position_meters"), "position_meters"),
            orientation == item.end()
                ? std::nullopt
                : std::optional<glm::dquat>{ReadOrientationQuaternion(*orientation)}});
    }
    return drones;
}

std::vector<ScenarioDrone> ReadDrones(const Json& document)
{
    const auto found = document.find("drones");
    if (found == document.end())
        return {};
    if (!found->is_array() || found->empty())
        throw std::invalid_argument("drones는 한 대 이상을 적은 배열이어야 합니다.");

    std::vector<ScenarioDrone> drones;
    std::unordered_set<std::string> names;
    drones.reserve(found->size());
    for (const Json& item : *found)
    {
        if (!item.is_object())
            throw std::invalid_argument("각 드론은 JSON 객체여야 합니다.");
        std::string name = ReadUniqueDroneName(item, names, "드론");
        const auto yaw = item.find("initial_yaw_radians");
        const double initialYawRadians = yaw == item.end()
            ? 0.0
            : ReadFiniteNumber(*yaw, "initial_yaw_radians");
        const auto velocity = item.find("initial_velocity_meters_per_second");
        const auto orientation = item.find("initial_orientation_quaternion_wxyz");
        const auto angularVelocity = item.find(
            "initial_angular_velocity_radians_per_second");
        const auto motorSpeeds = item.find(
            "initial_motor_speeds_radians_per_second");
        const auto mode = item.find("initial_mode");
        // 기본 위치·방향만 지정한 경우와 속도·자세·모터까지 직접 지정한
        // 경우를 구분해, 전체 초기 상태가 있을 때만 물리 상태로 덮어쓴다.
        const bool hasExtendedInitialState = velocity != item.end()
            || orientation != item.end() || angularVelocity != item.end()
            || motorSpeeds != item.end() || mode != item.end();
        DroneState initialState;
        initialState.positionMeters = ReadVector(
            Required(item, "initial_position_meters"), "initial_position_meters");
        initialState.yawRadians = initialYawRadians;
        initialState.orientation = orientation == item.end()
            ? glm::angleAxis(initialYawRadians, glm::dvec3{0.0, 0.0, 1.0})
            : ReadOrientationQuaternion(*orientation);
        if (velocity != item.end())
            initialState.velocityMetersPerSecond = ReadVector(
                *velocity, "initial_velocity_meters_per_second");
        if (angularVelocity != item.end())
            initialState.angularVelocityRadiansPerSecond = ReadVector(
                *angularVelocity, "initial_angular_velocity_radians_per_second");
        if (motorSpeeds != item.end())
        {
            if (!motorSpeeds->is_array()
                || motorSpeeds->size() != initialState.motorSpeedsRadiansPerSecond.size())
                throw std::invalid_argument(
                    "initial_motor_speeds_radians_per_second에는 로터별 값 4개가 필요합니다.");
            for (std::size_t index = 0;
                 index < initialState.motorSpeedsRadiansPerSecond.size(); ++index)
            {
                initialState.motorSpeedsRadiansPerSecond[index] = ReadFiniteNumber(
                    (*motorSpeeds)[index], "initial_motor_speeds_radians_per_second");
            }
        }
        if (mode != item.end())
            initialState.mode = ReadFlightMode(*mode);
        std::optional<double> batteryCapacity;
        const auto capacity = item.find("battery_capacity_watt_hours");
        if (capacity != item.end())
        {
            batteryCapacity = ReadFiniteNumber(
                *capacity, "battery_capacity_watt_hours");
        }
        const auto initialBattery = item.find("initial_battery_percent");
        const double initialBatteryPercent = initialBattery == item.end()
            ? 100.0 : ReadFiniteNumber(*initialBattery, "initial_battery_percent");
        std::optional<CameraModel> cameraModel;
        if (const auto camera = item.find("camera_model"); camera != item.end())
            cameraModel = ReadCameraModel(*camera);
        drones.push_back({std::move(name), initialState, hasExtendedInitialState,
            batteryCapacity, initialBatteryPercent, cameraModel});
    }
    return drones;
}

std::vector<DroneCommunicationOutage> ReadDroneCommunicationOutages(
    const Json& document)
{
    const auto found = document.find("communication_outages");
    if (found == document.end())
        return {};
    if (!found->is_array())
        throw std::invalid_argument("communication_outages는 배열이어야 합니다.");

    std::vector<DroneCommunicationOutage> outages;
    outages.reserve(found->size());
    for (const Json& item : *found)
    {
        if (!item.is_object())
            throw std::invalid_argument("각 통신 장애는 JSON 객체여야 합니다.");
        const Json& droneValue = Required(item, "drone");
        if (!droneValue.is_string())
            throw std::invalid_argument("통신 장애 drone은 이름 문자열이어야 합니다.");

        std::string droneName = droneValue.get<std::string>();
        const std::uint64_t startTick = ReadUnsigned(
            Required(item, "start_tick"), "start_tick");
        const std::uint64_t endTick = ReadUnsigned(
            Required(item, "end_tick"), "end_tick");
        const std::uint64_t heartbeatTimeoutTicks = ReadUnsigned(
            Required(item, "heartbeat_timeout_ticks"), "heartbeat_timeout_ticks");
        outages.push_back({std::move(droneName), startTick, endTick,
            heartbeatTimeoutTicks});
    }
    return outages;
}

/** @brief 임무 항목의 JSON 값을 실행 결과 구조체로 옮긴다. */
ScenarioMissionTask ReadMissionTask(
    const Json& taskValue)
{
    if (!taskValue.is_object())
        throw std::invalid_argument("각 mission task는 JSON 객체여야 합니다.");

    const Json& taskNameValue = Required(taskValue, "name");
    const Json& droneValue = Required(taskValue, "drone");
    if (!taskNameValue.is_string() || !droneValue.is_string())
    {
        throw std::invalid_argument("임무 name과 drone은 문자열이어야 합니다.");
    }

    ScenarioMissionTask task;
    task.name = taskNameValue.get<std::string>();
    task.droneName = droneValue.get<std::string>();
    const Json& waypoints = Required(taskValue, "waypoints_meters");
    if (!waypoints.is_array() || waypoints.empty())
        throw std::invalid_argument("waypoints_meters는 한 점 이상을 담은 배열이어야 합니다.");
    task.waypointsMeters.reserve(waypoints.size());
    for (const Json& waypoint : waypoints)
        task.waypointsMeters.push_back(ReadVector(waypoint, "waypoints_meters 항목"));
    task.startTick = ReadUnsigned(Required(taskValue, "start_tick"), "start_tick");
    task.deadlineTick = ReadUnsigned(
        Required(taskValue, "deadline_tick"), "deadline_tick");
    task.toleranceMeters = ReadFiniteNumber(
        Required(taskValue, "tolerance_meters"), "tolerance_meters");
    return task;
}

std::optional<ScenarioMission> ReadMission(
    const Json& document)
{
    const auto found = document.find("mission");
    if (found == document.end())
        return std::nullopt;
    if (!found->is_object())
        throw std::invalid_argument("mission은 JSON 객체여야 합니다.");

    const Json& nameValue = Required(*found, "name");
    if (!nameValue.is_string() || nameValue.get<std::string>().empty())
        throw std::invalid_argument("mission.name은 비어 있지 않은 문자열이어야 합니다.");

    const Json& tasks = Required(*found, "tasks");
    if (!tasks.is_array() || tasks.empty())
        throw std::invalid_argument("mission.tasks에는 한 개 이상의 작업이 필요합니다.");

    ScenarioMission mission;
    mission.name = nameValue.get<std::string>();
    mission.tasks.reserve(tasks.size());
    for (const Json& taskValue : tasks)
    {
        mission.tasks.push_back(ReadMissionTask(taskValue));
    }
    return mission;
}

std::string ReadCommandDroneName(
    const Json& command,
    const std::string& defaultDroneName)
{
    std::string droneName = defaultDroneName;
    const auto drone = command.find("drone");
    if (drone != command.end())
    {
        if (!drone->is_string())
            throw std::invalid_argument("명령 drone은 드론 name 문자열이어야 합니다.");
        droneName = drone->get<std::string>();
    }
    return droneName;
}

std::vector<TickCommand> ReadCommands(
    const Json& commands,
    const std::string& defaultDroneName)
{
    if (!commands.is_array())
    {
        throw std::invalid_argument("commands는 배열이어야 합니다.");
    }

    std::vector<TickCommand> result;
    result.reserve(commands.size());
    for (const Json& item : commands)
    {
        const std::uint64_t tick = ReadUnsigned(Required(item, "tick"), "tick");
        std::string droneName = ReadCommandDroneName(
            item, defaultDroneName);
        result.push_back({tick, std::move(droneName), ReadCommand(item)});
    }
    return result;
}

CameraStatus ReadCameraStatus(const Json& value)
{
    if (!value.is_string())
        throw std::invalid_argument("expected.camera_status는 문자열이어야 합니다.");
    const std::string status = value.get<std::string>();
    if (status == "active") return CameraStatus::Active;
    if (status == "disabled") return CameraStatus::Disabled;
    if (status == "fault") return CameraStatus::Fault;
    throw std::invalid_argument("알 수 없는 카메라 상태입니다: " + status);
}

Json ParseScenarioDocument(std::string_view jsonText)
{
    Json document;
    try
    {
        document = Json::parse(jsonText);
    }
    catch (const Json::exception& error)
    {
        throw std::invalid_argument(std::string("JSON을 읽지 못했습니다: ") + error.what());
    }
    // 이후 항목 읽기는 키 이름으로 접근하므로 최상위 값은 객체여야 한다.
    if (!document.is_object())
        throw std::invalid_argument("시나리오의 첫 항목은 JSON 객체여야 합니다.");
    return document;
}

void ReadEnvironmentSettings(const Json& document, ScenarioDefinition& definition)
{
    if (document.contains("wind_velocity_meters_per_second"))
    {
        definition.windVelocity = ReadVector(
            document["wind_velocity_meters_per_second"],
            "wind_velocity_meters_per_second");
    }
}

void ReadTrajectorySettings(const Json& document, ScenarioDefinition& definition)
{
    const auto setting = document.find("record_trajectory");
    if (setting == document.end())
        return;
    if (!setting->is_object())
        throw std::invalid_argument("record_trajectory는 설정 객체여야 합니다.");

    const std::uint64_t sampleEveryTicks = setting->contains("sample_every_ticks")
        ? ReadUnsigned((*setting)["sample_every_ticks"], "sample_every_ticks") : 1;
    if (sampleEveryTicks == 0)
        throw std::invalid_argument("sample_every_ticks는 1 이상이어야 합니다.");

    definition.trajectorySampleEveryTicks = sampleEveryTicks;
}

void ReadQuadrotorModelSettings(const Json& document, ScenarioDefinition& definition)
{
    const auto settings = document.find("quadrotor_model");
    if (settings == document.end())
        return;
    if (!settings->is_object())
        throw std::invalid_argument("quadrotor_model은 JSON 객체여야 합니다.");

    QuadrotorParameters& model = definition.quadrotorParameters;
    if (settings->contains("mass_kg"))
        model.massKilograms = ReadFiniteNumber((*settings)["mass_kg"], "mass_kg");
    if (settings->contains("principal_inertia_kg_m2"))
    {
        model.principalInertiaKilogramMetersSquared = ReadVector(
            (*settings)["principal_inertia_kg_m2"], "principal_inertia_kg_m2");
    }
    if (settings->contains("rotor_positions_meters"))
    {
        const Json& positions = (*settings)["rotor_positions_meters"];
        if (!positions.is_array() || positions.size() != model.rotors.size())
        {
            throw std::invalid_argument(
                "rotor_positions_meters에는 로터별 위치 4개가 필요합니다.");
        }
        for (std::size_t rotor = 0; rotor < model.rotors.size(); ++rotor)
        {
            model.rotors[rotor].positionMeters = ReadVector(
                positions[rotor], "rotor_positions_meters");
        }
    }
    if (settings->contains("thrust_coefficient_newtons_per_speed_squared"))
    {
        model.thrustCoefficientNewtonsPerSpeedSquared = ReadFiniteNumber(
            (*settings)["thrust_coefficient_newtons_per_speed_squared"],
            "thrust_coefficient_newtons_per_speed_squared");
    }
    if (settings->contains("moment_to_thrust_ratio"))
    {
        model.momentToThrustRatio = ReadFiniteNumber(
            (*settings)["moment_to_thrust_ratio"], "moment_to_thrust_ratio");
    }
    if (settings->contains("motor_time_constant_up_seconds"))
    {
        model.motorTimeConstantUpSeconds = ReadFiniteNumber(
            (*settings)["motor_time_constant_up_seconds"],
            "motor_time_constant_up_seconds");
    }
    if (settings->contains("motor_time_constant_down_seconds"))
    {
        model.motorTimeConstantDownSeconds = ReadFiniteNumber(
            (*settings)["motor_time_constant_down_seconds"],
            "motor_time_constant_down_seconds");
    }
    if (settings->contains("maximum_motor_speed_radians_per_second"))
    {
        model.maximumMotorSpeedRadiansPerSecond = ReadFiniteNumber(
            (*settings)["maximum_motor_speed_radians_per_second"],
            "maximum_motor_speed_radians_per_second");
    }

    definition.quadrotorParametersCustomized = true;
}

void ReadControllerSettings(const Json& document, ScenarioDefinition& definition)
{
    const auto settings = document.find("controller_gains");
    if (settings == document.end())
        return;
    if (!settings->is_object())
        throw std::invalid_argument("controller_gains는 JSON 객체여야 합니다.");

    QuadrotorControllerParameters& controller = definition.controllerParameters;
    if (settings->contains("position"))
        controller.positionGain = ReadFiniteNumber((*settings)["position"], "position");
    if (settings->contains("velocity"))
        controller.velocityGain = ReadFiniteNumber((*settings)["velocity"], "velocity");
    if (settings->contains("attitude"))
        controller.attitudeGain = ReadFiniteNumber((*settings)["attitude"], "attitude");
    if (settings->contains("angular_rate"))
        controller.angularRateGain = ReadFiniteNumber((*settings)["angular_rate"], "angular_rate");

    definition.controllerParametersCustomized = true;
}

void ReadCollisionAvoidanceSettings(const Json& document, ScenarioDefinition& definition)
{
    const auto settingsValue = document.find("collision_avoidance");
    if (settingsValue == document.end())
        return;
    if (!settingsValue->is_object())
        throw std::invalid_argument("collision_avoidance는 JSON 객체여야 합니다.");

    CollisionAvoidanceSettings settings{
        ReadFiniteNumber(Required(*settingsValue, "safety_distance_meters"),
            "safety_distance_meters"),
        ReadFiniteNumber(Required(*settingsValue, "prediction_horizon_seconds"),
            "prediction_horizon_seconds"),
        ReadFiniteNumber(Required(*settingsValue, "lateral_offset_meters"),
            "lateral_offset_meters")};
    definition.collisionAvoidance = settings;
}

void ReadEnergyModelSettings(const Json& document, ScenarioDefinition& definition)
{
    const auto settingsValue = document.find("energy_model");
    if (settingsValue == document.end())
        return;
    if (!settingsValue->is_object())
        throw std::invalid_argument("energy_model은 JSON 객체여야 합니다.");

    EnergyModelSettings settings{
        ReadFiniteNumber(Required(*settingsValue, "capacity_watt_hours"),
            "capacity_watt_hours"),
        ReadFiniteNumber(Required(*settingsValue, "hover_power_watts"),
            "hover_power_watts"),
        ReadFiniteNumber(Required(*settingsValue, "flight_power_watts"),
            "flight_power_watts"),
        ReadFiniteNumber(Required(*settingsValue, "reserve_percent"),
            "reserve_percent")};
    definition.energyModel = settings;
}

void ReadExpectedLimitsAndCamera(const Json& expected, ScenarioDefinition& definition)
{
    if (expected.contains("tolerance_meters"))
        definition.toleranceMeters = ReadFiniteNumber(expected["tolerance_meters"], "tolerance_meters");
    if (expected.contains("orientation_tolerance_radians"))
    {
        definition.orientationToleranceRadians = ReadFiniteNumber(
            expected["orientation_tolerance_radians"], "orientation_tolerance_radians");
    }
    if (expected.contains("minimum_drone_separation_meters"))
    {
        definition.minimumDroneSeparationMeters = ReadFiniteNumber(
            expected["minimum_drone_separation_meters"],
            "minimum_drone_separation_meters");
    }
    if (expected.contains("camera_status"))
        definition.expectedCameraStatus = ReadCameraStatus(expected["camera_status"]);
    if (expected.contains("camera_frame_count"))
    {
        definition.expectedFrameCount = ReadUnsigned(
            expected["camera_frame_count"], "camera_frame_count");
    }
}

void ReadExpectedResults(const Json& document, ScenarioDefinition& definition)
{
    const auto expectedEntry = document.find("expected");
    if (expectedEntry == document.end())
        return;

    const Json& expected = *expectedEntry;
    if (!expected.is_object())
        throw std::invalid_argument("expected는 JSON 객체여야 합니다.");

    if (const auto drones = expected.find("drones"); drones != expected.end())
        definition.expectedDrones = ReadExpectedDrones(*drones);
    else if (expected.contains("mode") || expected.contains("position_meters"))
        throw std::invalid_argument(
            "expected의 드론 상태는 drones 배열 안에 적어야 합니다.");
    ReadExpectedLimitsAndCamera(expected, definition);
}

}

/**
 * @brief JSON을 실행 설정 자료형으로 변환한다.
 * @details 여기서는 필수 항목·자료형과 안전한 숫자 변환을 확인한다.
 * 드론 이름 참조나 물리 방식처럼 여러 항목을 함께 봐야 하는 규칙은
 * 이 함수가 반환된 다음 ScenarioRunner에서 별도로 검사한다.
 */
ScenarioDefinition ParseScenarioDefinition(std::string_view jsonText)
{
    const Json document = ParseScenarioDocument(jsonText);

    const std::uint64_t version = ReadUnsigned(Required(document, "version"), "version");
    if (version != 1)
        throw std::invalid_argument("지원하는 시나리오 형식은 version 1입니다.");

    ScenarioDefinition definition;
    definition.tickCount = ReadUnsigned(Required(document, "ticks"), "ticks");
    if (definition.tickCount == 0 || definition.tickCount > maximumScenarioTicks)
        throw std::invalid_argument("ticks는 1부터 5,184,000 사이여야 합니다.");

    definition.drones = ReadDrones(document);
    definition.usesExistingPrimaryDrone = definition.drones.empty();
    if (definition.usesExistingPrimaryDrone)
    {
        // 드론 목록을 생략한 예전 단일 드론 형식은 기본 드론 한 대로 실행한다.
        ScenarioDrone primaryDrone;
        primaryDrone.name = "drone";
        definition.drones.push_back(std::move(primaryDrone));
    }
    if (const auto camera = document.find("camera_model"); camera != document.end())
        definition.defaultCameraModel = ReadCameraModel(*camera);
    definition.commands = ReadCommands(
        Required(document, "commands"), definition.drones.front().name);
    definition.communicationOutages = ReadDroneCommunicationOutages(document);

    // 독립 설정부터 읽고, 맨 마지막에 기대 결과를 읽는다. 설정 간 관계 검사는
    // 파서가 끝난 뒤 별도 검증기가 맡는다.
    ReadTrajectorySettings(document, definition);
    definition.mission = ReadMission(document);

    ReadEnvironmentSettings(document, definition);
    ReadQuadrotorModelSettings(document, definition);
    ReadControllerSettings(document, definition);
    ReadCollisionAvoidanceSettings(document, definition);
    ReadEnergyModelSettings(document, definition);
    ReadExpectedResults(document, definition);
    return definition;
}
}

