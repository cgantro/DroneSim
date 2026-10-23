#include "ScenarioDefinition.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace aerosimlab::simulation::detail
{
namespace
{
using Json = nlohmann::json;

constexpr std::uint64_t maximumScenarioTicks = 5'184'000;

const Json& Required(const Json& object, const char* key)
{
    const auto entry = object.find(key);
    if (entry == object.end())
    {
        throw std::invalid_argument(std::string("필수 항목이 없습니다: ") + key);
    }
    return *entry;
}

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
        drones.push_back({std::move(name), ReadVector(
            Required(item, "initial_position_meters"), "initial_position_meters"),
            initialYawRadians});
    }
    return drones;
}

std::vector<FormationOutage> ReadFormationOutages(
    const Json& link, std::uint64_t tickCount)
{
    const auto outages = link.find("outages");
    if (outages == link.end())
        return {};
    if (!outages->is_array())
        throw std::invalid_argument("formation link의 outages는 배열이어야 합니다.");

    std::vector<FormationOutage> result;
    std::uint64_t previousEndTick = 0;
    bool hasPreviousOutage = false;
    for (const Json& outage : *outages)
    {
        if (!outage.is_object())
            throw std::invalid_argument("각 통신 장애 구간은 JSON 객체여야 합니다.");
        const std::uint64_t startTick = ReadUnsigned(
            Required(outage, "start_tick"), "start_tick");
        const std::uint64_t endTick = ReadUnsigned(
            Required(outage, "end_tick"), "end_tick");
        if (startTick >= endTick || endTick > tickCount
            || (hasPreviousOutage && startTick < previousEndTick))
        {
            throw std::invalid_argument(
                "통신 장애 구간은 겹치지 않는 오름차순 범위이며 시나리오 안에 있어야 합니다.");
        }
        result.push_back({startTick, endTick});
        previousEndTick = endTick;
        hasPreviousOutage = true;
    }
    return result;
}

FormationLink ReadFormationLink(
    const Json& item,
    std::uint64_t tickCount,
    const std::unordered_set<std::string>& droneNames)
{
    if (!item.is_object())
        throw std::invalid_argument("각 formation link는 JSON 객체여야 합니다.");
    const Json& leaderValue = Required(item, "leader");
    const Json& followerValue = Required(item, "follower");
    if (!leaderValue.is_string() || !followerValue.is_string())
        throw std::invalid_argument("formation link의 leader와 follower는 이름 문자열이어야 합니다.");

    std::string leader = leaderValue.get<std::string>();
    std::string follower = followerValue.get<std::string>();
    if (!droneNames.contains(leader) || !droneNames.contains(follower)
        || leader == follower)
    {
        throw std::invalid_argument(
            "formation link는 서로 다른 드론을 지정하고, follower마다 하나만 설정해야 합니다.");
    }

    return {
        std::move(leader),
        std::move(follower),
        ReadVector(Required(item, "offset_meters"), "offset_meters"),
        ReadFormationOutages(item, tickCount)};
}

std::vector<FormationLink> ReadFormationLinks(
    const Json& document,
    std::uint64_t tickCount,
    const std::unordered_set<std::string>& droneNames)
{
    const auto found = document.find("formation_links");
    if (found == document.end())
        return {};
    if (!found->is_array() || found->empty())
        throw std::invalid_argument("formation_links는 한 개 이상의 링크를 적은 배열이어야 합니다.");

    std::vector<FormationLink> links;
    std::unordered_set<std::string> followers;
    links.reserve(found->size());
    for (const Json& item : *found)
    {
        FormationLink link = ReadFormationLink(item, tickCount, droneNames);
        if (!followers.insert(link.followerName).second)
        {
            throw std::invalid_argument(
                "formation link는 서로 다른 드론을 지정하고, follower마다 하나만 설정해야 합니다.");
        }
        links.push_back(std::move(link));
    }
    return links;
}

std::optional<ScenarioResult::Mission> ReadMission(
    const Json& document,
    std::uint64_t tickCount,
    const std::unordered_set<std::string>& droneNames)
{
    const auto found = document.find("mission");
    if (found == document.end())
        return std::nullopt;
    if (!found->is_object())
        throw std::invalid_argument("mission은 JSON 객체여야 합니다.");

    const Json& nameValue = Required(*found, "name");
    const Json& tasksValue = Required(*found, "tasks");
    if (!nameValue.is_string() || nameValue.get<std::string>().empty())
        throw std::invalid_argument("mission.name은 비어 있지 않은 문자열이어야 합니다.");
    if (!tasksValue.is_array() || tasksValue.size() < 2)
        throw std::invalid_argument("공동 임무에는 두 대 이상을 배정해야 합니다.");

    ScenarioResult::Mission mission;
    mission.name = nameValue.get<std::string>();
    std::unordered_set<std::string> taskNames;
    std::unordered_set<std::string> assignedDrones;
    mission.tasks.reserve(tasksValue.size());
    for (const Json& taskValue : tasksValue)
    {
        if (!taskValue.is_object())
            throw std::invalid_argument("각 mission task는 JSON 객체여야 합니다.");

        const Json& taskNameValue = Required(taskValue, "name");
        const Json& roleValue = Required(taskValue, "role");
        const Json& droneValue = Required(taskValue, "drone");
        if (!taskNameValue.is_string() || taskNameValue.get<std::string>().empty()
            || !roleValue.is_string() || roleValue.get<std::string>().empty()
            || !droneValue.is_string())
        {
            throw std::invalid_argument(
                "임무 name과 role은 비어 있지 않은 문자열이어야 하고 drone을 지정해야 합니다.");
        }

        ScenarioResult::MissionTask task;
        task.name = taskNameValue.get<std::string>();
        task.role = roleValue.get<std::string>();
        task.droneName = droneValue.get<std::string>();
        if (!taskNames.insert(task.name).second
            || !droneNames.contains(task.droneName)
            || !assignedDrones.insert(task.droneName).second)
        {
            throw std::invalid_argument("임무 이름은 고유해야 하며, 등록된 드론을 한 번만 배정할 수 있습니다.");
        }

        task.targetPositionMeters = ReadVector(
            Required(taskValue, "target_position_meters"), "target_position_meters");
        task.startTick = ReadUnsigned(Required(taskValue, "start_tick"), "start_tick");
        task.deadlineTick = ReadUnsigned(
            Required(taskValue, "deadline_tick"), "deadline_tick");
        task.toleranceMeters = ReadFiniteNumber(
            Required(taskValue, "tolerance_meters"), "tolerance_meters");
        if (task.targetPositionMeters.z < 0.0 || task.toleranceMeters < 0.0
            || task.startTick > task.deadlineTick || task.deadlineTick >= tickCount)
        {
            throw std::invalid_argument(
                "임무 목표 고도와 허용 오차는 0 이상이고, 시작·기한 Tick은 시나리오 범위 안이어야 합니다.");
        }
        mission.tasks.push_back(std::move(task));
    }
    return mission;
}

bool ReadOptionalBoolean(const Json& object, const char* name)
{
    const auto value = object.find(name);
    if (value == object.end())
        return false;
    if (!value->is_boolean())
        throw std::invalid_argument("drop과 duplicate는 true 또는 false여야 합니다.");
    return value->get<bool>();
}

std::string ReadCommandDroneName(
    const Json& command,
    const std::string& defaultDroneName,
    const std::unordered_set<std::string>& droneNames)
{
    std::string droneName = defaultDroneName;
    const auto drone = command.find("drone");
    if (drone != command.end())
    {
        if (!drone->is_string())
            throw std::invalid_argument("명령 drone은 드론 name 문자열이어야 합니다.");
        droneName = drone->get<std::string>();
    }
    if (!droneNames.contains(droneName))
        throw std::invalid_argument("명령이 가리키는 드론을 찾을 수 없습니다: " + droneName);
    return droneName;
}

std::vector<ScheduledCommand> ReadCommands(
    const Json& commands,
    std::uint64_t tickCount,
    const std::string& defaultDroneName,
    const std::unordered_set<std::string>& droneNames)
{
    if (!commands.is_array())
    {
        throw std::invalid_argument("commands는 배열이어야 합니다.");
    }

    std::vector<ScheduledCommand> result;
    result.reserve(commands.size());
    std::uint64_t previousTick = 0;
    bool hasPreviousTick = false;
    for (const Json& item : commands)
    {
        const std::uint64_t tick = ReadUnsigned(Required(item, "tick"), "tick");
        if (tick >= tickCount)
        {
            throw std::invalid_argument("명령 tick은 시나리오 실행 범위 안에 있어야 합니다.");
        }
        if (hasPreviousTick && tick < previousTick)
        {
            throw std::invalid_argument("명령은 tick 순서대로 작성해야 합니다.");
        }
        const std::uint64_t delay = item.contains("delay_ticks")
            ? ReadUnsigned(item["delay_ticks"], "delay_ticks")
            : 0;
        if (delay >= tickCount - tick)
            throw std::invalid_argument("통신 지연 뒤 명령이 시나리오 실행 범위를 벗어납니다.");
        const bool dropped = ReadOptionalBoolean(item, "drop");
        const bool duplicated = ReadOptionalBoolean(item, "duplicate");
        std::string droneName = ReadCommandDroneName(
            item, defaultDroneName, droneNames);
        result.push_back({tick + delay, tick, delay,
            static_cast<std::uint64_t>(result.size()) + 1, std::move(droneName),
            ReadCommand(item), dropped, duplicated});
        previousTick = tick;
        hasPreviousTick = true;
    }
    std::stable_sort(result.begin(), result.end(), [](const auto& left, const auto& right)
    {
        return left.tick < right.tick;
    });
    return result;
}

CameraEventAction ReadCameraEventAction(const std::string& action)
{
    if (action == "enable") return CameraEventAction::Enable;
    if (action == "disable") return CameraEventAction::Disable;
    if (action == "fault") return CameraEventAction::Fault;
    if (action == "recover") return CameraEventAction::Recover;
    throw std::invalid_argument("알 수 없는 카메라 이벤트입니다: " + action);
}

std::vector<ScheduledCameraEvent> ReadCameraEvents(const Json& document, std::uint64_t tickCount)
{
    const auto found = document.find("camera_events");
    if (found == document.end())
        return {};
    if (!found->is_array())
        throw std::invalid_argument("camera_events는 배열이어야 합니다.");

    std::vector<ScheduledCameraEvent> events;
    events.reserve(found->size());
    std::uint64_t previousTick = 0;
    bool hasPreviousTick = false;
    for (const Json& event : *found)
    {
        const std::uint64_t tick = ReadUnsigned(Required(event, "tick"), "tick");
        const Json& actionValue = Required(event, "action");
        if (tick >= tickCount || !actionValue.is_string())
            throw std::invalid_argument("카메라 이벤트의 tick 또는 action이 잘못되었습니다.");
        if (hasPreviousTick && tick < previousTick)
            throw std::invalid_argument("카메라 이벤트는 tick 순서대로 작성해야 합니다.");
        const std::string action = actionValue.get<std::string>();
        events.push_back({tick, ReadCameraEventAction(action)});
        previousTick = tick;
        hasPreviousTick = true;
    }
    return events;
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

SimulationDynamics ReadDynamics(const Json& document)
{
    const auto found = document.find("dynamics");
    if (found == document.end())
        return defaultSimulationDynamics;
    if (!found->is_string())
        throw std::invalid_argument("dynamics는 kinematic 또는 rigid_body 문자열이어야 합니다.");

    const std::string name = found->get<std::string>();
    if (name == "kinematic") return SimulationDynamics::Kinematic;
    if (name == "rigid_body") return SimulationDynamics::RigidBody;
    throw std::invalid_argument("알 수 없는 dynamics 설정입니다: " + name);
}

}

/** @brief JSON 문법과 설정을 확인한 뒤 실행에 필요한 자료만 묶어 돌려준다. */
ScenarioDefinition ReadScenarioDefinition(std::string_view jsonText)
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
    if (!document.is_object())
        throw std::invalid_argument("시나리오의 첫 항목은 JSON 객체여야 합니다.");

    const std::uint64_t version = ReadUnsigned(Required(document, "version"), "version");
    if (version != 1)
        throw std::invalid_argument("지원하는 시나리오 형식은 version 1입니다.");

    ScenarioDefinition definition;
    definition.tickCount = ReadUnsigned(Required(document, "ticks"), "ticks");
    if (definition.tickCount == 0 || definition.tickCount > maximumScenarioTicks)
        throw std::invalid_argument("ticks는 1부터 5,184,000 사이여야 합니다.");

    definition.drones = ReadDrones(document);
    definition.primaryDroneName = definition.drones.empty()
        ? "drone" : definition.drones.front().name;
    std::unordered_set<std::string> droneNames;
    if (definition.drones.empty())
        droneNames.insert(definition.primaryDroneName);
    else
        for (const ScenarioDrone& drone : definition.drones)
            droneNames.insert(drone.name);

    definition.commands = ReadCommands(
        Required(document, "commands"), definition.tickCount,
        definition.primaryDroneName, droneNames);
    definition.formationLinks = ReadFormationLinks(
        document, definition.tickCount, droneNames);
    if (!definition.formationLinks.empty() && definition.drones.size() < 2)
        throw std::invalid_argument("formation_links를 쓰려면 drones에 두 대 이상을 적어야 합니다.");

    definition.mission = ReadMission(document, definition.tickCount, droneNames);
    if (definition.mission)
    {
        for (const FormationLink& link : definition.formationLinks)
        {
            const bool followerHasMission = std::any_of(
                definition.mission->tasks.begin(), definition.mission->tasks.end(),
                [&link](const ScenarioResult::MissionTask& task)
                {
                    return task.droneName == link.followerName;
                });
            if (followerHasMission)
                throw std::invalid_argument(
                    "같은 드론에 formation link와 mission 목표를 함께 지정할 수 없습니다.");
        }
    }

    definition.dynamics = ReadDynamics(document);
    if (document.contains("wind_velocity_meters_per_second"))
    {
        definition.windVelocity = ReadVector(
            document["wind_velocity_meters_per_second"],
            "wind_velocity_meters_per_second");
    }
    if (definition.windVelocity
        && definition.dynamics != SimulationDynamics::RigidBody)
        throw std::invalid_argument("바람은 rigid_body 시나리오에서만 설정할 수 있습니다.");

    definition.cameraEvents = ReadCameraEvents(document, definition.tickCount);
    const Json& expected = Required(document, "expected");
    if (!expected.is_object())
        throw std::invalid_argument("expected는 JSON 객체여야 합니다.");

    definition.hasExpectedDrones = expected.contains("drones");
    if (definition.hasExpectedDrones
        && (expected.contains("mode") || expected.contains("position_meters")
            || expected.contains("orientation_quaternion_wxyz")))
        throw std::invalid_argument(
            "expected에는 drones 목록 또는 단일 드론 상태 중 하나만 적어야 합니다.");
    if (!definition.hasExpectedDrones && !definition.drones.empty())
        throw std::invalid_argument("여러 드론을 지정한 시나리오는 expected.drones가 필요합니다.");

    if (definition.hasExpectedDrones)
    {
        definition.expectedDrones = ReadExpectedDrones(expected["drones"]);
    }
    else
    {
        std::optional<glm::dquat> orientation;
        if (expected.contains("orientation_quaternion_wxyz"))
            orientation = ReadOrientationQuaternion(expected["orientation_quaternion_wxyz"]);
        definition.expectedDrones.push_back({
            definition.primaryDroneName,
            ReadFlightMode(Required(expected, "mode")),
            ReadVector(Required(expected, "position_meters"), "position_meters"),
            orientation});
    }

    if (expected.contains("tolerance_meters"))
        definition.toleranceMeters = ReadFiniteNumber(expected["tolerance_meters"], "tolerance_meters");
    if (definition.toleranceMeters < 0.0)
        throw std::invalid_argument("tolerance_meters는 0 이상이어야 합니다.");

    if (expected.contains("orientation_tolerance_radians"))
    {
        definition.orientationToleranceRadians = ReadFiniteNumber(
            expected["orientation_tolerance_radians"], "orientation_tolerance_radians");
    }
    if (definition.orientationToleranceRadians < 0.0
        || definition.orientationToleranceRadians > std::numbers::pi)
        throw std::invalid_argument("orientation_tolerance_radians는 0부터 π 사이여야 합니다.");

    if (expected.contains("minimum_drone_separation_meters"))
    {
        definition.minimumDroneSeparationMeters = ReadFiniteNumber(
            expected["minimum_drone_separation_meters"],
            "minimum_drone_separation_meters");
    }
    if (definition.minimumDroneSeparationMeters
        && (*definition.minimumDroneSeparationMeters < 0.0 || definition.drones.size() < 2))
        throw std::invalid_argument(
            "minimum_drone_separation_meters는 여러 드론이 있는 시나리오에서 0 이상이어야 합니다.");

    if (expected.contains("camera_status"))
        definition.expectedCameraStatus = ReadCameraStatus(expected["camera_status"]);
    if (expected.contains("camera_frame_count"))
    {
        definition.expectedFrameCount = ReadUnsigned(
            expected["camera_frame_count"], "camera_frame_count");
    }
    return definition;
}
}

