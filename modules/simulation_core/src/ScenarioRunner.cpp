#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"

#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace aerosimlab::simulation
{
namespace
{
using Json = nlohmann::json;

constexpr std::uint64_t maximumScenarioTicks = 5'184'000;

struct ExpectedDrone
{
    std::string name;
    FlightMode mode;
    glm::dvec3 positionMeters;
    std::optional<glm::dquat> orientation;
};

bool CanReceiveWaypoint(FlightMode mode) noexcept
{
    return mode == FlightMode::Hovering
        || mode == FlightMode::Flying
        || mode == FlightMode::FollowingWaypoint;
}

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

double OrientationErrorRadians(
    const glm::dquat& actual, const glm::dquat& expected) noexcept
{
    const double dot = std::abs(
        actual.w * expected.w + actual.x * expected.x
        + actual.y * expected.y + actual.z * expected.z);
    return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
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
    if (name == "grounded") return FlightMode::Grounded;
    if (name == "taking_off") return FlightMode::TakingOff;
    if (name == "hovering") return FlightMode::Hovering;
    if (name == "flying") return FlightMode::Flying;
    if (name == "following_waypoint") return FlightMode::FollowingWaypoint;
    if (name == "landing") return FlightMode::Landing;
    throw std::invalid_argument(std::string("알 수 없는 비행 상태입니다: ") + name);
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
        const Json& nameValue = Required(item, "name");
        if (!nameValue.is_string())
            throw std::invalid_argument("예상 드론 name은 문자열이어야 합니다.");
        std::string name = nameValue.get<std::string>();
        if (name.empty() || !names.insert(name).second)
            throw std::invalid_argument("예상 드론 name은 비어 있거나 중복될 수 없습니다.");
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

struct ScheduledCommand
{
    std::uint64_t tick;
    std::uint64_t sendTick;
    std::uint64_t delayTicks;
    std::uint64_t sequence;
    std::string droneName;
    DroneCommand command;
    bool dropped;
    bool duplicated;
};

struct ScenarioDrone
{
    std::string name;
    glm::dvec3 initialPositionMeters;
    double initialYawRadians;
};

struct FormationOutage
{
    std::uint64_t startTick;
    std::uint64_t endTick;
};

struct FormationLink
{
    std::string leaderName;
    std::string followerName;
    glm::dvec3 offsetMeters;
    std::vector<FormationOutage> outages;
};

struct ScheduledCameraEvent
{
    std::uint64_t tick;
    std::string action;
};

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
        const Json& nameValue = Required(item, "name");
        if (!nameValue.is_string())
            throw std::invalid_argument("드론 name은 문자열이어야 합니다.");
        std::string name = nameValue.get<std::string>();
        if (name.empty() || !names.insert(name).second)
            throw std::invalid_argument("드론 name은 비어 있거나 중복될 수 없습니다.");
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
        if (!item.is_object())
            throw std::invalid_argument("각 formation link는 JSON 객체여야 합니다.");
        const Json& leaderValue = Required(item, "leader");
        const Json& followerValue = Required(item, "follower");
        if (!leaderValue.is_string() || !followerValue.is_string())
            throw std::invalid_argument("formation link의 leader와 follower는 이름 문자열이어야 합니다.");

        std::string leader = leaderValue.get<std::string>();
        std::string follower = followerValue.get<std::string>();
        if (!droneNames.contains(leader) || !droneNames.contains(follower)
            || leader == follower || !followers.insert(follower).second)
        {
            throw std::invalid_argument(
                "formation link는 서로 다른 드론을 지정하고, follower마다 하나만 설정해야 합니다.");
        }

        FormationLink link{
            std::move(leader),
            std::move(follower),
            ReadVector(Required(item, "offset_meters"), "offset_meters"),
            {}};
        const auto outages = item.find("outages");
        if (outages != item.end())
        {
            if (!outages->is_array())
                throw std::invalid_argument("formation link의 outages는 배열이어야 합니다.");
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
                link.outages.push_back({startTick, endTick});
                previousEndTick = endTick;
                hasPreviousOutage = true;
            }
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
        if ((item.contains("drop") && !item["drop"].is_boolean())
            || (item.contains("duplicate") && !item["duplicate"].is_boolean()))
            throw std::invalid_argument("drop과 duplicate는 true 또는 false여야 합니다.");
        std::string droneName = defaultDroneName;
        if (item.contains("drone"))
        {
            if (!item["drone"].is_string())
                throw std::invalid_argument("명령 drone은 드론 name 문자열이어야 합니다.");
            droneName = item["drone"].get<std::string>();
        }
        if (!droneNames.contains(droneName))
            throw std::invalid_argument("명령이 가리키는 드론을 찾을 수 없습니다: " + droneName);
        const bool dropped = item.contains("drop") && item["drop"].get<bool>();
        const bool duplicated = item.contains("duplicate") && item["duplicate"].get<bool>();
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
        if (action != "enable" && action != "disable" && action != "fault" && action != "recover")
            throw std::invalid_argument("알 수 없는 카메라 이벤트입니다: " + action);
        events.push_back({tick, action});
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

struct ScenarioDefinition
{
    std::uint64_t tickCount = 0;
    std::vector<ScenarioDrone> drones;
    std::string primaryDroneName;
    std::vector<ScheduledCommand> commands;
    std::vector<FormationLink> formationLinks;
    std::optional<ScenarioResult::Mission> mission;
    SimulationDynamics dynamics = defaultSimulationDynamics;
    std::optional<glm::dvec3> windVelocity;
    std::vector<ScheduledCameraEvent> cameraEvents;
    std::vector<ExpectedDrone> expectedDrones;
    bool hasExpectedDrones = false;
    double toleranceMeters = 0.01;
    double orientationToleranceRadians = 0.01;
    std::optional<double> minimumDroneSeparationMeters;
    std::optional<CameraStatus> expectedCameraStatus;
    std::optional<std::uint64_t> expectedFrameCount;
};

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

ScenarioResult RunScenario(std::string_view jsonText)
{
    ScenarioDefinition definition = ReadScenarioDefinition(jsonText);
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
