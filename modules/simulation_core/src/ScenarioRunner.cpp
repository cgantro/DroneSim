#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"

#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace aerosimlab::simulation
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

struct ScheduledCommand
{
    std::uint64_t tick;
    std::uint64_t sendTick;
    std::uint64_t delayTicks;
    std::uint64_t sequence;
    DroneCommand command;
    bool dropped;
    bool duplicated;
};

struct ScheduledCameraEvent
{
    std::uint64_t tick;
    std::string action;
};

std::vector<ScheduledCommand> ReadCommands(const Json& commands, std::uint64_t tickCount)
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
        const bool dropped = item.contains("drop") && item["drop"].get<bool>();
        const bool duplicated = item.contains("duplicate") && item["duplicate"].get<bool>();
        result.push_back({tick + delay, tick, delay, static_cast<std::uint64_t>(result.size()) + 1,
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
        return SimulationDynamics::Kinematic;
    if (!found->is_string())
        throw std::invalid_argument("dynamics는 kinematic 또는 rigid_body 문자열이어야 합니다.");

    const std::string name = found->get<std::string>();
    if (name == "kinematic") return SimulationDynamics::Kinematic;
    if (name == "rigid_body") return SimulationDynamics::RigidBody;
    throw std::invalid_argument("알 수 없는 dynamics 설정입니다: " + name);
}
}

ScenarioResult RunScenario(std::string_view jsonText)
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
    {
        throw std::invalid_argument("시나리오의 첫 항목은 JSON 객체여야 합니다.");
    }

    const std::uint64_t version = ReadUnsigned(Required(document, "version"), "version");
    if (version != 1)
    {
        throw std::invalid_argument("지원하는 시나리오 형식은 version 1입니다.");
    }

    const std::uint64_t tickCount = ReadUnsigned(Required(document, "ticks"), "ticks");
    if (tickCount == 0 || tickCount > maximumScenarioTicks)
    {
        throw std::invalid_argument("ticks는 1부터 5,184,000 사이여야 합니다.");
    }

    const std::vector<ScheduledCommand> commands = ReadCommands(
        Required(document, "commands"), tickCount);
    const SimulationDynamics dynamics = ReadDynamics(document);
    const std::optional<glm::dvec3> windVelocity = document.contains("wind_velocity_meters_per_second")
        ? std::optional<glm::dvec3>{ReadVector(
            document["wind_velocity_meters_per_second"], "wind_velocity_meters_per_second")}
        : std::nullopt;
    if (windVelocity && dynamics != SimulationDynamics::RigidBody)
        throw std::invalid_argument("바람은 rigid_body 시나리오에서만 설정할 수 있습니다.");
    const std::vector<ScheduledCameraEvent> cameraEvents = ReadCameraEvents(document, tickCount);
    const Json& expected = Required(document, "expected");
    if (!expected.is_object())
    {
        throw std::invalid_argument("expected는 JSON 객체여야 합니다.");
    }
    const FlightMode expectedMode = ReadFlightMode(Required(expected, "mode"));
    const glm::dvec3 expectedPosition = ReadVector(
        Required(expected, "position_meters"), "position_meters");
    const double toleranceMeters = expected.contains("tolerance_meters")
        ? ReadFiniteNumber(expected["tolerance_meters"], "tolerance_meters")
        : 0.01;
    if (toleranceMeters < 0.0)
    {
        throw std::invalid_argument("tolerance_meters는 0 이상이어야 합니다.");
    }

    const std::optional<CameraStatus> expectedCameraStatus = expected.contains("camera_status")
        ? std::optional<CameraStatus>{ReadCameraStatus(expected["camera_status"])}
        : std::nullopt;
    const std::optional<std::uint64_t> expectedFrameCount = expected.contains("camera_frame_count")
        ? std::optional<std::uint64_t>{ReadUnsigned(expected["camera_frame_count"], "camera_frame_count")}
        : std::nullopt;

    SimulationCore simulation{dynamics};
    if (windVelocity)
        simulation.SetWindVelocityMetersPerSecond(*windVelocity);
    ScenarioResult result;
    std::size_t nextCommand = 0;
    std::size_t nextCameraEvent = 0;
    std::uint64_t lastDeliveredSequence = 0;
    for (std::uint64_t tick = 0; tick < tickCount; ++tick)
    {
        while (nextCommand < commands.size() && commands[nextCommand].tick == tick)
        {
            const ScheduledCommand& scheduled = commands[nextCommand];
            if (scheduled.dropped)
            {
                result.eventLog.push_back("tick " + std::to_string(tick) + ": 통신 장애로 명령을 버렸습니다.");
                ++nextCommand;
                continue;
            }

            if (scheduled.sequence <= lastDeliveredSequence)
            {
                result.issues.push_back("tick " + std::to_string(tick) + "에서 역순 명령을 거부했습니다.");
                ++nextCommand;
                continue;
            }
            lastDeliveredSequence = scheduled.sequence;
            if (scheduled.delayTicks > 0)
            {
                result.eventLog.push_back("tick " + std::to_string(scheduled.sendTick)
                    + "에 보낸 명령을 " + std::to_string(scheduled.delayTicks)
                    + " Tick 늦게 전달했습니다.");
            }
            const CommandResult commandResult = simulation.Apply(scheduled.command);
            if (commandResult != CommandResult::Accepted)
                result.issues.push_back("tick " + std::to_string(tick) + " 명령을 적용하지 못했습니다.");
            else
                result.eventLog.push_back("tick " + std::to_string(tick) + ": 명령을 적용했습니다.");
            if (scheduled.duplicated)
                result.eventLog.push_back("tick " + std::to_string(tick) + ": 재전송된 중복 명령을 한 번만 처리했습니다.");
            ++nextCommand;
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
    }

    result.tickCount = simulation.TickCount();
    result.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    result.finalState = simulation.State();
    result.finalCamera = simulation.Camera();
    if (result.finalState.mode != expectedMode)
    {
        result.issues.push_back(
            "예상 비행 상태는 " + std::string(ToString(expectedMode))
            + "이지만 실제 상태는 " + std::string(ToString(result.finalState.mode)) + "입니다.");
    }
    if (glm::length(result.finalState.positionMeters - expectedPosition) > toleranceMeters)
    {
        result.issues.push_back("최종 위치가 허용 오차를 벗어났습니다.");
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
