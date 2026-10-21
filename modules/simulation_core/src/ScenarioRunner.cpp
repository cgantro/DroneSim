#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"

#include <glm/geometric.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
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
    DroneCommand command;
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
        result.push_back({tick, ReadCommand(item)});
        previousTick = tick;
        hasPreviousTick = true;
    }
    return result;
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

    SimulationCore simulation;
    ScenarioResult result;
    std::size_t nextCommand = 0;
    for (std::uint64_t tick = 0; tick < tickCount; ++tick)
    {
        while (nextCommand < commands.size() && commands[nextCommand].tick == tick)
        {
            const CommandResult commandResult = simulation.Apply(commands[nextCommand].command);
            if (commandResult != CommandResult::Accepted)
            {
                result.issues.push_back(
                    "tick " + std::to_string(tick) + " 명령을 적용하지 못했습니다.");
            }
            ++nextCommand;
        }
        simulation.AdvanceOneTick();
    }

    result.tickCount = simulation.TickCount();
    result.simulationTimeSeconds = simulation.SimulationTimeSeconds();
    result.finalState = simulation.State();
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
