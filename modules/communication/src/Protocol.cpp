#include "communication/Protocol.h"
#include "simulation_core/ScenarioRunner.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aerosimlab::communication
{
namespace
{
using Json = nlohmann::json;

const Json& Required(const Json& object, const char* key)
{
    const auto found = object.find(key);
    if (found == object.end())
        throw std::invalid_argument(std::string("필수 메시지 항목이 없습니다: ") + key);
    return *found;
}

std::uint64_t ReadUnsigned(const Json& value, const char* key)
{
    if (!value.is_number_unsigned() && !value.is_number_integer())
        throw std::invalid_argument(std::string("0 이상의 정수가 필요합니다: ") + key);
    if (value.is_number_integer() && value.get<std::int64_t>() < 0)
        throw std::invalid_argument(std::string("음수는 사용할 수 없습니다: ") + key);
    return value.get<std::uint64_t>();
}

double ReadFinite(const Json& value, const char* key)
{
    if (!value.is_number())
        throw std::invalid_argument(std::string("숫자가 필요합니다: ") + key);
    const double result = value.get<double>();
    if (!std::isfinite(result))
        throw std::invalid_argument(std::string("유한한 숫자가 필요합니다: ") + key);
    return result;
}

glm::dvec3 ReadVector(const Json& value)
{
    if (!value.is_array() || value.size() != 3)
        throw std::invalid_argument("좌표 또는 속도는 숫자 3개로 된 배열이어야 합니다.");
    return {ReadFinite(value[0], "x"), ReadFinite(value[1], "y"), ReadFinite(value[2], "z")};
}

Json Wrap(std::string_view type, Json payload)
{
    Json envelope{{"version", 1}, {"type", type}, {"length", 0}, {"payload", std::move(payload)}};
    envelope["length"] = envelope["payload"].dump().size();
    return envelope;
}

Json Unwrap(std::string_view datagram, std::string_view expectedType)
{
    if (datagram.empty() || datagram.size() > maximumDatagramBytes)
        throw std::invalid_argument("UDP 메시지 크기가 허용 범위를 벗어났습니다.");

    Json envelope;
    try
    {
        envelope = Json::parse(datagram);
    }
    catch (const Json::exception& error)
    {
        throw std::invalid_argument(std::string("JSON 메시지를 읽지 못했습니다: ") + error.what());
    }
    if (!envelope.is_object() || ReadUnsigned(Required(envelope, "version"), "version") != 1)
        throw std::invalid_argument("지원하지 않는 통신 프로토콜 버전입니다.");
    if (!Required(envelope, "type").is_string()
        || Required(envelope, "type").get<std::string>() != expectedType)
        throw std::invalid_argument("예상한 메시지 종류가 아닙니다.");

    const Json& payload = Required(envelope, "payload");
    if (!payload.is_object() || ReadUnsigned(Required(envelope, "length"), "length") != payload.dump().size())
        throw std::invalid_argument("메시지 길이 정보가 맞지 않습니다.");
    return payload;
}

simulation::FlightMode ReadFlightMode(const Json& value)
{
    if (!value.is_string())
        throw std::invalid_argument("Telemetry의 비행 상태가 문자열이 아닙니다.");
    const auto mode = value.get<std::string>();
    if (mode == "grounded") return simulation::FlightMode::Grounded;
    if (mode == "taking_off") return simulation::FlightMode::TakingOff;
    if (mode == "hovering") return simulation::FlightMode::Hovering;
    if (mode == "flying") return simulation::FlightMode::Flying;
    if (mode == "following_waypoint") return simulation::FlightMode::FollowingWaypoint;
    if (mode == "landing") return simulation::FlightMode::Landing;
    throw std::invalid_argument("알 수 없는 비행 상태입니다.");
}

simulation::CameraStatus ReadCameraStatus(const Json& value)
{
    if (!value.is_string())
        throw std::invalid_argument("Telemetry의 카메라 상태가 문자열이 아닙니다.");
    const auto status = value.get<std::string>();
    if (status == "active") return simulation::CameraStatus::Active;
    if (status == "disabled") return simulation::CameraStatus::Disabled;
    if (status == "fault") return simulation::CameraStatus::Fault;
    throw std::invalid_argument("알 수 없는 카메라 상태입니다.");
}

std::uint32_t Read32(const Json& value, const char* name)
{
    const auto number = ReadUnsigned(value, name);
    if (number > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument(std::string("값이 허용 범위를 벗어났습니다: ") + name);
    return static_cast<std::uint32_t>(number);
}
}

std::string EncodeCommand(const CommandPacket& packet)
{
    const unsigned int targetCount = static_cast<unsigned int>(packet.droneCommand.has_value())
        + static_cast<unsigned int>(packet.cameraCommand.has_value())
        + static_cast<unsigned int>(packet.sessionCommand.has_value());
    if (targetCount != 1)
        throw std::invalid_argument("한 메시지에는 비행·카메라·기록 명령 중 하나만 넣을 수 있습니다.");

    Json payload{{"command_id", packet.commandId}, {"sequence", packet.sequence}};
    if (packet.droneCommand)
    {
        const auto& command = *packet.droneCommand;
        switch (command.kind)
        {
        case simulation::DroneCommand::Kind::TakeOff: payload["action"] = "takeoff"; break;
        case simulation::DroneCommand::Kind::Land: payload["action"] = "land"; break;
        case simulation::DroneCommand::Kind::Stop: payload["action"] = "stop"; break;
        case simulation::DroneCommand::Kind::SetVelocity:
            payload["action"] = "set_velocity";
            payload["vector"] = {command.vector.x, command.vector.y, command.vector.z};
            break;
        case simulation::DroneCommand::Kind::SetWaypoint:
            payload["action"] = "set_waypoint";
            payload["vector"] = {command.vector.x, command.vector.y, command.vector.z};
            break;
        case simulation::DroneCommand::Kind::SetYawRate:
            payload["action"] = "set_yaw_rate";
            payload["yaw_rate_radians_per_second"] = command.yawRateRadiansPerSecond;
            break;
        }
        payload["target"] = "drone";
    }
    else if (packet.cameraCommand)
    {
        payload["target"] = "camera";
        switch (*packet.cameraCommand)
        {
        case CameraCommand::Enable: payload["action"] = "enable"; break;
        case CameraCommand::Disable: payload["action"] = "disable"; break;
        case CameraCommand::InjectFault: payload["action"] = "fault"; break;
        case CameraCommand::Recover: payload["action"] = "recover"; break;
        }
    }
    else
    {
        payload["target"] = "session";
        switch (*packet.sessionCommand)
        {
        case SessionCommand::StartRecording: payload["action"] = "record_start"; break;
        case SessionCommand::StopRecording: payload["action"] = "record_stop"; break;
        case SessionCommand::ReplayRecording: payload["action"] = "record_replay"; break;
        }
    }
    return Wrap("command", std::move(payload)).dump();
}

CommandPacket DecodeCommand(std::string_view datagram)
{
    const Json payload = Unwrap(datagram, "command");
    CommandPacket packet;
    packet.commandId = Read32(Required(payload, "command_id"), "command_id");
    packet.sequence = Read32(Required(payload, "sequence"), "sequence");
    if (packet.commandId == 0 || packet.sequence == 0)
        throw std::invalid_argument("명령 ID와 순번은 1부터 시작해야 합니다.");
    const Json& targetValue = Required(payload, "target");
    const Json& actionValue = Required(payload, "action");
    if (!targetValue.is_string() || !actionValue.is_string())
        throw std::invalid_argument("명령 대상과 동작은 문자열이어야 합니다.");
    const std::string target = targetValue.get<std::string>();
    const std::string action = actionValue.get<std::string>();

    if (target == "camera")
    {
        if (action == "enable") packet.cameraCommand = CameraCommand::Enable;
        else if (action == "disable") packet.cameraCommand = CameraCommand::Disable;
        else if (action == "fault") packet.cameraCommand = CameraCommand::InjectFault;
        else if (action == "recover") packet.cameraCommand = CameraCommand::Recover;
        else throw std::invalid_argument("알 수 없는 카메라 명령입니다.");
        return packet;
    }
    if (target == "session")
    {
        if (action == "record_start") packet.sessionCommand = SessionCommand::StartRecording;
        else if (action == "record_stop") packet.sessionCommand = SessionCommand::StopRecording;
        else if (action == "record_replay") packet.sessionCommand = SessionCommand::ReplayRecording;
        else throw std::invalid_argument("알 수 없는 기록 제어 명령입니다.");
        return packet;
    }
    if (target != "drone")
        throw std::invalid_argument("알 수 없는 명령 대상입니다.");

    simulation::DroneCommand command;
    if (action == "takeoff") command.kind = simulation::DroneCommand::Kind::TakeOff;
    else if (action == "land") command.kind = simulation::DroneCommand::Kind::Land;
    else if (action == "stop") command.kind = simulation::DroneCommand::Kind::Stop;
    else if (action == "set_velocity")
    {
        command.kind = simulation::DroneCommand::Kind::SetVelocity;
        command.vector = ReadVector(Required(payload, "vector"));
    }
    else if (action == "set_waypoint")
    {
        command.kind = simulation::DroneCommand::Kind::SetWaypoint;
        command.vector = ReadVector(Required(payload, "vector"));
    }
    else if (action == "set_yaw_rate")
    {
        command.kind = simulation::DroneCommand::Kind::SetYawRate;
        command.yawRateRadiansPerSecond = ReadFinite(
            Required(payload, "yaw_rate_radians_per_second"), "yaw_rate_radians_per_second");
    }
    else throw std::invalid_argument("알 수 없는 드론 명령입니다.");
    packet.droneCommand = command;
    return packet;
}

std::string EncodeReply(const CommandReply& reply)
{
    const auto& state = reply.telemetry.drone;
    const auto& camera = reply.telemetry.camera;
    Json payload{
        {"command_id", reply.commandId}, {"sequence", reply.sequence},
        {"accepted", reply.accepted}, {"status", reply.status},
        {"received_at_ms", reply.receivedAtMilliseconds}, {"applied_tick", reply.appliedTick},
        {"telemetry", {
            {"tick", reply.telemetry.tick},
            {"simulation_time_seconds", reply.telemetry.simulationTimeSeconds},
            {"flight_mode", simulation::ToString(state.mode)},
            {"position_meters", {state.positionMeters.x, state.positionMeters.y, state.positionMeters.z}},
            {"velocity_meters_per_second", {
                state.velocityMetersPerSecond.x, state.velocityMetersPerSecond.y, state.velocityMetersPerSecond.z}},
            {"yaw_radians", state.yawRadians},
            {"camera_status", aerosimlab::communication::ToString(camera.status)},
            {"camera_frame_count", camera.frameCount},
            {"camera_last_frame_tick", camera.lastFrameTick}
        }}
    };
    return Wrap("reply", std::move(payload)).dump();
}

CommandReply DecodeReply(std::string_view datagram)
{
    const Json payload = Unwrap(datagram, "reply");
    CommandReply reply;
    reply.commandId = Read32(Required(payload, "command_id"), "command_id");
    reply.sequence = Read32(Required(payload, "sequence"), "sequence");
    if (!Required(payload, "accepted").is_boolean() || !Required(payload, "status").is_string())
        throw std::invalid_argument("명령 응답의 결과 정보가 잘못되었습니다.");
    reply.accepted = Required(payload, "accepted").get<bool>();
    reply.status = Required(payload, "status").get<std::string>();
    reply.receivedAtMilliseconds = ReadUnsigned(Required(payload, "received_at_ms"), "received_at_ms");
    reply.appliedTick = ReadUnsigned(Required(payload, "applied_tick"), "applied_tick");
    const Json& telemetry = Required(payload, "telemetry");
    reply.telemetry.tick = ReadUnsigned(Required(telemetry, "tick"), "tick");
    reply.telemetry.simulationTimeSeconds = ReadFinite(
        Required(telemetry, "simulation_time_seconds"), "simulation_time_seconds");
    auto& state = reply.telemetry.drone;
    state.mode = ReadFlightMode(Required(telemetry, "flight_mode"));
    state.positionMeters = ReadVector(Required(telemetry, "position_meters"));
    state.velocityMetersPerSecond = ReadVector(Required(telemetry, "velocity_meters_per_second"));
    state.yawRadians = ReadFinite(Required(telemetry, "yaw_radians"), "yaw_radians");
    auto& camera = reply.telemetry.camera;
    camera.status = ReadCameraStatus(Required(telemetry, "camera_status"));
    camera.frameCount = ReadUnsigned(Required(telemetry, "camera_frame_count"), "camera_frame_count");
    camera.lastFrameTick = ReadUnsigned(Required(telemetry, "camera_last_frame_tick"), "camera_last_frame_tick");
    return reply;
}

std::string EncodeError(std::string_view message)
{
    return Wrap("error", Json{{"message", message}}).dump();
}

std::string_view ToString(simulation::CameraStatus status) noexcept
{
    return aerosimlab::simulation::ToString(status);
}

SequenceGuard::Decision SequenceGuard::Inspect(const CommandPacket& packet) const noexcept
{
    if (!hasLast_ || (packet.commandId > lastCommandId_ && packet.sequence > lastSequence_))
        return Decision::New;
    if (packet.commandId == lastCommandId_ && packet.sequence == lastSequence_)
        return Decision::Duplicate;
    return Decision::OutOfOrder;
}

void SequenceGuard::Commit(const CommandPacket& packet) noexcept
{
    hasLast_ = true;
    lastCommandId_ = packet.commandId;
    lastSequence_ = packet.sequence;
}
}
