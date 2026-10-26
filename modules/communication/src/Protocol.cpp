#include "communication/Protocol.h"
#include "simulation_core/ScenarioRunner.h"

#include <nlohmann/json.hpp>

#include <cstddef>
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

glm::dquat ReadQuaternion(const Json& value)
{
    if (!value.is_object())
        throw std::invalid_argument("Quaternion 값은 성분별 객체여야 합니다.");
    return {
        ReadFinite(Required(value, "w"), "orientation.w"),
        ReadFinite(Required(value, "x"), "orientation.x"),
        ReadFinite(Required(value, "y"), "orientation.y"),
        ReadFinite(Required(value, "z"), "orientation.z")};
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
    const std::string name = value.get<std::string>();
    if (const auto mode = simulation::TryParseFlightMode(name))
        return *mode;
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

void ReadOptionalDroneState(const Json& telemetry, simulation::DroneState& state)
{
    if (telemetry.contains("velocity_meters_per_second"))
        state.velocityMetersPerSecond = ReadVector(Required(telemetry, "velocity_meters_per_second"));
    if (telemetry.contains("orientation_quaternion"))
        state.orientation = ReadQuaternion(Required(telemetry, "orientation_quaternion"));
    if (telemetry.contains("angular_velocity_radians_per_second"))
        state.angularVelocityRadiansPerSecond =
            ReadVector(Required(telemetry, "angular_velocity_radians_per_second"));
    if (telemetry.contains("motor_speeds_radians_per_second"))
    {
        const Json& speeds = Required(telemetry, "motor_speeds_radians_per_second");
        if (!speeds.is_array() || speeds.size() != state.motorSpeedsRadiansPerSecond.size())
            throw std::invalid_argument("로터 속도는 네 개 숫자로 된 배열이어야 합니다.");
        for (std::size_t index = 0; index < speeds.size(); ++index)
        {
            state.motorSpeedsRadiansPerSecond[index] =
                ReadFinite(speeds[index], "motor_speeds_radians_per_second");
        }
    }
    if (telemetry.contains("yaw_radians"))
        state.yawRadians = ReadFinite(Required(telemetry, "yaw_radians"), "yaw_radians");
}

std::vector<DroneTelemetry> ReadFleetTelemetry(const Json& telemetry)
{
    std::vector<DroneTelemetry> drones;
    if (!telemetry.contains("drones"))
        return drones;

    const Json& items = telemetry["drones"];
    if (!items.is_array() || items.size() > maximumFleetDroneCount)
        throw std::invalid_argument("편대 드론 목록의 형식이나 크기가 잘못되었습니다.");
    drones.reserve(items.size());
    for (const Json& item : items)
    {
        if (!item.is_object() || !Required(item, "name").is_string())
            throw std::invalid_argument("편대 드론 이름이 올바르지 않습니다.");
        drones.push_back({
            ReadUnsigned(Required(item, "id"), "drone.id"),
            Required(item, "name").get<std::string>(),
            ReadFlightMode(Required(item, "flight_mode")),
            ReadVector(Required(item, "position_meters"))});
    }
    return drones;
}

Telemetry ReadTelemetry(const Json& value)
{
    Telemetry telemetry;
    telemetry.tick = ReadUnsigned(Required(value, "tick"), "tick");
    telemetry.simulationTimeSeconds = ReadFinite(
        Required(value, "simulation_time_seconds"), "simulation_time_seconds");

    auto& drone = telemetry.drone;
    drone.mode = ReadFlightMode(Required(value, "flight_mode"));
    drone.positionMeters = ReadVector(Required(value, "position_meters"));
    ReadOptionalDroneState(value, drone);

    auto& camera = telemetry.camera;
    camera.status = ReadCameraStatus(Required(value, "camera_status"));
    camera.frameCount = ReadUnsigned(Required(value, "camera_frame_count"), "camera_frame_count");
    camera.lastFrameTick = ReadUnsigned(Required(value, "camera_last_frame_tick"), "camera_last_frame_tick");
    telemetry.drones = ReadFleetTelemetry(value);
    if (value.contains("inspection_mission_status"))
    {
        if (!value["inspection_mission_status"].is_string())
            throw std::invalid_argument("점검 임무 상태가 문자열이 아닙니다.");
        telemetry.inspectionMissionStatus = value["inspection_mission_status"].get<std::string>();
    }
    return telemetry;
}

CameraCommand ReadCameraCommand(std::string_view action)
{
    if (action == "enable") return CameraCommand::Enable;
    if (action == "disable") return CameraCommand::Disable;
    if (action == "fault") return CameraCommand::InjectFault;
    if (action == "recover") return CameraCommand::Recover;
    throw std::invalid_argument("알 수 없는 카메라 명령입니다.");
}

SessionCommand ReadSessionCommand(std::string_view action)
{
    if (action == "record_start") return SessionCommand::StartRecording;
    if (action == "record_stop") return SessionCommand::StopRecording;
    if (action == "record_replay") return SessionCommand::ReplayRecording;
    throw std::invalid_argument("알 수 없는 기록 제어 명령입니다.");
}

VideoRecordingCommand ReadVideoRecordingCommand(std::string_view action)
{
    if (action == "video_start") return VideoRecordingCommand::Start;
    if (action == "video_stop") return VideoRecordingCommand::Stop;
    throw std::invalid_argument("알 수 없는 영상 녹화 명령입니다.");
}

InspectionCommand ReadInspectionCommand(std::string_view action)
{
    if (action == "start") return InspectionCommand::Start;
    if (action == "cancel") return InspectionCommand::Cancel;
    throw std::invalid_argument("알 수 없는 점검 임무 명령입니다.");
}

simulation::DroneCommand ReadDroneCommand(const Json& payload, std::string_view action)
{
    using Kind = simulation::DroneCommand::Kind;
    simulation::DroneCommand command;

    if (action == "takeoff")
    {
        command.kind = Kind::TakeOff;
        return command;
    }
    if (action == "land")
    {
        command.kind = Kind::Land;
        return command;
    }
    if (action == "stop")
    {
        command.kind = Kind::Stop;
        return command;
    }
    if (action == "set_velocity")
    {
        command.kind = Kind::SetVelocity;
        command.vector = ReadVector(Required(payload, "vector"));
        return command;
    }
    if (action == "set_waypoint")
    {
        command.kind = Kind::SetWaypoint;
        command.vector = ReadVector(Required(payload, "vector"));
        return command;
    }
    if (action == "set_yaw_rate")
    {
        command.kind = Kind::SetYawRate;
        command.yawRateRadiansPerSecond = ReadFinite(
            Required(payload, "yaw_rate_radians_per_second"), "yaw_rate_radians_per_second");
        return command;
    }

    throw std::invalid_argument("알 수 없는 드론 명령입니다.");
}

Json EncodeCommandPayload(const simulation::DroneCommand& command)
{
    Json payload{{"target", "drone"}};
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
    return payload;
}

Json EncodeCommandPayload(CameraCommand command)
{
    const char* action = nullptr;
    switch (command)
    {
    case CameraCommand::Enable: action = "enable"; break;
    case CameraCommand::Disable: action = "disable"; break;
    case CameraCommand::InjectFault: action = "fault"; break;
    case CameraCommand::Recover: action = "recover"; break;
    }
    return {{"target", "camera"}, {"action", action}};
}

Json EncodeCommandPayload(SessionCommand command)
{
    const char* action = nullptr;
    switch (command)
    {
    case SessionCommand::StartRecording: action = "record_start"; break;
    case SessionCommand::StopRecording: action = "record_stop"; break;
    case SessionCommand::ReplayRecording: action = "record_replay"; break;
    }
    return {{"target", "session"}, {"action", action}};
}

Json EncodeCommandPayload(VideoRecordingCommand command)
{
    const char* action = command == VideoRecordingCommand::Start
        ? "video_start" : "video_stop";
    return {{"target", "video_recording"}, {"action", action}};
}

Json EncodeCommandPayload(InspectionCommand command)
{
    const char* action = command == InspectionCommand::Start ? "start" : "cancel";
    return {{"target", "inspection_mission"}, {"action", action}};
}

Json EncodeCommandPayload(FleetCommand command)
{
    const char* action = command == FleetCommand::AddDrone ? "add_drone" : "status";
    return {{"target", "fleet"}, {"action", action}};
}

Json EncodeCommandPayload(std::monostate)
{
    throw std::invalid_argument("보낼 명령을 선택해 주세요.");
}
}

std::string EncodeCommand(const CommandPacket& packet)
{
    Json payload = std::visit([](const auto& command)
    {
        return EncodeCommandPayload(command);
    }, packet.command);
    payload["command_id"] = packet.commandId;
    payload["sequence"] = packet.sequence;
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
        packet.command = ReadCameraCommand(action);
        return packet;
    }
    if (target == "session")
    {
        packet.command = ReadSessionCommand(action);
        return packet;
    }
    if (target == "video_recording")
    {
        packet.command = ReadVideoRecordingCommand(action);
        return packet;
    }
    if (target == "inspection_mission")
    {
        packet.command = ReadInspectionCommand(action);
        return packet;
    }
    if (target == "fleet")
    {
        if (action == "add_drone") packet.command = FleetCommand::AddDrone;
        else if (action == "status") packet.command = FleetCommand::GetStatus;
        else throw std::invalid_argument("알 수 없는 편대 동작입니다.");
        return packet;
    }
    if (target != "drone")
        throw std::invalid_argument("알 수 없는 명령 대상입니다.");

    packet.command = ReadDroneCommand(payload, action);
    return packet;
}

std::string EncodeReply(const CommandReply& reply)
{
    const auto& state = reply.telemetry.drone;
    const auto& camera = reply.telemetry.camera;
    Json telemetry{
        {"tick", reply.telemetry.tick},
        {"simulation_time_seconds", reply.telemetry.simulationTimeSeconds},
        {"flight_mode", simulation::ToString(state.mode)},
        {"position_meters", {state.positionMeters.x, state.positionMeters.y, state.positionMeters.z}},
        {"camera_status", aerosimlab::communication::ToString(camera.status)},
        {"camera_frame_count", camera.frameCount},
        {"camera_last_frame_tick", camera.lastFrameTick},
        {"inspection_mission_status", reply.telemetry.inspectionMissionStatus}
    };
    if (reply.status != "fleet_status" && reply.telemetry.drones.size() <= 1U)
    {
        telemetry["velocity_meters_per_second"] = {
            state.velocityMetersPerSecond.x, state.velocityMetersPerSecond.y,
            state.velocityMetersPerSecond.z};
        telemetry["orientation_quaternion"] = {
            {"w", state.orientation.w}, {"x", state.orientation.x},
            {"y", state.orientation.y}, {"z", state.orientation.z}};
        telemetry["angular_velocity_radians_per_second"] = {
            state.angularVelocityRadiansPerSecond.x,
            state.angularVelocityRadiansPerSecond.y,
            state.angularVelocityRadiansPerSecond.z};
        telemetry["motor_speeds_radians_per_second"] = state.motorSpeedsRadiansPerSecond;
        telemetry["yaw_radians"] = state.yawRadians;
    }
    Json payload{
        {"command_id", reply.commandId}, {"sequence", reply.sequence},
        {"accepted", reply.accepted}, {"status", reply.status},
        {"received_at_ms", reply.receivedAtMilliseconds}, {"applied_tick", reply.appliedTick},
        {"telemetry", std::move(telemetry)}
    };
    if (reply.telemetry.drones.size() > maximumFleetDroneCount)
        throw std::invalid_argument("편대 상태 응답에 포함된 드론 수가 허용 범위를 넘었습니다.");
    payload["telemetry"]["drones"] = Json::array();
    for (const DroneTelemetry& drone : reply.telemetry.drones)
    {
        payload["telemetry"]["drones"].push_back({
            {"id", drone.id}, {"name", drone.name},
            {"flight_mode", simulation::ToString(drone.mode)},
            {"position_meters", {drone.positionMeters.x,
                drone.positionMeters.y, drone.positionMeters.z}}
        });
    }
    const std::string datagram = Wrap("reply", std::move(payload)).dump();
    if (datagram.size() > maximumDatagramBytes)
        throw std::invalid_argument("편대 상태 응답이 UDP 메시지 크기 제한을 넘었습니다.");
    return datagram;
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
    reply.telemetry = ReadTelemetry(Required(payload, "telemetry"));
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
