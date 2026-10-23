#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"
#include "simulation_core/ScenarioRunner.h"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
constexpr std::string_view helpText =
    "사용법: aerosimlab_ground_control [--host IPv4] [--port 번호] --command 동작 [x y z]\n"
    "       aerosimlab_ground_control [--host IPv4] [--port 번호] --camera on|off|fault|recover\n"
    "드론 동작: takeoff, land, stop, velocity x y z, waypoint x y z, yaw-rate 값\n";

std::uint16_t ParsePort(std::string_view text)
{
    unsigned int value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() || value == 0 || value > 65535)
        throw std::invalid_argument("포트 번호는 1부터 65535 사이여야 합니다.");
    return static_cast<std::uint16_t>(value);
}

double ParseNumber(std::string_view text)
{
    double value = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() || !std::isfinite(value))
        throw std::invalid_argument("좌표와 속도는 유한한 숫자여야 합니다.");
    return value;
}

aerosimlab::simulation::DroneCommand ParseDroneCommand(
    std::string_view name, const std::vector<std::string_view>& values)
{
    using Command = aerosimlab::simulation::DroneCommand;
    using Kind = Command::Kind;

    Command command;
    std::size_t expectedValueCount = 0;
    if (name == "takeoff")
        command.kind = Kind::TakeOff;
    else if (name == "land")
        command.kind = Kind::Land;
    else if (name == "stop")
        command.kind = Kind::Stop;
    else if (name == "velocity")
    {
        command.kind = Kind::SetVelocity;
        expectedValueCount = 3;
    }
    else if (name == "waypoint")
    {
        command.kind = Kind::SetWaypoint;
        expectedValueCount = 3;
    }
    else if (name == "yaw-rate")
    {
        command.kind = Kind::SetYawRate;
        expectedValueCount = 1;
    }
    else
        throw std::invalid_argument("지원하지 않는 드론 동작입니다.");

    if (values.size() != expectedValueCount)
        throw std::invalid_argument("동작에 필요한 좌표·속도 값의 개수가 맞지 않습니다.");

    switch (command.kind)
    {
    case Kind::SetVelocity:
    case Kind::SetWaypoint:
        command.vector = {ParseNumber(values[0]), ParseNumber(values[1]), ParseNumber(values[2])};
        break;
    case Kind::SetYawRate:
        command.yawRateRadiansPerSecond = ParseNumber(values[0]);
        break;
    default:
        break;
    }
    return command;
}

aerosimlab::communication::CameraCommand ParseCameraCommand(std::string_view name)
{
    using aerosimlab::communication::CameraCommand;
    if (name == "on") return CameraCommand::Enable;
    if (name == "off") return CameraCommand::Disable;
    if (name == "fault") return CameraCommand::InjectFault;
    if (name == "recover") return CameraCommand::Recover;
    throw std::invalid_argument("카메라 동작은 on, off, fault, recover 중 하나여야 합니다.");
}

void PrintReply(const aerosimlab::communication::CommandReply& reply, std::uint64_t responseTimeMilliseconds)
{
    const auto& state = reply.telemetry.drone;
    std::cout << "명령 " << reply.commandId << ": " << reply.status
              << " (응답 " << responseTimeMilliseconds << "ms, 적용 Tick " << reply.appliedTick
              << ", 실행 시작 뒤 수신 " << reply.receivedAtMilliseconds << "ms)\n"
              << "비행 상태: " << aerosimlab::simulation::ToString(state.mode)
              << ", 위치: (" << state.positionMeters.x << ", " << state.positionMeters.y
              << ", " << state.positionMeters.z << ")m\n"
              << "카메라: " << aerosimlab::communication::ToString(reply.telemetry.camera.status)
              << ", 생성 프레임: " << reply.telemetry.camera.frameCount << '\n';
}
}

int main(int argc, char** argv)
{
    try
    {
        std::string host = "127.0.0.1";
        std::uint16_t port = aerosimlab::communication::defaultPort;
        std::string_view commandName;
        std::string_view cameraName;
        std::vector<std::string_view> actionValues;
        for (int index = 1; index < argc; ++index)
        {
            const std::string_view argument(argv[index]);
            if (argument == "--help")
            {
                std::cout << helpText;
                return 0;
            }
            if ((argument == "--host" || argument == "--port" || argument == "--command" || argument == "--camera")
                && index + 1 < argc)
            {
                const std::string_view value(argv[++index]);
                if (argument == "--host") host = value;
                else if (argument == "--port") port = ParsePort(value);
                else if (argument == "--command") commandName = value;
                else cameraName = value;
                continue;
            }
            if (!argument.empty() && argument.front() != '-')
            {
                actionValues.push_back(argument);
                continue;
            }
            throw std::invalid_argument("알 수 없는 옵션 또는 값입니다.");
        }

        aerosimlab::communication::CommandPacket packet;
        packet.commandId = 1;
        packet.sequence = 1;
        if (!commandName.empty() && cameraName.empty())
        {
            packet.command = ParseDroneCommand(commandName, actionValues);
        }
        else if (commandName.empty() && !cameraName.empty())
        {
            packet.command = ParseCameraCommand(cameraName);
        }
        else
        {
            std::cout << helpText;
            return 2;
        }

        aerosimlab::communication::UdpSocket socket(0);
        const auto result = aerosimlab::ground_control::SendCommand(host, port, packet, socket);
        PrintReply(result.reply, result.responseTimeMilliseconds);
        return result.reply.accepted ? 0 : 1;
    }
    catch (const aerosimlab::ground_control::CommandTimeout&)
    {
        std::cerr << "지상통제 응답 시간 초과: 시뮬레이터가 실행 중인지 확인하세요.\n";
        return 3;
    }
    catch (const std::exception& error)
    {
        std::cerr << "지상통제 명령 실패: " << error.what() << '\n';
        return 2;
    }
}
