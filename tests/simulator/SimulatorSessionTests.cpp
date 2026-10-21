#include "AeroSimLabApp.h"

#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <thread>

using namespace aerosimlab;

TEST(SimulatorSession, RecordingReplayOverUdp)
{
    std::uint16_t simulatorPort = 0;
    {
        communication::UdpSocket availablePort(0);
        simulatorPort = availablePort.LocalPort();
    }

    communication::UdpSocket clientSocket(0);
    std::promise<int> simulatorExit;
    auto exitResult = simulatorExit.get_future();
    std::jthread simulatorThread([&]
    {
        try
        {
            simulator::AeroSimLabApp app({
                .smokeTest = true,
                .udpPort = simulatorPort,
                .demoMode = false,
                .networkEnabled = true,
                .benchmarkMode = false
            });
            simulatorExit.set_value(app.Run());
        }
        catch (...)
        {
            simulatorExit.set_exception(std::current_exception());
        }
    });

    std::uint32_t sequence = 0;
    auto send = [&](const auto& setCommand)
    {
        communication::CommandPacket packet;
        packet.commandId = ++sequence;
        packet.sequence = sequence;
        setCommand(packet);
        return ground_control::SendCommand(
            "127.0.0.1", simulatorPort, packet, clientSocket,
            std::chrono::seconds(5)).reply;
    };

    std::exception_ptr commandFailure;
    try
    {
        const auto start = send([](communication::CommandPacket& packet)
        {
            packet.sessionCommand = communication::SessionCommand::StartRecording;
        });
        EXPECT_TRUE(start.accepted);
        EXPECT_EQ(start.status, "recording_started");

        const auto takeoff = send([](communication::CommandPacket& packet)
        {
            packet.droneCommand = simulation::DroneCommand{simulation::DroneCommand::Kind::TakeOff};
        });
        EXPECT_TRUE(takeoff.accepted);
        EXPECT_EQ(takeoff.status, "applied");

        const auto land = send([](communication::CommandPacket& packet)
        {
            packet.droneCommand = simulation::DroneCommand{simulation::DroneCommand::Kind::Land};
        });
        EXPECT_TRUE(land.accepted);
        EXPECT_EQ(land.status, "applied");

        const auto stop = send([](communication::CommandPacket& packet)
        {
            packet.sessionCommand = communication::SessionCommand::StopRecording;
        });
        EXPECT_TRUE(stop.accepted);
        EXPECT_EQ(stop.status, "recording_stopped");

        const auto replay = send([](communication::CommandPacket& packet)
        {
            packet.sessionCommand = communication::SessionCommand::ReplayRecording;
        });
        EXPECT_TRUE(replay.accepted);
        EXPECT_EQ(replay.status, "recording_replay_started");
    }
    catch (...)
    {
        commandFailure = std::current_exception();
    }

    const auto completed = exitResult.wait_for(std::chrono::seconds(20));
    EXPECT_EQ(completed, std::future_status::ready)
        << "재생이 끝나고 드론이 지상에 도달하면 숨김 시뮬레이터가 종료되어야 합니다.";

    if (completed == std::future_status::ready)
    {
        try
        {
            EXPECT_EQ(exitResult.get(), 0);
        }
        catch (const std::exception& error)
        {
            ADD_FAILURE() << "시뮬레이터 실행 실패: " << error.what();
        }
    }
    if (commandFailure)
    {
        try
        {
            std::rethrow_exception(commandFailure);
        }
        catch (const std::exception& error)
        {
            ADD_FAILURE() << "UDP 기록·재생 명령 실패: " << error.what();
        }
    }

    if (simulatorThread.joinable())
        simulatorThread.join();
}
