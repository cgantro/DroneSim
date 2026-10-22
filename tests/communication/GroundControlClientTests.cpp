#include "GroundControlClient.h"

#include "communication/UdpSocket.h"

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

namespace aerosimlab::ground_control
{
void Verify_GroundControlClient_SendsCommandAndReadsMatchingUdpReply()
{
    communication::UdpSocket simulatorSocket(0);
    communication::UdpSocket groundControlSocket(0);

    std::jthread simulator([&]
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline)
        {
            communication::UdpPeer peer;
            if (const auto request = simulatorSocket.Receive(peer))
            {
                const auto command = communication::DecodeCommand(*request);
                communication::CommandReply reply;
                reply.commandId = command.commandId;
                reply.sequence = command.sequence;
                reply.accepted = true;
                reply.status = "applied";
                reply.appliedTick = 12;
                simulatorSocket.Send(communication::EncodeReply(reply), peer);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    communication::CommandPacket command;
    command.commandId = 7;
    command.sequence = 3;
    command.droneCommand = simulation::DroneCommand{
        simulation::DroneCommand::Kind::TakeOff};

    auto request = std::async(std::launch::async, [&]
    {
        return SendCommand("127.0.0.1", simulatorSocket.LocalPort(),
            command, groundControlSocket, std::chrono::seconds(1));
    });
    const auto result = request.get();
    simulator.join();

    EXPECT_TRUE(result.reply.accepted);
    EXPECT_EQ(result.reply.commandId, command.commandId);
    EXPECT_EQ(result.reply.sequence, command.sequence);
    EXPECT_EQ(result.reply.appliedTick, 12);
}

void Verify_GroundControlClient_StopsWaitingWhenCancelled()
{
    communication::UdpSocket simulatorSocket(0);
    communication::UdpSocket groundControlSocket(0);
    communication::CommandPacket command;
    command.commandId = 8;
    command.sequence = 4;
    command.droneCommand = simulation::DroneCommand{
        simulation::DroneCommand::Kind::TakeOff};

    std::stop_source stopSource;
    auto request = std::async(std::launch::async, [&]
    {
        return SendCommand("127.0.0.1", simulatorSocket.LocalPort(), command,
            groundControlSocket, std::chrono::seconds(4), stopSource.get_token());
    });

    communication::UdpPeer peer;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool commandReceived = false;
    while (!commandReceived && std::chrono::steady_clock::now() < deadline)
    {
        commandReceived = simulatorSocket.Receive(peer).has_value();
        if (!commandReceived)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    stopSource.request_stop();
    ASSERT_TRUE(commandReceived);
    EXPECT_EQ(request.wait_for(std::chrono::milliseconds(100)),
        std::future_status::ready);
    EXPECT_THROW(static_cast<void>(request.get()), CommandCancelled);
}

TEST(GroundControlClientTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_GroundControlClient_SendsCommandAndReadsMatchingUdpReply());
    EXPECT_NO_FATAL_FAILURE(Verify_GroundControlClient_StopsWaitingWhenCancelled());
}

}
