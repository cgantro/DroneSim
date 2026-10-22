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

TEST(GroundControlClientTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_GroundControlClient_SendsCommandAndReadsMatchingUdpReply());
}

}
