#include "AeroSimLabApp.h"
#include "FleetDroneSpawn.h"

#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"

#include <glm/glm.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <optional>
#include <thread>

using namespace aerosimlab;

void Verify_FleetDroneSpawnRejectsAPositionOutsideTheTerrain()
{
    terrain::TerrainGrid terrainGrid;
    terrainGrid.sampleCount = 3;
    terrainGrid.spacingXMeters = 2.0F;
    terrainGrid.spacingYMeters = 2.0F;
    terrainGrid.heightOffsetsMeters.resize(9U);

    std::optional<simulator::detail::FleetDroneSpawnPose> spawn;
    EXPECT_NO_THROW(spawn = simulator::detail::FindFleetDroneSpawnPose(
        terrainGrid, glm::dvec3{0.0}, 0U));
    EXPECT_FALSE(spawn.has_value());
}

void Verify_SimulatorSession_RecordingReplayOverUdp()
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
                .benchmarkMode = false,
                .groundControlCommandTimeoutTicks = 10
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
    const auto requestTelemetry = [&]
    {
        return send([](communication::CommandPacket& packet)
        {
            packet.cameraCommand = communication::CameraCommand::Enable;
        });
    };
    const auto waitForTelemetry = [&](const auto& isReady, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        auto reply = requestTelemetry();
        while (!isReady(reply) && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            reply = requestTelemetry();
        }
        return reply;
    };

    std::exception_ptr commandFailure;
    try
    {
        const auto beforeStatus = send([](communication::CommandPacket& packet)
        {
            packet.fleetCommand = communication::FleetCommand::GetStatus;
        });
        ASSERT_TRUE(beforeStatus.accepted);
        ASSERT_EQ(beforeStatus.telemetry.drones.size(), 1U);
        EXPECT_EQ(beforeStatus.appliedTick, beforeStatus.telemetry.tick);

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

        const auto hovering = waitForTelemetry([](const communication::CommandReply& reply)
        {
            return reply.telemetry.drone.mode == simulation::FlightMode::Hovering;
        }, std::chrono::seconds(5));
        EXPECT_TRUE(hovering.accepted);
        EXPECT_EQ(hovering.telemetry.drone.mode, simulation::FlightMode::Hovering)
            << "tick=" << hovering.telemetry.tick
            << ", height=" << hovering.telemetry.drone.positionMeters.z
            << " m, vertical speed="
            << hovering.telemetry.drone.velocityMetersPerSecond.z << " m/s";

        const auto velocity = send([](communication::CommandPacket& packet)
        {
            packet.droneCommand = simulation::DroneCommand{
                simulation::DroneCommand::Kind::SetVelocity, {1.0, 0.0, 0.0}};
        });
        EXPECT_TRUE(velocity.accepted);

        const auto stopped = waitForTelemetry([&](const communication::CommandReply& reply)
        {
            return reply.telemetry.tick >= velocity.appliedTick + 12
                && reply.telemetry.drone.mode == simulation::FlightMode::Hovering
                && glm::length(reply.telemetry.drone.velocityMetersPerSecond) < 0.05;
        }, std::chrono::seconds(5));
        EXPECT_TRUE(stopped.accepted);
        EXPECT_EQ(stopped.telemetry.drone.mode, simulation::FlightMode::Hovering);
        EXPECT_LT(glm::length(stopped.telemetry.drone.velocityMetersPerSecond), 0.05);
        const double positionAfterStopMeters = stopped.telemetry.drone.positionMeters.x;

        const auto settled = waitForTelemetry([&](const communication::CommandReply& reply)
        {
            return reply.telemetry.tick >= stopped.telemetry.tick + 60;
        }, std::chrono::seconds(5));
        EXPECT_EQ(settled.telemetry.drone.mode, simulation::FlightMode::Hovering);
        EXPECT_NEAR(settled.telemetry.drone.positionMeters.x, positionAfterStopMeters, 0.1)
            << "stopped vx=" << stopped.telemetry.drone.velocityMetersPerSecond.x
            << ", settled vx=" << settled.telemetry.drone.velocityMetersPerSecond.x
            << ", stop tick=" << stopped.telemetry.tick
            << ", settled tick=" << settled.telemetry.tick;

        const auto added = send([](communication::CommandPacket& packet)
        {
            packet.fleetCommand = communication::FleetCommand::AddDrone;
        });
        ASSERT_TRUE(added.accepted);
        EXPECT_EQ(added.status, "drone_added");
        ASSERT_EQ(added.telemetry.drones.size(), 2U);
        EXPECT_EQ(added.telemetry.drones[1].name, "드론 2");
        EXPECT_EQ(added.telemetry.drones[1].mode, simulation::FlightMode::Grounded);
        EXPECT_GT(added.telemetry.drones[1].positionMeters.x,
            added.telemetry.drones[0].positionMeters.x);

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

TEST(SimulatorSessionTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_FleetDroneSpawnRejectsAPositionOutsideTheTerrain());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorSession_RecordingReplayOverUdp());
}
