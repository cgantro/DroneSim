#include "AeroSimLabApp.h"
#include "SimulationBenchmark.h"

#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"

#include <glm/geometric.hpp>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <future>
#include <string>
#include <thread>
#include <utility>

using namespace aerosimlab;

void Verify_SimulatorBenchmark_SummarizesLatencyPercentiles()
{
    const nlohmann::json summary = simulator::SummarizeDurations({5.0, 1.0, 4.0, 2.0, 3.0});

    EXPECT_EQ(summary.at("samples"), 5U);
    EXPECT_DOUBLE_EQ(summary.at("p50"), 3.0);
    EXPECT_DOUBLE_EQ(summary.at("p95"), 5.0);
    EXPECT_DOUBLE_EQ(summary.at("p99"), 5.0);
}

void Verify_SimulatorBenchmark_BuildsViewerReportFromMeasuredValues()
{
    simulator::ViewerBenchmarkMeasurements measurements;
    measurements.sourceRevision = "test-revision";
    measurements.framebufferWidth = 640;
    measurements.framebufferHeight = 480;
    measurements.graphicsRenderer = "test-renderer";
    measurements.graphicsVersion = "test-version";
    measurements.elapsedSeconds = 2.0;
    measurements.simulationTicks = 120;
    measurements.renderedFrames = 100;
    measurements.processCpuSeconds = 0.5;
    measurements.tickDurationsMicroseconds = {10000.0, 16000.0, 30000.0};
    measurements.applicationFrameDurationsMicroseconds = {1500.0, 1800.0, 2400.0};
    measurements.renderFrameDurationsMicroseconds = {700.0, 800.0, 900.0};
    measurements.renderFrameDurationsWithEoReadbackMicroseconds = {800.0, 1200.0};
    measurements.renderFrameDurationsWithoutEoReadbackMicroseconds = {600.0};
#ifndef __EMSCRIPTEN__
    measurements.physicsStepDurationsMicroseconds = {5000.0, 7000.0};
    measurements.eoReadbackEnqueueDurationsMicroseconds = {1200.0, 1500.0};
    measurements.eoReadbackPollDurationsMicroseconds = {10.0, 20.0};
    measurements.visionDurationsMicroseconds = {4000.0};
    measurements.meanBrightness = 0.4;
    measurements.meanContrast = 0.2;
    measurements.meanSharpness = 0.1;
    measurements.visionSampleCount = 1;
#endif

    const nlohmann::json report = simulator::BuildViewerBenchmarkReport(
        std::move(measurements));

    EXPECT_EQ(report.at("source_revision"), "test-revision");
    EXPECT_EQ(report.at("framebuffer").at("width"), 640);
    EXPECT_EQ(report.at("framebuffer").at("height"), 480);
    EXPECT_EQ(report.at("opengl").at("renderer"), "test-renderer");
    EXPECT_DOUBLE_EQ(report.at("render_fps"), 50.0);
    EXPECT_DOUBLE_EQ(report.at("application_frame_duration_us").at("p50"), 1800.0);
    EXPECT_DOUBLE_EQ(report.at("application_frame_duration_us").at("p95"), 2400.0);
    EXPECT_DOUBLE_EQ(report.at("render_frame_duration_us").at("p50"), 800.0);
    EXPECT_DOUBLE_EQ(report.at("render_frame_duration_us").at("p95"), 900.0);
    EXPECT_DOUBLE_EQ(report.at("render_frame_with_eo_readback_duration_us").at("p95"), 1200.0);
    EXPECT_DOUBLE_EQ(report.at("render_frame_without_eo_readback_duration_us").at("p95"), 600.0);
    EXPECT_DOUBLE_EQ(report.at("process_cpu_percent_approx"), 25.0);
    EXPECT_EQ(report.at("tick_duration_us").at("deadline_misses_over_16_67ms"), 1U);
    EXPECT_DOUBLE_EQ(report.at("tick_duration_us").at("p50"), 16000.0);
#ifndef __EMSCRIPTEN__
    EXPECT_DOUBLE_EQ(report.at("vision_analysis").at("mean_brightness"), 0.4);
    EXPECT_DOUBLE_EQ(report.at("physics_step_duration_us").at("p95"), 7000.0);
    EXPECT_DOUBLE_EQ(report.at("eo_pixel_readback_enqueue_duration_us").at("p95"), 1500.0);
    EXPECT_DOUBLE_EQ(report.at("eo_pixel_readback_poll_duration_us").at("p95"), 20.0);
#endif

    const std::filesystem::path outputPath = std::filesystem::temp_directory_path()
        / ("aerosimlab-benchmark-"
            + std::to_string(std::chrono::steady_clock::now()
                .time_since_epoch().count())
            + ".json");
    simulator::SaveViewerBenchmarkReport(report, outputPath);
    std::ifstream savedFile(outputPath, std::ios::binary);
    const nlohmann::json savedReport = nlohmann::json::parse(savedFile);
    EXPECT_EQ(savedReport.at("source_revision"), "test-revision");
    savedFile.close();
    std::filesystem::remove(outputPath);
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
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorBenchmark_SummarizesLatencyPercentiles());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorBenchmark_BuildsViewerReportFromMeasuredValues());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorSession_RecordingReplayOverUdp());
}
