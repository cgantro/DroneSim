#include "AeroSimLabApp.h"
#include "EoVideoRecorder.h"
#include "FleetDroneSpawn.h"

#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"

#include <glm/glm.hpp>
#include <gtest/gtest.h>

#include <opencv2/videoio.hpp>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

using namespace aerosimlab;

void Verify_EoVideoRecorderStoresH264AndFrameMetadata()
{
    const auto outputDirectory = std::filesystem::temp_directory_path()
        / "dronesim-video-recorder-test";
    std::filesystem::remove_all(outputDirectory);
    const auto videoPath = outputDirectory / "inspection.mp4";
    simulator::EoVideoRecorder recorder;
    std::string error;
    const simulation::CameraModel cameraModel{};
    ASSERT_TRUE(recorder.Start(videoPath, cameraModel.imageWidthPixels,
        cameraModel.imageHeightPixels, 60.0, error)) << error;

    sensor::RgbFrame frame;
    frame.width = cameraModel.imageWidthPixels;
    frame.height = cameraModel.imageHeightPixels;
    frame.rowOrder = sensor::RgbFrameRowOrder::BottomToTop;
    frame.pixels.resize(static_cast<std::size_t>(frame.width * frame.height * 3), 128U);
    simulation::CameraState camera;
    camera.positionMeters = {1.0, 2.0, 3.0};
    for (std::uint64_t tick = 1; tick <= 4; ++tick)
    {
        frame.simulationTick = tick;
        ASSERT_TRUE(recorder.WriteFrame(frame, camera,
            static_cast<double>(tick) / 60.0, error)) << error;
    }
    EXPECT_EQ(recorder.Stop(), 4U);
    ASSERT_TRUE(std::filesystem::exists(videoPath));
    EXPECT_GT(std::filesystem::file_size(videoPath), 0U);
    cv::VideoCapture decodedVideo(videoPath.string());
    ASSERT_TRUE(decodedVideo.isOpened());
    cv::Mat decodedFrame;
    ASSERT_TRUE(decodedVideo.read(decodedFrame));
    EXPECT_EQ(decodedFrame.cols, cameraModel.imageWidthPixels);
    EXPECT_EQ(decodedFrame.rows, cameraModel.imageHeightPixels);
    decodedVideo.release();

    auto metadataPath = videoPath;
    metadataPath.replace_extension(".jsonl");
    std::ifstream metadata(metadataPath);
    ASSERT_TRUE(metadata.good());
    const std::string contents{
        std::istreambuf_iterator<char>{metadata}, std::istreambuf_iterator<char>{}};
    EXPECT_EQ(std::count(contents.begin(), contents.end(), '\n'), 4);
    EXPECT_NE(contents.find("\"capture_state\":\"SAVED\""), std::string::npos);
    EXPECT_NE(contents.find("\"simulation_tick\":4"), std::string::npos);
    metadata.close();
    std::filesystem::remove_all(outputDirectory);
}

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
    simulator::AeroSimLabApp app({
        .smokeTest = true,
        .udpPort = simulatorPort,
        .demoMode = false,
        .networkEnabled = true,
        .benchmarkMode = false,
        .groundControlCommandTimeoutTicks = 10
    });

    std::uint32_t sequence = 0;
    std::exception_ptr commandFailure;
    std::jthread groundControlThread([&]
    {
        const auto send = [&](const char* operation, const auto& setCommand,
            std::chrono::milliseconds timeout = std::chrono::seconds(5))
        {
            communication::CommandPacket packet;
            packet.commandId = ++sequence;
            packet.sequence = sequence;
            setCommand(packet);
            try
            {
                return ground_control::SendCommand(
                    "127.0.0.1", simulatorPort, packet, clientSocket, timeout).reply;
            }
            catch (const std::exception& error)
            {
                throw std::runtime_error(std::string{operation} + ": " + error.what());
            }
        };
        const auto require = [](bool condition, const char* message)
        {
            if (!condition)
                throw std::runtime_error(message);
        };

        try
        {
            // 초기 그래픽 준비가 끝나고 응답할 때까지 상태 조회를 기다린다.
            const auto beforeStatus = send("초기 상태 조회",
                [](communication::CommandPacket& packet)
            {
                packet.command = communication::FleetCommand::GetStatus;
            }, std::chrono::seconds(15));
            require(beforeStatus.accepted, "초기 상태 조회가 거부되었습니다.");
            require(beforeStatus.telemetry.drones.size() == 1U,
                "초기 상태 응답에 드론 한 대가 있어야 합니다.");
            require(beforeStatus.appliedTick == beforeStatus.telemetry.tick,
                "초기 상태 응답의 Tick 값이 서로 다릅니다.");

            const auto start = send("기록 시작", [](communication::CommandPacket& packet)
            {
                packet.command = communication::SessionCommand::StartRecording;
            });
            require(start.accepted && start.status == "recording_started",
                "기록을 시작하지 못했습니다.");

            const auto takeoff = send("이륙", [](communication::CommandPacket& packet)
            {
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(takeoff.accepted && takeoff.status == "applied",
                "이륙 명령을 적용하지 못했습니다.");

            const auto land = send("착륙", [](communication::CommandPacket& packet)
            {
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::Land};
            });
            require(land.accepted && land.status == "applied",
                "착륙 명령을 적용하지 못했습니다.");

            const auto added = send("드론 추가", [](communication::CommandPacket& packet)
            {
                packet.command = communication::FleetCommand::AddDrone;
            });
            require(added.accepted && added.status == "drone_added",
                "두 번째 드론을 추가하지 못했습니다.");
            require(added.telemetry.drones.size() == 2U,
                "드론 추가 응답에 두 대가 있어야 합니다.");
            require(added.telemetry.drones[1].name == "드론 2",
                "추가된 드론 이름이 예상과 다릅니다.");
            require(added.telemetry.drones[1].mode == simulation::FlightMode::Grounded,
                "추가된 드론이 지상 상태가 아닙니다.");
            require(added.telemetry.drones[1].positionMeters.x
                    > added.telemetry.drones[0].positionMeters.x,
                "추가된 드론이 기존 드론과 겹친 위치에 있습니다.");

            const auto stop = send("기록 종료", [](communication::CommandPacket& packet)
            {
                packet.command = communication::SessionCommand::StopRecording;
            });
            require(stop.accepted && stop.status == "recording_stopped",
                "기록을 종료하지 못했습니다.");

            const auto replay = send("기록 재생", [](communication::CommandPacket& packet)
            {
                packet.command = communication::SessionCommand::ReplayRecording;
            });
            require(replay.accepted && replay.status == "recording_replay_started",
                "기록을 재생하지 못했습니다.");
        }
        catch (...)
        {
            commandFailure = std::current_exception();
        }
    });

    int simulatorExitCode = -1;
    std::exception_ptr simulatorFailure;
    try
    {
        // GLFW 창과 이벤트는 GLFW가 요구하는 메인 스레드에서만 다룬다.
        simulatorExitCode = app.Run();
    }
    catch (...)
    {
        simulatorFailure = std::current_exception();
    }

    groundControlThread.join();
    EXPECT_EQ(simulatorExitCode, 0)
        << "기록 재생이 끝나고 드론이 지상에 도달하면 숨김 시뮬레이터가 종료되어야 합니다.";

    const auto reportFailure = [](const char* context, std::exception_ptr failure)
    {
        if (!failure)
            return;
        try
        {
            std::rethrow_exception(failure);
        }
        catch (const std::exception& error)
        {
            ADD_FAILURE() << context << error.what();
        }
    };
    reportFailure("시뮬레이터 실행 실패: ", simulatorFailure);
    reportFailure("UDP 기록·재생 명령 실패: ", commandFailure);
}

TEST(SimulatorSessionTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_EoVideoRecorderStoresH264AndFrameMetadata());
    EXPECT_NO_FATAL_FAILURE(Verify_FleetDroneSpawnRejectsAPositionOutsideTheTerrain());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorSession_RecordingReplayOverUdp());
}
