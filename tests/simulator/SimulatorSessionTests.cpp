#include "AeroSimLabApp.h"
#include "EoVideoRecorder.h"
#include "FleetDroneSpawn.h"

#include "simulation_core/SimulationCore.h"
#include "terrain/MapBuildings.h"
#include "terrain/TerrainGrid.h"

#include "GroundControlClient.h"
#include "communication/Protocol.h"
#include "communication/UdpSocket.h"

#include <glm/glm.hpp>
#include <glm/geometric.hpp>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

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
        cameraModel.imageHeightPixels, 60.0, "osm-building-1", "facade-test-area",
        error)) << error;
    recorder.SetMissionOutcome(simulator::EoVideoRecorder::MissionOutcome::Completed);

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
            static_cast<double>(tick) / 60.0,
            tick == 2 ? simulation::CaptureAssessment::Occluded
                : simulation::CaptureAssessment::Ready,
            error)) << error;
    }
    std::fill(frame.pixels.begin(), frame.pixels.end(), 0U);
    auto metadataPath = videoPath;
    metadataPath.replace_extension(".jsonl");
    EXPECT_FALSE(std::filesystem::exists(metadataPath));
    auto partialMetadataPath = metadataPath;
    partialMetadataPath += ".partial";
    EXPECT_TRUE(std::filesystem::exists(partialMetadataPath));

    std::uint64_t savedFrameCount = 0;
    ASSERT_TRUE(recorder.Stop(savedFrameCount, error)) << error;
    EXPECT_EQ(savedFrameCount, 4U);
    EXPECT_TRUE(std::filesystem::exists(metadataPath));
    EXPECT_FALSE(std::filesystem::exists(partialMetadataPath));
    ASSERT_TRUE(std::filesystem::exists(videoPath));
    EXPECT_GT(std::filesystem::file_size(videoPath), 0U);
    cv::VideoCapture decodedVideo(videoPath.string());
    ASSERT_TRUE(decodedVideo.isOpened());
    cv::Mat decodedFrame;
    ASSERT_TRUE(decodedVideo.read(decodedFrame));
    EXPECT_EQ(decodedFrame.cols, cameraModel.imageWidthPixels);
    EXPECT_EQ(decodedFrame.rows, cameraModel.imageHeightPixels);
    EXPECT_GT(cv::mean(decodedFrame)[0], 100.0);
    decodedVideo.release();

    std::ifstream metadata(metadataPath);
    ASSERT_TRUE(metadata.good());
    const std::string contents{
        std::istreambuf_iterator<char>{metadata}, std::istreambuf_iterator<char>{}};
    EXPECT_EQ(std::count(contents.begin(), contents.end(), '\n'), 4);
    EXPECT_NE(contents.find("\"capture_state\":\"CAPTURED\""), std::string::npos);
    EXPECT_NE(contents.find("\"facility_id\":\"osm-building-1\""), std::string::npos);
    EXPECT_NE(contents.find("\"inspection_zone_id\":\"facade-test-area\""),
        std::string::npos);
    EXPECT_NE(contents.find("\"capture_assessment\":\"occluded\""),
        std::string::npos);
    EXPECT_NE(contents.find("\"simulation_tick\":4"), std::string::npos);
    metadata.close();

    auto manifestPath = videoPath;
    manifestPath.replace_extension(".manifest.json");
    std::ifstream manifest(manifestPath);
    ASSERT_TRUE(manifest.good());
    const std::string manifestText{
        std::istreambuf_iterator<char>{manifest}, std::istreambuf_iterator<char>{}};
    EXPECT_NE(manifestText.find("\"collection_state\": \"SAVED\""),
        std::string::npos);
    EXPECT_NE(manifestText.find("\"video_bytes\""), std::string::npos);
    manifest.close();
    const auto manifestData = nlohmann::json::parse(manifestText);
    EXPECT_EQ(manifestData.at("checksum_algorithm").get<std::string>(),
        "CRC-32/ISO-HDLC");
    EXPECT_EQ(manifestData.at("accepted_frame_count").get<std::uint64_t>(), 4U);
    EXPECT_EQ(manifestData.at("unwritten_frame_count").get<std::uint64_t>(), 0U);
    EXPECT_EQ(manifestData.at("facility_id").get<std::string>(), "osm-building-1");
    EXPECT_EQ(manifestData.at("inspection_zone_id").get<std::string>(),
        "facade-test-area");
    EXPECT_EQ(manifestData.at("mission_outcome").get<std::string>(), "COMPLETED");
    EXPECT_EQ(manifestData.at("geometry_ready_frame_count").get<std::uint64_t>(), 3U);
    std::string calculatedChecksum;
    ASSERT_TRUE(simulator::EoVideoRecorder::CalculateFileChecksum(
        videoPath, calculatedChecksum, error)) << error;
    EXPECT_EQ(calculatedChecksum,
        manifestData.at("video_checksum").get<std::string>());
    ASSERT_TRUE(simulator::EoVideoRecorder::CalculateFileChecksum(
        metadataPath, calculatedChecksum, error)) << error;
    EXPECT_EQ(calculatedChecksum,
        manifestData.at("metadata_checksum").get<std::string>());
    ASSERT_TRUE(simulator::EoVideoRecorder::MarkCaptureDelivered(
        manifestPath, error)) << error;
    std::ifstream deliveredManifestFile(manifestPath);
    ASSERT_TRUE(deliveredManifestFile.good());
    const auto deliveredManifest = nlohmann::json::parse(
        std::istreambuf_iterator<char>{deliveredManifestFile},
        std::istreambuf_iterator<char>{});
    EXPECT_EQ(deliveredManifest.at("collection_state").get<std::string>(),
        "DELIVERED");
    deliveredManifestFile.close();

    const auto failedVideoPath = outputDirectory / "failed-inspection.mp4";
    simulator::EoVideoRecorder failedRecorder;
    ASSERT_TRUE(failedRecorder.Start(failedVideoPath,
        cameraModel.imageWidthPixels, cameraModel.imageHeightPixels, 60.0,
        "osm-building-1", "facade-test-area", error)) << error;
    failedRecorder.SetMissionOutcome(
        simulator::EoVideoRecorder::MissionOutcome::Failed,
        "촬영 조건을 만족하지 못했습니다.");
    frame.simulationTick = 5;
    ASSERT_TRUE(failedRecorder.WriteFrame(frame, camera, 5.0 / 60.0,
        simulation::CaptureAssessment::TooFar, error)) << error;
    ASSERT_TRUE(failedRecorder.Stop(savedFrameCount, error)) << error;
    auto failedManifestPath = failedVideoPath;
    failedManifestPath.replace_extension(".manifest.json");
    std::ifstream failedManifestFile(failedManifestPath);
    ASSERT_TRUE(failedManifestFile.good());
    const auto failedManifest = nlohmann::json::parse(
        std::istreambuf_iterator<char>{failedManifestFile},
        std::istreambuf_iterator<char>{});
    EXPECT_EQ(failedManifest.at("collection_state").get<std::string>(), "SAVED");
    EXPECT_EQ(failedManifest.at("mission_outcome").get<std::string>(), "FAILED");
    EXPECT_EQ(failedManifest.at("mission_failure_reason").get<std::string>(),
        "촬영 조건을 만족하지 못했습니다.");
    failedManifestFile.close();
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

void Verify_TancheonBuildingDataStopsTheWebKinematicDrone()
{
    const std::filesystem::path mapDirectory{"maps/tancheon"};
    const auto terrainGrid = terrain::TerrainGrid::Load(
        mapDirectory / "terrain.aerotrn");
    const auto buildings = terrain::LoadMapBuildingFootprints(
        mapDirectory / "site.geojson", terrainGrid);
    ASSERT_FALSE(buildings.empty());

    simulation::SimulationCore webSimulation{
        simulation::SimulationDynamics::Kinematic};
    webSimulation.SetTerrain(terrainGrid);
    for (const auto& building : buildings)
    {
        if (building.outlineMeters.size() < 3U)
            continue;

        if (building.heightSource == terrain::BuildingHeightSource::OsmHeightTag
            && building.heightMeters > 0.0)
        {
            const auto bounds = terrain::GetMapBuildingBoundsMeters(building);
            webSimulation.AddStaticObstaclePrism(building.outlineMeters,
                bounds.centerMeters.z - bounds.halfExtentsMeters.z,
                bounds.centerMeters.z + bounds.halfExtentsMeters.z);
        }
        else
        {
            webSimulation.AddStaticObstacleFootprint(building.outlineMeters);
        }
    }

    bool foundTargetBuilding = false;
    terrain::MapBuildingFacade targetFacade;
    for (const auto& building : buildings)
    {
        if (building.heightSource != terrain::BuildingHeightSource::OsmHeightTag
            || !(building.heightMeters > 0.0))
            continue;

        const auto bounds = terrain::GetMapBuildingBoundsMeters(building);
        const auto facade = terrain::GetNearestMapBuildingFacade(building,
            {bounds.centerMeters.x + 100.0, bounds.centerMeters.y});
        if (facade.widthMeters < 3.0)
            continue;

        const glm::dvec2 startPosition =
            glm::dvec2{facade.centerMeters.x, facade.centerMeters.y}
            + glm::dvec2{facade.outwardNormal.x, facade.outwardNormal.y} * 2.0;
        if (!terrainGrid.ContainsLocalPositionMeters(startPosition))
            continue;

        foundTargetBuilding = true;
        targetFacade = facade;
        break;
    }
    ASSERT_TRUE(foundTargetBuilding)
        << "충돌 시험에 쓸 높이 정보가 있는 OSM 건물을 찾지 못했습니다.";

    const glm::dvec2 outwardNormal{
        targetFacade.outwardNormal.x, targetFacade.outwardNormal.y};
    const glm::dvec2 startPosition{
        targetFacade.centerMeters.x, targetFacade.centerMeters.y};
    simulation::DroneState start;
    start.positionMeters = {
        startPosition.x + outwardNormal.x * 2.0,
        startPosition.y + outwardNormal.y * 2.0,
        targetFacade.centerMeters.z};
    start.velocityMetersPerSecond = {
        -outwardNormal.x * 5.0, -outwardNormal.y * 5.0, 0.0};
    start.mode = simulation::FlightMode::Flying;
    ASSERT_NO_THROW(webSimulation.SetInitialState(
        webSimulation.PrimaryDroneId(), start));

    for (int tick = 0; tick < 60; ++tick)
        webSimulation.AdvanceOneTick();

    const auto& stopped = webSimulation.State().positionMeters;
    const double distanceFromFacade =
        (stopped.x - startPosition.x) * outwardNormal.x
        + (stopped.y - startPosition.y) * outwardNormal.y;
    EXPECT_GE(distanceFromFacade, 0.3)
        << "탄천 건물 외벽을 향해 간 드론이 건물 외곽선을 통과했습니다.";
    EXPECT_LE(distanceFromFacade, 1.2)
        << "드론이 선택한 건물 외벽에 도달하기 전에 다른 곳에서 멈췄습니다.";
    EXPECT_NEAR(glm::dot(webSimulation.State().velocityMetersPerSecond,
        glm::dvec3{outwardNormal, 0.0}), 0.0, 1.0e-9);
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
            require(beforeStatus.telemetry.primaryDroneId
                    == beforeStatus.telemetry.drones.front().id,
                "주 드론 식별값이 편대 목록과 일치해야 합니다.");
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
            const auto unrecordedSecondaryTakeoff = send("기록 중 두 번째 드론 이륙",
                [&](communication::CommandPacket& packet)
            {
                packet.droneId = added.telemetry.drones[1].id;
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(!unrecordedSecondaryTakeoff.accepted
                    && unrecordedSecondaryTakeoff.status == "recording_secondary_unsupported",
                "기록에 남지 않는 추가 드론 명령을 허용했습니다.");

            const auto stop = send("기록 종료", [](communication::CommandPacket& packet)
            {
                packet.command = communication::SessionCommand::StopRecording;
            });
            require(stop.accepted && stop.status == "recording_stopped",
                "기록을 종료하지 못했습니다.");

            const auto secondaryTakeoff = send("두 번째 드론 이륙",
                [&](communication::CommandPacket& packet)
            {
                packet.droneId = added.telemetry.drones[1].id;
                packet.command = simulation::DroneCommand{
                    simulation::DroneCommand::Kind::TakeOff};
            });
            require(secondaryTakeoff.accepted,
                "선택한 두 번째 드론에 이륙 명령을 보내지 못했습니다.");
            require(secondaryTakeoff.telemetry.drones[1].mode
                    == simulation::FlightMode::TakingOff,
                "선택한 드론이 이륙 상태로 바뀌지 않았습니다.");
            require(secondaryTakeoff.telemetry.drone.mode
                    != simulation::FlightMode::TakingOff,
                "두 번째 드론 명령이 주 드론 상태를 바꿨습니다.");

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
    EXPECT_NO_FATAL_FAILURE(Verify_TancheonBuildingDataStopsTheWebKinematicDrone());
    EXPECT_NO_FATAL_FAILURE(Verify_SimulatorSession_RecordingReplayOverUdp());
}
