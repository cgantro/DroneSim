#include "AeroSimLabApp.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#include <Psapi.h>
#endif

#include "graphics/Camera.h"
#include "graphics/GlApi.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#include <glm/gtc/quaternion.hpp>
#ifndef __EMSCRIPTEN__
#include "vision/RgbFrameQuality.h"
#include "vision/RgbFrameView.h"
#endif
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#ifndef _WIN32
#include <iostream>
#endif
#include <string>
#include <ctime>
#include <cmath>
#include <nlohmann/json.hpp>
#include <utility>
#include <vector>

#ifndef __EMSCRIPTEN__
#include <GLFW/glfw3.h>
#endif

namespace aerosimlab::simulator
{
namespace
{
constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
// 창이 멈춘 뒤 밀린 시간을 전부 따라잡느라 다음 화면까지 늦어지지 않게 한다.
constexpr std::size_t smokeTestFrameLimit = 8;
constexpr std::size_t demoLengthTicks = 900;
constexpr double benchmarkDurationSeconds = 5.0;

double Percentile(const std::vector<double>& sortedDurations, double fraction)
{
    if (sortedDurations.empty())
        return 0.0;

    const std::size_t index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(sortedDurations.size()))) - 1U;
    return sortedDurations[std::min(index, sortedDurations.size() - 1U)];
}

nlohmann::json SummarizeDurations(std::vector<double> durations)
{
    std::sort(durations.begin(), durations.end());
    return {
        {"p50", Percentile(durations, 0.50)},
        {"p95", Percentile(durations, 0.95)},
        {"p99", Percentile(durations, 0.99)},
        {"samples", durations.size()}};
}

#ifndef __EMSCRIPTEN__
nlohmann::json MeasureDroneScaling()
{
    constexpr std::array<std::size_t, 4> droneCounts{1U, 2U, 4U, 8U};
    constexpr std::size_t repetitions = 3U;
    constexpr std::size_t warmupTicks = 120U;
    constexpr std::size_t measuredTicks = 300U;
    nlohmann::json runs = nlohmann::json::array();

    for (const std::size_t droneCount : droneCounts)
    {
        for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
        {
            aerosimlab::simulation::SimulationCore simulation{
                aerosimlab::simulation::SimulationDynamics::RigidBody};
            simulation.SetPhysicsStepTimingEnabled(true);

            std::vector<aerosimlab::simulation::DroneId> droneIds;
            droneIds.reserve(droneCount);
            droneIds.push_back(simulation.PrimaryDroneId());
            const double groundOriginMeters = simulation.State().positionMeters.z;
            for (std::size_t index = 1; index < droneCount; ++index)
            {
                const auto xMeters = static_cast<double>(index) * 2.5;
                droneIds.push_back(simulation.AddDrone(
                    glm::dvec3{xMeters, 0.0, groundOriginMeters}));
            }

            const aerosimlab::simulation::DroneCommand takeoff{
                aerosimlab::simulation::DroneCommand::Kind::TakeOff};
            for (const aerosimlab::simulation::DroneId droneId : droneIds)
            {
                if (simulation.Apply(droneId, takeoff)
                    != aerosimlab::simulation::CommandResult::Accepted)
                    throw std::runtime_error("멀티드론 성능 측정용 이륙 명령을 적용하지 못했습니다.");
            }

            for (std::size_t tick = 0; tick < warmupTicks; ++tick)
                simulation.AdvanceOneTick();

            std::vector<double> tickDurations;
            std::vector<double> physicsDurations;
            tickDurations.reserve(measuredTicks);
            physicsDurations.reserve(measuredTicks);
            for (std::size_t tick = 0; tick < measuredTicks; ++tick)
            {
                const auto tickStart = std::chrono::steady_clock::now();
                simulation.AdvanceOneTick();
                tickDurations.push_back(std::chrono::duration<double, std::micro>(
                    std::chrono::steady_clock::now() - tickStart).count());
                physicsDurations.push_back(
                    simulation.LastPhysicsStepDurationMicroseconds());
            }

            runs.push_back({
                {"drone_count", droneCount},
                {"repetition", repetition + 1U},
                {"warmup_ticks", warmupTicks},
                {"measured_ticks", measuredTicks},
                {"tick_duration_us", SummarizeDurations(std::move(tickDurations))},
                {"physics_step_duration_us", SummarizeDurations(std::move(physicsDurations))}});
        }
    }

    return {
        {"method", "헤드리스 강체 모의; 한 Jolt 세계에서 모든 드론을 함께 갱신"},
        {"counts", droneCounts},
        {"repetitions_per_count", repetitions},
        {"runs", std::move(runs)}};
}
#endif

std::filesystem::path BenchmarkOutputPath()
{
#ifdef _WIN32
    std::wstring executablePath(32768U, L'\0');
    const DWORD pathLength = GetModuleFileNameW(
        nullptr, executablePath.data(), static_cast<DWORD>(executablePath.size()));
    if (pathLength == 0U || pathLength >= executablePath.size())
        throw std::runtime_error("실행 파일의 위치를 확인할 수 없습니다.");

    executablePath.resize(pathLength);
    return std::filesystem::path(executablePath).parent_path() / L"benchmark-latest.json";
#else
    return std::filesystem::current_path() / "benchmark-latest.json";
#endif
}

glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    // 모의는 Z축을 위로 쓰고 화면은 Y축을 위로 쓴다. Y축 방향은 화면의 -Z로 옮겨 오른손 좌표계를 유지한다.
    return {static_cast<float>(positionMeters.x), static_cast<float>(positionMeters.z),
        static_cast<float>(-positionMeters.y)};
}

glm::quat ToRenderOrientation(const glm::dquat& orientation)
{
    constexpr float halfPi = 1.57079632679F;
    const glm::quat simulationOrientation{
        static_cast<float>(orientation.w),
        static_cast<float>(orientation.x),
        static_cast<float>(orientation.y),
        static_cast<float>(orientation.z)};
    const glm::quat simulationToRender = glm::angleAxis(
        -halfPi, glm::vec3{1.0F, 0.0F, 0.0F});
    return glm::normalize(
        simulationToRender * simulationOrientation * glm::inverse(simulationToRender));
}

glm::mat4 CubeTransform(const glm::vec3& position, const glm::vec3& size)
{
    return glm::translate(glm::mat4{1.0F}, position) * glm::scale(glm::mat4{1.0F}, size);
}

/**
 * @brief 북쪽부터 기록한 높이 격자를 화면의 삼각형 지형으로 바꾼다.
 * @details 화면은 Y축을 위로 쓰므로 높이를 Y에 둔다. 북쪽은 화면의 -Z 방향이다.
 */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateTerrainMesh(
    const aerosimlab::terrain::TerrainGrid& terrain)
{
    const std::size_t sampleCount = terrain.sampleCount;
    const std::size_t vertexCount = sampleCount * sampleCount;
    std::vector<aerosimlab::graphics::Vertex> vertices(vertexCount);
    const auto heightAt = [&terrain, sampleCount](std::size_t row, std::size_t column)
    {
        return terrain.heightOffsetsMeters[row * sampleCount + column];
    };

    const float center = static_cast<float>(sampleCount - 1U) * 0.5F;
    for (std::size_t row = 0; row < sampleCount; ++row)
    {
        const std::size_t beforeRow = row == 0U ? row : row - 1U;
        const std::size_t afterRow = std::min(row + 1U, sampleCount - 1U);
        for (std::size_t column = 0; column < sampleCount; ++column)
        {
            const std::size_t beforeColumn = column == 0U ? column : column - 1U;
            const std::size_t afterColumn = std::min(column + 1U, sampleCount - 1U);
            const float eastWestDistance = static_cast<float>(afterColumn - beforeColumn)
                * terrain.spacingXMeters;
            const float northSouthDistance = static_cast<float>(afterRow - beforeRow)
                * terrain.spacingYMeters;
            const float slopeEast = eastWestDistance > 0.0F
                ? (heightAt(row, afterColumn) - heightAt(row, beforeColumn)) / eastWestDistance
                : 0.0F;
            const float slopeSouth = northSouthDistance > 0.0F
                ? (heightAt(afterRow, column) - heightAt(beforeRow, column)) / northSouthDistance
                : 0.0F;
            const glm::vec3 normal = glm::normalize(glm::vec3{-slopeEast, 1.0F, -slopeSouth});
            vertices[row * sampleCount + column] = {
                glm::vec3{
                    (static_cast<float>(column) - center) * terrain.spacingXMeters,
                    heightAt(row, column),
                    (static_cast<float>(row) - center) * terrain.spacingYMeters},
                normal};
        }
    }

    std::vector<std::uint32_t> indices;
    indices.reserve((sampleCount - 1U) * (sampleCount - 1U) * 6U);
    for (std::size_t row = 0; row + 1U < sampleCount; ++row)
    {
        for (std::size_t column = 0; column + 1U < sampleCount; ++column)
        {
            const auto northWest = static_cast<std::uint32_t>(row * sampleCount + column);
            const auto northEast = northWest + 1U;
            const auto southWest = northWest + static_cast<std::uint32_t>(sampleCount);
            const auto southEast = southWest + 1U;
            indices.insert(indices.end(), {
                northWest, southWest, northEast,
                northEast, southWest, southEast});
        }
    }

    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}
}

AeroSimLabApp::AeroSimLabApp(Options options)
    : smokeTest_(options.smokeTest), demoMode_(options.demoMode),
      networkEnabled_(options.networkEnabled), benchmarkMode_(options.benchmarkMode),
      udpPort_(options.udpPort)
{
    if (benchmarkMode_)
        simulation_.SetPhysicsStepTimingEnabled(true);
#ifndef __EMSCRIPTEN__
    visionAnalyzer_ = std::make_unique<aerosimlab::vision::RgbFrameAnalyzer>();
#endif
}

AeroSimLabApp::~AeroSimLabApp() = default;

int AeroSimLabApp::Run()
{
    InitializeGraphics();
#ifndef __EMSCRIPTEN__
    if (networkEnabled_)
    {
        udpSocket_ = std::make_unique<aerosimlab::communication::UdpSocket>(udpPort_);
    }
#endif
    startTime_ = std::chrono::steady_clock::now();
    benchmarkCpuStart_ = std::clock();
    auto previousFrame = startTime_;

    while (!window_->ShouldClose())
    {
        window_->PollEvents();
        PollGroundControl();
        PollGamepad();
        const auto now = std::chrono::steady_clock::now();
        const double frameSeconds = smokeTest_ && !benchmarkMode_
            ? aerosimlab::simulation::FixedTickAccumulator::fixedStepSeconds
            : std::chrono::duration<double>(now - previousFrame).count();
        previousFrame = now;

        AdvanceSimulation(frameSeconds);
        RenderFrame();
        window_->SwapBuffers();
        ++renderedFrames_;

        if (smokeTest_ && !networkEnabled_ && renderedFrames_ >= smokeTestFrameLimit)
            break;
        if (smokeTest_ && networkEnabled_ && smokeReplayStarted_
            && !recordingActive_ && !replaying_
            && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded)
            break;
        if (smokeTest_ && networkEnabled_ &&
            std::chrono::steady_clock::now() - startTime_ > std::chrono::seconds(15))
            return 4;
        if (benchmarkMode_ &&
            std::chrono::steady_clock::now() - startTime_ >= std::chrono::duration<double>(benchmarkDurationSeconds))
            break;
    }
    const bool cameraFrameHasPixels = eoFrame_.IsValid()
        && std::any_of(eoFrame_.pixels.begin(), eoFrame_.pixels.end(), [](std::uint8_t value)
        {
            return value != 0;
        });
    if (smokeTest_ && !cameraFrameHasPixels)
    {
        return 5;
    }
    if (benchmarkMode_)
    {
        const double elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime_).count();
        WriteBenchmarkReport(elapsedSeconds);
    }
    return 0;
}

void AeroSimLabApp::InitializeGraphics()
{
    window_ = std::make_unique<aerosimlab::graphics::Window>(
        aerosimlab::graphics::Window::Properties{windowWidth, windowHeight,
            "AeroSimLab | 드론 관찰 · EO 영상", !smokeTest_ && !benchmarkMode_,
            !smokeTest_ && !benchmarkMode_});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    window_->GetFramebufferSize(framebufferWidth, framebufferHeight);

    renderer_ = std::make_unique<aerosimlab::graphics::Renderer>();
    renderer_->Init(framebufferWidth, framebufferHeight);
    observerCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{7.0F, 5.0F, 7.0F}, glm::vec3{0.0F}, 1.0F);
    eoCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F, 70.0F, 0.05F, 200.0F);

    cube_ = aerosimlab::graphics::Mesh::CreateCube(1.0F);
    droneMesh_ = aerosimlab::graphics::Mesh::LoadGlb("models/drones/x500/visual.glb");
    terrain_ = aerosimlab::terrain::TerrainGrid::Load("maps/bukhansan-demo/terrain.grid");
    terrainMesh_ = CreateTerrainMesh(terrain_);
#ifndef __EMSCRIPTEN__
    simulation_.SetTerrain(terrain_);
#endif
    shader_ = std::make_unique<aerosimlab::graphics::Shader>("shaders/Scene.glsl");
}

void AeroSimLabApp::AdvanceSimulation(double elapsedSeconds)
{
    const std::size_t tickCount = fixedTickAccumulator_.Consume(elapsedSeconds);
    for (std::size_t tick = 0; tick < tickCount; ++tick)
    {
        const auto tickStart = std::chrono::steady_clock::now();
        const std::uint64_t previousSimulationTick = simulation_.TickCount();
        AdvanceSimulationTick();
        if (benchmarkMode_)
        {
            tickDurationsMicroseconds_.push_back(std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - tickStart).count());
#ifndef __EMSCRIPTEN__
            if (simulation_.TickCount() != previousSimulationTick)
                physicsStepDurationsMicroseconds_.push_back(
                    simulation_.LastPhysicsStepDurationMicroseconds());
#endif
        }
    }
}

void AeroSimLabApp::AdvanceSimulationTick()
{
    if (demoMode_ && !externalControl_)
        ApplyDemoCommands();
#ifndef __EMSCRIPTEN__
    const bool hasGroundCommand = pendingCommand_.has_value();
#else
    constexpr bool hasGroundCommand = false;
#endif
    ApplyGroundCommand();
    if (hasGroundCommand)
        return;

    if (replaying_)
    {
        AdvanceReplayTick();
        return;
    }

    AdvanceManualControlTick();
}

void AeroSimLabApp::AdvanceReplayTick()
{
    ApplyRecordedCommands();
    simulation_.AdvanceOneTick();
    ++replayTick_;
    if (replayTick_ >= recordingTickCount_)
    {
        replaying_ = false;
        window_->SetTitle("AeroSimLab | 기록 재생 완료");
    }
}

void AeroSimLabApp::AdvanceManualControlTick()
{
    if (!groundControlActive_ && pendingControllerAction_)
    {
        const auto result = simulation_.Apply(*pendingControllerAction_);
        if (result == aerosimlab::simulation::CommandResult::Accepted)
            RecordCommand(*pendingControllerAction_);
        pendingControllerAction_.reset();
    }
    if (!groundControlActive_ && pendingPilotCommand_)
    {
        const auto mode = simulation_.State().mode;
        if (mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying)
        {
            const aerosimlab::simulation::DroneCommand velocityCommand{
                aerosimlab::simulation::DroneCommand::Kind::SetVelocity,
                pendingPilotCommand_->velocityMetersPerSecond};
            const aerosimlab::simulation::DroneCommand yawCommand{
                aerosimlab::simulation::DroneCommand::Kind::SetYawRate,
                glm::dvec3{0.0},
                pendingPilotCommand_->yawRateRadiansPerSecond};
            if (simulation_.Apply(velocityCommand)
                == aerosimlab::simulation::CommandResult::Accepted)
                RecordCommand(velocityCommand);
            if (simulation_.Apply(yawCommand)
                == aerosimlab::simulation::CommandResult::Accepted)
                RecordCommand(yawCommand);
        }
    }

    simulation_.AdvanceOneTick();
    if (recordingActive_)
        ++recordingTickCount_;
}

void AeroSimLabApp::ApplyDemoCommands()
{
    using Kind = aerosimlab::simulation::DroneCommand::Kind;
    const std::size_t tick = static_cast<std::size_t>(simulation_.TickCount()) % demoLengthTicks;
    aerosimlab::simulation::DroneCommand command;
    bool hasCommand = true;

    if (tick == 0)
        command.kind = Kind::TakeOff;
    else if (tick == 120)
    {
        command.kind = Kind::SetWaypoint;
        command.vector = {8.0, 0.0, 2.0};
    }
    else if (tick == 300)
    {
        command.kind = Kind::SetYawRate;
        command.yawRateRadiansPerSecond = 0.35;
    }
    else if (tick == 360)
        command.kind = Kind::Stop;
    else if (tick == 600)
        command.kind = Kind::Land;
    else
        hasCommand = false;

    if (hasCommand)
        (void)simulation_.Apply(command);
}

void AeroSimLabApp::RenderFrame()
{
    int width = 0;
    int height = 0;
    window_->GetFramebufferSize(width, height);
    if (width <= 0 || height <= 0)
        return;

    renderer_->Resize(width, height);
    const int observerWidth = width / 2;
    const int eoWidth = width - observerWidth;
    UpdateCameraTransforms(observerWidth, eoWidth, height);

    renderer_->BeginFrame();
    renderer_->SetSceneViewport(0, 0, observerWidth, height);
    DrawWorld(observerCamera_->GetViewMatrix(), observerCamera_->GetProjectionMatrix(),
        observerCamera_->GetPosition());
    renderer_->SetSceneViewport(observerWidth, 0, eoWidth, height);
    if (simulation_.Camera().status == aerosimlab::simulation::CameraStatus::Active)
        DrawWorld(eoCamera_->GetViewMatrix(), eoCamera_->GetProjectionMatrix(),
            eoCamera_->GetPosition());
    renderer_->SetSceneViewport(0, 0, width, height);
    renderer_->EndFrame();

    CaptureEoFrame(observerWidth, eoWidth, height);
}

void AeroSimLabApp::UpdateCameraTransforms(int observerWidth, int eoWidth, int height)
{
    const auto& state = simulation_.State();
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const glm::mat4 heading = glm::mat4_cast(ToRenderOrientation(state.orientation));

    observerCamera_->SetPosition(dronePosition + glm::vec3{7.0F, 5.0F, 7.0F});
    observerCamera_->SetTarget(dronePosition + glm::vec3{0.0F, 0.2F, 0.0F});
    observerCamera_->SetAspectRatio(static_cast<float>(observerWidth) / static_cast<float>(height));

    const glm::vec3 cameraPosition = dronePosition + glm::vec3{
        heading * glm::vec4{0.35F, 0.12F, 0.0F, 0.0F}};
    const glm::vec3 forward = glm::vec3{heading * glm::vec4{1.0F, 0.0F, 0.0F, 0.0F}};
    eoCamera_->SetPosition(cameraPosition);
    eoCamera_->SetTarget(cameraPosition + forward * 25.0F + glm::vec3{0.0F, -2.5F, 0.0F});
    eoCamera_->SetAspectRatio(static_cast<float>(eoWidth) / static_cast<float>(height));
}

void AeroSimLabApp::CaptureEoFrame(int observerWidth, int eoWidth, int height)
{
    const auto& camera = simulation_.Camera();
    if (camera.status == aerosimlab::simulation::CameraStatus::Active
        && camera.lastFrameTick > eoFrame_.simulationTick)
    {
        eoFrame_.width = eoWidth;
        eoFrame_.height = height;
        eoFrame_.simulationTick = camera.lastFrameTick;
        renderer_->ReadRgbPixels(observerWidth, 0, eoWidth, height, eoFrame_.pixels);
        eoFrame_.FlipVertical();
#ifndef __EMSCRIPTEN__
        const auto analysisStart = std::chrono::steady_clock::now();
        // OpenCV는 eoFrame_의 픽셀을 빌려 쓴다. 다음 프레임이 버퍼를 덮기 전에 분석을 끝낸다.
        const auto quality = visionAnalyzer_->Analyze(
            aerosimlab::vision::MakeRgbFrameView(eoFrame_));
        totalMeanBrightness_ += quality.meanBrightness;
        totalMeanContrast_ += quality.meanContrast;
        totalSharpness_ += quality.sharpness;
        ++visionSampleCount_;
        if (benchmarkMode_)
        {
            visionDurationsMicroseconds_.push_back(std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - analysisStart).count());
        }
#endif
    }
}

void AeroSimLabApp::PollGroundControl()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    if (!udpSocket_)
        return;

    for (int receivedCount = 0; receivedCount < 16; ++receivedCount)
    {
        aerosimlab::communication::UdpPeer peer;
        const auto datagram = udpSocket_->Receive(peer);
        if (!datagram)
            return;

        ProcessGroundDatagram(*datagram, peer);
    }
#endif
}

#ifndef __EMSCRIPTEN__
void AeroSimLabApp::ProcessGroundDatagram(const std::string& datagram,
    const aerosimlab::communication::UdpPeer& peer)
{
    try
    {
        const auto packet = aerosimlab::communication::DecodeCommand(datagram);
        if (activePeer_ && (activePeer_->addressNetworkOrder != peer.addressNetworkOrder
            || activePeer_->portNetworkOrder != peer.portNetworkOrder) && !pendingCommand_)
        {
            activePeer_ = peer;
            sequenceGuard_ = {};
            lastReply_.reset();
        }
        else if (!activePeer_)
        {
            activePeer_ = peer;
        }

        const auto decision = sequenceGuard_.Inspect(packet);
        if (decision == aerosimlab::communication::SequenceGuard::Decision::Duplicate)
        {
            if (lastReply_)
                udpSocket_->Send(aerosimlab::communication::EncodeReply(*lastReply_), peer);
            return;
        }
        if (decision == aerosimlab::communication::SequenceGuard::Decision::OutOfOrder)
        {
            aerosimlab::communication::CommandReply reply;
            reply.commandId = packet.commandId;
            reply.sequence = packet.sequence;
            reply.status = "rejected_sequence";
            reply.telemetry = CurrentTelemetry();
            udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
            return;
        }
        if (pendingCommand_)
        {
            aerosimlab::communication::CommandReply reply;
            reply.commandId = packet.commandId;
            reply.sequence = packet.sequence;
            reply.status = "busy";
            reply.telemetry = CurrentTelemetry();
            udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
            return;
        }

        sequenceGuard_.Commit(packet);
        externalControl_ = true;
        if (!packet.sessionCommand)
        {
            groundControlActive_ = true;
            window_->SetTitle("AeroSimLab | 지상통제 입력 사용 중");
        }
        pendingControllerAction_.reset();
        pendingPilotCommand_.reset();
        const auto elapsed = std::chrono::steady_clock::now() - startTime_;
        const auto receivedAtMs = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
        pendingCommand_ = PendingGroundCommand{packet, peer, receivedAtMs};
    }
    catch (const std::exception& error)
    {
        udpSocket_->Send(aerosimlab::communication::EncodeError(error.what()), peer);
    }
}
#endif

void AeroSimLabApp::PollGamepad()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    GLFWgamepadstate state{};
    const bool connected = glfwJoystickIsGamepad(GLFW_JOYSTICK_1) == GLFW_TRUE
        && glfwGetGamepadState(GLFW_JOYSTICK_1, &state) == GLFW_TRUE;

    if (!connected)
    {
        HandleGamepadDisconnected();
        return;
    }

    const bool takeoffPressed = state.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS;
    const bool landPressed = state.buttons[GLFW_GAMEPAD_BUTTON_B] == GLFW_PRESS;
    std::optional<aerosimlab::manual_control::GamepadAxes> pilotAxes;
    if (!groundControlActive_)
    {
        const auto mode = simulation_.State().mode;
        const bool airborne = mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying;
        if (airborne)
        {
            // GLFW 트리거 값은 -1부터 시작하므로 0~1로 바꾼 뒤 양쪽 입력 차이를 높이 명령으로 쓴다.
            const auto triggerValue = [](float value)
            {
                return (static_cast<double>(value) + 1.0) * 0.5;
            };
            pilotAxes = aerosimlab::manual_control::GamepadAxes{
                .forward = -static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]),
                .right = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_X]),
                .vertical = triggerValue(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER])
                    - triggerValue(state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]),
                .yaw = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X])
            };
        }
    }
    HandleGamepadConnected(takeoffPressed, landPressed, pilotAxes);
#endif
}

void AeroSimLabApp::HandleGamepadDisconnected()
{
    using DisconnectAction = aerosimlab::manual_control::GamepadDisconnectAction;
    const auto action = aerosimlab::manual_control::ResolveGamepadDisconnectAction(
        gamepadConnected_, groundControlActive_ || replaying_, simulation_.State().mode);
    if (action == DisconnectAction::NoChange)
        return;

    pendingControllerAction_.reset();
    window_->SetTitle(groundControlActive_
        ? "AeroSimLab | 지상통제 입력 사용 중"
        : "AeroSimLab | 드론 관찰 · EO 영상");
    switch (action)
    {
    case DisconnectAction::Land:
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
            aerosimlab::simulation::DroneCommand::Kind::Land};
        break;
    case DisconnectAction::Stop:
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
            aerosimlab::simulation::DroneCommand::Kind::Stop};
        break;
    case DisconnectAction::NoChange:
    case DisconnectAction::ClearPendingAction:
        break;
    }
    if (!groundControlActive_)
        pendingPilotCommand_.reset();
    gamepadConnected_ = false;
    previousTakeoffButton_ = false;
    previousLandButton_ = false;
}

void AeroSimLabApp::HandleGamepadConnected(bool takeoffPressed, bool landPressed,
    const std::optional<aerosimlab::manual_control::GamepadAxes>& pilotAxes)
{
    if (!gamepadConnected_ && !groundControlActive_)
        window_->SetTitle("AeroSimLab | 게임패드 연결됨");
    gamepadConnected_ = true;
    externalControl_ = true;

    if (aerosimlab::manual_control::CanGamepadControlSimulation(
            groundControlActive_, replaying_))
    {
        if (takeoffPressed && !previousTakeoffButton_
            && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded)
        {
            pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::TakeOff};
        }
        if (landPressed && !previousLandButton_
            && simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
        {
            pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::Land};
        }

        if (pilotAxes)
            pendingPilotCommand_ = aerosimlab::manual_control::MapAxes(
                *pilotAxes, simulation_.State().yawRadians);
    }
    else if (replaying_)
    {
        pendingControllerAction_.reset();
        pendingPilotCommand_.reset();
    }

    previousTakeoffButton_ = takeoffPressed;
    previousLandButton_ = landPressed;
}

bool AeroSimLabApp::ApplySessionCommand(
    aerosimlab::communication::SessionCommand command,
    aerosimlab::communication::CommandReply& reply)
{
    using Command = aerosimlab::communication::SessionCommand;
    using FlightMode = aerosimlab::simulation::FlightMode;

    switch (command)
    {
    case Command::StartRecording:
        if (recordingActive_ || replaying_)
        {
            reply.status = "recording_unavailable";
            return false;
        }
        if (simulation_.State().mode != FlightMode::Grounded)
        {
            reply.status = "recording_requires_grounded";
            return false;
        }

        simulation_.Reset();
        recordedCommands_.clear();
        recordingTickCount_ = 0;
        recordingActive_ = true;
        externalControl_ = true;
        window_->SetTitle("AeroSimLab | 입력 기록 중");
        reply.accepted = true;
        reply.status = "recording_started";
        return false;

    case Command::StopRecording:
        if (!recordingActive_)
        {
            reply.status = "recording_not_active";
            return false;
        }

        recordingActive_ = false;
        window_->SetTitle("AeroSimLab | 입력 기록 완료");
        reply.accepted = true;
        reply.status = "recording_stopped";
        return false;

    case Command::ReplayRecording:
        if (recordingActive_ || replaying_)
        {
            reply.status = "recording_unavailable";
            return false;
        }
        if (recordedCommands_.empty() || recordingTickCount_ == 0)
        {
            reply.status = "recording_empty";
            return false;
        }

        simulation_.Reset();
        replaying_ = true;
        smokeReplayStarted_ = smokeTest_;
        replayTick_ = 0;
        replayCommandIndex_ = 0;
        pendingControllerAction_.reset();
        pendingPilotCommand_.reset();
        window_->SetTitle("AeroSimLab | 입력 기록 재생 중");
        ApplyRecordedCommands();
        ++replayTick_;
        reply.accepted = true;
        reply.status = "recording_replay_started";
        return true;
    }

    return false;
}

void AeroSimLabApp::ApplyDroneCommand(
    const aerosimlab::simulation::DroneCommand& command,
    aerosimlab::communication::CommandReply& reply)
{
    if (replaying_)
    {
        replaying_ = false;
        window_->SetTitle("AeroSimLab | 지상통제 입력 사용 중");
    }

    const auto result = simulation_.Apply(command);
    reply.accepted = result == aerosimlab::simulation::CommandResult::Accepted;
    if (reply.accepted)
        RecordCommand(command);

    using Result = aerosimlab::simulation::CommandResult;
    switch (result)
    {
    case Result::Accepted: reply.status = "applied"; break;
    case Result::InvalidValue: reply.status = "invalid_value"; break;
    case Result::InvalidFlightMode: reply.status = "invalid_flight_mode"; break;
    case Result::UnsupportedDynamics: reply.status = "unsupported_dynamics"; break;
    case Result::SimulationPaused: reply.status = "simulation_paused"; break;
    }
}

void AeroSimLabApp::ApplyCameraCommand(
    aerosimlab::communication::CameraCommand command,
    aerosimlab::communication::CommandReply& reply)
{
    using Command = aerosimlab::communication::CameraCommand;
    switch (command)
    {
    case Command::Enable: simulation_.SetCameraEnabled(true); break;
    case Command::Disable: simulation_.SetCameraEnabled(false); break;
    case Command::InjectFault: simulation_.SetCameraFault(true); break;
    case Command::Recover: simulation_.SetCameraFault(false); break;
    }

    reply.accepted = true;
    reply.status = "applied";
}

void AeroSimLabApp::ApplyGroundCommand()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    if (!pendingCommand_)
        return;

    const auto& pending = *pendingCommand_;
    aerosimlab::communication::CommandReply reply;
    reply.commandId = pending.packet.commandId;
    reply.sequence = pending.packet.sequence;
    reply.receivedAtMilliseconds = pending.receivedAtMilliseconds;

    bool replayTickAdvanced = false;
    if (pending.packet.sessionCommand)
        replayTickAdvanced = ApplySessionCommand(*pending.packet.sessionCommand, reply);
    else if (pending.packet.droneCommand)
        ApplyDroneCommand(*pending.packet.droneCommand, reply);
    else if (pending.packet.cameraCommand)
        ApplyCameraCommand(*pending.packet.cameraCommand, reply);

    if (!replayTickAdvanced)
        simulation_.AdvanceOneTick();
    if (recordingActive_)
        ++recordingTickCount_;

    if (replaying_ && replayTick_ >= recordingTickCount_)
    {
        replaying_ = false;
        window_->SetTitle("AeroSimLab | 기록 재생 완료");
    }
    reply.appliedTick = simulation_.TickCount();
    reply.telemetry = CurrentTelemetry();
    lastReply_ = reply;
    try
    {
        udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), pending.peer);
    }
    catch (const std::exception& error)
    {
#ifdef _WIN32
        const std::string message = std::string("지상통제 응답 전송 실패: ") + error.what() + "\n";
        OutputDebugStringA(message.c_str());
#else
        (void)error;
#endif
    }
    pendingCommand_.reset();
#endif
}

void AeroSimLabApp::ApplyRecordedCommands()
{
    while (replayCommandIndex_ < recordedCommands_.size()
        && recordedCommands_[replayCommandIndex_].tick == replayTick_)
    {
        (void)simulation_.Apply(recordedCommands_[replayCommandIndex_].command);
        ++replayCommandIndex_;
    }
}

void AeroSimLabApp::RecordCommand(const aerosimlab::simulation::DroneCommand& command)
{
    if (recordingActive_)
        recordedCommands_.push_back({recordingTickCount_, command});
}

aerosimlab::communication::Telemetry AeroSimLabApp::CurrentTelemetry() const
{
    aerosimlab::communication::Telemetry telemetry;
    telemetry.tick = simulation_.TickCount();
    telemetry.simulationTimeSeconds = simulation_.SimulationTimeSeconds();
    telemetry.drone = simulation_.State();
    telemetry.camera = simulation_.Camera();
    return telemetry;
}

void AeroSimLabApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    renderer_->Draw(glm::mat4{1.0F}, *terrainMesh_, groundMaterial_, *shader_,
        view, projection, cameraPosition);
    DrawCube(CubeTransform({6.0F, 1.5F, -3.0F}, {2.0F, 3.0F, 2.0F}), cameraPosition,
        buildingMaterial_, view, projection);
    DrawCube(CubeTransform({-5.0F, 2.0F, -7.0F}, {3.0F, 4.0F, 2.0F}), cameraPosition,
        buildingMaterial_, view, projection);
    DrawCube(CubeTransform({12.0F, 2.5F, 5.0F}, {2.5F, 5.0F, 2.5F}), cameraPosition,
        buildingMaterial_, view, projection);

    const glm::vec3 position = ToRenderPosition(simulation_.State().positionMeters);
    const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
        glm::mat4_cast(ToRenderOrientation(simulation_.State().orientation));
    renderer_->Draw(heading, *droneMesh_, droneMaterial_, *shader_,
        view, projection, cameraPosition);
}

void AeroSimLabApp::DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
    const aerosimlab::graphics::Material& material,
    const glm::mat4& view, const glm::mat4& projection)
{
    renderer_->Draw(transform, *cube_, material, *shader_, view, projection, cameraPosition);
}

void AeroSimLabApp::WriteBenchmarkReport(double elapsedSeconds) const
{
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    window_->GetFramebufferSize(framebufferWidth, framebufferHeight);

    const auto glString = [](GLenum name)
    {
        return reinterpret_cast<const char*>(glGetString(name));
    };

    std::vector<double> durations = tickDurationsMicroseconds_;
    const std::size_t deadlineMisses = static_cast<std::size_t>(std::count_if(
        durations.begin(), durations.end(), [](double duration)
        {
            return duration > 1'000'000.0 / 60.0;
        }));
#ifndef __EMSCRIPTEN__
    std::vector<double> physicsStepDurations = physicsStepDurationsMicroseconds_;
    std::vector<double> visionDurations = visionDurationsMicroseconds_;
    std::sort(visionDurations.begin(), visionDurations.end());
#endif
    nlohmann::json report{
        {"configuration", "Windows Release; hidden OpenGL window; one drone; PX4 X500 visual mesh"},
        {"source_revision", AEROSIMLAB_SOURCE_REVISION},
        {"framebuffer", {{"width", framebufferWidth}, {"height", framebufferHeight}}},
        {"opengl", {{"renderer", glString(GL_RENDERER)}, {"version", glString(GL_VERSION)}}},
        {"elapsed_seconds", elapsedSeconds},
        {"simulation_ticks", simulation_.TickCount()},
        {"rendered_frames", renderedFrames_},
        {"render_fps", elapsedSeconds > 0.0 ? static_cast<double>(renderedFrames_) / elapsedSeconds : 0.0},
        {"tick_duration_us", SummarizeDurations(std::move(durations))},
        {"process_cpu_percent_approx", elapsedSeconds > 0.0
            ? 100.0 * static_cast<double>(std::clock() - benchmarkCpuStart_) / CLOCKS_PER_SEC / elapsedSeconds : 0.0}
    };
    report["tick_duration_us"]["deadline_misses_over_16_67ms"] = deadlineMisses;
#ifndef __EMSCRIPTEN__
    report["physics_step_duration_us"] = SummarizeDurations(std::move(physicsStepDurations));
    report["vision_analysis"] = {
        {"samples", visionSampleCount_},
        {"mean_brightness", visionSampleCount_ > 0
            ? totalMeanBrightness_ / static_cast<double>(visionSampleCount_) : 0.0},
        {"mean_contrast", visionSampleCount_ > 0
            ? totalMeanContrast_ / static_cast<double>(visionSampleCount_) : 0.0},
        {"mean_sharpness", visionSampleCount_ > 0
            ? totalSharpness_ / static_cast<double>(visionSampleCount_) : 0.0},
        {"duration_us", {
            {"p50", Percentile(visionDurations, 0.50)},
            {"p95", Percentile(visionDurations, 0.95)},
            {"p99", Percentile(visionDurations, 0.99)},
            {"samples", visionDurations.size()}}}
    };
#endif
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
        report["working_set_bytes"] = memory.WorkingSetSize;
#endif
#ifndef __EMSCRIPTEN__
    report["entity_scaling"] = MeasureDroneScaling();
#endif
    const std::string output = report.dump(2) + '\n';
    std::ofstream resultFile(BenchmarkOutputPath(), std::ios::binary | std::ios::trunc);
    if (!resultFile)
        throw std::runtime_error("성능 측정 결과 파일을 저장할 수 없습니다.");
    resultFile.write(output.data(), static_cast<std::streamsize>(output.size()));
    if (!resultFile)
        throw std::runtime_error("성능 측정 결과를 파일에 모두 쓰지 못했습니다.");
#ifdef _WIN32
    OutputDebugStringA(output.c_str());
#else
    std::cout << output;
#endif
}
}
