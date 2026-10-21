#include "AeroSimLabApp.h"

#include "graphics/Camera.h"
#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <iostream>
#include <string>
#include <ctime>
#include <cmath>
#include <nlohmann/json.hpp>

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#include <Psapi.h>
#endif

namespace aerosimlab::simulator
{
namespace
{
constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
constexpr double maximumFrameSeconds = 0.1;
constexpr double fixedStepSeconds = 1.0 / 60.0;
constexpr std::size_t smokeTestFrameLimit = 8;
constexpr std::size_t demoLengthTicks = 900;
constexpr double benchmarkDurationSeconds = 5.0;

glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    // 모의는 Z축을 위로 쓰고 화면은 Y축을 위로 쓴다. Y축 방향은 화면의 -Z로 옮겨 오른손 좌표계를 유지한다.
    return {static_cast<float>(positionMeters.x), static_cast<float>(positionMeters.z),
        static_cast<float>(-positionMeters.y)};
}

glm::mat4 CubeTransform(const glm::vec3& position, const glm::vec3& size)
{
    return glm::translate(glm::mat4{1.0F}, position) * glm::scale(glm::mat4{1.0F}, size);
}
}

AeroSimLabApp::AeroSimLabApp(bool smokeTest, std::uint16_t udpPort, bool demoMode,
    bool networkEnabled, bool benchmarkMode)
    : smokeTest_(smokeTest), demoMode_(demoMode), networkEnabled_(networkEnabled),
      benchmarkMode_(benchmarkMode), udpPort_(udpPort)
{
}

AeroSimLabApp::~AeroSimLabApp() = default;

int AeroSimLabApp::Run()
{
    InitializeGraphics();
#ifndef __EMSCRIPTEN__
    if (networkEnabled_)
    {
        udpSocket_ = std::make_unique<aerosimlab::communication::UdpSocket>(udpPort_);
        std::cout << "지상통제 UDP 대기 중: 0.0.0.0:" << udpPort_ << '\n';
    }
#endif
    startTime_ = std::chrono::steady_clock::now();
    benchmarkCpuStart_ = std::clock();
    auto previousFrame = startTime_;

    while (!window_->ShouldClose())
    {
        window_->PollEvents();
        PollGroundControl();
        const auto now = std::chrono::steady_clock::now();
        const double frameSeconds = smokeTest_ && !benchmarkMode_
            ? fixedStepSeconds
            : std::chrono::duration<double>(now - previousFrame).count();
        previousFrame = now;

        AdvanceSimulation(std::min(frameSeconds, maximumFrameSeconds));
        RenderFrame();
        window_->SwapBuffers();
        ++renderedFrames_;

        if (smokeTest_ && !networkEnabled_ && renderedFrames_ >= smokeTestFrameLimit)
            break;
        if (smokeTest_ && networkEnabled_ && smokeCommandCount_ >= 3)
            break;
        if (smokeTest_ && networkEnabled_ &&
            std::chrono::steady_clock::now() - startTime_ > std::chrono::seconds(5))
            return 4;
        if (benchmarkMode_ &&
            std::chrono::steady_clock::now() - startTime_ >= std::chrono::duration<double>(benchmarkDurationSeconds))
            break;
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

    cube_ = std::shared_ptr<aerosimlab::graphics::Mesh>(
        aerosimlab::graphics::Mesh::CreateCube(1.0F));
    shader_ = aerosimlab::graphics::Shader::Create("shaders/Scene.glsl");
    droneMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.12F, 0.48F, 0.32F, 1.0F}, 0.0F, 0.75F);
    propellerMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.08F, 0.09F, 0.10F, 1.0F}, 0.0F, 0.9F);
    groundMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.35F, 0.39F, 0.34F, 1.0F}, 0.0F, 1.0F);
    buildingMaterial_ = std::make_shared<aerosimlab::graphics::Material>(
        glm::vec4{0.63F, 0.59F, 0.48F, 1.0F}, 0.0F, 0.9F);
}

void AeroSimLabApp::AdvanceSimulation(double elapsedSeconds)
{
    accumulatedSeconds_ += elapsedSeconds;
    while (accumulatedSeconds_ >= fixedStepSeconds)
    {
        const auto tickStart = std::chrono::steady_clock::now();
        if (demoMode_ && !externalControl_)
            ApplyDemoCommands();
#ifndef __EMSCRIPTEN__
        const bool hasGroundCommand = pendingCommand_.has_value();
#else
        constexpr bool hasGroundCommand = false;
#endif
        ApplyGroundCommand();
        if (!hasGroundCommand)
            simulation_.AdvanceOneTick();
        if (benchmarkMode_)
        {
            tickDurationsMicroseconds_.push_back(std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - tickStart).count());
        }
        accumulatedSeconds_ -= fixedStepSeconds;
    }
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
    const auto& state = simulation_.State();
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const glm::mat4 heading = glm::rotate(glm::mat4{1.0F},
        static_cast<float>(state.yawRadians), glm::vec3{0.0F, 1.0F, 0.0F});

    observerCamera_->SetPosition(dronePosition + glm::vec3{7.0F, 5.0F, 7.0F});
    observerCamera_->SetTarget(dronePosition + glm::vec3{0.0F, 0.2F, 0.0F});
    observerCamera_->SetAspectRatio(static_cast<float>(observerWidth) / static_cast<float>(height));

    const glm::vec3 cameraPosition = dronePosition + glm::vec3{
        heading * glm::vec4{0.35F, 0.12F, 0.0F, 0.0F}};
    const glm::vec3 forward = glm::vec3{heading * glm::vec4{1.0F, 0.0F, 0.0F, 0.0F}};
    eoCamera_->SetPosition(cameraPosition);
    eoCamera_->SetTarget(cameraPosition + forward * 25.0F + glm::vec3{0.0F, -2.5F, 0.0F});
    eoCamera_->SetAspectRatio(static_cast<float>(eoWidth) / static_cast<float>(height));

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

        try
        {
            const auto packet = aerosimlab::communication::DecodeCommand(*datagram);
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
                continue;
            }
            if (decision == aerosimlab::communication::SequenceGuard::Decision::OutOfOrder)
            {
                aerosimlab::communication::CommandReply reply;
                reply.commandId = packet.commandId;
                reply.sequence = packet.sequence;
                reply.status = "rejected_sequence";
                reply.telemetry = CurrentTelemetry();
                udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
                continue;
            }
            if (pendingCommand_)
            {
                aerosimlab::communication::CommandReply reply;
                reply.commandId = packet.commandId;
                reply.sequence = packet.sequence;
                reply.status = "busy";
                reply.telemetry = CurrentTelemetry();
                udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
                continue;
            }

            sequenceGuard_.Commit(packet);
            externalControl_ = true;
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

    if (pending.packet.droneCommand)
    {
        const auto result = simulation_.Apply(*pending.packet.droneCommand);
        reply.accepted = result == aerosimlab::simulation::CommandResult::Accepted;
        switch (result)
        {
        case aerosimlab::simulation::CommandResult::Accepted: reply.status = "applied"; break;
        case aerosimlab::simulation::CommandResult::InvalidValue: reply.status = "invalid_value"; break;
        case aerosimlab::simulation::CommandResult::InvalidFlightMode: reply.status = "invalid_flight_mode"; break;
        case aerosimlab::simulation::CommandResult::SimulationPaused: reply.status = "simulation_paused"; break;
        }
    }
    else if (pending.packet.cameraCommand)
    {
        using Command = aerosimlab::communication::CameraCommand;
        switch (*pending.packet.cameraCommand)
        {
        case Command::Enable: simulation_.SetCameraEnabled(true); break;
        case Command::Disable: simulation_.SetCameraEnabled(false); break;
        case Command::InjectFault: simulation_.SetCameraFault(true); break;
        case Command::Recover: simulation_.SetCameraFault(false); break;
        }
        reply.accepted = true;
        reply.status = "applied";
    }

    // 명령을 적용한 뒤 한 고정 Tick을 진행해 실제 적용 Tick을 응답에 담는다.
    simulation_.AdvanceOneTick();
    reply.appliedTick = simulation_.TickCount();
    reply.telemetry = CurrentTelemetry();
    lastReply_ = reply;
    try
    {
        udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), pending.peer);
    }
    catch (const std::exception& error)
    {
        std::cerr << "지상통제 응답 전송 실패: " << error.what() << '\n';
    }
    pendingCommand_.reset();
    ++smokeCommandCount_;
#endif
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
    DrawCube(CubeTransform({0.0F, -0.2F, 0.0F}, {80.0F, 0.2F, 80.0F}), cameraPosition,
        groundMaterial_, view, projection);
    DrawCube(CubeTransform({6.0F, 1.5F, -3.0F}, {2.0F, 3.0F, 2.0F}), cameraPosition,
        buildingMaterial_, view, projection);
    DrawCube(CubeTransform({-5.0F, 2.0F, -7.0F}, {3.0F, 4.0F, 2.0F}), cameraPosition,
        buildingMaterial_, view, projection);
    DrawCube(CubeTransform({12.0F, 2.5F, 5.0F}, {2.5F, 5.0F, 2.5F}), cameraPosition,
        buildingMaterial_, view, projection);

    const glm::vec3 position = ToRenderPosition(simulation_.State().positionMeters);
    const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
        glm::rotate(glm::mat4{1.0F}, static_cast<float>(simulation_.State().yawRadians),
            glm::vec3{0.0F, 1.0F, 0.0F});
    const auto drawPart = [&](const glm::vec3& offset, const glm::vec3& size,
                              const std::shared_ptr<aerosimlab::graphics::Material>& material)
    {
        DrawCube(heading * glm::translate(glm::mat4{1.0F}, offset) *
            glm::scale(glm::mat4{1.0F}, size), cameraPosition, material, view, projection);
    };

    drawPart({0.0F, 0.0F, 0.0F}, {0.55F, 0.16F, 0.42F}, droneMaterial_);
    drawPart({0.0F, 0.0F, -0.35F}, {0.50F, 0.05F, 0.05F}, droneMaterial_);
    drawPart({0.0F, 0.0F, 0.35F}, {0.50F, 0.05F, 0.05F}, droneMaterial_);
    drawPart({-0.32F, 0.08F, -0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
    drawPart({0.32F, 0.08F, -0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
    drawPart({-0.32F, 0.08F, 0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
    drawPart({0.32F, 0.08F, 0.32F}, {0.18F, 0.04F, 0.18F}, propellerMaterial_);
}

void AeroSimLabApp::DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
    const std::shared_ptr<aerosimlab::graphics::Material>& material,
    const glm::mat4& view, const glm::mat4& projection)
{
    renderer_->Draw(transform, *cube_, *material, *shader_, view, projection, cameraPosition);
}

void AeroSimLabApp::WriteBenchmarkReport(double elapsedSeconds) const
{
    std::vector<double> durations = tickDurationsMicroseconds_;
    std::sort(durations.begin(), durations.end());
    const auto percentile = [&durations](double fraction)
    {
        if (durations.empty())
            return 0.0;
        const std::size_t index = static_cast<std::size_t>(
            std::ceil(fraction * static_cast<double>(durations.size()))) - 1;
        return durations[std::min(index, durations.size() - 1)];
    };
    const std::size_t deadlineMisses = static_cast<std::size_t>(std::count_if(
        durations.begin(), durations.end(), [](double duration)
        {
            return duration > 1'000'000.0 / 60.0;
        }));
    nlohmann::json report{
        {"configuration", "Windows Release; hidden OpenGL window; one drone; simple geometry"},
        {"elapsed_seconds", elapsedSeconds},
        {"simulation_ticks", simulation_.TickCount()},
        {"rendered_frames", renderedFrames_},
        {"render_fps", elapsedSeconds > 0.0 ? static_cast<double>(renderedFrames_) / elapsedSeconds : 0.0},
        {"tick_duration_us", {
            {"p50", percentile(0.50)}, {"p95", percentile(0.95)}, {"p99", percentile(0.99)},
            {"samples", durations.size()}, {"deadline_misses_over_16_67ms", deadlineMisses}}},
        {"process_cpu_percent_approx", elapsedSeconds > 0.0
            ? 100.0 * static_cast<double>(std::clock() - benchmarkCpuStart_) / CLOCKS_PER_SEC / elapsedSeconds : 0.0}
    };
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
        report["working_set_bytes"] = memory.WorkingSetSize;
#endif
    std::cout << report.dump(2) << '\n';
}
}
