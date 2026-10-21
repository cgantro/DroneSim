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
#ifndef __EMSCRIPTEN__
#include "vision/RgbFrameQuality.h"
#include "vision/RgbFrameView.h"
#endif
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#ifndef _WIN32
#include <iostream>
#endif
#include <string>
#include <ctime>
#include <cmath>
#include <nlohmann/json.hpp>

#ifndef __EMSCRIPTEN__
#include <GLFW/glfw3.h>
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

AeroSimLabApp::AeroSimLabApp(Options options)
    : smokeTest_(options.smokeTest), demoMode_(options.demoMode),
      networkEnabled_(options.networkEnabled), benchmarkMode_(options.benchmarkMode),
      udpPort_(options.udpPort)
{
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
            ? fixedStepSeconds
            : std::chrono::duration<double>(now - previousFrame).count();
        previousFrame = now;

        AdvanceSimulation(std::min(frameSeconds, maximumFrameSeconds));
        RenderFrame();
        window_->SwapBuffers();
        ++renderedFrames_;

        if (smokeTest_ && !networkEnabled_ && renderedFrames_ >= smokeTestFrameLimit)
            break;
        if (smokeTest_ && networkEnabled_ && smokeCommandCount_ >= 4
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
    shader_ = std::make_unique<aerosimlab::graphics::Shader>("shaders/Scene.glsl");
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
        if (!hasGroundCommand && replaying_)
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
        else if (!hasGroundCommand)
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
}

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
        pendingControllerAction_.reset();
        if (gamepadConnected_)
        {
            window_->SetTitle(groundControlActive_
                ? "AeroSimLab | 지상통제 입력 사용 중"
                : "AeroSimLab | 드론 관찰 · EO 영상");
        }
        if (gamepadConnected_ && !groundControlActive_)
        {
            // 조종기가 끊기면 남아 있던 속도 명령을 지워 제자리 호버링으로 전환한다.
            const auto mode = simulation_.State().mode;
            if (mode == aerosimlab::simulation::FlightMode::TakingOff)
                pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                    aerosimlab::simulation::DroneCommand::Kind::Land};
            else if (mode == aerosimlab::simulation::FlightMode::Hovering
                || mode == aerosimlab::simulation::FlightMode::Flying)
                pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                    aerosimlab::simulation::DroneCommand::Kind::Stop};
            pendingPilotCommand_.reset();
        }
        gamepadConnected_ = false;
        previousTakeoffButton_ = false;
        previousLandButton_ = false;
        return;
    }

    if (!gamepadConnected_ && !groundControlActive_)
        window_->SetTitle("AeroSimLab | 게임패드 연결됨");
    gamepadConnected_ = true;
    externalControl_ = true;
    const bool takeoffPressed = state.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS;
    const bool landPressed = state.buttons[GLFW_GAMEPAD_BUTTON_B] == GLFW_PRESS;

    if (!groundControlActive_)
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

        const auto mode = simulation_.State().mode;
        const bool airborne = mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying;
        if (airborne)
        {
            const auto triggerValue = [](float value)
            {
                return (static_cast<double>(value) + 1.0) * 0.5;
            };
            const aerosimlab::manual_control::GamepadAxes axes{
                .forward = -static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]),
                .right = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_X]),
                .vertical = triggerValue(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER])
                    - triggerValue(state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]),
                .yaw = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X])
            };
            pendingPilotCommand_ = aerosimlab::manual_control::MapAxes(
                axes, simulation_.State().yawRadians);
        }
    }

    previousTakeoffButton_ = takeoffPressed;
    previousLandButton_ = landPressed;
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

    bool replayTickAdvanced = false;
    if (pending.packet.sessionCommand)
    {
        using Command = aerosimlab::communication::SessionCommand;
        switch (*pending.packet.sessionCommand)
        {
        case Command::StartRecording:
            if (recordingActive_ || replaying_)
                reply.status = "recording_unavailable";
            else if (simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
                reply.status = "recording_requires_grounded";
            else
            {
                simulation_.Reset();
                recordedCommands_.clear();
                recordingTickCount_ = 0;
                recordingActive_ = true;
                externalControl_ = true;
                window_->SetTitle("AeroSimLab | 입력 기록 중");
                reply.accepted = true;
                reply.status = "recording_started";
            }
            break;
        case Command::StopRecording:
            if (!recordingActive_)
                reply.status = "recording_not_active";
            else
            {
                recordingActive_ = false;
                window_->SetTitle("AeroSimLab | 입력 기록 완료");
                reply.accepted = true;
                reply.status = "recording_stopped";
            }
            break;
        case Command::ReplayRecording:
            if (recordingActive_ || replaying_)
                reply.status = "recording_unavailable";
            else if (recordedCommands_.empty() || recordingTickCount_ == 0)
                reply.status = "recording_empty";
            else
            {
                simulation_.Reset();
                replaying_ = true;
                replayTick_ = 0;
                replayCommandIndex_ = 0;
                pendingControllerAction_.reset();
                pendingPilotCommand_.reset();
                window_->SetTitle("AeroSimLab | 입력 기록 재생 중");
                ApplyRecordedCommands();
                ++replayTick_;
                replayTickAdvanced = true;
                reply.accepted = true;
                reply.status = "recording_replay_started";
            }
            break;
        }
    }
    else if (pending.packet.droneCommand)
    {
        if (replaying_)
        {
            replaying_ = false;
            window_->SetTitle("AeroSimLab | 지상통제 입력 사용 중");
        }
        const auto result = simulation_.Apply(*pending.packet.droneCommand);
        reply.accepted = result == aerosimlab::simulation::CommandResult::Accepted;
        if (reply.accepted)
            RecordCommand(*pending.packet.droneCommand);
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
    ++smokeCommandCount_;
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
                              const aerosimlab::graphics::Material& material)
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
#ifndef __EMSCRIPTEN__
    std::vector<double> visionDurations = visionDurationsMicroseconds_;
    std::sort(visionDurations.begin(), visionDurations.end());
    const auto visionPercentile = [&visionDurations](double fraction)
    {
        if (visionDurations.empty())
            return 0.0;
        const std::size_t index = static_cast<std::size_t>(
            std::ceil(fraction * static_cast<double>(visionDurations.size()))) - 1;
        return visionDurations[std::min(index, visionDurations.size() - 1)];
    };
#endif
    nlohmann::json report{
        {"configuration", "Windows Release; hidden OpenGL window; one drone; simple geometry"},
        {"source_revision", AEROSIMLAB_SOURCE_REVISION},
        {"framebuffer", {{"width", framebufferWidth}, {"height", framebufferHeight}}},
        {"opengl", {{"renderer", glString(GL_RENDERER)}, {"version", glString(GL_VERSION)}}},
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
#ifndef __EMSCRIPTEN__
    report["vision_analysis"] = {
        {"samples", visionSampleCount_},
        {"mean_brightness", visionSampleCount_ > 0
            ? totalMeanBrightness_ / static_cast<double>(visionSampleCount_) : 0.0},
        {"mean_contrast", visionSampleCount_ > 0
            ? totalMeanContrast_ / static_cast<double>(visionSampleCount_) : 0.0},
        {"mean_sharpness", visionSampleCount_ > 0
            ? totalSharpness_ / static_cast<double>(visionSampleCount_) : 0.0},
        {"duration_us", {
            {"p50", visionPercentile(0.50)},
            {"p95", visionPercentile(0.95)},
            {"p99", visionPercentile(0.99)},
            {"samples", visionDurations.size()}}}
    };
#endif
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
        report["working_set_bytes"] = memory.WorkingSetSize;
#endif
    const std::string output = report.dump(2) + '\n';
#ifdef _WIN32
    OutputDebugStringA(output.c_str());
#else
    std::cout << output;
#endif
}
}
