#include "AeroSimLabApp.h"
#include "ExecutableDirectory.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include "graphics/Camera.h"
#include "graphics/Mesh.h"
#ifndef __EMSCRIPTEN__
#include "graphics/MultisampleFramebuffer.h"
#endif
#include "graphics/Renderer.h"
#include "graphics/Texture2D.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#ifndef __EMSCRIPTEN__
#include "SimulatorHud.h"
#endif
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/ScopedDuration.h"
#include "terrain/TerrainGrid.h"
#include <glm/gtc/quaternion.hpp>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#ifndef __EMSCRIPTEN__
#include "vision/RgbFrameQuality.h"
#endif
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <ctime>
#include <cmath>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
namespace
{
constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
// 창이 멈춘 뒤 밀린 시간을 전부 따라잡느라 다음 화면까지 늦어지지 않게 한다.
constexpr std::size_t smokeTestFrameLimit = 8;
// 소프트웨어 렌더러에서도 네트워크 스모크의 모의가 화면 속도에 묶이지 않게 한다.
constexpr std::size_t networkSmokeSimulationBatchesPerFrame = 5;
constexpr std::size_t demoLengthTicks = 900;
constexpr glm::dvec2 demoDroneStartMeters{0.0, -9.0};
constexpr double benchmarkDurationSeconds = 5.0;

std::filesystem::path AssetPath(std::string_view relativePath)
{
    return detail::ExecutableDirectory() / relativePath;
}

glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    // 모의는 Z축을 위로 쓰고 화면은 Y축을 위로 쓴다. Y축 방향은 화면의 -Z로 옮겨 오른손 좌표계를 유지한다.
    return glm::vec3{
        static_cast<float>(positionMeters.x), static_cast<float>(positionMeters.z),
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

#ifdef __EMSCRIPTEN__
/**
 * @brief 웹 화면에서 빠르게 그릴 간단한 쿼드콥터 모형을 만든다.
 * @details 상세한 기체 GLB 대신 몸체·팔·모터·프로펠러를 상자 13개로 묶어 그린다.
 */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateWebDroneMesh()
{
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;
    constexpr std::array<std::array<std::uint32_t, 4>, 6> faces{{
        {{0U, 1U, 2U, 3U}}, {{5U, 4U, 7U, 6U}},
        {{4U, 0U, 3U, 7U}}, {{1U, 5U, 6U, 2U}},
        {{3U, 2U, 6U, 7U}}, {{4U, 5U, 1U, 0U}}
    }};
    const std::array<glm::vec3, 8> corners{{
        {-0.5F, -0.5F, -0.5F}, {0.5F, -0.5F, -0.5F},
        {0.5F, 0.5F, -0.5F}, {-0.5F, 0.5F, -0.5F},
        {-0.5F, -0.5F, 0.5F}, {0.5F, -0.5F, 0.5F},
        {0.5F, 0.5F, 0.5F}, {-0.5F, 0.5F, 0.5F}
    }};
    const auto addBox = [&](const glm::vec3& center, const glm::vec3& size, float angle)
    {
        const glm::mat4 rotation = glm::rotate(
            glm::mat4{1.0F}, angle, glm::vec3{0.0F, 1.0F, 0.0F});
        const std::uint32_t first = static_cast<std::uint32_t>(vertices.size());
        for (const auto& face : faces)
        {
            const glm::vec3 a = corners[face[0]] * size;
            const glm::vec3 b = corners[face[1]] * size;
            const glm::vec3 c = corners[face[2]] * size;
            const glm::vec3 normal = glm::normalize(glm::cross(c - a, b - a));
            for (const std::uint32_t corner : face)
            {
                const glm::vec3 local = corners[corner] * size;
                vertices.push_back({
                    center + glm::vec3(rotation * glm::vec4{local, 0.0F}),
                    glm::normalize(glm::vec3(rotation * glm::vec4{normal, 0.0F})),
                    {0.0F, 0.0F}});
            }
        }
        for (std::uint32_t face = 0; face < 6U; ++face)
        {
            const std::uint32_t start = first + face * 4U;
            indices.insert(indices.end(), {start, start + 2U, start + 1U,
                start, start + 3U, start + 2U});
        }
    };

    addBox({}, {0.42F, 0.12F, 0.28F}, 0.0F);
    constexpr std::array<float, 2> sides{-1.0F, 1.0F};
    for (const float xSide : sides)
    {
        for (const float zSide : sides)
        {
            const float angle = std::atan2(zSide, xSide);
            addBox({xSide * 0.20F, 0.0F, zSide * 0.20F}, {0.52F, 0.035F, 0.035F}, angle);
            addBox({xSide * 0.38F, 0.0F, zSide * 0.38F}, {0.14F, 0.055F, 0.14F}, 0.0F);
            addBox({xSide * 0.38F, 0.04F, zSide * 0.38F}, {0.30F, 0.012F, 0.025F}, 0.0F);
        }
    }
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}
#endif

glm::mat4 IndoorBoxTransform(const glm::dvec3& centerMeters,
    const glm::dvec3& sizeMeters)
{
    return CubeTransform(ToRenderPosition(centerMeters),
        glm::vec3{static_cast<float>(sizeMeters.x),
            static_cast<float>(sizeMeters.z),
            static_cast<float>(sizeMeters.y)});
}
}

AeroSimLabApp::AeroSimLabApp(Options options)
    : smokeTest_(options.smokeTest), demoMode_(options.demoMode),
      networkEnabled_(options.networkEnabled),
      benchmarkMode_(options.benchmarkMode || options.visibleBenchmarkMode),
      visibleBenchmarkMode_(options.visibleBenchmarkMode),
      groundControlCommandTimeoutTicks_(options.groundControlCommandTimeoutTicks),
      udpPort_(options.udpPort)
{
    if (groundControlCommandTimeoutTicks_ == 0)
        throw std::invalid_argument("지상통제 명령 유효기간은 1 Tick 이상이어야 합니다.");
    if (benchmarkMode_)
        simulation_.SetPhysicsStepTimingEnabled(true);
#ifndef __EMSCRIPTEN__
    visionAnalyzer_ = std::make_unique<aerosimlab::vision::RgbFrameAnalyzer>();
#endif
}

AeroSimLabApp::~AeroSimLabApp() = default;

int AeroSimLabApp::Run()
{
#ifndef __EMSCRIPTEN__
    if (networkEnabled_)
    {
        udpSocket_ = std::make_unique<aerosimlab::communication::UdpSocket>(udpPort_);
    }
#endif
    // 그래픽 자산을 불러오는 동안 도착한 UDP 요청도 소켓 수신 큐에 남긴다.
    InitializeGraphics();
    startTime_ = std::chrono::steady_clock::now();
#ifdef __EMSCRIPTEN__
    webPreviousFrame_ = startTime_;
    emscripten_set_main_loop_arg([](void* application)
    {
        auto& app = *static_cast<AeroSimLabApp*>(application);
        if (app.window_->ShouldClose())
        {
            emscripten_cancel_main_loop();
            return;
        }
        app.RunFrame(app.webPreviousFrame_);
    }, this, 0, false);
    return 0;
#else
    benchmarkCpuStart_ = std::clock();
    auto previousFrame = startTime_;

    if (benchmarkMode_)
    {
        while (!window_->ShouldClose()
            && std::chrono::steady_clock::now() - startTime_
                < std::chrono::duration<double>(benchmarkWarmupDurationSeconds))
        {
            RunFrame(previousFrame);
        }

        // 초기 셰이더·버퍼 생성 지연은 측정 구간에서 빼고, 워밍업 뒤 기준값부터 기록한다.
        benchmarkMeasurements_ = {};
        benchmarkStartRenderedFrames_ = renderedFrames_;
        benchmarkStartSimulationTicks_ = simulation_.TickCount();
        benchmarkCpuStart_ = std::clock();
        startTime_ = std::chrono::steady_clock::now();
        previousFrame = startTime_;
    }

    while (!window_->ShouldClose())
    {
        RunFrame(previousFrame);

        if (smokeTest_ && SmokeTestIsComplete())
            break;
        if (smokeTest_ && networkEnabled_
            && std::chrono::steady_clock::now() - startTime_ > std::chrono::seconds(30))
            return 4;
        if (benchmarkMode_ &&
            std::chrono::steady_clock::now() - startTime_ >= std::chrono::duration<double>(benchmarkDurationSeconds))
            break;
    }
#ifndef __EMSCRIPTEN__
    if (smokeTest_)
        DrainEoReadbacks();
#endif
    if (smokeTest_)
    {
        const int smokeTestResult = ValidateSmokeTestOutput();
        if (smokeTestResult != 0)
            return smokeTestResult;
    }
    if (benchmarkMode_)
    {
        const double elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime_).count();
        WriteBenchmarkReport(elapsedSeconds);
    }
    return 0;
#endif
}

void AeroSimLabApp::RunFrame(std::chrono::steady_clock::time_point& previousFrame)
{
    const auto applicationFrameStart = benchmarkMode_
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    frameMeasurements_ = {};
    {
        aerosimlab::simulation::ScopedDuration measureEvents(
            benchmarkMode_ ? &frameMeasurements_.eventPollingMicroseconds : nullptr);
        window_->PollEvents();
    }
    {
        aerosimlab::simulation::ScopedDuration measureGroundControl(
            benchmarkMode_ ? &frameMeasurements_.groundControlPollingMicroseconds : nullptr);
        PollGroundControl();
    }
    {
        aerosimlab::simulation::ScopedDuration measureGamepad(
            benchmarkMode_ ? &frameMeasurements_.gamepadPollingMicroseconds : nullptr);
        PollGamepad();
    }
    const auto now = std::chrono::steady_clock::now();
    const double elapsedSeconds = std::chrono::duration<double>(now - previousFrame).count();
    // 화면 스모크는 프레임마다 한 Tick, 네트워크 스모크는 프레임마다 최대 따라잡기만큼 진행한다.
    double frameSeconds = elapsedSeconds;
    if (smokeTest_ && !benchmarkMode_)
    {
        frameSeconds = networkEnabled_
            ? aerosimlab::simulation::FixedTickAccumulator::maximumFrameSeconds
            : aerosimlab::simulation::FixedTickAccumulator::fixedStepSeconds;
    }
    previousFrame = now;

    const std::size_t simulationBatches = smokeTest_ && networkEnabled_
        && !benchmarkMode_
        ? networkSmokeSimulationBatchesPerFrame
        : 1U;
    {
        aerosimlab::simulation::ScopedDuration measureSimulation(
            benchmarkMode_ ? &frameMeasurements_.simulationUpdateMicroseconds : nullptr);
        for (std::size_t batch = 0; batch < simulationBatches; ++batch)
            AdvanceSimulation(frameSeconds);
    }
    eoReadbackHappenedThisFrame_ = false;
    {
        aerosimlab::simulation::ScopedDuration measureRenderFrame(
            benchmarkMode_ ? &frameMeasurements_.renderFrameMicroseconds : nullptr);
        {
            aerosimlab::simulation::ScopedDuration measureRender(
                benchmarkMode_ ? &frameMeasurements_.renderMicroseconds : nullptr);
            RenderFrame();
        }
        {
            aerosimlab::simulation::ScopedDuration measureBufferSwap(
                benchmarkMode_ ? &frameMeasurements_.bufferSwapMicroseconds : nullptr);
            window_->SwapBuffers();
        }
    }
    ++renderedFrames_;
    if (benchmarkMode_)
    {
        const double applicationMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - applicationFrameStart).count();
        RecordBenchmarkFrame(applicationMicroseconds);
    }
}

bool AeroSimLabApp::SmokeTestIsComplete() const
{
    if (!networkEnabled_)
        return renderedFrames_ >= smokeTestFrameLimit;

    return smokeReplayStarted_
        && !commandRecording_.IsRecording()
        && !commandRecording_.IsReplaying()
        && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded;
}

int AeroSimLabApp::ValidateSmokeTestOutput() const
{
    const bool cameraFrameHasPixels = eoFrame_.IsValid()
        && std::any_of(eoFrame_.pixels.begin(), eoFrame_.pixels.end(), [](std::uint8_t value)
        {
            return value != 0;
        });
    if (!cameraFrameHasPixels)
        return 5;

    const auto& camera = simulation_.Camera();
    if (camera.status == aerosimlab::simulation::CameraStatus::Active
        && eoFrame_.simulationTick != camera.lastFrameTick)
    {
        std::cerr << "EO 영상 Tick이 카메라의 마지막 Tick과 다릅니다. 영상="
                  << eoFrame_.simulationTick << ", 카메라="
                  << camera.lastFrameTick << '\n';
        return 6;
    }

    return 0;
}

void AeroSimLabApp::InitializeGraphics()
{
    window_ = std::make_unique<aerosimlab::graphics::Window>(
        aerosimlab::graphics::Window::Properties{windowWidth, windowHeight,
            "AeroSimLab | 드론 관찰 · EO 영상 | © OpenStreetMap contributors",
            !smokeTest_ && !benchmarkMode_,
            !smokeTest_ && (!benchmarkMode_ || visibleBenchmarkMode_)});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    window_->GetFramebufferSize(framebufferWidth, framebufferHeight);

    renderer_ = std::make_unique<aerosimlab::graphics::Renderer>();
    renderer_->Init(framebufferWidth, framebufferHeight);
#ifndef __EMSCRIPTEN__
    sceneTarget_ = std::make_unique<aerosimlab::graphics::MultisampleFramebuffer>(
        framebufferWidth, framebufferHeight);
    cameraTarget_ = std::make_unique<aerosimlab::graphics::MultisampleFramebuffer>(
        framebufferWidth, framebufferHeight);
#endif
    observerCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{7.0F, 5.0F, 7.0F}, glm::vec3{0.0F}, 1.0F,
        45.0F, 0.1F, 120.0F);
    eoCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F, 70.0F, 0.05F, 200.0F);

    LoadScene();
    shader_ = std::make_unique<aerosimlab::graphics::Shader>(
        AssetPath("shaders/Scene.glsl").string());
#ifndef __EMSCRIPTEN__
    hud_ = std::make_unique<SimulatorHud>(window_->GetNativeHandle());
#endif
}

void AeroSimLabApp::LoadScene()
{
    cube_ = aerosimlab::graphics::Mesh::CreateCube(1.0F);
#ifdef __EMSCRIPTEN__
    droneMesh_ = CreateWebDroneMesh();
#else
    droneMesh_ = aerosimlab::graphics::Mesh::LoadGlb(
        AssetPath("models/drones/x500/visual.glb").string());
#endif
#ifndef __EMSCRIPTEN__
    droneSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadPpm(
        AssetPath("models/drones/x500/carbon-weave.ppm"));
#endif

    // 실내 장면의 바닥은 높이 차가 없는 작은 격자로 표현해 강체 충돌과 맞춘다.
    terrainGrid_.sampleCount = 3;
    terrainGrid_.spacingXMeters = 20.0F;
    terrainGrid_.spacingYMeters = 15.0F;
    terrainGrid_.referenceElevationMeters = 0.0F;
    terrainGrid_.heightOffsetsMeters.assign(9U, 0.0F);
    const glm::dvec3 floorSizeMeters{40.0, 30.0, 0.3};
    floorTransform_ = IndoorBoxTransform({0.0, 0.0, -0.15}, floorSizeMeters);

    const auto addWall = [this](const glm::dvec3& center, const glm::dvec3& halfExtents)
    {
        wallTransforms_.push_back(IndoorBoxTransform(center, halfExtents * 2.0));
#ifndef __EMSCRIPTEN__
        simulation_.AddStaticObstacleBox(center, glm::vec3{halfExtents});
#endif
    };
    const auto addDebris = [this](const glm::dvec3& center, const glm::dvec3& size)
    {
        debrisTransforms_.push_back(IndoorBoxTransform(center, size));
#ifndef __EMSCRIPTEN__
        simulation_.AddStaticObstacleBox(center, glm::vec3{size * 0.5});
#endif
    };
    const auto addPerson = [this](const glm::dvec2& position)
    {
        const glm::dvec3 torsoCenter{position.x, position.y, 0.8};
        targetTransforms_.push_back(IndoorBoxTransform(torsoCenter, {0.45, 0.3, 0.8}));
        targetTransforms_.push_back(IndoorBoxTransform(
            {position.x, position.y, 1.35}, {0.34, 0.34, 0.34}));
#ifndef __EMSCRIPTEN__
        simulation_.AddStaticObstacleBox(torsoCenter, {0.225F, 0.15F, 0.4F});
#endif
    };

    // 외벽과 칸막이에 문틈을 남겨 드론이 방 사이를 오갈 수 있게 한다.
    addWall({-15.0, 0.0, 2.0}, {0.2, 11.0, 2.0});
    addWall({15.0, 0.0, 2.0}, {0.2, 11.0, 2.0});
    addWall({0.0, -11.0, 2.0}, {15.0, 0.2, 2.0});
    addWall({0.0, 11.0, 2.0}, {15.0, 0.2, 2.0});
    addWall({-5.0, -6.5, 2.0}, {0.2, 4.5, 2.0});
    addWall({-5.0, 6.0, 2.0}, {0.2, 5.0, 2.0});
    addWall({5.0, -7.0, 2.0}, {0.2, 4.0, 2.0});
    addWall({5.0, 5.5, 2.0}, {0.2, 5.5, 2.0});
    addWall({-11.5, 0.0, 2.0}, {3.5, 0.2, 2.0});
    addWall({0.0, 0.0, 2.0}, {2.0, 0.2, 2.0});
    addWall({11.5, 0.0, 2.0}, {3.5, 0.2, 2.0});

    // 서로 다른 크기의 상자로 잔해를 만들고, 두 인명 표식을 각 방에 둔다.
    addDebris({-10.0, 6.5, 0.45}, {1.4, 0.9, 0.9});
    addDebris({2.5, -7.0, 0.55}, {1.0, 1.2, 1.1});
    addDebris({11.0, 5.5, 0.35}, {1.7, 0.8, 0.7});
    addPerson({-10.0, 7.5});
    addPerson({10.0, -6.5});

    missionGoalMeters_ = {-10.0, 7.5, 2.0};
    simulation_.SetInitialPose({demoDroneStartMeters.x, demoDroneStartMeters.y,
        aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters});
    fleetDroneIds_ = simulation_.DroneIds();
#ifndef __EMSCRIPTEN__
    simulation_.SetTerrain(terrainGrid_);
#endif
}

void AeroSimLabApp::AdvanceSimulation(double elapsedSeconds)
{
    const std::size_t tickCount = fixedTickAccumulator_.Consume(elapsedSeconds);
    for (std::size_t tick = 0; tick < tickCount; ++tick)
    {
        double tickDurationMicroseconds = 0.0;
        const std::uint64_t previousSimulationTick = simulation_.TickCount();
        {
            aerosimlab::simulation::ScopedDuration measureTick(
                benchmarkMode_ ? &tickDurationMicroseconds : nullptr);
            AdvanceSimulationTick();
        }
        if (benchmarkMode_)
        {
            benchmarkMeasurements_.tickDurationsMicroseconds.push_back(
                tickDurationMicroseconds);
#ifndef __EMSCRIPTEN__
            if (simulation_.TickCount() != previousSimulationTick)
                benchmarkMeasurements_.physicsStepDurationsMicroseconds.push_back(
                    simulation_.LastPhysicsStepDurationMicroseconds());
#endif
        }
    }
}

void AeroSimLabApp::AdvanceSimulationTick()
{
    if (demoMode_ && !externalControl_)
        ApplyDemoCommands();
    ExpireGroundControlCommand();
    const bool groundControlTickAdvanced = ProcessPendingGroundCommand();
    if (!groundControlTickAdvanced
        && !groundControlCommandExpiresAtTick_
        && groundControlActive_)
    {
        groundControlActive_ = false;
        window_->SetTitle(gamepadConnected_
            ? "AeroSimLab | 게임패드 연결됨"
            : "AeroSimLab | 드론 관찰 · EO 영상");
    }
    // 지상통제 명령이 이 Tick을 이미 진행했다면 재생·게임패드 입력을 더 적용하지 않는다.
    if (groundControlTickAdvanced)
        return;

    if (commandRecording_.IsReplaying())
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
    commandRecording_.AdvanceReplayTick();
    if (commandRecording_.ReplayFinished())
    {
        commandRecording_.StopReplay();
        window_->SetTitle("AeroSimLab | 기록 재생 완료");
    }
}

void AeroSimLabApp::AdvanceManualControlTick()
{
    if (!groundControlActive_ && pendingControllerAction_)
    {
        static_cast<void>(ApplyAndRecordCommand(*pendingControllerAction_));
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
            static_cast<void>(ApplyAndRecordCommand(velocityCommand));
            static_cast<void>(ApplyAndRecordCommand(yawCommand));
        }
    }

    simulation_.AdvanceOneTick();
    commandRecording_.FinishTick();
}

aerosimlab::simulation::CommandResult AeroSimLabApp::ApplyAndRecordCommand(
    const aerosimlab::simulation::DroneCommand& command)
{
    const auto result = simulation_.Apply(command);
    if (result == aerosimlab::simulation::CommandResult::Accepted)
        commandRecording_.Record(command);
    return result;
}

void AeroSimLabApp::ApplyDemoCommands()
{
    using Kind = aerosimlab::simulation::DroneCommand::Kind;
    const std::size_t tick = static_cast<std::size_t>(simulation_.TickCount()) % demoLengthTicks;
    aerosimlab::simulation::DroneCommand command;
    switch (tick)
    {
    case 0:
        command.kind = Kind::TakeOff;
        break;
    case 120:
    {
        command.kind = Kind::SetWaypoint;
        command.vector = missionGoalMeters_;
        break;
    }
    case 300:
    {
        command.kind = Kind::SetYawRate;
        command.yawRateRadiansPerSecond = 0.35;
        break;
    }
    case 360:
        command.kind = Kind::Stop;
        break;
    case 600:
        command.kind = Kind::Land;
        break;
    default:
        return;
    }

    (void)simulation_.Apply(command);
}

void AeroSimLabApp::RenderFrame()
{
#ifdef __EMSCRIPTEN__
    int width = 0;
    int height = 0;
    window_->GetFramebufferSize(width, height);
    if (width <= 0 || height <= 0)
        return;

    renderer_->Resize(width, height);
    const int observerWidth = width * 2 / 3;
    const int cameraWidth = width - observerWidth;
    UpdateObserverCamera(observerWidth, height);
    UpdateEoCamera(cameraWidth, height);
    renderer_->BeginFrame();
    renderer_->SetSceneViewport(0, 0, observerWidth, height);
    DrawWorld(observerCamera_->GetViewMatrix(), observerCamera_->GetProjectionMatrix(),
        observerCamera_->GetPosition());
    renderer_->SetSceneViewport(observerWidth, 0, cameraWidth, height);
    DrawWorld(eoCamera_->GetViewMatrix(), eoCamera_->GetProjectionMatrix(),
        eoCamera_->GetPosition());
    renderer_->EndFrame();
#else
    if (hud_)
    {
        const bool groundControlConnected = lastGroundControlMessage_
            && std::chrono::steady_clock::now() - *lastGroundControlMessage_
                < std::chrono::seconds(2);
        const bool cameraActive = simulation_.Camera().status
            == aerosimlab::simulation::CameraStatus::Active;
        const auto& camera = simulation_.Camera();
        const glm::dvec3& dronePosition = simulation_.State().positionMeters;
        const double missionDistanceMeters = std::hypot(
            missionGoalMeters_.x - dronePosition.x,
            missionGoalMeters_.y - dronePosition.y);
        const bool missionComplete = simulation_.State().mode
                == aerosimlab::simulation::FlightMode::Grounded
            && missionDistanceMeters <= 1.5;
        HudRenderTimings hudTimings;
        hudTimings.collectGpuTimings = visibleBenchmarkMode_;
        hudTimings.frameIndex = static_cast<std::uint64_t>(renderedFrames_ + 1U);
        hud_->Render(simulation_, gamepadConnected_, groundControlConnected, cameraActive,
            camera.imageWidthPixels, camera.imageHeightPixels,
            missionDistanceMeters, missionComplete,
            [this](int width, int height) { return RenderScenePanel(width, height); },
            [this] { return RenderCameraPanel(); },
            benchmarkMode_ ? &hudTimings : nullptr);
        frameMeasurements_.hudInterfaceBuildMicroseconds = hudTimings.interfaceBuildMicroseconds;
        frameMeasurements_.hudFinalizeMicroseconds = hudTimings.frameFinalizeMicroseconds;
        frameMeasurements_.hudBackendDrawMicroseconds = hudTimings.backendDrawMicroseconds;
        frameMeasurements_.hudDrawListCount = hudTimings.drawListCount;
        frameMeasurements_.hudDrawCommandCount = hudTimings.drawCommandCount;
        frameMeasurements_.hudVertexCount = hudTimings.vertexCount;
        frameMeasurements_.hudIndexCount = hudTimings.indexCount;
        frameMeasurements_.completedHudGpuTimingCount = std::min(
            hudTimings.completedGpuFrameCount, completedHudGpuTimings_.size());
        for (std::size_t index = 0;
             index < frameMeasurements_.completedHudGpuTimingCount; ++index)
        {
            completedHudGpuTimings_[index] = hudTimings.completedGpuFrames[index];
        }
    }
#endif
}

void AeroSimLabApp::UpdateObserverCamera(int width, int height)
{
    const auto& state = simulation_.State();
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    observerCamera_->SetPosition(dronePosition + glm::vec3{8.0F, 6.0F, 8.0F});
    observerCamera_->SetTarget(dronePosition + glm::vec3{0.0F, 0.4F, 0.0F});
    observerCamera_->SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
}

void AeroSimLabApp::UpdateEoCamera(int width, int height)
{
    const auto& cameraState = simulation_.Camera();
    const glm::vec3 cameraPosition = ToRenderPosition(cameraState.positionMeters);
    const glm::mat4 cameraOrientation = glm::mat4_cast(
        ToRenderOrientation(cameraState.orientation));

    const glm::vec3 viewDirection = glm::vec3{
        cameraOrientation * glm::vec4{25.0F, -2.5F, 0.0F, 0.0F}};
    const glm::vec3 cameraUp = glm::vec3{
        cameraOrientation * glm::vec4{0.0F, 1.0F, 0.0F, 0.0F}};
    eoCamera_->SetPosition(cameraPosition);
    eoCamera_->SetTarget(cameraPosition + viewDirection);
    eoCamera_->SetUp(cameraUp);
    eoCamera_->SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
}

void AeroSimLabApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    DrawCube(floorTransform_, cameraPosition, groundMaterial_, view, projection);
    for (const glm::mat4& transform : wallTransforms_)
        DrawCube(transform, cameraPosition, buildingMaterial_, view, projection);
    for (const glm::mat4& transform : debrisTransforms_)
        DrawCube(transform, cameraPosition, facilityMaterial_, view, projection);
    for (const glm::mat4& transform : targetTransforms_)
        DrawCube(transform, cameraPosition, targetMaterial_, view, projection);

    const glm::vec3 goal = ToRenderPosition(missionGoalMeters_);
    DrawCube(CubeTransform(goal + glm::vec3{0.0F, 0.09F, 0.0F},
        glm::vec3{2.4F, 0.18F, 2.4F}), cameraPosition,
        missionMarkerMaterial_, view, projection);
    DrawCube(CubeTransform(goal + glm::vec3{0.0F, 1.6F, 0.0F},
        glm::vec3{0.18F, 3.0F, 0.18F}), cameraPosition,
        missionMarkerMaterial_, view, projection);

    for (const aerosimlab::simulation::DroneId id : fleetDroneIds_)
    {
        const auto& drone = simulation_.State(id);
        const glm::vec3 position = ToRenderPosition(drone.positionMeters);
        const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
            glm::mat4_cast(ToRenderOrientation(drone.orientation));
        renderer_->Draw(heading, *droneMesh_, droneMaterial_, *shader_,
            view, projection, cameraPosition,
#ifndef __EMSCRIPTEN__
            droneSurfaceTexture_.get(),
#else
            nullptr,
#endif
            true);
    }
}

void AeroSimLabApp::DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
    const aerosimlab::graphics::Material& material,
    const glm::mat4& view, const glm::mat4& projection)
{
    renderer_->Draw(transform, *cube_, material, *shader_, view, projection, cameraPosition);
}

}
