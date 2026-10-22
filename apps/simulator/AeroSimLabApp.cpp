#include "AeroSimLabApp.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include "graphics/Camera.h"
#include "graphics/GlApi.h"
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
#include "terrain/BuildingFootprint.h"
#include "terrain/StaticFacility.h"
#include "terrain/TerrainGrid.h"
#include <glm/gtc/quaternion.hpp>
#ifndef __EMSCRIPTEN__
#include "vision/RgbFrameQuality.h"
#include "vision/RgbFrameView.h"
#endif
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <ctime>
#include <cmath>
#include <nlohmann/json.hpp>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
namespace
{
constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
// 산의 높낮이가 화면에서 보이도록 지형과 그 위의 물체를 세로로 두 배 그린다.
constexpr float terrainReliefScale = 2.0F;
// 창이 멈춘 뒤 밀린 시간을 전부 따라잡느라 다음 화면까지 늦어지지 않게 한다.
constexpr std::size_t smokeTestFrameLimit = 8;
// 소프트웨어 렌더러에서도 네트워크 스모크의 모의가 화면 속도에 묶이지 않게 한다.
constexpr std::size_t networkSmokeSimulationBatchesPerFrame = 5;
constexpr std::size_t demoLengthTicks = 900;
// 지도 원점은 경사가 급해 이륙이 막히므로 주변에서 고른 완만한 지점을 쓴다.
constexpr glm::dvec2 demoDroneStartMeters{171.3545, -92.7667};
constexpr double benchmarkDurationSeconds = 5.0;
// OSM에 높이가 없어서 시연 상자에만 쓰는 명시적 가정이다. 실측값이 아니다.
constexpr float missingBuildingHeightProxyMeters = 3.0F;

std::filesystem::path ApplicationDirectory()
{
#ifdef _WIN32
    std::wstring executablePath(32768U, L'\0');
    const DWORD pathLength = GetModuleFileNameW(
        nullptr, executablePath.data(), static_cast<DWORD>(executablePath.size()));
    if (pathLength == 0U || pathLength >= executablePath.size())
        throw std::runtime_error("프로그램이 있는 폴더를 확인하지 못했습니다.");
    executablePath.resize(pathLength);
    return std::filesystem::path(executablePath).parent_path();
#elif defined(__EMSCRIPTEN__)
    return {};
#else
    std::error_code error;
    const auto executablePath = std::filesystem::read_symlink("/proc/self/exe", error);
    if (error)
        throw std::runtime_error("프로그램이 있는 폴더를 확인하지 못했습니다.");
    return executablePath.parent_path();
#endif
}

std::filesystem::path AssetPath(std::string_view relativePath)
{
    return ApplicationDirectory() / relativePath;
}

glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    // 모의는 Z축을 위로 쓰고 화면은 Y축을 위로 쓴다. Y축 방향은 화면의 -Z로 옮겨 오른손 좌표계를 유지한다.
    return glm::vec3{
        static_cast<float>(positionMeters.x), static_cast<float>(positionMeters.z),
        static_cast<float>(-positionMeters.y)}
        * glm::vec3{1.0F, terrainReliefScale, 1.0F};
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
            const glm::vec3 normal = glm::normalize(glm::vec3{
                -slopeEast * terrainReliefScale, 1.0F,
                -slopeSouth * terrainReliefScale});
            vertices[row * sampleCount + column] = {
                glm::vec3{
                    (static_cast<float>(column) - center) * terrain.spacingXMeters,
                    heightAt(row, column) * terrainReliefScale,
                    (static_cast<float>(row) - center) * terrain.spacingYMeters},
                normal,
                glm::vec2{
                    static_cast<float>(column) / static_cast<float>(sampleCount - 1U),
                    static_cast<float>(row) / static_cast<float>(sampleCount - 1U)}};
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
        for (std::size_t batch = 0; batch < simulationBatches; ++batch)
            AdvanceSimulation(frameSeconds);
        eoReadbackHappenedThisFrame_ = false;
        double renderDurationMicroseconds = 0.0;
        {
            aerosimlab::simulation::ScopedDuration measureRender(
                benchmarkMode_ ? &renderDurationMicroseconds : nullptr);
            RenderFrame();
            window_->SwapBuffers();
        }
        if (benchmarkMode_)
        {
            benchmarkMeasurements_.renderFrameDurationsMicroseconds.push_back(
                renderDurationMicroseconds);
            auto& frameDurations = eoReadbackHappenedThisFrame_
                ? benchmarkMeasurements_.renderFrameDurationsWithEoReadbackMicroseconds
                : benchmarkMeasurements_.renderFrameDurationsWithoutEoReadbackMicroseconds;
            frameDurations.push_back(renderDurationMicroseconds);
        }
        ++renderedFrames_;

        if (smokeTest_ && SmokeTestIsComplete())
            break;
        if (smokeTest_ && networkEnabled_
            && std::chrono::steady_clock::now() - startTime_ > std::chrono::seconds(15))
            return 4;
        if (benchmarkMode_ &&
            std::chrono::steady_clock::now() - startTime_ >= std::chrono::duration<double>(benchmarkDurationSeconds))
            break;
    }
    if (smokeTest_)
        DrainEoReadbacks();
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
        return 6;

    return 0;
}

void AeroSimLabApp::InitializeGraphics()
{
    window_ = std::make_unique<aerosimlab::graphics::Window>(
        aerosimlab::graphics::Window::Properties{windowWidth, windowHeight,
            "AeroSimLab | 드론 관찰 · EO 영상 | © OpenStreetMap contributors",
            !smokeTest_ && !benchmarkMode_,
            !smokeTest_ && !benchmarkMode_});

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
        45.0F, 0.5F, 25000.0F);
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
    droneMesh_ = aerosimlab::graphics::Mesh::LoadGlb(
        AssetPath("models/drones/x500/visual.glb").string());
    terrainGrid_ = aerosimlab::terrain::TerrainGrid::Load(
        AssetPath("maps/bukhansan-demo/terrain.grid"));
    missionGoalMeters_ = {
        demoDroneStartMeters.x + 10.0,
        demoDroneStartMeters.y,
        terrainGrid_.HeightAtLocalMeters({demoDroneStartMeters.x + 10.0,
            demoDroneStartMeters.y})};
    terrainMesh_ = CreateTerrainMesh(terrainGrid_);
    terrainTexture_ = aerosimlab::graphics::Texture2D::LoadPpm(
        AssetPath("maps/bukhansan-demo/terrain-satellite.ppm"));
    droneSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadPpm(
        AssetPath("models/drones/x500/carbon-weave.ppm"));
    facilityMesh_ = aerosimlab::graphics::Mesh::LoadGlb(
        AssetPath("maps/bukhansan-demo/facilities.glb").string());
    const auto buildings = aerosimlab::terrain::LoadBuildingFootprints(
        AssetPath("maps/bukhansan-demo/buildings.geojson"), terrainGrid_);
    const auto facilities = aerosimlab::terrain::LoadStaticFacilities(
        AssetPath("maps/bukhansan-demo/facilities.json"));
#ifndef __EMSCRIPTEN__
    simulation_.SetInitialPose({demoDroneStartMeters,
        terrainGrid_.HeightAtLocalMeters(demoDroneStartMeters)
            + aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters});
    for (const auto& facility : facilities)
        simulation_.AddStaticObstacleBox(
            facility.centerMeters, facility.halfExtentsMeters);
#endif
    fleetDroneIds_ = simulation_.DroneIds();
    buildingTransforms_.reserve(buildings.size());
    for (const auto& building : buildings)
    {
        glm::dvec2 minimum = building.outlineMeters.front();
        glm::dvec2 maximum = minimum;
        for (const glm::dvec2& point : building.outlineMeters)
        {
            minimum = glm::min(minimum, point);
            maximum = glm::max(maximum, point);
        }

        const glm::dvec2 footprintSize = maximum - minimum;
        const glm::dvec2 center = (minimum + maximum) * 0.5;
        const float heightMeters = building.heightMeters.value_or(
            missingBuildingHeightProxyMeters);
        const double groundHeightMeters = terrainGrid_.HeightAtLocalMeters(center);
        const glm::dvec3 centerMeters{
            center.x, center.y, groundHeightMeters + heightMeters * 0.5};
        const glm::vec3 sizeMeters{
            static_cast<float>(footprintSize.x), heightMeters * terrainReliefScale,
            static_cast<float>(footprintSize.y)};

#ifndef __EMSCRIPTEN__
        simulation_.AddStaticObstacleBox(centerMeters, sizeMeters * 0.5F);
#endif
        buildingTransforms_.push_back(CubeTransform(
            ToRenderPosition(centerMeters), sizeMeters));
    }
#ifndef __EMSCRIPTEN__
    // 웹 빌드는 운동학 모드라 충돌 물리 월드에만 필요한 지형 설정을 건너뛴다.
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
#ifndef __EMSCRIPTEN__
    const bool hasGroundCommand = pendingCommand_.has_value();
#else
    constexpr bool hasGroundCommand = false;
#endif
    if (!hasGroundCommand && !groundControlCommandExpiresAtTick_ && groundControlActive_)
    {
        groundControlActive_ = false;
        window_->SetTitle(gamepadConnected_
            ? "AeroSimLab | 게임패드 연결됨"
            : "AeroSimLab | 드론 관찰 · EO 영상");
    }
    ApplyGroundCommand();
    if (hasGroundCommand)
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
    const auto applyAndRecord = [this](
        const aerosimlab::simulation::DroneCommand& command)
    {
        if (simulation_.Apply(command)
            == aerosimlab::simulation::CommandResult::Accepted)
        {
            RecordCommand(command);
        }
    };

    if (!groundControlActive_ && pendingControllerAction_)
    {
        applyAndRecord(*pendingControllerAction_);
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
            applyAndRecord(velocityCommand);
            applyAndRecord(yawCommand);
        }
    }

    simulation_.AdvanceOneTick();
    commandRecording_.FinishTick();
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
        command.vector = {8.0, 0.0, 2.0};
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
        hud_->Render(simulation_, gamepadConnected_, groundControlConnected, cameraActive,
            camera.imageWidthPixels, camera.imageHeightPixels,
            missionDistanceMeters, missionComplete,
            [this](int width, int height) { return RenderScenePanel(width, height); },
            [this] { return RenderCameraPanel(); });
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

#ifndef __EMSCRIPTEN__
std::uint32_t AeroSimLabApp::RenderScenePanel(int width, int height)
{
    sceneTarget_->Resize(width, height);
    UpdateObserverCamera(width, height);
    return RenderWorldToTarget(*sceneTarget_, *observerCamera_);
}

std::uint32_t AeroSimLabApp::RenderCameraPanel()
{
    const auto& camera = simulation_.Camera();
    const int imageWidth = camera.imageWidthPixels;
    const int imageHeight = camera.imageHeightPixels;
    cameraTarget_->Resize(imageWidth, imageHeight);
    UpdateEoCamera(imageWidth, imageHeight);
    const std::uint32_t texture = RenderWorldToTarget(*cameraTarget_, *eoCamera_);
    CaptureEoFrame();
    return texture;
}

std::uint32_t AeroSimLabApp::RenderWorldToTarget(
    aerosimlab::graphics::MultisampleFramebuffer& target,
    const aerosimlab::graphics::Camera& camera)
{
    renderer_->BeginView(target);
    DrawWorld(camera.GetViewMatrix(), camera.GetProjectionMatrix(), camera.GetPosition());
    renderer_->EndView(target);
    return target.GetTextureId();
}

void AeroSimLabApp::CaptureEoFrame()
{
    const auto& camera = simulation_.Camera();
    std::uint64_t simulationTick = 0;
    double pollDurationMicroseconds = 0.0;
    bool readCompleted = false;
    {
        aerosimlab::simulation::ScopedDuration measurePoll(
            benchmarkMode_ ? &pollDurationMicroseconds : nullptr);
        readCompleted = cameraTarget_->TryReadRgbPixels(eoFrame_.pixels, simulationTick);
    }
    if (benchmarkMode_)
        benchmarkMeasurements_.eoReadbackPollDurationsMicroseconds.push_back(
            pollDurationMicroseconds);
    if (readCompleted)
    {
        eoReadbackHappenedThisFrame_ = true;
        ProcessEoPixels(simulationTick);
    }

    if (camera.status != aerosimlab::simulation::CameraStatus::Active
        || camera.lastFrameTick <= lastQueuedEoFrameTick_)
        return;

    double enqueueDurationMicroseconds = 0.0;
    bool queued = false;
    {
        aerosimlab::simulation::ScopedDuration measureEnqueue(
            benchmarkMode_ ? &enqueueDurationMicroseconds : nullptr);
        queued = cameraTarget_->QueueRgbReadback(camera.lastFrameTick);
    }
    if (queued)
    {
        lastQueuedEoFrameTick_ = camera.lastFrameTick;
        if (benchmarkMode_)
        {
            eoReadbackHappenedThisFrame_ = true;
            benchmarkMeasurements_.eoReadbackEnqueueDurationsMicroseconds.push_back(
                enqueueDurationMicroseconds);
        }
    }
}

void AeroSimLabApp::ProcessEoPixels(std::uint64_t simulationTick)
{
    eoFrame_.width = cameraTarget_->GetWidth();
    eoFrame_.height = cameraTarget_->GetHeight();
    eoFrame_.simulationTick = simulationTick;
    eoFrame_.FlipVertical();
#ifndef __EMSCRIPTEN__
    double analysisDurationMicroseconds = 0.0;
    // OpenCV는 현재 프레임 메모리를 빌려 쓴다. 분석이 끝날 때까지 픽셀을 바꾸지 않는다.
    aerosimlab::vision::RgbFrameQuality quality;
    {
        aerosimlab::simulation::ScopedDuration measureAnalysis(
            benchmarkMode_ ? &analysisDurationMicroseconds : nullptr);
        quality = visionAnalyzer_->Analyze(
            aerosimlab::vision::MakeRgbFrameView(eoFrame_));
    }
    totalMeanBrightness_ += quality.meanBrightness;
    totalMeanContrast_ += quality.meanContrast;
    totalSharpness_ += quality.sharpness;
    ++benchmarkMeasurements_.visionSampleCount;
    if (benchmarkMode_)
        benchmarkMeasurements_.visionDurationsMicroseconds.push_back(
            analysisDurationMicroseconds);
#endif
}

void AeroSimLabApp::DrainEoReadbacks()
{
    const auto& camera = simulation_.Camera();
    while (true)
    {
        cameraTarget_->FinishRgbReadbacks();
        std::uint64_t simulationTick = 0;
        while (cameraTarget_->TryReadRgbPixels(eoFrame_.pixels, simulationTick))
            ProcessEoPixels(simulationTick);

        if (camera.status != aerosimlab::simulation::CameraStatus::Active
            || camera.lastFrameTick <= lastQueuedEoFrameTick_)
            return;
        if (!cameraTarget_->QueueRgbReadback(camera.lastFrameTick))
            return;
        lastQueuedEoFrameTick_ = camera.lastFrameTick;
    }
}
#endif

void AeroSimLabApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    renderer_->Draw(glm::mat4{1.0F}, *terrainMesh_, groundMaterial_, *shader_,
        view, projection, cameraPosition, terrainTexture_.get(), false, true);
    for (const glm::mat4& transform : buildingTransforms_)
        DrawCube(transform, cameraPosition, buildingMaterial_, view, projection);

    const glm::mat4 simulationToRender = glm::rotate(
        glm::mat4{1.0F}, -1.57079632679F, glm::vec3{1.0F, 0.0F, 0.0F})
        * glm::scale(glm::mat4{1.0F}, glm::vec3{1.0F, 1.0F, terrainReliefScale});
    renderer_->Draw(simulationToRender, *facilityMesh_, facilityMaterial_, *shader_,
        view, projection, cameraPosition);

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
            view, projection, cameraPosition, droneSurfaceTexture_.get(), true);
    }
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

    ViewerBenchmarkMeasurements measurements = benchmarkMeasurements_;
    measurements.sourceRevision = AEROSIMLAB_SOURCE_REVISION;
    measurements.framebufferWidth = framebufferWidth;
    measurements.framebufferHeight = framebufferHeight;
    measurements.graphicsRenderer = glString(GL_RENDERER);
    measurements.graphicsVersion = glString(GL_VERSION);
    measurements.elapsedSeconds = elapsedSeconds;
    measurements.simulationTicks = simulation_.TickCount();
    measurements.renderedFrames = renderedFrames_;
    measurements.processCpuSeconds = static_cast<double>(
        std::clock() - benchmarkCpuStart_) / CLOCKS_PER_SEC;
#ifndef __EMSCRIPTEN__
    if (benchmarkMeasurements_.visionSampleCount > 0)
    {
        const double sampleCount = static_cast<double>(
            benchmarkMeasurements_.visionSampleCount);
        measurements.meanBrightness = totalMeanBrightness_ / sampleCount;
        measurements.meanContrast = totalMeanContrast_ / sampleCount;
        measurements.meanSharpness = totalSharpness_ / sampleCount;
    }
#endif
    nlohmann::json report = BuildViewerBenchmarkReport(std::move(measurements));
#ifndef __EMSCRIPTEN__
    report["entity_scaling"] = {
        {"kinematic", MeasureDroneScaling(
            aerosimlab::simulation::SimulationDynamics::Kinematic,
            DroneScalingLayout::Separated)},
        {"rigid_body", MeasureDroneScaling(
            aerosimlab::simulation::SimulationDynamics::RigidBody,
            DroneScalingLayout::Separated)},
        {"rigid_body_compact", MeasureDroneScaling(
            aerosimlab::simulation::SimulationDynamics::RigidBody,
            DroneScalingLayout::Compact)}};
    report["cooperative_mission_scaling"] = MeasureCooperativeMissionScaling();
#endif
    SaveViewerBenchmarkReport(report);
}
}
