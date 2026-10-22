#include "AeroSimLabApp.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include "graphics/Camera.h"
#include "graphics/GlApi.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Texture2D.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#include "SimulationBenchmark.h"
#include "simulation_core/QuadrotorFlightController.h"
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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
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
// 창이 멈춘 뒤 밀린 시간을 전부 따라잡느라 다음 화면까지 늦어지지 않게 한다.
constexpr std::size_t smokeTestFrameLimit = 8;
constexpr std::size_t demoLengthTicks = 900;
// 지도 원점은 경사가 급해 이륙이 막히므로 주변에서 고른 완만한 지점을 쓴다.
constexpr glm::dvec2 demoDroneStartMeters{171.3545, -92.7667};
constexpr double benchmarkDurationSeconds = 5.0;
// OSM에 높이가 없어서 시연 상자에만 쓰는 명시적 가정이다. 실측값이 아니다.
constexpr float missingBuildingHeightProxyMeters = 3.0F;

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
        const double frameSeconds = smokeTest_ && !benchmarkMode_
            ? (networkEnabled_
                ? aerosimlab::simulation::FixedTickAccumulator::maximumFrameSeconds
                : aerosimlab::simulation::FixedTickAccumulator::fixedStepSeconds)
            : elapsedSeconds;
        previousFrame = now;

        AdvanceSimulation(frameSeconds);
        const auto renderStart = std::chrono::steady_clock::now();
        RenderFrame();
        window_->SwapBuffers();
        if (benchmarkMode_)
        {
            renderFrameDurationsMicroseconds_.push_back(
                std::chrono::duration<double, std::micro>(
                    std::chrono::steady_clock::now() - renderStart).count());
        }
        ++renderedFrames_;

        if (smokeTest_ && !networkEnabled_ && renderedFrames_ >= smokeTestFrameLimit)
            break;
        if (smokeTest_ && networkEnabled_ && smokeReplayStarted_
            && !commandRecording_.IsRecording() && !commandRecording_.IsReplaying()
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
    const auto& camera = simulation_.Camera();
    if (smokeTest_ && camera.status == aerosimlab::simulation::CameraStatus::Active
        && eoFrame_.simulationTick != camera.lastFrameTick)
    {
        return 6;
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
            "AeroSimLab | 드론 관찰 · EO 영상 | © OpenStreetMap contributors",
            !smokeTest_ && !benchmarkMode_,
            !smokeTest_ && !benchmarkMode_});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    window_->GetFramebufferSize(framebufferWidth, framebufferHeight);

    renderer_ = std::make_unique<aerosimlab::graphics::Renderer>();
    renderer_->Init(framebufferWidth, framebufferHeight);
    observerCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{7.0F, 5.0F, 7.0F}, glm::vec3{0.0F}, 1.0F,
        45.0F, 0.5F, 12000.0F);
    eoCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F, 70.0F, 0.05F, 200.0F);

    cube_ = aerosimlab::graphics::Mesh::CreateCube(1.0F);
    droneMesh_ = aerosimlab::graphics::Mesh::LoadGlb("models/drones/x500/visual.glb");
    auto terrain = aerosimlab::terrain::TerrainGrid::Load("maps/bukhansan-demo/terrain.grid");
    terrainMesh_ = CreateTerrainMesh(terrain);
    terrainTexture_ = aerosimlab::graphics::Texture2D::LoadPpm(
        "maps/bukhansan-demo/terrain-satellite.ppm");
    droneSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadPpm(
        "models/drones/x500/carbon-weave.ppm");
    facilityMesh_ = aerosimlab::graphics::Mesh::LoadGlb(
        "maps/bukhansan-demo/facilities.glb");
    const auto buildings = aerosimlab::terrain::LoadBuildingFootprints(
        "maps/bukhansan-demo/buildings.geojson", terrain);
    const auto facilities = aerosimlab::terrain::LoadStaticFacilities(
        "maps/bukhansan-demo/facilities.json");
#ifndef __EMSCRIPTEN__
    simulation_.SetInitialPose({demoDroneStartMeters,
        terrain.HeightAtLocalMeters(demoDroneStartMeters)
            + aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters});
    for (const auto& facility : facilities)
        simulation_.AddStaticObstacleBox(
            facility.centerMeters, facility.halfExtentsMeters);
#endif
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
        const double groundHeightMeters = terrain.HeightAtLocalMeters(center);
        const glm::dvec3 centerMeters{
            center.x, center.y, groundHeightMeters + heightMeters * 0.5};
        const glm::vec3 sizeMeters{
            static_cast<float>(footprintSize.x), heightMeters,
            static_cast<float>(footprintSize.y)};

#ifndef __EMSCRIPTEN__
        simulation_.AddStaticObstacleBox(centerMeters, sizeMeters * 0.5F);
#endif
        buildingTransforms_.push_back(CubeTransform(
            ToRenderPosition(centerMeters), sizeMeters));
    }
    simulation_.SetTerrain(std::move(terrain));
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
    commandRecording_.FinishTick();
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
    const auto& cameraState = simulation_.Camera();
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const glm::vec3 cameraPosition = ToRenderPosition(cameraState.positionMeters);
    const glm::mat4 cameraOrientation = glm::mat4_cast(
        ToRenderOrientation(cameraState.orientation));

    observerCamera_->SetPosition(dronePosition + glm::vec3{7.0F, 5.0F, 7.0F});
    observerCamera_->SetTarget(dronePosition + glm::vec3{0.0F, 0.2F, 0.0F});
    observerCamera_->SetAspectRatio(static_cast<float>(observerWidth) / static_cast<float>(height));

    const glm::vec3 viewDirection = glm::vec3{
        cameraOrientation * glm::vec4{25.0F, -2.5F, 0.0F, 0.0F}};
    const glm::vec3 cameraUp = glm::vec3{
        cameraOrientation * glm::vec4{0.0F, 1.0F, 0.0F, 0.0F}};
    eoCamera_->SetPosition(cameraPosition);
    eoCamera_->SetTarget(cameraPosition + viewDirection);
    eoCamera_->SetUp(cameraUp);
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

void AeroSimLabApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    renderer_->Draw(glm::mat4{1.0F}, *terrainMesh_, groundMaterial_, *shader_,
        view, projection, cameraPosition, terrainTexture_.get(), false, true);
    for (const glm::mat4& transform : buildingTransforms_)
        DrawCube(transform, cameraPosition, buildingMaterial_, view, projection);

    const glm::mat4 simulationToRender = glm::rotate(
        glm::mat4{1.0F}, -1.57079632679F, glm::vec3{1.0F, 0.0F, 0.0F});
    renderer_->Draw(simulationToRender, *facilityMesh_, facilityMaterial_, *shader_,
        view, projection, cameraPosition);

    const glm::vec3 position = ToRenderPosition(simulation_.State().positionMeters);
    const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
        glm::mat4_cast(ToRenderOrientation(simulation_.State().orientation));
    renderer_->Draw(heading, *droneMesh_, droneMaterial_, *shader_,
        view, projection, cameraPosition, droneSurfaceTexture_.get(), true);
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

    ViewerBenchmarkMeasurements measurements;
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
    measurements.tickDurationsMicroseconds = tickDurationsMicroseconds_;
    measurements.renderFrameDurationsMicroseconds = renderFrameDurationsMicroseconds_;
#ifndef __EMSCRIPTEN__
    measurements.physicsStepDurationsMicroseconds = physicsStepDurationsMicroseconds_;
    measurements.visionDurationsMicroseconds = visionDurationsMicroseconds_;
    measurements.visionSampleCount = visionSampleCount_;
    if (visionSampleCount_ > 0)
    {
        const double sampleCount = static_cast<double>(visionSampleCount_);
        measurements.meanBrightness = totalMeanBrightness_ / sampleCount;
        measurements.meanContrast = totalMeanContrast_ / sampleCount;
        measurements.meanSharpness = totalSharpness_ / sampleCount;
    }
#endif
    nlohmann::json report = BuildViewerBenchmarkReport(std::move(measurements));
#ifndef __EMSCRIPTEN__
    report["entity_scaling"] = MeasureDroneScaling();
#endif
    SaveViewerBenchmarkReport(report);
}
}
