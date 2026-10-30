#include "AeroSimLabApp.h"
#include "ExecutableDirectory.h"
#include "FleetMissionProgress.h"
#include "TrainingRangeScene.h"

#ifdef _WIN32
#include <Windows.h>
#endif
#include "graphics/Camera.h"
#include "graphics/GlbModel.h"
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
#include "flight_planning/BuildingObstacle.h"
#include "terrain/TerrainGrid.h"
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <array>
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
#include <ctime>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
namespace
{
#ifdef __EMSCRIPTEN__
AeroSimLabApp* activeWebApplication = nullptr;
#endif
void SetStartupDetail(std::string_view message)
{
#ifdef __EMSCRIPTEN__
    const std::string text(message);
    EM_ASM({ window.setSimulatorStartupStatus(UTF8ToString($0)); }, text.c_str());
#else
    static_cast<void>(message);
#endif
}

constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
// 전력값은 임무 흐름을 시험하기 위한 가정이며 실제 X500 배터리 사양이 아니다.
constexpr aerosimlab::simulation::BatteryModelParameters trainingBatteryModel{
    100.0, 40.0, 120.0, 25.0};
constexpr double trainingRangeCameraTransitAltitudeMeters = 25.0;
constexpr float glbAssetUpAxisCorrectionRadians = -1.57079632679F;

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

glm::vec3 ToRenderDirection(const glm::dvec3& direction)
{
    return glm::vec3{
        static_cast<float>(direction.x), static_cast<float>(direction.z),
        static_cast<float>(-direction.y)};
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
#ifdef __EMSCRIPTEN__
    activeWebApplication = this;
#endif
}

AeroSimLabApp::~AeroSimLabApp()
{
#ifdef __EMSCRIPTEN__
    if (activeWebApplication == this)
        activeWebApplication = nullptr;
#endif
}

#ifdef __EMSCRIPTEN__
void AeroSimLabApp::QueueWebFlightCommand(
    aerosimlab::simulation::DroneCommand::Kind kind)
{
    externalControl_ = true;
    if (CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive())
        return;
    pendingControllerAction_ = aerosimlab::simulation::DroneCommand{kind};
}

bool AeroSimLabApp::StartWebFleetMission()
{
    externalControl_ = true;
    return StartCompleteFleetMission();
}

bool AeroSimLabApp::StopWebFleetMission()
{
    externalControl_ = true;
    if (!CompleteFleetMissionIsActive() && !FleetReconnaissanceIsActive()
        && !formationTransit_.IsActive())
        return false;
    FailCompleteFleetMission("지상통제소에서 전체 임무를 중단했습니다.");
    return true;
}

void AeroSimLabApp::SetWebPilotAxes(
    const aerosimlab::manual_control::GamepadAxes& axes)
{
    externalControl_ = true;
    if (CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive())
    {
        webPilotAxes_.reset();
        pendingPilotCommand_.reset();
        return;
    }
    webPilotAxes_ = axes;
}
#endif

void AeroSimLabApp::InitializeGraphics()
{
    InitializeWindow();
    InitializeRendererAndCameras();
    LoadScene();
    InitializeShaderAndHud();
}

#ifdef __EMSCRIPTEN__
bool AeroSimLabApp::InitializeGraphicsStep()
{
    switch (webStartupStep_++)
    {
    case 0:
        EM_ASM({ window.setSimulatorStartupStatus("화면을 준비하고 있습니다."); });
        InitializeWindow();
        return false;
    case 1:
        EM_ASM({ window.setSimulatorStartupStatus("렌더링 장치를 준비하고 있습니다."); });
        InitializeRendererAndCameras();
        return false;
    case 2:
        EM_ASM({ window.setSimulatorStartupStatus("도시 지도와 충돌 형상을 읽고 있습니다."); });
        LoadSceneModels();
        LoadMapFeatures();
        return false;
    case 3:
        EM_ASM({ window.setSimulatorStartupStatus("지도 바닥과 물리 지면을 맞추고 있습니다."); });
        LoadTerrain();
        LoadReconTargets();
        return false;
    case 4:
        EM_ASM({ window.setSimulatorStartupStatus("비행 시작 위치를 준비하고 있습니다."); });
        InitializeStartingState();
        return false;
    case 5:
        EM_ASM({ window.setSimulatorStartupStatus("화면 구성을 마무리하고 있습니다."); });
        InitializeShaderAndHud();
        return false;
    default:
        return true;
    }
}
#endif

void AeroSimLabApp::InitializeWindow()
{
    window_ = std::make_unique<aerosimlab::graphics::Window>(
        aerosimlab::graphics::Window::Properties{windowWidth, windowHeight,
            "DroneSim | 드론 관찰 · EO 영상",
            !smokeTest_ && !benchmarkMode_,
            !smokeTest_ && (!benchmarkMode_ || visibleBenchmarkMode_)});
    window_->GetFramebufferSize(framebufferWidth_, framebufferHeight_);
}

void AeroSimLabApp::InitializeRendererAndCameras()
{
    renderer_ = std::make_unique<aerosimlab::graphics::Renderer>();
    renderer_->Init(framebufferWidth_, framebufferHeight_);
#ifndef __EMSCRIPTEN__
    sceneTarget_ = std::make_unique<aerosimlab::graphics::MultisampleFramebuffer>(
        framebufferWidth_, framebufferHeight_);
    cameraTarget_ = std::make_unique<aerosimlab::graphics::MultisampleFramebuffer>(
        framebufferWidth_, framebufferHeight_);
#endif
    observerCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F, 145.0F, 285.0F}, glm::vec3{0.0F, 12.0F, 0.0F}, 1.0F,
        45.0F, 0.1F, 2000.0F);
    eoCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F,
        static_cast<float>(simulation_.Camera().model.verticalFieldOfViewDegrees),
        0.05F, 200.0F);
}

void AeroSimLabApp::InitializeShaderAndHud()
{
    shader_ = std::make_unique<aerosimlab::graphics::Shader>(
        AssetPath("shaders/Scene.glsl").string());
#ifndef __EMSCRIPTEN__
    hud_ = std::make_unique<SimulatorHud>(window_->GetNativeHandle());
#endif
}

void AeroSimLabApp::LoadScene()
{
    LoadSceneModels();
    LoadMapFeatures();
    LoadTerrain();
    LoadReconTargets();
    InitializeStartingState();
}

void AeroSimLabApp::LoadSceneModels()
{
    cube_ = aerosimlab::graphics::Mesh::CreateCube(1.0F);
    droneModel_ = aerosimlab::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/visual.glb").string());
    droneGimbalModel_ = aerosimlab::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/cgo3-gimbal.glb").string());
    droneSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadFromFile(
        AssetPath("models/drones/x500/carbon-weave.ppm"));
}

void AeroSimLabApp::LoadTerrain()
{
    if (!trainingRangeScene_)
        throw std::logic_error("물리 지면을 만들기 전에 도시 바닥을 읽어야 합니다.");

    // GLB 바닥판의 크기와 같은 범위를 경로 계획과 Jolt 충돌에 사용한다.
    constexpr std::uint32_t sampleCount = 61U;
    const glm::dvec2 groundSize = trainingRangeScene_->GroundSizeMeters();
    terrainGrid_.sampleCount = sampleCount;
    terrainGrid_.spacingXMeters = static_cast<float>(
        groundSize.x / static_cast<double>(sampleCount - 1U));
    terrainGrid_.spacingYMeters = static_cast<float>(
        groundSize.y / static_cast<double>(sampleCount - 1U));
    terrainGrid_.referenceElevationMeters = 0.0F;
    terrainGrid_.heightOffsetsMeters.assign(
        static_cast<std::size_t>(sampleCount) * sampleCount, 0.0F);
    simulation_.SetTerrain(terrainGrid_);
}
void AeroSimLabApp::LoadMapFeatures()
{
    SetStartupDetail("도시 지도와 충돌 모양을 준비하고 있습니다.");
    trainingRangeScene_ = std::make_unique<TrainingRangeScene>();
    trainingRangeScene_->Load(AssetPath(""), simulation_);
    cameraTransitAltitudeMeters_ = trainingRangeCameraTransitAltitudeMeters;
}

void AeroSimLabApp::LoadReconTargets()
{
    reconTargets_.clear();

    for (const auto& point : trainingRangeScene_->ReconPoints())
    {
        const double groundMeters = terrainGrid_.HeightAtLocalMeters(point.positionMeters);
        reconTargets_.push_back({point.id, {point.positionMeters, groundMeters + 25.0}});
    }
}
void AeroSimLabApp::InitializeStartingState()
{
    if (reconTargets_.empty())
        throw std::runtime_error("훈련장 정찰 지점을 준비하지 못했습니다.");
    const glm::dvec2 demoDroneStartMeters =
        trainingRangeScene_->FormationStagingPositionMeters();
    const double startElevation = terrainGrid_.HeightAtLocalMeters(demoDroneStartMeters);
    const glm::dvec2 toFirstTarget = glm::dvec2{
        reconTargets_.front().approachPositionMeters.x,
        reconTargets_.front().approachPositionMeters.y} - demoDroneStartMeters;
    const double initialYawRadians = std::atan2(toFirstTarget.y, toFirstTarget.x);
    simulation_.SetInitialPose({demoDroneStartMeters.x, demoDroneStartMeters.y,
        startElevation +
            aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
        initialYawRadians);
    ConfigureTrainingBattery(simulation_.PrimaryDroneId());
    const bool singleDroneBenchmark = benchmarkMode_ && !demoMode_;
    if (singleDroneBenchmark)
    {
        const auto result = simulation_.Apply({
            aerosimlab::simulation::DroneCommand::Kind::TakeOff});
        if (result != aerosimlab::simulation::CommandResult::Accepted)
            throw std::runtime_error("단일 기체 성능 시험의 이륙 명령을 시작하지 못했습니다.");
    }
    else
    {
        for (const glm::dvec2 offset : std::array<glm::dvec2, 3>{
                 glm::dvec2{-2.0, 0.0}, glm::dvec2{2.0, 0.0}, glm::dvec2{0.0, -2.0}})
        {
            const glm::dvec2 position = demoDroneStartMeters + offset;
            const double groundMeters = terrainGrid_.HeightAtLocalMeters(position);
            const auto droneId = simulation_.AddDrone(
                {position, groundMeters
                    + aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
                initialYawRadians, groundMeters);
            ConfigureTrainingBattery(droneId);
        }
    }
    fleetDroneIds_ = simulation_.DroneIds();
}

void AeroSimLabApp::ConfigureTrainingBattery(
    aerosimlab::simulation::DroneId droneId)
{
    simulation_.ConfigureBattery(droneId, trainingBatteryModel);
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
    AdvanceAmbientWind();
    AdvanceFleetNetwork();
    AdvanceFleetReconnaissance();
    AdvanceFormationTransit();
    AdvanceCompleteFleetMission();
    if (demoMode_ && !externalControl_ && CompleteFleetMissionCanStart())
        static_cast<void>(StartCompleteFleetMission());
    ExpireGroundControlCommand();
    const bool groundControlTickAdvanced = ProcessPendingGroundCommand();
    const bool fleetMissionActive = std::any_of(
        fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return detail::IsFleetMissionInProgress(entry.second.status);
        });
    if (!groundControlTickAdvanced
        && groundControlCommandExpiresAtTick_.empty()
        && groundControlActive_ && !fleetMissionActive)
    {
        groundControlActive_ = false;
        window_->SetTitle(gamepadConnected_
            ? "AeroSimLab | 게임패드 연결됨"
            : "AeroSimLab | 드론 관찰 · EO 영상");
    }
    // 지상통제 명령이 이 Tick을 이미 진행했다면 게임패드 입력을 더 적용하지 않는다.
    if (groundControlTickAdvanced)
        return;

    AdvanceManualControlTick();
}

void AeroSimLabApp::AdvanceAmbientWind()
{
    constexpr std::uint32_t gustChangeIntervalTicks = 240U;
    constexpr double maximumWindMetersPerSecond = 0.5;
    constexpr double smoothingPerTick = 0.01;
    if (windGustTicksRemaining_ == 0U)
    {
        std::uniform_real_distribution<double> chooseWind(
            -maximumWindMetersPerSecond, maximumWindMetersPerSecond);
        windTargetMetersPerSecond_ = {
            chooseWind(windGenerator_), chooseWind(windGenerator_), 0.0};
        windGustTicksRemaining_ = gustChangeIntervalTicks;
    }
    --windGustTicksRemaining_;
    windVelocityMetersPerSecond_ += (windTargetMetersPerSecond_
        - windVelocityMetersPerSecond_) * smoothingPerTick;
    simulation_.SetWindVelocityMetersPerSecond(windVelocityMetersPerSecond_);
}

void AeroSimLabApp::AdvanceManualControlTick()
{
    if (CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive())
    {
        simulation_.AdvanceOneTick();
        return;
    }

#ifdef __EMSCRIPTEN__
    if (webPilotAxes_)
    {
        const auto mode = simulation_.State().mode;
        if (mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying)
        {
            pendingPilotCommand_ = aerosimlab::manual_control::MapAxes(
                *webPilotAxes_, simulation_.State().yawRadians);
        }
        else
        {
            pendingPilotCommand_.reset();
        }
    }
#endif
    if (!groundControlActive_ && pendingControllerAction_)
    {
        static_cast<void>(ApplySimulationCommand(*pendingControllerAction_));
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
            static_cast<void>(ApplySimulationCommand(velocityCommand));
            static_cast<void>(ApplySimulationCommand(yawCommand));
        }
    }

    simulation_.AdvanceOneTick();
}

aerosimlab::simulation::CommandResult AeroSimLabApp::ApplySimulationCommand(
    const aerosimlab::simulation::DroneCommand& command)
{
    return ApplySimulationCommand(simulation_.PrimaryDroneId(), command);
}

aerosimlab::simulation::CommandResult AeroSimLabApp::ApplySimulationCommand(
    aerosimlab::simulation::DroneId droneId,
    const aerosimlab::simulation::DroneCommand& command)
{
    return simulation_.Apply(droneId, command);
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
    const auto& camera = simulation_.Camera();
    const int cameraHeight = std::min(height,
        cameraWidth * static_cast<int>(camera.model.imageHeightPixels)
            / static_cast<int>(camera.model.imageWidthPixels));
    const int cameraBottom = (height - cameraHeight) / 2;
    const int cameraTop = height - cameraBottom - cameraHeight;
    UpdateObserverCamera(observerWidth, height);
    UpdateEoCamera(cameraWidth, cameraHeight);
    renderer_->BeginFrame();
    renderer_->SetSceneViewport(0, 0, observerWidth, height);
    DrawWorld(observerCamera_->GetViewMatrix(), observerCamera_->GetProjectionMatrix(),
        observerCamera_->GetPosition(), true);
    renderer_->SetSceneViewport(observerWidth, cameraBottom, cameraWidth, cameraHeight);
    DrawWorld(eoCamera_->GetViewMatrix(), eoCamera_->GetProjectionMatrix(),
        eoCamera_->GetPosition(), false, simulation_.PrimaryDroneId());
    renderer_->EndFrame();
    EM_ASM({
        if (window.copyDroneSimEoViewport) {
            window.copyDroneSimEoViewport($0, $1, $2, $3);
        }
    }, observerWidth, cameraTop, cameraWidth, cameraHeight);
#else
    if (hud_)
    {
        const bool groundControlConnected = lastGroundControlMessage_
            && std::chrono::steady_clock::now() - *lastGroundControlMessage_
                < std::chrono::seconds(2);
        const bool fleetReturnCanStart = FleetReturnCanStart();
        const bool completeFleetMissionCanStart = CompleteFleetMissionCanStart();
        const bool formationNetworkActive = formationTransit_.IsActive()
            || CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive();
        const auto droneIds = simulation_.DroneIds();
        GroundControlStatus groundControlStatus;
        groundControlStatus.groundControlPositions.reserve(droneIds.size());
        for (const aerosimlab::simulation::DroneId droneId : droneIds)
        {
            const auto report = telemetryAtGroundControl_.find(droneId);
            if (report == telemetryAtGroundControl_.end())
            {
                groundControlStatus.groundControlPositions.push_back({
                    .droneId = droneId});
                continue;
            }

            groundControlStatus.groundControlPositions.push_back({
                .droneId = droneId,
                .lastReceivedPositionMeters = report->second.state.positionMeters,
                .ageTicks = simulation_.TickCount() >= report->second.receivedAtTick
                    ? simulation_.TickCount() - report->second.receivedAtTick : 0U,
                .hasReport = true});
        }
        HudRenderTimings hudTimings;
        hudTimings.collectGpuTimings = visibleBenchmarkMode_;
        hudTimings.frameIndex = static_cast<std::uint64_t>(renderedFrames_ + 1U);
        hud_->Render(simulation_, observerDroneId_, gamepadConnected_, groundControlConnected,
            fleetReturnCanStart,
            [this] { static_cast<void>(StartFleetReturn()); },
            CompleteFleetMissionStatus(), completeFleetMissionCanStart,
            [this]
            {
                static_cast<void>(StartCompleteFleetMission());
            },
            formationNetwork_.get(),
            [this, formationNetworkActive](aerosimlab::simulation::DroneId droneId,
                bool enabled)
            {
                if (formationNetwork_ && formationNetworkActive)
                {
                    formationNetwork_->SetNodeRadioEnabled(droneId, enabled);
                }
            },
            groundControlStatus,
            [this](int width, int height) { return RenderScenePanel(width, height); },
            [this](aerosimlab::simulation::DroneId droneId)
            {
                return RenderCameraPanel(droneId);
            },
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
    const auto& state = observerDroneId_ == 0U
        ? simulation_.State()
        : simulation_.State(observerDroneId_);
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const glm::dvec3 forwardMeters{
        std::cos(state.yawRadians), std::sin(state.yawRadians), 0.0};
    constexpr double followDistanceMeters = 12.0;
    constexpr double followHeightMeters = 3.5;
    glm::dvec3 observerPositionMeters = state.positionMeters
        - forwardMeters * followDistanceMeters
        + glm::dvec3{0.0, 0.0, followHeightMeters};
    const auto isInsideBuilding = [this](const glm::dvec3& position)
    {
        const auto& buildings = trainingRangeScene_->BuildingObstacles();
        return std::any_of(buildings.begin(), buildings.end(),
            [&position](const aerosimlab::flight_planning::BuildingObstacle& building)
            {
                return aerosimlab::flight_planning::ContainsBuildingVolume(
                    building, position);
            });
    };
    // 건물 안에 걸리면 드론 쪽으로 당기지 않고, 시야가 열릴 때까지 뒤로 물러난다.
    if (isInsideBuilding(observerPositionMeters))
    {
        for (double retreatMeters = 1.0; retreatMeters <= 24.0; retreatMeters += 1.0)
        {
            const glm::dvec3 candidate = observerPositionMeters
                - forwardMeters * retreatMeters;
            if (!isInsideBuilding(candidate))
            {
                observerPositionMeters = candidate;
                break;
            }
        }
    }
    observerCamera_->SetPosition(ToRenderPosition(observerPositionMeters));
    observerCamera_->SetTarget(dronePosition + up * 0.04F);
    observerCamera_->SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    observerCamera_->SetFieldOfViewDegrees(45.0F);
}

void AeroSimLabApp::UpdateEoCamera(int width, int height)
{
    UpdateEoCamera(simulation_.PrimaryDroneId(), *eoCamera_, width, height);
}

void AeroSimLabApp::UpdateEoCamera(
    aerosimlab::simulation::DroneId droneId,
    aerosimlab::graphics::Camera& renderCamera,
    int width,
    int height)
{
    const auto& cameraState = simulation_.Camera(droneId);
    const glm::vec3 cameraPosition = ToRenderPosition(cameraState.positionMeters);
    const glm::dvec3 opticalAxis =
        aerosimlab::simulation::EoCameraModel::OpticalAxis(cameraState);
    const glm::dvec3 cameraUpAxis =
        aerosimlab::simulation::EoCameraModel::UpAxis(cameraState);
    const glm::vec3 viewDirection = ToRenderDirection(opticalAxis);
    const glm::vec3 cameraUp = ToRenderDirection(cameraUpAxis);
    renderCamera.SetPosition(cameraPosition);
    renderCamera.SetTarget(cameraPosition + viewDirection);
    renderCamera.SetUp(cameraUp);
    renderCamera.SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    renderCamera.SetFieldOfViewDegrees(
        static_cast<float>(cameraState.model.verticalFieldOfViewDegrees));
}

void AeroSimLabApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition, bool showMissionGuides,
    std::optional<aerosimlab::simulation::DroneId> hiddenDroneId)
{
    renderer_->SetAnimationTimeSeconds(static_cast<float>(simulation_.SimulationTimeSeconds()));
    if (trainingRangeScene_)
        trainingRangeScene_->Draw(*renderer_, *shader_, view, projection, cameraPosition);

    const auto drawRouteSegment = [this, &view, &projection, &cameraPosition](
        const glm::dvec3& startMeters, const glm::dvec3& endMeters,
        const aerosimlab::graphics::Material& material)
    {
        const glm::vec3 start = ToRenderPosition(startMeters);
        const glm::vec3 end = ToRenderPosition(endMeters);
        const glm::vec3 direction = end - start;
        const float length = glm::length(direction);
        if (length < 0.1F)
            return;

        const glm::vec3 localY = direction / length;
        const glm::vec3 referenceAxis = std::abs(localY.y) < 0.99F
            ? glm::vec3{0.0F, 1.0F, 0.0F}
            : glm::vec3{1.0F, 0.0F, 0.0F};
        const glm::vec3 localX = glm::normalize(glm::cross(referenceAxis, localY));
        const glm::vec3 localZ = glm::cross(localX, localY);
        glm::mat4 orientation{1.0F};
        orientation[0] = glm::vec4{localX, 0.0F};
        orientation[1] = glm::vec4{localY, 0.0F};
        orientation[2] = glm::vec4{localZ, 0.0F};
        constexpr float routeThicknessMeters = 0.12F;
        const glm::mat4 transform = glm::translate(
            glm::mat4{1.0F}, (start + end) * 0.5F)
            * orientation
            * glm::scale(glm::mat4{1.0F},
                glm::vec3{routeThicknessMeters, length, routeThicknessMeters});
        DrawCube(transform, cameraPosition, material, view, projection);
    };

    if (showMissionGuides)
    {
        for (const auto& [droneId, mission] : fleetReconnaissance_)
        {
            const auto missionStatus = mission.status;
            if (missionStatus != detail::FleetDroneStatus::TakeoffReady
                && missionStatus != detail::FleetDroneStatus::Reconnaissance
                && missionStatus != detail::FleetDroneStatus::Landing)
                continue;
            glm::dvec3 previous = mission.startPositionMeters;
            for (const glm::dvec3& waypoint : mission.routeWaypointsMeters)
            {
                drawRouteSegment(previous, waypoint, missionRouteMaterial_);
                previous = waypoint;
            }
            drawRouteSegment(previous,
                reconTargets_[mission.targetIndex].approachPositionMeters,
                missionRouteMaterial_);
            const glm::vec3 marker = ToRenderPosition(
                reconTargets_[mission.targetIndex].approachPositionMeters);
            DrawCube(glm::translate(glm::mat4{1.0F}, marker)
                * glm::scale(glm::mat4{1.0F}, glm::vec3{0.65F}),
                cameraPosition, missionRouteMaterial_, view, projection);
            static_cast<void>(droneId);
        }
    }

    for (const aerosimlab::simulation::DroneId id : fleetDroneIds_)
    {
        if (hiddenDroneId == id)
            continue;

        const auto& drone = simulation_.State(id);
        const glm::vec3 position = ToRenderPosition(drone.positionMeters);
        const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
            glm::mat4_cast(ToRenderOrientation(drone.orientation));
        droneModel_->Draw(heading, *renderer_, *shader_, view, projection,
            cameraPosition, &droneMaterial_, droneSurfaceTexture_.get(), true);
        // PX4 SDF는 Z축을 위로 쓰고 GLB 모델은 Y축을 위로 쓰므로, 기체 자세를 적용하기 전에 축을 맞춘다.
        const glm::mat4 gimbalAssetTransform = heading * glm::rotate(
            glm::mat4{1.0F}, glbAssetUpAxisCorrectionRadians, glm::vec3{1.0F, 0.0F, 0.0F});
        droneGimbalModel_->Draw(gimbalAssetTransform, *renderer_, *shader_, view, projection,
            cameraPosition);

    }
}

void AeroSimLabApp::DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
    const aerosimlab::graphics::Material& material,
    const glm::mat4& view, const glm::mat4& projection)
{
    renderer_->Draw(transform, *cube_, material, *shader_, view, projection, cameraPosition);
}

}

#ifdef __EMSCRIPTEN__
extern "C" EMSCRIPTEN_KEEPALIVE void DroneSimWebFlightCommand(int command)
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return;

    using Kind = aerosimlab::simulation::DroneCommand::Kind;
    switch (command)
    {
    case 0:
        aerosimlab::simulator::activeWebApplication->QueueWebFlightCommand(Kind::TakeOff);
        break;
    case 1:
        aerosimlab::simulator::activeWebApplication->QueueWebFlightCommand(Kind::Stop);
        break;
    case 2:
        aerosimlab::simulator::activeWebApplication->QueueWebFlightCommand(Kind::Land);
        break;
    default:
        break;
    }
}

extern "C" EMSCRIPTEN_KEEPALIVE void DroneSimWebPilotInput(
    double forward, double right, double vertical, double yaw)
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return;

    aerosimlab::simulator::activeWebApplication->SetWebPilotAxes({
        .forward = forward,
        .right = right,
        .vertical = vertical,
        .yaw = yaw});
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebStartFleetMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->StartWebFleetMission()
        ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebStopFleetMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->StopWebFleetMission() ? 1 : 0;
}
#endif
