#include "DroneSimApp.h"
#include "ExecutableDirectory.h"
#include "FleetMissionProgress.h"
#include "TrainingRangeScene.h"
#ifndef __EMSCRIPTEN__
#include "gui/SimulatorView.h"
#endif

#ifdef _WIN32
#include <Windows.h>
#endif
#ifdef __EMSCRIPTEN__
#include "graphics/Camera.h"
#endif
#include "graphics/GlbModel.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Texture2D.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
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

namespace dronesim::simulator
{
namespace
{
#ifdef __EMSCRIPTEN__
DroneSimApp* activeWebApplication = nullptr;
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
constexpr dronesim::simulation::BatteryModelParameters trainingBatteryModel{
    100.0, 40.0, 120.0, 25.0};
constexpr double trainingRangeCameraTransitAltitudeMeters = 25.0;

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

#ifdef __EMSCRIPTEN__
glm::vec3 ToRenderDirection(const glm::dvec3& direction)
{
    return glm::vec3{
        static_cast<float>(direction.x), static_cast<float>(direction.z),
        static_cast<float>(-direction.y)};
}
#endif

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

DroneSimApp::DroneSimApp(Options options)
    : smokeTest_(options.smokeTest), demoMode_(options.demoMode),
      benchmarkMode_(options.benchmarkMode || options.visibleBenchmarkMode),
      visibleBenchmarkMode_(options.visibleBenchmarkMode)
{
    if (benchmarkMode_)
        simulation_.SetPhysicsStepTimingEnabled(true);
#ifdef __EMSCRIPTEN__
    activeWebApplication = this;
#endif
}

DroneSimApp::~DroneSimApp()
{
#ifdef __EMSCRIPTEN__
    if (activeWebApplication == this)
        activeWebApplication = nullptr;
#endif
}

#ifdef __EMSCRIPTEN__
void DroneSimApp::QueueWebFlightCommand(
    dronesim::simulation::DroneCommand::Kind kind)
{
    externalControl_ = true;
    if (CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive())
        return;
    pendingControllerAction_ = dronesim::simulation::DroneCommand{kind};
}

bool DroneSimApp::StartWebFleetMission()
{
    externalControl_ = true;
    return StartCompleteFleetMission();
}

bool DroneSimApp::StopWebFleetMission()
{
    externalControl_ = true;
    if (!CompleteFleetMissionIsActive() && !FleetReconnaissanceIsActive()
        && !formationTransit_.IsActive())
        return false;
    FailCompleteFleetMission("화면에서 전체 임무를 중단했습니다.");
    return true;
}

void DroneSimApp::SetWebPilotAxes(
    const dronesim::manual_control::GamepadAxes& axes)
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

void DroneSimApp::InitializeGraphics()
{
    InitializeWindow();
    InitializeRendererAndCameras();
    LoadScene();
    InitializeShaderAndHud();
}

#ifdef __EMSCRIPTEN__
bool DroneSimApp::InitializeGraphicsStep()
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

void DroneSimApp::InitializeWindow()
{
    window_ = std::make_unique<dronesim::graphics::Window>(
        dronesim::graphics::Window::Properties{windowWidth, windowHeight,
            "DroneSim | 드론 관찰 · EO 영상",
            !smokeTest_ && !benchmarkMode_,
            !smokeTest_ && (!benchmarkMode_ || visibleBenchmarkMode_)});
    window_->GetFramebufferSize(framebufferWidth_, framebufferHeight_);
}

void DroneSimApp::InitializeRendererAndCameras()
{
    renderer_ = std::make_unique<dronesim::graphics::Renderer>();
    renderer_->Init(framebufferWidth_, framebufferHeight_);
#ifndef __EMSCRIPTEN__
    view_ = std::make_unique<dronesim::gui::SimulatorView>(*renderer_, framebufferWidth_,
        framebufferHeight_, static_cast<float>(
            simulation_.Camera().model.verticalFieldOfViewDegrees));
#else
    observerCamera_ = std::make_unique<dronesim::graphics::Camera>(
        glm::vec3{0.0F, 145.0F, 285.0F}, glm::vec3{0.0F, 12.0F, 0.0F}, 1.0F,
        45.0F, 0.1F, 2000.0F);
    eoCamera_ = std::make_unique<dronesim::graphics::Camera>(
        glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F,
        static_cast<float>(simulation_.Camera().model.verticalFieldOfViewDegrees),
        0.05F, 200.0F);
#endif
}

void DroneSimApp::InitializeShaderAndHud()
{
    shader_ = std::make_unique<dronesim::graphics::Shader>(
        AssetPath("shaders/Scene.glsl").string());
#ifndef __EMSCRIPTEN__
    hud_ = std::make_unique<dronesim::gui::SimulatorHud>(window_->GetNativeHandle());
#endif
}

void DroneSimApp::LoadScene()
{
    LoadSceneModels();
    LoadMapFeatures();
    LoadTerrain();
    LoadReconTargets();
    InitializeStartingState();
}

void DroneSimApp::LoadSceneModels()
{
    cube_ = dronesim::graphics::Mesh::CreateCube(1.0F);
    droneModel_ = dronesim::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/visual.glb").string());
    droneGimbalModel_ = dronesim::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/cgo3-gimbal.glb").string());
    droneSurfaceTexture_ = dronesim::graphics::Texture2D::LoadFromFile(
        AssetPath("models/drones/x500/carbon-weave.ppm"));
}

void DroneSimApp::LoadTerrain()
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
void DroneSimApp::LoadMapFeatures()
{
    SetStartupDetail("도시 지도와 충돌 모양을 준비하고 있습니다.");
    trainingRangeScene_ = std::make_unique<TrainingRangeScene>();
    trainingRangeScene_->Load(AssetPath(""), simulation_);
    cameraTransitAltitudeMeters_ = trainingRangeCameraTransitAltitudeMeters;
}

void DroneSimApp::LoadReconTargets()
{
    reconTargets_.clear();

    for (const auto& point : trainingRangeScene_->ReconPoints())
    {
        const double groundMeters = terrainGrid_.HeightAtLocalMeters(point.positionMeters);
        reconTargets_.push_back({point.id, {point.positionMeters, groundMeters + 25.0}});
    }
}
void DroneSimApp::InitializeStartingState()
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
            dronesim::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
        initialYawRadians);
    ConfigureTrainingBattery(simulation_.PrimaryDroneId());
    const bool singleDroneBenchmark = benchmarkMode_ && !demoMode_;
    if (singleDroneBenchmark)
    {
        const auto result = simulation_.Apply({
            dronesim::simulation::DroneCommand::Kind::TakeOff});
        if (result != dronesim::simulation::CommandResult::Accepted)
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
                    + dronesim::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
                initialYawRadians, groundMeters);
            ConfigureTrainingBattery(droneId);
        }
    }
    fleetDroneIds_ = simulation_.DroneIds();
}

void DroneSimApp::ConfigureTrainingBattery(
    dronesim::simulation::DroneId droneId)
{
    simulation_.ConfigureBattery(droneId, trainingBatteryModel);
}

void DroneSimApp::AdvanceSimulation(double elapsedSeconds)
{
    const std::size_t tickCount = fixedTickAccumulator_.Consume(elapsedSeconds);
    for (std::size_t tick = 0; tick < tickCount; ++tick)
    {
        double tickDurationMicroseconds = 0.0;
        const std::uint64_t previousSimulationTick = simulation_.TickCount();
        {
            dronesim::simulation::ScopedDuration measureTick(
                benchmarkMode_ ? &tickDurationMicroseconds : nullptr);
            AdvanceSimulationTick();
        }
        if (benchmarkMode_)
        {
            benchmarkSession_.AddTick(tickDurationMicroseconds);
#ifndef __EMSCRIPTEN__
            if (simulation_.TickCount() != previousSimulationTick)
                benchmarkSession_.AddPhysicsStep(
                    simulation_.LastPhysicsStepDurationMicroseconds());
#endif
        }
    }
}

void DroneSimApp::AdvanceSimulationTick()
{
    AdvanceAmbientWind();
    AdvanceFleetNetwork();
    AdvanceFleetReconnaissance();
    AdvanceFormationTransit();
    AdvanceCompleteFleetMission();
    if (demoMode_ && !externalControl_ && CompleteFleetMissionCanStart())
        static_cast<void>(StartCompleteFleetMission());
    const bool missionActive = formationTransit_.IsActive()
        || CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive();
    if (!missionActive && missionControlOwnsInput_)
    {
        missionControlOwnsInput_ = false;
        window_->SetTitle(gamepadController_.IsConnected()
            ? "DroneSim | 게임패드 연결됨"
            : "DroneSim | 드론 관찰 · EO 영상");
    }
    AdvanceManualControlTick();
}

void DroneSimApp::AdvanceAmbientWind()
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

void DroneSimApp::AdvanceManualControlTick()
{
    if (missionControlOwnsInput_ || CompleteFleetMissionIsActive()
        || FleetReconnaissanceIsActive() || formationTransit_.IsActive())
    {
        simulation_.AdvanceOneTick();
        return;
    }

#ifdef __EMSCRIPTEN__
    if (webPilotAxes_)
    {
        const auto mode = simulation_.State().mode;
        if (mode == dronesim::simulation::FlightMode::Hovering
            || mode == dronesim::simulation::FlightMode::Flying)
        {
            pendingPilotCommand_ = dronesim::manual_control::MapAxes(
                *webPilotAxes_, simulation_.State().yawRadians);
        }
        else
        {
            pendingPilotCommand_.reset();
        }
    }
#endif
    if (pendingControllerAction_)
    {
        static_cast<void>(ApplySimulationCommand(*pendingControllerAction_));
        pendingControllerAction_.reset();
    }
    if (pendingPilotCommand_)
    {
        const auto mode = simulation_.State().mode;
        if (mode == dronesim::simulation::FlightMode::Hovering
            || mode == dronesim::simulation::FlightMode::Flying)
        {
            const dronesim::simulation::DroneCommand velocityCommand{
                dronesim::simulation::DroneCommand::Kind::SetVelocity,
                pendingPilotCommand_->velocityMetersPerSecond};
            const dronesim::simulation::DroneCommand yawCommand{
                dronesim::simulation::DroneCommand::Kind::SetYawRate,
                glm::dvec3{0.0},
                pendingPilotCommand_->yawRateRadiansPerSecond};
            static_cast<void>(ApplySimulationCommand(velocityCommand));
            static_cast<void>(ApplySimulationCommand(yawCommand));
        }
    }

    simulation_.AdvanceOneTick();
}

dronesim::simulation::CommandResult DroneSimApp::ApplySimulationCommand(
    const dronesim::simulation::DroneCommand& command)
{
    return ApplySimulationCommand(simulation_.PrimaryDroneId(), command);
}

dronesim::simulation::CommandResult DroneSimApp::ApplySimulationCommand(
    dronesim::simulation::DroneId droneId,
    const dronesim::simulation::DroneCommand& command)
{
    return simulation_.Apply(droneId, command);
}

void DroneSimApp::RenderFrame()
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
        const bool fleetReturnCanStart = FleetReturnCanStart();
        const bool completeFleetMissionCanStart = CompleteFleetMissionCanStart();
        const bool formationNetworkActive = formationTransit_.IsActive()
            || CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive();
        dronesim::gui::HudRenderTimings hudTimings;
        hudTimings.collectGpuTimings = visibleBenchmarkMode_;
        hudTimings.frameIndex = static_cast<std::uint64_t>(renderedFrames_ + 1U);
        hud_->Render(simulation_, observerDroneId_, gamepadController_.IsConnected(),
            fleetReturnCanStart,
            [this] { static_cast<void>(StartFleetReturn()); },
            CompleteFleetMissionStatus(), completeFleetMissionCanStart,
            [this]
            {
                static_cast<void>(StartCompleteFleetMission());
            },
            formationNetwork_.get(),
            &droneSocketTransport_,
            [this, formationNetworkActive](dronesim::simulation::DroneId droneId,
                bool enabled)
            {
                if (formationNetwork_ && formationNetworkActive)
                {
                    formationNetwork_->SetNodeRadioEnabled(droneId, enabled);
                }
            },
            [this](int width, int height)
            {
                return view_->RenderScenePanel(simulation_, observerDroneId_,
                    trainingRangeScene_->BuildingObstacles(), width, height,
                    [this](const glm::mat4& view, const glm::mat4& projection,
                        const glm::vec3& cameraPosition, bool showMissionGuides,
                        std::optional<dronesim::simulation::DroneId> hiddenDroneId)
                    {
                        DrawWorld(view, projection, cameraPosition,
                            showMissionGuides, hiddenDroneId);
                    },
                    benchmarkMode_ ? &benchmarkSession_.CurrentFrame()
                        .scenePanelRenderMicroseconds : nullptr);
            },
            [this](dronesim::simulation::DroneId droneId)
            {
                return view_->RenderCameraPanel(simulation_, droneId,
                    [this](const glm::mat4& view, const glm::mat4& projection,
                        const glm::vec3& cameraPosition, bool showMissionGuides,
                        std::optional<dronesim::simulation::DroneId> hiddenDroneId)
                    {
                        DrawWorld(view, projection, cameraPosition,
                            showMissionGuides, hiddenDroneId);
                    },
                    benchmarkMode_ ? &benchmarkSession_.CurrentFrame()
                        .cameraSceneRenderMicroseconds : nullptr);
            },
            benchmarkMode_ ? &hudTimings : nullptr);
        auto& frame = benchmarkSession_.CurrentFrame();
        frame.hudInterfaceBuildMicroseconds = hudTimings.interfaceBuildMicroseconds;
        frame.hudFinalizeMicroseconds = hudTimings.frameFinalizeMicroseconds;
        frame.hudBackendDrawMicroseconds = hudTimings.backendDrawMicroseconds;
        frame.hudDrawListCount = hudTimings.drawListCount;
        frame.hudDrawCommandCount = hudTimings.drawCommandCount;
        frame.hudVertexCount = hudTimings.vertexCount;
        frame.hudIndexCount = hudTimings.indexCount;
        frame.completedHudGpuTimingCount = std::min(
            hudTimings.completedGpuFrameCount,
            benchmarkSession_.CompletedHudGpuTimings().size());
        for (std::size_t index = 0;
             index < frame.completedHudGpuTimingCount; ++index)
        {
            benchmarkSession_.CompletedHudGpuTimings()[index]
                = hudTimings.completedGpuFrames[index];
        }
    }
#endif
}

#ifdef __EMSCRIPTEN__
void DroneSimApp::UpdateObserverCamera(int width, int height)
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
            [&position](const dronesim::flight_planning::BuildingObstacle& building)
            {
                return dronesim::flight_planning::ContainsBuildingVolume(
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

void DroneSimApp::UpdateEoCamera(int width, int height)
{
    UpdateEoCamera(simulation_.PrimaryDroneId(), *eoCamera_, width, height);
}

void DroneSimApp::UpdateEoCamera(
    dronesim::simulation::DroneId droneId,
    dronesim::graphics::Camera& renderCamera,
    int width,
    int height)
{
    const auto& cameraState = simulation_.Camera(droneId);
    const glm::vec3 cameraPosition = ToRenderPosition(cameraState.positionMeters);
    const glm::dvec3 opticalAxis =
        dronesim::simulation::EoCameraModel::OpticalAxis(cameraState);
    const glm::dvec3 cameraUpAxis =
        dronesim::simulation::EoCameraModel::UpAxis(cameraState);
    const glm::vec3 viewDirection = ToRenderDirection(opticalAxis);
    const glm::vec3 cameraUp = ToRenderDirection(cameraUpAxis);
    renderCamera.SetPosition(cameraPosition);
    renderCamera.SetTarget(cameraPosition + viewDirection);
    renderCamera.SetUp(cameraUp);
    renderCamera.SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    renderCamera.SetFieldOfViewDegrees(
        static_cast<float>(cameraState.model.verticalFieldOfViewDegrees));
}
#endif

void DroneSimApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition, bool showMissionGuides,
    std::optional<dronesim::simulation::DroneId> hiddenDroneId)
{
    renderer_->SetAnimationTimeSeconds(static_cast<float>(simulation_.SimulationTimeSeconds()));
    if (trainingRangeScene_)
        trainingRangeScene_->Draw(*renderer_, *shader_, view, projection, cameraPosition);

    const auto drawRouteSegment = [this, &view, &projection, &cameraPosition](
        const glm::dvec3& startMeters, const glm::dvec3& endMeters,
        const dronesim::graphics::Material& material)
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

    for (const dronesim::simulation::DroneId id : fleetDroneIds_)
    {
        if (hiddenDroneId == id)
            continue;

        const auto& drone = simulation_.State(id);
        const glm::vec3 position = ToRenderPosition(drone.positionMeters);
        const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
            glm::mat4_cast(ToRenderOrientation(drone.orientation));
        droneModel_->Draw(heading, *renderer_, *shader_, view, projection,
            cameraPosition, &droneMaterial_, droneSurfaceTexture_.get(), true);
        // GLB에 기체 기준 장착 위치와 화면용 Y축 변환이 이미 들어 있어 기체 자세만 적용한다.
        droneGimbalModel_->Draw(heading, *renderer_, *shader_, view, projection,
            cameraPosition);

    }
}

void DroneSimApp::DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
    const dronesim::graphics::Material& material,
    const glm::mat4& view, const glm::mat4& projection)
{
    renderer_->Draw(transform, *cube_, material, *shader_, view, projection, cameraPosition);
}

}

#ifdef __EMSCRIPTEN__
extern "C" EMSCRIPTEN_KEEPALIVE void DroneSimWebFlightCommand(int command)
{
    if (dronesim::simulator::activeWebApplication == nullptr)
        return;

    using Kind = dronesim::simulation::DroneCommand::Kind;
    switch (command)
    {
    case 0:
        dronesim::simulator::activeWebApplication->QueueWebFlightCommand(Kind::TakeOff);
        break;
    case 1:
        dronesim::simulator::activeWebApplication->QueueWebFlightCommand(Kind::Stop);
        break;
    case 2:
        dronesim::simulator::activeWebApplication->QueueWebFlightCommand(Kind::Land);
        break;
    default:
        break;
    }
}

extern "C" EMSCRIPTEN_KEEPALIVE void DroneSimWebPilotInput(
    double forward, double right, double vertical, double yaw)
{
    if (dronesim::simulator::activeWebApplication == nullptr)
        return;

    dronesim::simulator::activeWebApplication->SetWebPilotAxes({
        .forward = forward,
        .right = right,
        .vertical = vertical,
        .yaw = yaw});
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebStartFleetMission()
{
    if (dronesim::simulator::activeWebApplication == nullptr)
        return 0;
    return dronesim::simulator::activeWebApplication
        ->StartWebFleetMission()
        ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebStopFleetMission()
{
    if (dronesim::simulator::activeWebApplication == nullptr)
        return 0;
    return dronesim::simulator::activeWebApplication
        ->StopWebFleetMission() ? 1 : 0;
}
#endif
