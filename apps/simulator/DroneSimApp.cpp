#include "DroneSimApp.h"
#include "SimulatorMiniMapSnapshot.h"
#include "SimulatorWorldRenderer.h"
#ifndef __EMSCRIPTEN__
#include "gui/SimulatorView.h"
#endif

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/ScopedDuration.h"
#include "flight_planning/BuildingObstacle.h"
#include "flight_planning/FormationPlanner.h"
#include <glm/geometric.hpp>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <ctime>
#include <cmath>
#include <filesystem>
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

#ifndef __EMSCRIPTEN__
/** @brief 통신 모듈의 현재 결과를 HUD가 읽을 수 있는 값으로 복사한다. */
dronesim::gui::FleetNetworkStatus MakeFleetNetworkStatus(
    const dronesim::simulation::SimulationCore& simulation,
    const FleetNetworkSession& network,
    dronesim::simulation::DroneId leaderId)
{
    dronesim::gui::FleetNetworkStatus status;
    const auto& metrics = network.Metrics();
    status.deliveredMessages = metrics.deliveredMessages;
    status.droppedMessages = metrics.droppedMessages;

    for (const auto droneId : simulation.DroneIds())
    {
        const auto port = network.PortFor(droneId);
        const bool isLeader = droneId == leaderId;
        status.drones.push_back({droneId, port, isLeader,
            network.IsRadioEnabled(droneId),
            isLeader || network.HasDirectLink(droneId, leaderId)});
    }

    const auto events = network.Events();
    const std::size_t firstVisibleEvent = events.size() > 4U
        ? events.size() - 4U
        : 0U;
    for (std::size_t index = firstVisibleEvent; index < events.size(); ++index)
    {
        const auto& event = events[index];
        auto kind = dronesim::gui::FleetNetworkStatus::EventKind::PacketDropped;
        switch (event.kind)
        {
        case dronesim::communication::MeshEventKind::NodeRadioDisabled:
            kind = dronesim::gui::FleetNetworkStatus::EventKind::RadioDisabled;
            break;
        case dronesim::communication::MeshEventKind::NodeRadioRestored:
            kind = dronesim::gui::FleetNetworkStatus::EventKind::RadioRestored;
            break;
        case dronesim::communication::MeshEventKind::MessageDelivered:
            kind = dronesim::gui::FleetNetworkStatus::EventKind::MessageDelivered;
            break;
        case dronesim::communication::MeshEventKind::PacketDropped:
            kind = dronesim::gui::FleetNetworkStatus::EventKind::PacketDropped;
            break;
        }
        status.recentEvents.push_back({event.tick, kind,
            event.source, event.destination});
    }
    return status;
}
#endif

constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
constexpr std::size_t trainingFleetSize = 4U;
// 전력값은 임무 흐름을 시험하기 위한 가정이며 실제 X500 배터리 사양이 아니다.
constexpr dronesim::simulation::BatteryModelParameters trainingBatteryModel{
    100.0, 40.0, 120.0, 25.0};

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
    if (missionController_->HasActiveMission())
        return;
    manualControlSession_.QueueFlightAction(
        dronesim::simulation::DroneCommand{kind});
}

bool DroneSimApp::StartWebFleetMission()
{
    externalControl_ = true;
    return StartCompleteFleetMission();
}

bool DroneSimApp::StopWebFleetMission()
{
    externalControl_ = true;
    if (!missionController_->HasActiveMission())
        return false;
    missionController_->Abort("화면에서 전체 임무를 중단했습니다.");
    return true;
}

void DroneSimApp::SetWebPilotAxes(
    const dronesim::manual_control::GamepadAxes& axes)
{
    externalControl_ = true;
    if (missionController_->HasActiveMission())
    {
        manualControlSession_.ClearPendingPilotCommand();
        return;
    }
    manualControlSession_.SetWebAxes(axes);
}
#endif

void DroneSimApp::InitializeGraphics()
{
    while (!InitializeGraphicsStep())
    {
    }
}

bool DroneSimApp::InitializeGraphicsStep()
{
    // 한 번에 한 단계만 처리한다. 데스크톱은 반복해서 끝까지 진행하고,
    // 브라우저는 프레임마다 한 단계씩 실행해 화면이 응답할 기회를 얻는다.
    switch (graphicsStartupStep_)
    {
    case GraphicsStartupStep::Window:
        SetStartupDetail("화면을 준비하고 있습니다.");
        InitializeWindow();
        graphicsStartupStep_ = GraphicsStartupStep::Renderer;
        return false;
    case GraphicsStartupStep::Renderer:
        SetStartupDetail("렌더링 장치를 준비하고 있습니다.");
        InitializeRendererAndCameras();
        graphicsStartupStep_ = GraphicsStartupStep::SceneModels;
        return false;
    case GraphicsStartupStep::SceneModels:
        SetStartupDetail("도시 모델을 읽고 있습니다.");
        LoadSceneModels();
        graphicsStartupStep_ = GraphicsStartupStep::MapFeatures;
        return false;
    case GraphicsStartupStep::MapFeatures:
        SetStartupDetail("지도 충돌 형상을 준비하고 있습니다.");
        LoadMapFeatures();
        graphicsStartupStep_ = GraphicsStartupStep::Terrain;
        return false;
    case GraphicsStartupStep::Terrain:
        SetStartupDetail("지도 바닥과 물리 지면을 맞추고 있습니다.");
        RegisterSceneTerrain();
        graphicsStartupStep_ = GraphicsStartupStep::StartingState;
        return false;
    case GraphicsStartupStep::StartingState:
        SetStartupDetail("비행 시작 위치를 준비하고 있습니다.");
        InitializeStartingState();
        graphicsStartupStep_ = GraphicsStartupStep::ShaderAndHud;
        return false;
    case GraphicsStartupStep::ShaderAndHud:
        SetStartupDetail("화면 구성을 마무리하고 있습니다.");
        InitializeShaderAndHud();
        graphicsStartupStep_ = GraphicsStartupStep::Complete;
        return false;
    case GraphicsStartupStep::Complete:
        return true;
    }
    return true;
}

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
        framebufferHeight_);
#else
    webCameraRig_.emplace(
        static_cast<float>(simulation_.Camera().model.verticalFieldOfViewDegrees));
#endif
}

void DroneSimApp::InitializeShaderAndHud()
{
    shader_ = std::make_unique<dronesim::graphics::Shader>(
        "shaders/Scene.glsl");
#ifndef __EMSCRIPTEN__
    hud_ = std::make_unique<dronesim::gui::SimulatorHud>(window_->GetNativeHandle());
#endif
}

void DroneSimApp::LoadSceneModels()
{
    worldRenderer_ = std::make_unique<SimulatorWorldRenderer>();
    worldRenderer_->LoadModels();
}

void DroneSimApp::RegisterSceneTerrain()
{
    if (!trainingRangeScene_)
        throw std::logic_error("물리 지면을 만들기 전에 도시 바닥을 읽어야 합니다.");

    const auto& terrain = trainingRangeScene_->Terrain();
    simulation_.SetTerrain(terrain);

    missionController_ = std::make_unique<FleetMissionController>(
        simulation_, terrain, *trainingRangeScene_);
}
void DroneSimApp::LoadMapFeatures()
{
    trainingRangeScene_ = std::make_unique<dronesim::scene::TrainingRangeScene>();
    trainingRangeScene_->Load({});
    const auto& collisionMesh = trainingRangeScene_->CollisionMesh();
    // 장면 모듈은 충돌 자료만 준비하고, 실행 앱이 이를 물리 월드에 연결한다.
    simulation_.AddStaticObstacleMesh(
        collisionMesh.verticesMeters, collisionMesh.triangleIndices);
}
void DroneSimApp::InitializeStartingState()
{
    const auto& targets = missionController_->Environment().Targets();
    if (targets.empty())
        throw std::runtime_error("훈련장 정찰 지점을 준비하지 못했습니다.");
    const glm::dvec2 demoDroneStartMeters =
        missionController_->Environment().StagingPositionMeters();
    const auto& terrain = trainingRangeScene_->Terrain();
    const double startElevation = terrain.HeightAtLocalMeters(demoDroneStartMeters);
    const glm::dvec2 toFirstTarget = glm::dvec2{
        targets.front().approachPositionMeters.x,
        targets.front().approachPositionMeters.y} - demoDroneStartMeters;
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
        const auto formationOffsets = dronesim::flight_planning::FormationOffsets(
            trainingFleetSize, 1.5, initialYawRadians);
        for (std::size_t index = 1U; index < trainingFleetSize; ++index)
        {
            const glm::dvec2 position = demoDroneStartMeters + formationOffsets[index];
            const double groundMeters = terrain.HeightAtLocalMeters(position);
            const auto droneId = simulation_.AddDrone(
                {position, groundMeters
                    + dronesim::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
                initialYawRadians, groundMeters);
            ConfigureTrainingBattery(droneId);
        }
    }
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
    simulation_.SetWindVelocityMetersPerSecond(windField_.AdvanceOneTick());
    missionController_->AdvanceTick();
    if (demoMode_ && !externalControl_
        && missionController_->CompleteFleetMissionCanStart())
        static_cast<void>(StartCompleteFleetMission());
    const bool missionActive = missionController_->HasActiveMission();
    if (!missionActive && missionControlOwnsInput_)
    {
        missionControlOwnsInput_ = false;
        window_->SetTitle(gamepadController_.IsConnected()
            ? "DroneSim | 게임패드 연결됨"
            : "DroneSim | 드론 관찰 · EO 영상");
    }
    const bool manualInputBlocked = missionControlOwnsInput_ || missionActive;
    manualControlSession_.AdvanceTick(simulation_, manualInputBlocked);
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
    RenderWebFrame();
#else
    RenderDesktopFrame();
#endif
}

#ifdef __EMSCRIPTEN__
void DroneSimApp::RenderWebFrame()
{
    int width = 0;
    int height = 0;
    window_->GetFramebufferSize(width, height);
    if (width <= 0 || height <= 0)
        return;

    renderer_->Resize(width, height);
    // 웹 화면은 관찰 장면에 가로 3분의 2를 주고, 카메라 영상은 실제 화면 비율을 유지한다.
    const int observerWidth = width * 2 / 3;
    const int cameraWidth = width - observerWidth;
    const auto& camera = simulation_.Camera();
    const int cameraHeight = std::min(height,
        cameraWidth * static_cast<int>(camera.model.imageHeightPixels)
            / static_cast<int>(camera.model.imageWidthPixels));
    const int cameraBottom = (height - cameraHeight) / 2;
    const int cameraTop = height - cameraBottom - cameraHeight;
    webCameraRig_->UpdateObserver(simulation_, observerDroneId_,
        trainingRangeScene_->BuildingObstacles(), observerWidth, height);
    webCameraRig_->UpdateEo(simulation_, simulation_.PrimaryDroneId(),
        cameraWidth, cameraHeight);
    renderer_->BeginFrame();
    renderer_->SetSceneViewport(0, 0, observerWidth, height);
    const auto& observerCamera = webCameraRig_->ObserverCamera();
    DrawWorld(observerCamera.GetViewMatrix(), observerCamera.GetProjectionMatrix(),
        observerCamera.GetPosition(), true);
    renderer_->SetSceneViewport(observerWidth, cameraBottom, cameraWidth, cameraHeight);
    const auto& eoCamera = webCameraRig_->EoCamera();
    DrawWorld(eoCamera.GetViewMatrix(), eoCamera.GetProjectionMatrix(),
        eoCamera.GetPosition(), false, simulation_.PrimaryDroneId());
    renderer_->EndFrame();
    EM_ASM({
        if (window.copyDroneSimEoViewport) {
            window.copyDroneSimEoViewport($0, $1, $2, $3);
        }
    }, observerWidth, cameraTop, cameraWidth, cameraHeight);
}
#else
void DroneSimApp::RenderDesktopFrame()
{
    if (hud_)
    {
        const bool fleetReturnCanStart = missionController_->FleetReturnCanStart();
        const bool completeFleetMissionCanStart =
            missionController_->CompleteFleetMissionCanStart();
        const bool showMissionNetworkStatus = missionController_->HasActiveMission();
        std::optional<dronesim::gui::FleetNetworkStatus> networkStatus;
        if (missionController_->NetworkSession().IsReady() && showMissionNetworkStatus)
        {
            networkStatus = MakeFleetNetworkStatus(
                simulation_, missionController_->NetworkSession(), missionController_->State().LeaderId());
        }
        dronesim::gui::HudRenderTimings hudTimings;
        hudTimings.collectGpuTimings = visibleBenchmarkMode_;
        hudTimings.frameIndex = static_cast<std::uint64_t>(renderedFrames_ + 1U);
        const auto miniMapSnapshot = dronesim::simulator::SimulatorMiniMapSnapshot::Capture(
            simulation_, missionController_->State());
        const dronesim::gui::SimulatorView::DrawWorld drawPanelWorld =
            [this](const glm::mat4& view, const glm::mat4& projection,
                const glm::vec3& cameraPosition, bool showMissionGuides,
                std::optional<dronesim::simulation::DroneId> hiddenDroneId)
            {
                DrawWorld(view, projection, cameraPosition,
                    showMissionGuides, hiddenDroneId);
            };
        const std::string completeMissionStatus =
            missionController_->CompleteFleetMissionStatus();
        const dronesim::gui::SimulatorHudFrame hudFrame{
            .scene = {
                .simulation = simulation_,
                .observerDroneId = observerDroneId_,
                .view = *view_,
                .render = [this, drawPanelWorld](int width, int height)
                {
                    return view_->RenderScenePanel(simulation_, observerDroneId_,
                        trainingRangeScene_->BuildingObstacles(), width, height,
                        drawPanelWorld,
                        benchmarkMode_ ? &benchmarkSession_.CurrentFrame()
                            .scenePanelRenderMicroseconds : nullptr);
                }},
            .miniMap = {
                .buildings = trainingRangeScene_->BuildingObstacles(),
                .drones = miniMapSnapshot.drones,
                .routes = miniMapSnapshot.routes,
                .groundSizeMeters = trainingRangeScene_->GroundSizeMeters()},
            .camera = {
                .simulation = simulation_,
                .observerDroneId = observerDroneId_,
                .render = [this, drawPanelWorld](
                    dronesim::simulation::DroneId droneId)
                {
                    return view_->RenderCameraPanel(simulation_, droneId,
                        drawPanelWorld,
                        benchmarkMode_ ? &benchmarkSession_.CurrentFrame()
                            .cameraSceneRenderMicroseconds : nullptr);
                },
                .setGimbalAngles = [this](
                    dronesim::simulation::DroneId droneId,
                    const dronesim::simulation::CameraGimbalAngles& angles)
                {
                    simulation_.SetCameraGimbalAngles(droneId, angles);
                }},
            .operations = {
                .simulation = simulation_,
                .gamepadConnected = gamepadController_.IsConnected(),
                .fleetReturnCanStart = fleetReturnCanStart,
                .startFleetReturn = [this]
                {
                    static_cast<void>(StartFleetReturn());
                },
                .completeFleetMissionStatus = completeMissionStatus,
                .completeFleetMissionCanStart = completeFleetMissionCanStart,
                .startCompleteFleetMission = [this]
                {
                    static_cast<void>(StartCompleteFleetMission());
                },
                .fleetNetworkStatus = networkStatus ? &*networkStatus : nullptr,
                .setRadioEnabled = [this, showMissionNetworkStatus](
                    dronesim::simulation::DroneId droneId, bool enabled)
                {
                    if (showMissionNetworkStatus)
                        missionController_->SetRadioEnabled(droneId, enabled);
                }},
            .timings = benchmarkMode_ ? &hudTimings : nullptr};
        hud_->Render(hudFrame);
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
            auto& destination = benchmarkSession_.CompletedHudGpuTimings()[index];
            const auto& source = hudTimings.completedGpuFrames[index];
            destination.frameIndex = source.frameIndex;
            destination.executionMicroseconds = source.executionMicroseconds;
        }
    }
}
#endif

void DroneSimApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition, bool showMissionGuides,
    std::optional<dronesim::simulation::DroneId> hiddenDroneId)
{
    const DroneSceneRenderContext context{
        *renderer_, *shader_, view, projection, cameraPosition,
        showMissionGuides, hiddenDroneId};
    worldRenderer_->Draw(*trainingRangeScene_, simulation_, missionController_->State(),
        missionController_->Environment().Targets(), context);
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
