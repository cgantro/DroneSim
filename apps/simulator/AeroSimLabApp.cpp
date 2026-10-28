#include "AeroSimLabApp.h"
#include "ExecutableDirectory.h"
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
#include "media/RtpVideoStream.h"
#endif
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/ScopedDuration.h"
#include "terrain/MapBuildings.h"
#include "terrain/TerrainGrid.h"
#include "terrain/TrainingRangeTerrain.h"
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#ifndef __EMSCRIPTEN__
#include "vision/RgbFrameQuality.h"
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
// 기지와 편대 출발 위치를 시험장 남서쪽에 둔다.
constexpr glm::dvec2 demoDroneStartMeters{-205.0, -155.0};
// GLB 원점은 장착 브래킷이고 렌즈 중심은 그 원점에서 앞·위로 떨어져 있다.
constexpr glm::vec3 cameraLensCenterInModelMeters{0.015F, 0.048F, 0.030F};
constexpr double trainingRangeCameraTransitAltitudeMeters = 48.0;

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
std::unique_ptr<aerosimlab::graphics::Mesh> CreateTerrainMesh(
    const aerosimlab::terrain::TerrainGrid& terrain)
{
    const std::uint32_t count = terrain.sampleCount;
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(static_cast<std::size_t>(count) * count);
    indices.reserve(static_cast<std::size_t>(count - 1U) * (count - 1U) * 6U);

    for (std::uint32_t row = 0; row < count; ++row)
    {
        for (std::uint32_t column = 0; column < count; ++column)
        {
            const float east = (static_cast<float>(column) -
                static_cast<float>(count - 1U) * 0.5F) * terrain.spacingXMeters;
            // 격자 0행은 북쪽 끝이므로, 지도와 지형의 남북 방향을 맞춘다.
            const float north = (static_cast<float>(count - 1U) * 0.5F -
                static_cast<float>(row)) * terrain.spacingYMeters;
            // 높이 변화량으로 경사를 구해야 지형의 조명이 실제 굴곡을 따라간다.
            const std::uint32_t westColumn = column > 0U ? column - 1U : column;
            const std::uint32_t eastColumn = std::min(column + 1U, count - 1U);
            const std::uint32_t northRow = row > 0U ? row - 1U : row;
            const std::uint32_t southRow = std::min(row + 1U, count - 1U);
            const float eastDistance = static_cast<float>(eastColumn - westColumn)
                * terrain.spacingXMeters;
            const float southDistance = static_cast<float>(southRow - northRow)
                * terrain.spacingYMeters;
            const float eastSlope = (terrain.HeightAt(row, eastColumn)
                - terrain.HeightAt(row, westColumn)) / eastDistance;
            const float southSlope = (terrain.HeightAt(southRow, column)
                - terrain.HeightAt(northRow, column)) / southDistance;
            const glm::vec3 surfaceNormal = glm::normalize(
                glm::vec3{-eastSlope, 1.0F, -southSlope});
            vertices.push_back({
                {east, terrain.referenceElevationMeters + terrain.HeightAt(row, column), -north},
                surfaceNormal,
                {static_cast<float>(column) / static_cast<float>(count - 1U),
                    static_cast<float>(row) / static_cast<float>(count - 1U)}});
        }
    }

    for (std::uint32_t row = 0; row + 1U < count; ++row)
    {
        for (std::uint32_t column = 0; column + 1U < count; ++column)
        {
            const std::uint32_t northwest = row * count + column;
            const std::uint32_t northeast = northwest + 1U;
            const std::uint32_t southwest = northwest + count;
            const std::uint32_t southeast = southwest + 1U;
            indices.insert(indices.end(), {northwest, southwest, northeast,
                northeast, southwest, southeast});
        }
    }

    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
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
#else
    activeWebApplication = this;
#endif
}

AeroSimLabApp::~AeroSimLabApp()
{
#ifndef __EMSCRIPTEN__
    if (eoVideoStartWorker_.joinable())
        eoVideoStartWorker_.join();
    if (eoVideoSender_)
        eoVideoSender_->Stop();
#endif
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
    if (InspectionMissionIsActive())
        return;
    if (!commandRecording_.IsReplaying())
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{kind};
}

bool AeroSimLabApp::StartWebInspectionMission(int targetIndex)
{
    externalControl_ = true;
    if (targetIndex < -1
        || (targetIndex >= 0
            && static_cast<std::size_t>(targetIndex) >= inspectionTargets_.size()))
        return false;

    std::optional<std::size_t> selectedTarget;
    if (targetIndex >= 0)
        selectedTarget = static_cast<std::size_t>(targetIndex);
    return RequestInspectionMission(selectedTarget);
}

bool AeroSimLabApp::PauseWebInspectionMission()
{
    externalControl_ = true;
    return PauseInspectionMission();
}

bool AeroSimLabApp::ResumeWebInspectionMission()
{
    externalControl_ = true;
    return ResumeInspectionMission();
}

bool AeroSimLabApp::CancelWebInspectionMission()
{
    externalControl_ = true;
    return CancelInspectionMission();
}

void AeroSimLabApp::SetWebPilotAxes(
    const aerosimlab::manual_control::GamepadAxes& axes)
{
    externalControl_ = true;
    if (InspectionMissionIsActive())
    {
        webPilotAxes_.reset();
        pendingPilotCommand_.reset();
        return;
    }
    if (!commandRecording_.IsReplaying())
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
        EM_ASM({ window.setSimulatorStartupStatus("지형 자료를 읽고 있습니다."); });
        LoadSceneModels();
        LoadTerrain();
        return false;
    case 3:
        EM_ASM({ window.setSimulatorStartupStatus("시설물 형상을 만들고 있습니다."); });
        LoadMapFeatures();
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
            "AeroSimLab | 드론 관찰 · EO 영상 | © OpenStreetMap contributors",
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
    LoadTerrain();
    LoadMapFeatures();
    InitializeStartingState();
}

void AeroSimLabApp::LoadSceneModels()
{
    cube_ = aerosimlab::graphics::Mesh::CreateCube(1.0F);
    droneModel_ = aerosimlab::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/visual.glb").string());
    droneCameraModel_ = aerosimlab::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/action-camera.glb").string());
    droneSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadFromFile(
        AssetPath("models/drones/x500/carbon-weave.ppm"));
}

void AeroSimLabApp::LoadTerrain()
{
    terrainGrid_ = aerosimlab::terrain::CreateTrainingRangeTerrain();
    simulation_.SetTerrain(terrainGrid_);
    terrainMesh_ = CreateTerrainMesh(terrainGrid_);
}
void AeroSimLabApp::LoadMapFeatures()
{
    SetStartupDetail("훈련장 시설물과 충돌 모양을 준비하고 있습니다.");
    trainingRangeScene_ = std::make_unique<TrainingRangeScene>();
    trainingRangeScene_->Load(AssetPath(""), terrainGrid_, simulation_);
    routeBuildings_ = trainingRangeScene_->Obstacles();
    cameraTransitAltitudeMeters_ = trainingRangeCameraTransitAltitudeMeters;
    inspectionTargets_.clear();
    inspectionOccluders_.clear();

    for (const auto& obstacle : routeBuildings_)
    {
        double bottomMeters = obstacle.groundElevationsMeters.front();
        for (const double height : obstacle.groundElevationsMeters)
            bottomMeters = std::min(bottomMeters, height);
        inspectionOccluders_.push_back({
            obstacle.outlineMeters, bottomMeters, bottomMeters + obstacle.heightMeters});
    }

    for (const auto& point : trainingRangeScene_->ReconPoints())
    {
        const double groundMeters = terrainGrid_.HeightAtLocalMeters(point.positionMeters);
        InspectionTarget target;
        target.facilityId = "industrial-training-range";
        target.displayName = point.id;
        target.zoneId = point.id;
        target.buildingOsmId = point.id;
        target.surface = {
            .centerMeters = {point.positionMeters, groundMeters + 2.0},
            .normal = {0.0, 0.0, 1.0},
            .widthAxis = {1.0, 0.0, 0.0},
            .widthMeters = 8.0,
            .heightMeters = 6.0,
            .minimumDistanceMeters = 7.0,
            .maximumDistanceMeters = 35.0,
            .maximumIncidenceAngleDegrees = 65.0,
            .minimumPixelWidth = 32,
            .minimumPixelHeight = 24};
        target.routeStartMeters = {demoDroneStartMeters,
            terrainGrid_.HeightAtLocalMeters(demoDroneStartMeters) + 4.0};
        target.approachPositionMeters = {point.positionMeters,
            groundMeters + 15.0};
        target.routeAutomaticallyPlanned = true;
        inspectionTargets_.push_back(std::move(target));
    }

#ifdef __EMSCRIPTEN__
    EM_ASM({
        if (window.clearDroneSimInspectionTargets)
            window.clearDroneSimInspectionTargets();
    });
    for (std::size_t index = 0; index < inspectionTargets_.size(); ++index)
    {
        const InspectionTarget& target = inspectionTargets_[index];
        EM_ASM({
            if (window.addDroneSimInspectionTarget)
                window.addDroneSimInspectionTarget(
                    $0, UTF8ToString($1), UTF8ToString($2), UTF8ToString($3));
        }, static_cast<int>(index), target.facilityId.c_str(),
            target.zoneId.c_str(), target.displayName.c_str());
    }
#endif
}
void AeroSimLabApp::InitializeStartingState()
{
    if (inspectionTargets_.empty())
        throw std::runtime_error("훈련장 정찰 지점을 준비하지 못했습니다.");
    const double startElevation = terrainGrid_.HeightAtLocalMeters(demoDroneStartMeters);
    const glm::dvec2 toFirstTarget = glm::dvec2{
        inspectionTargets_.front().surface.centerMeters.x,
        inspectionTargets_.front().surface.centerMeters.y} - demoDroneStartMeters;
    const double initialYawRadians = std::atan2(toFirstTarget.y, toFirstTarget.x);
    simulation_.SetInitialPose({demoDroneStartMeters.x, demoDroneStartMeters.y,
        startElevation +
            aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
        initialYawRadians);
    for (const glm::dvec2 offset : std::array<glm::dvec2, 3>{
             glm::dvec2{10.0, 0.0}, glm::dvec2{0.0, 10.0}, glm::dvec2{10.0, 10.0}})
    {
        const glm::dvec2 position = demoDroneStartMeters + offset;
        const double groundMeters = terrainGrid_.HeightAtLocalMeters(position);
        static_cast<void>(simulation_.AddDrone(
            {position, groundMeters
                + aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
            initialYawRadians, groundMeters));
    }
    fleetDroneIds_ = simulation_.DroneIds();
}

aerosimlab::simulation::CaptureAssessment AeroSimLabApp::AssessInspectionView() const
{
    return aerosimlab::simulation::EoCameraModel::AssessCapture(
        simulation_.Camera(), inspectionMission_.surface, {}, inspectionOccluders_);
}

void AeroSimLabApp::SetActiveInspectionTarget(std::size_t index)
{
    if (index >= inspectionTargets_.size())
        throw std::out_of_range("점검 구역 번호가 임무 범위를 벗어났습니다.");

    inspectionMission_.activeTargetIndex = index;
    const InspectionTarget& target = inspectionTargets_[index];
    inspectionMission_.facilityId = target.facilityId;
    inspectionMission_.targetDisplayName = target.displayName;
    inspectionMission_.zoneId = target.zoneId;
    inspectionMission_.surface = target.surface;
    inspectionMission_.routeWaypointIndex = 0;
    inspectionMission_.goalMeters = target.routeWaypointsMeters.empty()
        ? target.approachPositionMeters
        : target.routeWaypointsMeters.front();
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
    if (demoMode_ && !externalControl_
        && inspectionMission_.stage == InspectionMissionStage::Ready)
        static_cast<void>(RequestInspectionMission());
    AdvanceInspectionMission();
    ExpireGroundControlCommand();
    const bool groundControlTickAdvanced = ProcessPendingGroundCommand();
    const bool fleetMissionActive = std::any_of(
        fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return entry.second.status == "경로 전달 대기"
                || entry.second.status == "출발 준비"
                || entry.second.status == "정찰 이동 중"
                || entry.second.status == "집결 이동 중"
                || entry.second.status == "재집결 준비"
                || entry.second.status == "복귀 준비"
                || entry.second.status == "복귀 중"
                || entry.second.status == "착륙 중"
                || entry.second.status == "복귀 착륙 중";
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
    const bool inspectionMissionActive =
        inspectionMission_.stage == InspectionMissionStage::StartRequested
        || inspectionMission_.stage == InspectionMissionStage::Paused
        || inspectionMission_.stage == InspectionMissionStage::TakingOff
        || inspectionMission_.stage == InspectionMissionStage::Approaching
        || inspectionMission_.stage == InspectionMissionStage::AligningCamera
        || inspectionMission_.stage == InspectionMissionStage::Capturing
        || inspectionMission_.stage == InspectionMissionStage::Landing;
    if (inspectionMissionActive)
    {
        simulation_.AdvanceOneTick();
        commandRecording_.FinishTick();
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
    return ApplyAndRecordCommand(simulation_.PrimaryDroneId(), command);
}

aerosimlab::simulation::CommandResult AeroSimLabApp::ApplyAndRecordCommand(
    aerosimlab::simulation::DroneId droneId,
    const aerosimlab::simulation::DroneCommand& command)
{
    const auto result = simulation_.Apply(droneId, command);
    if (result == aerosimlab::simulation::CommandResult::Accepted)
    {
        if (droneId == simulation_.PrimaryDroneId())
            commandRecording_.Record(command);
    }
    return result;
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
        const glm::dvec3& dronePosition = simulation_.State().positionMeters;
        const glm::dvec3& capturePosition = inspectionTargets_[
            inspectionMission_.activeTargetIndex].approachPositionMeters;
        const double missionDistanceMeters = std::hypot(
            capturePosition.x - dronePosition.x,
            capturePosition.y - dronePosition.y);
        const bool missionCanStart = inspectionMission_.stage
                == InspectionMissionStage::Ready
            || inspectionMission_.stage == InspectionMissionStage::Saved
            || inspectionMission_.stage == InspectionMissionStage::GeometryOnly
            || inspectionMission_.stage == InspectionMissionStage::Failed;
        const bool fleetReconnaissanceCanStart = FleetReconnaissanceCanStart();
        const bool fleetReturnCanStart = FleetReturnCanStart();
        const bool completeFleetMissionCanStart = CompleteFleetMissionCanStart();
        const bool formationMissionCanStart = fleetReconnaissanceCanStart;
        const bool formationNetworkActive = formationTransit_.active
            || CompleteFleetMissionIsActive() || FleetReconnaissanceIsActive();
        const auto captureAssessment = AssessInspectionView();
        HudRenderTimings hudTimings;
        hudTimings.collectGpuTimings = visibleBenchmarkMode_;
        hudTimings.frameIndex = static_cast<std::uint64_t>(renderedFrames_ + 1U);
        hud_->Render(simulation_, gamepadConnected_, groundControlConnected,
            missionDistanceMeters, inspectionMission_.targetDisplayName,
            InspectionMissionStatus(),
            missionCanStart && fleetReconnaissanceCanStart,
            [this] { static_cast<void>(RequestInspectionMission()); },
            FleetReconnaissanceStatus(), fleetReconnaissanceCanStart,
            [this] { static_cast<void>(StartFleetReconnaissance()); },
            fleetReturnCanStart,
            [this] { static_cast<void>(StartFleetReturn()); },
            CompleteFleetMissionStatus(), completeFleetMissionCanStart,
            [this](aerosimlab::simulation::FormationShape shape)
            {
                static_cast<void>(StartCompleteFleetMission(shape));
            },
            formationTransit_.status, formationMissionCanStart,
            formationNetworkActive,
            [this](aerosimlab::simulation::FormationShape shape)
            {
                static_cast<void>(StartFormationTransit(shape));
            },
            fleetPacketLossPercent_,
            [this](float percentage)
            {
                fleetPacketLossPercent_ = percentage;
                if (formationNetwork_)
                    formationNetwork_->SetPacketLossProbability(
                        static_cast<double>(percentage) / 100.0);
            },
            formationNetwork_.get(),
            [this, formationNetworkActive](aerosimlab::simulation::DroneId first,
                aerosimlab::simulation::DroneId second, bool disabled)
            {
                if (formationNetwork_ && formationNetworkActive)
                    formationNetwork_->SetLinkDisabled(first, second, disabled);
            },
            captureAssessment,
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
    const auto& state = simulation_.State();
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const float yaw = static_cast<float>(state.yawRadians);
    const glm::vec3 forward{std::cos(yaw), 0.0F, -std::sin(yaw)};
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    constexpr double followDistanceMeters = 0.40;
    constexpr double followHeightMeters = 0.28;
    glm::dvec3 observerPositionMeters = state.positionMeters
        - glm::dvec3{std::cos(state.yawRadians), std::sin(state.yawRadians), 0.0}
            * followDistanceMeters
        + glm::dvec3{0.0, 0.0, followHeightMeters};
    // 카메라가 건물 안에 놓이면 선분을 따라 기체 쪽의 첫 안전 위치로 옮긴다.
    if (const auto clearPosition = aerosimlab::terrain::FindFirstPointOutsideMapBuildings(
        observerPositionMeters, state.positionMeters, routeBuildings_, 0.5))
        observerPositionMeters = *clearPosition;
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
    renderer_->Draw(glm::mat4{1.0F}, *terrainMesh_, groundMaterial_, *shader_,
        view, projection, cameraPosition);
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

    if (showMissionGuides
        && inspectionMission_.activeTargetIndex < inspectionTargets_.size())
    {
        const InspectionTarget& target =
            inspectionTargets_[inspectionMission_.activeTargetIndex];
        glm::dvec3 previous = target.routeStartMeters;
        for (const glm::dvec3& waypoint : target.routeWaypointsMeters)
        {
            drawRouteSegment(previous, waypoint, missionRouteMaterial_);
            previous = waypoint;
        }
        drawRouteSegment(previous, target.approachPositionMeters, missionRouteMaterial_);

        const glm::dvec3 verticalAxis = glm::normalize(glm::cross(
            target.surface.normal, target.surface.widthAxis));
        const glm::dvec3 halfWidth = target.surface.widthAxis
            * (target.surface.widthMeters * 0.5);
        const glm::dvec3 halfHeight = verticalAxis
            * (target.surface.heightMeters * 0.5);
        const glm::dvec3 center = target.surface.centerMeters;
        const std::array<glm::dvec3, 4> captureCorners{
            center - halfWidth - halfHeight,
            center + halfWidth - halfHeight,
            center + halfWidth + halfHeight,
            center - halfWidth + halfHeight};
        for (std::size_t corner = 0; corner < captureCorners.size(); ++corner)
        {
            drawRouteSegment(captureCorners[corner],
                captureCorners[(corner + 1U) % captureCorners.size()],
                inspectionTargetMaterial_);
        }
        const glm::vec3 targetMarker = ToRenderPosition(center);
        DrawCube(glm::translate(glm::mat4{1.0F}, targetMarker)
            * glm::scale(glm::mat4{1.0F}, glm::vec3{0.35F}),
            cameraPosition, inspectionTargetMaterial_, view, projection);
        const glm::vec3 approachMarker = ToRenderPosition(target.approachPositionMeters);
        DrawCube(glm::translate(glm::mat4{1.0F}, approachMarker)
            * glm::scale(glm::mat4{1.0F}, glm::vec3{0.65F}),
            cameraPosition, missionRouteMaterial_, view, projection);
    }

    if (showMissionGuides)
    {
        for (const auto& [droneId, mission] : fleetReconnaissance_)
        {
            if (mission.status != "출발 준비" && mission.status != "이동 중"
                && mission.status != "착륙 중")
                continue;
            glm::dvec3 previous = mission.startPositionMeters;
            for (const glm::dvec3& waypoint : mission.routeWaypointsMeters)
            {
                drawRouteSegment(previous, waypoint, missionRouteMaterial_);
                previous = waypoint;
            }
            drawRouteSegment(previous,
                inspectionTargets_[mission.targetIndex].approachPositionMeters,
                missionRouteMaterial_);
            const glm::vec3 marker = ToRenderPosition(
                inspectionTargets_[mission.targetIndex].approachPositionMeters);
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
        const auto& cameraState = simulation_.Camera(id);
        const glm::vec3 cameraForward = glm::normalize(ToRenderDirection(
            aerosimlab::simulation::EoCameraModel::OpticalAxis(cameraState)));
        const glm::vec3 cameraUp = glm::normalize(ToRenderDirection(
            aerosimlab::simulation::EoCameraModel::UpAxis(cameraState)));
        // 이 GLB의 렌즈는 모델의 -Z 방향을 향한다. 렌즈가 비행 방향을 보도록
        // 카메라 모델의 뒤쪽(+Z)을 광축의 반대편에 맞춘다.
        const glm::vec3 cameraBackward = -cameraForward;
        const glm::vec3 cameraRight = glm::normalize(
            glm::cross(cameraForward, cameraUp));
        const glm::vec3 correctedCameraUp = glm::normalize(
            glm::cross(cameraBackward, cameraRight));
        glm::mat3 cameraOrientation{1.0F};
        cameraOrientation[0] = cameraRight;
        cameraOrientation[1] = correctedCameraUp;
        cameraOrientation[2] = cameraBackward;
        constexpr float cameraModelScale = 1.0F;
        const glm::mat4 cameraMount = glm::translate(glm::mat4{1.0F},
                ToRenderPosition(cameraState.positionMeters))
            * glm::mat4_cast(glm::quat_cast(cameraOrientation))
            * glm::translate(glm::mat4{1.0F}, -cameraLensCenterInModelMeters)
            * glm::scale(glm::mat4{1.0F}, glm::vec3{cameraModelScale});
        droneCameraModel_->Draw(cameraMount, *renderer_, *shader_, view, projection,
            cameraPosition);

        if (showMissionGuides && InspectionMissionIsActive()
            && cameraState.status == aerosimlab::simulation::CameraStatus::Active)
        {
            const glm::dvec3 cameraForward = glm::normalize(
                aerosimlab::simulation::EoCameraModel::OpticalAxis(cameraState));
            const glm::dvec3 cameraUp = glm::normalize(
                aerosimlab::simulation::EoCameraModel::UpAxis(cameraState));
            const glm::dvec3 cameraRight = glm::normalize(
                glm::cross(cameraForward, cameraUp));
            constexpr double viewDistanceMeters = 10.0;
            const double halfHeightMeters = viewDistanceMeters * std::tan(
                glm::radians(cameraState.model.verticalFieldOfViewDegrees * 0.5));
            const double halfWidthMeters = halfHeightMeters
                * cameraState.model.imageWidthPixels
                / static_cast<double>(cameraState.model.imageHeightPixels);
            const glm::dvec3 farCenter = cameraState.positionMeters
                + cameraForward * viewDistanceMeters;
            const std::array<glm::dvec3, 4> corners{
                farCenter - cameraRight * halfWidthMeters - cameraUp * halfHeightMeters,
                farCenter + cameraRight * halfWidthMeters - cameraUp * halfHeightMeters,
                farCenter + cameraRight * halfWidthMeters + cameraUp * halfHeightMeters,
                farCenter - cameraRight * halfWidthMeters + cameraUp * halfHeightMeters};

            for (std::size_t corner = 0; corner < corners.size(); ++corner)
            {
                drawRouteSegment(cameraState.positionMeters, corners[corner],
                    cameraViewMaterial_);
                drawRouteSegment(corners[corner],
                    corners[(corner + 1U) % corners.size()], cameraViewMaterial_);
            }
        }
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

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebStartInspectionMission(int targetIndex)
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->StartWebInspectionMission(targetIndex)
        ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebPauseInspectionMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->PauseWebInspectionMission() ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebResumeInspectionMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->ResumeWebInspectionMission() ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebCancelInspectionMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->CancelWebInspectionMission() ? 1 : 0;
}
#endif
