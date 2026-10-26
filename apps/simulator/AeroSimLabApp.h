#pragma once

#include "SimulationBenchmark.h"
#ifndef __EMSCRIPTEN__
#include "EoVideoRecorder.h"
#endif
#include "communication/Protocol.h"
#include "graphics/Material.h"
#include "manual_control/GamepadCommandMapper.h"
#include "sensor/RgbFrame.h"
#include "simulation_core/CommandRecording.h"
#include "simulation_core/EoCameraModel.h"
#ifndef __EMSCRIPTEN__
#include "communication/UdpSocket.h"
#endif
#include "simulation_core/SimulationCore.h"

#include <glm/glm.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#ifndef __EMSCRIPTEN__
#include <unordered_map>
#endif

namespace aerosimlab::graphics
{
class Camera;
class Mesh;
class MultisampleFramebuffer;
class Renderer;
class Shader;
class Texture2D;
class Window;
}

#ifndef __EMSCRIPTEN__
namespace aerosimlab::vision
{
class RgbFrameAnalyzer;
}
#endif

namespace aerosimlab::simulator
{
class SimulatorHud;

/**
 * @brief 드론 모의를 진행하고 탄천 지형·건물·도로·교량 선형과 카메라 영상을 그린다.
 * @details 드론 위치는 미터, 모의 간격은 1/60초다. 장면과 카메라 영상은 크기와 위치를 바꿀 수 있는 별도 패널에 표시한다.
 * 화면 갱신 속도와 모의 속도를 분리해, 렌더링이 느려져도 같은 고정 간격으로 드론 상태를 계산한다.
 */
class AeroSimLabApp
{
public:
    /**
     * @brief 시뮬레이터의 실행 방식을 정한다.
     * @details 검사·시연·측정 목적과 통신 설정을 이름 있는 값으로 전달해, 여러 참·거짓 인자의 순서를 외우지 않게 한다.
     */
    struct Options
    {
        bool smokeTest = false;
        std::uint16_t udpPort = aerosimlab::communication::defaultPort;
        bool demoMode = false;
        bool networkEnabled = true;
        bool benchmarkMode = false;
        /** @brief 전체 확장성 시험 없이 보이는 창에서 프레임만 측정한다. */
        bool visibleBenchmarkMode = false;
        /** @brief 지상통제의 연속 명령을 유지할 시뮬레이션 Tick 수다. 기본값은 1초다. */
        std::uint64_t groundControlCommandTimeoutTicks = 60;
    };

    explicit AeroSimLabApp(Options options);
    ~AeroSimLabApp();

    /** @brief 그래픽 자원을 준비하고 창이 닫힐 때까지 모의를 실행한다. */
    int Run();

#ifdef __EMSCRIPTEN__
    /** @brief 웹 지상통제 버튼에서 비행 명령을 예약한다. */
    void QueueWebFlightCommand(aerosimlab::simulation::DroneCommand::Kind kind);
    /** @brief 웹 지상통제에서 자동 점검 임무를 시작한다. */
    [[nodiscard]] bool StartWebInspectionMission(int targetIndex);
    /** @brief 웹 조작 버튼의 입력을 다음 비행 Tick에 적용한다. */
    void SetWebPilotAxes(const aerosimlab::manual_control::GamepadAxes& axes);
#endif

private:
    static constexpr double benchmarkWarmupDurationSeconds = 1.0;

    /** @brief 입력부터 화면 출력까지 현재 프레임에서만 쓰는 측정값이다. */
    struct CurrentFrameMeasurements
    {
        double eventPollingMicroseconds = 0.0;
        double groundControlPollingMicroseconds = 0.0;
        double gamepadPollingMicroseconds = 0.0;
        double simulationUpdateMicroseconds = 0.0;
        double renderFrameMicroseconds = 0.0;
        double renderMicroseconds = 0.0;
        double bufferSwapMicroseconds = 0.0;
        double scenePanelRenderMicroseconds = 0.0;
        double cameraSceneRenderMicroseconds = 0.0;
        double eoReadbackHandlingMicroseconds = 0.0;
        double eoFenceWaitMicroseconds = 0.0;
        double eoBufferMapMicroseconds = 0.0;
        double eoPixelCopyMicroseconds = 0.0;
        double hudInterfaceBuildMicroseconds = 0.0;
        double hudFinalizeMicroseconds = 0.0;
        double hudBackendDrawMicroseconds = 0.0;
        std::size_t hudDrawListCount = 0;
        std::size_t hudDrawCommandCount = 0;
        std::size_t hudVertexCount = 0;
        std::size_t hudIndexCount = 0;
        std::size_t completedHudGpuTimingCount = 0;
    };

    /** @brief 입력을 읽고 시뮬레이션과 화면을 한 프레임 진행한다. */
    void RunFrame(std::chrono::steady_clock::time_point& previousFrame);
    /** @brief 이번 프레임 측정값을 요약 목록과 긴 프레임 기록에 추가한다. */
    void RecordBenchmarkFrame(double applicationMicroseconds);
    /** @brief 창과 그래픽 자원을 준비한다. */
    void InitializeGraphics();
#ifdef __EMSCRIPTEN__
    /** @brief 브라우저가 화면을 갱신할 수 있도록 웹 초기화를 한 단계씩 진행한다. */
    [[nodiscard]] bool InitializeGraphicsStep();
#endif
    /** @brief 화면 창과 OpenGL 문맥을 만든다. */
    void InitializeWindow();
    /** @brief 장면과 카메라를 그릴 기본 장치를 만든다. */
    void InitializeRendererAndCameras();
    /** @brief 화면 셰이더와 데스크톱 HUD를 준비한다. */
    void InitializeShaderAndHud();
#ifdef __EMSCRIPTEN__
    bool webFirstFrameRendered_ = false;
    bool webGraphicsInitialized_ = false;
    std::uint8_t webStartupStep_ = 0;
    std::optional<aerosimlab::manual_control::GamepadAxes> webPilotAxes_;
#endif
    /** @brief 지형·지도 형상·드론 모형과 강체 월드의 충돌 모양을 준비한다. */
    void LoadScene();
    void LoadSceneModels();
    void LoadTerrain();
    void LoadMapFeatures();
    void InitializeStartingState();
    /** @brief 카메라가 가상 점검면을 담을 수 있는지 공통 센서 판정으로 확인한다. */
    [[nodiscard]] aerosimlab::simulation::CaptureAssessment AssessInspectionView() const;
    void SetActiveInspectionTarget(std::size_t index);
    /** @brief 실제 프레임 시간을 누적해 고정 간격의 모의 Tick으로 바꾼다. */
    void AdvanceSimulation(double elapsedSeconds);
    /** @brief 한 번의 Tick에서 입력 경로의 우선순위를 정해 모의를 진행한다. */
    void AdvanceSimulationTick();
    /** @brief 현재 실행 경로에서 스모크 검사를 끝낼 조건이 충족됐는지 확인한다. */
    [[nodiscard]] bool SmokeTestIsComplete() const;
    /** @brief 스모크 검사에서 마지막으로 읽은 EO 영상과 카메라 Tick을 확인한다. */
    [[nodiscard]] int ValidateSmokeTestOutput() const;
    void AdvanceReplayTick();
    void AdvanceManualControlTick();
    [[nodiscard]] bool RequestInspectionMission(
        std::optional<std::size_t> selectedTargetIndex = std::nullopt);
    void AdvanceInspectionMission();
    void FailInspectionMission(const std::string& reason);
    [[nodiscard]] std::string InspectionMissionStatus() const;
    /** @brief 명령을 적용하고 성공한 명령만 입력 기록에 남긴다. */
    [[nodiscard]] aerosimlab::simulation::CommandResult ApplyAndRecordCommand(
        const aerosimlab::simulation::DroneCommand& command);
    void PollGroundControl();
#ifndef __EMSCRIPTEN__
    void ProcessGroundDatagram(const std::string& datagram,
        const aerosimlab::communication::UdpPeer& peer);
    /** @brief 중복·역순·처리 중인 명령에 응답하고 새 명령 접수 여부를 돌려준다. */
    [[nodiscard]] bool ReplyToUnqueueableGroundCommand(
        const aerosimlab::communication::CommandPacket& packet,
        const aerosimlab::communication::UdpPeer& peer,
        aerosimlab::communication::SequenceGuard::Decision decision);
#endif
    void PollGamepad();
    /** @brief 연결이 끊긴 뒤 남은 조종 입력을 지우고 필요한 정지 명령을 예약한다. */
    void HandleGamepadDisconnected();
    /**
     * @brief 버튼의 새 입력과 현재 비행 조작값을 반영한다.
     * @details 이륙·착륙 버튼은 눌린 순간 한 번 처리하고, 스틱 값은 비행 중 계속 갱신한다.
     */
    void HandleGamepadConnected(bool takeoffPressed, bool landPressed,
        const std::optional<aerosimlab::manual_control::GamepadAxes>& pilotAxes);
    /**
     * @brief 기록 시작·정지·재생 명령을 적용한다.
     * @return true면 재생 시작 과정에서 첫 Tick까지 진행했으므로 바깥에서 한 번 더 진행하지 않는다.
     */
    [[nodiscard]] bool ApplySessionCommand(
        aerosimlab::communication::SessionCommand command,
        aerosimlab::communication::CommandReply& reply);
    void ApplyVideoRecordingCommand(
        aerosimlab::communication::VideoRecordingCommand command,
        aerosimlab::communication::CommandReply& reply);
    void ApplyDroneCommand(const aerosimlab::simulation::DroneCommand& command,
        aerosimlab::communication::CommandReply& reply);
    void ApplyCameraCommand(aerosimlab::communication::CameraCommand command,
        aerosimlab::communication::CommandReply& reply);
    /** @brief 새 드론의 지면 위치를 정하고 편대에 추가한 결과를 응답에 담는다. */
    void ApplyAddDroneCommand(aerosimlab::communication::CommandReply& reply);
    /** @brief 대기 중인 지상통제 요청을 처리하고 이번 Tick을 진행했는지 반환한다. */
    [[nodiscard]] bool ProcessPendingGroundCommand();
    void ExpireGroundControlCommand();
    void ApplyRecordedCommands();
    void WriteBenchmarkReport(double elapsedSeconds) const;
    [[nodiscard]] aerosimlab::communication::Telemetry CurrentTelemetry() const;
    void RenderFrame();
    /** @brief 화면 크기에 맞춰 관찰 카메라의 시야 비율을 갱신한다. */
    void UpdateObserverCamera(int width, int height);
    /** @brief 화면 크기에 맞춰 EO 카메라의 시야 비율을 갱신한다. */
    void UpdateEoCamera(int width, int height);
#ifndef __EMSCRIPTEN__
    [[nodiscard]] std::uint32_t RenderScenePanel(int width, int height);
    [[nodiscard]] std::uint32_t RenderCameraPanel();
    [[nodiscard]] std::uint32_t RenderWorldToTarget(
        aerosimlab::graphics::MultisampleFramebuffer& target,
        const aerosimlab::graphics::Camera& camera);
    /** @brief 끝난 픽셀 읽기를 처리한 뒤 새 카메라 화면을 GPU에 맡긴다. */
    void CaptureEoFrame();
    /** @brief GPU에서 끝난 EO 픽셀 읽기와 측정값을 처리한다. */
    void ProcessCompletedEoReadback();
    /** @brief 아직 읽지 않은 카메라 Tick이 있으면 GPU에 읽기를 예약한다. */
    void QueueEoReadback();
    /** @brief 저장된 EO 화면의 Tick과 품질을 갱신한다. */
    void ProcessEoPixels(std::uint64_t simulationTick);
    /** @brief 스모크 검사 전에 마지막 EO 화면까지 읽어 온다. */
    void DrainEoReadbacks();
#endif
    void DrawWorld(const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition);
    void DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
        const aerosimlab::graphics::Material& material,
        const glm::mat4& view, const glm::mat4& projection);

    bool smokeTest_ = false;
    aerosimlab::simulation::FixedTickAccumulator fixedTickAccumulator_;
    bool demoMode_ = false;
    struct InspectionTarget
    {
        std::string facilityId;
        std::string zoneId;
        aerosimlab::simulation::InspectionSurface surface;
        std::vector<glm::dvec3> routeWaypointsMeters;
        glm::dvec3 approachPositionMeters{0.0};
    };
    enum class InspectionMissionStage
    {
        Ready,
        StartRequested,
        TakingOff,
        Approaching,
        AligningCamera,
        Capturing,
        Landing,
        Saved,
        GeometryOnly,
        Failed
    };
    InspectionMissionStage inspectionMissionStage_ = InspectionMissionStage::Ready;
    std::uint64_t inspectionMissionStartTick_ = 0;
    std::uint64_t inspectionCaptureStartFrame_ = 0;
    std::uint64_t inspectionCaptureReadyTick_ = 0;
    bool inspectionReadyFrameSaved_ = false;
    std::string inspectionMissionError_;
    std::vector<InspectionTarget> inspectionTargets_;
    std::size_t activeInspectionTargetIndex_ = 0;
    std::size_t currentInspectionRouteWaypointIndex_ = 0;
    std::size_t inspectionMissionEndTargetIndex_ = 0;
    std::size_t inspectionMissionTargetCount_ = 0;
    std::size_t completedInspectionTargetCount_ = 0;
    bool externalControl_ = false;
    bool groundControlActive_ = false;
    std::optional<std::chrono::steady_clock::time_point> lastGroundControlMessage_;
    bool gamepadConnected_ = false;
    bool previousTakeoffButton_ = false;
    bool previousLandButton_ = false;
    bool eoReadbackHappenedThisFrame_ = false;
    int gamepadJoystickId_ = -1;
    int unsupportedJoystickId_ = -1;
    std::optional<aerosimlab::simulation::DroneCommand> pendingControllerAction_;
    std::optional<aerosimlab::manual_control::PilotCommand> pendingPilotCommand_;
    aerosimlab::sensor::RgbFrame eoFrame_;
    bool networkEnabled_ = false;
    bool smokeReplayStarted_ = false;
    bool benchmarkMode_ = false;
    bool visibleBenchmarkMode_ = false;
    std::uint64_t groundControlCommandTimeoutTicks_ = 60;
    std::optional<std::uint64_t> groundControlCommandExpiresAtTick_;
    std::size_t renderedFrames_ = 0;
    std::size_t benchmarkStartRenderedFrames_ = 0;
    std::uint64_t benchmarkStartSimulationTicks_ = 0;
    std::uint64_t lastQueuedEoFrameTick_ = 0;
#ifndef __EMSCRIPTEN__
    // 같은 센서 Tick의 EO 화면은 화면 프레임마다 반복해서 그리지 않는다.
    std::optional<std::uint64_t> lastRenderedEoFrameTick_;
#endif
    std::clock_t benchmarkCpuStart_ = 0;
    CurrentFrameMeasurements frameMeasurements_;
    std::array<HudGpuTiming, 4> completedHudGpuTimings_{};
    ViewerBenchmarkMeasurements benchmarkMeasurements_;
    int framebufferWidth_ = 0;
    int framebufferHeight_ = 0;
#ifndef __EMSCRIPTEN__
    double totalMeanBrightness_ = 0.0;
    double totalMeanContrast_ = 0.0;
    double totalSharpness_ = 0.0;
    std::unique_ptr<aerosimlab::vision::RgbFrameAnalyzer> visionAnalyzer_;
#endif
    std::chrono::steady_clock::time_point startTime_;
#ifdef __EMSCRIPTEN__
    std::chrono::steady_clock::time_point webPreviousFrame_;
    std::chrono::steady_clock::time_point lastWebTelemetryUpdate_;
#endif
    std::uint16_t udpPort_ = 0;
    aerosimlab::simulation::CommandRecording commandRecording_;
#ifndef __EMSCRIPTEN__
    struct PendingGroundCommand
    {
        aerosimlab::communication::CommandPacket packet;
        aerosimlab::communication::UdpPeer peer;
        std::uint64_t receivedAtMilliseconds = 0;
    };
    std::optional<PendingGroundCommand> pendingCommand_;
    std::optional<aerosimlab::communication::UdpPeer> activePeer_;
    std::optional<aerosimlab::communication::CommandReply> lastReply_;
    aerosimlab::communication::SequenceGuard sequenceGuard_;
    std::unique_ptr<aerosimlab::communication::UdpSocket> udpSocket_;
#endif
    std::unique_ptr<aerosimlab::graphics::Window> window_;
#ifndef __EMSCRIPTEN__
    std::unique_ptr<SimulatorHud> hud_;
#endif
    std::unique_ptr<aerosimlab::graphics::Renderer> renderer_;
#ifndef __EMSCRIPTEN__
    std::unique_ptr<aerosimlab::graphics::MultisampleFramebuffer> sceneTarget_;
    std::unique_ptr<aerosimlab::graphics::MultisampleFramebuffer> cameraTarget_;
    std::unique_ptr<EoVideoRecorder> eoVideoRecorder_;
    std::unordered_map<std::uint64_t, aerosimlab::simulation::CameraState>
        queuedEoCameraStates_;
#endif
    std::unique_ptr<aerosimlab::graphics::Camera> observerCamera_;
    std::unique_ptr<aerosimlab::graphics::Camera> eoCamera_;
    std::unique_ptr<aerosimlab::graphics::Mesh> cube_;
    std::unique_ptr<aerosimlab::graphics::Mesh> terrainMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> waterMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> roadMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> bridgeMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> bridgeFootprintMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> taggedBuildingMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> estimatedBuildingMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> droneMesh_;
    std::unique_ptr<aerosimlab::graphics::Texture2D> droneSurfaceTexture_;
    std::vector<aerosimlab::simulation::DroneId> fleetDroneIds_;
    aerosimlab::terrain::TerrainGrid terrainGrid_;
    glm::dvec3 missionGoalMeters_{0.0};
    std::string inspectionFacilityId_;
    std::string inspectionZoneId_;
    aerosimlab::simulation::InspectionSurface inspectionSurface_;
    std::vector<aerosimlab::simulation::InspectionOccluderPolygon> inspectionOccluders_;
    std::unique_ptr<aerosimlab::graphics::Shader> shader_;
    aerosimlab::graphics::Material droneMaterial_{
        glm::vec4{1.0F}, 0.16F, 0.40F};
    aerosimlab::graphics::Material groundMaterial_{
        glm::vec4{0.25F, 0.40F, 0.25F, 1.0F}, 0.0F, 0.92F};
    aerosimlab::graphics::Material waterMaterial_{
        glm::vec4{0.08F, 0.35F, 0.55F, 1.0F}, 0.0F, 0.38F};
    aerosimlab::graphics::Material roadMaterial_{
        glm::vec4{0.22F, 0.25F, 0.27F, 1.0F}, 0.0F, 0.95F};
    aerosimlab::graphics::Material bridgeMaterial_{
        glm::vec4{0.55F, 0.37F, 0.22F, 1.0F}, 0.0F, 0.88F};
    aerosimlab::graphics::Material bridgeFootprintMaterial_{
        glm::vec4{0.95F, 0.58F, 0.12F, 1.0F}, 0.0F, 0.72F};
    aerosimlab::graphics::Material buildingMaterial_{
        glm::vec4{0.57F, 0.61F, 0.65F, 1.0F}, 0.0F, 0.9F};
    aerosimlab::graphics::Material estimatedBuildingMaterial_{
        glm::vec4{0.66F, 0.54F, 0.36F, 1.0F}, 0.0F, 0.9F};
    aerosimlab::graphics::Material missionMarkerMaterial_{
        glm::vec4{1.0F, 0.65F, 0.05F, 1.0F}, 0.0F, 0.55F};
#ifndef __EMSCRIPTEN__
    aerosimlab::simulation::SimulationCore simulation_{
        aerosimlab::simulation::SimulationDynamics::RigidBody};
#else
    aerosimlab::simulation::SimulationCore simulation_;
#endif
};
}
