#pragma once

#include "SimulationBenchmark.h"
#include "communication/Protocol.h"
#include "graphics/Material.h"
#include "manual_control/GamepadCommandMapper.h"
#include "sensor/RgbFrame.h"
#include "simulation_core/CommandRecording.h"
#ifndef __EMSCRIPTEN__
#include "communication/UdpSocket.h"
#endif
#include "simulation_core/SimulationCore.h"

#include <glm/glm.hpp>

#include <chrono>
#include <cstdint>
#include <ctime>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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
 * @brief 드론 모의를 진행하고 장면과 탑재 카메라 영상을 시뮬레이터 창에 그린다.
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
        /** @brief 지상통제의 연속 명령을 유지할 시뮬레이션 Tick 수다. 기본값은 1초다. */
        std::uint64_t groundControlCommandTimeoutTicks = 60;
    };

    explicit AeroSimLabApp(Options options);
    ~AeroSimLabApp();

    /** @brief 그래픽 자원을 준비하고 창이 닫힐 때까지 모의를 실행한다. */
    int Run();

private:
    /** @brief 입력을 읽고 시뮬레이션과 화면을 한 프레임 진행한다. */
    void RunFrame(std::chrono::steady_clock::time_point& previousFrame);
    void InitializeGraphics();
    /** @brief 지도·기체 화면 자산을 불러오고 강체 월드의 충돌 모양을 준비한다. */
    void LoadScene();
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
    void ApplyDemoCommands();
    void PollGroundControl();
#ifndef __EMSCRIPTEN__
    void ProcessGroundDatagram(const std::string& datagram,
        const aerosimlab::communication::UdpPeer& peer);
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
    void ApplyDroneCommand(const aerosimlab::simulation::DroneCommand& command,
        aerosimlab::communication::CommandReply& reply);
    void ApplyCameraCommand(aerosimlab::communication::CameraCommand command,
        aerosimlab::communication::CommandReply& reply);
    /** @brief 새 드론의 지면 위치를 정하고 편대에 추가한 결과를 응답에 담는다. */
    void ApplyAddDroneCommand(aerosimlab::communication::CommandReply& reply);
    void ApplyGroundCommand();
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
    /** @brief 완료된 EO 픽셀 읽기를 처리하고 새 화면 읽기를 GPU에 맡긴다. */
    void CaptureEoFrame();
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
    std::uint64_t groundControlCommandTimeoutTicks_ = 60;
    std::optional<std::uint64_t> groundControlCommandExpiresAtTick_;
    std::size_t renderedFrames_ = 0;
    std::uint64_t lastQueuedEoFrameTick_ = 0;
    std::clock_t benchmarkCpuStart_ = 0;
    ViewerBenchmarkMeasurements benchmarkMeasurements_;
#ifndef __EMSCRIPTEN__
    double totalMeanBrightness_ = 0.0;
    double totalMeanContrast_ = 0.0;
    double totalSharpness_ = 0.0;
    std::unique_ptr<aerosimlab::vision::RgbFrameAnalyzer> visionAnalyzer_;
#endif
    std::chrono::steady_clock::time_point startTime_;
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
#endif
    std::unique_ptr<aerosimlab::graphics::Camera> observerCamera_;
    std::unique_ptr<aerosimlab::graphics::Camera> eoCamera_;
    std::unique_ptr<aerosimlab::graphics::Mesh> cube_;
    std::unique_ptr<aerosimlab::graphics::Mesh> droneMesh_;
    std::unique_ptr<aerosimlab::graphics::Mesh> terrainMesh_;
    std::unique_ptr<aerosimlab::graphics::Texture2D> terrainTexture_;
    std::unique_ptr<aerosimlab::graphics::Texture2D> droneSurfaceTexture_;
    std::unique_ptr<aerosimlab::graphics::Mesh> facilityMesh_;
    std::vector<glm::mat4> buildingTransforms_;
    std::vector<aerosimlab::simulation::DroneId> fleetDroneIds_;
    aerosimlab::terrain::TerrainGrid terrainGrid_;
    glm::dvec3 missionGoalMeters_{0.0};
    std::unique_ptr<aerosimlab::graphics::Shader> shader_;
    aerosimlab::graphics::Material droneMaterial_{
        glm::vec4{1.0F}, 0.16F, 0.40F};
    aerosimlab::graphics::Material groundMaterial_{
        glm::vec4{1.0F}, 0.0F, 0.92F};
    aerosimlab::graphics::Material buildingMaterial_{
        glm::vec4{0.63F, 0.59F, 0.48F, 1.0F}, 0.0F, 0.9F};
    aerosimlab::graphics::Material facilityMaterial_{
        glm::vec4{0.48F, 0.52F, 0.38F, 1.0F}, 0.0F, 0.85F};
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
