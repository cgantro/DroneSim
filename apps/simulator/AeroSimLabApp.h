#pragma once

#include "communication/Protocol.h"
#include "graphics/Material.h"
#include "manual_control/GamepadCommandMapper.h"
#include "sensor/RgbFrame.h"
#ifndef __EMSCRIPTEN__
#include "communication/UdpSocket.h"
#endif
#include "simulation_core/SimulationCore.h"

#include <glm/glm.hpp>

#include <chrono>
#include <ctime>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aerosimlab::graphics
{
class Camera;
class Mesh;
class Renderer;
class Shader;
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
/**
 * @brief 드론 모의를 진행하고 관찰 화면과 탑재 카메라 화면을 나란히 그린다.
 * @details 드론 위치는 미터, 모의 간격은 1/60초다. 왼쪽 화면은 드론 주변을 보는 관찰자 시점이고 오른쪽은 기체 앞쪽에 고정한 RGB 카메라 시점이다.
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
    };

    explicit AeroSimLabApp(Options options);
    ~AeroSimLabApp();

    /** @brief 그래픽 자원을 준비하고 창이 닫힐 때까지 모의를 실행한다. */
    int Run();

private:
    void InitializeGraphics();
    /** @brief 실제 프레임 시간을 누적해 고정 간격의 모의 Tick으로 바꾼다. */
    void AdvanceSimulation(double elapsedSeconds);
    /** @brief 한 번의 Tick에서 입력 경로의 우선순위를 정해 모의를 진행한다. */
    void AdvanceSimulationTick();
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
    void ApplyGroundCommand();
    void ApplyRecordedCommands();
    void RecordCommand(const aerosimlab::simulation::DroneCommand& command);
    void WriteBenchmarkReport(double elapsedSeconds) const;
    [[nodiscard]] aerosimlab::communication::Telemetry CurrentTelemetry() const;
    void RenderFrame();
    /** @brief 드론의 현재 자세에 맞춰 관찰 카메라와 EO 카메라를 배치한다. */
    void UpdateCameraTransforms(int observerWidth, int eoWidth, int height);
    /** @brief 새 EO 화면을 픽셀로 읽고 지원되는 빌드에서는 영상 품질도 계산한다. */
    void CaptureEoFrame(int observerWidth, int eoWidth, int height);
    void DrawWorld(const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition);
    void DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
        const aerosimlab::graphics::Material& material,
        const glm::mat4& view, const glm::mat4& projection);

    bool smokeTest_ = false;
    bool demoMode_ = false;
    bool externalControl_ = false;
    bool groundControlActive_ = false;
    bool gamepadConnected_ = false;
    bool previousTakeoffButton_ = false;
    bool previousLandButton_ = false;
    std::optional<aerosimlab::simulation::DroneCommand> pendingControllerAction_;
    std::optional<aerosimlab::manual_control::PilotCommand> pendingPilotCommand_;
    aerosimlab::sensor::RgbFrame eoFrame_;
    bool networkEnabled_ = false;
    bool smokeReplayStarted_ = false;
    bool benchmarkMode_ = false;
    std::size_t renderedFrames_ = 0;
    double accumulatedSeconds_ = 0.0;
    std::clock_t benchmarkCpuStart_ = 0;
    std::vector<double> tickDurationsMicroseconds_;
#ifndef __EMSCRIPTEN__
    std::vector<double> visionDurationsMicroseconds_;
    double totalMeanBrightness_ = 0.0;
    double totalMeanContrast_ = 0.0;
    double totalSharpness_ = 0.0;
    std::size_t visionSampleCount_ = 0;
    std::unique_ptr<aerosimlab::vision::RgbFrameAnalyzer> visionAnalyzer_;
#endif
    std::chrono::steady_clock::time_point startTime_;
    std::uint16_t udpPort_ = 0;
    struct RecordedCommand
    {
        std::uint64_t tick = 0;
        aerosimlab::simulation::DroneCommand command;
    };
    std::vector<RecordedCommand> recordedCommands_;
    std::uint64_t recordingTickCount_ = 0;
    std::uint64_t replayTick_ = 0;
    std::size_t replayCommandIndex_ = 0;
    bool recordingActive_ = false;
    bool replaying_ = false;
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
    std::unique_ptr<aerosimlab::graphics::Renderer> renderer_;
    std::unique_ptr<aerosimlab::graphics::Camera> observerCamera_;
    std::unique_ptr<aerosimlab::graphics::Camera> eoCamera_;
    std::unique_ptr<aerosimlab::graphics::Mesh> cube_;
    std::unique_ptr<aerosimlab::graphics::Shader> shader_;
    aerosimlab::graphics::Material droneMaterial_{
        glm::vec4{0.12F, 0.48F, 0.32F, 1.0F}, 0.0F, 0.75F};
    aerosimlab::graphics::Material propellerMaterial_{
        glm::vec4{0.08F, 0.09F, 0.10F, 1.0F}, 0.0F, 0.9F};
    aerosimlab::graphics::Material groundMaterial_{
        glm::vec4{0.35F, 0.39F, 0.34F, 1.0F}, 0.0F, 1.0F};
    aerosimlab::graphics::Material buildingMaterial_{
        glm::vec4{0.63F, 0.59F, 0.48F, 1.0F}, 0.0F, 0.9F};
    aerosimlab::simulation::SimulationCore simulation_;
};
}
