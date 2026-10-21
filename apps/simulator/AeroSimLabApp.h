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
#include <vector>

namespace aerosimlab::graphics
{
class Camera;
class Mesh;
class Renderer;
class Shader;
class Window;
}

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
    explicit AeroSimLabApp(bool smokeTest, std::uint16_t udpPort, bool demoMode,
        bool networkEnabled, bool benchmarkMode);
    ~AeroSimLabApp();

    /** @brief 그래픽 자원을 준비하고 창이 닫힐 때까지 모의를 실행한다. */
    int Run();

private:
    void InitializeGraphics();
    void AdvanceSimulation(double elapsedSeconds);
    void ApplyDemoCommands();
    void PollGroundControl();
    void PollGamepad();
    void ApplyGroundCommand();
    void WriteBenchmarkReport(double elapsedSeconds) const;
    [[nodiscard]] aerosimlab::communication::Telemetry CurrentTelemetry() const;
    void RenderFrame();
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
    std::size_t smokeCommandCount_ = 0;
    bool benchmarkMode_ = false;
    std::size_t renderedFrames_ = 0;
    double accumulatedSeconds_ = 0.0;
    std::clock_t benchmarkCpuStart_ = 0;
    std::vector<double> tickDurationsMicroseconds_;
    std::chrono::steady_clock::time_point startTime_;
    std::uint16_t udpPort_ = 0;
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
