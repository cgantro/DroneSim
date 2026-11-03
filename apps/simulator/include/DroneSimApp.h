#pragma once

#include "BenchmarkSession.h"
#include "manual_control/GamepadController.h"
#include "gui/SimulatorHud.h"
#include "FleetMissionController.h"
#include "graphics/Material.h"
#include "manual_control/GamepadCommandMapper.h"
#include "simulation_core/EoCameraModel.h"
#include "flight_planning/FormationPlanner.h"
#include "flight_planning/BuildingObstacle.h"
#include "simulation_core/SimulationCore.h"
#include "simulation_core/WindField.h"
#include "TrainingRangeScene.h"
#ifdef __EMSCRIPTEN__
#include "WebCameraRig.h"
#endif

#include <glm/glm.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace dronesim::graphics
{
class Mesh;
class GlbModel;
class Renderer;
class Shader;
class Texture2D;
class Window;
class X500GimbalAssembly;
}

namespace dronesim::gui
{
class SimulatorView;
}

namespace dronesim::simulator
{

/**
 * @brief 군집 훈련장을 띄우고 드론 편대와 카메라 영상을 보여 준다.
 * @details 위치 단위는 미터이며 비행 상태는 초당 60회 계산한다. 화면 갱신과 비행 계산을 분리해
 * 렌더링 속도가 달라져도 같은 시간 간격으로 비행 상태를 갱신한다.
 */
class DroneSimApp
{
public:
    /**
     * @brief 시뮬레이터의 실행 방식을 정한다.
     * @details 검사·시연·측정 목적을 이름 있는 값으로 전달해 여러 참·거짓 인자의 순서를 외우지 않게 한다.
     */
    struct Options
    {
        bool smokeTest = false;
        bool demoMode = false;
        bool benchmarkMode = false;
        /** @brief 전체 확장성 시험 없이 보이는 창에서 프레임만 측정한다. */
        bool visibleBenchmarkMode = false;
    };

    explicit DroneSimApp(Options options);
    ~DroneSimApp();

    /** @brief 그래픽 자원을 준비하고 창이 닫힐 때까지 모의를 실행한다. */
    int Run();

#ifdef __EMSCRIPTEN__
    /** @brief 웹 비행 조작 화면에서 비행 명령을 예약한다. */
    void QueueWebFlightCommand(dronesim::simulation::DroneCommand::Kind kind);
    /** @brief 네 대가 편대 이동·정찰·복귀를 하는 전체 임무를 시작한다. */
    [[nodiscard]] bool StartWebFleetMission();
    /** @brief 전체 임무를 중단하고 네 대 모두 착륙시킨다. */
    [[nodiscard]] bool StopWebFleetMission();
    /** @brief 웹 조작 버튼의 입력을 다음 비행 Tick에 적용한다. */
    void SetWebPilotAxes(const dronesim::manual_control::GamepadAxes& axes);
#endif

private:
    friend class DroneSimAppTestAccess;


    /** @brief 입력을 읽고 시뮬레이션과 화면을 한 프레임 진행한다. */
    void RunFrame(std::chrono::steady_clock::time_point& previousFrame);
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
    std::optional<dronesim::manual_control::GamepadAxes> webPilotAxes_;
#endif
    /** @brief 지형·지도 형상·드론 모형과 강체 월드의 충돌 모양을 준비한다. */
    void LoadScene();
    void LoadSceneModels();
    void LoadTerrain();
    void LoadMapFeatures();
    void InitializeStartingState();
    void ConfigureTrainingBattery(dronesim::simulation::DroneId droneId);
    /** @brief 실제 프레임 시간을 누적해 고정 간격의 모의 Tick으로 바꾼다. */
    void AdvanceSimulation(double elapsedSeconds);
    /** @brief 한 번의 Tick에서 입력 경로의 우선순위를 정해 모의를 진행한다. */
    void AdvanceSimulationTick();
    /** @brief 현재 실행 경로에서 스모크 검사를 끝낼 조건이 충족됐는지 확인한다. */
    [[nodiscard]] bool SmokeTestIsComplete() const;
    /** @brief 카메라 모의가 한 번 이상 갱신됐는지 확인한다. 픽셀은 CPU로 읽지 않는다. */
    [[nodiscard]] int ValidateSmokeTestOutput() const;
    void AdvanceManualControlTick();
    /** @brief 전체 임무를 시작하고 비행 입력을 임무에 넘긴다. */
    [[nodiscard]] bool StartCompleteFleetMission();
    [[nodiscard]] bool StartFleetReconnaissance();
    [[nodiscard]] bool StartFleetReturn();
    /** @brief 임무 시작 뒤 조종 입력 소유권과 창 제목을 함께 갱신한다. */
    void TakeMissionControl(std::string_view windowTitle);
    /** @brief 지정한 드론에 비행 명령을 적용한다. */
    [[nodiscard]] dronesim::simulation::CommandResult ApplySimulationCommand(
        const dronesim::simulation::DroneCommand& command);
    [[nodiscard]] dronesim::simulation::CommandResult ApplySimulationCommand(
        dronesim::simulation::DroneId droneId,
        const dronesim::simulation::DroneCommand& command);
    void PollGamepad();
    /** @brief 연결이 끊긴 뒤 남은 조종 입력을 지우고 필요한 정지 명령을 예약한다. */
    /**
     * @brief 버튼의 새 입력과 현재 비행 조작값을 반영한다.
     * @details 이륙·착륙 버튼은 눌린 순간 한 번 처리하고, 스틱 값은 비행 중 계속 갱신한다.
     */
    void RenderFrame();
    void DrawWorld(const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition, bool showMissionGuides,
        std::optional<dronesim::simulation::DroneId> hiddenDroneId = std::nullopt);
    void DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
        const dronesim::graphics::Material& material,
        const glm::mat4& view, const glm::mat4& projection);

    bool smokeTest_ = false;
    dronesim::simulation::FixedTickAccumulator fixedTickAccumulator_;
    bool demoMode_ = false;
    bool externalControl_ = false;
    bool missionControlOwnsInput_ = false;
    dronesim::manual_control::GamepadController gamepadController_;
    std::optional<dronesim::simulation::DroneCommand> pendingControllerAction_;
    std::optional<dronesim::manual_control::PilotCommand> pendingPilotCommand_;
    bool benchmarkMode_ = false;
    bool visibleBenchmarkMode_ = false;
    std::size_t renderedFrames_ = 0;
    dronesim::performance::BenchmarkSession benchmarkSession_;
    int framebufferWidth_ = 0;
    int framebufferHeight_ = 0;
    std::chrono::steady_clock::time_point startTime_;
#ifdef __EMSCRIPTEN__
    std::chrono::steady_clock::time_point webPreviousFrame_;
    std::chrono::steady_clock::time_point lastWebTelemetryUpdate_;
#endif
    std::unique_ptr<dronesim::graphics::Window> window_;
#ifndef __EMSCRIPTEN__
    std::unique_ptr<dronesim::gui::SimulatorHud> hud_;
#endif
    std::unique_ptr<dronesim::graphics::Renderer> renderer_;
#ifndef __EMSCRIPTEN__
    std::unique_ptr<dronesim::gui::SimulatorView> view_;
#endif
#ifdef __EMSCRIPTEN__
    std::optional<WebCameraRig> webCameraRig_;
#endif
    std::unique_ptr<dronesim::graphics::Mesh> cube_;
    std::unique_ptr<dronesim::scene::TrainingRangeScene> trainingRangeScene_;
    std::unique_ptr<dronesim::graphics::GlbModel> droneAssemblyModel_;
    // 모델을 참조하는 조립 객체는 모델보다 먼저 해제되도록 뒤에 선언한다.
    std::unique_ptr<dronesim::graphics::X500GimbalAssembly> droneGimbalAssembly_;
    std::unique_ptr<dronesim::graphics::Texture2D> droneSurfaceTexture_;
    /** @brief 3D 관찰 시점이 따라갈 드론 식별값이다. 0이면 주 드론을 따른다. */
    dronesim::simulation::DroneId observerDroneId_ = 0;
    dronesim::terrain::TerrainGrid terrainGrid_;
    std::unique_ptr<dronesim::graphics::Shader> shader_;
    dronesim::graphics::Material droneMaterial_{
        glm::vec4{1.0F}, 0.16F, 0.40F};
    dronesim::graphics::Material missionRouteMaterial_{
        glm::vec4{0.12F, 0.82F, 0.92F, 1.0F}, 0.0F, 0.48F};
    dronesim::simulation::WindField windField_;
    dronesim::simulation::SimulationCore simulation_{};
    /** @brief 장면 준비가 끝난 뒤 생성하며 임무 상태와 통신 세션을 소유한다. */
    std::unique_ptr<FleetMissionController> missionController_;
};
}
