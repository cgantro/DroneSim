#pragma once

#include "BenchmarkSession.h"
#include "manual_control/GamepadController.h"
#include "manual_control/ManualControlSession.h"
#include "simulation_core/SimulationCore.h"
#include "simulation_core/WindField.h"
#ifdef __EMSCRIPTEN__
#include "WebCameraRig.h"
#endif

#include <glm/glm.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>

namespace dronesim::graphics
{
class Renderer;
class Shader;
class Window;
}

namespace dronesim::gui
{
class SimulatorHud;
class SimulatorView;
}

namespace dronesim::scene
{
class TrainingRangeScene;
}

namespace dronesim::simulator
{
class FleetMissionController;
class SimulatorWorldRenderer;

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
    /** @brief 그래픽 준비 단계 하나를 실행하고 남은 단계가 있는지 알려준다. */
    [[nodiscard]] bool InitializeGraphicsStep();
    enum class GraphicsStartupStep : std::uint8_t
    {
        Window,
        Renderer,
        SceneModels,
        MapFeatures,
        Terrain,
        StartingState,
        ShaderAndHud,
        Complete
    };
    /** @brief 화면 창과 OpenGL 문맥을 만든다. */
    void InitializeWindow();
    /** @brief 장면과 카메라를 그릴 기본 장치를 만든다. */
    void InitializeRendererAndCameras();
    /** @brief 화면 셰이더와 데스크톱 HUD를 준비한다. */
    void InitializeShaderAndHud();
#ifdef __EMSCRIPTEN__
    bool webFirstFrameRendered_ = false;
    bool webGraphicsInitialized_ = false;
#endif
    GraphicsStartupStep graphicsStartupStep_ = GraphicsStartupStep::Window;
    /** @brief 지형·지도 형상·드론 모형과 강체 월드의 충돌 모양을 준비한다. */
    void LoadSceneModels();
    void RegisterSceneTerrain();
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
    /** @brief 게임패드 상태를 읽어 창 제목과 수동 조종 입력을 갱신한다. */
    void PollGamepad();
    /**
     * @brief 버튼의 새 입력과 현재 비행 조작값을 반영한다.
     * @details 이륙·착륙 버튼은 눌린 순간 한 번 처리하고, 스틱 값은 비행 중 계속 갱신한다.
     */
    void RenderFrame();
#ifdef __EMSCRIPTEN__
    /** @brief 웹 화면의 장면과 EO 영상을 브라우저 캔버스에 그린다. */
    void RenderWebFrame();
#else
    /** @brief 데스크톱 HUD가 요청한 장면·카메라 패널을 그린다. */
    void RenderDesktopFrame();
#endif
    void DrawWorld(const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition, bool showMissionGuides,
        std::optional<dronesim::simulation::DroneId> hiddenDroneId = std::nullopt);

    bool smokeTest_ = false;
    dronesim::simulation::FixedTickAccumulator fixedTickAccumulator_;
    bool demoMode_ = false;
    bool externalControl_ = false;
    bool missionControlOwnsInput_ = false;
    dronesim::manual_control::GamepadController gamepadController_;
    dronesim::manual_control::ManualControlSession manualControlSession_;
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
    std::unique_ptr<dronesim::scene::TrainingRangeScene> trainingRangeScene_;
    std::unique_ptr<SimulatorWorldRenderer> worldRenderer_;
    /** @brief 3D 관찰 시점이 따라갈 드론 식별값이다. 0이면 주 드론을 따른다. */
    dronesim::simulation::DroneId observerDroneId_ = 0;
    std::unique_ptr<dronesim::graphics::Shader> shader_;
    dronesim::simulation::WindField windField_;
    dronesim::simulation::SimulationCore simulation_{};
    /** @brief 장면 준비가 끝난 뒤 생성하며 임무 상태와 통신 세션을 소유한다. */
    std::unique_ptr<FleetMissionController> missionController_;
};
}
