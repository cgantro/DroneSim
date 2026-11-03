#pragma once

#include "simulation_core/SimulationCore.h"
#include "simulation_core/EoCameraModel.h"
#include "gui/MiniMapPanel.h"
#include "gui/ImGuiFrameBackend.h"
#include "gui/OperationsPanel.h"

#include <cstdint>
#include <functional>

struct GLFWwindow;

namespace dronesim::gui
{
class SimulatorView;

using ScenePanelRenderer = std::function<std::uint32_t(int, int)>;
using CameraPanelRenderer = std::function<std::uint32_t(simulation::DroneId)>;
using CameraGimbalSetter = std::function<void(
    simulation::DroneId, const simulation::CameraGimbalAngles&)>;

/** @brief 장면 패널이 사용할 시점과 렌더링 함수다. */
struct ScenePanelFrame
{
    const simulation::SimulationCore& simulation;
    simulation::DroneId& observerDroneId;
    SimulatorView& view;
    ScenePanelRenderer render;
};

/** @brief 카메라 패널이 사용할 시점, 영상 생성, 짐벌 조작이다. */
struct CameraPanelFrame
{
    const simulation::SimulationCore& simulation;
    simulation::DroneId& observerDroneId;
    CameraPanelRenderer render;
    CameraGimbalSetter setGimbalAngles;
};

/** @brief 화면 패널별 자료를 묶어 HUD에 전달한다. */
struct SimulatorHudFrame
{
    ScenePanelFrame scene;
    MiniMapPanelFrame miniMap;
    CameraPanelFrame camera;
    OperationsPanelFrame operations;
    HudRenderTimings* timings = nullptr;
};

/** @brief 화면 패널을 한 프레임으로 조립해 표시한다. */
class SimulatorHud
{
public:
    explicit SimulatorHud(GLFWwindow* window);
    ~SimulatorHud() = default;

    SimulatorHud(const SimulatorHud&) = delete;
    SimulatorHud& operator=(const SimulatorHud&) = delete;

    void Render(const SimulatorHudFrame& frame);

private:
    ImGuiFrameBackend frameBackend_;
    MiniMapPanel miniMapPanel_;
    OperationsPanel operationsPanel_;
};
}
