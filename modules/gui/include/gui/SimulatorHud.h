#pragma once

#include "simulation_core/SimulationCore.h"
#include "simulation_core/EoCameraModel.h"
#include "flight_planning/BuildingObstacle.h"
#include "gui/MiniMapPanel.h"
#include "gui/NetworkFaultPanel.h"
#include "gui/ImGuiFrameBackend.h"

#include <cstdint>
#include <functional>
#include <span>
#include <string>

struct GLFWwindow;

namespace dronesim::gui
{
class SimulatorView;

using ScenePanelRenderer = std::function<std::uint32_t(int, int)>;
using CameraPanelRenderer = std::function<std::uint32_t(simulation::DroneId)>;
using CameraGimbalSetter = std::function<void(
    simulation::DroneId, const simulation::CameraGimbalAngles&)>;

/** @brief 한 프레임의 HUD 표시 자료와 사용자가 실행할 동작을 모은다. */
struct SimulatorHudFrame
{
    const simulation::SimulationCore& simulation;
    simulation::DroneId& observerDroneId;
    SimulatorView& sceneView;
    bool gamepadConnected = false;
    bool fleetReturnCanStart = false;
    std::function<void()> startFleetReturn;
    std::string completeFleetMissionStatus;
    bool completeFleetMissionCanStart = false;
    std::function<void()> startCompleteFleetMission;
    const FleetNetworkStatus* fleetNetworkStatus = nullptr;
    std::function<void(simulation::DroneId, bool)> setFormationNodeRadioEnabled;
    std::span<const flight_planning::BuildingObstacle> buildings;
    std::span<const MiniMapDrone> drones;
    std::span<const MiniMapRoute> routes;
    glm::dvec2 groundSizeMeters{0.0};
    ScenePanelRenderer renderScene;
    CameraPanelRenderer renderCamera;
    CameraGimbalSetter setCameraGimbalAngles;
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
    NetworkFaultPanel networkFaultPanel_;
};
}
