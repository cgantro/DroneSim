#pragma once

#include "simulation_core/SimulationCore.h"
#include "simulation_core/EoCameraModel.h"
#include "flight_planning/BuildingObstacle.h"
#include "gui/MiniMapPanel.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <functional>
#include <span>
#include <optional>
#include <string>

struct GLFWwindow;

namespace dronesim::gui
{
class SimulatorView;

/** @brief 화면 성능 측정에 사용할 GPU 구간 결과다. */
struct HudGpuTiming
{
    std::uint64_t frameIndex = 0;
    double executionMicroseconds = 0.0;
};

/** @brief ImGui 프레임을 만들고 그리는 CPU 구간이다. */
struct HudRenderTimings
{
    double interfaceBuildMicroseconds = 0.0;
    double frameFinalizeMicroseconds = 0.0;
    double backendDrawMicroseconds = 0.0;
    std::size_t drawListCount = 0;
    std::size_t drawCommandCount = 0;
    std::size_t vertexCount = 0;
    std::size_t indexCount = 0;
    bool collectGpuTimings = false;
    std::uint64_t frameIndex = 0;
    std::array<HudGpuTiming, 4> completedGpuFrames{};
    std::size_t completedGpuFrameCount = 0;
};

/** @brief HUD에 표시할 통신 장애 시험 결과다. 실제 통신망 객체는 포함하지 않는다. */
struct FleetNetworkStatus
{
    enum class EventKind
    {
        RadioDisabled,
        RadioRestored,
        MessageDelivered,
        PacketDropped
    };

    struct Drone
    {
        simulation::DroneId id = 0;
        std::optional<std::uint16_t> socketPort;
        bool isLeader = false;
        bool radioEnabled = true;
        bool linkedToLeader = false;
    };

    struct RecentEvent
    {
        std::uint64_t tick = 0;
        EventKind kind = EventKind::PacketDropped;
        simulation::DroneId source = 0;
        simulation::DroneId destination = 0;
    };

    std::uint64_t deliveredMessages = 0;
    std::uint64_t droppedMessages = 0;
    std::vector<Drone> drones;
    std::vector<RecentEvent> recentEvents;
};

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

/** @brief 비행 상태와 편대 요약을 3D 화면 위에 표시한다. */
class SimulatorHud
{
public:
    explicit SimulatorHud(GLFWwindow* window);
    ~SimulatorHud();

    SimulatorHud(const SimulatorHud&) = delete;
    SimulatorHud& operator=(const SimulatorHud&) = delete;

    void Render(const SimulatorHudFrame& frame);

private:
    struct PendingGpuQuery
    {
        std::uint32_t query = 0;
        std::uint64_t frameIndex = 0;
        bool pending = false;
    };

    void CollectCompletedGpuTimings(HudRenderTimings* timings);

    bool initialized_ = false;
    int selectedFaultSource_ = 0;
    std::string layoutFile_;
    MiniMapPanel miniMapPanel_;
    std::array<PendingGpuQuery, 4> backendGpuQueries_{};
};
}
