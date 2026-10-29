#pragma once

#include "communication/Protocol.h"
#include "communication/VirtualMeshNetwork.h"
#include "SimulationBenchmark.h"
#include "simulation_core/SimulationCore.h"
#include "simulation_core/EoCameraModel.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <functional>
#include <string>

struct GLFWwindow;

namespace aerosimlab::simulator
{
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

struct GroundControlDronePosition
{
    simulation::DroneId droneId = 0U;
    glm::dvec3 lastReceivedPositionMeters{0.0};
    std::uint64_t ageTicks = 0U;
    bool hasReport = false;
};

/** @brief 지상통제소가 마지막으로 받은 각 드론의 위치다. */
struct GroundControlStatus
{
    std::vector<GroundControlDronePosition> groundControlPositions;
};

/** @brief 비행 상태와 편대 요약을 3D 화면 위에 표시한다. */
class SimulatorHud
{
public:
    explicit SimulatorHud(GLFWwindow* window);
    ~SimulatorHud();

    SimulatorHud(const SimulatorHud&) = delete;
    SimulatorHud& operator=(const SimulatorHud&) = delete;

    void Render(const simulation::SimulationCore& simulation,
        simulation::DroneId& observerDroneId,
        bool gamepadConnected, bool groundControlConnected,
        bool fleetReturnCanStart,
        const std::function<void()>& startFleetReturn,
        const std::string& completeFleetMissionStatus,
        bool completeFleetMissionCanStart,
        const std::function<void()>& startCompleteFleetMission,
        const communication::VirtualMeshNetwork* formationNetwork,
        const std::function<void(simulation::DroneId, bool)>&
            setFormationNodeRadioEnabled,
        const GroundControlStatus& groundControlStatus,
        const std::function<std::uint32_t(int, int)>& renderScene,
        const std::function<std::uint32_t(simulation::DroneId)>& renderCamera,
        HudRenderTimings* timings = nullptr);

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
    std::array<PendingGpuQuery, 4> backendGpuQueries_{};
};
}
