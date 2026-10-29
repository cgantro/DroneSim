#pragma once

#include "communication/Protocol.h"
#include "communication/VirtualMeshNetwork.h"
#include "SimulationBenchmark.h"
#include "FaultScenarioTimeline.h"
#include "simulation_core/SimulationCore.h"
#include "simulation_core/EoCameraModel.h"
#include "simulation_core/FormationPlanner.h"

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

/** @brief 상태 패널에서 조작할 명령·영상 기록 기능이다. */
struct SessionRecordingControls
{
    std::vector<GroundControlDronePosition> groundControlPositions;
    bool commandRecording = false;
    bool commandReplay = false;
    bool canStartCommandRecording = false;
    bool canReplayCommands = false;
    bool hasRecordedCommands = false;
    std::uint64_t recordedTicks = 0;
    std::string commandStatus;
    std::function<void()> startCommandRecording;
    std::function<void()> stopCommandRecording;
    std::function<void()> replayCommands;
    bool faultScenarioRecording = false;
    bool faultScenarioReplaying = false;
    bool canStartFaultScenarioRecording = false;
    bool canStopFaultScenarioRecording = false;
    bool canSaveFaultScenario = false;
    bool canReplayFaultScenario = false;
    bool canScheduleFault = false;
    std::size_t faultScenarioEventCount = 0;
    std::size_t scheduledFaultCount = 0;
    std::vector<FaultScenarioTimeline::Event> pendingFaultEvents;
    std::string faultScenarioStatus;
    std::function<void()> startFaultScenarioRecording;
    std::function<void()> stopFaultScenarioRecording;
    std::function<void()> replayFaultScenario;
    std::function<void(FaultScenarioTimeline::Event, std::uint32_t,
        std::uint32_t)> scheduleFault;
    bool videoRecording = false;
    bool canStartVideoRecording = false;
    bool canStopVideoRecording = false;
    std::string videoStatus;
    std::function<void()> startVideoRecording;
    std::function<void()> stopVideoRecording;
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
        bool gamepadConnected, bool groundControlConnected,
        bool fleetReturnCanStart,
        const std::function<void()>& startFleetReturn,
        const std::string& completeFleetMissionStatus,
        bool completeFleetMissionCanStart,
        const std::function<void()>& startCompleteFleetMission,
        bool formationMissionActive,
        communication::VirtualMeshMode& selectedNetworkMode,
        const std::function<void(communication::VirtualMeshMode)>& setNetworkMode,
        float& packetLossPercent,
        const std::function<void(float)>& setPacketLossPercent,
        std::uint32_t& latencyJitterTicks,
        const std::function<void(std::uint32_t)>& setLatencyJitterTicks,
        const communication::VirtualMeshNetwork* formationNetwork,
        const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
            setFormationLinkDisabled,
        const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
            setFormationDirectedLinkDisabled,
        const std::function<void(simulation::DroneId, bool)>&
            setFormationNodeRadioEnabled,
        const std::function<void(simulation::DroneId, bool)>& setCameraFault,
        const std::function<void(simulation::DroneId, std::size_t, bool)>&
            setRotorMotorFault,
        const SessionRecordingControls& recordingControls,
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
    int selectedFaultDestination_ = 1;
    int selectedFaultRotor_ = 0;
    int selectedScheduleFaultKind_ = 0;
    int scheduleDelaySeconds_ = 2;
    int scheduleDurationSeconds_ = 5;
    std::string layoutFile_;
    std::array<PendingGpuQuery, 4> backendGpuQueries_{};
};
}
