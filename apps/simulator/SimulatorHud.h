#pragma once

#include "communication/Protocol.h"
#include "communication/VirtualMeshNetwork.h"
#include "SimulationBenchmark.h"
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

/** @brief 상태 패널에서 조작할 명령·영상 기록 기능이다. */
struct SessionRecordingControls
{
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
        double missionDistanceMeters,
        const std::string& inspectionTargetName,
        const std::string& inspectionMissionStatus, bool missionCanStart,
        const std::function<void()>& startInspectionMission,
        const std::string& fleetReconnaissanceStatus,
        bool fleetReconnaissanceCanStart,
        const std::function<void()>& startFleetReconnaissance,
        bool fleetReturnCanStart,
        const std::function<void()>& startFleetReturn,
        const std::string& completeFleetMissionStatus,
        bool completeFleetMissionCanStart,
        const std::function<void(simulation::FormationShape)>& startCompleteFleetMission,
        const std::string& formationMissionStatus, bool formationMissionCanStart,
        bool formationMissionActive,
        const std::function<void(simulation::FormationShape)>& startFormationTransit,
        float& packetLossPercent,
        const std::function<void(float)>& setPacketLossPercent,
        const communication::VirtualMeshNetwork* formationNetwork,
        const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
            setFormationLinkDisabled,
        const std::function<void(simulation::DroneId, bool)>&
            setFormationNodeRadioEnabled,
        simulation::CaptureAssessment captureAssessment,
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
    simulation::FormationShape selectedFormationShape_ = simulation::FormationShape::Vee;
    int selectedFaultSource_ = 0;
    int selectedFaultDestination_ = 1;
    std::string layoutFile_;
    std::array<PendingGpuQuery, 4> backendGpuQueries_{};
};
}
