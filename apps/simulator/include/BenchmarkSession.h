#pragma once

#include "SimulationBenchmark.h"
#include "gui/SimulatorHud.h"

#include <array>
#include <ctime>

namespace dronesim::graphics
{
class Window;
}

namespace dronesim::simulation
{
class SimulationCore;
}

namespace dronesim::simulator
{
/** @brief 화면 성능 측정 자료와 결과 파일 생성을 앱 실행 상태에서 분리해 맡는다. */
class BenchmarkSession final
{
public:
    struct Frame
    {
        double eventPollingMicroseconds = 0.0;
        double gamepadPollingMicroseconds = 0.0;
        double simulationUpdateMicroseconds = 0.0;
        double renderFrameMicroseconds = 0.0;
        double renderMicroseconds = 0.0;
        double bufferSwapMicroseconds = 0.0;
        double scenePanelRenderMicroseconds = 0.0;
        double cameraSceneRenderMicroseconds = 0.0;
        double hudInterfaceBuildMicroseconds = 0.0;
        double hudFinalizeMicroseconds = 0.0;
        double hudBackendDrawMicroseconds = 0.0;
        std::size_t hudDrawListCount = 0;
        std::size_t hudDrawCommandCount = 0;
        std::size_t hudVertexCount = 0;
        std::size_t hudIndexCount = 0;
        std::size_t completedHudGpuTimingCount = 0;
    };

    static constexpr double warmupSeconds = 1.0;

    Frame& CurrentFrame() noexcept { return frame_; }
    std::array<gui::HudGpuTiming, 4>& CompletedHudGpuTimings() noexcept
    {
        return completedHudGpuTimings_;
    }

    void ResetFrame() noexcept { frame_ = {}; }
    void ResetMeasurements(std::size_t renderedFrames, std::uint64_t simulationTicks);
    void RecordFrame(double applicationMicroseconds, std::size_t renderedFrames);
    void AddTick(double durationMicroseconds);
    void AddPhysicsStep(double durationMicroseconds);
    void WriteReport(const graphics::Window& window,
        const simulation::SimulationCore& simulation, bool demoMode,
        bool visible, double elapsedSeconds, std::size_t renderedFrames) const;

private:
    Frame frame_;
    std::array<gui::HudGpuTiming, 4> completedHudGpuTimings_{};
    ViewerBenchmarkMeasurements measurements_;
    std::size_t startRenderedFrames_ = 0;
    std::uint64_t startSimulationTicks_ = 0;
    std::clock_t cpuStart_ = 0;
};
}
