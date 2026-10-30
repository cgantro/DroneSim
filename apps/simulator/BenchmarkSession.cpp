#include "BenchmarkSession.h"

#include "BuildRevision.h"
#include "graphics/GlApi.h"
#include "graphics/Window.h"
#include "simulation_core/SimulationCore.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <ctime>
#include <utility>

namespace dronesim::simulator
{
void BenchmarkSession::ResetMeasurements(
    std::size_t renderedFrames, std::uint64_t simulationTicks)
{
    measurements_ = {};
    startRenderedFrames_ = renderedFrames;
    startSimulationTicks_ = simulationTicks;
    cpuStart_ = std::clock();
}

void BenchmarkSession::RecordFrame(double applicationMicroseconds,
    std::size_t renderedFrames)
{
    measurements_.renderFrameDurationsMicroseconds.push_back(
        frame_.renderFrameMicroseconds);
    measurements_.applicationFrameDurationsMicroseconds.push_back(
        applicationMicroseconds);

    for (std::size_t index = 0; index < frame_.completedHudGpuTimingCount; ++index)
    {
        const auto& timing = completedHudGpuTimings_[index];
        const auto recordedFrame = std::find_if(measurements_.slowFrames.begin(),
            measurements_.slowFrames.end(), [&timing](const auto& candidate)
            {
                return candidate.frameIndex == timing.frameIndex;
            });
        if (recordedFrame == measurements_.slowFrames.end())
            continue;
        recordedFrame->hudGpuExecutionMicroseconds = timing.executionMicroseconds;
        recordedFrame->hudGpuTimingAvailable = true;
    }

    // 60Hz에서 한 프레임 예산인 16.67ms를 넘긴 프레임만 원인 분석 목록에 남긴다.
    if (applicationMicroseconds <= 1'000'000.0 / 60.0)
        return;

    measurements_.slowFrames.push_back({
        .frameIndex = static_cast<std::uint64_t>(renderedFrames),
        .applicationMicroseconds = applicationMicroseconds,
        .eventPollingMicroseconds = frame_.eventPollingMicroseconds,
        .gamepadPollingMicroseconds = frame_.gamepadPollingMicroseconds,
        .simulationUpdateMicroseconds = frame_.simulationUpdateMicroseconds,
        .renderMicroseconds = frame_.renderMicroseconds,
        .scenePanelRenderMicroseconds = frame_.scenePanelRenderMicroseconds,
        .cameraSceneRenderMicroseconds = frame_.cameraSceneRenderMicroseconds,
        .bufferSwapMicroseconds = frame_.bufferSwapMicroseconds,
        .hudInterfaceBuildMicroseconds = frame_.hudInterfaceBuildMicroseconds,
        .hudFrameFinalizeMicroseconds = frame_.hudFinalizeMicroseconds,
        .hudBackendDrawMicroseconds = frame_.hudBackendDrawMicroseconds,
        .hudDrawListCount = frame_.hudDrawListCount,
        .hudDrawCommandCount = frame_.hudDrawCommandCount,
        .hudVertexCount = frame_.hudVertexCount,
        .hudIndexCount = frame_.hudIndexCount});
}

void BenchmarkSession::AddTick(double durationMicroseconds)
{
    measurements_.tickDurationsMicroseconds.push_back(durationMicroseconds);
}

void BenchmarkSession::AddPhysicsStep(double durationMicroseconds)
{
#ifndef __EMSCRIPTEN__
    measurements_.physicsStepDurationsMicroseconds.push_back(durationMicroseconds);
#else
    static_cast<void>(durationMicroseconds);
#endif
}

void BenchmarkSession::WriteReport(const graphics::Window& window,
    const simulation::SimulationCore& simulation, bool demoMode,
    bool visible, double elapsedSeconds, std::size_t renderedFrames) const
{
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    window.GetFramebufferSize(framebufferWidth, framebufferHeight);

    const auto glString = [](GLenum name)
    {
        return reinterpret_cast<const char*>(glGetString(name));
    };

    ViewerBenchmarkMeasurements report = measurements_;
    report.sourceRevision = BuildRevision();
    report.windowVisible = visible;
    report.warmupSeconds = warmupSeconds;
    report.framebufferWidth = framebufferWidth;
    report.framebufferHeight = framebufferHeight;
    report.graphicsRenderer = glString(GL_RENDERER);
    report.graphicsVersion = glString(GL_VERSION);
    report.elapsedSeconds = elapsedSeconds;
    report.simulationTicks = simulation.TickCount() - startSimulationTicks_;
    report.renderedFrames = renderedFrames - startRenderedFrames_;
    report.processCpuSeconds = static_cast<double>(
        std::clock() - cpuStart_) / CLOCKS_PER_SEC;
    nlohmann::json output = BuildViewerBenchmarkReport(std::move(report));
    output["drone_count"] = simulation.DroneCount();
    output["demo_mode"] = demoMode;
    SaveViewerBenchmarkReport(output);
}
}
