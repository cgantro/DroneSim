#include "AeroSimLabApp.h"
#include "BuildRevision.h"
#include "SimulationBenchmark.h"
#include "graphics/GlApi.h"
#include "graphics/Window.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <ctime>
#include <utility>

namespace aerosimlab::simulator
{
/** @brief 프레임 측정값을 모으고 60Hz 예산을 넘긴 프레임의 세부 항목을 남긴다. */
void AeroSimLabApp::RecordBenchmarkFrame(double applicationMicroseconds)
{
    benchmarkMeasurements_.renderFrameDurationsMicroseconds.push_back(
        frameMeasurements_.renderFrameMicroseconds);
    benchmarkMeasurements_.applicationFrameDurationsMicroseconds.push_back(
        applicationMicroseconds);

    for (std::size_t index = 0;
         index < frameMeasurements_.completedHudGpuTimingCount; ++index)
    {
        const aerosimlab::gui::HudGpuTiming& timing = completedHudGpuTimings_[index];
        const auto frame = std::find_if(benchmarkMeasurements_.slowFrames.begin(),
            benchmarkMeasurements_.slowFrames.end(), [&timing](const auto& candidate)
            {
                return candidate.frameIndex == timing.frameIndex;
            });
        if (frame == benchmarkMeasurements_.slowFrames.end())
            continue;

        frame->hudGpuExecutionMicroseconds = timing.executionMicroseconds;
        frame->hudGpuTimingAvailable = true;
    }

    // 60Hz에서 한 프레임에 쓸 수 있는 약 16.67ms를 넘긴 경우만 원인 분석 목록에 넣는다.
    if (applicationMicroseconds <= 1'000'000.0 / 60.0)
        return;

    benchmarkMeasurements_.slowFrames.push_back({
        .frameIndex = static_cast<std::uint64_t>(renderedFrames_),
        .applicationMicroseconds = applicationMicroseconds,
        .eventPollingMicroseconds = frameMeasurements_.eventPollingMicroseconds,
        .gamepadPollingMicroseconds = frameMeasurements_.gamepadPollingMicroseconds,
        .simulationUpdateMicroseconds = frameMeasurements_.simulationUpdateMicroseconds,
        .renderMicroseconds = frameMeasurements_.renderMicroseconds,
        .scenePanelRenderMicroseconds = frameMeasurements_.scenePanelRenderMicroseconds,
        .cameraSceneRenderMicroseconds = frameMeasurements_.cameraSceneRenderMicroseconds,
        .bufferSwapMicroseconds = frameMeasurements_.bufferSwapMicroseconds,
        .hudInterfaceBuildMicroseconds = frameMeasurements_.hudInterfaceBuildMicroseconds,
        .hudFrameFinalizeMicroseconds = frameMeasurements_.hudFinalizeMicroseconds,
        .hudBackendDrawMicroseconds = frameMeasurements_.hudBackendDrawMicroseconds,
        .hudDrawListCount = frameMeasurements_.hudDrawListCount,
        .hudDrawCommandCount = frameMeasurements_.hudDrawCommandCount,
        .hudVertexCount = frameMeasurements_.hudVertexCount,
        .hudIndexCount = frameMeasurements_.hudIndexCount});
}

void AeroSimLabApp::WriteBenchmarkReport(double elapsedSeconds) const
{
    // 재현 조건을 맞출 수 있도록 빌드, 화면 크기, 그래픽 장치, 실행량을 함께 기록한다.
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    window_->GetFramebufferSize(framebufferWidth, framebufferHeight);

    const auto glString = [](GLenum name)
    {
        return reinterpret_cast<const char*>(glGetString(name));
    };

    ViewerBenchmarkMeasurements measurements = benchmarkMeasurements_;
    measurements.sourceRevision = aerosimlab::simulator::BuildRevision();
    measurements.windowVisible = visibleBenchmarkMode_;
    measurements.warmupSeconds = benchmarkWarmupDurationSeconds;
    measurements.framebufferWidth = framebufferWidth;
    measurements.framebufferHeight = framebufferHeight;
    measurements.graphicsRenderer = glString(GL_RENDERER);
    measurements.graphicsVersion = glString(GL_VERSION);
    measurements.elapsedSeconds = elapsedSeconds;
    measurements.simulationTicks = simulation_.TickCount() - benchmarkStartSimulationTicks_;
    measurements.renderedFrames = renderedFrames_ - benchmarkStartRenderedFrames_;
    measurements.processCpuSeconds = static_cast<double>(
        std::clock() - benchmarkCpuStart_) / CLOCKS_PER_SEC;
    nlohmann::json report = BuildViewerBenchmarkReport(std::move(measurements));
    report["drone_count"] = simulation_.DroneCount();
    report["demo_mode"] = demoMode_;
    SaveViewerBenchmarkReport(report);
}
}
