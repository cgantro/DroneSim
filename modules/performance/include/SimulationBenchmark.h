#pragma once

#include <filesystem>
#include <nlohmann/json_fwd.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dronesim::performance
{
/** @brief 화면 성능 기록에 들어갈 한 번의 실행 측정값이다. */
struct ViewerBenchmarkMeasurements
{
    struct SlowFrame
    {
        std::uint64_t frameIndex = 0;
        double applicationMicroseconds = 0.0;
        double eventPollingMicroseconds = 0.0;
        double gamepadPollingMicroseconds = 0.0;
        double simulationUpdateMicroseconds = 0.0;
        double renderMicroseconds = 0.0;
        double scenePanelRenderMicroseconds = 0.0;
        double cameraSceneRenderMicroseconds = 0.0;
        double bufferSwapMicroseconds = 0.0;
        double hudInterfaceBuildMicroseconds = 0.0;
        double hudFrameFinalizeMicroseconds = 0.0;
        double hudBackendDrawMicroseconds = 0.0;
        std::size_t hudDrawListCount = 0;
        std::size_t hudDrawCommandCount = 0;
        std::size_t hudVertexCount = 0;
        std::size_t hudIndexCount = 0;
        double hudGpuExecutionMicroseconds = 0.0;
        bool hudGpuTimingAvailable = false;
    };

    bool windowVisible = false;
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    std::string graphicsRenderer;
    std::string graphicsVersion;
    double elapsedSeconds = 0.0;
    double warmupSeconds = 0.0;
    std::uint64_t simulationTicks = 0;
    std::size_t renderedFrames = 0;
    double processCpuSeconds = 0.0;
    std::vector<double> tickDurationsMicroseconds;
    std::vector<double> applicationFrameDurationsMicroseconds;
    std::vector<double> renderFrameDurationsMicroseconds;
    std::vector<SlowFrame> slowFrames;
#ifndef __EMSCRIPTEN__
    std::vector<double> physicsStepDurationsMicroseconds;
#endif
};

/** @brief 화면 성능 측정값을 읽기 쉬운 JSON 결과로 만든다. */
nlohmann::json BuildViewerBenchmarkReport(ViewerBenchmarkMeasurements measurements);

/** @brief 성능 결과를 지정한 경로에 JSON으로 저장한다. */
void SaveViewerBenchmarkReport(
    const nlohmann::json& report, const std::filesystem::path& outputPath);

}
