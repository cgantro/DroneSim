#pragma once

#include <filesystem>
#include <nlohmann/json_fwd.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace aerosimlab::simulator
{
/** @brief 실행 시간 표본에서 p50·p95·p99와 표본 수를 계산한다. */
nlohmann::json SummarizeDurations(std::vector<double> durations);

/** @brief 화면 성능 기록에 들어갈 한 번의 실행 측정값이다. */
struct ViewerBenchmarkMeasurements
{
    std::string sourceRevision;
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    std::string graphicsRenderer;
    std::string graphicsVersion;
    double elapsedSeconds = 0.0;
    std::uint64_t simulationTicks = 0;
    std::size_t renderedFrames = 0;
    double processCpuSeconds = 0.0;
    std::vector<double> tickDurationsMicroseconds;
    std::vector<double> renderFrameDurationsMicroseconds;
#ifndef __EMSCRIPTEN__
    std::vector<double> physicsStepDurationsMicroseconds;
    std::vector<double> visionDurationsMicroseconds;
    double meanBrightness = 0.0;
    double meanContrast = 0.0;
    double meanSharpness = 0.0;
    std::size_t visionSampleCount = 0;
#endif
};

/** @brief 화면 성능 측정값을 읽기 쉬운 JSON 결과로 만든다. */
nlohmann::json BuildViewerBenchmarkReport(ViewerBenchmarkMeasurements measurements);

/** @brief 성능 결과를 실행 파일 옆에 저장한다. 시험에서는 저장 경로를 따로 지정할 수 있다. */
void SaveViewerBenchmarkReport(
    const nlohmann::json& report, const std::filesystem::path& outputPath = {});

#ifndef __EMSCRIPTEN__
/** @brief 화면 없이 1·2·4·8대 강체 모의를 재고 실행별 결과를 돌려준다. */
nlohmann::json MeasureDroneScaling();
#endif
}
