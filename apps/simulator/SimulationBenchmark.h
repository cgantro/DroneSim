#pragma once

#include <filesystem>
#include <nlohmann/json_fwd.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace aerosimlab::simulation
{
enum class SimulationDynamics;
}

namespace aerosimlab::simulator
{
enum class DroneScalingLayout
{
    Separated,
    Compact
};

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
    std::vector<double> applicationFrameDurationsMicroseconds;
    std::vector<double> renderFrameDurationsMicroseconds;
    std::vector<double> renderFrameDurationsWithEoReadbackMicroseconds;
    std::vector<double> renderFrameDurationsWithoutEoReadbackMicroseconds;
#ifndef __EMSCRIPTEN__
    std::vector<double> physicsStepDurationsMicroseconds;
    // GPU에 픽셀 복사를 맡기는 CPU 호출 시간이다. 실제 GPU 복사 시간은 포함하지 않는다.
    std::vector<double> eoReadbackEnqueueDurationsMicroseconds;
    // 완료 여부를 확인하고 준비된 픽셀을 CPU 메모리로 복사하는 시간이다.
    std::vector<double> eoReadbackPollDurationsMicroseconds;
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
/** @brief 선택한 비행 계산 방식과 드론 배치에서 화면 없이 측정한다. */
nlohmann::json MeasureDroneScaling(
    simulation::SimulationDynamics dynamics, DroneScalingLayout layout);

/** @brief 실제 시나리오 실행 경로에서 여러 드론의 공동 임무를 측정한다. */
nlohmann::json MeasureCooperativeMissionScaling();
#endif
}
