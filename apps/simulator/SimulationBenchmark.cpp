#include "SimulationBenchmark.h"
#include "ExecutableDirectory.h"
#include "BenchmarkProcessMemory.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
using detail::ReadProcessMemorySample;
namespace
{
// 정렬된 표본에서 가장 가까운 상위 순위 값을 고른다.
double Percentile(const std::vector<double>& sortedDurations, double fraction)
{
    if (sortedDurations.empty())
        return 0.0;

    const std::size_t index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(sortedDurations.size()))) - 1U;
    return sortedDurations[std::min(index, sortedDurations.size() - 1U)];
}

nlohmann::json SummarizeDurations(std::vector<double> durations)
{
    // 복사한 표본을 정렬해 중앙값 주변과 긴 지연 꼬리를 함께 요약한다.
    std::sort(durations.begin(), durations.end());
    return {
        {"p50", Percentile(durations, 0.50)},
        {"p95", Percentile(durations, 0.95)},
        {"p99", Percentile(durations, 0.99)},
        {"maximum", durations.empty() ? 0.0 : durations.back()},
        {"samples", durations.size()}};
}

std::filesystem::path DefaultBenchmarkOutputPath()
{
    if (const char* outputPath = std::getenv("AEROSIMLAB_BENCHMARK_OUTPUT");
        outputPath != nullptr && outputPath[0] != '\0')
    {
        return std::filesystem::u8path(outputPath);
    }

#ifdef _WIN32
    // 실행 옵션이 없으면 실행 파일 옆에 최신 결과 하나를 덮어쓴다.
    return detail::ExecutableDirectory() / L"benchmark-latest.json";
#else
    return std::filesystem::current_path() / "benchmark-latest.json";
#endif
}

}

nlohmann::json BuildViewerBenchmarkReport(ViewerBenchmarkMeasurements measurements)
{
    // 60Hz 한 주기보다 오래 걸린 물리 Tick을 놓친 주기로 센다.
    const std::size_t deadlineMisses = static_cast<std::size_t>(std::count_if(
        measurements.tickDurationsMicroseconds.begin(),
        measurements.tickDurationsMicroseconds.end(), [](double duration)
        {
            return duration > 1'000'000.0 / 60.0;
        }));

    nlohmann::json report{
        {"source_revision", measurements.sourceRevision},
        {"window_visible", measurements.windowVisible},
        {"framebuffer", {
            {"width", measurements.framebufferWidth},
            {"height", measurements.framebufferHeight}}},
        {"opengl", {
            {"renderer", measurements.graphicsRenderer},
            {"version", measurements.graphicsVersion}}},
        {"elapsed_seconds", measurements.elapsedSeconds},
        {"warmup_seconds", measurements.warmupSeconds},
        {"simulation_ticks", measurements.simulationTicks},
        {"rendered_frames", measurements.renderedFrames},
        {"render_fps", measurements.elapsedSeconds > 0.0
            ? static_cast<double>(measurements.renderedFrames) / measurements.elapsedSeconds : 0.0},
        {"tick_duration_us", SummarizeDurations(
            std::move(measurements.tickDurationsMicroseconds))},
        {"application_frame_duration_us", SummarizeDurations(
            std::move(measurements.applicationFrameDurationsMicroseconds))},
        {"render_frame_duration_us", SummarizeDurations(
            std::move(measurements.renderFrameDurationsMicroseconds))},
        {"slow_frames_over_16_67ms", nlohmann::json::array()},
        {"process_cpu_percent_approx", measurements.elapsedSeconds > 0.0
            ? 100.0 * measurements.processCpuSeconds / measurements.elapsedSeconds : 0.0}};
    report["tick_duration_us"]["deadline_misses_over_16_67ms"] = deadlineMisses;
    for (const ViewerBenchmarkMeasurements::SlowFrame& frame : measurements.slowFrames)
    {
        report["slow_frames_over_16_67ms"].push_back({
            {"frame_index", frame.frameIndex},
            {"application_us", frame.applicationMicroseconds},
            {"event_polling_us", frame.eventPollingMicroseconds},
            {"gamepad_polling_us", frame.gamepadPollingMicroseconds},
            {"simulation_update_us", frame.simulationUpdateMicroseconds},
            {"render_us", frame.renderMicroseconds},
            {"scene_panel_render_us", frame.scenePanelRenderMicroseconds},
            {"camera_scene_render_us", frame.cameraSceneRenderMicroseconds},
            {"buffer_swap_us", frame.bufferSwapMicroseconds},
            {"hud_interface_build_us", frame.hudInterfaceBuildMicroseconds},
            {"hud_frame_finalize_us", frame.hudFrameFinalizeMicroseconds},
            {"hud_backend_draw_us", frame.hudBackendDrawMicroseconds},
            {"hud_draw_list_count", frame.hudDrawListCount},
            {"hud_draw_command_count", frame.hudDrawCommandCount},
            {"hud_vertex_count", frame.hudVertexCount},
            {"hud_index_count", frame.hudIndexCount},
            {"hud_gpu_execution_us", frame.hudGpuExecutionMicroseconds},
            {"hud_gpu_timing_available", frame.hudGpuTimingAvailable}});
    }

#ifndef __EMSCRIPTEN__
    report["physics_step_duration_us"] = SummarizeDurations(
        std::move(measurements.physicsStepDurationsMicroseconds));
#endif

    if (const auto memory = ReadProcessMemorySample())
    {
        report["working_set_bytes"] = memory->workingSetBytes;
        report["private_bytes"] = memory->privateBytes;
    }
    return report;
}

void SaveViewerBenchmarkReport(
    const nlohmann::json& report, const std::filesystem::path& outputPath)
{
    // 지정 경로가 없으면 기본 위치를 쓰고, 결과 파일은 UTF-8 JSON 한 개로 갱신한다.
    const std::string output = report.dump(2) + '\n';
    const std::filesystem::path destination = outputPath.empty()
        ? DefaultBenchmarkOutputPath()
        : outputPath;
    std::ofstream resultFile(destination, std::ios::binary | std::ios::trunc);
    if (!resultFile)
        throw std::runtime_error("성능 측정 결과 파일을 저장할 수 없습니다.");
    resultFile.write(output.data(), static_cast<std::streamsize>(output.size()));
    if (!resultFile)
        throw std::runtime_error("성능 측정 결과를 파일에 모두 쓰지 못했습니다.");
#ifdef _WIN32
    OutputDebugStringA(output.c_str());
#else
    std::cout << output;
#endif
}

}
