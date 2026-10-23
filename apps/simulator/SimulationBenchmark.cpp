#include "SimulationBenchmark.h"
#include "ExecutableDirectory.h"
#include "BenchmarkProcessMemory.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
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
double Percentile(const std::vector<double>& sortedDurations, double fraction)
{
    if (sortedDurations.empty())
        return 0.0;

    const std::size_t index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(sortedDurations.size()))) - 1U;
    return sortedDurations[std::min(index, sortedDurations.size() - 1U)];
}

std::filesystem::path DefaultBenchmarkOutputPath()
{
    if (const char* outputPath = std::getenv("AEROSIMLAB_BENCHMARK_OUTPUT");
        outputPath != nullptr && outputPath[0] != '\0')
    {
        return std::filesystem::u8path(outputPath);
    }

#ifdef _WIN32
    return detail::ExecutableDirectory() / L"benchmark-latest.json";
#else
    return std::filesystem::current_path() / "benchmark-latest.json";
#endif
}

}

nlohmann::json SummarizeDurations(std::vector<double> durations)
{
    std::sort(durations.begin(), durations.end());
    return {
        {"p50", Percentile(durations, 0.50)},
        {"p95", Percentile(durations, 0.95)},
        {"p99", Percentile(durations, 0.99)},
        {"maximum", durations.empty() ? 0.0 : durations.back()},
        {"samples", durations.size()}};
}

nlohmann::json BuildViewerBenchmarkReport(ViewerBenchmarkMeasurements measurements)
{
    const std::size_t deadlineMisses = static_cast<std::size_t>(std::count_if(
        measurements.tickDurationsMicroseconds.begin(),
        measurements.tickDurationsMicroseconds.end(), [](double duration)
        {
            return duration > 1'000'000.0 / 60.0;
        }));

    nlohmann::json report{
        {"configuration", std::string(measurements.windowVisible
            ? "visible OpenGL window with VSync off; "
            : "hidden OpenGL window with VSync off; ")
            + "one drone; 18.8km x 23.7km SRTM terrain; 17 OSM building boxes; 49 park facilities; textured PX4 X500 and park facility meshes; EO pixel readback uses a 3-slot asynchronous PBO ring"},
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
        {"render_frame_with_eo_readback_duration_us", SummarizeDurations(
            std::move(measurements.renderFrameDurationsWithEoReadbackMicroseconds))},
        {"render_frame_without_eo_readback_duration_us", SummarizeDurations(
            std::move(measurements.renderFrameDurationsWithoutEoReadbackMicroseconds))},
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
            {"ground_control_polling_us", frame.groundControlPollingMicroseconds},
            {"gamepad_polling_us", frame.gamepadPollingMicroseconds},
            {"simulation_update_us", frame.simulationUpdateMicroseconds},
            {"render_us", frame.renderMicroseconds},
            {"scene_panel_render_us", frame.scenePanelRenderMicroseconds},
            {"camera_scene_render_us", frame.cameraSceneRenderMicroseconds},
            {"buffer_swap_us", frame.bufferSwapMicroseconds},
            {"eo_readback_handling_us", frame.eoReadbackHandlingMicroseconds},
            {"eo_fence_wait_us", frame.eoFenceWaitMicroseconds},
            {"eo_buffer_map_us", frame.eoBufferMapMicroseconds},
            {"eo_pixel_copy_us", frame.eoPixelCopyMicroseconds},
            {"hud_interface_build_us", frame.hudInterfaceBuildMicroseconds},
            {"hud_frame_finalize_us", frame.hudFrameFinalizeMicroseconds},
            {"hud_backend_draw_us", frame.hudBackendDrawMicroseconds},
            {"hud_draw_list_count", frame.hudDrawListCount},
            {"hud_draw_command_count", frame.hudDrawCommandCount},
            {"hud_vertex_count", frame.hudVertexCount},
            {"hud_index_count", frame.hudIndexCount},
            {"hud_gpu_execution_us", frame.hudGpuExecutionMicroseconds},
            {"hud_gpu_timing_available", frame.hudGpuTimingAvailable},
            {"eo_readback_happened", frame.eoReadbackHappened}});
    }

#ifndef __EMSCRIPTEN__
    report["physics_step_duration_us"] = SummarizeDurations(
        std::move(measurements.physicsStepDurationsMicroseconds));
    report["eo_pixel_readback_enqueue_duration_us"] = SummarizeDurations(
        std::move(measurements.eoReadbackEnqueueDurationsMicroseconds));
    report["eo_pixel_readback_slot_setup_us"] = SummarizeDurations(
        std::move(measurements.eoReadbackSlotSetupDurationsMicroseconds));
    report["eo_pixel_buffer_setup_us"] = SummarizeDurations(
        std::move(measurements.eoPixelBufferSetupDurationsMicroseconds));
    report["eo_read_target_setup_us"] = SummarizeDurations(
        std::move(measurements.eoReadTargetSetupDurationsMicroseconds));
    report["eo_gl_readpixels_call_us"] = SummarizeDurations(
        std::move(measurements.eoReadPixelsCallDurationsMicroseconds));
    report["eo_gl_fence_creation_call_us"] = SummarizeDurations(
        std::move(measurements.eoFenceCreationCallDurationsMicroseconds));
    report["eo_gl_state_restore_us"] = SummarizeDurations(
        std::move(measurements.eoGlStateRestoreDurationsMicroseconds));
    report["eo_pixel_readback_poll_duration_us"] = SummarizeDurations(
        std::move(measurements.eoReadbackPollDurationsMicroseconds));
    report["eo_gl_fence_wait_call_us"] = SummarizeDurations(
        std::move(measurements.eoFenceWaitCallDurationsMicroseconds));
    report["eo_gl_fallback_finish_us"] = SummarizeDurations(
        std::move(measurements.eoReadbackFallbackFinishDurationsMicroseconds));
    report["eo_pixel_buffer_resize_us"] = SummarizeDurations(
        std::move(measurements.eoPixelBufferResizeDurationsMicroseconds));
    report["eo_gl_pixel_pack_buffer_bind_us"] = SummarizeDurations(
        std::move(measurements.eoPixelPackBufferBindDurationsMicroseconds));
    report["eo_gl_buffer_map_call_us"] = SummarizeDurations(
        std::move(measurements.eoPixelBufferMapCallDurationsMicroseconds));
    report["eo_pixel_copy_us"] = SummarizeDurations(
        std::move(measurements.eoPixelCopyDurationsMicroseconds));
    report["eo_gl_buffer_unmap_call_us"] = SummarizeDurations(
        std::move(measurements.eoPixelBufferUnmapCallDurationsMicroseconds));
    report["eo_readback_cleanup_us"] = SummarizeDurations(
        std::move(measurements.eoReadbackCleanupDurationsMicroseconds));
    report["vision_analysis"] = {
        {"samples", measurements.visionSampleCount},
        {"mean_brightness", measurements.meanBrightness},
        {"mean_contrast", measurements.meanContrast},
        {"mean_sharpness", measurements.meanSharpness},
        {"duration_us", SummarizeDurations(
            std::move(measurements.visionDurationsMicroseconds))}};
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
