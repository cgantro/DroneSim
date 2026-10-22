#include "SimulationBenchmark.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#include <Psapi.h>
#endif

#include "simulation_core/SimulationCore.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
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
#ifdef _WIN32
    std::wstring executablePath(32768U, L'\0');
    const DWORD pathLength = GetModuleFileNameW(
        nullptr, executablePath.data(), static_cast<DWORD>(executablePath.size()));
    if (pathLength == 0U || pathLength >= executablePath.size())
        throw std::runtime_error("실행 파일의 위치를 확인할 수 없습니다.");

    executablePath.resize(pathLength);
    return std::filesystem::path(executablePath).parent_path()
        / L"benchmark-latest.json";
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
        {"configuration", "hidden OpenGL window; one drone; 9.4km x 11.9km SRTM terrain; 17 OSM building boxes; 49 park facilities; textured PX4 X500 and park facility meshes"},
        {"source_revision", measurements.sourceRevision},
        {"framebuffer", {
            {"width", measurements.framebufferWidth},
            {"height", measurements.framebufferHeight}}},
        {"opengl", {
            {"renderer", measurements.graphicsRenderer},
            {"version", measurements.graphicsVersion}}},
        {"elapsed_seconds", measurements.elapsedSeconds},
        {"simulation_ticks", measurements.simulationTicks},
        {"rendered_frames", measurements.renderedFrames},
        {"render_fps", measurements.elapsedSeconds > 0.0
            ? static_cast<double>(measurements.renderedFrames) / measurements.elapsedSeconds : 0.0},
        {"tick_duration_us", SummarizeDurations(
            std::move(measurements.tickDurationsMicroseconds))},
        {"render_frame_duration_us", SummarizeDurations(
            std::move(measurements.renderFrameDurationsMicroseconds))},
        {"process_cpu_percent_approx", measurements.elapsedSeconds > 0.0
            ? 100.0 * measurements.processCpuSeconds / measurements.elapsedSeconds : 0.0}};
    report["tick_duration_us"]["deadline_misses_over_16_67ms"] = deadlineMisses;

#ifndef __EMSCRIPTEN__
    report["physics_step_duration_us"] = SummarizeDurations(
        std::move(measurements.physicsStepDurationsMicroseconds));
    report["vision_analysis"] = {
        {"samples", measurements.visionSampleCount},
        {"mean_brightness", measurements.meanBrightness},
        {"mean_contrast", measurements.meanContrast},
        {"mean_sharpness", measurements.meanSharpness},
        {"duration_us", SummarizeDurations(
            std::move(measurements.visionDurationsMicroseconds))}};
#endif

#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
        report["working_set_bytes"] = memory.WorkingSetSize;
#endif
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

#ifndef __EMSCRIPTEN__
nlohmann::json MeasureDroneScaling()
{
    constexpr std::array<std::size_t, 4> droneCounts{1U, 2U, 4U, 8U};
    constexpr std::size_t repetitions = 3U;
    constexpr std::size_t warmupTicks = 120U;
    constexpr std::size_t measuredTicks = 30000U;
    const auto readWorkingSetBytes = []() -> std::optional<std::size_t>
    {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS memory{};
        memory.cb = sizeof(memory);
        if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
            return memory.WorkingSetSize;
#endif
        return std::nullopt;
    };
    nlohmann::json runs = nlohmann::json::array();

    for (const std::size_t droneCount : droneCounts)
    {
        for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
        {
            aerosimlab::simulation::SimulationCore simulation{
                aerosimlab::simulation::SimulationDynamics::RigidBody};
            simulation.SetPhysicsStepTimingEnabled(true);

            std::vector<aerosimlab::simulation::DroneId> droneIds;
            droneIds.reserve(droneCount);
            droneIds.push_back(simulation.PrimaryDroneId());
            const double groundOriginMeters = simulation.State().positionMeters.z;
            for (std::size_t index = 1; index < droneCount; ++index)
            {
                const auto xMeters = static_cast<double>(index) * 2.5;
                droneIds.push_back(simulation.AddDrone(
                    glm::dvec3{xMeters, 0.0, groundOriginMeters}));
            }

            const aerosimlab::simulation::DroneCommand takeoff{
                aerosimlab::simulation::DroneCommand::Kind::TakeOff};
            for (const aerosimlab::simulation::DroneId droneId : droneIds)
            {
                if (simulation.Apply(droneId, takeoff)
                    != aerosimlab::simulation::CommandResult::Accepted)
                    throw std::runtime_error("멀티드론 성능 측정용 이륙 명령을 적용하지 못했습니다.");
            }

            for (std::size_t tick = 0; tick < warmupTicks; ++tick)
                simulation.AdvanceOneTick();

            std::vector<double> tickDurations;
            std::vector<double> physicsDurations;
            tickDurations.reserve(measuredTicks);
            physicsDurations.reserve(measuredTicks);
            const auto workingSetBefore = readWorkingSetBytes();
            const std::clock_t cpuStart = std::clock();
            const auto measurementStart = std::chrono::steady_clock::now();
            for (std::size_t tick = 0; tick < measuredTicks; ++tick)
            {
                const auto tickStart = std::chrono::steady_clock::now();
                simulation.AdvanceOneTick();
                tickDurations.push_back(std::chrono::duration<double, std::micro>(
                    std::chrono::steady_clock::now() - tickStart).count());
                physicsDurations.push_back(
                    simulation.LastPhysicsStepDurationMicroseconds());
            }

            const double wallSeconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - measurementStart).count();
            const double cpuSeconds = static_cast<double>(std::clock() - cpuStart)
                / CLOCKS_PER_SEC;
            const auto workingSetAfter = readWorkingSetBytes();
            nlohmann::json run{
                {"drone_count", droneCount},
                {"repetition", repetition + 1U},
                {"warmup_ticks", warmupTicks},
                {"measured_ticks", measuredTicks},
                {"tick_duration_us", SummarizeDurations(std::move(tickDurations))},
                {"physics_step_duration_us", SummarizeDurations(std::move(physicsDurations))},
                {"measurement_wall_seconds", wallSeconds},
                {"process_cpu_percent_approx", wallSeconds > 0.0
                    ? 100.0 * cpuSeconds / wallSeconds : 0.0}};
            if (workingSetBefore && workingSetAfter)
            {
                run["process_working_set_before_bytes"] = *workingSetBefore;
                run["process_working_set_after_bytes"] = *workingSetAfter;
                run["process_working_set_change_bytes"] =
                    static_cast<std::int64_t>(*workingSetAfter)
                    - static_cast<std::int64_t>(*workingSetBefore);
            }
            runs.push_back(std::move(run));
        }
    }

    return {
        {"method", "헤드리스 AirSim 강체 모의; 한 물리 세계에서 모든 드론을 함께 갱신. 각 실행은 워밍업 뒤 30,000 Tick을 측정하고 프로세스 CPU 시간과 작업 집합을 기록"},
        {"counts", droneCounts},
        {"repetitions_per_count", repetitions},
        {"runs", std::move(runs)}};
}
#endif
}
