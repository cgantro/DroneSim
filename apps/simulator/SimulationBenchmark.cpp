#include "SimulationBenchmark.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#include <Psapi.h>
#endif

#include "simulation_core/SimulationCore.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/ScopedDuration.h"
#include "simulation_core/ScenarioRunner.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
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
    if (const char* outputPath = std::getenv("AEROSIMLAB_BENCHMARK_OUTPUT");
        outputPath != nullptr && outputPath[0] != '\0')
    {
        return std::filesystem::u8path(outputPath);
    }

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

std::optional<std::size_t> ReadProcessWorkingSetBytes()
{
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
        return memory.WorkingSetSize;
#endif
    return std::nullopt;
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
        {"configuration", "hidden OpenGL window; one drone; 18.8km x 23.7km SRTM terrain; 17 OSM building boxes; 49 park facilities; textured PX4 X500 and park facility meshes; EO pixel readback uses a 3-slot asynchronous PBO ring"},
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
        {"application_frame_duration_us", SummarizeDurations(
            std::move(measurements.applicationFrameDurationsMicroseconds))},
        {"render_frame_duration_us", SummarizeDurations(
            std::move(measurements.renderFrameDurationsMicroseconds))},
        {"render_frame_with_eo_readback_duration_us", SummarizeDurations(
            std::move(measurements.renderFrameDurationsWithEoReadbackMicroseconds))},
        {"render_frame_without_eo_readback_duration_us", SummarizeDurations(
            std::move(measurements.renderFrameDurationsWithoutEoReadbackMicroseconds))},
        {"process_cpu_percent_approx", measurements.elapsedSeconds > 0.0
            ? 100.0 * measurements.processCpuSeconds / measurements.elapsedSeconds : 0.0}};
    report["tick_duration_us"]["deadline_misses_over_16_67ms"] = deadlineMisses;

#ifndef __EMSCRIPTEN__
    report["physics_step_duration_us"] = SummarizeDurations(
        std::move(measurements.physicsStepDurationsMicroseconds));
    report["eo_pixel_readback_enqueue_duration_us"] = SummarizeDurations(
        std::move(measurements.eoReadbackEnqueueDurationsMicroseconds));
    report["eo_pixel_readback_poll_duration_us"] = SummarizeDurations(
        std::move(measurements.eoReadbackPollDurationsMicroseconds));
    report["vision_analysis"] = {
        {"samples", measurements.visionSampleCount},
        {"mean_brightness", measurements.meanBrightness},
        {"mean_contrast", measurements.meanContrast},
        {"mean_sharpness", measurements.meanSharpness},
        {"duration_us", SummarizeDurations(
            std::move(measurements.visionDurationsMicroseconds))}};
#endif

    if (const auto workingSetBytes = ReadProcessWorkingSetBytes())
        report["working_set_bytes"] = *workingSetBytes;
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
nlohmann::json MeasureDroneScaling(
    aerosimlab::simulation::SimulationDynamics dynamics,
    DroneScalingLayout layout)
{
    using aerosimlab::simulation::SimulationDynamics;
    constexpr std::size_t repetitions = 3U;
    constexpr std::size_t warmupTicks = 120U;
    constexpr std::size_t measuredTicks = 30000U;
    constexpr std::size_t workingSetSampleIntervalTicks = 5000U;
    constexpr auto runTimeLimit = std::chrono::seconds{30};
    const bool usesRigidBodyPhysics = dynamics == SimulationDynamics::RigidBody;
    const bool usesCompactLayout = layout == DroneScalingLayout::Compact;
    const std::vector<std::size_t> droneCounts = usesCompactLayout
        ? std::vector<std::size_t>{1U, 8U, 32U, 64U, 128U, 256U}
        : std::vector<std::size_t>{1U, 8U, 32U, 64U, 128U, 256U, 512U, 1024U};
    const char* dynamicsName = usesRigidBodyPhysics ? "rigid_body" : "kinematic";
    const char* layoutName = usesCompactLayout ? "compact" : "separated";
    constexpr double separatedSpacingMeters = 2.5;
    constexpr double compactSpacingMeters = 0.55;
    nlohmann::json runs = nlohmann::json::array();
    bool stoppedAtTimeLimit = false;

    for (const std::size_t droneCount : droneCounts)
    {
        for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
        {
            aerosimlab::simulation::SimulationCore simulation{
                dynamics};
            simulation.SetEcsSystemTimingEnabled(true);
            simulation.SetPhysicsStepTimingEnabled(usesRigidBodyPhysics);

            std::vector<aerosimlab::simulation::DroneId> droneIds;
            droneIds.reserve(droneCount);
            droneIds.push_back(simulation.PrimaryDroneId());
            std::vector<double> droneCreationDurations;
            droneCreationDurations.reserve(droneCount - 1U);
            const double groundOriginMeters = simulation.State().positionMeters.z;
            const std::size_t compactGridWidth = static_cast<std::size_t>(
                std::ceil(std::sqrt(static_cast<double>(droneCount))));
            for (std::size_t index = 1; index < droneCount; ++index)
            {
                glm::dvec3 positionMeters{};
                positionMeters.z = groundOriginMeters;
                if (usesCompactLayout)
                {
                    positionMeters.x = static_cast<double>(index % compactGridWidth)
                        * compactSpacingMeters;
                    positionMeters.y = static_cast<double>(index / compactGridWidth)
                        * compactSpacingMeters;
                }
                else
                {
                    positionMeters.x = static_cast<double>(index)
                        * separatedSpacingMeters;
                }
                double creationDurationMicroseconds = 0.0;
                aerosimlab::simulation::DroneId droneId = 0;
                {
                    simulation::ScopedDuration measureCreation(
                        &creationDurationMicroseconds);
                    droneId = simulation.AddDrone(positionMeters);
                }
                droneCreationDurations.push_back(creationDurationMicroseconds);
                droneIds.push_back(droneId);
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
            std::vector<double> flecsProgressDurations;
            std::vector<double> kinematicDroneSystemDurations;
            std::vector<double> rigidBodyForceSystemDurations;
            std::vector<double> cameraSystemDurations;
            std::vector<double> physicsDurations;
            std::vector<double> contactGenerationDurations;
            std::vector<double> contactGenerationFractions;
            std::vector<std::size_t> workingSetSamples;
            tickDurations.reserve(measuredTicks);
            flecsProgressDurations.reserve(measuredTicks);
            kinematicDroneSystemDurations.reserve(measuredTicks);
            rigidBodyForceSystemDurations.reserve(measuredTicks);
            cameraSystemDurations.reserve(measuredTicks);
            physicsDurations.reserve(measuredTicks);
            contactGenerationDurations.reserve(measuredTicks);
            contactGenerationFractions.reserve(measuredTicks);
            workingSetSamples.reserve(
                measuredTicks / workingSetSampleIntervalTicks + 1U);
            const auto workingSetBefore = ReadProcessWorkingSetBytes();
            const std::clock_t cpuStart = std::clock();
            const auto measurementStart = std::chrono::steady_clock::now();
            bool timeLimitReached = false;
            for (std::size_t tick = 0; tick < measuredTicks; ++tick)
            {
                double tickDurationMicroseconds = 0.0;
                {
                    simulation::ScopedDuration measureTick(&tickDurationMicroseconds);
                    simulation.AdvanceOneTick();
                }
                tickDurations.push_back(tickDurationMicroseconds);
                flecsProgressDurations.push_back(
                    simulation.LastFlecsProgressDurationMicroseconds());
                const simulation::EcsSystemDurations systemDurations =
                    simulation.LastEcsSystemDurations();
                kinematicDroneSystemDurations.push_back(
                    systemDurations.advanceKinematicDroneMicroseconds);
                rigidBodyForceSystemDurations.push_back(
                    systemDurations.applyRigidBodyForcesMicroseconds);
                cameraSystemDurations.push_back(
                    systemDurations.advanceCameraMicroseconds);
                if (usesRigidBodyPhysics)
                {
                    const double physicsDuration =
                        simulation.LastPhysicsStepDurationMicroseconds();
                    const double contactGenerationDuration =
                        simulation.LastContactGenerationDurationMicroseconds();
                    physicsDurations.push_back(physicsDuration);
                    contactGenerationDurations.push_back(contactGenerationDuration);
                    if (physicsDuration > 0.0)
                    {
                        contactGenerationFractions.push_back(
                            contactGenerationDuration / physicsDuration);
                    }
                }

                if ((tick + 1U) % workingSetSampleIntervalTicks == 0U)
                {
                    if (const auto workingSet = ReadProcessWorkingSetBytes())
                        workingSetSamples.push_back(*workingSet);
                }

                if ((tick + 1U) % 1000U == 0U
                    && std::chrono::steady_clock::now() - measurementStart >= runTimeLimit)
                {
                    timeLimitReached = true;
                    break;
                }
            }

            const std::size_t completedTicks = tickDurations.size();
            const double wallSeconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - measurementStart).count();
            const double cpuSeconds = static_cast<double>(std::clock() - cpuStart)
                / CLOCKS_PER_SEC;
            const auto workingSetAfter = ReadProcessWorkingSetBytes();
            if (workingSetAfter)
                workingSetSamples.push_back(*workingSetAfter);
            const std::size_t deadlineMisses = static_cast<std::size_t>(std::count_if(
                tickDurations.begin(), tickDurations.end(), [](double duration)
                {
                    return duration > 1'000'000.0 / 60.0;
                }));
            std::vector<double> droneRemovalDurations;
            droneRemovalDurations.reserve(droneIds.size() - 1U);
            for (std::size_t index = droneIds.size(); index > 1U; --index)
            {
                double removalDurationMicroseconds = 0.0;
                bool removed = false;
                {
                    simulation::ScopedDuration measureRemoval(
                        &removalDurationMicroseconds);
                    removed = simulation.RemoveDrone(droneIds[index - 1U]);
                }
                if (!removed)
                    throw std::runtime_error("멀티드론 성능 측정용 드론을 삭제하지 못했습니다.");
                droneRemovalDurations.push_back(removalDurationMicroseconds);
            }

            nlohmann::json run{
                {"dynamics", dynamicsName},
                {"layout", layoutName},
                {"drone_count", droneCount},
                {"repetition", repetition + 1U},
                {"warmup_ticks", warmupTicks},
                {"target_ticks", measuredTicks},
                {"measured_ticks", completedTicks},
                {"tick_duration_us", SummarizeDurations(std::move(tickDurations))},
                {"flecs_progress_duration_us", SummarizeDurations(
                    std::move(flecsProgressDurations))},
                {"advance_camera_system_duration_us", SummarizeDurations(
                    std::move(cameraSystemDurations))},
                {"measurement_wall_seconds", wallSeconds},
                {"simulation_ticks_per_second", wallSeconds > 0.0
                    ? static_cast<double>(completedTicks) / wallSeconds : 0.0},
                {"drone_ticks_per_second", wallSeconds > 0.0
                    ? static_cast<double>(droneCount) * static_cast<double>(completedTicks)
                        / wallSeconds : 0.0},
                {"real_time_factor", wallSeconds > 0.0
                    ? static_cast<double>(completedTicks)
                        * aerosimlab::simulation::DroneModel::fixedStepSeconds / wallSeconds
                    : 0.0},
                {"deadline_misses_over_16_67ms", deadlineMisses},
                {"time_limit_reached", timeLimitReached},
                {"process_cpu_percent_approx", wallSeconds > 0.0
                    ? 100.0 * cpuSeconds / wallSeconds : 0.0}};
            run["entity_lifecycle"] = {
                {"additional_drone_add_duration_us", SummarizeDurations(
                    std::move(droneCreationDurations))},
                {"additional_drone_remove_duration_us", SummarizeDurations(
                    std::move(droneRemovalDurations))},
                {"measured_add_count", droneCount - 1U},
                {"measured_remove_count", droneCount - 1U}};
            if (usesRigidBodyPhysics)
            {
                run["apply_rigid_body_forces_system_duration_us"] = SummarizeDurations(
                    std::move(rigidBodyForceSystemDurations));
                run["physics_step_duration_us"] = SummarizeDurations(
                    std::move(physicsDurations));
                run["contact_generation_duration_us"] = SummarizeDurations(
                    std::move(contactGenerationDurations));
                run["contact_generation_fraction"] = SummarizeDurations(
                    std::move(contactGenerationFractions));
            }
            else
            {
                run["advance_kinematic_drone_system_duration_us"] = SummarizeDurations(
                    std::move(kinematicDroneSystemDurations));
            }
            if (workingSetBefore && workingSetAfter)
            {
                run["process_working_set_before_bytes"] = *workingSetBefore;
                run["process_working_set_after_bytes"] = *workingSetAfter;
                run["process_working_set_change_bytes"] =
                    static_cast<std::int64_t>(*workingSetAfter)
                    - static_cast<std::int64_t>(*workingSetBefore);
            }
            if (!workingSetSamples.empty())
            {
                const double totalWorkingSetBytes = std::accumulate(
                    workingSetSamples.begin(), workingSetSamples.end(), 0.0);
                run["sampled_average_working_set_bytes"] =
                    totalWorkingSetBytes / static_cast<double>(workingSetSamples.size());
                run["sampled_max_working_set_bytes"] = *std::max_element(
                    workingSetSamples.begin(), workingSetSamples.end());
                run["working_set_sample_interval_ticks"] =
                    workingSetSampleIntervalTicks;
                run["working_set_sample_count"] = workingSetSamples.size();
            }
            runs.push_back(std::move(run));
            if (timeLimitReached)
            {
                stoppedAtTimeLimit = true;
                break;
            }
        }
        if (stoppedAtTimeLimit)
            break;
    }

    nlohmann::json measuredCounts = nlohmann::json::array();
    std::optional<std::size_t> previousCount;
    for (const nlohmann::json& run : runs)
    {
        const std::size_t measuredCount = run.at("drone_count").get<std::size_t>();
        if (!previousCount || *previousCount != measuredCount)
        {
            measuredCounts.push_back(measuredCount);
            previousCount = measuredCount;
        }
    }

    return {
        {"dynamics", dynamicsName},
        {"layout", layoutName},
        {"spacing_meters", usesCompactLayout
            ? compactSpacingMeters : separatedSpacingMeters},
        {"method", usesRigidBodyPhysics
            ? "헤드리스 AirSim 강체 모의. 여러 드론을 이륙·호버링시키고 한 물리 세계에서 함께 갱신"
            : "헤드리스 운동학 모의. 여러 드론을 이륙·호버링시키고 매 Tick 상태를 갱신"},
        {"target_counts", droneCounts},
        {"measured_counts", std::move(measuredCounts)},
        {"target_repetitions_per_count", repetitions},
        {"target_ticks_per_run", measuredTicks},
        {"run_time_limit_seconds", runTimeLimit.count()},
        {"stopped_at_time_limit", stoppedAtTimeLimit},
        {"stop_reason", stoppedAtTimeLimit
            ? "한 번의 측정이 제한 시간에 도달해 더 큰 규모의 실행을 멈췄습니다."
            : "모든 지정 규모를 측정했습니다."},
        {"runs", std::move(runs)}};
}

nlohmann::json MeasureCooperativeMissionScaling()
{
    using aerosimlab::simulation::RunScenario;
    using aerosimlab::simulation::SimulationDynamics;

    const auto buildScenario = [](SimulationDynamics dynamics, std::size_t droneCount)
    {
        const bool usesRigidBodyPhysics = dynamics == SimulationDynamics::RigidBody;
        const std::uint64_t missionStartTick = usesRigidBodyPhysics ? 270U : 90U;
        const std::uint64_t missionDeadlineTick = missionStartTick + 180U;
        const std::uint64_t scenarioTicks = missionDeadlineTick + 30U;
        const double initialAltitudeMeters = usesRigidBodyPhysics
            ? aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters
            : 0.0;
        const double targetAltitudeMeters = initialAltitudeMeters
            + aerosimlab::simulation::DroneModel::takeoffHeightMeters;
        const double taskToleranceMeters = usesRigidBodyPhysics ? 0.15 : 0.05;
        const double expectedToleranceMeters = usesRigidBodyPhysics ? 0.25 : 0.05;
        constexpr double spacingMeters = 5.0;
        constexpr double waypointOffsetMeters = 0.5;

        nlohmann::json drones = nlohmann::json::array();
        nlohmann::json commands = nlohmann::json::array();
        nlohmann::json tasks = nlohmann::json::array();
        nlohmann::json expectedDrones = nlohmann::json::array();
        for (std::size_t index = 0; index < droneCount; ++index)
        {
            const std::string droneName = "drone-" + std::to_string(index);
            const double startX = static_cast<double>(index) * spacingMeters;
            const double targetX = startX + waypointOffsetMeters;
            drones.push_back({
                {"name", droneName},
                {"initial_position_meters", {startX, 0.0, initialAltitudeMeters}}});
            commands.push_back({
                {"tick", 0},
                {"drone", droneName},
                {"type", "takeoff"}});
            tasks.push_back({
                {"name", "task-" + std::to_string(index)},
                {"role", "survey"},
                {"drone", droneName},
                {"target_position_meters", {targetX, 0.0, targetAltitudeMeters}},
                {"start_tick", missionStartTick},
                {"deadline_tick", missionDeadlineTick},
                {"tolerance_meters", taskToleranceMeters}});
            expectedDrones.push_back({
                {"name", droneName},
                {"mode", "hovering"},
                {"position_meters", {targetX, 0.0, targetAltitudeMeters}}});
        }

        return nlohmann::json{
            {"version", 1},
            {"dynamics", usesRigidBodyPhysics ? "rigid_body" : "kinematic"},
            {"ticks", scenarioTicks},
            {"drones", std::move(drones)},
            {"commands", std::move(commands)},
            {"mission", {
                {"name", "parallel-survey"},
                {"tasks", std::move(tasks)}}},
            {"expected", {
                {"tolerance_meters", expectedToleranceMeters},
                {"drones", std::move(expectedDrones)}}}};
    };

    const auto measureDynamics = [&buildScenario](SimulationDynamics dynamics)
    {
        const bool usesRigidBodyPhysics = dynamics == SimulationDynamics::RigidBody;
        const char* dynamicsName = usesRigidBodyPhysics ? "rigid_body" : "kinematic";
        const std::vector<std::size_t> droneCounts = usesRigidBodyPhysics
            ? std::vector<std::size_t>{2U, 8U, 32U, 64U, 128U}
            : std::vector<std::size_t>{2U, 8U, 32U, 64U, 128U, 256U, 512U, 1024U};
        constexpr std::size_t repetitions = 3U;
        constexpr std::size_t warmupRuns = 1U;
        nlohmann::json runs = nlohmann::json::array();
        nlohmann::json summaries = nlohmann::json::array();
        std::vector<std::size_t> measuredCounts;

        for (const std::size_t droneCount : droneCounts)
        {
            const std::string scenarioText = buildScenario(dynamics, droneCount).dump();
            for (std::size_t warmup = 0; warmup < warmupRuns; ++warmup)
            {
                const aerosimlab::simulation::ScenarioResult result = RunScenario(scenarioText);
                if (!result.passed || !result.mission || !result.mission->passed)
                    throw std::runtime_error("공동 임무 성능 측정의 준비 실행이 통과하지 못했습니다.");
            }

            std::vector<double> scenarioDurationsMicroseconds;
            scenarioDurationsMicroseconds.reserve(repetitions);
            for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
            {
                const std::clock_t cpuStart = std::clock();
                double scenarioDurationMicroseconds = 0.0;
                const aerosimlab::simulation::ScenarioResult result = [&]
                {
                    simulation::ScopedDuration measureScenario(
                        &scenarioDurationMicroseconds);
                    return RunScenario(scenarioText);
                }();
                const double wallSeconds = scenarioDurationMicroseconds / 1'000'000.0;
                const double cpuSeconds = static_cast<double>(std::clock() - cpuStart)
                    / CLOCKS_PER_SEC;
                if (!result.passed || !result.mission || !result.mission->passed)
                {
                    const std::string reason = result.issues.empty()
                        ? "임무 완료 판정이 실패했습니다."
                        : result.issues.front();
                    throw std::runtime_error(
                        "공동 임무 성능 측정 결과가 통과하지 못했습니다: " + reason);
                }

                std::size_t completedTasks = 0;
                for (const auto& task : result.mission->tasks)
                {
                    if (task.completed)
                        ++completedTasks;
                }
                const double durationMicroseconds = wallSeconds * 1'000'000.0;
                scenarioDurationsMicroseconds.push_back(durationMicroseconds);
                runs.push_back({
                    {"dynamics", dynamicsName},
                    {"drone_count", droneCount},
                    {"repetition", repetition + 1U},
                    {"warmup_runs", warmupRuns},
                    {"scenario_ticks", result.tickCount},
                    {"scenario_duration_us", durationMicroseconds},
                    {"scenario_ticks_per_second", wallSeconds > 0.0
                        ? static_cast<double>(result.tickCount) / wallSeconds : 0.0},
                    {"drone_ticks_per_second", wallSeconds > 0.0
                        ? static_cast<double>(droneCount)
                            * static_cast<double>(result.tickCount) / wallSeconds : 0.0},
                    {"process_cpu_percent_approx", wallSeconds > 0.0
                        ? 100.0 * cpuSeconds / wallSeconds : 0.0},
                    {"scenario_passed", result.passed},
                    {"mission_passed", result.mission->passed},
                    {"mission_task_count", result.mission->tasks.size()},
                    {"completed_task_count", completedTasks}});
            }
            measuredCounts.push_back(droneCount);
            summaries.push_back({
                {"drone_count", droneCount},
                {"scenario_duration_us",
                    SummarizeDurations(std::move(scenarioDurationsMicroseconds))}});
        }

        const std::uint64_t missionStartTick = usesRigidBodyPhysics ? 270U : 90U;
        const std::uint64_t scenarioTicks = missionStartTick + 210U;
        return nlohmann::json{
            {"dynamics", dynamicsName},
            {"layout", "separated"},
            {"spacing_meters", 5.0},
            {"mission_start_tick", missionStartTick},
            {"target_scenario_ticks", scenarioTicks},
            {"warmup_runs_per_count", warmupRuns},
            {"target_repetitions_per_count", repetitions},
            {"target_counts", droneCounts},
            {"measured_counts", std::move(measuredCounts)},
            {"method", "실제 JSON 시나리오 실행기로 모든 드론에 조사 목표를 배정하고, 비행·목표 도달·임무 완료 판정을 함께 측정"},
            {"summaries", std::move(summaries)},
            {"runs", std::move(runs)}};
    };

    return {
        {"kinematic", measureDynamics(SimulationDynamics::Kinematic)},
        {"rigid_body", measureDynamics(SimulationDynamics::RigidBody)}};
}
#endif
}
