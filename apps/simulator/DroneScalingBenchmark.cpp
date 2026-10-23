#include "SimulationBenchmark.h"
#include "BenchmarkProcessMemory.h"
#include "simulation_core/SimulationCore.h"
#include "simulation_core/ScopedDuration.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
#ifndef __EMSCRIPTEN__
namespace
{
using aerosimlab::simulation::DroneId;
using detail::ProcessMemorySample;
using detail::ReadProcessMemorySample;
using aerosimlab::simulation::SimulationCore;

struct DroneScalingMeasurements
{
    std::vector<double> tickDurations;
    std::vector<double> flecsProgressDurations;
    std::vector<double> kinematicDroneSystemDurations;
    std::vector<double> rigidBodyForceSystemDurations;
    std::vector<double> cameraSystemDurations;
    std::vector<double> physicsDurations;
    std::vector<double> contactGenerationDurations;
    std::vector<double> contactGenerationFractions;
    std::vector<ProcessMemorySample> memorySamples;
    std::optional<ProcessMemorySample> memoryBefore;
    std::optional<ProcessMemorySample> memoryAfter;
    double wallSeconds = 0.0;
    double cpuSeconds = 0.0;
    bool timeLimitReached = false;
};

std::vector<DroneId> AddScalingDrones(
    SimulationCore& simulation,
    std::size_t droneCount,
    bool usesCompactLayout,
    double separatedSpacingMeters,
    double compactSpacingMeters,
    std::size_t compactGridWidth,
    std::vector<double>& creationDurations)
{
    std::vector<DroneId> droneIds;
    droneIds.reserve(droneCount);
    droneIds.push_back(simulation.PrimaryDroneId());
    creationDurations.reserve(droneCount - 1U);
    const double groundOriginMeters = simulation.State().positionMeters.z;

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
            positionMeters.x = static_cast<double>(index) * separatedSpacingMeters;
        }

        double durationMicroseconds = 0.0;
        DroneId droneId = 0;
        {
            simulation::ScopedDuration measureCreation(&durationMicroseconds);
            droneId = simulation.AddDrone(positionMeters);
        }
        creationDurations.push_back(durationMicroseconds);
        droneIds.push_back(droneId);
    }

    return droneIds;
}

void StartScalingDrones(SimulationCore& simulation, const std::vector<DroneId>& droneIds)
{
    const aerosimlab::simulation::DroneCommand takeoff{
        aerosimlab::simulation::DroneCommand::Kind::TakeOff};
    for (const DroneId droneId : droneIds)
    {
        if (simulation.Apply(droneId, takeoff)
            != aerosimlab::simulation::CommandResult::Accepted)
            throw std::runtime_error("멀티드론 성능 측정용 이륙 명령을 적용하지 못했습니다.");
    }
}

DroneScalingMeasurements MeasureScalingTicks(
    SimulationCore& simulation,
    bool usesRigidBodyPhysics,
    std::size_t measuredTicks,
    std::size_t workingSetSampleIntervalTicks,
    std::chrono::seconds runTimeLimit)
{
    DroneScalingMeasurements measurements;
    measurements.tickDurations.reserve(measuredTicks);
    measurements.flecsProgressDurations.reserve(measuredTicks);
    measurements.kinematicDroneSystemDurations.reserve(measuredTicks);
    measurements.rigidBodyForceSystemDurations.reserve(measuredTicks);
    measurements.cameraSystemDurations.reserve(measuredTicks);
    measurements.physicsDurations.reserve(measuredTicks);
    measurements.contactGenerationDurations.reserve(measuredTicks);
    measurements.contactGenerationFractions.reserve(measuredTicks);
    measurements.memorySamples.reserve(measuredTicks / workingSetSampleIntervalTicks + 1U);
    measurements.memoryBefore = ReadProcessMemorySample();

    const std::clock_t cpuStart = std::clock();
    const auto measurementStart = std::chrono::steady_clock::now();
    for (std::size_t tick = 0; tick < measuredTicks; ++tick)
    {
        double tickDurationMicroseconds = 0.0;
        {
            simulation::ScopedDuration measureTick(&tickDurationMicroseconds);
            simulation.AdvanceOneTick();
        }
        measurements.tickDurations.push_back(tickDurationMicroseconds);
        measurements.flecsProgressDurations.push_back(
            simulation.LastFlecsProgressDurationMicroseconds());
        const aerosimlab::simulation::EcsSystemDurations systemDurations =
            simulation.LastEcsSystemDurations();
        measurements.kinematicDroneSystemDurations.push_back(
            systemDurations.advanceKinematicDroneMicroseconds);
        measurements.rigidBodyForceSystemDurations.push_back(
            systemDurations.applyRigidBodyForcesMicroseconds);
        measurements.cameraSystemDurations.push_back(
            systemDurations.advanceCameraMicroseconds);

        if (usesRigidBodyPhysics)
        {
            const double physicsDuration = simulation.LastPhysicsStepDurationMicroseconds();
            const double contactDuration =
                simulation.LastContactGenerationDurationMicroseconds();
            measurements.physicsDurations.push_back(physicsDuration);
            measurements.contactGenerationDurations.push_back(contactDuration);
            if (physicsDuration > 0.0)
                measurements.contactGenerationFractions.push_back(
                    contactDuration / physicsDuration);
        }

        if ((tick + 1U) % workingSetSampleIntervalTicks == 0U)
        {
            if (const auto memory = ReadProcessMemorySample())
                measurements.memorySamples.push_back(*memory);
        }

        if ((tick + 1U) % 1000U == 0U
            && std::chrono::steady_clock::now() - measurementStart >= runTimeLimit)
        {
            measurements.timeLimitReached = true;
            break;
        }
    }

    measurements.wallSeconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - measurementStart).count();
    measurements.cpuSeconds = static_cast<double>(std::clock() - cpuStart) / CLOCKS_PER_SEC;
    measurements.memoryAfter = ReadProcessMemorySample();
    if (measurements.memoryAfter)
        measurements.memorySamples.push_back(*measurements.memoryAfter);
    return measurements;
}

std::vector<double> RemoveScalingDrones(
    SimulationCore& simulation, const std::vector<DroneId>& droneIds)
{
    std::vector<double> durations;
    durations.reserve(droneIds.size() - 1U);
    for (std::size_t index = droneIds.size(); index > 1U; --index)
    {
        double durationMicroseconds = 0.0;
        bool removed = false;
        {
            simulation::ScopedDuration measureRemoval(&durationMicroseconds);
            removed = simulation.RemoveDrone(droneIds[index - 1U]);
        }
        if (!removed)
            throw std::runtime_error("멀티드론 성능 측정용 드론을 삭제하지 못했습니다.");
        durations.push_back(durationMicroseconds);
    }
    return durations;
}

void AddMemorySummary(
    nlohmann::json& run, const std::vector<ProcessMemorySample>& samples,
    std::size_t sampleIntervalTicks)
{
    if (samples.empty())
        return;

    const double totalWorkingSetBytes = std::accumulate(
        samples.begin(), samples.end(), 0.0,
        [](double total, const ProcessMemorySample& sample)
        {
            return total + static_cast<double>(sample.workingSetBytes);
        });
    const double totalPrivateBytes = std::accumulate(
        samples.begin(), samples.end(), 0.0,
        [](double total, const ProcessMemorySample& sample)
        {
            return total + static_cast<double>(sample.privateBytes);
        });
    const auto maximumWorkingSet = std::max_element(
        samples.begin(), samples.end(), [](const auto& first, const auto& second)
        {
            return first.workingSetBytes < second.workingSetBytes;
        });
    const auto maximumPrivateMemory = std::max_element(
        samples.begin(), samples.end(), [](const auto& first, const auto& second)
        {
            return first.privateBytes < second.privateBytes;
        });
    const double sampleCount = static_cast<double>(samples.size());
    run["sampled_average_working_set_bytes"] = totalWorkingSetBytes / sampleCount;
    run["sampled_max_working_set_bytes"] = maximumWorkingSet->workingSetBytes;
    run["working_set_sample_interval_ticks"] = sampleIntervalTicks;
    run["working_set_sample_count"] = samples.size();
    run["sampled_average_private_bytes"] = totalPrivateBytes / sampleCount;
    run["sampled_max_private_bytes"] = maximumPrivateMemory->privateBytes;
    run["private_memory_sample_interval_ticks"] = sampleIntervalTicks;
    run["private_memory_sample_count"] = samples.size();
}

nlohmann::json BuildDroneScalingRun(
    DroneScalingMeasurements& measurements,
    std::vector<double> creationDurations,
    std::vector<double> removalDurations,
    bool usesRigidBodyPhysics,
    const char* dynamicsName,
    const char* layoutName,
    std::size_t droneCount,
    std::size_t repetition,
    std::size_t warmupTicks,
    std::size_t targetTicks,
    std::size_t sampleIntervalTicks)
{
    const std::size_t completedTicks = measurements.tickDurations.size();
    const std::size_t deadlineMisses = static_cast<std::size_t>(std::count_if(
        measurements.tickDurations.begin(), measurements.tickDurations.end(),
        [](double duration) { return duration > 1'000'000.0 / 60.0; }));
    nlohmann::json run{
        {"dynamics", dynamicsName},
        {"layout", layoutName},
        {"drone_count", droneCount},
        {"repetition", repetition + 1U},
        {"warmup_ticks", warmupTicks},
        {"target_ticks", targetTicks},
        {"measured_ticks", completedTicks},
        {"tick_duration_us", SummarizeDurations(std::move(measurements.tickDurations))},
        {"flecs_progress_duration_us", SummarizeDurations(
            std::move(measurements.flecsProgressDurations))},
        {"advance_camera_system_duration_us", SummarizeDurations(
            std::move(measurements.cameraSystemDurations))},
        {"measurement_wall_seconds", measurements.wallSeconds},
        {"simulation_ticks_per_second", measurements.wallSeconds > 0.0
            ? static_cast<double>(completedTicks) / measurements.wallSeconds : 0.0},
        {"drone_ticks_per_second", measurements.wallSeconds > 0.0
            ? static_cast<double>(droneCount) * static_cast<double>(completedTicks)
                / measurements.wallSeconds : 0.0},
        {"real_time_factor", measurements.wallSeconds > 0.0
            ? static_cast<double>(completedTicks)
                * aerosimlab::simulation::DroneModel::fixedStepSeconds / measurements.wallSeconds
            : 0.0},
        {"deadline_misses_over_16_67ms", deadlineMisses},
        {"time_limit_reached", measurements.timeLimitReached},
        {"process_cpu_percent_approx", measurements.wallSeconds > 0.0
            ? 100.0 * measurements.cpuSeconds / measurements.wallSeconds : 0.0}};
    run["entity_lifecycle"] = {
        {"additional_drone_add_duration_us", SummarizeDurations(std::move(creationDurations))},
        {"additional_drone_remove_duration_us", SummarizeDurations(std::move(removalDurations))},
        {"measured_add_count", droneCount - 1U},
        {"measured_remove_count", droneCount - 1U}};
    if (usesRigidBodyPhysics)
    {
        run["apply_rigid_body_forces_system_duration_us"] =
            SummarizeDurations(std::move(measurements.rigidBodyForceSystemDurations));
        run["physics_step_duration_us"] = SummarizeDurations(
            std::move(measurements.physicsDurations));
        run["contact_generation_duration_us"] =
            SummarizeDurations(std::move(measurements.contactGenerationDurations));
        run["contact_generation_fraction"] =
            SummarizeDurations(std::move(measurements.contactGenerationFractions));
    }
    else
    {
        run["advance_kinematic_drone_system_duration_us"] =
            SummarizeDurations(std::move(measurements.kinematicDroneSystemDurations));
    }
    if (measurements.memoryBefore && measurements.memoryAfter)
    {
        run["process_working_set_before_bytes"] = measurements.memoryBefore->workingSetBytes;
        run["process_working_set_after_bytes"] = measurements.memoryAfter->workingSetBytes;
        run["process_working_set_change_bytes"] =
            static_cast<std::int64_t>(measurements.memoryAfter->workingSetBytes)
            - static_cast<std::int64_t>(measurements.memoryBefore->workingSetBytes);
        run["process_private_bytes_before"] = measurements.memoryBefore->privateBytes;
        run["process_private_bytes_after"] = measurements.memoryAfter->privateBytes;
        run["process_private_bytes_change"] =
            static_cast<std::int64_t>(measurements.memoryAfter->privateBytes)
            - static_cast<std::int64_t>(measurements.memoryBefore->privateBytes);
    }
    AddMemorySummary(run, measurements.memorySamples, sampleIntervalTicks);
    return run;
}
}

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
    constexpr double separatedSpacingMeters = 2.5;
    constexpr double compactSpacingMeters = 0.55;
    const bool usesRigidBodyPhysics = dynamics == SimulationDynamics::RigidBody;
    const bool usesCompactLayout = layout == DroneScalingLayout::Compact;
    const std::vector<std::size_t> droneCounts = usesCompactLayout
        ? std::vector<std::size_t>{1U, 8U, 32U, 64U, 128U, 256U}
        : std::vector<std::size_t>{1U, 8U, 32U, 64U, 128U, 256U, 512U, 1024U};
    const char* dynamicsName = usesRigidBodyPhysics ? "rigid_body" : "kinematic";
    const char* layoutName = usesCompactLayout ? "compact" : "separated";
    nlohmann::json runs = nlohmann::json::array();
    bool stoppedAtTimeLimit = false;

    for (const std::size_t droneCount : droneCounts)
    {
        const std::size_t compactGridWidth = static_cast<std::size_t>(
            std::ceil(std::sqrt(static_cast<double>(droneCount))));
        for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
        {
            SimulationCore simulation{dynamics};
            simulation.SetEcsSystemTimingEnabled(true);
            simulation.SetPhysicsStepTimingEnabled(usesRigidBodyPhysics);

            std::vector<double> creationDurations;
            const std::vector<DroneId> droneIds = AddScalingDrones(
                simulation, droneCount, usesCompactLayout, separatedSpacingMeters,
                compactSpacingMeters, compactGridWidth, creationDurations);
            StartScalingDrones(simulation, droneIds);
            for (std::size_t tick = 0; tick < warmupTicks; ++tick)
                simulation.AdvanceOneTick();

            DroneScalingMeasurements measurements = MeasureScalingTicks(
                simulation, usesRigidBodyPhysics, measuredTicks,
                workingSetSampleIntervalTicks, runTimeLimit);
            std::vector<double> removalDurations = RemoveScalingDrones(simulation, droneIds);
            runs.push_back(BuildDroneScalingRun(
                measurements, std::move(creationDurations), std::move(removalDurations),
                usesRigidBodyPhysics, dynamicsName, layoutName, droneCount, repetition,
                warmupTicks, measuredTicks, workingSetSampleIntervalTicks));

            if (measurements.timeLimitReached)
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

nlohmann::json MeasureDroneMemoryScaling(
    aerosimlab::simulation::SimulationDynamics dynamics)
{
    constexpr std::array<std::size_t, 8> droneCounts{
        1U, 8U, 32U, 64U, 128U, 256U, 512U, 1024U};
    constexpr double droneSpacingMeters = 2.5;
    const char* dynamicsName = dynamics
            == aerosimlab::simulation::SimulationDynamics::RigidBody
        ? "rigid_body" : "kinematic";

    simulation::SimulationCore simulation{dynamics};
    const auto oneDroneMemory = ReadProcessMemorySample();
    const double groundOriginMeters = simulation.State().positionMeters.z;
    nlohmann::json snapshots = nlohmann::json::array();

    for (const std::size_t droneCount : droneCounts)
    {
        while (simulation.DroneCount() < droneCount)
        {
            const std::size_t nextIndex = simulation.DroneCount();
            const glm::dvec3 positionMeters{
                static_cast<double>(nextIndex) * droneSpacingMeters,
                0.0,
                groundOriginMeters};
            static_cast<void>(simulation.AddDrone(positionMeters));
        }

        nlohmann::json snapshot{
            {"drone_count", droneCount},
            {"additional_drone_count", droneCount - 1U}};
        const auto currentMemory = ReadProcessMemorySample();
        if (currentMemory)
        {
            snapshot["process_private_bytes"] = currentMemory->privateBytes;
            snapshot["process_working_set_bytes"] = currentMemory->workingSetBytes;
        }
        if (droneCount > 1U && oneDroneMemory && currentMemory)
        {
            const std::int64_t addedPrivateBytes =
                static_cast<std::int64_t>(currentMemory->privateBytes)
                - static_cast<std::int64_t>(oneDroneMemory->privateBytes);
            snapshot["private_bytes_added_since_one_drone"] = addedPrivateBytes;
            snapshot["estimate_valid"] = addedPrivateBytes >= 0;
            if (addedPrivateBytes >= 0)
            {
                snapshot["estimated_private_bytes_per_additional_drone"] =
                    static_cast<double>(addedPrivateBytes)
                    / static_cast<double>(droneCount - 1U);
            }
        }
        else
        {
            snapshot["estimate_valid"] = false;
        }
        snapshots.push_back(std::move(snapshot));
    }

    return {
        {"dynamics", dynamicsName},
        {"layout", "separated"},
        {"spacing_meters", droneSpacingMeters},
        {"method", "같은 SimulationCore에 드론을 1대부터 차례로 추가하고, 첫 1대 기준의 프로세스 메모리 증가를 기록"},
        {"memory_source", "Windows PROCESS_MEMORY_COUNTERS_EX.PrivateUsage and WorkingSetSize"},
        {"snapshots", std::move(snapshots)}};
}

#endif
}
