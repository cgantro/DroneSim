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
#include <optional>
#include <stdexcept>
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
        {"method", "헤드리스 강체 모의; 한 Jolt 세계에서 모든 드론을 함께 갱신. 각 실행은 워밍업 뒤 30,000 Tick을 측정하고 프로세스 CPU 시간과 작업 집합을 기록"},
        {"counts", droneCounts},
        {"repetitions_per_count", repetitions},
        {"runs", std::move(runs)}};
}
#endif
}
