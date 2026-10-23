#include "SimulationBenchmark.h"

#ifndef __EMSCRIPTEN__

#include "simulation_core/DroneModel.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/SimulationCore.h"
#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/ScopedDuration.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <ctime>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
namespace
{
using aerosimlab::simulation::ScenarioResult;
using aerosimlab::simulation::SimulationDynamics;

nlohmann::json BuildCooperativeMissionScenario(
    SimulationDynamics dynamics, std::size_t droneCount)
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
        {"mission", {{"name", "parallel-survey"}, {"tasks", std::move(tasks)}}},
        {"expected", {
            {"tolerance_meters", expectedToleranceMeters},
            {"drones", std::move(expectedDrones)}}}};
}

void RequireCooperativeMissionSuccess(const ScenarioResult& result, bool isWarmup)
{
    if (result.passed && result.mission && result.mission->passed)
        return;
    if (isWarmup)
        throw std::runtime_error("공동 임무 성능 측정의 준비 실행이 통과하지 못했습니다.");

    const std::string reason = result.issues.empty()
        ? "임무 완료 판정이 실패했습니다."
        : result.issues.front();
    throw std::runtime_error("공동 임무 성능 측정 결과가 통과하지 못했습니다: " + reason);
}

nlohmann::json MeasureCooperativeMissionRun(
    const std::string& scenarioText,
    const char* dynamicsName,
    std::size_t droneCount,
    std::size_t repetition,
    std::size_t warmupRuns,
    double& durationMicroseconds)
{
    const std::clock_t cpuStart = std::clock();
    double scenarioDurationMicroseconds = 0.0;
    const ScenarioResult result = [&]
    {
        simulation::ScopedDuration measureScenario(&scenarioDurationMicroseconds);
        return aerosimlab::simulation::RunScenario(scenarioText);
    }();
    const double wallSeconds = scenarioDurationMicroseconds / 1'000'000.0;
    const double cpuSeconds = static_cast<double>(std::clock() - cpuStart) / CLOCKS_PER_SEC;
    RequireCooperativeMissionSuccess(result, false);

    std::size_t completedTasks = 0;
    for (const auto& task : result.mission->tasks)
    {
        if (task.completed)
            ++completedTasks;
    }
    durationMicroseconds = wallSeconds * 1'000'000.0;
    return {
        {"dynamics", dynamicsName},
        {"drone_count", droneCount},
        {"repetition", repetition + 1U},
        {"warmup_runs", warmupRuns},
        {"scenario_ticks", result.tickCount},
        {"scenario_duration_us", durationMicroseconds},
        {"scenario_ticks_per_second", wallSeconds > 0.0
            ? static_cast<double>(result.tickCount) / wallSeconds : 0.0},
        {"drone_ticks_per_second", wallSeconds > 0.0
            ? static_cast<double>(droneCount) * static_cast<double>(result.tickCount)
                / wallSeconds : 0.0},
        {"process_cpu_percent_approx", wallSeconds > 0.0
            ? 100.0 * cpuSeconds / wallSeconds : 0.0},
        {"scenario_passed", result.passed},
        {"scenario_outcome", aerosimlab::simulation::ToString(result.outcome)},
        {"mission_outcome", aerosimlab::simulation::ToString(result.mission->outcome)},
        {"mission_passed", result.mission->passed},
        {"mission_task_count", result.mission->tasks.size()},
        {"completed_task_count", completedTasks}};
}

void MeasureCooperativeMissionCount(
    SimulationDynamics dynamics,
    const char* dynamicsName,
    std::size_t droneCount,
    std::size_t repetitions,
    std::size_t warmupRuns,
    nlohmann::json& runs,
    nlohmann::json& summaries)
{
    const std::string scenarioText =
        BuildCooperativeMissionScenario(dynamics, droneCount).dump();
    for (std::size_t warmup = 0; warmup < warmupRuns; ++warmup)
    {
        const ScenarioResult result = aerosimlab::simulation::RunScenario(scenarioText);
        RequireCooperativeMissionSuccess(result, true);
    }

    std::vector<double> durationsMicroseconds;
    durationsMicroseconds.reserve(repetitions);
    for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
    {
        double durationMicroseconds = 0.0;
        nlohmann::json run = MeasureCooperativeMissionRun(
            scenarioText, dynamicsName, droneCount, repetition, warmupRuns,
            durationMicroseconds);
        durationsMicroseconds.push_back(durationMicroseconds);
        runs.push_back(std::move(run));
    }
    summaries.push_back({
        {"drone_count", droneCount},
        {"scenario_duration_us", SummarizeDurations(std::move(durationsMicroseconds))}});
}

nlohmann::json MeasureCooperativeMissionDynamics(SimulationDynamics dynamics)
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

    for (const std::size_t droneCount : droneCounts)
        MeasureCooperativeMissionCount(
            dynamics, dynamicsName, droneCount, repetitions, warmupRuns, runs, summaries);

    const std::uint64_t missionStartTick = usesRigidBodyPhysics ? 270U : 90U;
    return {
        {"dynamics", dynamicsName},
        {"layout", "separated"},
        {"spacing_meters", 5.0},
        {"mission_start_tick", missionStartTick},
        {"target_scenario_ticks", missionStartTick + 210U},
        {"warmup_runs_per_count", warmupRuns},
        {"target_repetitions_per_count", repetitions},
        {"target_counts", droneCounts},
        {"measured_counts", droneCounts},
        {"method", "실제 JSON 시나리오 실행기로 모든 드론에 조사 목표를 배정하고, 비행·목표 도달·임무 완료 판정을 함께 측정"},
        {"summaries", std::move(summaries)},
        {"runs", std::move(runs)}};
}
}

nlohmann::json MeasureCooperativeMissionScaling()
{
    return {
        {"kinematic", MeasureCooperativeMissionDynamics(
            aerosimlab::simulation::SimulationDynamics::Kinematic)},
        {"rigid_body", MeasureCooperativeMissionDynamics(
            aerosimlab::simulation::SimulationDynamics::RigidBody)}};
}

}

#endif
