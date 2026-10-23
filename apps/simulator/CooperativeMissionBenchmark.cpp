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
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
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
using Json = nlohmann::json;

Json BuildFourDroneOutageScenario(SimulationDynamics dynamics)
{
    const std::filesystem::path fixturePath = std::filesystem::path{
        AEROSIMLAB_SOURCE_DIR} / "scenarios/multi-drone/four-drone-survey-outage.json";
    std::ifstream fixture(fixturePath, std::ios::binary);
    if (!fixture)
        throw std::runtime_error("4대 드론 통신 단절 시나리오를 열지 못했습니다.");

    Json scenario = Json::parse(std::string{
        std::istreambuf_iterator<char>{fixture}, std::istreambuf_iterator<char>{}});
    if (dynamics != SimulationDynamics::RigidBody)
        return scenario;

    // 강체 물리는 이륙 시간이 더 필요하므로 기존 물리 회귀 시험과 같은 조건으로 늘린다.
    constexpr std::uint64_t scenarioTicks = 1800U;
    scenario["dynamics"] = "rigid_body";
    scenario["ticks"] = scenarioTicks;
    scenario["communication_outages"][0]["end_tick"] = scenarioTicks;
    for (Json& task : scenario["mission"]["tasks"])
    {
        task["deadline_tick"] = scenarioTicks - 10U;
        task["tolerance_meters"] = 0.5;
    }
    scenario["expected"]["tolerance_meters"] = 1000.0;
    return scenario;
}

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

Json BuildCoordinatorReallocationScenario(std::size_t droneCount)
{
    Json scenario = BuildCooperativeMissionScenario(
        SimulationDynamics::Kinematic, droneCount);
    constexpr std::uint64_t missionStartTick = 90U;
    constexpr std::uint64_t outageStartTick = missionStartTick - 5U;
    constexpr std::uint64_t missionDeadlineTick = missionStartTick + 600U;
    const std::uint64_t scenarioTicks = missionDeadlineTick + 30U;
    const std::size_t outageCount = droneCount / 4U > 0U
        ? droneCount / 4U : 1U;

    scenario["ticks"] = scenarioTicks;
    scenario["expected"] = Json::object();
    for (Json& task : scenario["mission"]["tasks"])
        task["deadline_tick"] = missionDeadlineTick;

    Json outages = Json::array();
    for (std::size_t index = 0; index < outageCount; ++index)
    {
        outages.push_back({
            {"drone", "drone-" + std::to_string(index)},
            {"start_tick", outageStartTick},
            {"end_tick", scenarioTicks},
            {"heartbeat_timeout_ticks", 3U},
            {"on_timeout", "hold_position"}});
    }
    scenario["communication_outages"] = std::move(outages);
    return scenario;
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
    double& durationMicroseconds,
    std::optional<double>& missionCompletionTimeSeconds)
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
    std::size_t reassignedTasks = 0;
    std::size_t totalTaskReassignments = 0;
    std::size_t unknownDrones = 0;
    for (const auto& task : result.mission->tasks)
    {
        if (task.completed)
            ++completedTasks;
        if (task.reassignmentCount > 0U)
            ++reassignedTasks;
        totalTaskReassignments += task.reassignmentCount;
    }
    for (const auto& drone : result.drones)
    {
        if (drone.communicationStatus ==
            ScenarioResult::Drone::CommunicationStatus::Unknown)
            ++unknownDrones;
    }
    durationMicroseconds = wallSeconds * 1'000'000.0;
    if (result.mission->startedTick && result.mission->completedTick
        && *result.mission->completedTick >= *result.mission->startedTick)
    {
        missionCompletionTimeSeconds = static_cast<double>(
            *result.mission->completedTick - *result.mission->startedTick)
            * aerosimlab::simulation::DroneModel::fixedStepSeconds;
    }
    Json run{
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
        {"completed_task_count", completedTasks},
        {"reassigned_task_count", reassignedTasks},
        {"total_task_reassignment_events", totalTaskReassignments},
        {"unknown_drone_count", unknownDrones},
        {"mission_completion_percent", result.mission->completionPercent},
        {"mission_coverage_percent", result.mission->coveragePercent}};
    run["mission_started_tick"] = result.mission->startedTick
        ? Json(*result.mission->startedTick) : Json(nullptr);
    run["mission_completed_tick"] = result.mission->completedTick
        ? Json(*result.mission->completedTick) : Json(nullptr);
    run["mission_completion_time_seconds"] = missionCompletionTimeSeconds
        ? Json(*missionCompletionTimeSeconds) : Json(nullptr);
    if (result.minimumDroneSeparationMeters)
        run["minimum_drone_separation_meters"] =
            *result.minimumDroneSeparationMeters;
    else
        run["minimum_drone_separation_meters"] = nullptr;
    return run;
}

Json MeasureFourDroneOutageRecovery(SimulationDynamics dynamics)
{
    const bool usesRigidBodyPhysics = dynamics == SimulationDynamics::RigidBody;
    const char* dynamicsName = usesRigidBodyPhysics ? "rigid_body" : "kinematic";
    const std::string scenarioText = BuildFourDroneOutageScenario(dynamics).dump();
    constexpr std::size_t repetitions = 3U;
    constexpr std::size_t warmupRuns = 1U;

    for (std::size_t warmup = 0; warmup < warmupRuns; ++warmup)
    {
        const ScenarioResult result = aerosimlab::simulation::RunScenario(scenarioText);
        RequireCooperativeMissionSuccess(result, true);
    }

    Json runs = Json::array();
    std::vector<double> durationsMicroseconds;
    std::vector<double> missionCompletionTimesSeconds;
    durationsMicroseconds.reserve(repetitions);
    missionCompletionTimesSeconds.reserve(repetitions);
    for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
    {
        double durationMicroseconds = 0.0;
        std::optional<double> missionCompletionTimeSeconds;
        runs.push_back(MeasureCooperativeMissionRun(
            scenarioText, dynamicsName, 4U, repetition, warmupRuns,
            durationMicroseconds, missionCompletionTimeSeconds));
        durationsMicroseconds.push_back(durationMicroseconds);
        if (missionCompletionTimeSeconds)
            missionCompletionTimesSeconds.push_back(*missionCompletionTimeSeconds);
    }

    return {
        {"dynamics", dynamicsName},
        {"drone_count", 4U},
        {"outage_drone", "scout-2"},
        {"warmup_runs", warmupRuns},
        {"target_repetitions", repetitions},
        {"scenario_ticks", usesRigidBodyPhysics ? 1800U : 320U},
        {"outcome", "all measured runs passed; four survey tasks completed"},
        {"duration_summary_us", SummarizeDurations(std::move(durationsMicroseconds))},
        {"mission_completion_time_summary_seconds",
            SummarizeDurations(std::move(missionCompletionTimesSeconds))},
        {"runs", std::move(runs)}};
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
    std::vector<double> missionCompletionTimesSeconds;
    durationsMicroseconds.reserve(repetitions);
    missionCompletionTimesSeconds.reserve(repetitions);
    for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
    {
        double durationMicroseconds = 0.0;
        std::optional<double> missionCompletionTimeSeconds;
        nlohmann::json run = MeasureCooperativeMissionRun(
            scenarioText, dynamicsName, droneCount, repetition, warmupRuns,
            durationMicroseconds, missionCompletionTimeSeconds);
        durationsMicroseconds.push_back(durationMicroseconds);
        if (missionCompletionTimeSeconds)
            missionCompletionTimesSeconds.push_back(*missionCompletionTimeSeconds);
        runs.push_back(std::move(run));
    }
    summaries.push_back({
        {"drone_count", droneCount},
        {"scenario_duration_us", SummarizeDurations(std::move(durationsMicroseconds))},
        {"mission_completion_time_seconds",
            SummarizeDurations(std::move(missionCompletionTimesSeconds))}});
}

Json MeasureCoordinatorReallocationCount(
    std::size_t droneCount,
    std::size_t repetitions,
    std::size_t warmupRuns)
{
    const Json scenario = BuildCoordinatorReallocationScenario(droneCount);
    const std::size_t outageCount = droneCount / 4U > 0U
        ? droneCount / 4U : 1U;
    const std::string scenarioText = scenario.dump();
    for (std::size_t warmup = 0; warmup < warmupRuns; ++warmup)
    {
        const ScenarioResult result = aerosimlab::simulation::RunScenario(scenarioText);
        RequireCooperativeMissionSuccess(result, true);
    }

    Json runs = Json::array();
    std::vector<double> durationsMicroseconds;
    std::vector<double> missionCompletionTimesSeconds;
    std::vector<double> taskReassignmentEvents;
    durationsMicroseconds.reserve(repetitions);
    missionCompletionTimesSeconds.reserve(repetitions);
    taskReassignmentEvents.reserve(repetitions);
    for (std::size_t repetition = 0; repetition < repetitions; ++repetition)
    {
        double durationMicroseconds = 0.0;
        std::optional<double> missionCompletionTimeSeconds;
        Json run = MeasureCooperativeMissionRun(
            scenarioText, "kinematic", droneCount, repetition, warmupRuns,
            durationMicroseconds, missionCompletionTimeSeconds);
        run["outage_drone_count"] = outageCount;
        durationsMicroseconds.push_back(durationMicroseconds);
        taskReassignmentEvents.push_back(
            run["total_task_reassignment_events"].get<double>());
        if (missionCompletionTimeSeconds)
            missionCompletionTimesSeconds.push_back(*missionCompletionTimeSeconds);
        runs.push_back(std::move(run));
    }

    return {
        {"drone_count", droneCount},
        {"outage_drone_count", outageCount},
        {"outage_fraction_percent", 100.0 * static_cast<double>(outageCount)
            / static_cast<double>(droneCount)},
        {"layout", "separated"},
        {"mission_start_tick", 90U},
        {"outage_start_tick", 85U},
        {"target_scenario_ticks", scenario["ticks"]},
        {"warmup_runs", warmupRuns},
        {"target_repetitions", repetitions},
        {"scenario_duration_us", SummarizeDurations(std::move(durationsMicroseconds))},
        {"mission_completion_time_seconds",
            SummarizeDurations(std::move(missionCompletionTimesSeconds))},
        {"total_task_reassignment_events",
            SummarizeDurations(std::move(taskReassignmentEvents))},
        {"method", "운동학 모델에서 전체 드론의 과제를 미리 배정한 뒤 일부 드론의 통신을 끊고, 미완료 과제를 다시 나누는 실행 비용을 측정"},
        {"runs", std::move(runs)}};
}

Json MeasureCoordinatorReallocationScaling()
{
    constexpr std::size_t repetitions = 3U;
    constexpr std::size_t warmupRuns = 1U;
    const std::vector<std::size_t> droneCounts{4U, 16U, 64U, 256U};
    Json summaries = Json::array();
    Json runs = Json::array();
    for (const std::size_t droneCount : droneCounts)
    {
        Json count = MeasureCoordinatorReallocationCount(
            droneCount, repetitions, warmupRuns);
        Json countRuns = std::move(count["runs"]);
        count.erase("runs");
        summaries.push_back(std::move(count));
        for (Json& run : countRuns)
            runs.push_back(std::move(run));
    }

    return {
        {"dynamics", "kinematic"},
        {"target_counts", droneCounts},
        {"target_repetitions_per_count", repetitions},
        {"warmup_runs_per_count", warmupRuns},
        {"summaries", std::move(summaries)},
        {"runs", std::move(runs)}};
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
        {"coordinator_reallocation_scaling",
            MeasureCoordinatorReallocationScaling()},
        {"kinematic", MeasureCooperativeMissionDynamics(
            aerosimlab::simulation::SimulationDynamics::Kinematic)},
        {"rigid_body", MeasureCooperativeMissionDynamics(
            aerosimlab::simulation::SimulationDynamics::RigidBody)},
        {"four_drone_outage_recovery", {
            {"kinematic", MeasureFourDroneOutageRecovery(
                aerosimlab::simulation::SimulationDynamics::Kinematic)},
            {"rigid_body", MeasureFourDroneOutageRecovery(
                aerosimlab::simulation::SimulationDynamics::RigidBody)}}}};
}

}

#endif
