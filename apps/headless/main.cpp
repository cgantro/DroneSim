#include "simulation_core/ScenarioRunner.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
constexpr std::string_view helpText =
    "사용법: aerosimlab_headless --scenario <JSON 파일>\n"
    "        aerosimlab_headless --help\n";
constexpr std::uintmax_t maximumScenarioFileBytes = 16 * 1024 * 1024;

nlohmann::json ToJson(const aerosimlab::simulation::DroneState& state)
{
    return {
        {"mode", aerosimlab::simulation::ToString(state.mode)},
        {"position_meters", {state.positionMeters.x, state.positionMeters.y, state.positionMeters.z}},
        {"velocity_meters_per_second", {
            state.velocityMetersPerSecond.x,
            state.velocityMetersPerSecond.y,
            state.velocityMetersPerSecond.z}},
        {"orientation_quaternion_wxyz", {
            state.orientation.w,
            state.orientation.x,
            state.orientation.y,
            state.orientation.z}},
        {"motor_speeds_radians_per_second", state.motorSpeedsRadiansPerSecond},
        {"yaw_radians", state.yawRadians}};
}

nlohmann::json ToJson(const aerosimlab::simulation::ScenarioResult& result)
{
    using CommunicationStatus =
        aerosimlab::simulation::ScenarioResult::Drone::CommunicationStatus;
    const auto communicationStatusName = [](CommunicationStatus status)
    {
        switch (status)
        {
        case CommunicationStatus::Connected: return "connected";
        case CommunicationStatus::Stale: return "stale";
        case CommunicationStatus::Unknown: return "unknown";
        }
        return "unknown";
    };

    nlohmann::json drones = nlohmann::json::array();
    for (const auto& drone : result.drones)
    {
        nlohmann::json battery = nullptr;
        if (drone.batteryCapacityWattHours)
        {
            battery = {
                {"capacity_watt_hours", drone.batteryCapacityWattHours},
                {"consumed_watt_hours", drone.energyConsumedWattHours},
                {"remaining_watt_hours", drone.batteryRemainingWattHours},
                {"remaining_percent", drone.batteryPercent},
                {"reported_percent", drone.reportedBatteryPercent},
                {"returned_for_low_battery", drone.returnedForLowBattery}};
        }
        drones.push_back({
            {"name", drone.name},
            {"state", ToJson(drone.finalState)},
            {"communication_status", communicationStatusName(drone.communicationStatus)},
            {"last_heartbeat_tick", drone.lastHeartbeatTick},
            {"battery", std::move(battery)}});
    }

    nlohmann::json mission = nullptr;
    if (result.mission)
    {
        nlohmann::json tasks = nlohmann::json::array();
        for (const auto& task : result.mission->tasks)
        {
            nlohmann::json waypoints = nlohmann::json::array();
            for (const glm::dvec3& waypoint : task.waypointsMeters)
            {
                waypoints.push_back({waypoint.x, waypoint.y, waypoint.z});
            }
            tasks.push_back({
                {"name", task.name},
                {"role", task.role},
                {"drone", task.droneName},
                {"initial_drone", task.initialDroneName},
                {"assignment_history", task.assignmentHistory},
                {"reassignment_count", task.reassignmentCount},
                {"target_position_meters", {
                    task.targetPositionMeters.x,
                    task.targetPositionMeters.y,
                    task.targetPositionMeters.z}},
                {"waypoints_meters", std::move(waypoints)},
                {"current_waypoint_index", task.currentWaypointIndex},
                {"started", task.started},
                {"completed", task.completed},
                {"failed", task.failed},
                {"completed_tick", task.completedTick}});
        }
        mission = {{"name", result.mission->name},
            {"passed", result.mission->passed},
            {"completed_task_count", result.mission->completedTaskCount},
            {"task_count", result.mission->tasks.size()},
            {"completion_percent", result.mission->completionPercent},
            {"coverage_percent", result.mission->coveragePercent},
            {"started_tick", result.mission->startedTick},
            {"completed_tick", result.mission->completedTick},
            {"completion_time_seconds", result.mission->completedTick
                    && result.mission->startedTick
                ? nlohmann::json(static_cast<double>(*result.mission->completedTick
                    - *result.mission->startedTick)
                    * aerosimlab::simulation::DroneModel::fixedStepSeconds)
                : nlohmann::json(nullptr)},
            {"tasks", std::move(tasks)}};
    }

    nlohmann::json physicalModel = nullptr;
    if (result.physicalModel)
    {
        const auto& model = *result.physicalModel;
        physicalModel = {
            {"mass_kg", model.massKilograms},
            {"principal_inertia_kg_m2", {
                model.principalInertiaKilogramMetersSquared.x,
                model.principalInertiaKilogramMetersSquared.y,
                model.principalInertiaKilogramMetersSquared.z}},
            {"thrust_coefficient_newtons_per_speed_squared",
                model.thrustCoefficientNewtonsPerSpeedSquared},
            {"moment_to_thrust_ratio", model.momentToThrustRatio},
            {"motor_time_constant_up_seconds", model.motorTimeConstantUpSeconds},
            {"motor_time_constant_down_seconds", model.motorTimeConstantDownSeconds},
            {"maximum_motor_speed_radians_per_second",
                model.maximumMotorSpeedRadiansPerSecond},
            {"customized", model.customized}};
    }

    nlohmann::json controllerSettings = nullptr;
    if (result.controllerSettings)
    {
        const auto& controller = *result.controllerSettings;
        controllerSettings = {
            {"position", controller.positionGain},
            {"velocity", controller.velocityGain},
            {"attitude", controller.attitudeGain},
            {"angular_rate", controller.angularRateGain},
            {"customized", controller.customized}};
    }

    return {
        {"passed", result.passed},
        {"tick_count", result.tickCount},
        {"simulation_time_seconds", result.simulationTimeSeconds},
        {"final_state", ToJson(result.finalState)},
        {"drones", std::move(drones)},
        {"physical_model", std::move(physicalModel)},
        {"controller_gains", std::move(controllerSettings)},
        {"mission", std::move(mission)},
        {"minimum_drone_separation_meters", result.minimumDroneSeparationMeters},
        {"minimum_predicted_drone_separation_meters",
            result.minimumPredictedDroneSeparationMeters},
        {"collision_avoidance_events", result.collisionAvoidanceEvents},
        {"camera", {
            {"status", aerosimlab::simulation::ToString(result.finalCamera.status)},
            {"frame_count", result.finalCamera.frameCount},
            {"last_frame_tick", result.finalCamera.lastFrameTick}
        }},
        {"events", result.eventLog},
        {"issues", result.issues}
    };
}
}

int main(int argc, char** argv)
{
    if (argc == 2 && std::string_view(argv[1]) == "--help")
    {
        std::cout << helpText;
        return 0;
    }
    if (argc != 3 || std::string_view(argv[1]) != "--scenario")
    {
        std::cerr << helpText;
        return 2;
    }

    try
    {
        const std::filesystem::path scenarioPath(argv[2]);
        const std::uintmax_t fileSize = std::filesystem::file_size(scenarioPath);
        if (fileSize > maximumScenarioFileBytes)
        {
            throw std::invalid_argument("시나리오 파일은 16 MiB를 넘을 수 없습니다.");
        }

        std::ifstream input(scenarioPath, std::ios::binary);
        if (!input)
        {
            throw std::runtime_error("시나리오 파일을 열 수 없습니다.");
        }
        const std::string jsonText{
            std::istreambuf_iterator<char>{input},
            std::istreambuf_iterator<char>{}};
        const auto result = aerosimlab::simulation::RunScenario(jsonText);
        std::cout << ToJson(result).dump(2) << '\n';
        return result.passed ? 0 : 1;
    }
    catch (const std::exception& error)
    {
        std::cerr << "시나리오 실행 실패: " << error.what() << '\n';
        return 2;
    }
}
