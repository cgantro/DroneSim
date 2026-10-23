#include "simulation_core/ScenarioRunner.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

namespace aerosimlab::simulation
{
namespace
{
std::string ReadScenarioFixture(std::string_view relativePath)
{
    const std::filesystem::path path =
        std::filesystem::path{AEROSIMLAB_SOURCE_DIR} / relativePath;
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("시나리오 시험 자료를 열지 못했습니다.");
    return {std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

void Verify_ScenarioRunner_UsesAndReportsPhysicalModelOverrides()
{
    const ScenarioResult result = RunScenario(
        ReadScenarioFixture("scenarios/flight/physical-model-override.json"));
    ASSERT_TRUE(result.passed)
        << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_TRUE(result.physicalModel.has_value());
    EXPECT_TRUE(result.physicalModel->customized);
    EXPECT_DOUBLE_EQ(result.physicalModel->massKilograms, 2.5);
    EXPECT_EQ(result.physicalModel->principalInertiaKilogramMetersSquared,
        (glm::dvec3{0.03, 0.031, 0.05}));
    EXPECT_DOUBLE_EQ(
        result.physicalModel->thrustCoefficientNewtonsPerSpeedSquared, 0.000011);
    EXPECT_DOUBLE_EQ(result.physicalModel->momentToThrustRatio, 0.015);
    EXPECT_DOUBLE_EQ(result.physicalModel->motorTimeConstantUpSeconds, 0.02);
    EXPECT_DOUBLE_EQ(result.physicalModel->maximumMotorSpeedRadiansPerSecond, 900.0);

    constexpr std::string_view kinematicOverride = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,"commands":[],
     "quadrotor_model":{"mass_kg":2.5}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(kinematicOverride)),
        std::invalid_argument);
}
}

void Verify_ScenarioRunner_RunsNamedDronesWithIndependentCommandsAndDeliveryOrder()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":180,
     "drones":[
       {"name":"lead","initial_position_meters":[0,0,0]},
       {"name":"scout","initial_position_meters":[10,0,0],"initial_yaw_radians":1.2}],
     "commands":[
       {"tick":0,"drone":"lead","type":"takeoff","delay_ticks":5},
       {"tick":0,"drone":"scout","type":"takeoff"},
       {"tick":90,"drone":"scout","type":"set_waypoint","position_meters":[12,0,1.5]}],
     "expected":{"tolerance_meters":0.000001,"orientation_tolerance_radians":0.000001,
       "minimum_drone_separation_meters":9.9,"drones":[
       {"name":"lead","mode":"hovering","position_meters":[0,0,1.5]},
       {"name":"scout","mode":"hovering","position_meters":[12,0,1.5],
        "orientation_quaternion_wxyz":[-0.8253356149096783,0,0,-0.5646424733950354]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_EQ(result.drones.size(), 2);
    EXPECT_EQ(result.drones[0].name, "lead");
    EXPECT_EQ(result.drones[1].name, "scout");
    EXPECT_EQ(result.drones[0].finalState.mode, FlightMode::Hovering);
    EXPECT_EQ(result.drones[1].finalState.mode, FlightMode::Hovering);
    EXPECT_NEAR(result.drones[1].finalState.yawRadians, 1.2, 1.0e-12);
    EXPECT_DOUBLE_EQ(result.drones[0].finalState.positionMeters.z, 1.5);
    EXPECT_DOUBLE_EQ(result.drones[1].finalState.positionMeters.x, 12.0);
    EXPECT_EQ(result.finalState.positionMeters, result.drones[0].finalState.positionMeters);
    ASSERT_GE(result.eventLog.size(), 3);
    EXPECT_NE(result.eventLog[1].find("늦게 전달"), std::string::npos);
    ASSERT_TRUE(result.minimumDroneSeparationMeters.has_value());
    EXPECT_NEAR(*result.minimumDroneSeparationMeters, 10.0, 1.0e-12);

    std::string unsafeScenario{scenario};
    unsafeScenario.replace(unsafeScenario.find("9.9"), 3, "10.1");
    const ScenarioResult unsafeResult = RunScenario(unsafeScenario);
    EXPECT_FALSE(unsafeResult.passed);
    ASSERT_EQ(unsafeResult.issues.size(), 1);
    EXPECT_NE(unsafeResult.issues.front().find("최소 거리"), std::string::npos);

    std::string wrongOrientationScenario{scenario};
    const std::string expectedOrientation =
        "-0.8253356149096783,0,0,-0.5646424733950354";
    wrongOrientationScenario.replace(
        wrongOrientationScenario.find(expectedOrientation), expectedOrientation.size(), "1,0,0,0");
    const ScenarioResult wrongOrientationResult = RunScenario(wrongOrientationScenario);
    EXPECT_FALSE(wrongOrientationResult.passed);
    ASSERT_EQ(wrongOrientationResult.issues.size(), 1);
    EXPECT_NE(wrongOrientationResult.issues.front().find("최종 자세"), std::string::npos);

    constexpr std::string_view communicationFaultScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":100,
     "commands":[
       {"tick":0,"type":"takeoff","delay_ticks":10},
       {"tick":1,"type":"takeoff","drop":true},
       {"tick":2,"type":"takeoff","duplicate":true}],
     "expected":{"mode":"hovering","position_meters":[0,0,1.5],
       "tolerance_meters":0.000001}}
    )json";

    const ScenarioResult communicationFaults = RunScenario(communicationFaultScenario);
    EXPECT_FALSE(communicationFaults.passed);
    ASSERT_EQ(communicationFaults.issues.size(), 1);
    EXPECT_NE(communicationFaults.issues.front().find("역순 명령"), std::string::npos);
    ASSERT_EQ(communicationFaults.eventLog.size(), 3);
    EXPECT_NE(communicationFaults.eventLog[0].find("통신 장애"), std::string::npos);
    EXPECT_NE(communicationFaults.eventLog[1].find("명령을 적용했습니다"), std::string::npos);
    EXPECT_NE(communicationFaults.eventLog[2].find("중복 재전송"), std::string::npos);
    EXPECT_EQ(communicationFaults.finalState.mode, FlightMode::Hovering);
}

void Verify_ScenarioRunner_FollowerHoldsItsLastTargetDuringLinkOutageAndResumesAfterward()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":300,
     "drones":[
       {"name":"lead","initial_position_meters":[0,0,0]},
       {"name":"wing","initial_position_meters":[-2,0,0]}],
     "formation_links":[
       {"leader":"lead","follower":"wing","offset_meters":[-2,0,0],
        "outages":[{"start_tick":120,"end_tick":180}]}],
     "commands":[
       {"tick":0,"drone":"lead","type":"takeoff"},
       {"tick":0,"drone":"wing","type":"takeoff"},
       {"tick":90,"drone":"lead","type":"set_waypoint","position_meters":[4,0,1.5]}],
     "expected":{"tolerance_meters":0.05,"minimum_drone_separation_meters":1.9,
       "drones":[
         {"name":"lead","mode":"hovering","position_meters":[4,0,1.5]},
         {"name":"wing","mode":"hovering","position_meters":[2,0,1.5]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_TRUE(result.minimumDroneSeparationMeters.has_value());
    EXPECT_GT(*result.minimumDroneSeparationMeters, 1.9);
    ASSERT_EQ(result.eventLog.size(), 5);
    EXPECT_NE(result.eventLog[3].find("위치 공유가 끊겼습니다"), std::string::npos);
    EXPECT_NE(result.eventLog[4].find("위치 공유가 복구됐습니다"), std::string::npos);
}

void Verify_ScenarioRunner_FindsClosestPairAcrossThreeDrones()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,
     "drones":[
       {"name":"first","initial_position_meters":[0,0,0]},
       {"name":"second","initial_position_meters":[3,0,0]},
       {"name":"third","initial_position_meters":[0,2,0]}],
     "commands":[],
     "expected":{"tolerance_meters":0.0,"minimum_drone_separation_meters":1.9,
       "drones":[
         {"name":"third","mode":"grounded","position_meters":[0,2,0]},
         {"name":"first","mode":"grounded","position_meters":[0,0,0]},
         {"name":"second","mode":"grounded","position_meters":[3,0,0]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_TRUE(result.minimumDroneSeparationMeters.has_value());
    EXPECT_DOUBLE_EQ(*result.minimumDroneSeparationMeters, 2.0);
}

void Verify_ScenarioRunner_ReassignsAnUnconfirmedSurveyArea()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":320,
     "drones":[
       {"name":"scout-1","initial_position_meters":[0,0,0]},
       {"name":"scout-2","initial_position_meters":[10,0,0]},
       {"name":"scout-3","initial_position_meters":[20,0,0]},
       {"name":"scout-4","initial_position_meters":[30,0,0]}],
     "commands":[
       {"tick":0,"drone":"scout-1","type":"takeoff"},
       {"tick":0,"drone":"scout-2","type":"takeoff"},
       {"tick":0,"drone":"scout-3","type":"takeoff"},
       {"tick":0,"drone":"scout-4","type":"takeoff"}],
     "communication_outages":[
       {"drone":"scout-2","start_tick":95,"end_tick":320,"heartbeat_timeout_ticks":3}],
     "mission":{"name":"four-sector-survey","tasks":[
       {"name":"sector-1","role":"survey","drone":"scout-1",
        "target_position_meters":[4,0,1.5],"start_tick":90,"deadline_tick":310,"tolerance_meters":0.1},
       {"name":"sector-2","role":"survey","drone":"scout-2",
        "target_position_meters":[14,0,1.5],"start_tick":90,"deadline_tick":310,"tolerance_meters":0.1},
       {"name":"sector-3","role":"survey","drone":"scout-3",
        "target_position_meters":[24,0,1.5],"start_tick":90,"deadline_tick":310,"tolerance_meters":0.1},
       {"name":"sector-4","role":"survey","drone":"scout-4",
        "target_position_meters":[34,0,1.5],"start_tick":90,"deadline_tick":310,"tolerance_meters":0.1}]},
     "expected":{"tolerance_meters":0.01,"drones":[
       {"name":"scout-1","mode":"hovering","position_meters":[4,0,1.5]},
       {"name":"scout-2","mode":"following_waypoint","position_meters":[10.333333333333336,0,1.5]},
       {"name":"scout-3","mode":"hovering","position_meters":[14,0,1.5]},
       {"name":"scout-4","mode":"hovering","position_meters":[24,0,1.5]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    ASSERT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_TRUE(result.mission.has_value());
    EXPECT_TRUE(result.mission->passed);
    ASSERT_EQ(result.mission->tasks.size(), 4);
    EXPECT_EQ(result.mission->completedTaskCount, 4U);
    EXPECT_DOUBLE_EQ(result.mission->completionPercent, 100.0);
    EXPECT_DOUBLE_EQ(result.mission->coveragePercent, 100.0);
    ASSERT_TRUE(result.mission->completedTick.has_value());
    EXPECT_LT(*result.mission->completedTick, result.tickCount);
    const auto& reassignedTask = result.mission->tasks[1];
    EXPECT_TRUE(reassignedTask.completed);
    EXPECT_EQ(reassignedTask.droneName, "scout-3");
    EXPECT_EQ(reassignedTask.reassignmentCount, 1U);
    ASSERT_EQ(reassignedTask.assignmentHistory.size(), 2U);
    EXPECT_EQ(reassignedTask.assignmentHistory[0], "scout-2");
    EXPECT_EQ(reassignedTask.assignmentHistory[1], "scout-3");

    const auto lostDrone = std::find_if(result.drones.begin(), result.drones.end(),
        [](const ScenarioResult::Drone& drone) { return drone.name == "scout-2"; });
    ASSERT_NE(lostDrone, result.drones.end());
    EXPECT_EQ(lostDrone->communicationStatus,
        ScenarioResult::Drone::CommunicationStatus::Unknown);
    EXPECT_LT(lostDrone->lastHeartbeatTick, result.tickCount);
    EXPECT_LT(lostDrone->finalState.positionMeters.x, 14.0);

    constexpr std::string_view recoveredLinkScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":10,
     "drones":[{"name":"scout","initial_position_meters":[0,0,0]}],
     "commands":[],
     "communication_outages":[
       {"drone":"scout","start_tick":3,"end_tick":6,"heartbeat_timeout_ticks":2}],
     "expected":{"drones":[
       {"name":"scout","mode":"grounded","position_meters":[0,0,0]}]}}
    )json";
    const ScenarioResult recoveredResult = RunScenario(recoveredLinkScenario);
    ASSERT_TRUE(recoveredResult.passed);
    ASSERT_EQ(recoveredResult.drones.size(), 1);
    EXPECT_EQ(recoveredResult.drones.front().communicationStatus,
        ScenarioResult::Drone::CommunicationStatus::Connected);
    EXPECT_EQ(recoveredResult.drones.front().lastHeartbeatTick, 10U);
    EXPECT_TRUE(std::any_of(recoveredResult.eventLog.begin(), recoveredResult.eventLog.end(),
        [](const std::string& event)
        {
            return event.find("통신이 복구") != std::string::npos;
        }));

    constexpr std::string_view dividedSurveyScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":480,
     "drones":[
       {"name":"survey-1","initial_position_meters":[2,0,0]},
       {"name":"survey-2","initial_position_meters":[6,0,0]},
       {"name":"survey-3","initial_position_meters":[10,0,0]},
       {"name":"survey-4","initial_position_meters":[14,0,0]}],
     "commands":[
       {"tick":0,"drone":"survey-1","type":"takeoff"},
       {"tick":0,"drone":"survey-2","type":"takeoff"},
       {"tick":0,"drone":"survey-3","type":"takeoff"},
       {"tick":0,"drone":"survey-4","type":"takeoff"}],
     "mission":{"name":"grid-survey","survey_area":{
       "name":"field","bounds_meters":[0,0,16,8],
       "altitude_meters":1.5,"lane_spacing_meters":8,
       "start_tick":100,"deadline_tick":470,"tolerance_meters":0.2}},
     "expected":{"tolerance_meters":0.2,"drones":[
       {"name":"survey-1","mode":"hovering","position_meters":[0,8,1.5]},
       {"name":"survey-2","mode":"hovering","position_meters":[4,8,1.5]},
       {"name":"survey-3","mode":"hovering","position_meters":[8,8,1.5]},
       {"name":"survey-4","mode":"hovering","position_meters":[12,8,1.5]}]}}
    )json";
    const ScenarioResult areaResult = RunScenario(dividedSurveyScenario);
    ASSERT_TRUE(areaResult.passed) << (areaResult.issues.empty() ? "" : areaResult.issues.front());
    ASSERT_TRUE(areaResult.mission.has_value());
    EXPECT_TRUE(areaResult.mission->passed);
    EXPECT_DOUBLE_EQ(areaResult.mission->coveragePercent, 100.0);
    ASSERT_EQ(areaResult.mission->tasks.size(), 4);
    for (std::size_t index = 0; index < areaResult.mission->tasks.size(); ++index)
    {
        const auto& task = areaResult.mission->tasks[index];
        EXPECT_TRUE(task.completed) << task.name;
        ASSERT_EQ(task.waypointsMeters.size(), 4);
        EXPECT_DOUBLE_EQ(task.waypointsMeters[0].x, 4.0 * static_cast<double>(index));
        EXPECT_DOUBLE_EQ(task.waypointsMeters[1].x, 4.0 * static_cast<double>(index + 1));
        EXPECT_DOUBLE_EQ(task.waypointsMeters.back().y, 8.0);
    }

    constexpr std::string_view collisionAvoidanceScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":600,
     "drones":[
       {"name":"west","initial_position_meters":[-4,0,0]},
       {"name":"east","initial_position_meters":[4,0,0]}],
     "commands":[
       {"tick":0,"drone":"west","type":"takeoff"},
       {"tick":0,"drone":"east","type":"takeoff"}],
     "collision_avoidance":{"safety_distance_meters":3,
       "prediction_horizon_seconds":2,"lateral_offset_meters":10},
     "mission":{"name":"crossing-flight","tasks":[
       {"name":"west-to-east","role":"cross","drone":"west",
        "target_position_meters":[4,0,1.5],"start_tick":100,"deadline_tick":580,"tolerance_meters":0.2},
       {"name":"east-to-west","role":"cross","drone":"east",
        "target_position_meters":[-4,0,1.5],"start_tick":100,"deadline_tick":580,"tolerance_meters":0.2}]},
     "expected":{"tolerance_meters":0.5,"minimum_drone_separation_meters":2.5,
       "drones":[
         {"name":"west","mode":"hovering","position_meters":[4,0,1.5]},
         {"name":"east","mode":"hovering","position_meters":[-4,0,1.5]}]}}
    )json";
    const ScenarioResult avoidanceResult = RunScenario(collisionAvoidanceScenario);
    EXPECT_TRUE(avoidanceResult.passed)
        << (avoidanceResult.issues.empty() ? "" : avoidanceResult.issues.front())
        << " minimum=" << avoidanceResult.minimumDroneSeparationMeters.value_or(-1.0)
        << " avoidance_events=" << avoidanceResult.collisionAvoidanceEvents;
    ASSERT_TRUE(avoidanceResult.mission.has_value());
    EXPECT_TRUE(avoidanceResult.mission->passed);
    EXPECT_GT(avoidanceResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(avoidanceResult.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*avoidanceResult.minimumDroneSeparationMeters, 2.5);
    ASSERT_TRUE(avoidanceResult.minimumPredictedDroneSeparationMeters.has_value());
    EXPECT_LT(*avoidanceResult.minimumPredictedDroneSeparationMeters, 3.0);

    const std::string lowBatteryScenario = ReadScenarioFixture(
        "scenarios/multi-drone/low-battery-reallocation.json");
    const ScenarioResult energyResult = RunScenario(lowBatteryScenario);
    ASSERT_TRUE(energyResult.passed)
        << (energyResult.issues.empty() ? "" : energyResult.issues.front());
    ASSERT_TRUE(energyResult.mission.has_value());
    EXPECT_TRUE(energyResult.mission->passed);
    ASSERT_EQ(energyResult.mission->tasks.size(), 2);
    EXPECT_EQ(energyResult.mission->tasks[0].droneName, "reserve");
    EXPECT_EQ(energyResult.mission->tasks[0].reassignmentCount, 1U);
    ASSERT_EQ(energyResult.drones.size(), 2);
    const auto& depletedDrone = energyResult.drones[0];
    EXPECT_TRUE(depletedDrone.returnedForLowBattery);
    ASSERT_TRUE(depletedDrone.batteryPercent.has_value());
    ASSERT_TRUE(depletedDrone.energyConsumedWattHours.has_value());
    EXPECT_LT(*depletedDrone.batteryPercent, 40.0);
    EXPECT_GT(*depletedDrone.batteryPercent, 0.0);
    EXPECT_GT(*depletedDrone.energyConsumedWattHours, 0.0);
    EXPECT_EQ(depletedDrone.finalState.mode, FlightMode::Grounded);
    EXPECT_LT(depletedDrone.finalState.positionMeters.x, 30.0);
    const auto& healthyDrone = energyResult.drones[1];
    EXPECT_FALSE(healthyDrone.returnedForLowBattery);
    ASSERT_TRUE(healthyDrone.batteryPercent.has_value());
    EXPECT_GT(*healthyDrone.batteryPercent, 40.0);
    EXPECT_NEAR(healthyDrone.finalState.positionMeters.x, 30.0, 0.2);

    constexpr std::string_view insufficientLandingReserve = R"json(
    {"version":1,"dynamics":"kinematic","ticks":10,
     "drones":[
       {"name":"first","initial_position_meters":[0,0,0],"battery_capacity_watt_hours":0.15},
       {"name":"second","initial_position_meters":[1,0,0]}],
     "commands":[],
     "energy_model":{"capacity_watt_hours":1,"hover_power_watts":40,
       "flight_power_watts":120,"reserve_percent":20},
     "expected":{"drones":[
       {"name":"first","mode":"grounded","position_meters":[0,0,0]},
       {"name":"second","mode":"grounded","position_meters":[1,0,0]}]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(insufficientLandingReserve)),
        std::invalid_argument);
}

TEST(ScenarioRunnerTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_UsesAndReportsPhysicalModelOverrides());
    constexpr std::array flightModes{
        FlightMode::Grounded,
        FlightMode::TakingOff,
        FlightMode::Hovering,
        FlightMode::Flying,
        FlightMode::FollowingWaypoint,
        FlightMode::Landing};
    for (const FlightMode mode : flightModes)
    {
        const auto parsed = TryParseFlightMode(ToString(mode));
        ASSERT_TRUE(parsed.has_value());
        EXPECT_EQ(*parsed, mode);
    }
    EXPECT_FALSE(TryParseFlightMode("unknown_mode").has_value());

    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_RunsNamedDronesWithIndependentCommandsAndDeliveryOrder());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_FollowerHoldsItsLastTargetDuringLinkOutageAndResumesAfterward());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_FindsClosestPairAcrossThreeDrones());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_ReassignsAnUnconfirmedSurveyArea());

    constexpr std::string_view duplicateMissionAssignment = R"json(
    {"version":1,"dynamics":"kinematic","ticks":2,
     "drones":[{"name":"lead"},{"name":"wing"}],
     "mission":{"name":"survey","tasks":[
       {"name":"first","role":"scan","drone":"lead",
        "target_position_meters":[0,0,1],"start_tick":0,"deadline_tick":1,"tolerance_meters":0},
       {"name":"second","role":"guard","drone":"lead",
        "target_position_meters":[1,0,1],"start_tick":0,"deadline_tick":1,"tolerance_meters":0}]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(duplicateMissionAssignment)), std::invalid_argument);
}

}
