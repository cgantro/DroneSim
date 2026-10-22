#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/QuadrotorFlightController.h"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <string_view>

namespace aerosimlab::simulation
{
namespace
{
constexpr std::string_view completeFlightScenario = R"json(
{
  "version": 1,
  "dynamics": "kinematic",
  "ticks": 300,
  "commands": [
    {"tick": 0, "type": "takeoff"},
    {"tick": 90, "type": "set_waypoint", "position_meters": [3.0, 0.0, 1.5]},
    {"tick": 150, "type": "stop"},
    {"tick": 200, "type": "land"}
  ],
  "expected": {
    "mode": "grounded",
    "position_meters": [3.0, 0.0, 0.0],
    "orientation_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    "orientation_tolerance_radians": 0.000001,
    "tolerance_meters": 0.000001
  }
})json";
}

void Verify_ScenarioRunner_ReplaysFlightCommandsAndChecksFinalState()
{
    const ScenarioResult result = RunScenario(completeFlightScenario);
    const ScenarioResult repeatedResult = RunScenario(completeFlightScenario);
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    EXPECT_EQ(result.tickCount, 300);
    EXPECT_DOUBLE_EQ(result.simulationTimeSeconds, 5.0);
    EXPECT_EQ(result.finalState.mode, FlightMode::Grounded);
    EXPECT_DOUBLE_EQ(result.finalState.positionMeters.x, 3.0);
    EXPECT_DOUBLE_EQ(result.finalState.positionMeters.z, 0.0);
    EXPECT_EQ(result.passed, repeatedResult.passed);
    EXPECT_EQ(result.tickCount, repeatedResult.tickCount);
    EXPECT_DOUBLE_EQ(result.simulationTimeSeconds, repeatedResult.simulationTimeSeconds);
    EXPECT_EQ(result.finalState.positionMeters, repeatedResult.finalState.positionMeters);
    EXPECT_EQ(result.finalState.velocityMetersPerSecond,
        repeatedResult.finalState.velocityMetersPerSecond);
    EXPECT_EQ(result.finalState.orientation, repeatedResult.finalState.orientation);
    EXPECT_EQ(result.finalState.angularVelocityRadiansPerSecond,
        repeatedResult.finalState.angularVelocityRadiansPerSecond);
    EXPECT_DOUBLE_EQ(result.finalState.yawRadians, repeatedResult.finalState.yawRadians);
    EXPECT_EQ(result.finalState.mode, repeatedResult.finalState.mode);
    EXPECT_EQ(result.finalCamera.status, repeatedResult.finalCamera.status);
    EXPECT_EQ(result.finalCamera.frameCount, repeatedResult.finalCamera.frameCount);
    EXPECT_EQ(result.finalCamera.positionMeters, repeatedResult.finalCamera.positionMeters);
    EXPECT_EQ(result.finalCamera.orientation, repeatedResult.finalCamera.orientation);
    EXPECT_EQ(result.eventLog, repeatedResult.eventLog);
    EXPECT_EQ(result.issues, repeatedResult.issues);
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
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    EXPECT_NEAR(result.drones[1].finalState.yawRadians, 1.2, 1.0e-12);
    EXPECT_DOUBLE_EQ(result.drones[0].finalState.positionMeters.z, 1.5);
    EXPECT_DOUBLE_EQ(result.drones[1].finalState.positionMeters.x, 12.0);
    EXPECT_EQ(result.finalState.positionMeters, result.drones[0].finalState.positionMeters);
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

void Verify_ScenarioRunner_AssignsDroneRolesAndJudgesMissionDeadlines()
{
    constexpr std::string_view missionScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":180,
     "drones":[
       {"name":"lead","initial_position_meters":[0,0,0]},
       {"name":"wing","initial_position_meters":[10,0,0]}],
     "commands":[
       {"tick":0,"drone":"lead","type":"takeoff"},
       {"tick":0,"drone":"wing","type":"takeoff"}],
     "mission":{"name":"two-point-survey","tasks":[
       {"name":"west","role":"west survey","drone":"lead",
        "target_position_meters":[2,0,1.5],"start_tick":90,
        "deadline_tick":150,"tolerance_meters":0.05},
       {"name":"east","role":"east survey","drone":"wing",
        "target_position_meters":[8,0,1.5],"start_tick":90,
        "deadline_tick":150,"tolerance_meters":0.05}]},
     "expected":{"tolerance_meters":0.05,"drones":[
       {"name":"lead","mode":"hovering","position_meters":[2,0,1.5]},
       {"name":"wing","mode":"hovering","position_meters":[8,0,1.5]}]}}
    )json";

    const ScenarioResult success = RunScenario(missionScenario);
    ASSERT_TRUE(success.passed) << (success.issues.empty() ? "" : success.issues.front());
    ASSERT_TRUE(success.mission.has_value());
    EXPECT_TRUE(success.mission->passed);
    ASSERT_EQ(success.mission->tasks.size(), 2);
    EXPECT_TRUE(success.mission->tasks[0].completed);
    EXPECT_TRUE(success.mission->tasks[1].completed);
    ASSERT_TRUE(success.mission->tasks[0].completedTick.has_value());
    ASSERT_TRUE(success.mission->tasks[1].completedTick.has_value());
    EXPECT_LE(*success.mission->tasks[0].completedTick, 150);
    EXPECT_LE(*success.mission->tasks[1].completedTick, 150);

    constexpr std::string_view missionMissesDeadline = R"json(
    {"version":1,"dynamics":"kinematic","ticks":92,
     "drones":[
       {"name":"lead","initial_position_meters":[0,0,0]},
       {"name":"wing","initial_position_meters":[4,0,0]}],
     "commands":[
       {"tick":0,"drone":"lead","type":"takeoff"},
       {"tick":0,"drone":"wing","type":"takeoff"}],
     "mission":{"name":"late-arrival","tasks":[
       {"name":"west","role":"west survey","drone":"lead",
        "target_position_meters":[100,0,1.5],"start_tick":90,
        "deadline_tick":91,"tolerance_meters":0.05},
       {"name":"east","role":"east survey","drone":"wing",
        "target_position_meters":[100,0,1.5],"start_tick":90,
        "deadline_tick":91,"tolerance_meters":0.05}]},
     "expected":{"tolerance_meters":1000,"drones":[
       {"name":"lead","mode":"following_waypoint","position_meters":[0,0,1.5]},
       {"name":"wing","mode":"following_waypoint","position_meters":[4,0,1.5]}]}}
    )json";

    const ScenarioResult missedDeadline = RunScenario(missionMissesDeadline);
    EXPECT_FALSE(missedDeadline.passed);
    ASSERT_TRUE(missedDeadline.mission.has_value());
    EXPECT_FALSE(missedDeadline.mission->passed);
    EXPECT_TRUE(missedDeadline.mission->tasks[0].failed);
    EXPECT_TRUE(missedDeadline.mission->tasks[1].failed);
    EXPECT_NE(missedDeadline.issues.front().find("기한"), std::string::npos);
}

void Verify_ScenarioRunner_DefaultsToAirSimForWindyFlightThroughTheHeadlessPath()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,
     "wind_velocity_meters_per_second":[2,0,0],"ticks":600,
     "commands":[{"tick":0,"type":"takeoff"},{"tick":240,"type":"land"}],
       "expected":{"mode":"grounded","position_meters":[0.0,0.0,0.24],
       "tolerance_meters":0.1}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    const ScenarioResult repeatedResult = RunScenario(scenario);
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    EXPECT_EQ(result.tickCount, 600);
    EXPECT_EQ(result.tickCount, repeatedResult.tickCount);
    EXPECT_EQ(result.finalState.mode, FlightMode::Grounded);
    EXPECT_LE(glm::length(result.finalState.positionMeters
        - repeatedResult.finalState.positionMeters), 1.0e-6);
    EXPECT_EQ(result.finalState.mode, repeatedResult.finalState.mode);
    EXPECT_NEAR(result.finalState.positionMeters.z,
        QuadrotorFlightController::groundedModelOriginHeightMeters, 0.02);
    EXPECT_GT(result.finalState.positionMeters.x, 0.01);
    EXPECT_EQ(result.finalCamera.status, repeatedResult.finalCamera.status);
    EXPECT_EQ(result.finalCamera.frameCount, repeatedResult.finalCamera.frameCount);
}

void Verify_ScenarioRunner_ReportsCommandFailuresAndRejectsInvalidScenarios()
{
    constexpr std::string_view invalidFlight = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,
     "commands":[{"tick":0,"type":"set_velocity","velocity_meters_per_second":[1,0,0]}],
     "expected":{"mode":"grounded","position_meters":[0,0,0]}}
    )json";
    const ScenarioResult result = RunScenario(invalidFlight);
    EXPECT_FALSE(result.passed);
    ASSERT_EQ(result.issues.size(), 1);
    EXPECT_NE(result.issues.front().find("tick 0"), std::string::npos);
    EXPECT_THROW(static_cast<void>(RunScenario(R"json({"version":2})json")), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(RunScenario("not json")), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(RunScenario(R"json({"version":1,"dynamics":"kinematic","ticks":1,
        "commands":[],"expected":{"mode":7,"position_meters":[0,0,0]}})json")),
        std::invalid_argument);
}

void Verify_ScenarioRunner_ReplaysDelayLossDuplicateAndCameraRecovery()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":180,
     "commands":[
       {"tick":0,"type":"takeoff","duplicate":true},
       {"tick":90,"type":"set_waypoint","position_meters":[2,0,1.5],"delay_ticks":10},
       {"tick":170,"type":"land","drop":true}],
     "camera_events":[{"tick":30,"action":"fault"},{"tick":60,"action":"recover"}],
     "expected":{"mode":"hovering","position_meters":[2,0,1.5],
       "camera_status":"active","camera_frame_count":150}}
    )json";
    const ScenarioResult result = RunScenario(scenario);
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    EXPECT_EQ(result.finalCamera.status, CameraStatus::Active);
    EXPECT_EQ(result.finalCamera.frameCount, 150);
    ASSERT_EQ(result.eventLog.size(), 7);
    EXPECT_NE(result.eventLog[1].find("중복"), std::string::npos);
    EXPECT_NE(result.eventLog[2].find("fault"), std::string::npos);
    EXPECT_NE(result.eventLog[4].find("늦게 전달"), std::string::npos);
}

TEST(ScenarioRunnerTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_ReplaysFlightCommandsAndChecksFinalState());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_RunsNamedDronesWithIndependentCommandsAndDeliveryOrder());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_FollowerHoldsItsLastTargetDuringLinkOutageAndResumesAfterward());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_AssignsDroneRolesAndJudgesMissionDeadlines());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_DefaultsToAirSimForWindyFlightThroughTheHeadlessPath());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_ReportsCommandFailuresAndRejectsInvalidScenarios());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_ReplaysDelayLossDuplicateAndCameraRecovery());
}

}
