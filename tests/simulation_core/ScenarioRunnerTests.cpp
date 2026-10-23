#include "simulation_core/ScenarioRunner.h"
#include <gtest/gtest.h>

#include <string>
#include <string_view>

namespace aerosimlab::simulation
{
namespace
{
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
         {"name":"first","mode":"grounded","position_meters":[0,0,0]},
         {"name":"second","mode":"grounded","position_meters":[3,0,0]},
         {"name":"third","mode":"grounded","position_meters":[0,2,0]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_TRUE(result.minimumDroneSeparationMeters.has_value());
    EXPECT_DOUBLE_EQ(*result.minimumDroneSeparationMeters, 2.0);
}

TEST(ScenarioRunnerTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_RunsNamedDronesWithIndependentCommandsAndDeliveryOrder());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_FollowerHoldsItsLastTargetDuringLinkOutageAndResumesAfterward());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_FindsClosestPairAcrossThreeDrones());
}

}
