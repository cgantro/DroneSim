#include "simulation_core/ScenarioRunner.h"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string_view>

namespace aerosimlab::simulation
{
namespace
{
constexpr std::string_view completeFlightScenario = R"json(
{
  "version": 1,
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
    "tolerance_meters": 0.000001
  }
})json";
}

TEST(ScenarioRunner, ReplaysFlightCommandsAndChecksFinalState)
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
    EXPECT_EQ(result.finalState.positionMeters, repeatedResult.finalState.positionMeters);
    EXPECT_EQ(result.finalState.mode, repeatedResult.finalState.mode);
    EXPECT_EQ(result.issues, repeatedResult.issues);
}

TEST(ScenarioRunner, ReportsCommandFailuresAndRejectsInvalidScenarios)
{
    constexpr std::string_view invalidFlight = R"json(
    {"version":1,"ticks":1,
     "commands":[{"tick":0,"type":"set_velocity","velocity_meters_per_second":[1,0,0]}],
     "expected":{"mode":"grounded","position_meters":[0,0,0]}}
    )json";
    const ScenarioResult result = RunScenario(invalidFlight);
    EXPECT_FALSE(result.passed);
    ASSERT_EQ(result.issues.size(), 1);
    EXPECT_NE(result.issues.front().find("tick 0"), std::string::npos);
    EXPECT_THROW(static_cast<void>(RunScenario(R"json({"version":2})json")), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(RunScenario(R"json({"version":1,"ticks":3,"commands":[
        {"tick":2,"type":"stop"},{"tick":1,"type":"stop"}],
        "expected":{"mode":"grounded","position_meters":[0,0,0]}})json")),
        std::invalid_argument);
    EXPECT_THROW(static_cast<void>(RunScenario("not json")), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(RunScenario(R"json({"version":1,"ticks":1,
        "commands":[],"expected":{"mode":7,"position_meters":[0,0,0]}})json")),
        std::invalid_argument);
}

TEST(ScenarioRunner, ReplaysDelayLossDuplicateAndCameraRecovery)
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"ticks":180,
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
}
