#include "simulation_core/ScenarioRunner.h"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
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

bool ReplaceAll(
    std::string& text, std::string_view oldValue, std::string_view newValue)
{
    std::size_t position = 0;
    bool replaced = false;
    while ((position = text.find(oldValue, position)) != std::string::npos)
    {
        text.replace(position, oldValue.size(), newValue);
        position += newValue.size();
        replaced = true;
    }
    return replaced;
}

void Verify_ScenarioRunner_UsesAndReportsPhysicalModelOverrides()
{
    std::string customModelScenario =
        ReadScenarioFixture("scenarios/flight/physical-model-override.json");
    const ScenarioResult result = RunScenario(customModelScenario);
    EXPECT_TRUE(result.trajectory.empty());
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
    EXPECT_DOUBLE_EQ(result.physicalModel->motorTimeConstantDownSeconds, 0.025);
    EXPECT_DOUBLE_EQ(result.physicalModel->maximumMotorSpeedRadiansPerSecond, 900.0);

    const std::string customMass = "\"mass_kg\": 2.5";
    const std::string baselineMass = "\"mass_kg\": 2.0643076923076924";
    customModelScenario.replace(customModelScenario.find(customMass),
        customMass.size(), baselineMass);
    const ScenarioResult baselineResult = RunScenario(customModelScenario);
    ASSERT_TRUE(baselineResult.passed)
        << (baselineResult.issues.empty() ? "" : baselineResult.issues.front());
    const auto meanMotorSpeed = [](const DroneState& state)
    {
        double sum = 0.0;
        for (const double speed : state.motorSpeedsRadiansPerSecond)
            sum += speed;
        return sum / static_cast<double>(state.motorSpeedsRadiansPerSecond.size());
    };
    EXPECT_GT(meanMotorSpeed(result.finalState), meanMotorSpeed(baselineResult.finalState));

    constexpr std::string_view kinematicOverride = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,"commands":[],
     "quadrotor_model":{"mass_kg":2.5}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(kinematicOverride)),
        std::invalid_argument);
}

void Verify_ScenarioRunner_RecordsBoundedTimeAlignedTrajectories()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":5,
     "drones":[
       {"name":"east","initial_position_meters":[1,0,0]},
       {"name":"north","initial_position_meters":[0,2,0]}],
     "commands":[],"record_trajectory":{"sample_every_ticks":2},
     "expected":{"drones":[
       {"name":"east","mode":"grounded","position_meters":[1,0,0]},
       {"name":"north","mode":"grounded","position_meters":[0,2,0]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    ASSERT_TRUE(result.passed);
    ASSERT_EQ(result.trajectory.size(), 4);
    EXPECT_EQ(result.trajectory[0].tick, 0U);
    EXPECT_DOUBLE_EQ(result.trajectory[0].timeSeconds, 0.0);
    EXPECT_EQ(result.trajectory[1].tick, 2U);
    EXPECT_NEAR(result.trajectory[1].timeSeconds, 2.0 / 60.0, 1.0e-12);
    EXPECT_EQ(result.trajectory[2].tick, 4U);
    EXPECT_EQ(result.trajectory[3].tick, 5U);
    ASSERT_EQ(result.trajectory[3].drones.size(), 2);
    EXPECT_EQ(result.trajectory[3].drones[0].first, "east");
    EXPECT_EQ(result.trajectory[3].drones[1].first, "north");
    EXPECT_EQ(result.trajectory[3].drones[0].second.positionMeters,
        (glm::dvec3{1.0, 0.0, 0.0}));

    constexpr std::string_view oversizedTrace = R"json(
    {"version":1,"dynamics":"kinematic","ticks":100000,
     "commands":[],"record_trajectory":{"sample_every_ticks":1}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(oversizedTrace)), std::invalid_argument);
}

void Verify_ScenarioRunner_ReplaysSixDofStateAndRotorThrust()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":2,
     "drones":[{"name":"x500","initial_position_meters":[1,2,3],
       "initial_velocity_meters_per_second":[0.2,-0.1,0.05],
       "initial_orientation_quaternion_wxyz":[0.9238795325,0,0,0.3826834324],
       "initial_angular_velocity_radians_per_second":[0.01,0.02,0.03],
       "initial_motor_speeds_radians_per_second":[120,240,360,480],
       "initial_mode":"flying"}],
     "commands":[{"tick":0,"type":"set_rotor_thrusts",
       "normalized_thrust":[0.5,0.5,0.5,0.5]}],
     "record_trajectory":{"sample_every_ticks":1},
     "expected":{"drones":[{"name":"x500","mode":"flying",
       "position_meters":[1,2,3],"tolerance_meters":1000}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    ASSERT_TRUE(result.passed)
        << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_EQ(result.trajectory.size(), 3);
    const DroneState& initial = result.trajectory.front().drones.front().second;
    EXPECT_EQ(initial.positionMeters, (glm::dvec3{1, 2, 3}));
    EXPECT_NEAR(initial.velocityMetersPerSecond.x, 0.2, 1.0e-6);
    EXPECT_NEAR(initial.velocityMetersPerSecond.y, -0.1, 1.0e-6);
    EXPECT_NEAR(initial.velocityMetersPerSecond.z, 0.05, 1.0e-6);
    EXPECT_EQ(initial.mode, FlightMode::Flying);
    EXPECT_NEAR(initial.orientation.z, 0.3826834324, 1.0e-6);
    EXPECT_NEAR(initial.angularVelocityRadiansPerSecond.x, 0.01, 1.0e-6);
    EXPECT_NEAR(initial.angularVelocityRadiansPerSecond.y, 0.02, 1.0e-6);
    EXPECT_NEAR(initial.angularVelocityRadiansPerSecond.z, 0.03, 1.0e-6);
    EXPECT_EQ(initial.motorSpeedsRadiansPerSecond,
        (std::array<double, 4>{120.0, 240.0, 360.0, 480.0}));
    EXPECT_GT(result.trajectory[2].drones.front().second
                  .motorSpeedsRadiansPerSecond[0],
        0.0);

    constexpr std::string_view kinematicRotorInput = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,
     "commands":[{"tick":0,"type":"set_rotor_thrusts",
       "normalized_thrust":[0,0,0,0]}],
     "expected":{"mode":"grounded","position_meters":[0,0,0]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(kinematicRotorInput)),
        std::invalid_argument);

    constexpr std::string_view invalidInitialMotorSpeeds = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":1,
     "drones":[{"name":"x500","initial_position_meters":[0,0,2],
       "initial_motor_speeds_radians_per_second":[100,200,300]}],
     "commands":[],"expected":{"mode":"grounded","position_meters":[0,0,2]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(invalidInitialMotorSpeeds)),
        std::invalid_argument);
}

void Verify_ScenarioRunner_ComparesPx4ReferenceWithEstimatedAndZeroRotorStarts()
{
    using Json = nlohmann::json;
    const std::string fixtureText = ReadScenarioFixture(
        "tests/data/px4_x500_position_replay_12_05.json");
    const Json fixture = Json::parse(fixtureText);
    const Json& reference = fixture.at("reference_samples");
    const auto calculateRmse = [&](const ScenarioResult& result)
    {
        const glm::dvec3 initialPosition =
            result.trajectory.front().drones.front().second.positionMeters;
        std::array<double, 4> squaredErrorSums{};
        for (std::size_t index = 0; index < reference.size(); ++index)
        {
            const DroneState& state = result.trajectory[index].drones.front().second;
            const Json& sample = reference[index];
            const auto readVector = [&](const char* key)
            {
                const Json& values = sample.at(key);
                return glm::dvec3{
                    values[0].get<double>(), values[1].get<double>(),
                    values[2].get<double>()};
            };
            const glm::dvec3 positionError = state.positionMeters - initialPosition
                - readVector("position_delta_enu");
            const glm::dvec3 velocityError = state.velocityMetersPerSecond
                - readVector("velocity_enu");
            const glm::dvec3 angularVelocityError =
                state.angularVelocityRadiansPerSecond
                - readVector("angular_velocity_flu");
            squaredErrorSums[0] += glm::dot(positionError, positionError);
            squaredErrorSums[1] += glm::dot(velocityError, velocityError);
            squaredErrorSums[3] += glm::dot(angularVelocityError, angularVelocityError);

            const Json& values = sample.at("orientation_wxyz");
            const glm::dquat referenceOrientation{
                values[0].get<double>(), values[1].get<double>(),
                values[2].get<double>(), values[3].get<double>()};
            const double cosineHalfAngle = std::clamp(
                std::abs(glm::dot(glm::normalize(state.orientation), referenceOrientation)),
                0.0, 1.0);
            const double attitudeErrorRadians = 2.0 * std::acos(cosineHalfAngle);
            squaredErrorSums[2] += attitudeErrorRadians * attitudeErrorRadians;
        }

        for (double& sum : squaredErrorSums)
            sum = std::sqrt(sum / static_cast<double>(reference.size()));
        squaredErrorSums[2] *= 180.0 / std::numbers::pi;
        return squaredErrorSums;
    };

    const ScenarioResult estimatedStartResult = RunScenario(fixtureText);
    ASSERT_TRUE(estimatedStartResult.passed);
    ASSERT_EQ(estimatedStartResult.trajectory.size(), reference.size());
    const std::array<double, 4> estimatedStartRmse = calculateRmse(estimatedStartResult);

    Json zeroStartScenario = fixture;
    zeroStartScenario["drones"][0]["initial_motor_speeds_radians_per_second"] =
        {0.0, 0.0, 0.0, 0.0};
    const ScenarioResult zeroStartResult = RunScenario(zeroStartScenario.dump());
    ASSERT_TRUE(zeroStartResult.passed);
    ASSERT_EQ(zeroStartResult.trajectory.size(), reference.size());
    const std::array<double, 4> zeroStartRmse = calculateRmse(zeroStartResult);

    // 초기 상태와 명령은 그대로 두고 추력 계수만 10% 높여 민감도를 비교한다.
    Json increasedThrustScenario = fixture;
    increasedThrustScenario["quadrotor_model"]
        ["thrust_coefficient_newtons_per_speed_squared"] = 1.1
            * fixture.at("quadrotor_model")
                .at("thrust_coefficient_newtons_per_speed_squared").get<double>();
    const ScenarioResult increasedThrustResult =
        RunScenario(increasedThrustScenario.dump());
    ASSERT_TRUE(increasedThrustResult.passed);
    ASSERT_EQ(increasedThrustResult.trajectory.size(), reference.size());
    const std::array<double, 4> increasedThrustRmse =
        calculateRmse(increasedThrustResult);

    EXPECT_NEAR(estimatedStartRmse[0], 0.4779, 0.02);
    EXPECT_NEAR(estimatedStartRmse[1], 0.3707, 0.02);
    EXPECT_NEAR(estimatedStartRmse[2], 52.29, 0.5);
    EXPECT_NEAR(estimatedStartRmse[3], 0.8800, 0.02);
    EXPECT_NEAR(zeroStartRmse[0], 0.4113, 0.02);
    EXPECT_NEAR(zeroStartRmse[1], 0.2666, 0.02);
    EXPECT_NEAR(zeroStartRmse[2], 27.70, 0.5);
    EXPECT_NEAR(zeroStartRmse[3], 0.5568, 0.02);
    EXPECT_NEAR(increasedThrustRmse[0], 0.9727, 0.02);
    EXPECT_NEAR(increasedThrustRmse[1], 0.9695, 0.02);
    EXPECT_NEAR(increasedThrustRmse[2], 99.85, 0.5);
    EXPECT_NEAR(increasedThrustRmse[3], 1.4210, 0.02);
}

void Verify_ScenarioRunner_ChangesWaypointResponseWhenControllerGainsChange()
{
    constexpr std::string_view scenarioTemplate = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":300,
     "commands":[
       {"tick":0,"type":"takeoff"},
       {"tick":240,"type":"set_waypoint","position_meters":[2,0,1.5]}],
     "controller_gains":{"position":POSITION_GAIN,"velocity":2.5,
       "attitude":25,"angular_rate":8.5},
     "expected":{"mode":"hovering","position_meters":[0,0,1.5],
       "tolerance_meters":10}}
    )json";
    std::string lowGainScenario{scenarioTemplate};
    std::string highGainScenario{scenarioTemplate};
    std::string defaultGainScenario{scenarioTemplate};
    const std::string gainPlaceholder = "POSITION_GAIN";
    ASSERT_NE(lowGainScenario.find(gainPlaceholder), std::string::npos);
    lowGainScenario.replace(lowGainScenario.find(gainPlaceholder),
        gainPlaceholder.size(), "0.5");
    highGainScenario.replace(highGainScenario.find(gainPlaceholder),
        gainPlaceholder.size(), "3.0");
    defaultGainScenario.replace(defaultGainScenario.find(gainPlaceholder),
        gainPlaceholder.size(), "3.0");
    defaultGainScenario.replace(defaultGainScenario.find("25"), 2, "20");
    defaultGainScenario.replace(defaultGainScenario.find("8.5"), 3, "7.0");

    const ScenarioResult lowGain = RunScenario(lowGainScenario);
    const ScenarioResult highGain = RunScenario(highGainScenario);
    const ScenarioResult explicitDefault = RunScenario(defaultGainScenario);
    const std::string customGainBlock =
        "     \"controller_gains\":{\"position\":3.0,\"velocity\":2.5,\n"
        "       \"attitude\":20,\"angular_rate\":7.0},\n";
    ASSERT_NE(defaultGainScenario.find(customGainBlock), std::string::npos);
    defaultGainScenario.erase(defaultGainScenario.find(customGainBlock), customGainBlock.size());
    const ScenarioResult implicitDefault = RunScenario(defaultGainScenario);
    ASSERT_TRUE(lowGain.controllerSettings.has_value());
    ASSERT_TRUE(highGain.controllerSettings.has_value());
    EXPECT_TRUE(lowGain.controllerSettings->customized);
    EXPECT_DOUBLE_EQ(lowGain.controllerSettings->positionGain, 0.5);
    EXPECT_DOUBLE_EQ(lowGain.controllerSettings->velocityGain, 2.5);
    EXPECT_DOUBLE_EQ(lowGain.controllerSettings->attitudeGain, 25.0);
    EXPECT_DOUBLE_EQ(lowGain.controllerSettings->angularRateGain, 8.5);
    EXPECT_DOUBLE_EQ(highGain.controllerSettings->positionGain, 3.0);
    EXPECT_GT(highGain.finalState.positionMeters.x, lowGain.finalState.positionMeters.x);
    ASSERT_TRUE(implicitDefault.controllerSettings.has_value());
    EXPECT_FALSE(implicitDefault.controllerSettings->customized);
    EXPECT_EQ(explicitDefault.finalState.positionMeters,
        implicitDefault.finalState.positionMeters);

    constexpr std::string_view unsupportedMode = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,"commands":[],
     "controller_gains":{"position":2.0}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(unsupportedMode)),
        std::invalid_argument);

    constexpr std::string_view invalidGain = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":1,"commands":[],
     "controller_gains":{"position":0}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(invalidGain)),
        std::invalid_argument);
}

void Verify_ScenarioRunner_RecordsWindAndChangesTheFlightPath()
{
    constexpr std::string_view calmScenario = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":600,
     "commands":[{"tick":0,"type":"takeoff"},
       {"tick":240,"type":"set_waypoint","position_meters":[0,0,1.5]}],
     "expected":{"mode":"hovering","position_meters":[0,0,1.5],
       "tolerance_meters":10}}
    )json";
    constexpr std::string_view windScenario = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":600,
     "wind_velocity_meters_per_second":[2,0,0],
     "commands":[{"tick":0,"type":"takeoff"},
       {"tick":240,"type":"set_waypoint","position_meters":[0,0,1.5]}],
     "expected":{"mode":"hovering","position_meters":[0,0,1.5],
       "tolerance_meters":10}}
    )json";

    const ScenarioResult calm = RunScenario(calmScenario);
    const ScenarioResult wind = RunScenario(windScenario);
    ASSERT_TRUE(calm.passed);
    ASSERT_TRUE(wind.passed);
    ASSERT_TRUE(calm.windVelocityMetersPerSecond == std::nullopt);
    ASSERT_EQ(wind.windVelocityMetersPerSecond, (glm::dvec3{2.0, 0.0, 0.0}));
    EXPECT_GT(std::abs(wind.finalState.positionMeters.x
        - calm.finalState.positionMeters.x), 0.01);
}

void Verify_ScenarioRunner_ReassignsFourDroneSurveyAfterHeartbeatLoss()
{
    const ScenarioResult result = RunScenario(
        ReadScenarioFixture("scenarios/multi-drone/four-drone-survey-outage.json"));
    EXPECT_TRUE(result.passed)
        << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_TRUE(result.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*result.minimumDroneSeparationMeters, 1.0);
    EXPECT_EQ(result.outcome, ScenarioResult::Outcome::Passed);
    ASSERT_TRUE(result.mission.has_value());
    ASSERT_TRUE(result.mission->passed);
    EXPECT_EQ(result.mission->outcome, ScenarioResult::Outcome::Passed);
    EXPECT_EQ(result.mission->completedTaskCount, 4U);
    ASSERT_TRUE(result.mission->startedTick.has_value());
    EXPECT_EQ(*result.mission->startedTick, 90U);

    const auto disconnectedDrone = std::find_if(result.drones.begin(), result.drones.end(),
        [](const ScenarioResult::Drone& drone) { return drone.name == "scout-2"; });
    ASSERT_NE(disconnectedDrone, result.drones.end());
    EXPECT_EQ(disconnectedDrone->communicationStatus,
        ScenarioResult::Drone::CommunicationStatus::Unknown);
    EXPECT_TRUE(std::any_of(result.eventLog.begin(), result.eventLog.end(),
        [](const std::string& event)
        {
            return event.find("현재 위치를 유지합니다") != std::string::npos;
        }));

    const auto reassignedTask = std::find_if(result.mission->tasks.begin(),
        result.mission->tasks.end(),
        [](const ScenarioResult::MissionTask& task) { return task.name == "sector-2"; });
    ASSERT_NE(reassignedTask, result.mission->tasks.end());
    EXPECT_TRUE(reassignedTask->completed);
    EXPECT_FALSE(reassignedTask->failed);
    EXPECT_EQ(reassignedTask->initialDroneName, "scout-2");
    EXPECT_EQ(reassignedTask->droneName, "scout-3");
    ASSERT_EQ(reassignedTask->assignmentHistory.size(), 2U);
    EXPECT_EQ(reassignedTask->assignmentHistory[0], "scout-2");
    EXPECT_EQ(reassignedTask->assignmentHistory[1], "scout-3");

#if !defined(__EMSCRIPTEN__)
    std::string physicalScenario =
        ReadScenarioFixture("scenarios/multi-drone/four-drone-survey-outage.json");
    constexpr std::string_view kinematicMode = "\"dynamics\": \"kinematic\"";
    const std::size_t dynamicsPosition = physicalScenario.find(kinematicMode);
    ASSERT_NE(dynamicsPosition, std::string::npos);
    physicalScenario.replace(dynamicsPosition, kinematicMode.size(),
        "\"dynamics\": \"rigid_body\"");
    ASSERT_TRUE(ReplaceAll(physicalScenario,
        "\"ticks\": 320", "\"ticks\": 1800"));
    ASSERT_TRUE(ReplaceAll(physicalScenario,
        "\"end_tick\": 320", "\"end_tick\": 1800"));
    ASSERT_TRUE(ReplaceAll(physicalScenario,
        "\"deadline_tick\": 310", "\"deadline_tick\": 1790"));
    ASSERT_TRUE(ReplaceAll(physicalScenario,
        "\"tolerance_meters\": 0.1", "\"tolerance_meters\": 0.5"));
    ASSERT_TRUE(ReplaceAll(physicalScenario,
        "\"tolerance_meters\": 0.01", "\"tolerance_meters\": 1000"));
    const ScenarioResult physicalResult = RunScenario(physicalScenario);
    EXPECT_EQ(physicalResult.outcome, ScenarioResult::Outcome::Passed)
        << (physicalResult.issues.empty() ? "" : physicalResult.issues.front())
        << " minimum=" << physicalResult.minimumDroneSeparationMeters.value_or(-1.0)
        << " predicted=" << physicalResult.minimumPredictedDroneSeparationMeters.value_or(-1.0)
        << " avoidance_events=" << physicalResult.collisionAvoidanceEvents;
    ASSERT_TRUE(physicalResult.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*physicalResult.minimumDroneSeparationMeters, 1.0);
    EXPECT_GT(physicalResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(physicalResult.mission.has_value());
    EXPECT_EQ(physicalResult.mission->outcome, ScenarioResult::Outcome::Passed);
    ASSERT_TRUE(physicalResult.mission->startedTick.has_value());
    EXPECT_GT(*physicalResult.mission->startedTick, 90U);
    EXPECT_EQ(physicalResult.mission->completedTaskCount, 4U);
    EXPECT_EQ(physicalResult.mission->tasks[1].assignmentHistory,
        (std::vector<std::string>{"scout-2", "scout-3"}));
#endif
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
    EXPECT_EQ(communicationFaults.outcome, ScenarioResult::Outcome::Failed);
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

void Verify_ScenarioRunner_LeavesUnreachableWorkInconclusive()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":145,
     "drones":[{"name":"scout","initial_position_meters":[0,0,0]},
       {"name":"relay","initial_position_meters":[5,0,0]}],
     "commands":[{"tick":0,"drone":"scout","type":"takeoff"},
       {"tick":0,"drone":"relay","type":"takeoff"}],
     "communication_outages":[
       {"drone":"scout","start_tick":122,"end_tick":145,"heartbeat_timeout_ticks":2},
       {"drone":"relay","start_tick":122,"end_tick":145,"heartbeat_timeout_ticks":2}],
     "mission":{"name":"unconfirmed-survey","tasks":[
       {"name":"sector-1","role":"survey","drone":"scout",
        "target_position_meters":[100,0,1.5],"start_tick":120,
        "deadline_tick":144,"tolerance_meters":0.1},
       {"name":"sector-2","role":"survey","drone":"relay",
        "target_position_meters":[105,0,1.5],"start_tick":120,
        "deadline_tick":144,"tolerance_meters":0.1}]},
     "expected":{"tolerance_meters":1000,"drones":[
       {"name":"scout","mode":"following_waypoint","position_meters":[0,0,1.5]},
       {"name":"relay","mode":"following_waypoint","position_meters":[5,0,1.5]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    ASSERT_TRUE(result.mission.has_value());
    EXPECT_EQ(result.outcome, ScenarioResult::Outcome::Inconclusive);
    EXPECT_FALSE(result.passed);
    EXPECT_EQ(result.mission->outcome, ScenarioResult::Outcome::Inconclusive);
    EXPECT_FALSE(result.mission->passed);
    ASSERT_EQ(result.mission->tasks.size(), 2U);
    for (const ScenarioResult::MissionTask& task : result.mission->tasks)
    {
        EXPECT_FALSE(task.completed);
        EXPECT_FALSE(task.failed);
        EXPECT_TRUE(task.droneName.empty());
    }
    ASSERT_EQ(result.drones.size(), 2U);
    EXPECT_EQ(result.drones.front().communicationStatus,
        ScenarioResult::Drone::CommunicationStatus::Unknown);
    EXPECT_EQ(ToString(result.outcome), "inconclusive");

    constexpr std::string_view missedDeadlineScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":123,
     "drones":[{"name":"scout","initial_position_meters":[0,0,0]},
       {"name":"relay","initial_position_meters":[5,0,0]}],
     "commands":[{"tick":0,"drone":"scout","type":"takeoff"},
       {"tick":0,"drone":"relay","type":"takeoff"}],
     "mission":{"name":"missed-survey","tasks":[
       {"name":"sector-1","role":"survey","drone":"scout",
        "target_position_meters":[100,0,1.5],"start_tick":120,
        "deadline_tick":121,"tolerance_meters":0.1},
       {"name":"sector-2","role":"survey","drone":"relay",
        "target_position_meters":[105,0,1.5],"start_tick":120,
        "deadline_tick":121,"tolerance_meters":0.1}]},
     "expected":{"tolerance_meters":1000,"drones":[
       {"name":"scout","mode":"following_waypoint","position_meters":[0,0,1.5]},
       {"name":"relay","mode":"following_waypoint","position_meters":[5,0,1.5]}]}}
    )json";

    const ScenarioResult failedResult = RunScenario(missedDeadlineScenario);
    ASSERT_TRUE(failedResult.mission.has_value());
    EXPECT_EQ(failedResult.outcome, ScenarioResult::Outcome::Failed);
    EXPECT_EQ(failedResult.mission->outcome, ScenarioResult::Outcome::Failed);
    EXPECT_FALSE(failedResult.passed);
    EXPECT_FALSE(failedResult.issues.empty());
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
    ASSERT_TRUE(result.mission->startedTick.has_value());
    EXPECT_EQ(*result.mission->startedTick, 90U);
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

#if !defined(__EMSCRIPTEN__)
    std::string physicalAvoidanceScenario{collisionAvoidanceScenario};
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario, "\"dynamics\":\"kinematic\"",
        "\"dynamics\":\"rigid_body\""));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"ticks\":600", "\"ticks\":1200"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"deadline_tick\":580", "\"deadline_tick\":1190"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"safety_distance_meters\":3",
        "\"safety_distance_meters\":8"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"prediction_horizon_seconds\":2",
        "\"prediction_horizon_seconds\":6"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"tolerance_meters\":0.2",
        "\"tolerance_meters\":0.5"));
    // 임무 목표 반경에 들어와도 강체 비행 상태는 Hovering으로 바뀌지 않을 수 있다.
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"name\":\"east\",\"mode\":\"hovering\"",
        "\"name\":\"east\",\"mode\":\"following_waypoint\""));

    const ScenarioResult physicalAvoidanceResult = RunScenario(physicalAvoidanceScenario);
    ASSERT_TRUE(physicalAvoidanceResult.passed)
        << (physicalAvoidanceResult.issues.empty()
            ? "" : physicalAvoidanceResult.issues.front())
        << " minimum="
        << physicalAvoidanceResult.minimumDroneSeparationMeters.value_or(-1.0)
        << " predicted="
        << physicalAvoidanceResult.minimumPredictedDroneSeparationMeters.value_or(-1.0)
        << " avoidance_events=" << physicalAvoidanceResult.collisionAvoidanceEvents
        << " west_mode=" << (physicalAvoidanceResult.drones.size() > 0U
            ? static_cast<int>(physicalAvoidanceResult.drones[0].finalState.mode) : -1)
        << " east_mode=" << (physicalAvoidanceResult.drones.size() > 1U
            ? static_cast<int>(physicalAvoidanceResult.drones[1].finalState.mode) : -1)
        << " mission=" << (physicalAvoidanceResult.mission.has_value()
            ? static_cast<int>(physicalAvoidanceResult.mission->outcome) : -1);
    ASSERT_TRUE(physicalAvoidanceResult.mission.has_value());
    EXPECT_TRUE(physicalAvoidanceResult.mission->passed);
    EXPECT_EQ(physicalAvoidanceResult.mission->completedTaskCount, 2U);
    EXPECT_GT(physicalAvoidanceResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(physicalAvoidanceResult.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*physicalAvoidanceResult.minimumDroneSeparationMeters, 3.0);
#endif

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
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_RecordsBoundedTimeAlignedTrajectories());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ReplaysSixDofStateAndRotorThrust());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ComparesPx4ReferenceWithEstimatedAndZeroRotorStarts());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_UsesAndReportsPhysicalModelOverrides());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ChangesWaypointResponseWhenControllerGainsChange());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_RecordsWindAndChangesTheFlightPath());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ReassignsFourDroneSurveyAfterHeartbeatLoss());
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
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_LeavesUnreachableWorkInconclusive());
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
