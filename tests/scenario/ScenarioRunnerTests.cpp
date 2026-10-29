#include "scenario/ScenarioRunner.h"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace aerosimlab::scenario
{
using namespace aerosimlab::simulation;

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
    ASSERT_TRUE(result.IsPassed())
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
    ASSERT_TRUE(baselineResult.IsPassed())
        << (baselineResult.issues.empty() ? "" : baselineResult.issues.front());
    const auto meanMotorSpeed = [](const DroneState& state)
    {
        double sum = 0.0;
        for (const double speed : state.motorSpeedsRadiansPerSecond)
            sum += speed;
        return sum / static_cast<double>(state.motorSpeedsRadiansPerSecond.size());
    };
    EXPECT_GT(meanMotorSpeed(result.drones.front().finalState),
        meanMotorSpeed(baselineResult.drones.front().finalState));

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
    ASSERT_TRUE(result.IsPassed());
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

void Verify_ScenarioRunnerCompletesTasksAtWaypoints()
{
    constexpr std::string_view source = R"json(
    {"version":1,"dynamics":"kinematic","ticks":150,
     "drones":[
       {"name":"east","initial_position_meters":[0,0,0]},
       {"name":"north","initial_position_meters":[0,4,0]}],
     "commands":[
       {"tick":0,"drone":"east","type":"takeoff"},
       {"tick":0,"drone":"north","type":"takeoff"}],
     "mission":{"name":"two-drone-survey","tasks":[
       {"name":"east-face","drone":"east",
        "waypoints_meters":[[0,0,1.5]],"start_tick":110,
        "deadline_tick":145,"tolerance_meters":0.1},
       {"name":"north-face","drone":"north",
        "waypoints_meters":[[0,4,1.5]],"start_tick":110,
        "deadline_tick":145,"tolerance_meters":0.1}]},
     "expected":{"drones":[
       {"name":"east","mode":"hovering","position_meters":[0,0,1.5]},
       {"name":"north","mode":"hovering","position_meters":[0,4,1.5]}]}}
    )json";

    const ScenarioResult result = RunScenario(source);
    ASSERT_TRUE(result.IsPassed())
        << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_TRUE(result.mission.has_value());
    ASSERT_EQ(result.mission->tasks.size(), 2U);
    EXPECT_TRUE(result.mission->tasks[0].IsCompleted());
    EXPECT_TRUE(result.mission->tasks[1].IsCompleted());
    EXPECT_EQ(result.mission->CompletedTaskCount(), 2U);
    EXPECT_TRUE(result.mission->CompletedTick().has_value());
    EXPECT_EQ(result.drones[0].finalCamera.status, CameraStatus::Active);
    EXPECT_EQ(result.drones[0].finalCamera.model.imageWidthPixels, 1280);

    nlohmann::json duplicateTask = nlohmann::json::parse(source);
    duplicateTask["mission"]["tasks"][1]["name"] = "east-face";
    EXPECT_THROW(static_cast<void>(RunScenario(duplicateTask.dump())),
        std::invalid_argument);

    nlohmann::json invalidCamera = nlohmann::json::parse(source);
    invalidCamera["drones"][0]["camera_model"] = {
        {"image_width_pixels", 160},
        {"image_height_pixels", 120},
        {"vertical_field_of_view_degrees", 180.0}};
    EXPECT_THROW(static_cast<void>(RunScenario(invalidCamera.dump())),
        std::invalid_argument);
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
    ASSERT_TRUE(result.IsPassed())
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
     "expected":{"drones":[{"name":"drone","mode":"grounded",
       "position_meters":[0,0,0]}]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(kinematicRotorInput)),
        std::invalid_argument);

    constexpr std::string_view invalidInitialMotorSpeeds = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":1,
     "drones":[{"name":"x500","initial_position_meters":[0,0,2],
       "initial_motor_speeds_radians_per_second":[100,200,300]}],
     "commands":[],"expected":{"drones":[{"name":"x500","mode":"grounded",
       "position_meters":[0,0,2]}]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(invalidInitialMotorSpeeds)),
        std::invalid_argument);
}

void Verify_ScenarioRunner_RecordsPx4ReferenceErrorsAcrossModelVariants()
{
    using Json = nlohmann::json;
    const std::string fixtureText = ReadScenarioFixture(
        "tests/data/px4_x500_position_replay_12_05.json");
    const Json fixture = Json::parse(fixtureText);
    const Json& reference = fixture.at("reference_samples");
    const auto calculateRmse = [&](const Json& referenceSamples,
                                   const ScenarioResult& result)
    {
        const glm::dvec3 initialPosition =
            result.trajectory.front().drones.front().second.positionMeters;
        std::array<double, 5> squaredErrorSums{};
        for (std::size_t index = 0; index < referenceSamples.size(); ++index)
        {
            const DroneState& state = result.trajectory[index].drones.front().second;
            const Json& sample = referenceSamples[index];
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
            if (sample.contains("motor_speeds_radians_per_second"))
            {
                const Json& motorSpeeds = sample.at("motor_speeds_radians_per_second");
                for (std::size_t rotor = 0;
                     rotor < state.motorSpeedsRadiansPerSecond.size();
                     ++rotor)
                {
                    const double speedError = state.motorSpeedsRadiansPerSecond[rotor]
                        - motorSpeeds[rotor].get<double>();
                    squaredErrorSums[4] += speedError * speedError;
                }
            }

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

        const double sampleCount = static_cast<double>(referenceSamples.size());
        for (std::size_t metric = 0; metric < 4; ++metric)
            squaredErrorSums[metric] = std::sqrt(squaredErrorSums[metric] / sampleCount);
        squaredErrorSums[4] = std::sqrt(
            squaredErrorSums[4] / (sampleCount * 4.0));
        squaredErrorSums[2] *= 180.0 / std::numbers::pi;
        return squaredErrorSums;
    };

    const ScenarioResult estimatedStartResult = RunScenario(fixtureText);
    ASSERT_TRUE(estimatedStartResult.IsPassed());
    ASSERT_EQ(estimatedStartResult.trajectory.size(), reference.size());
    const std::array<double, 5> estimatedStartRmse = calculateRmse(
        reference, estimatedStartResult);

    Json zeroStartScenario = fixture;
    zeroStartScenario["drones"][0]["initial_motor_speeds_radians_per_second"] =
        {0.0, 0.0, 0.0, 0.0};
    const ScenarioResult zeroStartResult = RunScenario(zeroStartScenario.dump());
    ASSERT_TRUE(zeroStartResult.IsPassed());
    ASSERT_EQ(zeroStartResult.trajectory.size(), reference.size());
    const std::array<double, 5> zeroStartRmse = calculateRmse(
        reference, zeroStartResult);

    // 초기 상태와 명령은 그대로 두고 추력 계수만 10% 높여 민감도를 비교한다.
    Json increasedThrustScenario = fixture;
    increasedThrustScenario["quadrotor_model"]
        ["thrust_coefficient_newtons_per_speed_squared"] = 1.1
            * fixture.at("quadrotor_model")
                .at("thrust_coefficient_newtons_per_speed_squared").get<double>();
    const ScenarioResult increasedThrustResult =
        RunScenario(increasedThrustScenario.dump());
    ASSERT_TRUE(increasedThrustResult.IsPassed());
    ASSERT_EQ(increasedThrustResult.trajectory.size(), reference.size());
    const std::array<double, 5> increasedThrustRmse = calculateRmse(
        reference, increasedThrustResult);

    EXPECT_NEAR(estimatedStartRmse[0], 0.7520, 0.02);
    EXPECT_NEAR(estimatedStartRmse[1], 0.8016, 0.02);
    EXPECT_NEAR(estimatedStartRmse[2], 89.98, 0.5);
    EXPECT_NEAR(estimatedStartRmse[3], 1.2986, 0.02);
    EXPECT_NEAR(zeroStartRmse[0], 0.6534, 0.02);
    EXPECT_NEAR(zeroStartRmse[1], 0.5808, 0.02);
    EXPECT_NEAR(zeroStartRmse[2], 71.81, 0.5);
    EXPECT_NEAR(zeroStartRmse[3], 1.2205, 0.02);
    EXPECT_NEAR(increasedThrustRmse[0], 1.6491, 0.02);
    EXPECT_NEAR(increasedThrustRmse[1], 2.6448, 0.02);
    EXPECT_NEAR(increasedThrustRmse[2], 108.95, 0.5);
    EXPECT_NEAR(increasedThrustRmse[3], 2.9164, 0.02);

    const Json recentFlightFixture = Json::parse(ReadScenarioFixture(
        "tests/data/px4_x500_position_replay_2026_10_16_100s.json"));
    const Json& recentReference = recentFlightFixture.at("reference_samples");
    const ScenarioResult recentFlightResult = RunScenario(recentFlightFixture.dump());
    ASSERT_TRUE(recentFlightResult.IsPassed());
    ASSERT_EQ(recentFlightResult.trajectory.size(), recentReference.size());
    const std::array<double, 5> recentFlightRmse = calculateRmse(
        recentReference, recentFlightResult);

    EXPECT_NEAR(recentFlightRmse[0], 0.7140, 0.03);
    EXPECT_NEAR(recentFlightRmse[1], 0.6175, 0.03);
    EXPECT_NEAR(recentFlightRmse[2], 29.17, 0.5);
    EXPECT_NEAR(recentFlightRmse[3], 0.4113, 0.03);
    EXPECT_NEAR(recentFlightRmse[4], 3.12, 0.1);

    // 상대 궤적과 입력은 그대로 두고 시작 높이만 올려 평지 접촉을 피한다.
    Json freeFlightScenario = recentFlightFixture;
    freeFlightScenario["drones"][0]["initial_position_meters"][2] =
        freeFlightScenario["drones"][0]["initial_position_meters"][2].get<double>()
        + 100.0;
    const ScenarioResult freeFlightResult = RunScenario(freeFlightScenario.dump());
    ASSERT_TRUE(freeFlightResult.IsPassed());
    ASSERT_EQ(freeFlightResult.trajectory.size(), recentReference.size());
    const std::array<double, 5> freeFlightRmse = calculateRmse(
        recentReference, freeFlightResult);
    double lowestFreeFlightHeightMeters = std::numeric_limits<double>::infinity();
    for (const ScenarioResult::TrajectorySample& frame : freeFlightResult.trajectory)
    {
        lowestFreeFlightHeightMeters = std::min(
            lowestFreeFlightHeightMeters,
            frame.drones.front().second.positionMeters.z);
    }
    EXPECT_GT(lowestFreeFlightHeightMeters, 50.0);
    EXPECT_NEAR(freeFlightRmse[0], 23.0456, 0.03);
    EXPECT_NEAR(freeFlightRmse[1], 14.7522, 0.03);
    EXPECT_NEAR(freeFlightRmse[2], 108.16, 0.5);
    EXPECT_NEAR(freeFlightRmse[3], 2.8490, 0.03);

    // 같은 구간에서 추력 계수만 높였을 때 오차가 어떻게 달라지는지 확인한다.
    Json increasedThrustRecentScenario = recentFlightFixture;
    auto& recentQuadrotorModel = increasedThrustRecentScenario["quadrotor_model"];
    recentQuadrotorModel["thrust_coefficient_newtons_per_speed_squared"] = 1.1
        * recentQuadrotorModel
            ["thrust_coefficient_newtons_per_speed_squared"].get<double>();
    const ScenarioResult increasedThrustRecentResult =
        RunScenario(increasedThrustRecentScenario.dump());
    ASSERT_TRUE(increasedThrustRecentResult.IsPassed());
    ASSERT_EQ(increasedThrustRecentResult.trajectory.size(), recentReference.size());
    const std::array<double, 5> increasedThrustRecentRmse = calculateRmse(
        recentReference, increasedThrustRecentResult);

    EXPECT_NEAR(increasedThrustRecentRmse[0], 0.6905, 0.03);
    EXPECT_NEAR(increasedThrustRecentRmse[1], 0.5915, 0.03);
    EXPECT_NEAR(increasedThrustRecentRmse[2], 28.73, 0.5);
    EXPECT_NEAR(increasedThrustRecentRmse[3], 0.4297, 0.03);

    Json px4GeometryRecentScenario = recentFlightFixture;
    // 공개 기록의 FRD 로터 위치를 프로젝트의 FLU 좌표로 바꿔 적용한다.
    px4GeometryRecentScenario["quadrotor_model"]["rotor_positions_meters"] = {
        {0.24, -0.24, 0.0}, {-0.24, 0.24, 0.0},
        {0.24, 0.24, 0.0}, {-0.24, -0.24, 0.0}};
    const ScenarioResult px4GeometryRecentResult =
        RunScenario(px4GeometryRecentScenario.dump());
    ASSERT_TRUE(px4GeometryRecentResult.IsPassed());
    ASSERT_EQ(px4GeometryRecentResult.trajectory.size(), recentReference.size());
    const std::array<double, 5> px4GeometryRecentRmse = calculateRmse(
        recentReference, px4GeometryRecentResult);

    EXPECT_NEAR(px4GeometryRecentRmse[0], 0.7152, 0.03);
    EXPECT_NEAR(px4GeometryRecentRmse[1], 0.6206, 0.03);
    EXPECT_NEAR(px4GeometryRecentRmse[2], 28.70, 0.5);
    EXPECT_NEAR(px4GeometryRecentRmse[3], 0.4664, 0.03);

    Json zeroSpeedRecentScenario = recentFlightFixture;
    zeroSpeedRecentScenario["drones"][0]["initial_motor_speeds_radians_per_second"] =
        {0.0, 0.0, 0.0, 0.0};
    const ScenarioResult zeroSpeedRecentResult =
        RunScenario(zeroSpeedRecentScenario.dump());
    ASSERT_TRUE(zeroSpeedRecentResult.IsPassed());
    ASSERT_EQ(zeroSpeedRecentResult.trajectory.size(), recentReference.size());
    const std::array<double, 5> zeroSpeedRecentRmse = calculateRmse(
        recentReference, zeroSpeedRecentResult);

    EXPECT_NEAR(zeroSpeedRecentRmse[0], 0.7286, 0.03);
    EXPECT_NEAR(zeroSpeedRecentRmse[1], 0.6383, 0.03);
    EXPECT_NEAR(zeroSpeedRecentRmse[2], 29.86, 0.5);
    EXPECT_NEAR(zeroSpeedRecentRmse[3], 0.3840, 0.03);

    // ESC가 측정한 회전수를 유지한 채 제조사 시험표 계수의 영향만 비교한다.
    Json motorTableRecentScenario = recentFlightFixture;
    auto& motorModel = motorTableRecentScenario["quadrotor_model"];
    motorModel["thrust_coefficient_newtons_per_speed_squared"] = 1.2294935e-5;
    motorModel["moment_to_thrust_ratio"] = 0.01403352;
    motorModel["maximum_motor_speed_radians_per_second"] = 1032.2226;
    for (std::size_t tick = 0;
         tick < motorTableRecentScenario["commands"].size(); ++tick)
    {
        const Json& measuredSpeeds = recentReference[tick]
            ["motor_speeds_radians_per_second"];
        Json normalizedThrust = Json::array();
        for (const Json& speed : measuredSpeeds)
        {
            const double ratio = speed.get<double>() / 1032.2226;
            normalizedThrust.push_back(ratio * ratio);
        }
        motorTableRecentScenario["commands"][tick]["normalized_thrust"] =
            std::move(normalizedThrust);
    }
    const ScenarioResult motorTableRecentResult =
        RunScenario(motorTableRecentScenario.dump());
    ASSERT_TRUE(motorTableRecentResult.IsPassed());
    ASSERT_EQ(motorTableRecentResult.trajectory.size(), recentReference.size());
    const std::array<double, 5> motorTableRecentRmse = calculateRmse(
        recentReference, motorTableRecentResult);

    EXPECT_NEAR(motorTableRecentRmse[0], 0.6832, 0.03);
    EXPECT_NEAR(motorTableRecentRmse[1], 0.5570, 0.03);
    EXPECT_NEAR(motorTableRecentRmse[2], 26.32, 0.5);
    EXPECT_NEAR(motorTableRecentRmse[3], 0.5646, 0.03);
    EXPECT_NEAR(motorTableRecentRmse[4], 3.12, 0.1);
}

void Verify_ScenarioRunner_ChangesWaypointResponseWhenControllerGainsChange()
{
    constexpr std::string_view scenarioTemplate = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":2500,
     "commands":[
       {"tick":0,"type":"takeoff"},
       {"tick":2401,"type":"set_waypoint","position_meters":[2,0,1.5]}],
     "controller_gains":{"position":POSITION_GAIN,"velocity":2.5,
       "attitude":25,"angular_rate":8.5},
     "expected":{"drones":[{"name":"drone","mode":"following_waypoint",
       "position_meters":[0,0,1.5]}],
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
    defaultGainScenario.replace(defaultGainScenario.find("\"attitude\":25"),
        std::string_view{"\"attitude\":25"}.size(), "\"attitude\":20");
    defaultGainScenario.replace(defaultGainScenario.find("8.5"), 3, "7.0");

    const ScenarioResult lowGain = RunScenario(lowGainScenario);
    const ScenarioResult highGain = RunScenario(highGainScenario);
    const ScenarioResult explicitDefault = RunScenario(defaultGainScenario);
    ASSERT_TRUE(lowGain.IsPassed())
        << (lowGain.issues.empty() ? "" : lowGain.issues.front());
    ASSERT_TRUE(highGain.IsPassed())
        << (highGain.issues.empty() ? "" : highGain.issues.front());
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
    EXPECT_GT(highGain.drones.front().finalState.positionMeters.x,
        lowGain.drones.front().finalState.positionMeters.x);
    ASSERT_TRUE(implicitDefault.controllerSettings.has_value());
    EXPECT_FALSE(implicitDefault.controllerSettings->customized);
    EXPECT_EQ(explicitDefault.drones.front().finalState.positionMeters,
        implicitDefault.drones.front().finalState.positionMeters);

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

void Verify_ScenarioRunner_AccountsForFlightAndHoverPower()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,
     "drones":[
       {"name":"flight-drone","initial_position_meters":[0,0,0],
        "battery_capacity_watt_hours":10},
       {"name":"hover-drone","initial_position_meters":[0,2,1.5],
        "battery_capacity_watt_hours":10}],
     "commands":[
       {"tick":0,"drone":"flight-drone","type":"takeoff"},
       {"tick":0,"drone":"hover-drone","type":"takeoff"}],
     "energy_model":{"capacity_watt_hours":10,"hover_power_watts":40,
       "flight_power_watts":120,"reserve_percent":20},
     "expected":{"tolerance_meters":0.000001,"drones":[
       {"name":"flight-drone","mode":"taking_off",
        "position_meters":[0,0,0.016666666666666666]},
       {"name":"hover-drone","mode":"hovering","position_meters":[0,2,1.5]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    ASSERT_TRUE(result.IsPassed());
    ASSERT_EQ(result.drones.size(), 2U);
    for (const auto& drone : result.drones)
    {
        ASSERT_TRUE(drone.energyConsumedWattHours.has_value());
        ASSERT_TRUE(drone.batteryRemainingWattHours.has_value());
    }

    constexpr double expectedFlightUseWattHours = 120.0 / 60.0 / 3600.0;
    constexpr double expectedHoverUseWattHours = 40.0 / 60.0 / 3600.0;
    EXPECT_NEAR(*result.drones[0].energyConsumedWattHours,
        expectedFlightUseWattHours, 1.0e-12);
    EXPECT_NEAR(*result.drones[0].batteryRemainingWattHours,
        10.0 - expectedFlightUseWattHours, 1.0e-12);
    EXPECT_NEAR(*result.drones[1].energyConsumedWattHours,
        expectedHoverUseWattHours, 1.0e-12);
    EXPECT_NEAR(*result.drones[1].batteryRemainingWattHours,
        10.0 - expectedHoverUseWattHours, 1.0e-12);
}

void Verify_ScenarioRunner_RecordsWindAndChangesTheFlightPath()
{
    constexpr std::string_view calmScenario = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":1200,
     "commands":[{"tick":0,"type":"takeoff"},
       {"tick":600,"type":"set_waypoint","position_meters":[0,0,1.5]}],
     "expected":{"drones":[{"name":"drone","mode":"hovering",
       "position_meters":[0,0,1.5]}],
       "tolerance_meters":10}}
    )json";
    constexpr std::string_view windScenario = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":1200,
     "wind_velocity_meters_per_second":[2,0,0],
     "commands":[{"tick":0,"type":"takeoff"},
       {"tick":600,"type":"set_waypoint","position_meters":[0,0,1.5]}],
     "expected":{"drones":[{"name":"drone","mode":"hovering",
       "position_meters":[0,0,1.5]}],
       "tolerance_meters":10}}
    )json";

    const ScenarioResult calm = RunScenario(calmScenario);
    const ScenarioResult wind = RunScenario(windScenario);
    ASSERT_TRUE(calm.IsPassed());
    ASSERT_TRUE(wind.IsPassed());
    ASSERT_TRUE(calm.windVelocityMetersPerSecond == std::nullopt);
    ASSERT_EQ(wind.windVelocityMetersPerSecond, (glm::dvec3{2.0, 0.0, 0.0}));
    EXPECT_GT(std::abs(wind.drones.front().finalState.positionMeters.x
        - calm.drones.front().finalState.positionMeters.x), 0.01);
}

void Verify_ScenarioRunner_ContinuesTaskDuringHeartbeatLoss()
{
    const ScenarioResult result = RunScenario(
        ReadScenarioFixture("scenarios/multi-drone/four-drone-survey-outage.json"));
    EXPECT_FALSE(result.IsPassed());
    ASSERT_TRUE(result.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*result.minimumDroneSeparationMeters, 2.0);
    EXPECT_EQ(result.outcome, ScenarioResult::Outcome::Inconclusive);
    ASSERT_TRUE(result.mission.has_value());
    EXPECT_FALSE(result.mission->IsPassed());
    EXPECT_EQ(result.mission->GetOutcome(), ScenarioResult::Outcome::Inconclusive);
    EXPECT_EQ(result.mission->CompletedTaskCount(), 3U);
    EXPECT_FALSE(result.mission->CompletedTick().has_value());
    ASSERT_TRUE(result.mission->startedTick.has_value());
    EXPECT_EQ(*result.mission->startedTick, 90U);
    for (const ScenarioResult::MissionTask& task : result.mission->tasks)
    {
        if (task.name == "sector-2")
        {
            EXPECT_FALSE(task.IsCompleted());
            EXPECT_EQ(task.droneName, "scout-2");
        }
        else
        {
            EXPECT_TRUE(task.IsCompleted()) << task.name;
        }
    }

    const auto disconnectedDrone = std::find_if(result.drones.begin(), result.drones.end(),
        [](const ScenarioResult::Drone& drone) { return drone.name == "scout-2"; });
    ASSERT_NE(disconnectedDrone, result.drones.end());
    EXPECT_EQ(disconnectedDrone->communicationStatus,
        ScenarioResult::Drone::CommunicationStatus::Unknown);
    EXPECT_DOUBLE_EQ(disconnectedDrone->finalState.positionMeters.x,
        10.333333333333336);
    EXPECT_TRUE(std::any_of(result.eventLog.begin(), result.eventLog.end(),
        [](const std::string& event)
        {
            return event.find("상태를 확인할 수 없어 임무 정보를 미확정으로 바꿨습니다")
                != std::string::npos;
        }));

    const auto assignedTask = std::find_if(result.mission->tasks.begin(),
        result.mission->tasks.end(),
        [](const ScenarioResult::MissionTask& task) { return task.name == "sector-2"; });
    ASSERT_NE(assignedTask, result.mission->tasks.end());
    EXPECT_FALSE(assignedTask->failed);

    ASSERT_FALSE(result.trajectory.empty());
    const ScenarioResult::TrajectorySample& lastSample = result.trajectory.back();
    EXPECT_EQ(lastSample.tick, 320U);
    const auto actualDrone = std::find_if(lastSample.drones.begin(), lastSample.drones.end(),
        [](const auto& entry) { return entry.first == "scout-2"; });
    ASSERT_NE(actualDrone, lastSample.drones.end());
    EXPECT_EQ(actualDrone->second.mode, FlightMode::Hovering);
    EXPECT_DOUBLE_EQ(actualDrone->second.positionMeters.x, 14.0);
}
}

void Verify_ScenarioRunner_RunsNamedDronesWithIndependentCommands()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":180,
     "drones":[
       {"name":"lead","initial_position_meters":[0,0,0]},
       {"name":"scout","initial_position_meters":[10,0,0],"initial_yaw_radians":1.2}],
     "commands":[
       {"tick":0,"drone":"lead","type":"takeoff"},
       {"tick":0,"drone":"scout","type":"takeoff"},
       {"tick":90,"drone":"scout","type":"set_waypoint",
        "position_meters":[10.724715508953347,1.8640781719344526,1.5]}],
     "expected":{"tolerance_meters":0.000001,"orientation_tolerance_radians":0.000001,
       "minimum_drone_separation_meters":9.9,"drones":[
       {"name":"lead","mode":"hovering","position_meters":[0,0,1.5]},
       {"name":"scout","mode":"hovering",
        "position_meters":[10.724715508953347,1.8640781719344526,1.5],
        "orientation_quaternion_wxyz":[-0.8253356149096783,0,0,-0.5646424733950354]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    EXPECT_TRUE(result.IsPassed()) << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_EQ(result.drones.size(), 2);
    EXPECT_EQ(result.drones[0].name, "lead");
    EXPECT_EQ(result.drones[1].name, "scout");
    EXPECT_EQ(result.drones[0].finalState.mode, FlightMode::Hovering);
    EXPECT_EQ(result.drones[1].finalState.mode, FlightMode::Hovering);
    EXPECT_NEAR(result.drones[1].finalState.yawRadians, 1.2, 1.0e-12);
    EXPECT_DOUBLE_EQ(result.drones[0].finalState.positionMeters.z, 1.5);
    EXPECT_NEAR(result.drones[1].finalState.positionMeters.x,
        10.724715508953347, 1.0e-12);
    EXPECT_NEAR(result.drones[1].finalState.positionMeters.y,
        1.8640781719344526, 1.0e-12);
    EXPECT_NEAR(result.drones[0].flightDistanceMeters, 1.5, 1.0e-9);
    EXPECT_NEAR(result.drones[1].flightDistanceMeters, 3.5, 1.0e-9);
    EXPECT_NEAR(result.drones[0].flightTimeSeconds, 3.0, 1.0e-12);
    EXPECT_NEAR(result.drones[1].flightTimeSeconds, 3.0, 1.0e-12);
    EXPECT_TRUE(result.eventLog.empty());
    ASSERT_TRUE(result.minimumDroneSeparationMeters.has_value());
    EXPECT_NEAR(*result.minimumDroneSeparationMeters, 10.0, 1.0e-12);

    std::string unsafeScenario{scenario};
    unsafeScenario.replace(unsafeScenario.find("9.9"), 3, "10.1");
    const ScenarioResult unsafeResult = RunScenario(unsafeScenario);
    EXPECT_FALSE(unsafeResult.IsPassed());
    ASSERT_EQ(unsafeResult.issues.size(), 1);
    EXPECT_NE(unsafeResult.issues.front().find("최소 거리"), std::string::npos);

    std::string wrongOrientationScenario{scenario};
    const std::string expectedOrientation =
        "-0.8253356149096783,0,0,-0.5646424733950354";
    wrongOrientationScenario.replace(
        wrongOrientationScenario.find(expectedOrientation), expectedOrientation.size(), "1,0,0,0");
    const ScenarioResult wrongOrientationResult = RunScenario(wrongOrientationScenario);
    EXPECT_FALSE(wrongOrientationResult.IsPassed());
    ASSERT_EQ(wrongOrientationResult.issues.size(), 1);
    EXPECT_NE(wrongOrientationResult.issues.front().find("최종 자세"), std::string::npos);

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
    EXPECT_TRUE(result.IsPassed()) << (result.issues.empty() ? "" : result.issues.front());
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
       {"name":"sector-1","drone":"scout",
        "waypoints_meters":[[100,0,1.5]],"start_tick":120,
        "deadline_tick":144,"tolerance_meters":0.1},
       {"name":"sector-2","drone":"relay",
        "waypoints_meters":[[105,0,1.5]],"start_tick":120,
        "deadline_tick":144,"tolerance_meters":0.1}]},
     "expected":{"tolerance_meters":1000,"drones":[
       {"name":"scout","mode":"following_waypoint","position_meters":[0,0,1.5]},
       {"name":"relay","mode":"following_waypoint","position_meters":[5,0,1.5]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    ASSERT_TRUE(result.mission.has_value());
    EXPECT_EQ(result.outcome, ScenarioResult::Outcome::Inconclusive);
    EXPECT_FALSE(result.IsPassed());
    EXPECT_EQ(result.mission->GetOutcome(), ScenarioResult::Outcome::Inconclusive);
    EXPECT_FALSE(result.mission->IsPassed());
    ASSERT_EQ(result.mission->tasks.size(), 2U);
    for (const ScenarioResult::MissionTask& task : result.mission->tasks)
    {
        EXPECT_FALSE(task.IsCompleted());
        EXPECT_FALSE(task.failed);
        EXPECT_FALSE(task.droneName.empty());
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
       {"name":"sector-1","drone":"scout",
        "waypoints_meters":[[100,0,1.5]],"start_tick":120,
        "deadline_tick":121,"tolerance_meters":0.1},
       {"name":"sector-2","drone":"relay",
        "waypoints_meters":[[105,0,1.5]],"start_tick":120,
        "deadline_tick":121,"tolerance_meters":0.1}]},
     "expected":{"tolerance_meters":1000,"drones":[
       {"name":"scout","mode":"following_waypoint","position_meters":[0,0,1.5]},
       {"name":"relay","mode":"following_waypoint","position_meters":[5,0,1.5]}]}}
    )json";

    const ScenarioResult failedResult = RunScenario(missedDeadlineScenario);
    ASSERT_TRUE(failedResult.mission.has_value());
    EXPECT_EQ(failedResult.outcome, ScenarioResult::Outcome::Failed);
    EXPECT_EQ(failedResult.mission->GetOutcome(), ScenarioResult::Outcome::Failed);
    EXPECT_FALSE(failedResult.IsPassed());
    EXPECT_FALSE(failedResult.issues.empty());
}

void Verify_ScenarioRunner_RunsAssignedWaypointsAndCollisionScenarios()
{
    constexpr std::string_view assignedSurveyScenario = R"json(
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
     "mission":{"name":"grid-survey","tasks":[
       {"name":"sector-1","drone":"survey-1",
        "waypoints_meters":[[0,0,1.5],[4,0,1.5],[4,8,1.5],[0,8,1.5]],
        "start_tick":100,"deadline_tick":470,"tolerance_meters":0.2},
       {"name":"sector-2","drone":"survey-2",
        "waypoints_meters":[[4,0,1.5],[8,0,1.5],[8,8,1.5],[4,8,1.5]],
        "start_tick":100,"deadline_tick":470,"tolerance_meters":0.2},
       {"name":"sector-3","drone":"survey-3",
        "waypoints_meters":[[8,0,1.5],[12,0,1.5],[12,8,1.5],[8,8,1.5]],
        "start_tick":100,"deadline_tick":470,"tolerance_meters":0.2},
       {"name":"sector-4","drone":"survey-4",
        "waypoints_meters":[[12,0,1.5],[16,0,1.5],[16,8,1.5],[12,8,1.5]],
        "start_tick":100,"deadline_tick":470,"tolerance_meters":0.2}]},
     "expected":{"tolerance_meters":0.2,"drones":[
       {"name":"survey-1","mode":"hovering","position_meters":[0,8,1.5]},
       {"name":"survey-2","mode":"hovering","position_meters":[4,8,1.5]},
       {"name":"survey-3","mode":"hovering","position_meters":[8,8,1.5]},
       {"name":"survey-4","mode":"hovering","position_meters":[12,8,1.5]}]}}
    )json";
    const ScenarioResult surveyResult = RunScenario(assignedSurveyScenario);
    ASSERT_TRUE(surveyResult.IsPassed())
        << (surveyResult.issues.empty() ? "" : surveyResult.issues.front());
    ASSERT_TRUE(surveyResult.mission.has_value());
    EXPECT_TRUE(surveyResult.mission->IsPassed());
    EXPECT_DOUBLE_EQ(surveyResult.mission->WaypointProgressPercent(), 100.0);
    ASSERT_EQ(surveyResult.mission->tasks.size(), 4);
    for (std::size_t index = 0; index < surveyResult.mission->tasks.size(); ++index)
    {
        const auto& task = surveyResult.mission->tasks[index];
        EXPECT_TRUE(task.IsCompleted()) << task.name;
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
       {"name":"west-to-east","drone":"west",
        "waypoints_meters":[[4,0,1.5]],"start_tick":100,"deadline_tick":580,"tolerance_meters":0.2},
       {"name":"east-to-west","drone":"east",
        "waypoints_meters":[[-4,0,1.5]],"start_tick":100,"deadline_tick":580,"tolerance_meters":0.2}]},
     "expected":{"tolerance_meters":0.5,"minimum_drone_separation_meters":2.5,
       "drones":[
         {"name":"west","mode":"hovering","position_meters":[4,0,1.5]},
         {"name":"east","mode":"hovering","position_meters":[-4,0,1.5]}]}}
    )json";
    const ScenarioResult avoidanceResult = RunScenario(collisionAvoidanceScenario);
    EXPECT_TRUE(avoidanceResult.IsPassed())
        << (avoidanceResult.issues.empty() ? "" : avoidanceResult.issues.front())
        << " minimum=" << avoidanceResult.minimumDroneSeparationMeters.value_or(-1.0)
        << " avoidance_events=" << avoidanceResult.collisionAvoidanceEvents;
    ASSERT_TRUE(avoidanceResult.mission.has_value());
    EXPECT_TRUE(avoidanceResult.mission->IsPassed());
    EXPECT_GT(avoidanceResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(avoidanceResult.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*avoidanceResult.minimumDroneSeparationMeters, 2.5);
    ASSERT_TRUE(avoidanceResult.minimumPredictedDroneSeparationMeters.has_value());
    EXPECT_LT(*avoidanceResult.minimumPredictedDroneSeparationMeters, 3.0);

    constexpr std::string_view stationaryDroneScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":600,
     "drones":[
       {"name":"worker","initial_position_meters":[-4,0,0]},
       {"name":"stationary","initial_position_meters":[0,0,0]},
       {"name":"support","initial_position_meters":[20,0,0]}],
     "commands":[
       {"tick":0,"drone":"worker","type":"takeoff"},
       {"tick":0,"drone":"stationary","type":"takeoff"},
       {"tick":0,"drone":"support","type":"takeoff"}],
     "collision_avoidance":{"safety_distance_meters":3,
       "prediction_horizon_seconds":2,"lateral_offset_meters":5},
     "mission":{"name":"pass-the-hovering-drone","tasks":[
       {"name":"cross","drone":"worker",
        "waypoints_meters":[[4,0,1.5]],"start_tick":100,
        "deadline_tick":590,"tolerance_meters":0.2},
       {"name":"support","drone":"support",
        "waypoints_meters":[[21,0,1.5]],"start_tick":100,
        "deadline_tick":590,"tolerance_meters":0.2}]},
     "expected":{"tolerance_meters":0.5,
       "minimum_drone_separation_meters":2.5,
       "drones":[
         {"name":"worker","mode":"hovering","position_meters":[4,0,1.5]},
         {"name":"stationary","mode":"hovering","position_meters":[0,0,1.5]},
         {"name":"support","mode":"hovering","position_meters":[21,0,1.5]}]}}
    )json";
    const ScenarioResult stationaryDroneResult = RunScenario(stationaryDroneScenario);
    EXPECT_TRUE(stationaryDroneResult.IsPassed())
        << (stationaryDroneResult.issues.empty()
            ? "" : stationaryDroneResult.issues.front())
        << " minimum="
        << stationaryDroneResult.minimumDroneSeparationMeters.value_or(-1.0)
        << " avoidance_events=" << stationaryDroneResult.collisionAvoidanceEvents;
    ASSERT_TRUE(stationaryDroneResult.mission.has_value());
    EXPECT_TRUE(stationaryDroneResult.mission->IsPassed());
    EXPECT_EQ(stationaryDroneResult.mission->CompletedTaskCount(), 2U);
    EXPECT_GT(stationaryDroneResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(stationaryDroneResult.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*stationaryDroneResult.minimumDroneSeparationMeters, 2.5);

#if !defined(__EMSCRIPTEN__)
    std::string physicalAvoidanceScenario{collisionAvoidanceScenario};
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario, "\"dynamics\":\"kinematic\"",
        "\"dynamics\":\"rigid_body\""));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"ticks\":600", "\"ticks\":3600"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"deadline_tick\":580", "\"deadline_tick\":3590"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"safety_distance_meters\":3",
        "\"safety_distance_meters\":8"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"prediction_horizon_seconds\":2",
        "\"prediction_horizon_seconds\":6"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"tolerance_meters\":0.2",
        "\"tolerance_meters\":0.5"));
    // 완료한 드론과 목표 간격을 안전거리보다 멀리 둬 임무 자체도 도달 가능하게 한다.
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"waypoints_meters\":[[4,0,1.5]]",
        "\"waypoints_meters\":[[8,0,1.5]]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"waypoints_meters\":[[-4,0,1.5]]",
        "\"waypoints_meters\":[[-8,0,1.5]]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"position_meters\":[4,0,1.5]",
        "\"position_meters\":[8,0,1.5]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"position_meters\":[-4,0,1.5]",
        "\"position_meters\":[-8,0,1.5]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"initial_position_meters\":[-4,0,0]",
        "\"initial_position_meters\":[-8,0,0]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"initial_position_meters\":[4,0,0]",
        "\"initial_position_meters\":[8,0,0]"));
    const ScenarioResult physicalAvoidanceResult = RunScenario(physicalAvoidanceScenario);
    ASSERT_TRUE(physicalAvoidanceResult.IsPassed())
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
            ? static_cast<int>(physicalAvoidanceResult.mission->GetOutcome()) : -1);
    ASSERT_TRUE(physicalAvoidanceResult.mission.has_value());
    EXPECT_TRUE(physicalAvoidanceResult.mission->IsPassed());
    EXPECT_EQ(physicalAvoidanceResult.mission->CompletedTaskCount(), 2U);
    EXPECT_GT(physicalAvoidanceResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(physicalAvoidanceResult.minimumDroneSeparationMeters.has_value());
    // 시작 간격을 안전거리보다 넉넉히 둬 회피 명령이 작동할 시간을 준다.
    EXPECT_GE(*physicalAvoidanceResult.minimumDroneSeparationMeters, 8.0);
#endif

    const ScenarioResult energyResult = RunScenario(ReadScenarioFixture(
        "scenarios/multi-drone/low-battery-return.json"));
    EXPECT_FALSE(energyResult.IsPassed());
    ASSERT_TRUE(energyResult.mission.has_value());
    EXPECT_EQ(energyResult.mission->GetOutcome(), ScenarioResult::Outcome::Failed);
    ASSERT_EQ(energyResult.mission->tasks.size(), 2U);
    EXPECT_EQ(energyResult.mission->tasks[0].droneName, "low-charge");
    EXPECT_TRUE(energyResult.mission->tasks[0].failed);
    EXPECT_EQ(energyResult.mission->tasks[1].droneName, "reserve");
    EXPECT_TRUE(energyResult.mission->tasks[1].IsCompleted());
    ASSERT_EQ(energyResult.drones.size(), 2U);
    const auto& depletedDrone = energyResult.drones[0];
    EXPECT_TRUE(depletedDrone.returnedForLowBattery);
    ASSERT_TRUE(depletedDrone.batteryPercent.has_value());
    EXPECT_EQ(depletedDrone.finalState.mode, FlightMode::Grounded);
    EXPECT_NEAR(depletedDrone.finalState.positionMeters.x, 0.0, 0.2);
    EXPECT_NEAR(energyResult.drones[1].finalState.positionMeters.x, 21.0, 0.2);

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
        Verify_ScenarioRunnerCompletesTasksAtWaypoints());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ReplaysSixDofStateAndRotorThrust());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_RecordsPx4ReferenceErrorsAcrossModelVariants());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_UsesAndReportsPhysicalModelOverrides());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ChangesWaypointResponseWhenControllerGainsChange());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_AccountsForFlightAndHoverPower());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_RecordsWindAndChangesTheFlightPath());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ContinuesTaskDuringHeartbeatLoss());
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

    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_RunsNamedDronesWithIndependentCommands());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_FindsClosestPairAcrossThreeDrones());
    EXPECT_NO_FATAL_FAILURE(Verify_ScenarioRunner_LeavesUnreachableWorkInconclusive());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_RunsAssignedWaypointsAndCollisionScenarios());

    constexpr std::string_view sequentialDroneTasks = R"json(
    {"version":1,"dynamics":"kinematic","ticks":500,
     "commands":[{"tick":0,"type":"takeoff"}],
     "mission":{"name":"two-stop-survey","tasks":[
       {"name":"scan","drone":"drone",
        "waypoints_meters":[[0,0,1.5]],"start_tick":0,
        "deadline_tick":499,"tolerance_meters":0.1},
       {"name":"guard","drone":"drone",
        "waypoints_meters":[[2,0,1.5]],"start_tick":0,
        "deadline_tick":499,"tolerance_meters":0.1}]},
     "expected":{"drones":[
       {"name":"drone","mode":"hovering","position_meters":[2,0,1.5]}]}}
    )json";
    const ScenarioResult sequentialResult = RunScenario(sequentialDroneTasks);
    ASSERT_TRUE(sequentialResult.IsPassed())
        << (sequentialResult.issues.empty() ? "" : sequentialResult.issues.front());
    ASSERT_TRUE(sequentialResult.mission.has_value());
    EXPECT_EQ(sequentialResult.mission->CompletedTaskCount(), 2U);

    constexpr std::string_view targetPositionAlias = R"json(
    {"version":1,"dynamics":"kinematic","ticks":2,
     "drones":[{"name":"lead"}],"commands":[],
     "mission":{"name":"survey","tasks":[
       {"name":"first","drone":"lead",
        "target_position_meters":[0,0,1],"start_tick":0,
        "deadline_tick":1,"tolerance_meters":0}]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(targetPositionAlias)), std::invalid_argument);

    constexpr std::string_view singleDroneExpectedState = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,"commands":[],
     "expected":{"mode":"grounded","position_meters":[0,0,0]}}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(singleDroneExpectedState)),
        std::invalid_argument);
}

}
