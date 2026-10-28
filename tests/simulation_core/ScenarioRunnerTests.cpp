#include "simulation_core/ScenarioRunner.h"
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

void Verify_ScenarioRunner_ControlsAndReportsCamerasPerDrone()
{
    constexpr std::string_view scenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":4,
     "drones":[
       {"name":"east","initial_position_meters":[0,0,0]},
       {"name":"north","initial_position_meters":[0,1,0]}],
     "commands":[],
     "camera_events":[
       {"tick":0,"drone":"east","action":"disable"},
       {"tick":0,"drone":"north","action":"fault"},
       {"tick":2,"drone":"north","action":"recover"}],
     "expected":{"camera_status":"disabled","camera_frame_count":0,
       "drones":[
         {"name":"east","mode":"grounded","position_meters":[0,0,0]},
         {"name":"north","mode":"grounded","position_meters":[0,1,0]}]}}
    )json";

    const ScenarioResult result = RunScenario(scenario);
    ASSERT_TRUE(result.passed)
        << (result.issues.empty() ? "" : result.issues.front());
    ASSERT_EQ(result.drones.size(), 2U);
    EXPECT_EQ(result.drones[0].name, "east");
    EXPECT_EQ(result.drones[0].finalCamera.status, CameraStatus::Disabled);
    EXPECT_EQ(result.drones[0].finalCamera.frameCount, 0U);
    EXPECT_EQ(result.drones[1].name, "north");
    EXPECT_EQ(result.drones[1].finalCamera.status, CameraStatus::Active);
    EXPECT_EQ(result.drones[1].finalCamera.frameCount, 2U);
    EXPECT_EQ(result.finalCamera.status, result.drones[0].finalCamera.status);
    EXPECT_EQ(result.eventLog.size(), 3U);

    constexpr std::string_view unknownDrone = R"json(
    {"version":1,"dynamics":"kinematic","ticks":1,"commands":[],
     "camera_events":[{"tick":0,"drone":"missing","action":"fault"}]}
    )json";
    EXPECT_THROW(static_cast<void>(RunScenario(unknownDrone)), std::invalid_argument);
}

void Verify_ScenarioRunner_RequiresCameraGeometryForInspectionTasks()
{
    constexpr std::string_view source = R"json(
    {"version":1,"dynamics":"kinematic","ticks":150,
     "drones":[
       {"name":"east","initial_position_meters":[0,0,0]},
       {"name":"north","initial_position_meters":[0,4,0]}],
     "commands":[
       {"tick":0,"drone":"east","type":"takeoff"},
       {"tick":0,"drone":"north","type":"takeoff"}],
     "mission":{"name":"bridge-inspection","tasks":[
       {"name":"east-face","role":"inspect","drone":"east",
        "facility_id":"tancheon-bridge","inspection_zone_id":"deck-east",
        "target_position_meters":[0,0,1.5],"start_tick":110,
        "deadline_tick":145,"tolerance_meters":0.1,
        "inspection_surface":{
          "center_meters":[10,0,0.62],"normal":[-1,0,0],
          "width_axis":[0,1,0],"width_meters":2,"height_meters":1,
          "minimum_distance_meters":5,"maximum_distance_meters":15,
          "maximum_incidence_angle_degrees":30,
          "minimum_pixel_width":100,"minimum_pixel_height":50}},
       {"name":"north-face","role":"inspect","drone":"north",
        "facility_id":"tancheon-bridge","inspection_zone_id":"deck-north",
        "target_position_meters":[0,4,1.5],"start_tick":110,
        "deadline_tick":145,"tolerance_meters":0.1,
        "inspection_surface":{
          "center_meters":[10,4,0.62],"normal":[-1,0,0],
          "width_axis":[0,1,0],"width_meters":2,"height_meters":1,
          "minimum_distance_meters":5,"maximum_distance_meters":15,
          "maximum_incidence_angle_degrees":30,
          "minimum_pixel_width":100,"minimum_pixel_height":50}}]},
     "expected":{"drones":[
       {"name":"east","mode":"hovering","position_meters":[0,0,1.5]},
       {"name":"north","mode":"hovering","position_meters":[0,4,1.5]}]}}
    )json";

    const ScenarioResult successful = RunScenario(source);
    ASSERT_TRUE(successful.passed)
        << (successful.issues.empty() ? "" : successful.issues.front());
    ASSERT_TRUE(successful.mission.has_value());
    ASSERT_EQ(successful.mission->tasks.size(), 2U);
    EXPECT_TRUE(successful.mission->tasks[0].completed);
    EXPECT_TRUE(successful.mission->tasks[0].firstGeometryCaptureTick.has_value());
    EXPECT_EQ(successful.mission->tasks[0].lastCaptureAssessment,
        CaptureAssessment::Ready);
    EXPECT_EQ(successful.mission->tasks[0].facilityId, "tancheon-bridge");
    EXPECT_EQ(successful.mission->tasks[0].inspectionZoneId, "deck-east");
    EXPECT_EQ(successful.drones[0].finalCamera.model.imageWidthPixels, 1280);

    nlohmann::json singleDroneMission = nlohmann::json::parse(source);
    singleDroneMission["mission"]["tasks"].erase(1);
    singleDroneMission["drones"].erase(1);
    singleDroneMission["commands"].erase(1);
    singleDroneMission["expected"]["drones"].erase(1);
    const ScenarioResult singleDrone = RunScenario(singleDroneMission.dump());
    ASSERT_TRUE(singleDrone.passed)
        << (singleDrone.issues.empty() ? "" : singleDrone.issues.front());
    ASSERT_TRUE(singleDrone.mission.has_value());
    ASSERT_EQ(singleDrone.mission->tasks.size(), 1U);
    EXPECT_TRUE(singleDrone.mission->tasks[0].completed);

    nlohmann::json sequentialZones = nlohmann::json::parse(source);
    sequentialZones["mission"]["tasks"][1]["drone"] = "east";
    sequentialZones["mission"]["tasks"][1]["target_position_meters"] = {0.0, 0.0, 1.5};
    sequentialZones["mission"]["tasks"][1]["inspection_surface"]["center_meters"] =
        {10.0, 0.7, 0.62};
    const ScenarioResult oneDroneTwoZones = RunScenario(sequentialZones.dump());
    ASSERT_TRUE(oneDroneTwoZones.passed)
        << (oneDroneTwoZones.issues.empty() ? "" : oneDroneTwoZones.issues.front());
    ASSERT_TRUE(oneDroneTwoZones.mission.has_value());
    EXPECT_TRUE(oneDroneTwoZones.mission->tasks[0].completed);
    EXPECT_TRUE(oneDroneTwoZones.mission->tasks[1].completed);
    ASSERT_TRUE(oneDroneTwoZones.mission->tasks[0].completedTick.has_value());
    ASSERT_TRUE(oneDroneTwoZones.mission->tasks[1].completedTick.has_value());
    EXPECT_GT(*oneDroneTwoZones.mission->tasks[1].completedTick,
        *oneDroneTwoZones.mission->tasks[0].completedTick);

    nlohmann::json missingFacilityId = nlohmann::json::parse(source);
    missingFacilityId["mission"]["tasks"][0].erase("facility_id");
    EXPECT_THROW(static_cast<void>(RunScenario(missingFacilityId.dump())),
        std::invalid_argument);

    nlohmann::json obstructedSurface = nlohmann::json::parse(source);
    obstructedSurface["mission"]["tasks"][0]["inspection_occluders"] =
        nlohmann::json::array({{{"center_meters", {5.0, 0.0, 1.0}},
            {"half_extents_meters", {0.5, 2.0, 2.0}}}});
    const ScenarioResult obstructed = RunScenario(obstructedSurface.dump());
    ASSERT_TRUE(obstructed.mission.has_value());
    EXPECT_FALSE(obstructed.mission->tasks[0].completed);
    EXPECT_FALSE(obstructed.mission->tasks[0].firstGeometryCaptureTick.has_value());
    EXPECT_EQ(obstructed.mission->tasks[0].lastCaptureAssessment,
        CaptureAssessment::Occluded);

    nlohmann::json boxBehindSurface = nlohmann::json::parse(source);
    boxBehindSurface["mission"]["tasks"][0]["inspection_occluders"] =
        nlohmann::json::array({{{"center_meters", {12.0, 0.0, 1.0}},
            {"half_extents_meters", {0.5, 2.0, 2.0}}}});
    const ScenarioResult visibleSurface = RunScenario(boxBehindSurface.dump());
    ASSERT_TRUE(visibleSurface.mission.has_value());
    EXPECT_TRUE(visibleSurface.mission->tasks[0].completed);
    EXPECT_EQ(visibleSurface.mission->tasks[0].lastCaptureAssessment,
        CaptureAssessment::Ready);

    nlohmann::json invalidOccluder = nlohmann::json::parse(source);
    invalidOccluder["mission"]["tasks"][0]["inspection_occluders"] =
        nlohmann::json::array({{{"center_meters", {5.0, 0.0, 1.0}},
            {"half_extents_meters", {0.5, 0.0, 2.0}}}});
    EXPECT_THROW(static_cast<void>(RunScenario(invalidOccluder.dump())),
        std::invalid_argument);

    nlohmann::json duplicateZone = nlohmann::json::parse(source);
    duplicateZone["mission"]["tasks"][1]["inspection_zone_id"] = "deck-east";
    EXPECT_THROW(static_cast<void>(RunScenario(duplicateZone.dump())),
        std::invalid_argument);

    nlohmann::json differentCameraModels = nlohmann::json::parse(source);
    differentCameraModels["camera_model"] = {
        {"image_width_pixels", 1920},
        {"image_height_pixels", 1080},
        {"vertical_field_of_view_degrees", 60.0}};
    differentCameraModels["drones"][0]["camera_model"] = {
        {"image_width_pixels", 160},
        {"image_height_pixels", 120},
        {"vertical_field_of_view_degrees", 70.0}};
    const ScenarioResult differentCameras = RunScenario(differentCameraModels.dump());
    EXPECT_EQ(differentCameras.drones[0].finalCamera.model.imageWidthPixels, 160);
    EXPECT_EQ(differentCameras.drones[0].finalCamera.model.imageHeightPixels, 120);
    EXPECT_EQ(differentCameras.drones[1].finalCamera.model.imageWidthPixels, 1920);
    EXPECT_EQ(differentCameras.drones[1].finalCamera.model.imageHeightPixels, 1080);
    ASSERT_TRUE(differentCameras.mission.has_value());
    EXPECT_FALSE(differentCameras.mission->tasks[0].completed);
    EXPECT_EQ(differentCameras.mission->tasks[0].lastCaptureAssessment,
        CaptureAssessment::ResolutionTooLow);
    EXPECT_TRUE(differentCameras.mission->tasks[1].completed);

    nlohmann::json invalidCamera = nlohmann::json::parse(source);
    invalidCamera["drones"][0]["camera_model"]["vertical_field_of_view_degrees"] = 180;
    EXPECT_THROW(static_cast<void>(RunScenario(invalidCamera.dump())), std::invalid_argument);

    nlohmann::json withCameraDisabled = nlohmann::json::parse(source);
    withCameraDisabled["camera_events"] = nlohmann::json::array({
        {{"tick", 105}, {"drone", "east"}, {"action", "disable"}}});
    const ScenarioResult incomplete = RunScenario(withCameraDisabled.dump());
    ASSERT_FALSE(incomplete.passed);
    ASSERT_TRUE(incomplete.mission.has_value());
    ASSERT_TRUE(incomplete.mission->tasks[0].failed);
    EXPECT_FALSE(incomplete.mission->tasks[0].firstGeometryCaptureTick.has_value());
    EXPECT_EQ(incomplete.mission->tasks[0].lastCaptureAssessment,
        CaptureAssessment::CameraUnavailable);

    nlohmann::json recoveredCamera = withCameraDisabled;
    recoveredCamera["ticks"] = 220;
    recoveredCamera["mission"]["tasks"].erase(1);
    recoveredCamera["mission"]["tasks"][0]["max_capture_retries"] = 1;
    recoveredCamera["drones"][1]["initial_position_meters"] = {0.0, 1.0, 0.0};
    recoveredCamera["expected"]["drones"][1]["position_meters"] = {0.0, 0.0, 1.5};
    const ScenarioResult retried = RunScenario(recoveredCamera.dump());
    ASSERT_TRUE(retried.passed)
        << (retried.issues.empty() ? "" : retried.issues.front());
    ASSERT_TRUE(retried.mission.has_value());
    const auto& retriedTask = retried.mission->tasks[0];
    EXPECT_TRUE(retriedTask.completed);
    EXPECT_EQ(retriedTask.captureRetryCount, 1U);
    EXPECT_EQ(retriedTask.maximumCaptureRetries, 1U);
    EXPECT_TRUE(retriedTask.firstGeometryCaptureTick.has_value());
    EXPECT_EQ(retriedTask.assignmentHistory.size(), 2U);
    EXPECT_EQ(retriedTask.assignmentHistory[0], "east");
    EXPECT_EQ(retriedTask.assignmentHistory[1], "north");

    nlohmann::json excessiveCaptureRetries = recoveredCamera;
    excessiveCaptureRetries["mission"]["tasks"][0]["max_capture_retries"] = 11;
    EXPECT_THROW(static_cast<void>(RunScenario(excessiveCaptureRetries.dump())),
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
    ASSERT_TRUE(estimatedStartResult.passed);
    ASSERT_EQ(estimatedStartResult.trajectory.size(), reference.size());
    const std::array<double, 5> estimatedStartRmse = calculateRmse(
        reference, estimatedStartResult);

    Json zeroStartScenario = fixture;
    zeroStartScenario["drones"][0]["initial_motor_speeds_radians_per_second"] =
        {0.0, 0.0, 0.0, 0.0};
    const ScenarioResult zeroStartResult = RunScenario(zeroStartScenario.dump());
    ASSERT_TRUE(zeroStartResult.passed);
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
    ASSERT_TRUE(increasedThrustResult.passed);
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
    ASSERT_TRUE(recentFlightResult.passed);
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
    ASSERT_TRUE(freeFlightResult.passed);
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
    ASSERT_TRUE(increasedThrustRecentResult.passed);
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
    ASSERT_TRUE(px4GeometryRecentResult.passed);
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
    ASSERT_TRUE(zeroSpeedRecentResult.passed);
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
    ASSERT_TRUE(motorTableRecentResult.passed);
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
     "expected":{"mode":"following_waypoint","position_meters":[0,0,1.5],
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
    ASSERT_TRUE(lowGain.passed)
        << (lowGain.issues.empty() ? "" : lowGain.issues.front());
    ASSERT_TRUE(highGain.passed)
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
    ASSERT_TRUE(result.passed);
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
     "expected":{"mode":"hovering","position_meters":[0,0,1.5],
       "tolerance_meters":10}}
    )json";
    constexpr std::string_view windScenario = R"json(
    {"version":1,"dynamics":"rigid_body","ticks":1200,
     "wind_velocity_meters_per_second":[2,0,0],
     "commands":[{"tick":0,"type":"takeoff"},
       {"tick":600,"type":"set_waypoint","position_meters":[0,0,1.5]}],
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
    EXPECT_GE(*result.minimumDroneSeparationMeters, 2.0);
    EXPECT_EQ(result.outcome, ScenarioResult::Outcome::Passed);
    ASSERT_TRUE(result.mission.has_value());
    ASSERT_TRUE(result.mission->passed);
    EXPECT_EQ(result.mission->outcome, ScenarioResult::Outcome::Passed);
    EXPECT_EQ(result.mission->completedTaskCount, 4U);
    ASSERT_TRUE(result.mission->startedTick.has_value());
    EXPECT_EQ(*result.mission->startedTick, 90U);
    for (const ScenarioResult::MissionTask& task : result.mission->tasks)
    {
        EXPECT_TRUE(task.completed) << task.inspectionZoneId;
        EXPECT_FALSE(task.inspectionZoneId.empty());
        ASSERT_TRUE(task.firstGeometryCaptureTick.has_value()) << task.inspectionZoneId;
        EXPECT_EQ(task.lastCaptureAssessment, CaptureAssessment::Ready)
            << task.inspectionZoneId;
    }

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
    EXPECT_GE(*physicalResult.minimumDroneSeparationMeters, 2.0);
    EXPECT_GT(physicalResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(physicalResult.mission.has_value());
    EXPECT_EQ(physicalResult.mission->outcome, ScenarioResult::Outcome::Passed);
    for (const ScenarioResult::MissionTask& task : physicalResult.mission->tasks)
    {
        EXPECT_TRUE(task.completed) << task.inspectionZoneId;
        ASSERT_TRUE(task.firstGeometryCaptureTick.has_value()) << task.inspectionZoneId;
        EXPECT_EQ(task.lastCaptureAssessment, CaptureAssessment::Ready)
            << task.inspectionZoneId;
    }
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
    EXPECT_TRUE(result.passed) << (result.issues.empty() ? "" : result.issues.front());
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
    EXPECT_NEAR(result.drones[0].flightTimeSeconds, 175.0 / 60.0, 1.0e-12);
    EXPECT_NEAR(result.drones[1].flightTimeSeconds, 3.0, 1.0e-12);
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

    constexpr std::string_view recoveredMissionScenario = R"json(
    {"version":1,"dynamics":"kinematic","ticks":320,
     "drones":[
       {"name":"scout","initial_position_meters":[0,0,0]},
       {"name":"reserve","initial_position_meters":[10,0,0]},
       {"name":"worker","initial_position_meters":[20,0,0]},
       {"name":"spare","initial_position_meters":[30,0,0]}],
     "commands":[
       {"tick":0,"drone":"scout","type":"takeoff"},
       {"tick":0,"drone":"reserve","type":"takeoff"},
       {"tick":0,"drone":"worker","type":"takeoff"},
       {"tick":0,"drone":"spare","type":"takeoff"}],
     "communication_outages":[
       {"drone":"scout","start_tick":95,"end_tick":250,
        "heartbeat_timeout_ticks":3,"on_timeout":"hold_position"}],
     "mission":{"name":"four-sector-recovered-link-survey","tasks":[
       {"name":"sector-1","role":"survey","drone":"scout",
        "target_position_meters":[1,0,1.5],"start_tick":90,
        "deadline_tick":310,"tolerance_meters":0.1},
       {"name":"sector-2","role":"survey","drone":"reserve",
        "target_position_meters":[11,0,1.5],"start_tick":90,
        "deadline_tick":310,"tolerance_meters":0.1},
       {"name":"sector-3","role":"survey","drone":"worker",
        "target_position_meters":[21,0,1.5],"start_tick":90,
        "deadline_tick":310,"tolerance_meters":0.1},
       {"name":"sector-4","role":"survey","drone":"spare",
        "target_position_meters":[31,0,1.5],"start_tick":90,
        "deadline_tick":310,"tolerance_meters":0.1}]},
     "expected":{"tolerance_meters":1000,"drones":[
       {"name":"scout","mode":"hovering","position_meters":[0,0,0]},
       {"name":"reserve","mode":"hovering","position_meters":[0,0,0]},
       {"name":"worker","mode":"hovering","position_meters":[0,0,0]},
       {"name":"spare","mode":"hovering","position_meters":[0,0,0]}]}}
    )json";
    const ScenarioResult recoveredMissionResult = RunScenario(recoveredMissionScenario);
    ASSERT_TRUE(recoveredMissionResult.passed)
        << (recoveredMissionResult.issues.empty()
            ? "" : recoveredMissionResult.issues.front());
    ASSERT_TRUE(recoveredMissionResult.mission.has_value());
    ASSERT_EQ(recoveredMissionResult.mission->tasks.size(), 4U);
    EXPECT_EQ(recoveredMissionResult.mission->completedTaskCount, 4U);
    EXPECT_DOUBLE_EQ(recoveredMissionResult.mission->completionPercent, 100.0);
    const auto& recoveredTask = recoveredMissionResult.mission->tasks.front();
    EXPECT_TRUE(recoveredTask.completed);
    EXPECT_EQ(recoveredTask.droneName, "reserve");
    EXPECT_EQ(recoveredTask.assignmentHistory,
        (std::vector<std::string>{"scout", "reserve"}));
    const auto recoveredScout = std::find_if(
        recoveredMissionResult.drones.begin(), recoveredMissionResult.drones.end(),
        [](const ScenarioResult::Drone& drone) { return drone.name == "scout"; });
    ASSERT_NE(recoveredScout, recoveredMissionResult.drones.end());
    EXPECT_EQ(recoveredScout->communicationStatus,
        ScenarioResult::Drone::CommunicationStatus::Connected);
    EXPECT_EQ(recoveredScout->lastHeartbeatTick, 320U);
    EXPECT_TRUE(std::any_of(recoveredMissionResult.eventLog.begin(),
        recoveredMissionResult.eventLog.end(), [](const std::string& event)
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
       {"name":"cross","role":"survey","drone":"worker",
        "target_position_meters":[4,0,1.5],"start_tick":100,
        "deadline_tick":590,"tolerance_meters":0.2},
       {"name":"support","role":"survey","drone":"support",
        "target_position_meters":[21,0,1.5],"start_tick":100,
        "deadline_tick":590,"tolerance_meters":0.2}]},
     "expected":{"tolerance_meters":0.5,
       "minimum_drone_separation_meters":2.5,
       "drones":[
         {"name":"worker","mode":"hovering","position_meters":[4,0,1.5]},
         {"name":"stationary","mode":"hovering","position_meters":[0,0,1.5]},
         {"name":"support","mode":"hovering","position_meters":[21,0,1.5]}]}}
    )json";
    const ScenarioResult stationaryDroneResult = RunScenario(stationaryDroneScenario);
    EXPECT_TRUE(stationaryDroneResult.passed)
        << (stationaryDroneResult.issues.empty()
            ? "" : stationaryDroneResult.issues.front())
        << " minimum="
        << stationaryDroneResult.minimumDroneSeparationMeters.value_or(-1.0)
        << " avoidance_events=" << stationaryDroneResult.collisionAvoidanceEvents;
    ASSERT_TRUE(stationaryDroneResult.mission.has_value());
    EXPECT_TRUE(stationaryDroneResult.mission->passed);
    EXPECT_EQ(stationaryDroneResult.mission->completedTaskCount, 2U);
    EXPECT_GT(stationaryDroneResult.collisionAvoidanceEvents, 0U);
    ASSERT_TRUE(stationaryDroneResult.minimumDroneSeparationMeters.has_value());
    EXPECT_GE(*stationaryDroneResult.minimumDroneSeparationMeters, 2.5);

#if !defined(__EMSCRIPTEN__)
    std::string physicalAvoidanceScenario{collisionAvoidanceScenario};
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario, "\"dynamics\":\"kinematic\"",
        "\"dynamics\":\"rigid_body\""));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"ticks\":600", "\"ticks\":1800"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"deadline_tick\":580", "\"deadline_tick\":1790"));
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
        "\"target_position_meters\":[4,0,1.5]",
        "\"target_position_meters\":[8,0,1.5]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"target_position_meters\":[-4,0,1.5]",
        "\"target_position_meters\":[-8,0,1.5]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"position_meters\":[4,0,1.5]",
        "\"position_meters\":[8,0,1.5]"));
    ASSERT_TRUE(ReplaceAll(physicalAvoidanceScenario,
        "\"position_meters\":[-4,0,1.5]",
        "\"position_meters\":[-8,0,1.5]"));
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
    // 강체 응답 지연을 포함해도 X500 충돌 형상보다 넉넉한 간격을 유지해야 한다.
    EXPECT_GE(*physicalAvoidanceResult.minimumDroneSeparationMeters, 5.0);
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
    EXPECT_NEAR(depletedDrone.finalState.positionMeters.x, 0.0, 0.2);
    const auto& healthyDrone = energyResult.drones[1];
    EXPECT_FALSE(healthyDrone.returnedForLowBattery);
    ASSERT_TRUE(healthyDrone.batteryPercent.has_value());
    EXPECT_GT(*healthyDrone.batteryPercent, 40.0);
    EXPECT_NEAR(healthyDrone.finalState.positionMeters.x, 30.0, 0.2);

    const ScenarioResult mixedFaultResult = RunScenario(ReadScenarioFixture(
        "scenarios/multi-drone/energy-and-outage-reallocation.json"));
    ASSERT_TRUE(mixedFaultResult.passed)
        << (mixedFaultResult.issues.empty() ? "" : mixedFaultResult.issues.front());
    ASSERT_TRUE(mixedFaultResult.mission.has_value());
    EXPECT_EQ(mixedFaultResult.mission->outcome, ScenarioResult::Outcome::Passed);
    EXPECT_EQ(mixedFaultResult.mission->completedTaskCount, 3U);
    ASSERT_EQ(mixedFaultResult.mission->tasks.size(), 3U);

    const auto findTask = [&](std::string_view name)
        -> const ScenarioResult::MissionTask*
    {
        const auto task = std::find_if(
            mixedFaultResult.mission->tasks.begin(),
            mixedFaultResult.mission->tasks.end(),
            [name](const ScenarioResult::MissionTask& item)
            {
                return item.name == name;
            });
        return task == mixedFaultResult.mission->tasks.end() ? nullptr : &*task;
    };
    const auto* lowBatteryTask = findTask("long-area");
    ASSERT_NE(lowBatteryTask, nullptr);
    EXPECT_EQ(lowBatteryTask->initialDroneName, "low-charge");
    EXPECT_EQ(lowBatteryTask->droneName, "reserve-2");
    EXPECT_EQ(lowBatteryTask->assignmentHistory,
        (std::vector<std::string>{"low-charge", "reserve-2"}));
    const auto* unconfirmedTask = findTask("unconfirmed-area");
    ASSERT_NE(unconfirmedTask, nullptr);
    EXPECT_EQ(unconfirmedTask->initialDroneName, "lost-link");
    EXPECT_EQ(unconfirmedTask->droneName, "reserve-1");
    EXPECT_TRUE(unconfirmedTask->completed);
    EXPECT_FALSE(unconfirmedTask->failed);
    EXPECT_EQ(unconfirmedTask->assignmentHistory,
        (std::vector<std::string>{"lost-link", "reserve-1"}));
    const auto* displacedTask = findTask("near-area");
    ASSERT_NE(displacedTask, nullptr);
    EXPECT_EQ(displacedTask->droneName, "reserve-2");
    EXPECT_EQ(displacedTask->assignmentHistory,
        (std::vector<std::string>{"reserve-1", "reserve-2"}));

    ASSERT_EQ(mixedFaultResult.drones.size(), 4U);
    const auto& lowBatteryDrone = mixedFaultResult.drones[0];
    EXPECT_TRUE(lowBatteryDrone.returnedForLowBattery);
    EXPECT_EQ(lowBatteryDrone.finalState.mode, FlightMode::Grounded);
    const auto& disconnectedDrone = mixedFaultResult.drones[1];
    EXPECT_EQ(disconnectedDrone.communicationStatus,
        ScenarioResult::Drone::CommunicationStatus::Unknown);
    EXPECT_EQ(disconnectedDrone.lastHeartbeatTick, 94U);
    // 통신 단절 기체의 위치는 마지막 수신값이므로 실제 현재 상태로 단정하지 않는다.
    EXPECT_EQ(disconnectedDrone.finalState.mode, FlightMode::FollowingWaypoint);
    EXPECT_NEAR(disconnectedDrone.finalState.positionMeters.x, 10.3333333333, 1.0e-6);
    ASSERT_TRUE(disconnectedDrone.batteryPercent.has_value());
    ASSERT_TRUE(disconnectedDrone.reportedBatteryPercent.has_value());
    EXPECT_GT(*disconnectedDrone.reportedBatteryPercent,
        *disconnectedDrone.batteryPercent);

    struct ReplayCondition
    {
        int reservePercent;
        int outageStartTick;
        int heartbeatTimeoutTicks;
        int expectedCompletionTick;
    };
    constexpr std::array<ReplayCondition, 4> replayConditions{{
        {40, 95, 3, 322},
        {30, 95, 3, 376},
        {40, 115, 8, 217},
        {50, 95, 8, 171}}};
    const auto baseMixedFaultScenario = nlohmann::json::parse(ReadScenarioFixture(
        "scenarios/multi-drone/energy-and-outage-reallocation.json"));
    for (const ReplayCondition& condition : replayConditions)
    {
        auto scenarioVariant = baseMixedFaultScenario;
        scenarioVariant["energy_model"]["reserve_percent"] = condition.reservePercent;
        scenarioVariant["communication_outages"][0]["start_tick"] =
            condition.outageStartTick;
        scenarioVariant["communication_outages"][0]["heartbeat_timeout_ticks"] =
            condition.heartbeatTimeoutTicks;
        scenarioVariant["expected"]["tolerance_meters"] = 1000.0;

        for (int repeat = 0; repeat < 3; ++repeat)
        {
            const ScenarioResult repeatedResult = RunScenario(scenarioVariant.dump());
            ASSERT_TRUE(repeatedResult.passed)
                << (repeatedResult.issues.empty() ? "" : repeatedResult.issues.front());
            ASSERT_TRUE(repeatedResult.mission.has_value());
            EXPECT_EQ(repeatedResult.mission->completedTaskCount, 3U);
            ASSERT_TRUE(repeatedResult.mission->completedTick.has_value());
            EXPECT_EQ(*repeatedResult.mission->completedTick,
                static_cast<std::uint64_t>(condition.expectedCompletionTick));
            ASSERT_EQ(repeatedResult.drones.size(), 4U);
            EXPECT_TRUE(repeatedResult.drones[0].returnedForLowBattery);
            EXPECT_EQ(repeatedResult.drones[1].communicationStatus,
                ScenarioResult::Drone::CommunicationStatus::Unknown);
            EXPECT_EQ(repeatedResult.drones[1].lastHeartbeatTick,
                static_cast<std::uint64_t>(condition.outageStartTick - 1));
        }
    }

#if !defined(__EMSCRIPTEN__)
    // 강체 비행의 응답 지연을 고려해 계획 간격은 2.1m로 두고, 실제 간격이 2m를 지키는지 본다.
    const std::string rigidBodyMixedFaultScenario = ReadScenarioFixture(
        "scenarios/multi-drone/rigid-body-energy-and-outage.json");
    for (int repeat = 0; repeat < 3; ++repeat)
    {
        const ScenarioResult rigidBodyResult = RunScenario(rigidBodyMixedFaultScenario);
        ASSERT_TRUE(rigidBodyResult.passed)
            << (rigidBodyResult.issues.empty() ? "" : rigidBodyResult.issues.front());
        ASSERT_TRUE(rigidBodyResult.mission.has_value());
        EXPECT_EQ(rigidBodyResult.mission->outcome, ScenarioResult::Outcome::Passed);
        EXPECT_EQ(rigidBodyResult.mission->completedTaskCount, 3U);
        ASSERT_TRUE(rigidBodyResult.mission->startedTick.has_value());
        EXPECT_GT(*rigidBodyResult.mission->startedTick, 90U);
        EXPECT_LE(*rigidBodyResult.mission->startedTick, 512U);
        ASSERT_TRUE(rigidBodyResult.mission->completedTick.has_value());
        EXPECT_LE(*rigidBodyResult.mission->completedTick, 1200U);
        ASSERT_EQ(rigidBodyResult.mission->tasks.size(), 3U);
        ASSERT_TRUE(rigidBodyResult.mission->tasks[0].completedTick.has_value());
        EXPECT_LE(*rigidBodyResult.mission->tasks[0].completedTick, 512U);
        EXPECT_EQ(rigidBodyResult.mission->tasks[0].assignmentHistory,
            (std::vector<std::string>{"low-charge", "reserve-2"}));
        ASSERT_TRUE(rigidBodyResult.mission->tasks[1].completedTick.has_value());
        EXPECT_LE(*rigidBodyResult.mission->tasks[1].completedTick, 1200U);
        EXPECT_EQ(rigidBodyResult.mission->tasks[1].assignmentHistory,
            (std::vector<std::string>{"lost-link", "reserve-2"}));
        EXPECT_GT(rigidBodyResult.collisionAvoidanceEvents, 0U);
        ASSERT_TRUE(rigidBodyResult.minimumDroneSeparationMeters.has_value());
        EXPECT_GE(*rigidBodyResult.minimumDroneSeparationMeters, 2.0);
        ASSERT_EQ(rigidBodyResult.drones.size(), 4U);
        EXPECT_TRUE(rigidBodyResult.drones[0].returnedForLowBattery);
        EXPECT_EQ(rigidBodyResult.drones[0].finalState.mode, FlightMode::Grounded);
        ASSERT_TRUE(rigidBodyResult.drones[0].batteryPercent.has_value());
        EXPECT_GT(*rigidBodyResult.drones[0].batteryPercent, 0.0);
        EXPECT_LT(*rigidBodyResult.drones[0].batteryPercent, 50.0);
        EXPECT_EQ(rigidBodyResult.drones[1].communicationStatus,
            ScenarioResult::Drone::CommunicationStatus::Unknown);
        EXPECT_EQ(rigidBodyResult.drones[1].lastHeartbeatTick, 179U);
        EXPECT_NEAR(rigidBodyResult.drones[1].finalState.positionMeters.x,
            10.0, 0.01);
        EXPECT_TRUE(std::any_of(rigidBodyResult.eventLog.begin(),
            rigidBodyResult.eventLog.end(), [](const std::string& event)
            {
                return event.find("tick 185: lost-link 드론의 상태를 확인할 수 없어")
                    != std::string::npos;
            }));
    }
#endif

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

void Verify_ScenarioRunner_CompletesTasksAfterSimultaneousOutage()
{
    using Json = nlohmann::json;
    constexpr std::uint64_t missionStartTick = 90U;
    constexpr std::uint64_t missionDeadlineTick = missionStartTick + 600U;
    constexpr std::uint64_t scenarioTicks = missionDeadlineTick + 30U;
    constexpr std::array droneCounts{4U, 16U, 64U, 256U};
    for (const std::size_t droneCount : droneCounts)
    {
        Json drones = Json::array();
        Json commands = Json::array();
        Json tasks = Json::array();
        Json expectedDrones = Json::array();
        Json outages = Json::array();
        const std::size_t outageCount = droneCount / 4U;
        for (std::size_t index = 0; index < droneCount; ++index)
        {
            const std::string name = "drone-" + std::to_string(index);
            const double startX = static_cast<double>(index) * 5.0;
            drones.push_back({
                {"name", name},
                {"initial_position_meters", {startX, 0.0, 0.0}}});
            commands.push_back({{"tick", 0}, {"drone", name}, {"type", "takeoff"}});
            tasks.push_back({
                {"name", "task-" + std::to_string(index)},
                {"role", "survey"},
                {"drone", name},
                {"target_position_meters", {startX + 0.5, 0.0, 1.5}},
                {"start_tick", missionStartTick},
                {"deadline_tick", missionDeadlineTick},
                {"tolerance_meters", 0.05}});
            expectedDrones.push_back({
                {"name", name},
                {"mode", "hovering"},
                {"position_meters", {startX + 0.5, 0.0, 1.5}}});
            if (index % 4U == 0U)
            {
                outages.push_back({
                    {"drone", name},
                    {"start_tick", missionStartTick - 5U},
                    {"end_tick", scenarioTicks},
                    {"heartbeat_timeout_ticks", 3U},
                    {"on_timeout", "hold_position"}});
            }
        }

        const Json scenario{
            {"version", 1},
            {"dynamics", "kinematic"},
            {"ticks", scenarioTicks},
            {"drones", std::move(drones)},
            {"commands", std::move(commands)},
            {"communication_outages", std::move(outages)},
            {"mission", {{"name", "reallocation"}, {"tasks", std::move(tasks)}}},
            {"expected", {
                {"tolerance_meters", 10'000.0},
                {"drones", std::move(expectedDrones)}}}};

        const ScenarioResult result = RunScenario(scenario.dump());
        ASSERT_TRUE(result.passed) << "drone_count=" << droneCount << ": "
            << (result.issues.empty() ? "임무가 완료되지 않았습니다." : result.issues.front());
        ASSERT_TRUE(result.mission.has_value());
        EXPECT_TRUE(result.mission->passed) << "drone_count=" << droneCount;
        EXPECT_EQ(result.mission->completedTaskCount, droneCount)
            << "drone_count=" << droneCount;
        const std::size_t reassignedTaskCount = static_cast<std::size_t>(std::count_if(
            result.mission->tasks.begin(), result.mission->tasks.end(),
            [](const ScenarioResult::MissionTask& task)
            {
                return task.reassignmentCount > 0U;
            }));
        EXPECT_GE(reassignedTaskCount, outageCount) << "drone_count=" << droneCount;
    }
}

TEST(ScenarioRunnerTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_RecordsBoundedTimeAlignedTrajectories());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_ControlsAndReportsCamerasPerDrone());
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_RequiresCameraGeometryForInspectionTasks());
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
    EXPECT_NO_FATAL_FAILURE(
        Verify_ScenarioRunner_CompletesTasksAfterSimultaneousOutage());

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
