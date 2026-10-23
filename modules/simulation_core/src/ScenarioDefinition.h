#pragma once

#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aerosimlab::simulation::detail
{
struct ExpectedDrone
{
    std::string name;
    FlightMode mode;
    glm::dvec3 positionMeters;
    std::optional<glm::dquat> orientation;
};

struct ScheduledCommand
{
    std::uint64_t tick;
    std::uint64_t sendTick;
    std::uint64_t delayTicks;
    std::uint64_t sequence;
    std::string droneName;
    DroneCommand command;
    bool dropped;
    bool duplicated;
};

struct ScenarioDrone
{
    std::string name;
    glm::dvec3 initialPositionMeters;
    double initialYawRadians;
    glm::dvec3 initialVelocityMetersPerSecond{0.0};
    std::optional<glm::dquat> initialOrientation;
    glm::dvec3 initialAngularVelocityRadiansPerSecond{0.0};
    std::array<double, QuadrotorParameters::rotorCount> initialMotorSpeedsRadiansPerSecond{};
    FlightMode initialMode = FlightMode::Grounded;
    bool hasFullInitialState = false;
    std::optional<double> batteryCapacityWattHours;
    double initialBatteryPercent = 100.0;
};

struct FormationOutage
{
    std::uint64_t startTick;
    std::uint64_t endTick;
};

struct FormationLink
{
    std::string leaderName;
    std::string followerName;
    glm::dvec3 offsetMeters;
    std::vector<FormationOutage> outages;
};

struct DroneCommunicationOutage
{
    std::string droneName;
    std::uint64_t startTick;
    std::uint64_t endTick;
    std::uint64_t heartbeatTimeoutTicks;
};

struct CollisionAvoidanceSettings
{
    double safetyDistanceMeters;
    double predictionHorizonSeconds;
    double lateralOffsetMeters;
};

struct EnergyModelSettings
{
    double capacityWattHours;
    double hoverPowerWatts;
    double flightPowerWatts;
    double reservePercent;
};

/** @brief JSON 카메라 동작을 실행 단계에서 안전하게 구분한다. */
enum class CameraEventAction
{
    Enable,
    Disable,
    Fault,
    Recover
};

struct ScheduledCameraEvent
{
    std::uint64_t tick;
    CameraEventAction action;
};

/** @brief JSON 시나리오를 실행하기 전에 읽고 검증한 값이다. */
struct ScenarioDefinition
{
    std::uint64_t tickCount = 0;
    std::vector<ScenarioDrone> drones;
    std::string primaryDroneName;
    std::vector<ScheduledCommand> commands;
    std::vector<FormationLink> formationLinks;
    std::vector<DroneCommunicationOutage> communicationOutages;
    std::optional<CollisionAvoidanceSettings> collisionAvoidance;
    std::optional<EnergyModelSettings> energyModel;
    std::optional<ScenarioResult::Mission> mission;
    SimulationDynamics dynamics = defaultSimulationDynamics;
    QuadrotorParameters quadrotorParameters =
        QuadrotorParameters::X500MotorTableEstimate();
    bool quadrotorParametersCustomized = false;
    QuadrotorControllerParameters controllerParameters;
    bool controllerParametersCustomized = false;
    std::optional<glm::dvec3> windVelocity;
    /** @brief 궤적 저장을 요청한 경우의 샘플 간격이다. */
    std::optional<std::uint64_t> trajectorySampleEveryTicks;
    std::vector<ScheduledCameraEvent> cameraEvents;
    std::vector<ExpectedDrone> expectedDrones;
    bool hasExpectedDrones = false;
    double toleranceMeters = 0.01;
    double orientationToleranceRadians = 0.01;
    std::optional<double> minimumDroneSeparationMeters;
    std::optional<CameraStatus> expectedCameraStatus;
    std::optional<std::uint64_t> expectedFrameCount;
};

/** @brief JSON 설정을 읽고, 실행에 필요한 값만 묶어 돌려준다. */
[[nodiscard]] ScenarioDefinition ReadScenarioDefinition(std::string_view jsonText);
}
