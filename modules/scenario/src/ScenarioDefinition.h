#pragma once

#include "simulation_core/BatteryModel.h"
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "scenario/ScenarioRunner.h"
#include "simulation_core/SimulationCore.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aerosimlab::scenario::detail
{
using namespace aerosimlab::simulation;

struct ExpectedDrone
{
    std::string name;
    FlightMode mode;
    glm::dvec3 positionMeters;
    std::optional<glm::dquat> orientation;
};

struct TickCommand
{
    std::uint64_t tick;
    std::string droneName;
    DroneCommand command;
};

struct ScenarioDrone
{
    std::string name;
    DroneState initialState;
    /** @brief 위치·방향 외의 시작 상태를 JSON에서 지정했는지 나타낸다. */
    bool hasExtendedInitialState = false;
    std::optional<double> batteryCapacityWattHours;
    double initialBatteryPercent = 100.0;
    std::optional<CameraModel> cameraModel;
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

using EnergyModelSettings = BatteryModelParameters;

/** @brief JSON 시나리오를 실행하기 전에 읽고 검증한 값이다. */
struct ScenarioDefinition
{
    std::uint64_t tickCount = 0;
    std::vector<ScenarioDrone> drones;
    std::string primaryDroneName;
    CameraModel defaultCameraModel;
    std::vector<TickCommand> commands;
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
    std::vector<ExpectedDrone> expectedDrones;
    double toleranceMeters = 0.01;
    double orientationToleranceRadians = 0.01;
    std::optional<double> minimumDroneSeparationMeters;
    std::optional<CameraStatus> expectedCameraStatus;
    std::optional<std::uint64_t> expectedFrameCount;
};

/** @brief JSON 설정을 읽고, 실행에 필요한 값만 묶어 돌려준다. */
[[nodiscard]] ScenarioDefinition ReadScenarioDefinition(std::string_view jsonText);
}
