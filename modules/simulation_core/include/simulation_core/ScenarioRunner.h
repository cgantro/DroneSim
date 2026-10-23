#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aerosimlab::simulation
{
/**
 * @brief JSON 시나리오 실행의 최종 상태와 시험 결과를 돌려준다.
 * @details 통과 여부는 명령 처리와 마지막 상태·위치·선택한 자세 기대값으로 정한다.
 */
struct ScenarioResult
{
    bool passed = false;
    std::uint64_t tickCount = 0;
    double simulationTimeSeconds = 0.0;
    DroneState finalState;
    CameraState finalCamera;
    std::vector<std::string> issues;
    std::vector<std::string> eventLog;
    /** @brief 기준을 요청한 시나리오에서 잰 최소 3차원 중심 간 거리다. */
    std::optional<double> minimumDroneSeparationMeters;
    /** @brief 예측 회피를 켠 시나리오에서 일정 시간 앞의 최소 예상 거리다. */
    std::optional<double> minimumPredictedDroneSeparationMeters;
    /** @brief 예측 충돌 위험으로 회피 경로를 처음 적용한 횟수다. */
    std::uint64_t collisionAvoidanceEvents = 0;
    /** @brief 강체 실행에 실제 사용한 기체 값으로, 시험 결과의 조건을 다시 확인하는 데 쓴다. */
    struct PhysicalModel
    {
        double massKilograms = 0.0;
        glm::dvec3 principalInertiaKilogramMetersSquared{0.0};
        double thrustCoefficientNewtonsPerSpeedSquared = 0.0;
        double momentToThrustRatio = 0.0;
        double motorTimeConstantUpSeconds = 0.0;
        double motorTimeConstantDownSeconds = 0.0;
        double maximumMotorSpeedRadiansPerSecond = 0.0;
        bool customized = false;
    };
    std::optional<PhysicalModel> physicalModel;
    /** @brief 강체 시나리오에서 사용한 제어기 이득이다. */
    struct ControllerSettings
    {
        double positionGain = 0.0;
        double velocityGain = 0.0;
        double attitudeGain = 0.0;
        double angularRateGain = 0.0;
        bool customized = false;
    };
    std::optional<ControllerSettings> controllerSettings;
    /** @brief 시나리오에서 이름으로 지정한 드론의 마지막 상태다. */
    struct Drone
    {
        std::string name;
        DroneState finalState;
        enum class CommunicationStatus { Connected, Stale, Unknown } communicationStatus =
            CommunicationStatus::Connected;
        /** @brief finalState가 실제 현재 위치인지, 마지막으로 전달된 위치인지 나타낸다. */
        std::uint64_t lastHeartbeatTick = 0;
        /** @brief 에너지 모형을 켠 시나리오에서 설정한 배터리 용량과 사용·잔량이다. */
        std::optional<double> batteryCapacityWattHours;
        /** @brief 시작 잔량에서 시나리오 동안 실제로 소비한 에너지다. */
        std::optional<double> energyConsumedWattHours;
        std::optional<double> batteryRemainingWattHours;
        std::optional<double> batteryPercent;
        /** @brief 조정기에 마지막으로 전달된 배터리 비율이다. 통신 단절 중에는 오래된 값일 수 있다. */
        std::optional<double> reportedBatteryPercent;
        /** @brief 배터리 예비량에 도달해 임무를 넘기고 착륙을 시작했는지 나타낸다. */
        bool returnedForLowBattery = false;
    };
    /** @brief 시나리오에 등장한 드론의 마지막 상태를 입력 순서대로 보관한다. */
    std::vector<Drone> drones;

    /** @brief 시나리오에서 맡은 임무 하나와 완료 결과다. */
    struct MissionTask
    {
        std::string name;
        std::string role;
        std::string droneName;
        std::string initialDroneName;
        std::vector<std::string> assignmentHistory;
        std::uint32_t reassignmentCount = 0;
        glm::dvec3 targetPositionMeters{0.0};
        /** @brief 점 임무는 한 점, 구역 임무는 왕복 경로의 순서 있는 지점 목록이다. */
        std::vector<glm::dvec3> waypointsMeters;
        std::size_t currentWaypointIndex = 0;
        std::uint64_t startTick = 0;
        std::uint64_t deadlineTick = 0;
        double toleranceMeters = 0.0;
        bool started = false;
        bool completed = false;
        bool failed = false;
        std::optional<std::uint64_t> completedTick;
    };
    /** @brief 모든 임무가 기한 안에 끝났는지와 역할별 결과다. */
    struct Mission
    {
        std::string name;
        bool passed = false;
        /** @brief 완료한 작업 수와 전체 작업 중 완료한 비율이다. */
        std::size_t completedTaskCount = 0;
        double completionPercent = 0.0;
        /** @brief 방문한 경유점 수를 전체 경유점 수로 나눈 비율이다. */
        double coveragePercent = 0.0;
        /** @brief 첫 임무를 실제로 시작한 Tick과 모든 임무를 끝낸 Tick이다. */
        std::optional<std::uint64_t> startedTick;
        std::optional<std::uint64_t> completedTick;
        std::vector<MissionTask> tasks;
    };
    /** @brief mission을 지정한 시나리오에만 결과가 들어간다. */
    std::optional<Mission> mission;
};

/**
 * @brief JSON 문자열에 적힌 명령을 고정 tick에 적용하고 예상 결과와 비교한다.
 * @details 형식 버전은 1이다. 선택한 드론 목록이 있으면 이름별 위치와 명령을 시험하고, 없으면 기본 드론 한 대를 실행한다.
 * formation_links는 위치 공유 장애를, mission은 드론별 역할·목표·기한과 완료 판정을 다룬다.
 * 잘못된 문법이나 빠진 필수 값은 std::invalid_argument로 알린다.
 */
[[nodiscard]] ScenarioResult RunScenario(std::string_view jsonText);

}
