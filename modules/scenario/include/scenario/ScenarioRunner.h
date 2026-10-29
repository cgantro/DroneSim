#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace aerosimlab::scenario
{
/**
 * @brief JSON 시나리오 실행의 최종 상태와 시험 결과를 돌려준다.
 * @details 통신 단절로 임무가 남은 경우 실패로 단정하지 않고 미확정으로 기록한다.
 */
struct ScenarioResult
{
    /** @brief 모든 기준을 만족하면 Passed, 명시적 오류면 Failed, 근거가 부족하면 Inconclusive다. */
    enum class Outcome { Passed, Failed, Inconclusive };
    Outcome outcome = Outcome::Inconclusive;
    /** @brief 결과 상태가 Passed인지 확인한다. */
    [[nodiscard]] bool IsPassed() const noexcept { return outcome == Outcome::Passed; }
    std::uint64_t tickCount = 0;
    double simulationTimeSeconds = 0.0;
    /** @brief 시나리오에서 강체 물리에 적용한 ENU 기준 바람 속도다. */
    std::optional<glm::dvec3> windVelocityMetersPerSecond;
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
    /** @brief 요청한 시나리오에서 일정 간격으로 저장한 드론 궤적이다. */
    struct TrajectorySample
    {
        std::uint64_t tick = 0;
        double timeSeconds = 0.0;
        std::vector<std::pair<std::string, simulation::DroneState>> drones;
    };
    std::vector<TrajectorySample> trajectory;
    /** @brief 시나리오에서 이름으로 지정한 드론의 마지막 상태다. */
    struct Drone
    {
        std::string name;
        simulation::DroneState finalState;
        enum class CommunicationStatus { Connected, Stale, Unknown } communicationStatus =
            CommunicationStatus::Connected;
        /** @brief finalState가 실제 현재 위치인지, 마지막으로 전달된 위치인지 나타낸다. */
        std::uint64_t lastHeartbeatTick = 0;
        simulation::CameraState finalCamera;
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
        /** @brief 매 고정 Tick의 위치 변화를 더한 3차원 이동거리다. 단위는 미터다. */
        double flightDistanceMeters = 0.0;
        /** @brief Grounded가 아닌 비행 상태였던 누적 모의 시간이다. 단위는 초다. */
        double flightTimeSeconds = 0.0;
    };
    /** @brief 시나리오에 등장한 드론의 마지막 상태를 입력 순서대로 보관한다. 기본 드론도 포함한다. */
    std::vector<Drone> drones;

    /** @brief 시나리오에서 맡은 임무 하나와 완료 결과다. */
    struct MissionTask
    {
        std::string name;
        std::string droneName;
        /** @brief 점 임무는 한 점, 구역 임무는 왕복 경로의 순서 있는 지점 목록이다. */
        std::vector<glm::dvec3> waypointsMeters;
        std::size_t currentWaypointIndex = 0;
        std::uint64_t startTick = 0;
        std::uint64_t deadlineTick = 0;
        double toleranceMeters = 0.0;
        bool started = false;
        bool failed = false;
        std::optional<std::uint64_t> completedTick;

        /** @brief 완료 Tick이 기록됐는지 확인한다. */
        [[nodiscard]] bool IsCompleted() const noexcept { return completedTick.has_value(); }
    };
    /** @brief 임무의 완료·실패·미확정 결과와 역할별 진행 상태다. */
    struct Mission
    {
        std::string name;
        /** @brief 작업 상태에서 전체 임무 결과를 계산한다. */
        [[nodiscard]] Outcome GetOutcome() const noexcept;
        /** @brief 모든 작업이 완료됐는지 확인한다. 실패와 미확정은 outcome에 남는다. */
        [[nodiscard]] bool IsPassed() const noexcept
        {
            return GetOutcome() == Outcome::Passed;
        }
        /** @brief 완료한 작업 수를 작업 상태에서 계산한다. */
        [[nodiscard]] std::size_t CompletedTaskCount() const noexcept
        {
            std::size_t count = 0;
            for (const MissionTask& task : tasks)
            {
                if (task.IsCompleted())
                    ++count;
            }
            return count;
        }
        /** @brief 방문한 경유점 비율을 작업별 진행 상태에서 계산한다. */
        [[nodiscard]] double WaypointProgressPercent() const noexcept
        {
            std::size_t total = 0;
            std::size_t visited = 0;
            for (const MissionTask& task : tasks)
            {
                total += task.waypointsMeters.size();
                visited += task.IsCompleted()
                    ? task.waypointsMeters.size() : task.currentWaypointIndex;
            }
            return total == 0 ? 0.0
                : 100.0 * static_cast<double>(visited) / static_cast<double>(total);
        }
        /** @brief 모든 작업이 끝났을 때 가장 늦게 완료한 Tick을 돌려준다. */
        [[nodiscard]] std::optional<std::uint64_t> CompletedTick() const noexcept
        {
            if (tasks.empty())
                return std::nullopt;
            std::optional<std::uint64_t> latestTick;
            for (const MissionTask& task : tasks)
            {
                if (!task.completedTick)
                    return std::nullopt;
                if (!latestTick || *task.completedTick > *latestTick)
                    latestTick = task.completedTick;
            }
            return latestTick;
        }
        /** @brief 첫 임무가 시작된 Tick이다. */
        std::optional<std::uint64_t> startedTick;
        std::vector<MissionTask> tasks;
    };
    /** @brief mission을 지정한 시나리오에만 결과가 들어간다. */
    std::optional<Mission> mission;
};

/** @brief 결과 상태를 JSON과 사용자 안내에 쓸 짧은 영문 이름으로 바꾼다. */
[[nodiscard]] std::string_view ToString(ScenarioResult::Outcome outcome) noexcept;

/**
 * @brief JSON 문자열에 적힌 명령을 고정 tick에 적용하고 예상 결과와 비교한다.
 * @details 형식 버전은 1이다. 선택한 드론 목록이 있으면 이름별 위치와 명령을 시험하고, 없으면 기본 드론 한 대를 실행한다.
 * communication_outages는 상태 보고 단절을, mission은 드론별 작업·경유점·기한과 완료 판정을 다룬다.
 * 잘못된 문법이나 빠진 필수 값은 std::invalid_argument로 알린다.
 */
[[nodiscard]] ScenarioResult RunScenario(std::string_view jsonText);

}
