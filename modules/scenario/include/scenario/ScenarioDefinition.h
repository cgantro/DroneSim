#pragma once

#include "simulation_core/BatteryModel.h"
#include "simulation_core/EoCameraModel.h"
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/QuadrotorFlightController.h"
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
    /** @brief 최종 상태를 비교할 드론 이름이다. */
    std::string name;
    /** @brief 시험이 끝났을 때 기대하는 비행 상태다. */
    FlightMode mode;
    /** @brief 기대 위치를 미터 단위 ENU 좌표로 적는다. */
    glm::dvec3 positionMeters;
    /** @brief 비교할 자세가 있을 때만 w, x, y, z 순서의 회전을 보관한다. */
    std::optional<glm::dquat> orientation;
};

struct TickCommand
{
    /** @brief 명령을 적용할 시뮬레이션 Tick 번호다. 첫 Tick은 0이다. */
    std::uint64_t tick;
    /** @brief 명령을 받을 시나리오 안의 드론 이름이다. */
    std::string droneName;
    /** @brief SimulationCore가 이해하는 비행 명령이다. */
    DroneCommand command;
};

struct ScenarioDrone
{
    /** @brief 시나리오에서 이 기체를 가리킬 고유 이름이다. */
    std::string name;
    /** @brief 위치·자세·속도·비행 상태 등 실행을 시작할 때의 값이다. */
    DroneState initialState;
    /** @brief 위치·방향 외의 시작 상태를 JSON에서 지정했는지 나타낸다. */
    bool hasExtendedInitialState = false;
    /** @brief 에너지 모형을 쓸 때 기본 용량을 덮어쓸 값이며 단위는 Wh다. */
    std::optional<double> batteryCapacityWattHours;
    /** @brief 에너지 모형을 시작할 충전 비율이다. 범위는 0~100%다. */
    double initialBatteryPercent = 100.0;
    /** @brief 기본 카메라 대신 이 기체에 사용할 카메라 설정이다. */
    std::optional<CameraModel> cameraModel;
};

struct DroneCommunicationOutage
{
    /** @brief 상태 보고를 끊을 드론 이름이다. */
    std::string droneName;
    /** @brief 통신 단절이 시작되는 Tick이다. 이 Tick부터 단절로 처리한다. */
    std::uint64_t startTick;
    /** @brief 통신 단절이 끝나는 Tick이다. 이 Tick부터 보고가 복구된다. */
    std::uint64_t endTick;
    /** @brief 마지막 하트비트 뒤 이 Tick 수가 지나면 상태를 미확정으로 바꾼다. */
    std::uint64_t heartbeatTimeoutTicks;
};

struct CollisionAvoidanceSettings
{
    /** @brief 이보다 가까워질 것으로 예측되면 회피를 시작한다. 단위는 미터다. */
    double safetyDistanceMeters;
    /** @brief 이 시간까지 두 기체의 상대 위치를 예측한다. 단위는 초다. */
    double predictionHorizonSeconds;
    /** @brief 충돌 지점 옆으로 잡을 회피 경로의 최소 이동 거리다. 단위는 미터다. */
    double lateralOffsetMeters;
};

using EnergyModelSettings = BatteryModelParameters;

/** @brief JSON 임무 한 건에 적힌 입력값이다. 실행 중 진행 상태는 담지 않는다. */
struct ScenarioMissionTask
{
    std::string name;
    std::string droneName;
    std::vector<glm::dvec3> waypointsMeters;
    std::uint64_t startTick = 0;
    std::uint64_t deadlineTick = 0;
    double toleranceMeters = 0.0;
};

/** @brief JSON에 적힌 임무 이름과 작업 목록이다. */
struct ScenarioMission
{
    std::string name;
    std::vector<ScenarioMissionTask> tasks;
};

/**
 * @brief JSON 시나리오를 실행 자료형으로 바꾼 설정 묶음이다.
 * @details 이 구조체는 실행 중 바뀌는 임무 진행 상태를 담지 않는다.
 * 파서는 JSON 값을 옮기고, 검증기는 이 설정의 값 범위와 조합을 확인한다.
 */
struct ScenarioDefinition
{
    /** @brief 실행할 고정 Tick 수다. 한 Tick은 DroneModel의 고정 시간 간격이다. */
    std::uint64_t tickCount = 0;
    /** @brief 이름별 초기 상태를 지정한 드론 목록이다. */
    std::vector<ScenarioDrone> drones;
    /** @brief JSON에서 드론 목록을 생략해 SimulationCore의 기본 기체를 쓰는지 나타낸다. */
    bool usesExistingPrimaryDrone = false;
    /** @brief 개별 드론 카메라를 지정하지 않았을 때 적용할 기본 카메라다. */
    CameraModel defaultCameraModel;
    /** @brief 시각 순으로 적용할 Tick별 외부 조종 명령이다. */
    std::vector<TickCommand> commands;
    /** @brief 지정한 드론의 상태 보고만 끊는 가상 통신 장애다. */
    std::vector<DroneCommunicationOutage> communicationOutages;
    /** @brief 설정된 경우 임무 중 드론 사이 충돌 위험을 예측한다. */
    std::optional<CollisionAvoidanceSettings> collisionAvoidance;
    /** @brief 설정된 경우 비행 중 전력 소모와 저전력 귀환을 계산한다. */
    std::optional<EnergyModelSettings> energyModel;
    /** @brief 설정된 경우 드론별 경유점 임무를 실행한다. */
    std::optional<ScenarioMission> mission;
    /** @brief 운동학 또는 강체 물리 중 시나리오가 사용할 비행 계산 방식이다. */
    SimulationDynamics dynamics = defaultSimulationDynamics;
    QuadrotorParameters quadrotorParameters =
        QuadrotorParameters::X500MotorTableEstimate();
    bool quadrotorParametersCustomized = false;
    QuadrotorControllerParameters controllerParameters;
    bool controllerParametersCustomized = false;
    /** @brief ENU 축 기준 바람 속도(m/s)다. */
    std::optional<glm::dvec3> windVelocity;
    /** @brief 궤적 저장을 요청한 경우의 샘플 간격이다. */
    std::optional<std::uint64_t> trajectorySampleEveryTicks;
    /** @brief 실행 뒤 비교할 드론별 기대 상태 목록이다. */
    std::vector<ExpectedDrone> expectedDrones;
    /** @brief 기대 위치와 실제 위치 사이에 허용할 거리 오차다. 단위는 미터다. */
    double toleranceMeters = 0.01;
    /** @brief 기대 자세와 실제 자세 사이에 허용할 각도 오차다. 단위는 라디안이다. */
    double orientationToleranceRadians = 0.01;
    /** @brief 실행 중 기록한 최소 거리가 이 값보다 작으면 시험을 실패시킨다. 단위는 미터다. */
    std::optional<double> minimumDroneSeparationMeters;
    /** @brief 실행 뒤 비교할 기본 드론 카메라 상태다. */
    std::optional<CameraStatus> expectedCameraStatus;
    /** @brief 실행 뒤 비교할 기본 드론의 카메라 프레임 수다. */
    std::optional<std::uint64_t> expectedFrameCount;
};

/** @brief JSON 형식을 확인해 실행 설정으로 변환한다. */
[[nodiscard]] ScenarioDefinition ParseScenarioDefinition(std::string_view jsonText);

/** @brief 설정 값의 허용 범위와 항목 사이의 규칙을 확인한다. */
void ValidateScenarioDefinition(const ScenarioDefinition& definition);
}
