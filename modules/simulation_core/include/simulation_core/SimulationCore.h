#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"
#include "simulation_core/BatteryModel.h"
#include "terrain/TerrainGrid.h"

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace aerosimlab::simulation
{
struct QuadrotorParameters;
struct QuadrotorControllerParameters;

/**
 * @brief 운동학 기준선과 AirSim 강체 경로 중 사용할 계산 방식을 고른다.
 * @details 강체 경로에서는 AirSim이 로터 힘, 이동, 회전과 접촉 반응을 계산한다.
 */
enum class SimulationDynamics
{
    Kinematic,
    RigidBody
};

/** @brief 네이티브는 AirSim, WebAssembly는 운동학 기준선을 기본으로 사용한다. */
#if defined(__EMSCRIPTEN__)
inline constexpr SimulationDynamics defaultSimulationDynamics =
    SimulationDynamics::Kinematic;
#else
inline constexpr SimulationDynamics defaultSimulationDynamics =
    SimulationDynamics::RigidBody;
#endif

/** @brief 한 시뮬레이션 안에서 드론 하나를 식별한다. */
using DroneId = std::uint64_t;

/** @brief 마지막 Tick에서 측정한 Flecs 시스템별 실행 시간이다. 단위는 마이크로초다. */
struct EcsSystemDurations
{
    double advanceKinematicDroneMicroseconds = 0.0;
    double applyRigidBodyForcesMicroseconds = 0.0;
    double advanceCameraMicroseconds = 0.0;
};

/**
 * @brief 화면 프레임 시간을 고정 간격 모의 Tick 수로 바꾼다.
 * @details Tick보다 짧은 나머지 시간은 다음 프레임으로 넘긴다. 한 프레임에서 처리할 시간은 100ms로 제한해 화면이 멈춘 뒤 과도한 따라잡기를 막는다.
 */
class FixedTickAccumulator
{
public:
    static constexpr double fixedStepSeconds = DroneModel::fixedStepSeconds;
    static constexpr double maximumFrameSeconds = 0.1;

    /** @brief 지난 프레임 시간에 따라 지금 실행할 Tick 수를 반환한다. 0 이하이거나 NaN인 입력은 무시한다. */
    [[nodiscard]] std::size_t Consume(double elapsedSeconds) noexcept;

private:
    double accumulatedSeconds_ = 0.0;
};

/**
 * @brief 화면이 없어도 같은 순서로 한 단계씩 실행할 수 있는 드론 모의 코어다.
 * @details 호출자가 AdvanceOneTick을 부를 때마다 모의 시간이 1/60초 진행된다. 일시정지 중에는 상태와 시간이 바뀌지 않는다.
 */
class SimulationCore
{
public:
    /** @brief Flecs 실행 공간을 만들고 드론과 탑재 카메라를 준비한다. */
    explicit SimulationCore(SimulationDynamics dynamics = defaultSimulationDynamics);

    /** @brief 지정한 기체 물성으로 강체 드론을 준비한다. */
    SimulationCore(SimulationDynamics dynamics, QuadrotorParameters quadrotorParameters);

    /** @brief 기체 물성과 제어기 이득을 지정해 강체 드론을 준비한다. */
    SimulationCore(SimulationDynamics dynamics, QuadrotorParameters quadrotorParameters,
        const QuadrotorControllerParameters& controllerParameters);

    /** @brief 모의 객체와 Flecs 실행 공간을 함께 정리한다. */
    ~SimulationCore();

    SimulationCore(const SimulationCore&) = delete;
    SimulationCore& operator=(const SimulationCore&) = delete;

    /** @brief 드론에 명령을 보내고 적용 결과를 반환한다. */
    [[nodiscard]] CommandResult Apply(const DroneCommand& command) noexcept;

    /** @brief 지정한 드론에 명령을 보내고 적용 결과를 반환한다. */
    [[nodiscard]] CommandResult Apply(DroneId droneId, const DroneCommand& command) noexcept;

    /** @brief 시작 위치와 방향을 지정해 드론 하나를 추가한다. 방향은 월드 Z축 기준 라디안이다. */
    [[nodiscard]] DroneId AddDrone(
        const glm::dvec3& positionMeters, double yawRadians = 0.0);

    /** @brief 지형 높이가 있는 환경에서 지면 높이까지 정해 드론을 추가한다. */
    [[nodiscard]] DroneId AddDrone(const glm::dvec3& positionMeters,
        double yawRadians, double groundElevationMeters);

    /** @brief 첫 Tick 전에 주 드론의 시작 위치와 방향을 지정한다. */
    void SetInitialPose(const glm::dvec3& positionMeters, double yawRadians = 0.0);

    /** @brief 첫 Tick 전에 위치·속도·자세·각속도를 지정한다. 강체 모드에서는 로터 시작 속도도 설정한다. */
    void SetInitialState(DroneId droneId, const DroneState& state);

    /** @brief 드론과 연결된 물리 강체를 함께 제거한다. 마지막 드론은 제거할 수 없다. */
    [[nodiscard]] bool RemoveDrone(DroneId droneId);

    /** @brief 기본 단일 드론 호출이 대상으로 삼는 드론의 식별값을 반환한다. */
    [[nodiscard]] DroneId PrimaryDroneId() const noexcept;

    /** @brief 현재 시뮬레이션에 있는 드론 수를 반환한다. */
    [[nodiscard]] std::size_t DroneCount() const noexcept;

    /** @brief 현재 시뮬레이션의 물리 계산 방식을 반환한다. */
    [[nodiscard]] SimulationDynamics Dynamics() const noexcept;

    /** @brief 현재 드론 식별값을 오름차순으로 돌려준다. */
    [[nodiscard]] std::vector<DroneId> DroneIds() const;

    /** @brief 드론별 배터리 계산을 설정한다. 초기 충전량은 용량의 백분율이다. */
    void ConfigureBattery(DroneId droneId, BatteryModelParameters parameters,
        double initialPercent = 100.0);

    /** @brief 지정한 드론에 배터리 계산이 설정되어 있는지 확인한다. */
    [[nodiscard]] bool HasBattery(DroneId droneId) const noexcept;

    /** @brief 지정한 드론의 배터리 상태를 돌려준다. 설정되지 않았다면 예외를 던진다. */
    [[nodiscard]] BatteryModel& Battery(DroneId droneId);

    /** @brief 지정한 드론의 배터리 상태를 읽기 전용으로 돌려준다. */
    [[nodiscard]] const BatteryModel& Battery(DroneId droneId) const;

    /** @brief 실행 중이면 고정 간격 한 단계만 진행하고, 일시정지 중이면 아무것도 바꾸지 않는다. */
    void AdvanceOneTick();

    /** @brief 모의 시간 진행을 멈춘다. */
    void Pause() noexcept;

    /** @brief 모의 시간 진행을 다시 시작한다. */
    void Resume() noexcept;

    /** @brief 시간, 드론 위치와 상태를 처음 값으로 되돌리고 실행을 계속한다. */
    void Reset();

    /** @brief 현재 드론 상태를 반환한다. */
    [[nodiscard]] const DroneState& State() const noexcept;

    /** @brief 지정한 드론 상태를 반환한다. 없는 식별값이면 예외를 던진다. */
    [[nodiscard]] const DroneState& State(DroneId droneId) const;

    /** @brief 모의 카메라의 상태와 프레임 생성 정보를 반환한다. */
    [[nodiscard]] const CameraState& Camera() const noexcept;

    /** @brief 지정한 드론에 탑재된 카메라 상태를 반환한다. */
    [[nodiscard]] const CameraState& Camera(DroneId droneId) const;

    /** @brief 지정한 드론에 탑재할 카메라 해상도와 시야각을 설정한다. */
    void SetCameraModel(DroneId droneId, const CameraModel& model);

    /** @brief 카메라를 켜거나 끈다. 고장 상태는 복구 명령으로만 해제된다. */
    void SetCameraEnabled(bool enabled) noexcept;

    /** @brief 지정한 드론의 카메라를 켜거나 끈다. */
    void SetCameraEnabled(DroneId droneId, bool enabled);

    /** @brief 카메라 장애를 주입하거나 해제한다. */
    void SetCameraFault(bool faulted) noexcept;

    /** @brief 지정한 드론의 카메라 장애를 주입하거나 해제한다. */
    void SetCameraFault(DroneId droneId, bool faulted);

    /** @brief AirSim 강체 모의에서 지정한 드론 로터의 전원을 끊거나 복구한다. 로터 번호는 0부터 시작한다. */
    void SetRotorMotorFault(DroneId droneId, std::size_t rotorIndex, bool faulted);

    /** @brief 지정한 드론 로터가 고장 상태인지 확인한다. */
    [[nodiscard]] bool IsRotorMotorFaulted(DroneId droneId, std::size_t rotorIndex) const;

    /** @brief 강체 모드에서 사용할 월드 기준 바람 속도를 설정한다. */
    void SetWindVelocityMetersPerSecond(const glm::dvec3& velocity);

    /** @brief 벤치마크에서 물리 단계 시간 측정을 켜거나 끈다. */
    void SetPhysicsStepTimingEnabled(bool enabled) noexcept;

    /** @brief 벤치마크에서 마지막 Tick의 Flecs 시스템별 시간을 확인하도록 측정을 켜거나 끈다. */
    void SetEcsSystemTimingEnabled(bool enabled) noexcept;

    /** @brief 마지막 Tick에서 개별 Flecs 시스템이 실행된 시간을 반환한다. */
    [[nodiscard]] EcsSystemDurations LastEcsSystemDurations() const noexcept;

    /** @brief 마지막 Tick의 Flecs 진행 전체 시간을 마이크로초로 반환한다. */
    [[nodiscard]] double LastFlecsProgressDurationMicroseconds() const noexcept;

    /** @brief 마지막 Tick의 AirSim 물리 계산 시간을 마이크로초로 반환한다. */
    [[nodiscard]] double LastPhysicsStepDurationMicroseconds() const noexcept;

    /** @brief 마지막 Tick에서 접촉 정보를 찾는 데 든 시간을 마이크로초로 반환한다. */
    [[nodiscard]] double LastContactGenerationDurationMicroseconds() const noexcept;

    /** @brief 지도 건물처럼 움직이지 않는 상자 장애물을 추가한다. */
    void AddStaticObstacleBox(
        const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters);

    /** @brief 건물 외곽선을 높이로 돌출해 충돌 장애물로 등록한다. */
    void AddStaticObstaclePrism(
        const std::vector<glm::dvec2>& outlineMeters,
        double bottomMeters, double topMeters);

    /** @brief 높이를 확인할 수 없는 구조물 외곽선을 비행 금지 구역으로 등록한다. */
    void AddStaticObstacleFootprint(
        const std::vector<glm::dvec2>& outlineMeters);

    /** @brief 지형 높이 자료를 설정해 비행체가 지면 아래로 내려가지 않게 한다. */
    void SetTerrain(terrain::TerrainGrid terrain);

    /** @brief 지금까지 실행한 고정 간격 단계 수를 반환한다. */
    [[nodiscard]] std::uint64_t TickCount() const noexcept;

    /** @brief 누적된 모의 시간을 초 단위로 반환한다. */
    [[nodiscard]] double SimulationTimeSeconds() const noexcept;

    /** @brief 모의가 일시정지 상태인지 반환한다. */
    [[nodiscard]] bool IsPaused() const noexcept;

private:
    struct EcsStorage;
    std::unique_ptr<EcsStorage> ecs_;
    bool paused_ = false;
};
}
