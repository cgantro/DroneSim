#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"
#include "terrain/TerrainGrid.h"

#include <glm/vec3.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>

namespace aerosimlab::simulation
{
/**
 * @brief 기존 운동학 기준선과 Jolt 강체 경로 중 사용할 계산 방식을 고른다.
 * @details 강체 경로는 모터와 비행 제어기를 거쳐 명령을 Jolt 물리에 적용한다.
 */
enum class SimulationDynamics
{
    Kinematic,
    RigidBody
};

/** @brief 한 시뮬레이션 안에서 드론 하나를 식별한다. */
using DroneId = std::uint64_t;

/**
 * @brief 화면 프레임 시간을 고정 간격 모의 Tick 수로 바꾼다.
 * @details Tick보다 짧은 나머지 시간은 다음 프레임으로 넘긴다. 한 프레임에서 처리할 시간은 100ms로 제한해 화면이 멈춘 뒤 과도한 따라잡기를 막는다.
 */
class FixedTickAccumulator
{
public:
    static constexpr double fixedStepSeconds = DroneModel::fixedStepSeconds;
    static constexpr double maximumFrameSeconds = 0.1;

    /** @brief 지난 프레임 시간에 따라 지금 실행할 Tick 수를 반환한다. */
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
    explicit SimulationCore(SimulationDynamics dynamics = SimulationDynamics::Kinematic);

    /** @brief 모의 객체와 Flecs 실행 공간을 함께 정리한다. */
    ~SimulationCore();

    SimulationCore(const SimulationCore&) = delete;
    SimulationCore& operator=(const SimulationCore&) = delete;

    /** @brief 드론에 명령을 보내고 적용 결과를 반환한다. */
    [[nodiscard]] CommandResult Apply(const DroneCommand& command) noexcept;

    /** @brief 지정한 드론에 명령을 보내고 적용 결과를 반환한다. */
    [[nodiscard]] CommandResult Apply(DroneId droneId, const DroneCommand& command) noexcept;

    /** @brief 지정한 월드 위치에 드론 하나를 추가하고 그 식별값을 반환한다. */
    [[nodiscard]] DroneId AddDrone(const glm::dvec3& positionMeters);

    /** @brief 드론과 연결된 물리 강체를 함께 제거한다. 마지막 드론은 제거할 수 없다. */
    [[nodiscard]] bool RemoveDrone(DroneId droneId);

    /** @brief 기본 단일 드론 호출이 대상으로 삼는 드론의 식별값을 반환한다. */
    [[nodiscard]] DroneId PrimaryDroneId() const noexcept;

    /** @brief 현재 시뮬레이션에 있는 드론 수를 반환한다. */
    [[nodiscard]] std::size_t DroneCount() const noexcept;

    /** @brief 실행 중이면 고정 간격 한 단계만 진행하고, 일시정지 중이면 아무것도 바꾸지 않는다. */
    void AdvanceOneTick() noexcept;

    /** @brief 모의 시간 진행을 멈춘다. */
    void Pause() noexcept;

    /** @brief 모의 시간 진행을 다시 시작한다. */
    void Resume() noexcept;

    /** @brief 시간, 드론 위치와 상태를 처음 값으로 되돌리고 실행을 계속한다. */
    void Reset() noexcept;

    /** @brief 현재 드론 상태를 반환한다. */
    [[nodiscard]] const DroneState& State() const noexcept;

    /** @brief 지정한 드론 상태를 반환한다. 없는 식별값이면 예외를 던진다. */
    [[nodiscard]] const DroneState& State(DroneId droneId) const;

    /** @brief 모의 카메라의 상태와 프레임 생성 정보를 반환한다. */
    [[nodiscard]] const CameraState& Camera() const noexcept;

    /** @brief 카메라를 켜거나 끈다. 고장 상태는 복구 명령으로만 해제된다. */
    void SetCameraEnabled(bool enabled) noexcept;

    /** @brief 카메라 장애를 주입하거나 해제한다. */
    void SetCameraFault(bool faulted) noexcept;

    /** @brief 강체 모드에서 사용할 월드 기준 바람 속도를 설정한다. */
    void SetWindVelocityMetersPerSecond(const glm::dvec3& velocity);

    /** @brief 벤치마크에서 물리 단계 시간 측정을 켜거나 끈다. */
    void SetPhysicsStepTimingEnabled(bool enabled) noexcept;

    /** @brief 마지막 Tick의 Jolt 물리 계산 시간을 마이크로초로 반환한다. */
    [[nodiscard]] double LastPhysicsStepDurationMicroseconds() const noexcept;

#ifndef __EMSCRIPTEN__
    /** @brief 강체 모드에 지도 건물 같은 고정 상자 장애물을 추가한다. */
    void AddStaticObstacleBox(
        const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters);

    /** @brief 강체 모드의 평평한 바닥을 높이 자료에 맞는 충돌 지형으로 바꾼다. */
    void SetTerrain(const terrain::TerrainGrid& terrain);
#endif

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
