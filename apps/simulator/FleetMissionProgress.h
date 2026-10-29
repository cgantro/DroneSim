#pragma once

#include "simulation_core/DroneModel.h"

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace aerosimlab::simulator
{
/** @brief 드론 한 대가 수행 중인 임무 구간이다. */
enum class FleetMissionLeg : std::uint8_t
{
    Reconnaissance,
    Regrouping,
    Returning
};

namespace detail
{
/** @brief 화면 문구와 분리해 임무 판정에 사용하는 드론 상태다. */
enum class FleetDroneStatus : std::uint8_t
{
    Other,
    RoutePending,
    TakeoffReady,
    Reconnaissance,
    RegroupPreparing,
    Regrouping,
    ReturnPreparing,
    Returning,
    LowBatteryReturning,
    LowBatteryLanding,
    Landing,
    ReturnLanding,
    TargetReached,
    RegroupComplete,
    ReturnComplete,
    LowBatteryReturnComplete,
    RouteFailed,
    OutsideTargetLanding,
    DroneFailureLanding,
    LeaderCommunicationFailureLanding,
    BatteryReturnRouteFailed,
    AssignmentExpiredLanding,
    AssignmentExpiredWaiting,
    LeaderReassignmentWaiting,
    RegroupFailed,
    ReturnFailed
};

/** @brief 통신으로 받은 표시 문구를 내부 판정 상태로 바꾼다. */
[[nodiscard]] FleetDroneStatus ParseFleetDroneStatus(
    std::string_view status) noexcept;

/** @brief 표시 상태가 진행 중인 드론 임무인지 확인한다. */
[[nodiscard]] bool IsFleetMissionInProgress(std::string_view status) noexcept;

/** @brief 한 드론이 리더에게 보고한 임무 진행 정보다. */
struct FleetMissionReport
{
    simulation::FlightMode mode;
    FleetMissionLeg leg;
    FleetDroneStatus status;
};

/** @brief 모든 드론의 보고를 모아 내린 단계 판정이다. */
enum class FleetMissionProgress : std::uint8_t
{
    Waiting,
    Advance,
    Complete,
    Failed
};

struct FleetMissionProgressDecision
{
    FleetMissionProgress progress;
    std::string_view failureReason;
};

/**
 * @brief 최신 보고가 현재 임무 단계를 마쳤는지 확인한다.
 * @details 보고가 없거나 다른 단계의 보고면 기다린다. 실패 상태는 중단으로,
 * 복귀 단계는 모든 드론의 착륙까지 확인한 뒤 완료로 판정한다.
 */
[[nodiscard]] FleetMissionProgressDecision EvaluateFleetMissionProgress(
    FleetMissionLeg leg,
    std::span<const std::optional<FleetMissionReport>> reports) noexcept;
}
}
