#pragma once

#include "simulation_core/DroneModel.h"

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace aerosimlab::simulator::detail
{
/** @brief 한 드론이 리더에게 보고한 임무 진행 정보다. */
struct FleetMissionReport
{
    simulation::FlightMode mode;
    std::uint8_t missionLeg;
    std::string_view status;
};

/** @brief 전체 임무에서 확인 중인 단계다. */
enum class FleetMissionPhase : std::uint8_t
{
    Reconnaissance,
    Regrouping,
    Returning
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
    FleetMissionPhase phase,
    std::span<const std::optional<FleetMissionReport>> reports) noexcept;
}
