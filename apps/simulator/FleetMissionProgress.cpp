#include "FleetMissionProgress.h"

namespace dronesim::simulator::detail
{
/** @brief 내부 임무 상태를 HUD에서 읽을 수 있는 한국어 문구로 바꾼다. */
std::string_view FleetDroneStatusLabel(FleetDroneStatus status) noexcept
{
    switch (status)
    {
    case FleetDroneStatus::RoutePending: return "경로 전달 대기";
    case FleetDroneStatus::TakeoffReady: return "출발 준비";
    case FleetDroneStatus::Reconnaissance: return "정찰 이동 중";
    case FleetDroneStatus::RegroupPreparing: return "재집결 준비";
    case FleetDroneStatus::Regrouping: return "집결 이동 중";
    case FleetDroneStatus::ReturnPreparing: return "복귀 준비";
    case FleetDroneStatus::Returning: return "복귀 중";
    case FleetDroneStatus::LowBatteryReturning: return "저전력 복귀 중";
    case FleetDroneStatus::LowBatteryLanding: return "저전력 착륙 중";
    case FleetDroneStatus::Landing: return "착륙 중";
    case FleetDroneStatus::ReturnLanding: return "복귀 착륙 중";
    case FleetDroneStatus::TargetReached: return "정찰 위치 도착";
    case FleetDroneStatus::RegroupComplete: return "재집결 완료";
    case FleetDroneStatus::ReturnComplete: return "복귀 완료";
    case FleetDroneStatus::LowBatteryReturnComplete: return "저전력 복귀 완료";
    case FleetDroneStatus::RouteFailed: return "경로 오류";
    case FleetDroneStatus::OutsideTargetLanding: return "목표 외 착륙";
    case FleetDroneStatus::BatteryReturnRouteFailed:
        return "배터리 부족 · 복귀 경로 오류";
    case FleetDroneStatus::AssignmentExpiredLanding:
        return "임무 권한 만료 · 안전 착륙";
    case FleetDroneStatus::RegroupFailed: return "집결 실패";
    case FleetDroneStatus::ReturnFailed: return "복귀 실패";
    case FleetDroneStatus::Other: return {};
    }
    return {};
}

/** @brief 드론 상태가 아직 임무 수행 중인지 판정한다. */
bool IsFleetMissionInProgress(FleetDroneStatus status) noexcept
{
    switch (status)
    {
    case FleetDroneStatus::RoutePending:
    case FleetDroneStatus::TakeoffReady:
    case FleetDroneStatus::Reconnaissance:
    case FleetDroneStatus::RegroupPreparing:
    case FleetDroneStatus::Regrouping:
    case FleetDroneStatus::ReturnPreparing:
    case FleetDroneStatus::Returning:
    case FleetDroneStatus::LowBatteryReturning:
    case FleetDroneStatus::LowBatteryLanding:
    case FleetDroneStatus::Landing:
    case FleetDroneStatus::ReturnLanding:
        return true;
    default:
        return false;
    }
}

FleetMissionProgressDecision EvaluateFleetMissionProgress(
    FleetMissionLeg leg,
    std::span<const std::optional<FleetMissionReport>> reports) noexcept
{
    // 각 드론의 최신 보고가 모일 때까지 기다린 뒤 현재 임무 단계의 완료 조건을 검사한다.
    if (reports.empty())
        return {FleetMissionProgress::Waiting, {}};

    for (const auto& report : reports)
    {
        if (!report)
            return {FleetMissionProgress::Waiting, {}};

        const bool returnedForLowBattery =
            report->status == FleetDroneStatus::LowBatteryReturnComplete
            && report->mode == simulation::FlightMode::Grounded;
        if (returnedForLowBattery)
        {
            if (leg == FleetMissionLeg::Reconnaissance)
                return {FleetMissionProgress::Failed,
                    "배터리 예비량에 도달해 정찰을 중단했습니다."};
            if (leg == FleetMissionLeg::Returning)
                continue;
        }

        if (report->leg != leg)
            return {FleetMissionProgress::Waiting, {}};

        switch (leg)
        {
        case FleetMissionLeg::Reconnaissance:
            if (report->status == FleetDroneStatus::RouteFailed
                || report->status == FleetDroneStatus::OutsideTargetLanding)
                return {FleetMissionProgress::Failed,
                    "정찰 목표에 도착하지 못해 임무를 중단했습니다."};
            if (report->mode != simulation::FlightMode::Grounded
                || report->status != FleetDroneStatus::TargetReached)
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionLeg::Regrouping:
            if (report->status == FleetDroneStatus::RegroupFailed)
                return {FleetMissionProgress::Failed,
                    "모든 기체가 집결하지 못해 임무를 중단했습니다."};
            if (report->status != FleetDroneStatus::RegroupComplete)
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionLeg::Returning:
            if (report->status == FleetDroneStatus::ReturnFailed)
                return {FleetMissionProgress::Failed,
                    "복귀에 실패한 드론이 있습니다."};
            if (report->mode != simulation::FlightMode::Grounded
                || report->status != FleetDroneStatus::ReturnComplete)
                return {FleetMissionProgress::Waiting, {}};
            break;
        }
    }

    return {leg == FleetMissionLeg::Returning
            ? FleetMissionProgress::Complete : FleetMissionProgress::Advance,
        {}};
}
}
