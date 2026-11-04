#include "FleetMissionProgress.h"

namespace dronesim::mission
{

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
    // 임무 단계는 편대원 전원의 보고로 판정한다. 한 대라도 보고가 없으면
    // 아직 실패나 완료를 확정할 정보가 없으므로 다음 Tick까지 기다린다.
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
            // 예비 배터리에 도달해 먼저 착륙한 기체는 정찰을 마친 것으로
            // 볼 수 없다. 복귀 단계에서는 이미 안전하게 착륙한 것으로 인정한다.
            if (leg == FleetMissionLeg::Reconnaissance)
                return {FleetMissionProgress::Failed,
                    "배터리 예비량에 도달해 정찰을 중단했습니다."};
            if (leg == FleetMissionLeg::Returning)
                continue;
        }

        // 이전 구간의 늦은 보고가 새 구간 완료로 오인되지 않게 구간도 맞춘다.
        if (report->leg != leg)
            return {FleetMissionProgress::Waiting, {}};

        switch (leg)
        {
        case FleetMissionLeg::Reconnaissance:
            // 도착 상태만으로는 충분하지 않다. 목표 지점에서 착륙 상태까지
            // 확인해야 해당 기체의 정찰 구간이 끝난 것으로 친다.
            if (report->status == FleetDroneStatus::RouteFailed
                || report->status == FleetDroneStatus::OutsideTargetLanding)
                return {FleetMissionProgress::Failed,
                    "정찰 목표에 도착하지 못해 임무를 중단했습니다."};
            if (report->mode != simulation::FlightMode::Grounded
                || report->status != FleetDroneStatus::TargetReached)
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionLeg::Regrouping:
            // 집결 구간은 모든 드론이 집결 완료를 보고해야 다음 구간으로 간다.
            if (report->status == FleetDroneStatus::RegroupFailed)
                return {FleetMissionProgress::Failed,
                    "모든 기체가 집결하지 못해 임무를 중단했습니다."};
            if (report->status != FleetDroneStatus::RegroupComplete)
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionLeg::Returning:
            // 복귀 완료는 기지 도착뿐 아니라 착륙 상태까지 포함한다.
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
