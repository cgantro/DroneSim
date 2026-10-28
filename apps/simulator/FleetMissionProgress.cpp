#include "FleetMissionProgress.h"

namespace aerosimlab::simulator::detail
{
FleetDroneStatus ParseFleetDroneStatus(std::string_view status) noexcept
{
    if (status == "경로 전달 대기") return FleetDroneStatus::RoutePending;
    if (status == "출발 준비") return FleetDroneStatus::TakeoffReady;
    if (status == "정찰 이동 중" || status == "이동 중")
        return FleetDroneStatus::Reconnaissance;
    if (status == "중계 준비") return FleetDroneStatus::RelayPreparing;
    if (status == "중계 위치로 이동 중") return FleetDroneStatus::MovingToRelay;
    if (status == "집결 이동 중") return FleetDroneStatus::Regrouping;
    if (status == "재집결 준비") return FleetDroneStatus::RegroupPreparing;
    if (status == "복귀 준비") return FleetDroneStatus::ReturnPreparing;
    if (status == "복귀 중") return FleetDroneStatus::Returning;
    if (status == "저전력 복귀 중") return FleetDroneStatus::LowBatteryReturning;
    if (status == "저전력 착륙 중") return FleetDroneStatus::LowBatteryLanding;
    if (status == "착륙 중") return FleetDroneStatus::Landing;
    if (status == "복귀 착륙 중") return FleetDroneStatus::ReturnLanding;
    if (status == "정찰 위치 도착") return FleetDroneStatus::TargetReached;
    if (status == "중계 위치 확보") return FleetDroneStatus::RelayPositionSecured;
    if (status == "재집결 완료") return FleetDroneStatus::RegroupComplete;
    if (status == "복귀 완료") return FleetDroneStatus::ReturnComplete;
    if (status == "저전력 복귀 완료") return FleetDroneStatus::LowBatteryReturnComplete;
    if (status == "경로 오류") return FleetDroneStatus::RouteFailed;
    if (status == "목표 외 착륙") return FleetDroneStatus::OutsideTargetLanding;
    if (status == "기체 고장 · 안전 착륙")
        return FleetDroneStatus::DroneFailureLanding;
    if (status == "리더 통신 장애 · 기체 안전 착륙")
        return FleetDroneStatus::LeaderCommunicationFailureLanding;
    if (status == "배터리 부족 · 복귀 경로 오류")
        return FleetDroneStatus::BatteryReturnRouteFailed;
    if (status == "임무 권한 만료 · 안전 착륙")
        return FleetDroneStatus::AssignmentExpiredLanding;
    if (status == "임무 권한 만료 · 재배정 대기")
        return FleetDroneStatus::AssignmentExpiredWaiting;
    if (status == "리더 임무 재배정 대기")
        return FleetDroneStatus::LeaderReassignmentWaiting;
    if (status == "집결 실패") return FleetDroneStatus::RegroupFailed;
    if (status == "복귀 실패") return FleetDroneStatus::ReturnFailed;
    if (status == "중계 실패 · 안전 착륙") return FleetDroneStatus::RelayFailed;
    return FleetDroneStatus::Other;
}

bool IsFleetMissionInProgress(std::string_view status) noexcept
{
    switch (ParseFleetDroneStatus(status))
    {
    case FleetDroneStatus::RoutePending:
    case FleetDroneStatus::TakeoffReady:
    case FleetDroneStatus::Reconnaissance:
    case FleetDroneStatus::RelayPreparing:
    case FleetDroneStatus::MovingToRelay:
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
    if (reports.empty())
        return {FleetMissionProgress::Waiting, {}};

    for (const auto& report : reports)
    {
        if (!report)
            return {FleetMissionProgress::Waiting, {}};

        const bool returnedForLowBattery =
            report->status == FleetDroneStatus::LowBatteryReturnComplete
            && report->mode == simulation::FlightMode::Grounded;
        if (returnedForLowBattery
            && (leg == FleetMissionLeg::Reconnaissance
                || leg == FleetMissionLeg::Returning))
            continue;

        if (leg == FleetMissionLeg::Reconnaissance
            && report->leg == FleetMissionLeg::Relaying)
        {
            if (report->status == FleetDroneStatus::RelayFailed)
                return {FleetMissionProgress::Failed,
                    "통신 중계 위치를 확보하지 못했습니다."};
            if (report->status != FleetDroneStatus::RelayPositionSecured
                || report->mode == simulation::FlightMode::Grounded
                || report->mode == simulation::FlightMode::Landing)
                return {FleetMissionProgress::Waiting, {}};
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
        case FleetMissionLeg::Relaying:
            return {FleetMissionProgress::Waiting, {}};
        }
    }

    return {leg == FleetMissionLeg::Returning
            ? FleetMissionProgress::Complete : FleetMissionProgress::Advance,
        {}};
}
}
