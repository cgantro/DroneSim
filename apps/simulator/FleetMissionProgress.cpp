#include "FleetMissionProgress.h"

#include <algorithm>
#include <array>

namespace aerosimlab::simulator::detail
{
bool IsFleetMissionInProgress(std::string_view status) noexcept
{
    // 상태 문구는 화면과 원격 보고에 쓰고, 진행 여부는 이 목록에서만 판단한다.
    constexpr std::array<std::string_view, 13> activeStatuses{
        "경로 전달 대기",
        "출발 준비",
        "정찰 이동 중",
        "중계 준비",
        "중계 위치로 이동 중",
        "집결 이동 중",
        "재집결 준비",
        "복귀 준비",
        "복귀 중",
        "저전력 복귀 중",
        "저전력 착륙 중",
        "착륙 중",
        "복귀 착륙 중"};
    return std::find(activeStatuses.begin(), activeStatuses.end(), status)
        != activeStatuses.end();
}

FleetMissionProgressDecision EvaluateFleetMissionProgress(
    FleetMissionPhase phase,
    std::span<const std::optional<FleetMissionReport>> reports) noexcept
{
    if (reports.empty())
        return {FleetMissionProgress::Waiting, {}};

    const auto expectedLeg = static_cast<std::uint8_t>(phase);
    for (const auto& report : reports)
    {
        if (!report)
            return {FleetMissionProgress::Waiting, {}};

        const bool returnedForLowBattery =
            report->status == "저전력 복귀 완료"
            && report->mode == simulation::FlightMode::Grounded;
        if (returnedForLowBattery
            && (phase == FleetMissionPhase::Reconnaissance
                || phase == FleetMissionPhase::Returning))
            continue;

        if (phase == FleetMissionPhase::Reconnaissance
            && report->missionLeg == 3U)
        {
            if (report->status == "중계 실패 · 안전 착륙")
                return {FleetMissionProgress::Failed,
                    "통신 중계 위치를 확보하지 못했습니다."};
            if (report->status != "중계 위치 확보"
                || report->mode == simulation::FlightMode::Grounded
                || report->mode == simulation::FlightMode::Landing)
                return {FleetMissionProgress::Waiting, {}};
            continue;
        }

        if (report->missionLeg != expectedLeg)
            return {FleetMissionProgress::Waiting, {}};

        switch (phase)
        {
        case FleetMissionPhase::Reconnaissance:
            if (report->status == "경로 오류" || report->status == "목표 외 착륙")
                return {FleetMissionProgress::Failed,
                    "정찰 목표에 도착하지 못해 임무를 중단했습니다."};
            if (report->mode != simulation::FlightMode::Grounded
                || report->status != "정찰 위치 도착")
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionPhase::Regrouping:
            if (report->status == "집결 실패")
                return {FleetMissionProgress::Failed,
                    "모든 기체가 집결하지 못해 임무를 중단했습니다."};
            if (report->status != "재집결 완료")
                return {FleetMissionProgress::Waiting, {}};
            break;
        case FleetMissionPhase::Returning:
            if (report->status == "복귀 실패")
                return {FleetMissionProgress::Failed,
                    "복귀에 실패한 드론이 있습니다."};
            if (report->mode != simulation::FlightMode::Grounded
                || report->status != "복귀 완료")
                return {FleetMissionProgress::Waiting, {}};
            break;
        }
    }

    return {phase == FleetMissionPhase::Returning
            ? FleetMissionProgress::Complete : FleetMissionProgress::Advance,
        {}};
}
}
