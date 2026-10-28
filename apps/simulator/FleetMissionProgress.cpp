#include "FleetMissionProgress.h"

namespace aerosimlab::simulator::detail
{
FleetMissionProgressDecision EvaluateFleetMissionProgress(
    FleetMissionPhase phase,
    std::span<const std::optional<FleetMissionReport>> reports) noexcept
{
    if (reports.empty())
        return {FleetMissionProgress::Waiting, {}};

    const auto expectedLeg = static_cast<std::uint8_t>(phase);
    for (const auto& report : reports)
    {
        if (!report || report->missionLeg != expectedLeg)
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
