#include "AeroSimLabApp.h"
#include "terrain/MapFlightRoute.h"
#include "graphics/Window.h"
#include <glm/geometric.hpp>
#include <algorithm>
#include <cmath>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace aerosimlab::simulator
{
bool AeroSimLabApp::InspectionMissionIsActive() const
{
    using Stage = InspectionMissionStage;
    switch (inspectionMission_.stage)
    {
    case Stage::StartRequested:
    case Stage::Paused:
    case Stage::TakingOff:
    case Stage::Approaching:
    case Stage::AligningCamera:
    case Stage::Capturing:
    case Stage::Landing:
        return true;
    default:
        return false;
    }
}

bool AeroSimLabApp::RequestInspectionMission(
    std::optional<std::size_t> selectedTargetIndex)
{
    if (std::any_of(fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return entry.second.status == "출발 준비"
                || entry.second.status == "이동 중"
                || entry.second.status == "착륙 중";
        }))
    {
        inspectionMission_.error = "편대 정찰이 진행 중일 때는 주 드론 임무를 시작할 수 없습니다.";
        return false;
    }

    if (InspectionMissionIsActive())
    {
        inspectionMission_.error = "정찰 임무가 이미 진행 중입니다.";
        return false;
    }

    if (simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
    {
        inspectionMission_.error = "기체가 지상에 있을 때 임무를 시작할 수 있습니다.";
        inspectionMission_.stage = InspectionMissionStage::Failed;
        return false;
    }

    inspectionMission_.error.clear();
    inspectionMission_.startTick = 0;
    inspectionMission_.captureStartFrame = 0;
    inspectionMission_.captureReadyTick = 0;
    inspectionMission_.readyFrameSaved = false;
    inspectionMission_.completedTargetCount = 0;
    if (selectedTargetIndex
        && *selectedTargetIndex >= inspectionTargets_.size())
    {
        inspectionMission_.error = "선택한 정찰 위치를 찾을 수 없습니다.";
        inspectionMission_.stage = InspectionMissionStage::Failed;
        return false;
    }
    inspectionMission_.activeTargetIndex = selectedTargetIndex.value_or(0U);
    inspectionMission_.endTargetIndex = selectedTargetIndex
        ? *selectedTargetIndex + 1U
        : inspectionTargets_.size();
    inspectionMission_.targetCount = inspectionMission_.endTargetIndex
        - inspectionMission_.activeTargetIndex;
    InspectionTarget& firstTarget = inspectionTargets_[inspectionMission_.activeTargetIndex];
    firstTarget.routeStartMeters = simulation_.State().positionMeters;
    if (firstTarget.routeAutomaticallyPlanned)
    {
        const auto route = aerosimlab::terrain::PlanMapFlightRoute(
            terrainGrid_, routeBuildings_, simulation_.State().positionMeters,
            firstTarget.approachPositionMeters, 3.0,
            cameraTransitAltitudeMeters_);
        if (!route)
        {
            inspectionMission_.error = "현재 위치에서 첫 정찰 위치까지 건물 우회 경로를 찾지 못했습니다.";
            inspectionMission_.stage = InspectionMissionStage::Failed;
            return false;
        }
        firstTarget.routeWaypointsMeters.assign(route->begin(), route->end() - 1);
    }
    SetActiveInspectionTarget(inspectionMission_.activeTargetIndex);
    pendingControllerAction_.reset();
    pendingPilotCommand_.reset();
#ifdef __EMSCRIPTEN__
    webPilotAxes_.reset();
#endif
    inspectionMission_.stage = InspectionMissionStage::StartRequested;
    return true;
}

bool AeroSimLabApp::StartFleetReconnaissance()
{
    if (InspectionMissionIsActive() || inspectionTargets_.empty())
        return false;
    if (std::any_of(fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return entry.second.status == "출발 준비"
                || entry.second.status == "이동 중"
                || entry.second.status == "착륙 중";
        }))
        return false;

    const auto droneIds = simulation_.DroneIds();
    if (droneIds.empty() || std::any_of(droneIds.begin(), droneIds.end(),
        [this](aerosimlab::simulation::DroneId id)
        {
            return simulation_.State(id).mode != aerosimlab::simulation::FlightMode::Grounded;
        }))
        return false;

    std::vector<std::size_t> targetOrder{0U};
    for (const std::string_view preferredName : {"탄천교 하부", "탄천 물길"})
    {
        const auto target = std::find_if(inspectionTargets_.begin(), inspectionTargets_.end(),
            [preferredName](const InspectionTarget& item)
            {
                return item.displayName == preferredName;
            });
        if (target != inspectionTargets_.end())
            targetOrder.push_back(static_cast<std::size_t>(
                std::distance(inspectionTargets_.begin(), target)));
    }

    std::vector<FleetReconnaissanceState> missions;
    missions.reserve(droneIds.size());
    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const std::size_t targetIndex = index < targetOrder.size()
            ? targetOrder[index] : index % inspectionTargets_.size();
        const auto& target = inspectionTargets_[targetIndex];
        const auto& start = simulation_.State(droneIds[index]).positionMeters;
        const auto route = aerosimlab::terrain::PlanMapFlightRoute(
            terrainGrid_, routeBuildings_, start, target.approachPositionMeters,
            3.0, cameraTransitAltitudeMeters_);
        if (!route)
            return false;

        FleetReconnaissanceState mission;
        mission.targetIndex = targetIndex;
        mission.startPositionMeters = start;
        mission.routeWaypointsMeters.assign(route->begin() + 1, route->end());
        mission.status = "출발 준비";
        missions.push_back(std::move(mission));
    }

    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const aerosimlab::simulation::DroneCommand takeoff{
            aerosimlab::simulation::DroneCommand::Kind::TakeOff};
        if (simulation_.Apply(droneIds[index], takeoff)
            != aerosimlab::simulation::CommandResult::Accepted)
            return false;
        fleetReconnaissance_[droneIds[index]] = std::move(missions[index]);
    }
    externalControl_ = true;
    groundControlActive_ = true;
    window_->SetTitle("DroneSim | 편대 정찰 임무 진행 중");
    return true;
}

void AeroSimLabApp::AdvanceFleetReconnaissance()
{
    using DroneId = aerosimlab::simulation::DroneId;
    using Command = aerosimlab::simulation::DroneCommand;
    using FlightMode = aerosimlab::simulation::FlightMode;

    for (auto& [droneId, mission] : fleetReconnaissance_)
    {
        if (mission.status == "정찰 위치 도착" || mission.status == "경로 오류")
            continue;

        const auto& state = simulation_.State(droneId);
        if (state.mode == FlightMode::Grounded)
        {
            const auto& target = inspectionTargets_[mission.targetIndex];
            const glm::dvec2 horizontalError{
                state.positionMeters.x - target.approachPositionMeters.x,
                state.positionMeters.y - target.approachPositionMeters.y};
            mission.status = glm::length(horizontalError) <= 5.0
                ? "정찰 위치 도착" : "목표 외 착륙";
            continue;
        }
        if (state.mode == FlightMode::TakingOff || state.mode == FlightMode::Landing)
            continue;

        if (!mission.waypointCommanded)
        {
            mission.waypointCommanded = true;
            mission.status = "이동 중";
        }
        else if (mission.nextWaypoint >= mission.routeWaypointsMeters.size())
        {
            const Command land{Command::Kind::Land};
            if (simulation_.Apply(droneId, land) == aerosimlab::simulation::CommandResult::Accepted)
                mission.status = "착륙 중";
            else
                mission.status = "경로 오류";
            continue;
        }
        else if (glm::length(mission.routeWaypointsMeters[mission.nextWaypoint]
                - state.positionMeters) <= 2.0)
        {
            ++mission.nextWaypoint;
            if (mission.nextWaypoint >= mission.routeWaypointsMeters.size())
                continue;
        }

        if (mission.nextWaypoint < mission.routeWaypointsMeters.size())
        {
            Command waypoint;
            waypoint.kind = Command::Kind::SetWaypoint;
            waypoint.vector = mission.routeWaypointsMeters[mission.nextWaypoint];
            if (simulation_.Apply(droneId, waypoint)
                != aerosimlab::simulation::CommandResult::Accepted)
                mission.status = "경로 오류";
        }
    }
}

bool AeroSimLabApp::CancelInspectionMission()
{
    if (!InspectionMissionIsActive())
        return false;

    FailInspectionMission("지상통제소에서 임무를 중단했습니다.");
    return true;
}

bool AeroSimLabApp::PauseInspectionMission()
{
    using Stage = InspectionMissionStage;
    const bool canPause = inspectionMission_.stage == Stage::Approaching
        || inspectionMission_.stage == Stage::AligningCamera;
    if (!canPause)
        return false;

    const aerosimlab::simulation::DroneCommand stop{
        aerosimlab::simulation::DroneCommand::Kind::Stop};
    if (ApplyAndRecordCommand(stop) != aerosimlab::simulation::CommandResult::Accepted)
        return false;

    inspectionMission_.pausedStage = inspectionMission_.stage;
    inspectionMission_.pauseStartTick = simulation_.TickCount();
    inspectionMission_.stage = Stage::Paused;
    return true;
}

bool AeroSimLabApp::ResumeInspectionMission()
{
    if (inspectionMission_.stage != InspectionMissionStage::Paused)
        return false;

    inspectionMission_.startTick += simulation_.TickCount()
        - inspectionMission_.pauseStartTick;
    inspectionMission_.stage = inspectionMission_.pausedStage;
    if (inspectionMission_.pausedStage == InspectionMissionStage::Approaching)
    {
        aerosimlab::simulation::DroneCommand waypoint;
        waypoint.kind = aerosimlab::simulation::DroneCommand::Kind::SetWaypoint;
        waypoint.vector = inspectionMission_.goalMeters;
        if (ApplyAndRecordCommand(waypoint)
            != aerosimlab::simulation::CommandResult::Accepted)
        {
            FailInspectionMission("정찰 경로를 다시 시작하지 못했습니다.");
            return false;
        }
    }
    return true;
}

void AeroSimLabApp::AdvanceInspectionMission()
{
    using Stage = InspectionMissionStage;
    using CommandKind = aerosimlab::simulation::DroneCommand::Kind;

    if (inspectionMission_.stage == Stage::Paused)
        return;

    if (inspectionMission_.stage == Stage::StartRequested)
    {
#ifndef __EMSCRIPTEN__
        aerosimlab::communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            aerosimlab::communication::VideoRecordingCommand::Start, recordingReply);
        if (!recordingReply.accepted)
        {
            FailInspectionMission(recordingReply.status);
            return;
        }
#endif
        const aerosimlab::simulation::DroneCommand takeoff{CommandKind::TakeOff};
        if (ApplyAndRecordCommand(takeoff)
            != aerosimlab::simulation::CommandResult::Accepted)
        {
            FailInspectionMission("이륙 명령을 적용하지 못했습니다.");
            return;
        }
        inspectionMission_.startTick = simulation_.TickCount();
        inspectionMission_.stage = Stage::TakingOff;
    }

    const std::uint64_t elapsedTicks = simulation_.TickCount()
        - inspectionMission_.startTick;
    if (inspectionMission_.stage != Stage::Ready
        && inspectionMission_.stage != Stage::Saved
        && inspectionMission_.stage != Stage::GeometryOnly
        && inspectionMission_.stage != Stage::Failed
        && elapsedTicks > 60U * 180U * inspectionMission_.targetCount)
    {
        FailInspectionMission("제한 시간 안에 정찰 위치의 영상 수집을 마치지 못했습니다.");
        return;
    }

    if (inspectionMission_.stage == Stage::TakingOff)
    {
        const auto mode = simulation_.State().mode;
        if (mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying)
        {
            aerosimlab::simulation::DroneCommand waypoint;
            waypoint.kind = CommandKind::SetWaypoint;
            waypoint.vector = inspectionMission_.goalMeters;
            if (ApplyAndRecordCommand(waypoint)
                != aerosimlab::simulation::CommandResult::Accepted)
            {
                FailInspectionMission("정찰 위치 이동 명령을 적용하지 못했습니다.");
                return;
            }
            inspectionMission_.stage = Stage::Approaching;
        }
        return;
    }

    if (inspectionMission_.stage == Stage::Approaching)
    {
        const double distanceMeters = glm::length(
            inspectionMission_.goalMeters - simulation_.State().positionMeters);
        if (distanceMeters <= 2.0)
        {
            const InspectionTarget& target =
                inspectionTargets_[inspectionMission_.activeTargetIndex];
            if (inspectionMission_.routeWaypointIndex < target.routeWaypointsMeters.size())
            {
                ++inspectionMission_.routeWaypointIndex;
                inspectionMission_.goalMeters = inspectionMission_.routeWaypointIndex
                        < target.routeWaypointsMeters.size()
                    ? target.routeWaypointsMeters[inspectionMission_.routeWaypointIndex]
                    : target.approachPositionMeters;
                aerosimlab::simulation::DroneCommand waypoint;
                waypoint.kind = CommandKind::SetWaypoint;
                waypoint.vector = inspectionMission_.goalMeters;
                if (ApplyAndRecordCommand(waypoint)
                    != aerosimlab::simulation::CommandResult::Accepted)
                {
                    FailInspectionMission("다음 정찰 경유점 이동 명령을 적용하지 못했습니다.");
                    return;
                }
            }
            else
            {
                inspectionMission_.stage = Stage::AligningCamera;
            }
        }
        return;
    }

    if (inspectionMission_.stage == Stage::AligningCamera)
    {
        const auto& state = simulation_.State();
        const glm::dvec2 direction{
            inspectionMission_.surface.centerMeters.x - state.positionMeters.x,
            inspectionMission_.surface.centerMeters.y - state.positionMeters.y};
        const double targetYawRadians = std::atan2(direction.y, direction.x);
        constexpr double twoPi = 6.28318530717958647692;
        const double yawErrorRadians = std::remainder(
            targetYawRadians - state.yawRadians, twoPi);
        if (std::abs(yawErrorRadians) <= 0.04)
        {
            const aerosimlab::simulation::DroneCommand stopYaw{
                CommandKind::SetYawRate, glm::dvec3{0.0}, 0.0};
            static_cast<void>(ApplyAndRecordCommand(stopYaw));
            inspectionMission_.captureStartFrame = simulation_.Camera().lastFrameTick;
            inspectionMission_.stage = Stage::Capturing;
#ifdef __EMSCRIPTEN__
            EM_ASM({
                if (window.startDroneSimInspectionZoneRecording)
                    window.startDroneSimInspectionZoneRecording(
                        UTF8ToString($0), UTF8ToString($1), $2);
            }, inspectionMission_.facilityId.c_str(), inspectionMission_.zoneId.c_str(),
                static_cast<double>(simulation_.TickCount()));
#endif
        }
        else
        {
            const aerosimlab::simulation::DroneCommand alignYaw{
                CommandKind::SetYawRate, glm::dvec3{0.0},
                std::clamp(yawErrorRadians * 2.0, -1.0, 1.0)};
            if (ApplyAndRecordCommand(alignYaw)
                != aerosimlab::simulation::CommandResult::Accepted)
            {
                FailInspectionMission("정찰 대상에 카메라 방향을 맞추지 못했습니다.");
                return;
            }
        }
        return;
    }

    if (inspectionMission_.stage == Stage::Capturing)
    {
#ifdef __EMSCRIPTEN__
        const auto& camera = simulation_.Camera();
        if (camera.lastFrameTick > inspectionMission_.captureStartFrame)
        {
            if (AssessInspectionView()
                == aerosimlab::simulation::CaptureAssessment::Ready)
            {
                if (!inspectionMission_.readyFrameSaved)
                    inspectionMission_.captureReadyTick = simulation_.TickCount();
                inspectionMission_.readyFrameSaved = true;
            }
            else
            {
                inspectionMission_.readyFrameSaved = false;
                inspectionMission_.captureReadyTick = 0;
            }
        }
        constexpr std::uint64_t captureHoldTicks = 60U;
        const bool captureHoldComplete = inspectionMission_.readyFrameSaved
            && simulation_.TickCount() - inspectionMission_.captureReadyTick >= captureHoldTicks;
#endif
#ifdef __EMSCRIPTEN__
        if (captureHoldComplete)
#else
        if (inspectionMission_.readyFrameSaved)
#endif
        {
#ifdef __EMSCRIPTEN__
            EM_ASM({
                if (window.recordDroneSimCapturedZone) {
                    window.recordDroneSimCapturedZone(
                        UTF8ToString($0), UTF8ToString($1), $2);
                }
            }, inspectionMission_.facilityId.c_str(), inspectionMission_.zoneId.c_str(),
                static_cast<double>(camera.lastFrameTick));
#endif
            ++inspectionMission_.completedTargetCount;
            if (inspectionMission_.activeTargetIndex + 1U < inspectionMission_.endTargetIndex)
            {
                SetActiveInspectionTarget(inspectionMission_.activeTargetIndex + 1U);
                aerosimlab::simulation::DroneCommand waypoint;
                waypoint.kind = CommandKind::SetWaypoint;
                waypoint.vector = inspectionMission_.goalMeters;
                if (ApplyAndRecordCommand(waypoint)
                    != aerosimlab::simulation::CommandResult::Accepted)
                {
                    FailInspectionMission("다음 정찰 위치 이동 명령을 적용하지 못했습니다.");
                    return;
                }
                inspectionMission_.captureStartFrame = 0;
                inspectionMission_.captureReadyTick = 0;
                inspectionMission_.readyFrameSaved = false;
                inspectionMission_.stage = Stage::Approaching;
                return;
            }

            const aerosimlab::simulation::DroneCommand land{CommandKind::Land};
            if (ApplyAndRecordCommand(land)
                != aerosimlab::simulation::CommandResult::Accepted)
            {
                FailInspectionMission("촬영 후 착륙 명령을 적용하지 못했습니다.");
                return;
            }
            inspectionMission_.stage = Stage::Landing;
        }
        return;
    }

    if (inspectionMission_.stage == Stage::Landing
        && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded)
    {
#ifndef __EMSCRIPTEN__
        if (eoVideoRecorder_)
            eoVideoRecorder_->SetMissionOutcome(
                EoVideoRecorder::MissionOutcome::Completed);
        aerosimlab::communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            aerosimlab::communication::VideoRecordingCommand::Stop, recordingReply);
        if (!recordingReply.accepted)
        {
            FailInspectionMission(recordingReply.status);
            return;
        }
        inspectionMission_.stage = Stage::Saved;
#else
        inspectionMission_.stage = Stage::GeometryOnly;
#endif
    }
}

void AeroSimLabApp::FailInspectionMission(const std::string& reason)
{
    inspectionMission_.error = reason;
    inspectionMission_.stage = InspectionMissionStage::Failed;
    if (simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
    {
        const aerosimlab::simulation::DroneCommand land{
            aerosimlab::simulation::DroneCommand::Kind::Land};
        static_cast<void>(ApplyAndRecordCommand(land));
    }
#ifndef __EMSCRIPTEN__
    if (eoVideoRecorder_ && eoVideoRecorder_->IsRecording())
    {
        eoVideoRecorder_->SetMissionOutcome(
            EoVideoRecorder::MissionOutcome::Failed, reason);
        aerosimlab::communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            aerosimlab::communication::VideoRecordingCommand::Stop, recordingReply);
    }
#endif
}

std::string AeroSimLabApp::InspectionMissionStatus() const
{
    const std::size_t targetCount = inspectionMission_.targetCount == 0U
        ? inspectionTargets_.size()
        : inspectionMission_.targetCount;
    const std::string progress = "목표 "
        + std::to_string(std::min(inspectionMission_.completedTargetCount + 1U,
            targetCount))
        + "/" + std::to_string(targetCount) + ": ";
    switch (inspectionMission_.stage)
    {
    case InspectionMissionStage::Ready: return "대기 중";
    case InspectionMissionStage::StartRequested: return progress + "시작 준비 중";
    case InspectionMissionStage::Paused: return progress + "일시 정지";
    case InspectionMissionStage::TakingOff: return progress + "이륙 중";
    case InspectionMissionStage::Approaching: return progress + "촬영 위치로 이동 중";
    case InspectionMissionStage::AligningCamera:
        return progress + "카메라 방향을 맞추는 중";
    case InspectionMissionStage::Capturing: return progress + "영상 수집 중";
    case InspectionMissionStage::Landing: return "착륙 중";
    case InspectionMissionStage::Saved:
        return "영상 저장 완료 (" + std::to_string(inspectionMission_.completedTargetCount)
            + "개 위치)";
    case InspectionMissionStage::GeometryOnly:
        return "영상 수집 완료 (" + std::to_string(inspectionMission_.completedTargetCount)
            + "개 위치)";
    case InspectionMissionStage::Failed: return "실패: " + inspectionMission_.error;
    }
    return "상태를 알 수 없습니다.";
}


}
