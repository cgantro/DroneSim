#include "AeroSimLabApp.h"
#include "FleetMissionProgress.h"
#include "TrainingRangeScene.h"
#include "terrain/MapFlightRoute.h"
#include "graphics/Window.h"
#include <glm/geometric.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace aerosimlab::simulator
{
namespace detail
{
std::string EncodePeerTelemetry(
    const glm::dvec3& positionMeters,
    aerosimlab::simulation::FlightMode mode,
    std::uint8_t missionLeg,
    const std::string& missionStatus,
    std::uint64_t assignmentVersion,
    std::uint64_t assignmentLeaseExpiresAtTick)
{
    std::ostringstream payload;
    payload << std::setprecision(17) << positionMeters.x << ' '
        << positionMeters.y << ' ' << positionMeters.z << ' '
        << aerosimlab::simulation::ToString(mode) << ' '
        << static_cast<unsigned int>(missionLeg) << ' '
        << std::quoted(missionStatus) << ' '
        << assignmentVersion << ' ' << assignmentLeaseExpiresAtTick;
    return payload.str();
}

std::optional<PeerTelemetry> DecodePeerTelemetry(const std::string& payload)
{
    std::istringstream values(payload);
    PeerTelemetry telemetry;
    std::string modeName;
    unsigned int missionLeg = 0;
    if (!(values >> telemetry.positionMeters.x >> telemetry.positionMeters.y
            >> telemetry.positionMeters.z >> modeName >> missionLeg
            >> std::quoted(telemetry.missionStatus)
            >> telemetry.assignmentVersion
            >> telemetry.assignmentLeaseExpiresAtTick)
        || missionLeg > 2U
        || !std::isfinite(telemetry.positionMeters.x)
        || !std::isfinite(telemetry.positionMeters.y)
        || !std::isfinite(telemetry.positionMeters.z))
        return std::nullopt;

    values >> std::ws;
    if (!values.eof())
        return std::nullopt;
    const auto mode = aerosimlab::simulation::TryParseFlightMode(modeName);
    if (!mode)
        return std::nullopt;
    telemetry.mode = *mode;
    telemetry.missionLeg = static_cast<std::uint8_t>(missionLeg);
    return telemetry;
}
}

namespace
{
constexpr communication::NetworkNodeId groundControlNodeId =
    std::numeric_limits<communication::NetworkNodeId>::max();
constexpr std::uint64_t peerTelemetrySuspectAfterTicks = 30U;
constexpr std::uint64_t peerTelemetryUnknownAfterTicks = 180U;
constexpr std::uint64_t missionLeaseGraceTicks = 1800U;
constexpr double missionLeasePlanningSpeedMetersPerSecond = 1.0;

std::uint64_t EstimateMissionLeaseExpiry(
    std::uint64_t currentTick,
    const glm::dvec3& startPositionMeters,
    std::span<const glm::dvec3> routeWaypointsMeters)
{
    double routeLengthMeters = 0.0;
    glm::dvec3 previousPositionMeters = startPositionMeters;
    for (const glm::dvec3& waypointMeters : routeWaypointsMeters)
    {
        routeLengthMeters += glm::distance(previousPositionMeters, waypointMeters);
        previousPositionMeters = waypointMeters;
    }
    const double estimatedTravelTicks = std::ceil(routeLengthMeters
        / missionLeasePlanningSpeedMetersPerSecond * 60.0);
    return currentTick + static_cast<std::uint64_t>(estimatedTravelTicks)
        + missionLeaseGraceTicks;
}

std::string EncodeWaypoint(const glm::dvec3& waypoint)
{
    std::ostringstream payload;
    payload << std::setprecision(17)
        << waypoint.x << ' ' << waypoint.y << ' ' << waypoint.z;
    return payload.str();
}

std::optional<glm::dvec3> DecodeWaypoint(const std::string& payload)
{
    std::istringstream values(payload);
    glm::dvec3 waypoint{0.0};
    if (!(values >> waypoint.x >> waypoint.y >> waypoint.z))
        return std::nullopt;

    values >> std::ws;
    if (!values.eof() || !std::isfinite(waypoint.x)
        || !std::isfinite(waypoint.y) || !std::isfinite(waypoint.z))
        return std::nullopt;
    return waypoint;
}

struct FleetRouteMessage
{
    std::uint8_t leg = 0;
    std::size_t targetIndex = 0;
    std::uint64_t assignmentVersion = 0;
    std::uint64_t assignmentLeaseExpiresAtTick = 0;
    std::vector<glm::dvec3> waypointsMeters;
};

std::string EncodeFleetRoute(
    std::uint8_t leg,
    std::size_t targetIndex,
    std::uint64_t assignmentVersion,
    std::uint64_t assignmentLeaseExpiresAtTick,
    std::span<const glm::dvec3> waypointsMeters)
{
    std::ostringstream payload;
    payload << static_cast<unsigned int>(leg) << ' ' << targetIndex << ' '
        << assignmentVersion << ' '
        << assignmentLeaseExpiresAtTick << ' ' << waypointsMeters.size();
    for (const glm::dvec3& waypoint : waypointsMeters)
    {
        payload << ' ' << std::setprecision(17)
            << waypoint.x << ' ' << waypoint.y << ' ' << waypoint.z;
    }
    return payload.str();
}

std::optional<FleetRouteMessage> DecodeFleetRoute(const std::string& payload)
{
    std::istringstream values(payload);
    unsigned int leg = 0;
    FleetRouteMessage route;
    std::size_t waypointCount = 0;
    if (!(values >> leg >> route.targetIndex >> route.assignmentVersion
            >> route.assignmentLeaseExpiresAtTick >> waypointCount)
        || leg > 2U)
        return std::nullopt;

    route.leg = static_cast<std::uint8_t>(leg);
    for (std::size_t index = 0; index < waypointCount; ++index)
    {
        glm::dvec3 waypoint{0.0};
        if (!(values >> waypoint.x >> waypoint.y >> waypoint.z)
            || !std::isfinite(waypoint.x) || !std::isfinite(waypoint.y)
            || !std::isfinite(waypoint.z))
            return std::nullopt;
        route.waypointsMeters.push_back(waypoint);
    }

    values >> std::ws;
    if (!values.eof())
        return std::nullopt;
    return route;
}

}

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
            return detail::IsFleetMissionInProgress(entry.second.status);
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

bool AeroSimLabApp::FleetReconnaissanceCanStart() const
{
    const auto stage = completeFleetMission_.stage;
    const bool sequenceActive = stage == CompleteFleetMissionState::Stage::FormationTransit
        || stage == CompleteFleetMissionState::Stage::Reconnaissance
        || stage == CompleteFleetMissionState::Stage::Regrouping
        || stage == CompleteFleetMissionState::Stage::Returning;
    if (InspectionMissionIsActive() || formationTransit_.active || sequenceActive
        || inspectionTargets_.empty())
        return false;
    if (std::any_of(fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return detail::IsFleetMissionInProgress(entry.second.status);
        }))
        return false;

    const auto droneIds = simulation_.DroneIds();
    return !droneIds.empty() && std::all_of(droneIds.begin(), droneIds.end(),
        [this](aerosimlab::simulation::DroneId id)
        {
            return simulation_.State(id).mode
                == aerosimlab::simulation::FlightMode::Grounded;
        });
}

const AeroSimLabApp::ReceivedPeerTelemetry* AeroSimLabApp::FreshPeerTelemetry(
    aerosimlab::simulation::DroneId droneId) const noexcept
{
    constexpr std::uint64_t telemetryTimeoutTicks = 30U;
    const auto telemetry = peerTelemetryAtLeader_.find(droneId);
    if (telemetry == peerTelemetryAtLeader_.end()
        || simulation_.TickCount() > telemetry->second.receivedAtTick
            + telemetryTimeoutTicks)
        return nullptr;
    return &telemetry->second;
}

bool AeroSimLabApp::CompleteFleetMissionCanStart() const
{
    using Stage = CompleteFleetMissionState::Stage;
    const bool sequenceActive = completeFleetMission_.stage == Stage::FormationTransit
        || completeFleetMission_.stage == Stage::Reconnaissance
        || completeFleetMission_.stage == Stage::Regrouping
        || completeFleetMission_.stage == Stage::Returning;
    if (sequenceActive || InspectionMissionIsActive() || formationTransit_.active
        || inspectionTargets_.empty())
        return false;

    const auto droneIds = simulation_.DroneIds();
    return droneIds.size() == 4U && std::all_of(droneIds.begin(), droneIds.end(),
        [this](aerosimlab::simulation::DroneId id)
        {
            return simulation_.State(id).mode
                == aerosimlab::simulation::FlightMode::Grounded;
        });
}

bool AeroSimLabApp::CompleteFleetMissionIsActive() const
{
    using Stage = CompleteFleetMissionState::Stage;
    return completeFleetMission_.stage == Stage::FormationTransit
        || completeFleetMission_.stage == Stage::Reconnaissance
        || completeFleetMission_.stage == Stage::Regrouping
        || completeFleetMission_.stage == Stage::Returning;
}

bool AeroSimLabApp::FleetReconnaissanceIsActive() const
{
    return std::any_of(fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return detail::IsFleetMissionInProgress(entry.second.status);
        });
}

bool AeroSimLabApp::StartCompleteFleetMission()
{
    if (!CompleteFleetMissionCanStart())
        return false;

    completeFleetMission_.homePositionsMeters.clear();
    for (const auto droneId : simulation_.DroneIds())
        completeFleetMission_.homePositionsMeters.emplace(
            droneId, simulation_.State(droneId).positionMeters);

    if (!StartFormationTransit())
    {
        completeFleetMission_.homePositionsMeters.clear();
        return false;
    }

    completeFleetMission_.stage = CompleteFleetMissionState::Stage::FormationTransit;
    completeFleetMission_.status = "M01 편대 이동 중";
    return true;
}

std::string AeroSimLabApp::CompleteFleetMissionStatus() const
{
    return completeFleetMission_.status;
}

bool AeroSimLabApp::StartFleetReconnaissance()
{
    if (!FleetReconnaissanceCanStart())
        return false;

    std::map<aerosimlab::simulation::DroneId, glm::dvec3> homePositions;
    for (const auto droneId : simulation_.DroneIds())
        homePositions.emplace(droneId, simulation_.State(droneId).positionMeters);
    return StartFleetReconnaissance(true, homePositions);
}

bool AeroSimLabApp::StartFleetReconnaissance(
    bool takeOffDrones,
    const std::map<aerosimlab::simulation::DroneId, glm::dvec3>& homePositions)
{
    using namespace aerosimlab::simulation;
    if (takeOffDrones)
        InitializeFleetLeadership();
    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    const DroneId leaderId = fleetLeaderId_;
    const auto dronesReady = std::all_of(droneIds.begin(), droneIds.end(),
        [this, leaderId, takeOffDrones](DroneId id)
        {
            FlightMode mode;
            if (takeOffDrones || id == leaderId)
                mode = simulation_.State(id).mode;
            else
            {
                const ReceivedPeerTelemetry* report = FreshPeerTelemetry(id);
                if (report == nullptr)
                    return false;
                mode = report->state.mode;
            }
            return takeOffDrones
                ? mode == FlightMode::Grounded
                : (mode == FlightMode::Hovering || mode == FlightMode::Flying
                    || mode == FlightMode::FollowingWaypoint);
        });
    if (droneIds.empty() || inspectionTargets_.empty() || !dronesReady
        || InspectionMissionIsActive())
        return false;

    std::vector<FleetReconnaissanceState> missions;
    missions.reserve(droneIds.size());
    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const std::size_t targetIndex = index % inspectionTargets_.size();
        const auto& target = inspectionTargets_[targetIndex];
        const DroneId droneId = droneIds[index];
        const glm::dvec3* start = &simulation_.State(droneId).positionMeters;
        if (!takeOffDrones && droneId != leaderId)
        {
            const ReceivedPeerTelemetry* report = FreshPeerTelemetry(droneId);
            if (report == nullptr)
                return false;
            start = &report->state.positionMeters;
        }
        const auto route = aerosimlab::terrain::PlanMapFlightRoute(
            terrainGrid_, routeBuildings_, *start, target.approachPositionMeters,
            3.0, cameraTransitAltitudeMeters_);
        if (!route)
            return false;

        FleetReconnaissanceState mission;
        mission.targetIndex = targetIndex;
        const auto home = homePositions.find(droneIds[index]);
        mission.startPositionMeters = home != homePositions.end()
            ? home->second : *start;
        mission.routeWaypointsMeters.assign(route->begin(), route->end());
        mission.status = takeOffDrones ? "출발 준비" : "정찰 이동 중";
        missions.push_back(std::move(mission));
    }

    if (!PrepareFleetNetwork(takeOffDrones))
        return false;

    if (takeOffDrones)
    {
        std::vector<DroneId> takingOff;
        takingOff.reserve(droneIds.size());
        for (const DroneId droneId : droneIds)
        {
            if (simulation_.Apply(droneId, {DroneCommand::Kind::TakeOff})
                != CommandResult::Accepted)
            {
                for (const DroneId airborneDrone : takingOff)
                    static_cast<void>(simulation_.Apply(airborneDrone,
                        {DroneCommand::Kind::Land}));
                return false;
            }
            takingOff.push_back(droneId);
        }
    }
    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const DroneId droneId = droneIds[index];
        const FleetMissionLeg leg = missions[index].leg;
        const std::size_t targetIndex = missions[index].targetIndex;
        std::vector<glm::dvec3> route = std::move(missions[index].routeWaypointsMeters);
        missions[index].status = "경로 전달 대기";
        fleetReconnaissance_[droneId] = std::move(missions[index]);
        if (!SendFleetRouteAssignment(
                droneId, leg, targetIndex, std::move(route)))
        {
            fleetReconnaissance_.clear();
            for (const DroneId airborneId : droneIds)
                if (simulation_.State(airborneId).mode != FlightMode::Grounded)
                    static_cast<void>(simulation_.Apply(
                        airborneId, {DroneCommand::Kind::Land}));
            return false;
        }
    }
    externalControl_ = true;
    groundControlActive_ = true;
    window_->SetTitle("DroneSim | 편대 정찰 임무 진행 중");
    return true;
}

bool AeroSimLabApp::FleetReturnCanStart() const
{
    if (fleetReconnaissance_.empty() || InspectionMissionIsActive()
        || formationTransit_.active
        || completeFleetMission_.stage == CompleteFleetMissionState::Stage::FormationTransit
        || completeFleetMission_.stage == CompleteFleetMissionState::Stage::Reconnaissance
        || completeFleetMission_.stage == CompleteFleetMissionState::Stage::Regrouping
        || completeFleetMission_.stage == CompleteFleetMissionState::Stage::Returning)
        return false;

    const auto& droneIds = fleetDroneIds_;
    if (droneIds.empty())
        return false;
    const auto leaderId = fleetLeaderId_;

    return std::all_of(droneIds.begin(), droneIds.end(), [this, leaderId](auto droneId)
    {
        const auto mission = fleetReconnaissance_.find(droneId);
        if (mission == fleetReconnaissance_.end())
            return false;
        if (droneId == leaderId)
            return (simulation_.State(droneId).mode
                    == aerosimlab::simulation::FlightMode::Grounded
                && mission->second.leg == FleetMissionLeg::Reconnaissance
                && !detail::IsFleetMissionInProgress(mission->second.status))
                || (mission->second.lowBatteryReturn
                    && detail::ParseFleetDroneStatus(mission->second.status)
                        == detail::FleetDroneStatus::LowBatteryReturnComplete
                    && simulation_.State(droneId).mode
                        == aerosimlab::simulation::FlightMode::Grounded);

        const ReceivedPeerTelemetry* report = FreshPeerTelemetry(droneId);
        if (report == nullptr)
            return false;
        if (mission->second.lowBatteryReturn
                    && detail::ParseFleetDroneStatus(report->state.missionStatus)
                        == detail::FleetDroneStatus::LowBatteryReturnComplete
            && report->state.mode == aerosimlab::simulation::FlightMode::Grounded)
            return true;
        const bool finishedReconnaissance = report->state.mode
                == aerosimlab::simulation::FlightMode::Grounded
            && report->state.missionLeg
                == static_cast<std::uint8_t>(FleetMissionLeg::Reconnaissance)
            && !detail::IsFleetMissionInProgress(report->state.missionStatus);
        return finishedReconnaissance;
    });
}

bool AeroSimLabApp::StartFleetReturn()
{
    if (!FleetReturnCanStart())
        return false;

    return StartFleetReturn(true);
}

bool AeroSimLabApp::StartFleetReturn(bool takeOffDrones)
{
    using namespace aerosimlab::simulation;

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    const DroneId leaderId = fleetLeaderId_;
    if (droneIds.empty() || leaderId == 0U)
        return false;
    std::vector<std::vector<glm::dvec3>> returnRoutes;
    returnRoutes.reserve(droneIds.size());
    for (const auto droneId : droneIds)
    {
        const auto& mission = fleetReconnaissance_.at(droneId);
        if (mission.lowBatteryReturn
            && detail::ParseFleetDroneStatus(mission.status)
                == detail::FleetDroneStatus::LowBatteryReturnComplete)
        {
            returnRoutes.emplace_back();
            continue;
        }
        glm::dvec3 currentPosition{0.0};
        FlightMode mode = FlightMode::Grounded;
        if (takeOffDrones || droneId == leaderId)
        {
            const auto& state = simulation_.State(droneId);
            currentPosition = state.positionMeters;
            mode = state.mode;
        }
        else
        {
            const ReceivedPeerTelemetry* report = FreshPeerTelemetry(droneId);
            if (report == nullptr)
                return false;
            currentPosition = report->state.positionMeters;
            mode = report->state.mode;
        }
        const bool ready = takeOffDrones
            ? mode == FlightMode::Grounded
            : (mode == FlightMode::Hovering || mode == FlightMode::Flying
                || mode == FlightMode::FollowingWaypoint);
        if (!ready || mission.leg == FleetMissionLeg::Returning)
            return false;
        glm::dvec3 returnApproach = mission.startPositionMeters;
        returnApproach.z += 5.0;
        const auto route = aerosimlab::terrain::PlanMapFlightRoute(
            terrainGrid_, routeBuildings_, currentPosition,
            returnApproach, 3.0, cameraTransitAltitudeMeters_);
        if (!route)
            return false;
        returnRoutes.emplace_back(route->begin(), route->end());
    }

    if (takeOffDrones)
    {
        std::vector<DroneId> takingOff;
        takingOff.reserve(droneIds.size());
        for (const DroneId droneId : droneIds)
        {
            const auto& mission = fleetReconnaissance_.at(droneId);
            if (mission.lowBatteryReturn
                && detail::ParseFleetDroneStatus(mission.status)
                    == detail::FleetDroneStatus::LowBatteryReturnComplete)
                continue;
            if (simulation_.State(droneId).mode == FlightMode::Grounded
                && simulation_.Apply(droneId, {DroneCommand::Kind::TakeOff})
                    != CommandResult::Accepted)
            {
                for (const DroneId airborneDrone : takingOff)
                    static_cast<void>(simulation_.Apply(airborneDrone,
                        {DroneCommand::Kind::Land}));
                return false;
            }
            takingOff.push_back(droneId);
        }
    }

    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        auto& mission = fleetReconnaissance_.at(droneIds[index]);
        if (mission.lowBatteryReturn
            && detail::ParseFleetDroneStatus(mission.status)
                == detail::FleetDroneStatus::LowBatteryReturnComplete)
            continue;
        mission.leg = FleetMissionLeg::Returning;
        mission.status = "경로 전달 대기";
        if (!SendFleetRouteAssignment(droneIds[index],
                FleetMissionLeg::Returning, mission.targetIndex,
                std::move(returnRoutes[index])))
            return false;
    }
    externalControl_ = true;
    groundControlActive_ = true;
    window_->SetTitle("DroneSim | 출발 지점으로 복귀 중");
    return true;
}

bool AeroSimLabApp::StartFleetRally()
{
    using namespace aerosimlab::simulation;
    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.empty() || fleetLeaderId_ == 0U
        || std::any_of(droneIds.begin(), droneIds.end(), [this](DroneId droneId)
            {
                return !completeFleetMission_.homePositionsMeters.contains(droneId);
            }))
        return false;
    const DroneId leaderId = fleetLeaderId_;
    std::vector<DroneId> rallyDroneIds;
    rallyDroneIds.reserve(droneIds.size());
    for (const DroneId droneId : droneIds)
    {
        const auto& mission = fleetReconnaissance_.at(droneId);
        const bool alreadyReturnedForLowBattery = mission.lowBatteryReturn
            && detail::ParseFleetDroneStatus(mission.status)
                == detail::FleetDroneStatus::LowBatteryReturnComplete;
        if (!alreadyReturnedForLowBattery)
            rallyDroneIds.push_back(droneId);
    }
    if (rallyDroneIds.empty())
    {
        if (!StartFleetReturn(false))
            return false;
        completeFleetMission_.stage = CompleteFleetMissionState::Stage::Returning;
        completeFleetMission_.status = "M03 기지로 복귀 중";
        return true;
    }

    glm::dvec3 centerMeters{0.0};
    for (const DroneId droneId : droneIds)
        centerMeters += completeFleetMission_.homePositionsMeters.at(droneId);
    centerMeters /= static_cast<double>(droneIds.size());
    centerMeters.z += 12.0;
    const auto offsets = FormationOffsets(rallyDroneIds.size(), 8.0, 0.0);

    std::vector<std::vector<glm::dvec3>> rallyRoutes;
    rallyRoutes.reserve(rallyDroneIds.size());
    for (std::size_t index = 0; index < rallyDroneIds.size(); ++index)
    {
        const DroneId droneId = rallyDroneIds[index];
        glm::dvec3 currentPosition{0.0};
        FlightMode mode = FlightMode::Grounded;
        if (droneId == leaderId)
        {
            const DroneState& state = simulation_.State(droneId);
            currentPosition = state.positionMeters;
            mode = state.mode;
        }
        else
        {
            const ReceivedPeerTelemetry* report = FreshPeerTelemetry(droneId);
            if (report == nullptr
                || detail::ParseFleetDroneStatus(report->state.missionStatus)
                    != detail::FleetDroneStatus::TargetReached)
                return false;
            currentPosition = report->state.positionMeters;
            mode = report->state.mode;
        }
        if (mode != FlightMode::Grounded)
            return false;
        const glm::dvec3 rallyTarget{
            centerMeters.x + offsets[index].x,
            centerMeters.y + offsets[index].y,
            centerMeters.z};
        const auto route = aerosimlab::terrain::PlanMapFlightRoute(
            terrainGrid_, routeBuildings_, currentPosition, rallyTarget,
            3.0, cameraTransitAltitudeMeters_);
        if (!route)
            return false;
        rallyRoutes.emplace_back(route->begin(), route->end());
    }

    std::vector<DroneId> takingOff;
    takingOff.reserve(rallyDroneIds.size());
    for (const DroneId droneId : rallyDroneIds)
    {
        if (simulation_.State(droneId).mode == FlightMode::Grounded
            && simulation_.Apply(droneId, {DroneCommand::Kind::TakeOff})
                != CommandResult::Accepted)
        {
            for (const DroneId airborneDrone : takingOff)
                static_cast<void>(simulation_.Apply(airborneDrone,
                    {DroneCommand::Kind::Land}));
            return false;
        }
        takingOff.push_back(droneId);
    }

    for (std::size_t index = 0; index < rallyDroneIds.size(); ++index)
    {
        const DroneId droneId = rallyDroneIds[index];
        auto& mission = fleetReconnaissance_.at(droneId);
        mission.leg = FleetMissionLeg::Regrouping;
        mission.status = "경로 전달 대기";
        if (!SendFleetRouteAssignment(droneId,
                FleetMissionLeg::Regrouping, mission.targetIndex,
                std::move(rallyRoutes[index])))
            return false;
    }
    completeFleetMission_.stage = CompleteFleetMissionState::Stage::Regrouping;
    completeFleetMission_.status = "M03 재집결 중";
    return true;
}

std::string AeroSimLabApp::FleetReconnaissanceStatus() const
{
    std::ostringstream status;
    status << fleetLeadershipStatus_ << '\n';

    for (const auto droneId : fleetDroneIds_)
    {
        const auto mission = fleetReconnaissance_.find(droneId);
        if (mission == fleetReconnaissance_.end())
            continue;

        const std::size_t targetIndex = mission->second.targetIndex;
        const std::string& targetName = inspectionTargets_[targetIndex].displayName;
        status << "드론 " << droneId << " · " << targetName
            << " · " << mission->second.status;
        if (simulation_.HasBattery(droneId))
            status << " · 배터리 " << std::fixed << std::setprecision(0)
                << simulation_.Battery(droneId).RemainingPercent() << '%';
        if (droneId != fleetLeaderId_)
        {
            const auto telemetry = peerTelemetryAtLeader_.find(droneId);
            if (telemetry == peerTelemetryAtLeader_.end())
                status << " · 연결 대기";
            else
            {
                const std::uint64_t ageTicks = simulation_.TickCount()
                    - telemetry->second.receivedAtTick;
                status << (ageTicks <= peerTelemetrySuspectAfterTicks
                    ? " · 연결됨"
                    : ageTicks <= peerTelemetryUnknownAfterTicks
                        ? " · 통신 의심" : " · 상태 미확인");
            }
        }
        if (mission->second.pendingRouteSequence != 0U)
        {
            status << (mission->second.routeTransmissionCount >= 10U
                    ? " · 경로 확인 응답 없음"
                    : " · 경로 확인 대기");
        }
        status << '\n';
    }
    return status.str();
}

void AeroSimLabApp::AdvanceFleetReconnaissance()
{
    using DroneId = aerosimlab::simulation::DroneId;
    using Command = aerosimlab::simulation::DroneCommand;
    using FlightMode = aerosimlab::simulation::FlightMode;
    constexpr std::uint64_t routeDeliveryTimeoutTicks = 600U;

    for (auto& [droneId, mission] : fleetReconnaissance_)
    {
        const bool returningFromTarget =
            (mission.leg == FleetMissionLeg::Reconnaissance
                && detail::IsFleetMissionInProgress(mission.status))
            || (mission.leg == FleetMissionLeg::Reconnaissance
                && detail::ParseFleetDroneStatus(mission.status)
                    == detail::FleetDroneStatus::TargetReached);
        if (returningFromTarget && simulation_.HasBattery(droneId))
        {
            auto& battery = simulation_.Battery(droneId);
            if (!battery.IsReturningForLowBattery()
                && battery.ShouldReturnForLowBattery())
            {
                battery.MarkReturningForLowBattery();

                glm::dvec3 returnApproach = mission.startPositionMeters;
                returnApproach.z += 5.0;
                const auto route = aerosimlab::terrain::PlanMapFlightRoute(
                    terrainGrid_, routeBuildings_,
                    simulation_.State(droneId).positionMeters, returnApproach,
                    3.0, cameraTransitAltitudeMeters_);
                bool returnStarted = route.has_value();
                if (returnStarted
                    && simulation_.State(droneId).mode == FlightMode::Grounded)
                    returnStarted = simulation_.Apply(droneId,
                        {Command::Kind::TakeOff})
                        == aerosimlab::simulation::CommandResult::Accepted;
                if (returnStarted)
                {
                    mission.lowBatteryReturn = true;
                    returnStarted = SendFleetRouteAssignment(droneId,
                        FleetMissionLeg::Returning, mission.targetIndex,
                        std::vector<glm::dvec3>(route->begin(), route->end()));
                }
                if (!returnStarted)
                {
                    mission.status = "배터리 부족 · 복귀 경로 오류";
                    mission.assignmentLeaseExpiresAtTick = simulation_.TickCount();
                    const auto mode = simulation_.State(droneId).mode;
                    if (mode != FlightMode::Grounded && mode != FlightMode::Landing)
                        static_cast<void>(simulation_.Apply(droneId,
                            {Command::Kind::Land}));
                }
                else
                {
                    fleetLeadershipStatus_ = "배터리 예비량 도달 · 드론 "
                        + std::to_string(droneId) + " 복귀 중";
                }
                continue;
            }
        }

        const auto droneStatus = detail::ParseFleetDroneStatus(mission.status);
        bool legIsFinished = false;
        switch (mission.leg)
        {
        case FleetMissionLeg::Reconnaissance:
            legIsFinished = droneStatus == detail::FleetDroneStatus::TargetReached
                || droneStatus == detail::FleetDroneStatus::RouteFailed
                || droneStatus == detail::FleetDroneStatus::OutsideTargetLanding
                || droneStatus == detail::FleetDroneStatus::DroneFailureLanding
                || droneStatus
                    == detail::FleetDroneStatus::LeaderCommunicationFailureLanding;
            break;
        case FleetMissionLeg::Regrouping:
            legIsFinished = droneStatus == detail::FleetDroneStatus::RegroupComplete
                || droneStatus == detail::FleetDroneStatus::RegroupFailed;
            break;
        case FleetMissionLeg::Returning:
            legIsFinished = droneStatus == detail::FleetDroneStatus::ReturnComplete
                || droneStatus == detail::FleetDroneStatus::ReturnFailed
                || droneStatus
                    == detail::FleetDroneStatus::LowBatteryReturnComplete;
            break;
        }
        if (legIsFinished)
            continue;

        if (!mission.routeReceived)
        {
            if (simulation_.TickCount() >= mission.routeSentAtTick
                    + routeDeliveryTimeoutTicks)
                mission.status = "경로 오류";
            continue;
        }

        const auto& state = simulation_.State(droneId);
        if (state.mode == FlightMode::Grounded)
        {
            const auto groundedStatus =
                detail::ParseFleetDroneStatus(mission.status);
            if (groundedStatus == detail::FleetDroneStatus::TakeoffReady
                || groundedStatus == detail::FleetDroneStatus::ReturnPreparing
                || groundedStatus == detail::FleetDroneStatus::RegroupPreparing)
                continue;

            if (mission.leg == FleetMissionLeg::Regrouping)
            {
                mission.status = "집결 실패";
                continue;
            }
            const glm::dvec3& destination = mission.leg == FleetMissionLeg::Returning
                ? mission.startPositionMeters
                : inspectionTargets_[mission.targetIndex].approachPositionMeters;
            const glm::dvec2 horizontalError{
                state.positionMeters.x - destination.x,
                state.positionMeters.y - destination.y};
            if (mission.leg == FleetMissionLeg::Returning)
            {
                if (glm::length(horizontalError) > 5.0)
                    mission.status = "복귀 실패";
                else
                    mission.status = mission.lowBatteryReturn
                        ? "저전력 복귀 완료" : "복귀 완료";
            }
            else
                mission.status = glm::length(horizontalError) <= 5.0
                    ? "정찰 위치 도착" : "목표 외 착륙";
            continue;
        }
        if (state.mode == FlightMode::TakingOff || state.mode == FlightMode::Landing)
            continue;

        if (!mission.waypointCommanded)
        {
            mission.waypointCommanded = true;
            if (mission.leg == FleetMissionLeg::Returning)
                mission.status = mission.lowBatteryReturn
                    ? "저전력 복귀 중" : "복귀 중";
            else if (mission.leg == FleetMissionLeg::Regrouping)
                mission.status = "집결 이동 중";
            else
                mission.status = "정찰 이동 중";
        }
        else if (mission.nextWaypoint >= mission.routeWaypointsMeters.size())
        {
            if (mission.leg == FleetMissionLeg::Regrouping)
            {
                mission.status = "재집결 완료";
                continue;
            }
            const Command land{Command::Kind::Land};
            if (simulation_.Apply(droneId, land) == aerosimlab::simulation::CommandResult::Accepted)
            {
                if (mission.leg == FleetMissionLeg::Returning)
                    mission.status = mission.lowBatteryReturn
                        ? "저전력 착륙 중" : "복귀 착륙 중";
                else
                    mission.status = "착륙 중";
            }
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

void AeroSimLabApp::AdvanceCompleteFleetMission()
{
    using Stage = CompleteFleetMissionState::Stage;
    using DroneId = aerosimlab::simulation::DroneId;
    if (completeFleetMission_.stage == Stage::Idle
        || completeFleetMission_.stage == Stage::Completed
        || completeFleetMission_.stage == Stage::Failed)
        return;

    if (completeFleetMission_.stage == Stage::FormationTransit)
    {
        if (formationTransit_.active)
            return;
        if (formationTransit_.status != "목적지 도착 · 편대 유지 중")
        {
            FailCompleteFleetMission("M01 편대 이동이 끝나지 않아 전체 임무를 중단했습니다.");
            return;
        }
        if (!StartFleetReconnaissance(false, completeFleetMission_.homePositionsMeters))
        {
            FailCompleteFleetMission("M02 정찰 경로를 만들지 못해 기체를 착륙시킵니다.");
            return;
        }
        completeFleetMission_.stage = Stage::Reconnaissance;
        completeFleetMission_.status = "M02 분산 정찰 중";
        return;
    }

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.empty())
        return;
    const auto leaderId = fleetLeaderId_;
    std::vector<std::optional<detail::FleetMissionReport>> reports;
    reports.reserve(droneIds.size());
    for (const auto droneId : droneIds)
    {
        const auto mission = fleetReconnaissance_.find(droneId);
        if (mission == fleetReconnaissance_.end())
        {
            if (completeFleetMission_.stage == Stage::Returning)
            {
                completeFleetMission_.stage = Stage::Failed;
                completeFleetMission_.status = "복귀에 실패한 드론이 있습니다.";
            }
            else
            {
                FailCompleteFleetMission(completeFleetMission_.stage
                        == Stage::Reconnaissance
                    ? "드론별 정찰 상태를 찾지 못해 임무를 중단했습니다."
                    : "모든 기체가 집결하지 못해 임무를 중단했습니다.");
            }
            return;
        }

        if (droneId == leaderId)
        {
            reports.emplace_back(detail::FleetMissionReport{
                simulation_.State(droneId).mode,
                mission->second.leg,
                detail::ParseFleetDroneStatus(mission->second.status)});
            continue;
        }

        const ReceivedPeerTelemetry* telemetry = FreshPeerTelemetry(droneId);
        if (telemetry == nullptr)
        {
            const auto cached = peerTelemetryAtLeader_.find(droneId);
            if (cached != peerTelemetryAtLeader_.end()
                && cached->second.state.assignmentVersion
                    == mission->second.assignmentVersion
                && cached->second.state.mode
                    == aerosimlab::simulation::FlightMode::Grounded)
            {
                const bool returnedAfterBatteryReserve = mission->second.lowBatteryReturn
                    && cached->second.state.missionLeg
                        == static_cast<std::uint8_t>(FleetMissionLeg::Returning)
                    && detail::ParseFleetDroneStatus(
                        cached->second.state.missionStatus)
                        == detail::FleetDroneStatus::LowBatteryReturnComplete;
                const bool returnedToBase =
                    completeFleetMission_.stage == Stage::Returning
                    && cached->second.state.missionLeg
                        == static_cast<std::uint8_t>(FleetMissionLeg::Returning)
                    && detail::ParseFleetDroneStatus(
                        cached->second.state.missionStatus)
                        == detail::FleetDroneStatus::ReturnComplete;
                if (returnedAfterBatteryReserve || returnedToBase)
                    telemetry = &cached->second;
            }
        }
        if (telemetry == nullptr)
        {
            reports.emplace_back(std::nullopt);
            continue;
        }
        reports.emplace_back(detail::FleetMissionReport{telemetry->state.mode,
            static_cast<FleetMissionLeg>(telemetry->state.missionLeg),
            detail::ParseFleetDroneStatus(telemetry->state.missionStatus)});
    }

    const auto missionLeg = completeFleetMission_.stage == Stage::Reconnaissance
        ? FleetMissionLeg::Reconnaissance
        : completeFleetMission_.stage == Stage::Regrouping
            ? FleetMissionLeg::Regrouping
            : FleetMissionLeg::Returning;
    const auto decision = detail::EvaluateFleetMissionProgress(missionLeg, reports);
    if (decision.progress == detail::FleetMissionProgress::Failed)
    {
        if (completeFleetMission_.stage == Stage::Returning)
        {
            completeFleetMission_.stage = Stage::Failed;
            completeFleetMission_.status = decision.failureReason;
        }
        else
        {
            FailCompleteFleetMission(std::string{decision.failureReason});
        }
        return;
    }
    if (decision.progress == detail::FleetMissionProgress::Waiting)
    {
        completeFleetMission_.status = "드론 상태 보고를 기다리는 중";
        return;
    }

    if (completeFleetMission_.stage == Stage::Reconnaissance)
    {
        if (!StartFleetRally())
        {
            FailCompleteFleetMission("공통 집결 경로를 만들지 못해 임무를 중단했습니다.");
            return;
        }
        return;
    }

    if (completeFleetMission_.stage == Stage::Regrouping)
    {
        if (!StartFleetReturn(false))
        {
            FailCompleteFleetMission("집결지에서 기지까지 복귀 경로를 만들지 못했습니다.");
            return;
        }
        completeFleetMission_.stage = Stage::Returning;
        completeFleetMission_.status = "M03 기지로 복귀 중";
        return;
    }

    if (completeFleetMission_.stage == Stage::Returning)
    {
        completeFleetMission_.stage = Stage::Completed;
        completeFleetMission_.status = "전체 임무 완료";
        return;
    }
}

void AeroSimLabApp::FailCompleteFleetMission(const std::string& reason)
{
    using FlightMode = aerosimlab::simulation::FlightMode;
    using Command = aerosimlab::simulation::DroneCommand;
    completeFleetMission_.stage = CompleteFleetMissionState::Stage::Failed;
    completeFleetMission_.status = reason;
    formationTransit_.active = false;
    for (const auto droneId : simulation_.DroneIds())
    {
        const FlightMode mode = simulation_.State(droneId).mode;
        if (mode != FlightMode::Grounded && mode != FlightMode::Landing)
            static_cast<void>(simulation_.Apply(droneId, {Command::Kind::Land}));
    }
}

bool AeroSimLabApp::StartFormationTransit()
{
    using namespace aerosimlab::simulation;

    if (formationTransit_.active || inspectionTargets_.empty()
        || InspectionMissionIsActive())
        return false;

    InitializeFleetLeadership();

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.empty())
        return false;
    if (droneIds.size() != 4U || std::any_of(droneIds.begin(), droneIds.end(),
        [this](DroneId droneId)
        {
            return simulation_.State(droneId).mode != FlightMode::Grounded;
        }))
        return false;

    const glm::dvec2 stagingPosition =
        trainingRangeScene_->FormationStagingPositionMeters();
    const double stagingAltitude = std::max(
        terrainGrid_.HeightAtLocalMeters(stagingPosition) + 15.0,
        cameraTransitAltitudeMeters_);
    const glm::dvec3 destination{stagingPosition, stagingAltitude};
    const auto route = PlanFormationRoute(terrainGrid_, routeBuildings_,
        simulation_.State(droneIds.front()).positionMeters, destination,
        droneIds.size(), 8.0, 1.5, 2.0,
        cameraTransitAltitudeMeters_);
    if (!route)
    {
        formationTransit_.status = "목적지까지 편대 경로를 찾지 못했습니다.";
        return false;
    }

    if (!PrepareFleetNetwork(true))
    {
        formationTransit_.status = "가상 통신망을 준비하지 못했습니다.";
        return false;
    }

    std::vector<DroneId> takingOff;
    takingOff.reserve(droneIds.size());
    for (const DroneId droneId : droneIds)
    {
        if (simulation_.Apply(droneId, {DroneCommand::Kind::TakeOff})
            != CommandResult::Accepted)
        {
            for (const DroneId airborneId : takingOff)
                static_cast<void>(simulation_.Apply(airborneId,
                    {DroneCommand::Kind::Land}));
            formationTransit_.status = "드론 이륙 명령을 적용하지 못했습니다.";
            return false;
        }
        takingOff.push_back(droneId);
    }

    formationTransit_.route = *route;
    formationTransit_.waypointIndex = 0U;
    formationTransit_.nextMessageSequence = 1U;
    formationTransit_.sentFollowerTargets.clear();
    formationTransit_.lastTargetSentAtTick.clear();
    formationTransit_.lastAppliedSequence.clear();
    formationTransit_.lastNetworkNodeUpdateTick.reset();
    formationTransit_.active = true;
    formationTransit_.status = "출발 준비 · V자 대형 · "
        + inspectionTargets_.back().displayName;
    externalControl_ = true;
    groundControlActive_ = true;
    pendingControllerAction_.reset();
    pendingPilotCommand_.reset();
#ifdef __EMSCRIPTEN__
    webPilotAxes_.reset();
#endif
    if (window_)
        window_->SetTitle("DroneSim | 편대 이동 임무 진행 중");
    return true;
}

bool AeroSimLabApp::PrepareFleetNetwork(bool reset)
{
    if (formationNetwork_ && !reset)
        return true;

    InitializeFleetLeadership();

    nextTelemetrySequence_ = 1U;
    nextGroundControlTelemetrySequence_ = 1U;
    lastTelemetrySentAtTick_.clear();
    lastGroundControlTelemetrySentAtTick_.clear();
    peerTelemetryAtLeader_.clear();
    telemetryAtGroundControl_.clear();

    communication::VirtualMeshSettings settings;
    settings.mode = fleetNetworkMode_;
    settings.leaderNodeId = fleetLeaderId_;
    settings.maximumLinkRangeMeters = 130.0;
    settings.helloIntervalTicks = 15;
    settings.neighborTimeoutTicks = 90;
    settings.routeLifetimeTicks = 300;
    settings.maximumHops = 5;
    settings.packetLossProbability = static_cast<double>(fleetPacketLossPercent_) / 100.0;
    settings.linkLatencyJitterTicks = fleetLatencyJitterTicks_;
    formationNetwork_ = std::make_unique<
        communication::VirtualMeshNetwork>(settings);

    formationRadioObstacles_.clear();
    formationRadioObstacles_.reserve(routeBuildings_.size());
    for (const aerosimlab::terrain::MapBuilding& building : routeBuildings_)
    {
        if (building.outlineMeters.size() < 3U || building.heightMeters <= 0.0)
            continue;
        const double groundHeight = building.groundElevationsMeters.empty()
            ? 0.0
            : *std::min_element(building.groundElevationsMeters.begin(),
                building.groundElevationsMeters.end());
        formationRadioObstacles_.push_back({
            building.outlineMeters, groundHeight, groundHeight + building.heightMeters});
    }

    std::vector<communication::NetworkNodeSnapshot> nodes;
    for (const auto droneId : simulation_.DroneIds())
        nodes.push_back({droneId, simulation_.State(droneId).positionMeters});
    nodes.push_back({groundControlNodeId, groundControlPositionMeters_});
    formationNetwork_->UpdateNodes(nodes, formationRadioObstacles_);
    formationTransit_.lastNetworkNodeUpdateTick = simulation_.TickCount();
    return !nodes.empty();
}

void AeroSimLabApp::InitializeFleetLeadership()
{
    fleetDroneIds_ = simulation_.DroneIds();
    fleetReconnaissance_.clear();
    fleetLeaderId_ = fleetDroneIds_.empty() ? 0U : fleetDroneIds_.front();
    groundControlPositionMeters_ = simulation_.State().positionMeters;
    groundControlPositionMeters_.z = terrainGrid_.HeightAtLocalMeters(
        {groundControlPositionMeters_.x, groundControlPositionMeters_.y}) + 1.5;
    fleetLeadershipStatus_ = fleetDroneIds_.empty()
        ? "리더 없음" : "리더 정상 · 드론 " + std::to_string(fleetLeaderId_);
}

bool AeroSimLabApp::SendFleetRouteAssignment(
    aerosimlab::simulation::DroneId droneId,
    FleetMissionLeg leg,
    std::size_t targetIndex,
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    if (!formationNetwork_ || inspectionTargets_.empty()
        || targetIndex >= inspectionTargets_.size())
        return false;

    auto mission = fleetReconnaissance_.find(droneId);
    if (mission == fleetReconnaissance_.end() || fleetDroneIds_.empty())
        return false;

    const std::uint64_t assignmentVersion = nextMissionAssignmentVersion_++;
    std::uint64_t leaseExpiresAtTick = 0U;
    if (leg == FleetMissionLeg::Reconnaissance)
    {
        glm::dvec3 startPositionMeters = simulation_.State(droneId).positionMeters;
        if (droneId != fleetLeaderId_)
        {
            if (const auto* report = FreshPeerTelemetry(droneId))
                startPositionMeters = report->state.positionMeters;
        }
        leaseExpiresAtTick = EstimateMissionLeaseExpiry(
            simulation_.TickCount(), startPositionMeters, routeWaypointsMeters);
    }
    const std::uint32_t sequence = std::max(mission->second.routeSequence,
        mission->second.pendingRouteSequence) + 1U;
    const std::string payload = EncodeFleetRoute(
        static_cast<std::uint8_t>(leg), targetIndex, assignmentVersion,
        leaseExpiresAtTick, routeWaypointsMeters);
    if (droneId == fleetLeaderId_)
    {
        ApplyFleetRouteAssignment(droneId, leg, targetIndex,
            std::move(routeWaypointsMeters), sequence, assignmentVersion,
            leaseExpiresAtTick);
        mission->second.pendingRouteSequence = 0U;
        mission->second.pendingRoutePayload.clear();
        return true;
    }

    const auto messageId = formationNetwork_->SendReliable(fleetLeaderId_, droneId,
        sequence, "fleet_route", payload);
    if (!messageId)
        return false;

    mission->second.pendingRouteSequence = sequence;
    mission->second.assignmentVersion = assignmentVersion;
    mission->second.assignmentLeaseExpiresAtTick = leaseExpiresAtTick;
    mission->second.communicationLostAtTick = 0U;
    mission->second.pendingRoutePayload = payload;
    mission->second.routeSentAtTick = simulation_.TickCount();
    mission->second.lastRouteTransmissionTick = simulation_.TickCount();
    mission->second.routeTransmissionCount = 1U;
    mission->second.routeReceived = false;
    mission->second.routeWaypointsMeters.clear();
    mission->second.nextWaypoint = 0U;
    mission->second.waypointCommanded = false;
    mission->second.status = "경로 전달 대기";
    return true;
}

void AeroSimLabApp::ApplyFleetRouteAssignment(
    aerosimlab::simulation::DroneId droneId,
    FleetMissionLeg leg,
    std::size_t targetIndex,
    std::vector<glm::dvec3> routeWaypointsMeters,
    std::uint32_t sequence,
    std::uint64_t assignmentVersion,
    std::uint64_t assignmentLeaseExpiresAtTick)
{
    auto mission = fleetReconnaissance_.find(droneId);
    if (mission == fleetReconnaissance_.end())
        return;

    mission->second.leg = leg;
    mission->second.targetIndex = targetIndex;
    mission->second.routeSequence = sequence;
    mission->second.assignmentVersion = assignmentVersion;
    mission->second.assignmentLeaseExpiresAtTick = assignmentLeaseExpiresAtTick;
    mission->second.communicationLostAtTick = 0U;
    mission->second.routeSentAtTick = simulation_.TickCount();
    mission->second.routeWaypointsMeters = std::move(routeWaypointsMeters);
    mission->second.nextWaypoint = 0U;
    mission->second.routeReceived = true;
    mission->second.waypointCommanded = false;

    auto mode = simulation_.State(droneId).mode;
    if (leg == FleetMissionLeg::Reconnaissance
        && mode == aerosimlab::simulation::FlightMode::Grounded)
    {
        if (simulation_.Apply(droneId,
                {aerosimlab::simulation::DroneCommand::Kind::TakeOff})
            != aerosimlab::simulation::CommandResult::Accepted)
        {
            mission->second.status = "경로 오류";
            return;
        }
        mode = simulation_.State(droneId).mode;
    }
    const bool waitingForTakeoff = mode == aerosimlab::simulation::FlightMode::Grounded
        || mode == aerosimlab::simulation::FlightMode::TakingOff;
    switch (leg)
    {
    case FleetMissionLeg::Reconnaissance:
        mission->second.status = waitingForTakeoff ? "출발 준비" : "정찰 이동 중";
        break;
    case FleetMissionLeg::Regrouping:
        mission->second.status = waitingForTakeoff ? "재집결 준비" : "집결 이동 중";
        break;
    case FleetMissionLeg::Returning:
        mission->second.status = waitingForTakeoff ? "복귀 준비" : "복귀 중";
        break;
    }
}

void AeroSimLabApp::AdvanceFleetNetwork()
{
    using aerosimlab::simulation::DroneCommand;
    using aerosimlab::simulation::DroneId;

    if (!formationNetwork_ || (!formationTransit_.active
        && !CompleteFleetMissionIsActive() && !FleetReconnaissanceIsActive()))
        return;

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    constexpr std::uint64_t networkPositionRefreshTicks = 6U;
    const std::uint64_t currentTick = simulation_.TickCount();
    const bool refreshNetworkPositions = !formationTransit_.lastNetworkNodeUpdateTick
        || currentTick >= *formationTransit_.lastNetworkNodeUpdateTick
            + networkPositionRefreshTicks;
    if (refreshNetworkPositions)
    {
        std::vector<communication::NetworkNodeSnapshot> nodes;
        const auto allDroneIds = simulation_.DroneIds();
        nodes.reserve(allDroneIds.size() + 1U);
        for (const DroneId droneId : allDroneIds)
        {
            const auto& drone = simulation_.State(droneId);
            nodes.push_back({droneId, drone.positionMeters});
        }
        nodes.push_back({groundControlNodeId, groundControlPositionMeters_});
        formationNetwork_->UpdateNodes(nodes, formationRadioObstacles_);
        formationTransit_.lastNetworkNodeUpdateTick = currentTick;
    }

    formationNetwork_->Advance(currentTick);
    DroneId leaderId = fleetLeaderId_;
    for (const auto& delivered : formationNetwork_->DeliveredMessages())
    {
        const auto& message = delivered.message;
        if (message.destination == groundControlNodeId
            && message.kind == "ground_control_telemetry")
        {
            if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.source)
                == fleetDroneIds_.end())
                continue;
            const auto telemetry = detail::DecodePeerTelemetry(message.payload);
            if (!telemetry)
                continue;
            auto& latest = telemetryAtGroundControl_[message.source];
            if (message.sequence > latest.sequence)
            {
                latest.state = *telemetry;
                latest.receivedAtTick = delivered.deliveredAtTick;
                latest.sequence = message.sequence;
            }
            continue;
        }
        if (message.kind == "drone_telemetry"
            && message.destination == leaderId && message.source != leaderId)
        {
            if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.source)
                == fleetDroneIds_.end())
                continue;
            const auto telemetry = detail::DecodePeerTelemetry(message.payload);
            auto& latest = peerTelemetryAtLeader_[message.source];
            if (telemetry && message.sequence > latest.sequence)
            {
                latest.state = *telemetry;
                latest.receivedAtTick = delivered.deliveredAtTick;
                latest.sequence = message.sequence;
            }
            continue;
        }

        if (message.kind == "fleet_route_ack"
            && message.destination == leaderId)
        {
            const auto mission = fleetReconnaissance_.find(message.source);
            if (mission != fleetReconnaissance_.end()
                && mission->second.pendingRouteSequence == message.sequence
                && mission->second.routeSequence == message.sequence)
            {
                mission->second.pendingRouteSequence = 0U;
                mission->second.pendingRoutePayload.clear();
                mission->second.routeTransmissionCount = 0U;
            }
            continue;
        }

        if (message.source != leaderId)
            continue;

        if (message.kind == "formation_waypoint" && formationTransit_.active)
        {
            if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.destination)
                == fleetDroneIds_.end())
                continue;
            const auto waypoint = DecodeWaypoint(message.payload);
            if (!waypoint)
                continue;
            const auto lastSequence = formationTransit_.lastAppliedSequence.find(
                message.destination);
            if (lastSequence != formationTransit_.lastAppliedSequence.end()
                && message.sequence <= lastSequence->second)
                continue;

            DroneCommand command;
            command.kind = DroneCommand::Kind::SetWaypoint;
            command.vector = *waypoint;
            if (simulation_.Apply(message.destination, command)
                != aerosimlab::simulation::CommandResult::Accepted)
            {
                formationTransit_.active = false;
                formationTransit_.status = "통신으로 받은 편대 목표를 적용하지 못했습니다.";
                return;
            }
            formationTransit_.lastAppliedSequence[message.destination] = message.sequence;
            continue;
        }

        if (message.kind != "fleet_route")
            continue;

        if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.destination)
            == fleetDroneIds_.end())
            continue;

        auto mission = fleetReconnaissance_.find(message.destination);
        const auto route = DecodeFleetRoute(message.payload);
        if (mission == fleetReconnaissance_.end() || !route
            || route->targetIndex >= inspectionTargets_.size())
        {
            if (mission != fleetReconnaissance_.end())
                mission->second.status = "경로 오류";
            continue;
        }

        if (route->assignmentLeaseExpiresAtTick != 0U
            && currentTick >= route->assignmentLeaseExpiresAtTick)
        {
            const auto mode = simulation_.State(message.destination).mode;
            if (mode != aerosimlab::simulation::FlightMode::Grounded
                && mode != aerosimlab::simulation::FlightMode::Landing)
                static_cast<void>(simulation_.Apply(message.destination,
                    {DroneCommand::Kind::Land}));
            mission->second.status = "임무 권한 만료 · 안전 착륙";
            continue;
        }

        if (message.sequence == mission->second.routeSequence)
        {
            // 같은 경로가 다시 오면 비행 상태를 초기화하지 않고 확인 응답만 다시 보낸다.
            static_cast<void>(formationNetwork_->Send(message.destination, leaderId,
                message.sequence, "fleet_route_ack", {}));
            continue;
        }
        if (message.sequence != mission->second.pendingRouteSequence
            || message.sequence < mission->second.routeSequence)
            continue;

        ApplyFleetRouteAssignment(message.destination,
            static_cast<FleetMissionLeg>(route->leg), route->targetIndex,
            route->waypointsMeters, message.sequence, route->assignmentVersion,
            route->assignmentLeaseExpiresAtTick);
        static_cast<void>(formationNetwork_->Send(message.destination, leaderId,
            message.sequence, "fleet_route_ack", {}));
    }

    constexpr std::uint64_t telemetryIntervalTicks = 6U;

    // 관제소는 각 기체에서 직접 받은 상태만 표시한다.
    for (const DroneId droneId : droneIds)
    {
        if (!formationNetwork_->HasRecentNeighbor(droneId, groundControlNodeId))
            continue;
        const auto lastSent = lastGroundControlTelemetrySentAtTick_.find(droneId);
        if (lastSent != lastGroundControlTelemetrySentAtTick_.end()
            && currentTick < lastSent->second + telemetryIntervalTicks)
            continue;
        const auto& state = simulation_.State(droneId);
        const auto mission = fleetReconnaissance_.find(droneId);
        const std::uint8_t leg = mission == fleetReconnaissance_.end()
            ? 0U : static_cast<std::uint8_t>(mission->second.leg);
        const std::string status = mission == fleetReconnaissance_.end()
            ? std::string{} : mission->second.status;
        if (formationNetwork_->SendLatest(droneId, groundControlNodeId,
                nextGroundControlTelemetrySequence_++, "ground_control_telemetry",
                detail::EncodePeerTelemetry(state.positionMeters, state.mode,
                    leg, status, 0U, 0U)))
            lastGroundControlTelemetrySentAtTick_[droneId] = currentTick;
    }

    for (const DroneId droneId : droneIds)
    {
        if (droneId == leaderId)
            continue;
        const auto lastSent = lastTelemetrySentAtTick_.find(droneId);
        if (lastSent != lastTelemetrySentAtTick_.end()
            && currentTick < lastSent->second + telemetryIntervalTicks)
            continue;

        const auto mission = fleetReconnaissance_.find(droneId);
        const std::uint8_t missionLeg = mission == fleetReconnaissance_.end()
            ? 0U : static_cast<std::uint8_t>(mission->second.leg);
        const std::string missionStatus = mission == fleetReconnaissance_.end()
            ? std::string{} : mission->second.status;
        const std::uint64_t assignmentVersion = mission == fleetReconnaissance_.end()
            ? 0U : mission->second.assignmentVersion;
        const std::uint64_t leaseExpiresAtTick = mission == fleetReconnaissance_.end()
            ? 0U : mission->second.assignmentLeaseExpiresAtTick;
        static_cast<void>(formationNetwork_->SendLatest(droneId, leaderId,
            nextTelemetrySequence_++, "drone_telemetry",
            detail::EncodePeerTelemetry(simulation_.State(droneId).positionMeters,
                simulation_.State(droneId).mode, missionLeg, missionStatus,
                assignmentVersion, leaseExpiresAtTick)));
        lastTelemetrySentAtTick_[droneId] = currentTick;
    }

    constexpr std::uint64_t routeRetryIntervalTicks = 60U;
    constexpr std::uint32_t maximumRouteTransmissions = 10U;
    for (auto& [droneId, mission] : fleetReconnaissance_)
    {
        if (droneId == leaderId
            || std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), droneId)
                == fleetDroneIds_.end()
            || mission.pendingRouteSequence == 0U
            || mission.routeTransmissionCount >= maximumRouteTransmissions
            || currentTick < mission.lastRouteTransmissionTick + routeRetryIntervalTicks)
            continue;

        if (formationNetwork_->Send(leaderId, droneId,
                mission.pendingRouteSequence, "fleet_route",
                mission.pendingRoutePayload))
        {
            mission.lastRouteTransmissionTick = currentTick;
            ++mission.routeTransmissionCount;
        }
    }

}

void AeroSimLabApp::AdvanceFormationTransit()
{
    using namespace aerosimlab::simulation;
    if (!formationTransit_.active)
        return;

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.size() < 2U
        || formationTransit_.route.leaderWaypointsMeters.empty())
    {
        formationTransit_.active = false;
        formationTransit_.status = "편대 구성을 유지할 수 없어 임무를 멈췄습니다.";
        return;
    }

    const DroneId leaderId = fleetLeaderId_;
    const DroneState& leader = simulation_.State(leaderId);
    if (leader.mode == FlightMode::Grounded || leader.mode == FlightMode::TakingOff)
        return;
    if (leader.mode == FlightMode::Landing)
    {
        formationTransit_.active = false;
        formationTransit_.status = "리더가 착륙해 편대 이동을 멈췄습니다.";
        return;
    }

    constexpr std::uint64_t peerTelemetryTimeoutTicks = 30U;
    constexpr std::uint64_t peerTelemetryRecoveryTimeoutTicks = 180U;
    for (std::size_t index = 1U; index < droneIds.size(); ++index)
    {
        const auto peer = peerTelemetryAtLeader_.find(droneIds[index]);
        if (peer == peerTelemetryAtLeader_.end())
        {
            formationTransit_.active = false;
            formationTransit_.status = "통신 끊김 · 편대 안전 착륙";
            if (completeFleetMission_.stage
                == CompleteFleetMissionState::Stage::FormationTransit)
            {
                FailCompleteFleetMission(
                    "추종 드론 상태 보고가 끊겨 편대를 안전 착륙시킵니다.");
            }
            else
            {
                for (const DroneId fleetDroneId : droneIds)
                {
                    const FlightMode mode = simulation_.State(fleetDroneId).mode;
                    if (mode != FlightMode::Grounded && mode != FlightMode::Landing)
                        static_cast<void>(simulation_.Apply(fleetDroneId,
                            {DroneCommand::Kind::Land}));
                }
            }
            return;
        }

        const std::uint64_t reportAgeTicks = simulation_.TickCount()
            - peer->second.receivedAtTick;
        if (reportAgeTicks <= peerTelemetryTimeoutTicks)
            continue;

        if (reportAgeTicks <= peerTelemetryTimeoutTicks
                + peerTelemetryRecoveryTimeoutTicks)
        {
            // 편대원 보고가 잠깐 끊기면 리더를 멈춰 연결 회복을 기다린다.
            DroneCommand holdLeader;
            holdLeader.kind = DroneCommand::Kind::SetWaypoint;
            holdLeader.vector = leader.positionMeters;
            static_cast<void>(simulation_.Apply(leaderId, holdLeader));
            formationTransit_.status = "추종 드론 통신 회복 대기 · 편대 이동 일시 정지";
            return;
        }

        formationTransit_.active = false;
        formationTransit_.status = "통신 끊김 · 편대 안전 착륙";
        if (completeFleetMission_.stage
            == CompleteFleetMissionState::Stage::FormationTransit)
        {
            FailCompleteFleetMission(
                "추종 드론과 통신이 복구되지 않아 편대를 안전 착륙시킵니다.");
        }
        else
        {
            for (const DroneId fleetDroneId : droneIds)
            {
                const FlightMode mode = simulation_.State(fleetDroneId).mode;
                if (mode != FlightMode::Grounded && mode != FlightMode::Landing)
                    static_cast<void>(simulation_.Apply(fleetDroneId,
                        {DroneCommand::Kind::Land}));
            }
        }
        return;
    }

    const auto& route = formationTransit_.route.leaderWaypointsMeters;
    formationTransit_.waypointIndex = std::min(
        formationTransit_.waypointIndex, route.size() - 1U);
    const glm::dvec3 waypoint = route[formationTransit_.waypointIndex];
    if (glm::distance(leader.positionMeters, waypoint) <= 3.0
        && formationTransit_.waypointIndex + 1U < route.size())
        ++formationTransit_.waypointIndex;

    const std::size_t nextIndex = formationTransit_.waypointIndex;
    glm::dvec2 direction{0.0, 0.0};
    if (nextIndex + 1U < route.size())
        direction = {route[nextIndex + 1U].x - route[nextIndex].x,
            route[nextIndex + 1U].y - route[nextIndex].y};
    else if (nextIndex > 0U)
        direction = {route[nextIndex].x - route[nextIndex - 1U].x,
            route[nextIndex].y - route[nextIndex - 1U].y};
    else if (!route.empty())
        direction = {route[nextIndex].x - formationTransit_.route.leaderStartMeters.x,
            route[nextIndex].y - formationTransit_.route.leaderStartMeters.y};
    const double heading = glm::length(direction) > 1.0e-6
        ? std::atan2(direction.y, direction.x) : leader.yawRadians;
    const auto offsets = FormationOffsets(
        droneIds.size(), formationTransit_.route.spacingMeters, heading);

    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const DroneId droneId = droneIds[index];
        const ReceivedPeerTelemetry* peerState = index == 0U
            ? nullptr
            : &peerTelemetryAtLeader_.at(droneId);
        const FlightMode mode = index == 0U ? leader.mode : peerState->state.mode;
        if (mode == FlightMode::Grounded || mode == FlightMode::TakingOff)
            continue;
        if (mode == FlightMode::Landing)
        {
            formationTransit_.active = false;
            formationTransit_.status = "추종 드론이 착륙해 편대 이동을 멈췄습니다.";
            return;
        }

        glm::dvec3 target = leader.positionMeters;
        if (index == 0U)
            target = route[nextIndex];
        else
        {
            target.x += offsets[index].x;
            target.y += offsets[index].y;
        }
        DroneCommand command;
        command.kind = DroneCommand::Kind::SetWaypoint;
        command.vector = target;
        if (index == 0U)
        {
            if (simulation_.Apply(droneId, command) != CommandResult::Accepted)
            {
                formationTransit_.active = false;
                formationTransit_.status = "리더 목표를 적용하지 못해 임무를 멈췄습니다.";
                return;
            }
            continue;
        }

        constexpr std::uint64_t targetRefreshTicks = 6U;
        const auto previousTarget = formationTransit_.sentFollowerTargets.find(droneId);
        const auto lastSentTick = formationTransit_.lastTargetSentAtTick.find(droneId);
        const bool targetChanged = previousTarget == formationTransit_.sentFollowerTargets.end()
            || glm::distance(previousTarget->second, target) >= 0.75;
        const bool refreshDue = lastSentTick == formationTransit_.lastTargetSentAtTick.end()
            || simulation_.TickCount() >= lastSentTick->second + targetRefreshTicks;
        if (targetChanged || refreshDue)
        {
            const auto messageId = formationNetwork_->Send(leaderId, droneId,
                formationTransit_.nextMessageSequence++, "formation_waypoint",
                EncodeWaypoint(target));
            if (!messageId)
            {
                formationTransit_.active = false;
                formationTransit_.status = "추종 드론에게 편대 목표를 보내지 못했습니다.";
                return;
            }
            formationTransit_.sentFollowerTargets[droneId] = target;
            formationTransit_.lastTargetSentAtTick[droneId] = simulation_.TickCount();
        }
    }

    const bool leaderArrived = formationTransit_.waypointIndex + 1U == route.size()
        && glm::distance(leader.positionMeters, route.back()) <= 3.0;
    bool followersArrived = true;
    for (std::size_t index = 1U; index < droneIds.size(); ++index)
    {
        const glm::dvec3 target{leader.positionMeters.x + offsets[index].x,
            leader.positionMeters.y + offsets[index].y, leader.positionMeters.z};
        const auto& peer = peerTelemetryAtLeader_.at(droneIds[index]).state;
        if (glm::distance(peer.positionMeters, target) > 4.0)
        {
            followersArrived = false;
            break;
        }
    }
    if (leaderArrived && followersArrived)
    {
        formationTransit_.active = false;
        formationTransit_.status = "목적지 도착 · 편대 유지 중";
        return;
    }
    formationTransit_.status = "이동 중 · 경유점 ";
    formationTransit_.status += std::to_string(formationTransit_.waypointIndex + 1U)
        + "/" + std::to_string(route.size());
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
        communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            communication::VideoRecordingCommand::Start, recordingReply);
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
        communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            communication::VideoRecordingCommand::Stop, recordingReply);
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
        communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            communication::VideoRecordingCommand::Stop, recordingReply);
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
