#include "DroneSimApp.h"
#include "flight_planning/RoutePlanner.h"
#include "graphics/Window.h"
#include <glm/geometric.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace dronesim::simulator
{
namespace
{
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

bool DroneSimApp::FleetIsReadyForMission() const
{
    if (CompleteFleetMissionIsActive() || formationTransit_.IsActive()
        || reconTargets_.empty() || FleetReconnaissanceIsActive())
        return false;

    const auto droneIds = simulation_.DroneIds();
    return !droneIds.empty() && std::all_of(droneIds.begin(), droneIds.end(),
        [this](dronesim::simulation::DroneId id)
        {
            return simulation_.State(id).mode
                == dronesim::simulation::FlightMode::Grounded;
        });
}

bool DroneSimApp::FleetReconnaissanceCanStart() const
{
    return FleetIsReadyForMission();
}

bool DroneSimApp::CompleteFleetMissionCanStart() const
{
    return simulation_.DroneCount() == 4U && FleetIsReadyForMission();
}

bool DroneSimApp::CompleteFleetMissionIsActive() const
{
    using Stage = dronesim::mission::CompleteFleetMissionState::Stage;
    return completeFleetMission_.stage == Stage::FormationTransit
        || completeFleetMission_.stage == Stage::Reconnaissance
        || completeFleetMission_.stage == Stage::Regrouping
        || completeFleetMission_.stage == Stage::Returning;
}

bool DroneSimApp::FleetReconnaissanceIsActive() const
{
    return std::any_of(fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return dronesim::mission::IsFleetMissionInProgress(entry.second.status);
        });
}

bool DroneSimApp::StartCompleteFleetMission()
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

    completeFleetMission_.stage = dronesim::mission::CompleteFleetMissionState::Stage::FormationTransit;
    completeFleetMission_.status = "M01 편대 이동 중";
    return true;
}

std::string DroneSimApp::CompleteFleetMissionStatus() const
{
    return completeFleetMission_.status;
}

bool DroneSimApp::StartFleetReconnaissance()
{
    if (!FleetReconnaissanceCanStart())
        return false;

    std::map<dronesim::simulation::DroneId, glm::dvec3> homePositions;
    for (const auto droneId : simulation_.DroneIds())
        homePositions.emplace(droneId, simulation_.State(droneId).positionMeters);
    return StartFleetReconnaissance(true, homePositions);
}

bool DroneSimApp::StartFleetReconnaissance(
    bool takeOffDrones,
    const std::map<dronesim::simulation::DroneId, glm::dvec3>& homePositions)
{
    using namespace dronesim::simulation;
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
                const auto* report = peerTelemetryAtLeader_.Fresh(
                    id, simulation_.TickCount());
                if (report == nullptr)
                    return false;
                mode = report->state.mode;
            }
            return takeOffDrones
                ? mode == FlightMode::Grounded
                : (mode == FlightMode::Hovering || mode == FlightMode::Flying
                    || mode == FlightMode::FollowingWaypoint);
        });
    if (droneIds.empty() || reconTargets_.empty() || !dronesReady
        || (takeOffDrones && CompleteFleetMissionIsActive()))
        return false;

    std::vector<dronesim::mission::FleetReconnaissanceState> missions;
    missions.reserve(droneIds.size());
    for (std::size_t index = 0; index < droneIds.size(); ++index)
    {
        const std::size_t targetIndex = index % reconTargets_.size();
        const auto& target = reconTargets_[targetIndex];
        const DroneId droneId = droneIds[index];
        const glm::dvec3* start = &simulation_.State(droneId).positionMeters;
        if (!takeOffDrones && droneId != leaderId)
        {
            const auto* report = peerTelemetryAtLeader_.Fresh(
                droneId, simulation_.TickCount());
            if (report == nullptr)
                return false;
            start = &report->state.positionMeters;
        }
        const auto route = dronesim::flight_planning::PlanRoute(
            terrainGrid_, trainingRangeScene_->BuildingObstacles(), *start, target.approachPositionMeters,
            3.0, cameraTransitAltitudeMeters_);
        if (!route)
            return false;

        dronesim::mission::FleetReconnaissanceState mission;
        mission.targetIndex = targetIndex;
        const auto home = homePositions.find(droneIds[index]);
        mission.startPositionMeters = home != homePositions.end()
            ? home->second : *start;
        mission.routeWaypointsMeters.assign(route->begin(), route->end());
        mission.status = takeOffDrones
            ? dronesim::mission::FleetDroneStatus::TakeoffReady
            : dronesim::mission::FleetDroneStatus::Reconnaissance;
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
        const dronesim::mission::FleetMissionLeg leg = missions[index].leg;
        const std::size_t targetIndex = missions[index].targetIndex;
        std::vector<glm::dvec3> route = std::move(missions[index].routeWaypointsMeters);
        missions[index].status = dronesim::mission::FleetDroneStatus::RoutePending;
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
    missionControlOwnsInput_ = true;
    window_->SetTitle("DroneSim | 편대 정찰 임무 진행 중");
    return true;
}

bool DroneSimApp::FleetReturnCanStart() const
{
    if (fleetReconnaissance_.empty() || CompleteFleetMissionIsActive()
        || formationTransit_.IsActive())
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
                    == dronesim::simulation::FlightMode::Grounded
                && mission->second.leg == dronesim::mission::FleetMissionLeg::Reconnaissance
                && !dronesim::mission::IsFleetMissionInProgress(mission->second.status))
                || (mission->second.lowBatteryReturn
                    && mission->second.status
                        == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete
                    && simulation_.State(droneId).mode
                        == dronesim::simulation::FlightMode::Grounded);

        const auto* report = peerTelemetryAtLeader_.Fresh(
            droneId, simulation_.TickCount());
        if (report == nullptr)
            return false;
        if (mission->second.lowBatteryReturn
                    && report->state.missionStatus
                        == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete
            && report->state.mode == dronesim::simulation::FlightMode::Grounded)
            return true;
        const bool finishedReconnaissance = report->state.mode
                == dronesim::simulation::FlightMode::Grounded
            && report->state.missionLeg
                == static_cast<std::uint8_t>(dronesim::mission::FleetMissionLeg::Reconnaissance)
            && !dronesim::mission::IsFleetMissionInProgress(report->state.missionStatus);
        return finishedReconnaissance;
    });
}

bool DroneSimApp::StartFleetReturn()
{
    if (!FleetReturnCanStart())
        return false;

    return StartFleetReturn(true);
}

bool DroneSimApp::StartFleetReturn(bool takeOffDrones)
{
    using namespace dronesim::simulation;

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
            && mission.status
                == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete)
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
            const auto* report = peerTelemetryAtLeader_.Fresh(
                droneId, simulation_.TickCount());
            if (report == nullptr)
                return false;
            currentPosition = report->state.positionMeters;
            mode = report->state.mode;
        }
        const bool ready = takeOffDrones
            ? mode == FlightMode::Grounded
            : (mode == FlightMode::Hovering || mode == FlightMode::Flying
                || mode == FlightMode::FollowingWaypoint);
        if (!ready || mission.leg == dronesim::mission::FleetMissionLeg::Returning)
            return false;
        glm::dvec3 returnApproach = mission.startPositionMeters;
        returnApproach.z += 5.0;
        const auto route = dronesim::flight_planning::PlanRoute(
            terrainGrid_, trainingRangeScene_->BuildingObstacles(), currentPosition,
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
                && mission.status
                    == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete)
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
            && mission.status
                == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete)
            continue;
        mission.leg = dronesim::mission::FleetMissionLeg::Returning;
        mission.status = dronesim::mission::FleetDroneStatus::RoutePending;
        if (!SendFleetRouteAssignment(droneIds[index],
                dronesim::mission::FleetMissionLeg::Returning, mission.targetIndex,
                std::move(returnRoutes[index])))
            return false;
    }
    externalControl_ = true;
    missionControlOwnsInput_ = true;
    window_->SetTitle("DroneSim | 출발 지점으로 복귀 중");
    return true;
}

bool DroneSimApp::StartFleetRally()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;
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
            && mission.status
                == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete;
        if (!alreadyReturnedForLowBattery)
            rallyDroneIds.push_back(droneId);
    }
    if (rallyDroneIds.empty())
    {
        if (!StartFleetReturn(false))
            return false;
        completeFleetMission_.stage = dronesim::mission::CompleteFleetMissionState::Stage::Returning;
        completeFleetMission_.status = "M03 기지로 복귀 중";
        return true;
    }

    glm::dvec3 centerMeters{0.0};
    for (const DroneId droneId : droneIds)
        centerMeters += completeFleetMission_.homePositionsMeters.at(droneId);
    centerMeters /= static_cast<double>(droneIds.size());
    centerMeters.z += 12.0;
    const auto offsets = FormationOffsets(rallyDroneIds.size(), 3.0, 0.0);

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
            const auto* report = peerTelemetryAtLeader_.Fresh(
                droneId, simulation_.TickCount());
            if (report == nullptr
                || report->state.missionStatus
                    != dronesim::mission::FleetDroneStatus::TargetReached)
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
        const auto route = dronesim::flight_planning::PlanRoute(
            terrainGrid_, trainingRangeScene_->BuildingObstacles(), currentPosition, rallyTarget,
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
        mission.leg = dronesim::mission::FleetMissionLeg::Regrouping;
        mission.status = dronesim::mission::FleetDroneStatus::RoutePending;
        if (!SendFleetRouteAssignment(droneId,
                dronesim::mission::FleetMissionLeg::Regrouping, mission.targetIndex,
                std::move(rallyRoutes[index])))
            return false;
    }
    completeFleetMission_.stage = dronesim::mission::CompleteFleetMissionState::Stage::Regrouping;
    completeFleetMission_.status = "M03 재집결 중";
    return true;
}

void DroneSimApp::AdvanceFleetReconnaissance()
{
    using DroneId = dronesim::simulation::DroneId;
    using Command = dronesim::simulation::DroneCommand;
    using FlightMode = dronesim::simulation::FlightMode;
    constexpr std::uint64_t routeDeliveryTimeoutTicks = 600U;

    for (auto& [droneId, mission] : fleetReconnaissance_)
    {
        const bool returningFromTarget =
            (mission.leg == dronesim::mission::FleetMissionLeg::Reconnaissance
                && dronesim::mission::IsFleetMissionInProgress(mission.status))
            || (mission.leg == dronesim::mission::FleetMissionLeg::Reconnaissance
                && mission.status
                    == dronesim::mission::FleetDroneStatus::TargetReached);
        if (returningFromTarget && simulation_.HasBattery(droneId))
        {
            auto& battery = simulation_.Battery(droneId);
            if (!battery.IsReturningForLowBattery()
                && battery.ShouldReturnForLowBattery())
            {
                battery.MarkReturningForLowBattery();

                glm::dvec3 returnApproach = mission.startPositionMeters;
                returnApproach.z += 5.0;
                const auto route = dronesim::flight_planning::PlanRoute(
                    terrainGrid_, trainingRangeScene_->BuildingObstacles(),
                    simulation_.State(droneId).positionMeters, returnApproach,
                    3.0, cameraTransitAltitudeMeters_);
                bool returnStarted = route.has_value();
                if (returnStarted
                    && simulation_.State(droneId).mode == FlightMode::Grounded)
                    returnStarted = simulation_.Apply(droneId,
                        {Command::Kind::TakeOff})
                        == dronesim::simulation::CommandResult::Accepted;
                if (returnStarted)
                {
                    mission.lowBatteryReturn = true;
                    returnStarted = SendFleetRouteAssignment(droneId,
                        dronesim::mission::FleetMissionLeg::Returning, mission.targetIndex,
                        std::vector<glm::dvec3>(route->begin(), route->end()));
                }
                if (!returnStarted)
                {
                    mission.status = dronesim::mission::FleetDroneStatus::BatteryReturnRouteFailed;
                    mission.assignmentLeaseExpiresAtTick = simulation_.TickCount();
                    const auto mode = simulation_.State(droneId).mode;
                    if (mode != FlightMode::Grounded && mode != FlightMode::Landing)
                        static_cast<void>(simulation_.Apply(droneId,
                            {Command::Kind::Land}));
                }
                continue;
            }
        }

        const auto droneStatus = mission.status;
        bool legIsFinished = false;
        switch (mission.leg)
        {
        case dronesim::mission::FleetMissionLeg::Reconnaissance:
            legIsFinished = droneStatus == dronesim::mission::FleetDroneStatus::TargetReached
                || droneStatus == dronesim::mission::FleetDroneStatus::RouteFailed
                || droneStatus == dronesim::mission::FleetDroneStatus::OutsideTargetLanding;
            break;
        case dronesim::mission::FleetMissionLeg::Regrouping:
            legIsFinished = droneStatus == dronesim::mission::FleetDroneStatus::RegroupComplete
                || droneStatus == dronesim::mission::FleetDroneStatus::RegroupFailed;
            break;
        case dronesim::mission::FleetMissionLeg::Returning:
            legIsFinished = droneStatus == dronesim::mission::FleetDroneStatus::ReturnComplete
                || droneStatus == dronesim::mission::FleetDroneStatus::ReturnFailed
                || droneStatus
                    == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete;
            break;
        }
        if (legIsFinished)
            continue;

        if (!mission.routeReceived)
        {
            if (simulation_.TickCount() >= mission.routeSentAtTick
                    + routeDeliveryTimeoutTicks)
                mission.status = dronesim::mission::FleetDroneStatus::RouteFailed;
            continue;
        }

        const auto& state = simulation_.State(droneId);
        if (state.mode == FlightMode::Grounded)
        {
            const auto groundedStatus = mission.status;
            if (groundedStatus == dronesim::mission::FleetDroneStatus::TakeoffReady
                || groundedStatus == dronesim::mission::FleetDroneStatus::ReturnPreparing
                || groundedStatus == dronesim::mission::FleetDroneStatus::RegroupPreparing)
                continue;

            if (mission.leg == dronesim::mission::FleetMissionLeg::Regrouping)
            {
                mission.status = dronesim::mission::FleetDroneStatus::RegroupFailed;
                continue;
            }
            const glm::dvec3& destination = mission.leg == dronesim::mission::FleetMissionLeg::Returning
                ? mission.startPositionMeters
                : reconTargets_[mission.targetIndex].approachPositionMeters;
            const glm::dvec2 horizontalError{
                state.positionMeters.x - destination.x,
                state.positionMeters.y - destination.y};
            if (mission.leg == dronesim::mission::FleetMissionLeg::Returning)
            {
                if (glm::length(horizontalError) > 5.0)
                    mission.status = dronesim::mission::FleetDroneStatus::ReturnFailed;
                else
                    mission.status = mission.lowBatteryReturn
                        ? dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete
                        : dronesim::mission::FleetDroneStatus::ReturnComplete;
            }
            else
                mission.status = glm::length(horizontalError) <= 5.0
                    ? dronesim::mission::FleetDroneStatus::TargetReached
                    : dronesim::mission::FleetDroneStatus::OutsideTargetLanding;
            continue;
        }
        if (state.mode == FlightMode::TakingOff || state.mode == FlightMode::Landing)
            continue;

        if (!mission.waypointCommanded)
        {
            mission.waypointCommanded = true;
            if (mission.leg == dronesim::mission::FleetMissionLeg::Returning)
                mission.status = mission.lowBatteryReturn
                    ? dronesim::mission::FleetDroneStatus::LowBatteryReturning
                    : dronesim::mission::FleetDroneStatus::Returning;
            else if (mission.leg == dronesim::mission::FleetMissionLeg::Regrouping)
                mission.status = dronesim::mission::FleetDroneStatus::Regrouping;
            else
                mission.status = dronesim::mission::FleetDroneStatus::Reconnaissance;
        }
        else if (mission.nextWaypoint >= mission.routeWaypointsMeters.size())
        {
            if (mission.leg == dronesim::mission::FleetMissionLeg::Regrouping)
            {
                mission.status = dronesim::mission::FleetDroneStatus::RegroupComplete;
                continue;
            }
            const Command land{Command::Kind::Land};
            if (simulation_.Apply(droneId, land) == dronesim::simulation::CommandResult::Accepted)
            {
                if (mission.leg == dronesim::mission::FleetMissionLeg::Returning)
                    mission.status = mission.lowBatteryReturn
                        ? dronesim::mission::FleetDroneStatus::LowBatteryLanding
                        : dronesim::mission::FleetDroneStatus::ReturnLanding;
                else
                    mission.status = dronesim::mission::FleetDroneStatus::Landing;
            }
            else
                mission.status = dronesim::mission::FleetDroneStatus::RouteFailed;
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
                != dronesim::simulation::CommandResult::Accepted)
                mission.status = dronesim::mission::FleetDroneStatus::RouteFailed;
        }
    }
}

void DroneSimApp::AdvanceCompleteFleetMission()
{
    using Stage = dronesim::mission::CompleteFleetMissionState::Stage;
    using DroneId = dronesim::simulation::DroneId;
    if (completeFleetMission_.stage == Stage::Idle
        || completeFleetMission_.stage == Stage::Completed
        || completeFleetMission_.stage == Stage::Failed)
        return;

    if (completeFleetMission_.stage == Stage::FormationTransit)
    {
        if (formationTransit_.IsActive())
            return;
        if (formationTransit_.phase != dronesim::mission::FormationTransitState::Phase::Arrived)
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
    std::vector<std::optional<dronesim::mission::FleetMissionReport>> reports;
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
            reports.emplace_back(dronesim::mission::FleetMissionReport{
                simulation_.State(droneId).mode,
                mission->second.leg,
                mission->second.status});
            continue;
        }

        const auto* telemetry = peerTelemetryAtLeader_.Fresh(
            droneId, simulation_.TickCount());
        if (telemetry == nullptr)
        {
            const auto* cached = peerTelemetryAtLeader_.Latest(droneId);
            if (cached != nullptr
                && cached->state.assignmentVersion
                    == mission->second.assignmentVersion
                && cached->state.mode
                    == dronesim::simulation::FlightMode::Grounded)
            {
                const bool returnedAfterBatteryReserve = mission->second.lowBatteryReturn
                    && cached->state.missionLeg
                        == static_cast<std::uint8_t>(dronesim::mission::FleetMissionLeg::Returning)
                    && cached->state.missionStatus
                        == dronesim::mission::FleetDroneStatus::LowBatteryReturnComplete;
                const bool returnedToBase =
                    completeFleetMission_.stage == Stage::Returning
                    && cached->state.missionLeg
                        == static_cast<std::uint8_t>(dronesim::mission::FleetMissionLeg::Returning)
                    && cached->state.missionStatus
                        == dronesim::mission::FleetDroneStatus::ReturnComplete;
                if (returnedAfterBatteryReserve || returnedToBase)
                    telemetry = cached;
            }
        }
        if (telemetry == nullptr)
        {
            reports.emplace_back(std::nullopt);
            continue;
        }
        reports.emplace_back(dronesim::mission::FleetMissionReport{telemetry->state.mode,
            static_cast<dronesim::mission::FleetMissionLeg>(telemetry->state.missionLeg),
            telemetry->state.missionStatus});
    }

    const auto missionLeg = completeFleetMission_.stage == Stage::Reconnaissance
        ? dronesim::mission::FleetMissionLeg::Reconnaissance
        : completeFleetMission_.stage == Stage::Regrouping
            ? dronesim::mission::FleetMissionLeg::Regrouping
            : dronesim::mission::FleetMissionLeg::Returning;
    const auto decision = dronesim::mission::EvaluateFleetMissionProgress(missionLeg, reports);
    if (decision.progress == dronesim::mission::FleetMissionProgress::Failed)
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
    if (decision.progress == dronesim::mission::FleetMissionProgress::Waiting)
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

void DroneSimApp::FailCompleteFleetMission(const std::string& reason)
{
    using FlightMode = dronesim::simulation::FlightMode;
    using Command = dronesim::simulation::DroneCommand;
    completeFleetMission_.stage = dronesim::mission::CompleteFleetMissionState::Stage::Failed;
    completeFleetMission_.status = reason;
    if (formationTransit_.IsActive())
        formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
    for (const auto droneId : simulation_.DroneIds())
    {
        const FlightMode mode = simulation_.State(droneId).mode;
        if (mode != FlightMode::Grounded && mode != FlightMode::Landing)
            static_cast<void>(simulation_.Apply(droneId, {Command::Kind::Land}));
    }
}

bool DroneSimApp::StartFormationTransit()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;

    if (formationTransit_.IsActive() || reconTargets_.empty())
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
    const auto route = PlanFormationRoute(terrainGrid_, trainingRangeScene_->BuildingObstacles(),
        simulation_.State(droneIds.front()).positionMeters, destination,
        droneIds.size(), 1.5, 0.75, 1.0,
        cameraTransitAltitudeMeters_);
    if (!route)
    {
        formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
        formationTransit_.status = "목적지까지 편대 경로를 찾지 못했습니다.";
        return false;
    }

    if (!PrepareFleetNetwork(true))
    {
        formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
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
            formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
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
    formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::InProgress;
    formationTransit_.status = "출발 준비 · V자 대형 · "
        + reconTargets_.back().displayName;
    externalControl_ = true;
    missionControlOwnsInput_ = true;
    pendingControllerAction_.reset();
    pendingPilotCommand_.reset();
#ifdef __EMSCRIPTEN__
    webPilotAxes_.reset();
#endif
    if (window_)
        window_->SetTitle("DroneSim | 편대 이동 임무 진행 중");
    return true;
}

bool DroneSimApp::PrepareFleetNetwork(bool reset)
{
    if (formationNetwork_ && !reset)
        return true;

    InitializeFleetLeadership();

    nextTelemetrySequence_ = 1U;
    lastTelemetrySentAtTick_.clear();
    peerTelemetryAtLeader_.Clear();

    communication::VirtualMeshSettings settings;
    settings.maximumLinkRangeMeters = 130.0;
    const auto droneIds = simulation_.DroneIds();
    formationNetwork_ = std::make_unique<
        communication::VirtualMeshNetwork>(droneSocketTransport_, settings);

    formationRadioObstacles_.clear();
    const auto& buildings = trainingRangeScene_->BuildingObstacles();
    formationRadioObstacles_.reserve(buildings.size());
    for (const dronesim::flight_planning::BuildingObstacle& building : buildings)
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
    nodes.reserve(droneIds.size());
    for (const auto droneId : droneIds)
    {
        nodes.push_back({droneId, simulation_.State(droneId).positionMeters});
    }
    formationNetwork_->UpdateNodes(nodes, formationRadioObstacles_);
    formationTransit_.lastNetworkNodeUpdateTick = simulation_.TickCount();
    return !nodes.empty();
}

void DroneSimApp::InitializeFleetLeadership()
{
    fleetDroneIds_ = simulation_.DroneIds();
    fleetReconnaissance_.clear();
    fleetLeaderId_ = fleetDroneIds_.empty() ? 0U : fleetDroneIds_.front();
}

bool DroneSimApp::SendFleetRouteAssignment(
    dronesim::simulation::DroneId droneId,
    dronesim::mission::FleetMissionLeg leg,
    std::size_t targetIndex,
    std::vector<glm::dvec3> routeWaypointsMeters)
{
    if (!formationNetwork_ || reconTargets_.empty()
        || targetIndex >= reconTargets_.size())
        return false;

    auto mission = fleetReconnaissance_.find(droneId);
    if (mission == fleetReconnaissance_.end() || fleetDroneIds_.empty())
        return false;

    const std::uint64_t assignmentVersion = nextMissionAssignmentVersion_++;
    std::uint64_t leaseExpiresAtTick = 0U;
    if (leg == dronesim::mission::FleetMissionLeg::Reconnaissance)
    {
        glm::dvec3 startPositionMeters = simulation_.State(droneId).positionMeters;
        if (droneId != fleetLeaderId_)
        {
            if (const auto* report = peerTelemetryAtLeader_.Fresh(
                    droneId, simulation_.TickCount()))
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

    const auto messageId = formationNetwork_->Send(
        fleetLeaderId_, droneId,
        sequence, "fleet_route", payload);
    if (!messageId)
        return false;

    mission->second.pendingRouteSequence = sequence;
    mission->second.assignmentVersion = assignmentVersion;
    mission->second.assignmentLeaseExpiresAtTick = leaseExpiresAtTick;
    mission->second.pendingRoutePayload = payload;
    mission->second.routeSentAtTick = simulation_.TickCount();
    mission->second.lastRouteTransmissionTick = simulation_.TickCount();
    mission->second.routeTransmissionCount = 1U;
    mission->second.routeReceived = false;
    mission->second.routeWaypointsMeters.clear();
    mission->second.nextWaypoint = 0U;
    mission->second.waypointCommanded = false;
    mission->second.status = dronesim::mission::FleetDroneStatus::RoutePending;
    return true;
}

void DroneSimApp::ApplyFleetRouteAssignment(
    dronesim::simulation::DroneId droneId,
    dronesim::mission::FleetMissionLeg leg,
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
    mission->second.routeSentAtTick = simulation_.TickCount();
    mission->second.routeWaypointsMeters = std::move(routeWaypointsMeters);
    mission->second.nextWaypoint = 0U;
    mission->second.routeReceived = true;
    mission->second.waypointCommanded = false;

    auto mode = simulation_.State(droneId).mode;
    if (leg == dronesim::mission::FleetMissionLeg::Reconnaissance
        && mode == dronesim::simulation::FlightMode::Grounded)
    {
        if (simulation_.Apply(droneId,
                {dronesim::simulation::DroneCommand::Kind::TakeOff})
            != dronesim::simulation::CommandResult::Accepted)
        {
            mission->second.status = dronesim::mission::FleetDroneStatus::RouteFailed;
            return;
        }
        mode = simulation_.State(droneId).mode;
    }
    const bool waitingForTakeoff = mode == dronesim::simulation::FlightMode::Grounded
        || mode == dronesim::simulation::FlightMode::TakingOff;
    switch (leg)
    {
    case dronesim::mission::FleetMissionLeg::Reconnaissance:
        mission->second.status = waitingForTakeoff
            ? dronesim::mission::FleetDroneStatus::TakeoffReady
            : dronesim::mission::FleetDroneStatus::Reconnaissance;
        break;
    case dronesim::mission::FleetMissionLeg::Regrouping:
        mission->second.status = waitingForTakeoff
            ? dronesim::mission::FleetDroneStatus::RegroupPreparing
            : dronesim::mission::FleetDroneStatus::Regrouping;
        break;
    case dronesim::mission::FleetMissionLeg::Returning:
        mission->second.status = waitingForTakeoff
            ? dronesim::mission::FleetDroneStatus::ReturnPreparing
            : dronesim::mission::FleetDroneStatus::Returning;
        break;
    }
}

void DroneSimApp::AdvanceFleetNetwork()
{
    using dronesim::simulation::DroneCommand;
    using dronesim::simulation::DroneId;

    if (!formationNetwork_ || (!formationTransit_.IsActive()
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
        nodes.reserve(allDroneIds.size());
        for (const DroneId droneId : allDroneIds)
        {
            const auto& drone = simulation_.State(droneId);
            nodes.push_back({droneId, drone.positionMeters});
        }
        formationNetwork_->UpdateNodes(nodes, formationRadioObstacles_);
        formationTransit_.lastNetworkNodeUpdateTick = currentTick;
    }

    formationNetwork_->Advance(currentTick);
    DroneId leaderId = fleetLeaderId_;
    for (const auto& delivered : formationNetwork_->DeliveredMessages())
    {
        const auto& message = delivered.message;
        if (message.kind == "drone_telemetry"
            && message.destination == leaderId && message.source != leaderId)
        {
            if (std::find(fleetDroneIds_.begin(), fleetDroneIds_.end(), message.source)
                == fleetDroneIds_.end())
                continue;
            const auto telemetry = dronesim::mission::DecodePeerTelemetry(message.payload);
            if (telemetry)
                static_cast<void>(peerTelemetryAtLeader_.Record(message.source,
                    *telemetry, delivered.deliveredAtTick, message.sequence));
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

        if (message.kind == "formation_waypoint" && formationTransit_.IsActive())
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
                != dronesim::simulation::CommandResult::Accepted)
            {
                formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
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
            || route->targetIndex >= reconTargets_.size())
        {
            if (mission != fleetReconnaissance_.end())
                mission->second.status = dronesim::mission::FleetDroneStatus::RouteFailed;
            continue;
        }

        if (route->assignmentLeaseExpiresAtTick != 0U
            && currentTick >= route->assignmentLeaseExpiresAtTick)
        {
            const auto mode = simulation_.State(message.destination).mode;
            if (mode != dronesim::simulation::FlightMode::Grounded
                && mode != dronesim::simulation::FlightMode::Landing)
                static_cast<void>(simulation_.Apply(message.destination,
                    {DroneCommand::Kind::Land}));
            mission->second.status = dronesim::mission::FleetDroneStatus::AssignmentExpiredLanding;
            continue;
        }

        if (message.sequence == mission->second.routeSequence)
        {
            // 같은 경로가 다시 오면 비행 상태를 초기화하지 않고 확인 응답만 다시 보낸다.
            static_cast<void>(formationNetwork_->Send(
                message.destination, leaderId,
                message.sequence, "fleet_route_ack", {}));
            continue;
        }
        if (message.sequence != mission->second.pendingRouteSequence
            || message.sequence < mission->second.routeSequence)
            continue;

        ApplyFleetRouteAssignment(message.destination,
            static_cast<dronesim::mission::FleetMissionLeg>(route->leg), route->targetIndex,
            route->waypointsMeters, message.sequence, route->assignmentVersion,
            route->assignmentLeaseExpiresAtTick);
        static_cast<void>(formationNetwork_->Send(
            message.destination, leaderId,
            message.sequence, "fleet_route_ack", {}));
    }

    constexpr std::uint64_t telemetryIntervalTicks = 6U;

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
        const dronesim::mission::FleetDroneStatus missionStatus =
            mission == fleetReconnaissance_.end()
            ? dronesim::mission::FleetDroneStatus::Other : mission->second.status;
        const std::uint64_t assignmentVersion = mission == fleetReconnaissance_.end()
            ? 0U : mission->second.assignmentVersion;
        const auto& state = simulation_.State(droneId);
        static_cast<void>(formationNetwork_->Send(
            droneId, leaderId,
            nextTelemetrySequence_++, "drone_telemetry",
            dronesim::mission::EncodePeerTelemetry(state.positionMeters, state.mode,
                missionLeg, missionStatus,
                assignmentVersion)));
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

void DroneSimApp::AdvanceFormationTransit()
{
    using namespace dronesim::simulation;
    using namespace dronesim::flight_planning;
    if (!formationTransit_.IsActive())
        return;

    const std::vector<DroneId>& droneIds = fleetDroneIds_;
    if (droneIds.size() < 2U
        || formationTransit_.route.leaderWaypointsMeters.empty())
    {
        formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
        formationTransit_.status = "편대 구성을 유지할 수 없어 임무를 멈췄습니다.";
        return;
    }

    const DroneId leaderId = fleetLeaderId_;
    const DroneState& leader = simulation_.State(leaderId);
    if (leader.mode == FlightMode::Grounded || leader.mode == FlightMode::TakingOff)
        return;
    if (leader.mode == FlightMode::Landing)
    {
        formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
        formationTransit_.status = "리더가 착륙해 편대 이동을 멈췄습니다.";
        return;
    }

    constexpr std::uint64_t peerTelemetryTimeoutTicks = 30U;
    constexpr std::uint64_t peerTelemetryRecoveryTimeoutTicks = 180U;
    for (std::size_t index = 1U; index < droneIds.size(); ++index)
    {
        const auto* peer = peerTelemetryAtLeader_.Latest(droneIds[index]);
        if (peer == nullptr)
        {
            formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
            formationTransit_.status = "통신 끊김 · 편대 안전 착륙";
            if (completeFleetMission_.stage
                == dronesim::mission::CompleteFleetMissionState::Stage::FormationTransit)
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
            - peer->receivedAtTick;
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

        formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
        formationTransit_.status = "통신 끊김 · 편대 안전 착륙";
        if (completeFleetMission_.stage
            == dronesim::mission::CompleteFleetMissionState::Stage::FormationTransit)
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
        const auto* peerState = index == 0U
            ? nullptr
            : peerTelemetryAtLeader_.Latest(droneId);
        if (index != 0U && peerState == nullptr)
        {
            formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
            formationTransit_.status = "추종 드론 상태 보고를 기다리는 중입니다.";
            return;
        }
        const FlightMode mode = index == 0U ? leader.mode : peerState->state.mode;
        if (mode == FlightMode::Grounded || mode == FlightMode::TakingOff)
            continue;
        if (mode == FlightMode::Landing)
        {
            formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
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
                formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
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
            const auto messageId = formationNetwork_->Send(
                leaderId, droneId,
                formationTransit_.nextMessageSequence++, "formation_waypoint",
                EncodeWaypoint(target));
            if (!messageId)
            {
                formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Failed;
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
        const auto* peer = peerTelemetryAtLeader_.Latest(droneIds[index]);
        if (peer == nullptr)
        {
            followersArrived = false;
            break;
        }
        if (glm::distance(peer->state.positionMeters, target) > 4.0)
        {
            followersArrived = false;
            break;
        }
    }
    if (leaderArrived && followersArrived)
    {
        formationTransit_.phase = dronesim::mission::FormationTransitState::Phase::Arrived;
        formationTransit_.status = "목적지 도착 · 편대 유지 중";
        return;
    }
    formationTransit_.status = "이동 중 · 경유점 ";
    formationTransit_.status += std::to_string(formationTransit_.waypointIndex + 1U)
        + "/" + std::to_string(route.size());
}

}
