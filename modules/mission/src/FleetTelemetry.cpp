#include "FleetTelemetry.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace dronesim::mission
{
bool FleetTelemetryStore::Record(simulation::DroneId droneId,
    const PeerTelemetry& state,
    std::uint64_t receivedAtTick,
    std::uint32_t sequence)
{
    auto& latest = latestByDrone_[droneId];
    if (sequence <= latest.sequence)
        return false;

    latest = {state, receivedAtTick, sequence};
    return true;
}

const ReceivedPeerTelemetry* FleetTelemetryStore::Latest(
    simulation::DroneId droneId) const noexcept
{
    const auto found = latestByDrone_.find(droneId);
    return found == latestByDrone_.end() ? nullptr : &found->second;
}

const ReceivedPeerTelemetry* FleetTelemetryStore::Fresh(
    simulation::DroneId droneId,
    std::uint64_t currentTick) const noexcept
{
    constexpr std::uint64_t timeoutTicks = 30U;
    const ReceivedPeerTelemetry* latest = Latest(droneId);
    if (latest == nullptr || currentTick > latest->receivedAtTick + timeoutTicks)
        return nullptr;
    return latest;
}

void FleetTelemetryStore::Clear() noexcept
{
    latestByDrone_.clear();
}

std::string EncodePeerTelemetry(
    const glm::dvec3& positionMeters,
    simulation::FlightMode mode,
    std::uint8_t missionLeg,
    FleetDroneStatus missionStatus,
    std::uint64_t assignmentVersion)
{
    std::ostringstream payload;
    payload << std::setprecision(17) << positionMeters.x << ' '
        << positionMeters.y << ' ' << positionMeters.z << ' '
        << simulation::ToString(mode) << ' '
        << static_cast<unsigned int>(missionLeg) << ' '
        << static_cast<unsigned int>(missionStatus) << ' '
        << assignmentVersion;
    return payload.str();
}

std::optional<PeerTelemetry> DecodePeerTelemetry(const std::string& payload)
{
    std::istringstream values(payload);
    PeerTelemetry telemetry;
    std::string modeName;
    unsigned int missionLeg = 0;
    unsigned int missionStatus = 0;
    if (!(values >> telemetry.positionMeters.x >> telemetry.positionMeters.y
            >> telemetry.positionMeters.z >> modeName >> missionLeg
            >> missionStatus >> telemetry.assignmentVersion)
        || missionLeg > static_cast<unsigned int>(FleetMissionLeg::Returning)
        || missionStatus > static_cast<unsigned int>(FleetDroneStatus::ReturnFailed)
        || !std::isfinite(telemetry.positionMeters.x)
        || !std::isfinite(telemetry.positionMeters.y)
        || !std::isfinite(telemetry.positionMeters.z))
        return std::nullopt;

    values >> std::ws;
    if (!values.eof())
        return std::nullopt;

    const auto mode = simulation::TryParseFlightMode(modeName);
    if (!mode)
        return std::nullopt;

    telemetry.mode = *mode;
    telemetry.missionLeg = static_cast<std::uint8_t>(missionLeg);
    telemetry.missionStatus = static_cast<FleetDroneStatus>(missionStatus);
    return telemetry;
}
}
