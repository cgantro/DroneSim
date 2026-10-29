#pragma once

#include "FleetMissionProgress.h"
#include "simulation_core/DroneModel.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace aerosimlab::simulator::detail
{
struct PeerTelemetry
{
    glm::dvec3 positionMeters{0.0};
    simulation::FlightMode mode = simulation::FlightMode::Grounded;
    std::uint8_t missionLeg = 0;
    FleetDroneStatus missionStatus = FleetDroneStatus::Other;
    std::uint64_t assignmentVersion = 0;
};

[[nodiscard]] std::string EncodePeerTelemetry(
    const glm::dvec3& positionMeters,
    simulation::FlightMode mode,
    std::uint8_t missionLeg,
    FleetDroneStatus missionStatus,
    std::uint64_t assignmentVersion);

[[nodiscard]] std::optional<PeerTelemetry> DecodePeerTelemetry(
    const std::string& payload);
}
