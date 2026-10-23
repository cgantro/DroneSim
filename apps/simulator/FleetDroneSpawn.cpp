#include "FleetDroneSpawn.h"

#include "simulation_core/QuadrotorFlightController.h"

#include <stdexcept>

namespace aerosimlab::simulator
{
std::optional<detail::FleetDroneSpawnPose> detail::FindFleetDroneSpawnPose(
    const terrain::TerrainGrid& terrain,
    const glm::dvec3& primaryPositionMeters,
    std::size_t slot)
{
    const double eastMeters = static_cast<double>(slot % 3U + 1U) * 5.0;
    const std::size_t row = slot / 3U;
    const double northMeters = static_cast<double>(row) * 5.0;
    const glm::dvec2 spawnPosition{
        primaryPositionMeters.x + eastMeters,
        primaryPositionMeters.y + northMeters};
    double groundHeightMeters = 0.0;
    try
    {
        groundHeightMeters = terrain.HeightAtLocalMeters(spawnPosition);
    }
    catch (const std::out_of_range&)
    {
        return std::nullopt;
    }

    return FleetDroneSpawnPose{
        {spawnPosition.x,
            spawnPosition.y,
            groundHeightMeters
                + aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
        groundHeightMeters};
}
}
