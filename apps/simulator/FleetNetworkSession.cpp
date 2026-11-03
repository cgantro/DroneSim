#include "FleetNetworkSession.h"

#include <algorithm>

namespace dronesim::simulator
{
bool FleetNetworkSession::Prepare(
    std::span<const communication::NetworkNodeSnapshot> nodes,
    std::span<const flight_planning::BuildingObstacle> buildings,
    communication::VirtualMeshSettings settings,
    bool reset)
{
    if (link_ && !reset)
        return true;

    SetBuildingObstacles(buildings);
    link_ = std::make_unique<communication::VirtualMeshNetwork>(transport_, settings);
    UpdateNodes(nodes);
    return !nodes.empty();
}

void FleetNetworkSession::UpdateNodes(
    std::span<const communication::NetworkNodeSnapshot> nodes)
{
    if (link_)
        link_->UpdateNodes(nodes, obstacles_);
}

bool FleetNetworkSession::IsReady() const noexcept
{
    return link_ != nullptr;
}

communication::VirtualMeshNetwork* FleetNetworkSession::VirtualNetwork() noexcept
{
    return link_.get();
}

const communication::VirtualMeshNetwork* FleetNetworkSession::VirtualNetwork() const noexcept
{
    return link_.get();
}

const communication::DroneSocketTransport& FleetNetworkSession::SocketTransport() const noexcept
{
    return transport_;
}

void FleetNetworkSession::SetBuildingObstacles(
    std::span<const flight_planning::BuildingObstacle> buildings)
{
    obstacles_.clear();
    obstacles_.reserve(buildings.size());
    for (const auto& building : buildings)
    {
        if (building.outlineMeters.size() < 3U || building.heightMeters <= 0.0)
            continue;

        const double groundHeight = building.groundElevationsMeters.empty()
            ? 0.0
            : *std::min_element(building.groundElevationsMeters.begin(),
                building.groundElevationsMeters.end());
        obstacles_.push_back({building.outlineMeters, groundHeight,
            groundHeight + building.heightMeters});
    }
}
}
