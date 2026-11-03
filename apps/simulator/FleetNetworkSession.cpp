#include "FleetNetworkSession.h"

#include <algorithm>
#include <utility>

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

std::optional<std::uint64_t> FleetNetworkSession::Send(
    communication::NetworkNodeId source,
    communication::NetworkNodeId destination,
    std::uint32_t sequence, std::string kind, std::string payload)
{
    if (!link_)
        return std::nullopt;
    return link_->Send(source, destination, sequence,
        std::move(kind), std::move(payload));
}

std::span<const communication::DeliveredMeshMessage>
FleetNetworkSession::DeliveredMessages() const noexcept
{
    return link_ ? link_->DeliveredMessages()
                 : std::span<const communication::DeliveredMeshMessage>{};
}

void FleetNetworkSession::Advance(std::uint64_t tick)
{
    if (link_)
        link_->Advance(tick);
}

void FleetNetworkSession::SetRadioEnabled(
    communication::NetworkNodeId node, bool enabled)
{
    if (link_)
        link_->SetNodeRadioEnabled(node, enabled);
}

bool FleetNetworkSession::IsRadioEnabled(
    communication::NetworkNodeId node) const noexcept
{
    return link_ && link_->IsNodeRadioEnabled(node);
}

bool FleetNetworkSession::HasDirectLink(
    communication::NetworkNodeId first,
    communication::NetworkNodeId second) const noexcept
{
    return link_ && link_->HasDirectLink(first, second);
}

const communication::VirtualMeshMetrics& FleetNetworkSession::Metrics() const noexcept
{
    static const communication::VirtualMeshMetrics emptyMetrics{};
    return link_ ? link_->Metrics() : emptyMetrics;
}

std::span<const communication::MeshEvent> FleetNetworkSession::Events() const noexcept
{
    return link_ ? link_->Events() : std::span<const communication::MeshEvent>{};
}

std::optional<communication::SocketPort> FleetNetworkSession::PortFor(
    communication::NetworkNodeId node) const noexcept
{
    return transport_.PortFor(node);
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
