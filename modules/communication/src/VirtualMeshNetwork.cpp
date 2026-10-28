#include "communication/VirtualMeshNetwork.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <glm/geometric.hpp>

namespace aerosimlab::communication
{
namespace
{
constexpr double geometryTolerance = 1.0e-9;

std::pair<NetworkNodeId, NetworkNodeId> MakePair(
    NetworkNodeId first, NetworkNodeId second)
{
    return first < second ? std::pair{first, second} : std::pair{second, first};
}

double Cross(const glm::dvec2& first, const glm::dvec2& second)
{
    return first.x * second.y - first.y * second.x;
}

bool PointInside(const glm::dvec2& point,
    const std::vector<glm::dvec2>& polygon)
{
    bool inside = false;
    for (std::size_t index = 0, previous = polygon.size() - 1U;
         index < polygon.size(); previous = index++)
    {
        const auto& first = polygon[index];
        const auto& second = polygon[previous];
        const bool crosses = (first.y > point.y) != (second.y > point.y);
        if (crosses && point.x < (second.x - first.x)
                * (point.y - first.y) / (second.y - first.y) + first.x)
            inside = !inside;
    }
    return inside;
}

std::optional<double> SegmentIntersectionParameter(
    const glm::dvec2& start, const glm::dvec2& end,
    const glm::dvec2& edgeStart, const glm::dvec2& edgeEnd)
{
    const glm::dvec2 ray = end - start;
    const glm::dvec2 edge = edgeEnd - edgeStart;
    const double denominator = Cross(ray, edge);
    if (std::abs(denominator) <= geometryTolerance)
        return std::nullopt;

    const double rayParameter = Cross(edgeStart - start, edge) / denominator;
    const double edgeParameter = Cross(edgeStart - start, ray) / denominator;
    if (rayParameter < -geometryTolerance || rayParameter > 1.0 + geometryTolerance
        || edgeParameter < -geometryTolerance || edgeParameter > 1.0 + geometryTolerance)
        return std::nullopt;
    return std::clamp(rayParameter, 0.0, 1.0);
}
}

VirtualMeshNetwork::VirtualMeshNetwork(VirtualMeshSettings settings)
    : settings_(settings)
    , randomState_(settings.randomSeed)
{
    if (!std::isfinite(settings_.maximumLinkRangeMeters)
        || settings_.maximumLinkRangeMeters <= 0.0
        || settings_.helloIntervalTicks == 0U
        || settings_.neighborTimeoutTicks < settings_.helloIntervalTicks
        || settings_.routeLifetimeTicks == 0U || settings_.maximumHops == 0U
        || !std::isfinite(settings_.packetLossProbability)
        || settings_.packetLossProbability < 0.0
        || settings_.packetLossProbability > 1.0)
        throw std::invalid_argument("가상 통신망 설정값을 확인해 주세요.");
    if (randomState_ == 0U)
        randomState_ = 0x9E3779B9U;
}

void VirtualMeshNetwork::UpdateNodes(
    std::span<const NetworkNodeSnapshot> nodes,
    std::span<const RadioObstacle> obstacles)
{
    std::map<NetworkNodeId, NetworkNodeSnapshot> updated;
    for (const NetworkNodeSnapshot& node : nodes)
    {
        if (!std::isfinite(node.positionMeters.x)
            || !std::isfinite(node.positionMeters.y)
            || !std::isfinite(node.positionMeters.z))
            throw std::invalid_argument("통신 노드 위치는 유한한 좌표여야 합니다.");
        if (!updated.emplace(node.id, node).second)
            throw std::invalid_argument("통신 노드 ID가 중복되었습니다.");
    }

    nodes_ = std::move(updated);
    obstacles_.assign(obstacles.begin(), obstacles.end());
    for (auto iterator = neighbors_.begin(); iterator != neighbors_.end();)
    {
        if (!nodes_.contains(iterator->first))
        {
            iterator = neighbors_.erase(iterator);
            continue;
        }
        std::erase_if(iterator->second, [this, source = iterator->first](const auto& entry)
        {
            return !nodes_.contains(entry.first) || !LinkAvailable(source, entry.first);
        });
        ++iterator;
    }
    for (auto iterator = routes_.begin(); iterator != routes_.end();)
    {
        if (!nodes_.contains(iterator->first))
        {
            iterator = routes_.erase(iterator);
            continue;
        }
        std::erase_if(iterator->second,
            [this, source = iterator->first](const auto& entry)
        {
            return !nodes_.contains(entry.second.nextHop)
                || !LinkAvailable(source, entry.second.nextHop);
        });
        ++iterator;
    }
}

void VirtualMeshNetwork::SetLinkDisabled(
    NetworkNodeId first, NetworkNodeId second, bool disabled)
{
    if (first == second)
        throw std::invalid_argument("한 노드와 자기 자신 사이에는 링크 장애를 설정할 수 없습니다.");
    const NodePair pair = MakePair(first, second);
    if (disabled)
        disabledLinks_.insert(pair);
    else
        disabledLinks_.erase(pair);

    if (disabled)
    {
        for (auto& [node, table] : routes_)
        {
            std::erase_if(table, [pair, node](const auto& entry)
            {
                return MakePair(node, entry.second.nextHop) == pair;
            });
        }
        for (auto& [node, knownNeighbors] : neighbors_)
        {
            if (node == first)
                static_cast<void>(knownNeighbors.erase(second));
            else if (node == second)
                static_cast<void>(knownNeighbors.erase(first));
        }
    }
}

void VirtualMeshNetwork::SetNodeRadioEnabled(NetworkNodeId node, bool enabled)
{
    if (enabled)
        radioDisabledNodes_.erase(node);
    else
        radioDisabledNodes_.insert(node);
    for (auto& [source, table] : routes_)
    {
        static_cast<void>(source);
        std::erase_if(table, [node](const auto& entry)
        {
            return entry.first == node || entry.second.nextHop == node;
        });
    }
    if (!enabled)
    {
        neighbors_.erase(node);
        for (auto& [source, table] : neighbors_)
        {
            static_cast<void>(table.erase(node));
            static_cast<void>(source);
        }
    }
}

std::optional<std::uint64_t> VirtualMeshNetwork::Send(
    NetworkNodeId source, NetworkNodeId destination,
    std::uint32_t sequence, std::string kind, std::string payload)
{
    if (!nodes_.contains(source) || !nodes_.contains(destination)
        || source == destination || kind.empty())
        return std::nullopt;

    VirtualMeshMessage message{
        .id = nextMessageId_++,
        .source = source,
        .destination = destination,
        .sequence = sequence,
        .kind = std::move(kind),
        .payload = std::move(payload)};
    const auto route = routes_[source].find(destination);
    if (route != routes_[source].end() && route->second.expiresAtTick > currentTick_
        && IsKnownNeighbor(source, route->second.nextHop))
    {
        Packet packet;
        packet.type = Packet::Type::Data;
        packet.message = message;
        packet.remainingHops = settings_.maximumHops;
        packet.message.route.push_back(source);
        ForwardData(source, std::move(packet));
    }
    else
    {
        pendingMessages_[{source, destination}].push_back(std::move(message));
        const NodePair pair{source, destination};
        if (!activeRequests_.contains(pair))
            StartRouteDiscovery(source, destination);
    }
    return nextMessageId_ - 1U;
}

void VirtualMeshNetwork::Advance(std::uint64_t tick)
{
    if (tick < currentTick_)
        throw std::invalid_argument("가상 통신망 시간은 뒤로 갈 수 없습니다.");
    currentTick_ = tick;
    deliveredMessages_.clear();

    if (tick >= nextHelloTick_)
    {
        SendHelloMessages();
        nextHelloTick_ = tick + settings_.helloIntervalTicks;
    }

    while (!transmissions_.empty())
    {
        const auto next = std::min_element(transmissions_.begin(), transmissions_.end(),
            [](const Transmission& first, const Transmission& second)
            {
                return first.dueTick < second.dueTick
                    || (first.dueTick == second.dueTick && first.order < second.order);
            });
        if (next->dueTick > tick)
            break;
        Transmission transmission = std::move(*next);
        transmissions_.erase(next);
        ProcessTransmission(std::move(transmission));
    }
    ExpireOldState();
}

std::span<const DeliveredMeshMessage>
VirtualMeshNetwork::DeliveredMessages() const noexcept
{
    return deliveredMessages_;
}

const VirtualMeshMetrics& VirtualMeshNetwork::Metrics() const noexcept
{
    return metrics_;
}

std::optional<NetworkNodeId> VirtualMeshNetwork::NextHop(
    NetworkNodeId source, NetworkNodeId destination) const noexcept
{
    const auto sourceRoutes = routes_.find(source);
    if (sourceRoutes == routes_.end())
        return std::nullopt;
    const auto route = sourceRoutes->second.find(destination);
    if (route == sourceRoutes->second.end() || route->second.expiresAtTick <= currentTick_)
        return std::nullopt;
    return route->second.nextHop;
}

bool VirtualMeshNetwork::LinkAvailable(
    NetworkNodeId sender, NetworkNodeId receiver) const
{
    const auto senderNode = nodes_.find(sender);
    const auto receiverNode = nodes_.find(receiver);
    if (senderNode == nodes_.end() || receiverNode == nodes_.end()
        || senderNode->second.powered == false || receiverNode->second.powered == false
        || !senderNode->second.canTransmit || !receiverNode->second.canReceive
        || radioDisabledNodes_.contains(sender) || radioDisabledNodes_.contains(receiver)
        || disabledLinks_.contains(MakePair(sender, receiver)))
        return false;
    if (glm::distance(senderNode->second.positionMeters,
            receiverNode->second.positionMeters) > settings_.maximumLinkRangeMeters)
        return false;
    return HasLineOfSight(senderNode->second.positionMeters,
        receiverNode->second.positionMeters);
}

bool VirtualMeshNetwork::HasLineOfSight(
    const glm::dvec3& start, const glm::dvec3& end) const
{
    const glm::dvec2 horizontalStart{start.x, start.y};
    const glm::dvec2 horizontalEnd{end.x, end.y};
    for (const RadioObstacle& obstacle : obstacles_)
    {
        if (obstacle.footprintMeters.size() < 3U
            || !std::isfinite(obstacle.bottomMeters)
            || !std::isfinite(obstacle.topMeters)
            || obstacle.topMeters <= obstacle.bottomMeters)
            continue;

        std::vector<double> crossings{0.0, 1.0};
        for (std::size_t index = 0; index < obstacle.footprintMeters.size(); ++index)
        {
            const auto parameter = SegmentIntersectionParameter(horizontalStart,
                horizontalEnd, obstacle.footprintMeters[index],
                obstacle.footprintMeters[(index + 1U) % obstacle.footprintMeters.size()]);
            if (parameter)
                crossings.push_back(*parameter);
        }
        std::sort(crossings.begin(), crossings.end());
        crossings.erase(std::unique(crossings.begin(), crossings.end(),
            [](double first, double second)
            {
                return std::abs(first - second) <= geometryTolerance;
            }), crossings.end());

        for (std::size_t index = 1; index < crossings.size(); ++index)
        {
            const double parameter = std::midpoint(crossings[index - 1U], crossings[index]);
            const glm::dvec2 point = glm::mix(horizontalStart, horizontalEnd, parameter);
            const double altitude = std::lerp(start.z, end.z, parameter);
            if (PointInside(point, obstacle.footprintMeters)
                && altitude >= obstacle.bottomMeters && altitude <= obstacle.topMeters)
                return false;
        }
        for (const double parameter : crossings)
        {
            const glm::dvec2 point = glm::mix(horizontalStart, horizontalEnd, parameter);
            const double altitude = std::lerp(start.z, end.z, parameter);
            if (PointInside(point, obstacle.footprintMeters)
                && altitude >= obstacle.bottomMeters && altitude <= obstacle.topMeters)
                return false;
        }
    }
    return true;
}

void VirtualMeshNetwork::SendHelloMessages()
{
    for (const auto& [source, node] : nodes_)
    {
        static_cast<void>(node);
        Packet packet;
        packet.type = Packet::Type::Hello;
        packet.sender = source;
        for (const auto& [receiver, receiverNode] : nodes_)
        {
            static_cast<void>(receiverNode);
            if (source == receiver || !LinkAvailable(source, receiver))
                continue;
            ++metrics_.helloMessages;
            Transmit(source, receiver, packet);
        }
    }
}

void VirtualMeshNetwork::Transmit(
    NetworkNodeId sender, NetworkNodeId receiver, Packet packet)
{
    if (!LinkAvailable(sender, receiver)
        || (packet.type != Packet::Type::Hello
            && !IsKnownNeighbor(sender, receiver)))
    {
        ++metrics_.droppedMessages;
        return;
    }

    randomState_ ^= randomState_ << 13U;
    randomState_ ^= randomState_ >> 17U;
    randomState_ ^= randomState_ << 5U;
    const double randomValue = static_cast<double>(randomState_)
        / static_cast<double>(std::numeric_limits<std::uint32_t>::max());
    if (randomValue < settings_.packetLossProbability)
    {
        ++metrics_.droppedMessages;
        return;
    }

    packet.sender = sender;
    transmissions_.push_back({
        currentTick_ + std::max<std::uint64_t>(settings_.linkLatencyTicks, 1U),
        nextTransmissionOrder_++, sender, receiver, std::move(packet)});
}

void VirtualMeshNetwork::ProcessTransmission(Transmission transmission)
{
    if (!LinkAvailable(transmission.sender, transmission.receiver))
    {
        ++metrics_.droppedMessages;
        auto sourceRoutes = routes_.find(transmission.sender);
        if (sourceRoutes != routes_.end())
        {
            std::erase_if(sourceRoutes->second,
                [receiver = transmission.receiver](const auto& entry)
            {
                return entry.second.nextHop == receiver;
            });
        }

        Packet& droppedPacket = transmission.packet;
        if (droppedPacket.type == Packet::Type::Data)
        {
            const NodePair endpoints{
                droppedPacket.message.source, droppedPacket.message.destination};
            if (transmission.sender == endpoints.first)
            {
                droppedPacket.message.route.clear();
                pendingMessages_[endpoints].push_back(std::move(droppedPacket.message));
                if (!activeRequests_.contains(endpoints))
                    StartRouteDiscovery(endpoints.first, endpoints.second);
                return;
            }

            Packet routeError;
            routeError.type = Packet::Type::RouteError;
            routeError.requestOrigin = endpoints.first;
            routeError.requestDestination = endpoints.second;
            routeError.message = std::move(droppedPacket.message);
            routeError.remainingHops = settings_.maximumHops;
            const auto reverseRoute = routes_[transmission.sender].find(endpoints.first);
            if (reverseRoute != routes_[transmission.sender].end())
                Transmit(transmission.sender, reverseRoute->second.nextHop,
                    std::move(routeError));
            else
                ++metrics_.routeErrors;
        }
        return;
    }

    const NetworkNodeId receiver = transmission.receiver;
    Packet packet = std::move(transmission.packet);
    switch (packet.type)
    {
    case Packet::Type::Hello:
        neighbors_[receiver][transmission.sender] = {currentTick_};
        return;
    case Packet::Type::RouteRequest:
    {
        auto& seen = seenRequests_[receiver];
        if (seen.contains(packet.requestId) || packet.remainingHops == 0U)
            return;
        seen[packet.requestId] = currentTick_ + settings_.routeLifetimeTicks;
        routes_[receiver][packet.requestOrigin] = {
            transmission.sender, packet.hopCount + 1U,
            currentTick_ + settings_.routeLifetimeTicks};
        if (receiver == packet.requestDestination)
        {
            Packet reply;
            reply.type = Packet::Type::RouteReply;
            reply.requestId = packet.requestId;
            reply.requestOrigin = packet.requestOrigin;
            reply.requestDestination = packet.requestDestination;
            reply.remainingHops = settings_.maximumHops;
            reply.hopCount = 0U;
            ++metrics_.routeReplies;
            ForwardRouteReply(receiver, std::move(reply));
            return;
        }
        --packet.remainingHops;
        ++packet.hopCount;
        for (const auto& [neighbor, details] : neighbors_[receiver])
        {
            static_cast<void>(details);
            if (neighbor == transmission.sender)
                continue;
            Transmit(receiver, neighbor, packet);
        }
        return;
    }
    case Packet::Type::RouteReply:
        routes_[receiver][packet.requestDestination] = {
            transmission.sender, packet.hopCount + 1U,
            currentTick_ + settings_.routeLifetimeTicks};
        if (receiver == packet.requestOrigin)
        {
            const NodePair pair{packet.requestOrigin, packet.requestDestination};
            activeRequests_.erase(pair);
            activeRequestExpiryTicks_.erase(pair);
            auto pending = pendingMessages_.find(pair);
            if (pending != pendingMessages_.end())
            {
                for (VirtualMeshMessage& message : pending->second)
                {
                    Packet data;
                    data.type = Packet::Type::Data;
                    data.message = std::move(message);
                    data.remainingHops = settings_.maximumHops;
                    if (data.message.route.empty()
                        || data.message.route.back() != receiver)
                        data.message.route.push_back(receiver);
                    ForwardData(receiver, std::move(data));
                }
                pendingMessages_.erase(pending);
            }
            return;
        }
        if (packet.remainingHops == 0U)
        {
            ++metrics_.routeErrors;
            return;
        }
        --packet.remainingHops;
        ++packet.hopCount;
        ForwardRouteReply(receiver, std::move(packet));
        return;
    case Packet::Type::RouteError:
        InvalidateRoute(receiver, packet.requestDestination);
        if (receiver == packet.requestOrigin)
        {
            const NodePair pair{packet.requestOrigin, packet.requestDestination};
            packet.message.route.clear();
            pendingMessages_[pair].push_back(std::move(packet.message));
            if (!activeRequests_.contains(pair))
                StartRouteDiscovery(pair.first, pair.second);
        }
        else
        {
            const auto reverse = routes_[receiver].find(packet.requestOrigin);
            if (reverse == routes_[receiver].end())
            {
                ++metrics_.routeErrors;
                return;
            }
            Transmit(receiver, reverse->second.nextHop, std::move(packet));
        }
        return;
    case Packet::Type::Data:
        packet.message.route.push_back(receiver);
        if (receiver == packet.message.destination)
        {
            deliveredMessages_.push_back({std::move(packet.message), currentTick_});
            ++metrics_.deliveredMessages;
            return;
        }
        ForwardData(receiver, std::move(packet));
        return;
    }
}

void VirtualMeshNetwork::StartRouteDiscovery(
    NetworkNodeId source, NetworkNodeId destination)
{
    const NodePair pair{source, destination};
    const std::uint64_t requestId = nextRequestId_++;
    activeRequests_[pair] = requestId;
    activeRequestExpiryTicks_[pair] = currentTick_ + settings_.routeLifetimeTicks;
    Packet request;
    request.type = Packet::Type::RouteRequest;
    request.requestId = requestId;
    request.requestOrigin = source;
    request.requestDestination = destination;
    request.remainingHops = settings_.maximumHops;
    request.hopCount = 0U;
    ++metrics_.routeRequests;
    for (const auto& [neighbor, details] : neighbors_[source])
    {
        static_cast<void>(details);
        Transmit(source, neighbor, request);
    }
}

void VirtualMeshNetwork::ForwardData(NetworkNodeId node, Packet packet)
{
    if (packet.remainingHops == 0U)
    {
        ++metrics_.routeErrors;
        return;
    }
    const auto routes = routes_.find(node);
    if (routes == routes_.end())
    {
        ++metrics_.routeErrors;
        return;
    }
    const auto route = routes->second.find(packet.message.destination);
    if (route == routes->second.end() || route->second.expiresAtTick <= currentTick_
        || !IsKnownNeighbor(node, route->second.nextHop))
    {
        InvalidateRoute(node, packet.message.destination);
        const NodePair pair{node, packet.message.destination};
        pendingMessages_[pair].push_back(std::move(packet.message));
        ++metrics_.routeErrors;
        if (!activeRequests_.contains(pair))
            StartRouteDiscovery(pair.first, pair.second);
        return;
    }
    if (node != packet.message.source)
        ++metrics_.forwardedMessages;
    --packet.remainingHops;
    ++packet.hopCount;
    Transmit(node, route->second.nextHop, std::move(packet));
}

void VirtualMeshNetwork::ForwardRouteReply(NetworkNodeId node, Packet packet)
{
    if (packet.remainingHops == 0U)
    {
        ++metrics_.routeErrors;
        return;
    }
    const auto route = routes_[node].find(packet.requestOrigin);
    if (route == routes_[node].end() || route->second.expiresAtTick <= currentTick_)
    {
        ++metrics_.routeErrors;
        return;
    }
    --packet.remainingHops;
    ++packet.hopCount;
    Transmit(node, route->second.nextHop, std::move(packet));
}

void VirtualMeshNetwork::InvalidateRoute(
    NetworkNodeId node, NetworkNodeId destination)
{
    const auto routes = routes_.find(node);
    if (routes == routes_.end())
        return;
    routes->second.erase(destination);
}

void VirtualMeshNetwork::ExpireOldState()
{
    for (auto& [node, routes] : routes_)
    {
        static_cast<void>(node);
        std::erase_if(routes, [this](const auto& entry)
        {
            return entry.second.expiresAtTick <= currentTick_;
        });
    }
    for (auto& [node, neighbors] : neighbors_)
    {
        static_cast<void>(node);
        std::erase_if(neighbors, [this](const auto& entry)
        {
            return currentTick_ - entry.second.lastHeardTick
                > settings_.neighborTimeoutTicks;
        });
    }
    for (auto& [node, requests] : seenRequests_)
    {
        static_cast<void>(node);
        std::erase_if(requests, [this](const auto& entry)
        {
            return entry.second <= currentTick_;
        });
    }
    std::vector<NodePair> expiredDiscoveries;
    for (const auto& [pair, expiresAtTick] : activeRequestExpiryTicks_)
    {
        if (expiresAtTick <= currentTick_)
            expiredDiscoveries.push_back(pair);
    }
    for (const NodePair& pair : expiredDiscoveries)
    {
        activeRequestExpiryTicks_.erase(pair);
        activeRequests_.erase(pair);
        if (pendingMessages_.contains(pair))
            StartRouteDiscovery(pair.first, pair.second);
    }
}

bool VirtualMeshNetwork::IsKnownNeighbor(
    NetworkNodeId node, NetworkNodeId neighbor) const noexcept
{
    const auto neighbors = neighbors_.find(node);
    if (neighbors == neighbors_.end())
        return false;
    const auto entry = neighbors->second.find(neighbor);
    return entry != neighbors->second.end()
        && currentTick_ - entry->second.lastHeardTick <= settings_.neighborTimeoutTicks;
}
}
