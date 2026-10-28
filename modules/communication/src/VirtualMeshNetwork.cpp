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
constexpr std::size_t maximumRecordedEvents = 64U;

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
        || settings_.reliableRetryIntervalTicks == 0U
        || settings_.maximumReliableAttempts == 0U
        || settings_.maximumQueuedMessages == 0U
        || settings_.maximumScheduledTransmissions == 0U
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
    if (settings_.mode == VirtualMeshMode::LeaderCentric
        && !updated.contains(settings_.leaderNodeId))
        throw std::invalid_argument("리더 중심 통신 모드에 지정한 리더가 없습니다.");

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

void VirtualMeshNetwork::SetLeaderNodeId(NetworkNodeId leaderNodeId)
{
    if (!nodes_.contains(leaderNodeId))
        throw std::invalid_argument("새 리더가 통신망에 없습니다.");
    if (settings_.leaderNodeId == leaderNodeId)
        return;

    settings_.leaderNodeId = leaderNodeId;
    routes_.clear();
    activeRequests_.clear();
    activeRequestExpiryTicks_.clear();
    seenRequests_.clear();
    unavailableRoutes_.clear();

    std::vector<NodePair> pendingRoutes;
    pendingRoutes.reserve(pendingMessages_.size());
    for (const auto& [endpoints, messages] : pendingMessages_)
    {
        static_cast<void>(messages);
        pendingRoutes.push_back(endpoints);
    }
    for (const NodePair& endpoints : pendingRoutes)
        StartRouteDiscovery(endpoints.first, endpoints.second);
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
        RecordEvent(MeshEventKind::LinkDisabled, first, second, "링크를 끊었습니다.");
        for (auto& [node, table] : routes_)
        {
            for (auto route = table.begin(); route != table.end();)
            {
                if (MakePair(node, route->second.nextHop) != pair)
                {
                    ++route;
                    continue;
                }
                if (settings_.mode == VirtualMeshMode::MultiHop)
                    unavailableRoutes_.insert({node, route->first});
                route = table.erase(route);
            }
        }
        for (auto& [node, knownNeighbors] : neighbors_)
        {
            if (node == first)
                static_cast<void>(knownNeighbors.erase(second));
            else if (node == second)
                static_cast<void>(knownNeighbors.erase(first));
        }
    }
    else
    {
        if (disabledLinks_.empty() && disabledDirectedLinks_.empty()
            && radioDisabledNodes_.empty())
            unavailableRoutes_.clear();
        RecordEvent(MeshEventKind::LinkRestored, first, second, "링크를 복구했습니다.");
    }
}

void VirtualMeshNetwork::SetDirectedLinkDisabled(
    NetworkNodeId sender, NetworkNodeId receiver, bool disabled)
{
    if (sender == receiver)
        throw std::invalid_argument("한 노드와 자기 자신 사이에는 방향 장애를 설정할 수 없습니다.");

    const NodePair direction{sender, receiver};
    if (disabled)
    {
        if (!disabledDirectedLinks_.insert(direction).second)
            return;

        const auto routes = routes_.find(sender);
        if (routes != routes_.end())
        {
            for (auto route = routes->second.begin(); route != routes->second.end();)
            {
                if (route->second.nextHop != receiver)
                {
                    ++route;
                    continue;
                }
                if (settings_.mode == VirtualMeshMode::MultiHop)
                    unavailableRoutes_.insert({sender, route->first});
                route = routes->second.erase(route);
            }
        }
        const auto neighbors = neighbors_.find(sender);
        if (neighbors != neighbors_.end())
            neighbors->second.erase(receiver);
        RecordEvent(MeshEventKind::DirectedLinkDisabled, sender, receiver,
            "선택한 방향의 통신을 끊었습니다.");
        return;
    }

    if (disabledDirectedLinks_.erase(direction) == 0U)
        return;
    if (disabledLinks_.empty() && disabledDirectedLinks_.empty()
        && radioDisabledNodes_.empty())
        unavailableRoutes_.clear();
    RecordEvent(MeshEventKind::DirectedLinkRestored, sender, receiver,
        "선택한 방향의 통신을 복구했습니다.");
}

void VirtualMeshNetwork::SetPacketLossProbability(double probability)
{
    if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0)
        throw std::invalid_argument("패킷 손실 확률은 0에서 1 사이여야 합니다.");
    settings_.packetLossProbability = probability;
}

void VirtualMeshNetwork::SetLinkLatencyJitterTicks(
    std::uint32_t maximumExtraTicks) noexcept
{
    settings_.linkLatencyJitterTicks = maximumExtraTicks;
}

bool VirtualMeshNetwork::IsLinkDisabled(
    NetworkNodeId first, NetworkNodeId second) const noexcept
{
    return first != second && disabledLinks_.contains(MakePair(first, second));
}

bool VirtualMeshNetwork::IsDirectedLinkDisabled(
    NetworkNodeId sender, NetworkNodeId receiver) const noexcept
{
    return sender != receiver && disabledDirectedLinks_.contains({sender, receiver});
}

void VirtualMeshNetwork::SetNodeRadioEnabled(NetworkNodeId node, bool enabled)
{
    if (!nodes_.contains(node) || IsNodeRadioEnabled(node) == enabled)
        return;

    if (enabled)
        radioDisabledNodes_.erase(node);
    else
        radioDisabledNodes_.insert(node);
    for (auto& [source, table] : routes_)
    {
        for (auto route = table.begin(); route != table.end();)
        {
            if (route->first != node && route->second.nextHop != node)
            {
                ++route;
                continue;
            }
            if (!enabled && settings_.mode == VirtualMeshMode::MultiHop)
                unavailableRoutes_.insert({source, route->first});
            route = table.erase(route);
        }
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
    else if (disabledLinks_.empty() && disabledDirectedLinks_.empty()
        && radioDisabledNodes_.empty())
    {
        unavailableRoutes_.clear();
    }
    RecordEvent(enabled ? MeshEventKind::NodeRadioRestored
                        : MeshEventKind::NodeRadioDisabled,
        node, node, enabled ? "드론 라디오를 복구했습니다." : "드론 송수신을 차단했습니다.");
}

bool VirtualMeshNetwork::IsNodeRadioEnabled(NetworkNodeId node) const noexcept
{
    return nodes_.contains(node) && !radioDisabledNodes_.contains(node);
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
    const std::uint64_t messageId = message.id;
    return SendMessage(message) ? std::optional{messageId} : std::nullopt;
}

std::optional<std::uint64_t> VirtualMeshNetwork::SendReliable(
    NetworkNodeId source, NetworkNodeId destination,
    std::uint32_t sequence, std::string kind, std::string payload)
{
    if (!nodes_.contains(source) || !nodes_.contains(destination)
        || source == destination || kind.empty())
        return std::nullopt;
    if (pendingReliableMessages_.size() >= settings_.maximumQueuedMessages)
    {
        ++metrics_.queueFullDrops;
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::QueueFull, source, destination,
            "중요 메시지 대기 한도에 도달했습니다.");
        return std::nullopt;
    }

    VirtualMeshMessage message{
        .id = nextMessageId_++,
        .source = source,
        .destination = destination,
        .sequence = sequence,
        .kind = std::move(kind),
        .payload = std::move(payload),
        .reliable = true};
    if (!SendMessage(message))
        return std::nullopt;

    const std::uint64_t messageId = message.id;
    pendingReliableMessages_.emplace(messageId, PendingReliableMessage{
        .message = std::move(message),
        .lastAttemptTick = currentTick_,
        .attempts = 1U});
    return messageId;
}

bool VirtualMeshNetwork::SendMessage(const VirtualMeshMessage& message)
{
    const NetworkNodeId source = message.source;
    const NetworkNodeId destination = message.destination;
    const NodePair pair{source, destination};
    if (settings_.mode == VirtualMeshMode::MultiHop
        && unavailableRoutes_.contains(pair))
    {
        ++metrics_.routeErrors;
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::RouteError, source, destination,
            "고정 다중 홉 경로가 끊겨 재탐색하지 않았습니다.");
        return false;
    }
    const auto route = routes_[source].find(destination);
    if (route != routes_[source].end() && route->second.expiresAtTick > currentTick_
        && IsKnownNeighbor(source, route->second.nextHop)
        && IsAllowedNextHop(source, destination, route->second.nextHop))
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
        if (!QueuePendingMessage(message))
            return false;
        if (!activeRequests_.contains(pair))
            StartRouteDiscovery(source, destination);
    }
    return true;
}

bool VirtualMeshNetwork::QueuePendingMessage(VirtualMeshMessage message)
{
    for (const auto& [endpoints, messages] : pendingMessages_)
    {
        static_cast<void>(endpoints);
        if (std::any_of(messages.begin(), messages.end(),
            [&message](const VirtualMeshMessage& queued)
            {
                return queued.id == message.id;
            }))
            return true;
    }
    if (metrics_.queuedMessages >= settings_.maximumQueuedMessages)
    {
        ++metrics_.queueFullDrops;
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::QueueFull, message.source, message.destination,
            "경로를 기다리는 메시지 대기 한도에 도달했습니다.");
        return false;
    }
    pendingMessages_[{message.source, message.destination}]
        .push_back(std::move(message));
    ++metrics_.queuedMessages;
    return true;
}

void VirtualMeshNetwork::RemovePendingMessage(std::uint64_t messageId)
{
    for (auto iterator = pendingMessages_.begin(); iterator != pendingMessages_.end();)
    {
        const std::size_t before = iterator->second.size();
        std::erase_if(iterator->second, [messageId](const VirtualMeshMessage& message)
        {
            return message.id == messageId;
        });
        metrics_.queuedMessages -= before - iterator->second.size();
        if (iterator->second.empty())
            iterator = pendingMessages_.erase(iterator);
        else
            ++iterator;
    }
}

std::optional<std::uint64_t> VirtualMeshNetwork::SendLatest(
    NetworkNodeId source, NetworkNodeId destination,
    std::uint32_t sequence, std::string kind, std::string payload)
{
    if (!nodes_.contains(source) || !nodes_.contains(destination)
        || source == destination || kind.empty())
        return std::nullopt;

    const NodePair pair{source, destination};
    const auto pending = pendingMessages_.find(pair);
    if (pending != pendingMessages_.end())
    {
        const std::size_t before = pending->second.size();
        std::erase_if(pending->second, [&kind](const VirtualMeshMessage& message)
        {
            return message.kind == kind;
        });
        metrics_.queuedMessages -= before - pending->second.size();
        if (pending->second.empty())
            pendingMessages_.erase(pending);
    }

    return Send(source, destination, sequence, std::move(kind), std::move(payload));
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
    AdvanceReliableMessages();
}

std::span<const DeliveredMeshMessage>
VirtualMeshNetwork::DeliveredMessages() const noexcept
{
    return deliveredMessages_;
}

std::span<const MeshEvent> VirtualMeshNetwork::Events() const noexcept
{
    return events_;
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

bool VirtualMeshNetwork::HasRecentNeighbor(
    NetworkNodeId node, NetworkNodeId neighbor) const noexcept
{
    return IsKnownNeighbor(node, neighbor);
}

bool VirtualMeshNetwork::CanExchangeAtPositions(
    NetworkNodeId first, NetworkNodeId second,
    const glm::dvec3& firstPositionMeters,
    const glm::dvec3& secondPositionMeters) const
{
    return LinkAvailableAt(first, second, firstPositionMeters, secondPositionMeters)
        && LinkAvailableAt(second, first, secondPositionMeters, firstPositionMeters);
}

bool VirtualMeshNetwork::LinkAvailable(
    NetworkNodeId sender, NetworkNodeId receiver) const
{
    const auto senderNode = nodes_.find(sender);
    const auto receiverNode = nodes_.find(receiver);
    if (senderNode == nodes_.end() || receiverNode == nodes_.end())
        return false;
    return LinkAvailableAt(sender, receiver,
        senderNode->second.positionMeters, receiverNode->second.positionMeters);
}

bool VirtualMeshNetwork::LinkAvailableAt(
    NetworkNodeId sender, NetworkNodeId receiver,
    const glm::dvec3& senderPositionMeters,
    const glm::dvec3& receiverPositionMeters) const
{
    const auto senderNode = nodes_.find(sender);
    const auto receiverNode = nodes_.find(receiver);
    if (senderNode == nodes_.end() || receiverNode == nodes_.end()
        || !senderNode->second.powered || !receiverNode->second.powered
        || !senderNode->second.canTransmit || !receiverNode->second.canReceive
        || radioDisabledNodes_.contains(sender) || radioDisabledNodes_.contains(receiver)
        || disabledLinks_.contains(MakePair(sender, receiver))
        || disabledDirectedLinks_.contains({sender, receiver}))
        return false;
    if (!std::isfinite(senderPositionMeters.x) || !std::isfinite(senderPositionMeters.y)
        || !std::isfinite(senderPositionMeters.z)
        || !std::isfinite(receiverPositionMeters.x)
        || !std::isfinite(receiverPositionMeters.y)
        || !std::isfinite(receiverPositionMeters.z)
        || glm::distance(senderPositionMeters, receiverPositionMeters)
            > settings_.maximumLinkRangeMeters)
        return false;
    return HasLineOfSight(senderPositionMeters, receiverPositionMeters);
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

void VirtualMeshNetwork::AdvanceReliableMessages()
{
    const std::uint64_t retryInterval = settings_.reliableRetryIntervalTicks;
    for (auto iterator = pendingReliableMessages_.begin();
         iterator != pendingReliableMessages_.end();)
    {
        PendingReliableMessage& pending = iterator->second;
        const std::uint64_t retryAt = pending.lastAttemptTick
                > std::numeric_limits<std::uint64_t>::max() - retryInterval
            ? std::numeric_limits<std::uint64_t>::max()
            : pending.lastAttemptTick + retryInterval;
        if (currentTick_ < retryAt)
        {
            ++iterator;
            continue;
        }

        if (pending.attempts >= settings_.maximumReliableAttempts)
        {
            ++metrics_.reliableFailures;
            ++metrics_.droppedMessages;
            RecordEvent(MeshEventKind::ReliableFailed, pending.message.source,
                pending.message.destination,
                pending.message.kind + " 메시지 확인 응답을 받지 못했습니다.");
            RemovePendingMessage(pending.message.id);
            iterator = pendingReliableMessages_.erase(iterator);
            continue;
        }

        if (SendMessage(pending.message))
        {
            pending.lastAttemptTick = currentTick_;
            ++pending.attempts;
            ++metrics_.reliableRetries;
            RecordEvent(MeshEventKind::ReliableRetry, pending.message.source,
                pending.message.destination,
                pending.message.kind + " 메시지를 다시 보냈습니다.");
        }
        ++iterator;
    }
}

void VirtualMeshNetwork::AcknowledgeReliableMessage(
    NetworkNodeId destination, const VirtualMeshMessage& message)
{
    Packet acknowledgement;
    acknowledgement.type = Packet::Type::DataAcknowledgement;
    acknowledgement.message.id = message.id;
    acknowledgement.message.source = destination;
    acknowledgement.message.destination = message.source;
    acknowledgement.message.sequence = message.sequence;
    acknowledgement.message.kind = message.kind;
    acknowledgement.remainingHops = settings_.maximumHops;
    acknowledgement.message.route.push_back(destination);
    ForwardData(destination, std::move(acknowledgement));
}

void VirtualMeshNetwork::Transmit(
    NetworkNodeId sender, NetworkNodeId receiver, Packet packet)
{
    if (!LinkAvailable(sender, receiver)
        || (packet.type != Packet::Type::Hello
            && !IsKnownNeighbor(sender, receiver)))
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, sender, receiver,
            "메시지를 보낼 이웃 링크가 없습니다.");
        return;
    }
    if (transmissions_.size() >= settings_.maximumScheduledTransmissions)
    {
        ++metrics_.droppedMessages;
        ++metrics_.queueFullDrops;
        RecordEvent(MeshEventKind::QueueFull, sender, receiver,
            "예약된 전송 대기 한도에 도달했습니다.");
        return;
    }

    const auto drawRandomValue = [this]
    {
        randomState_ ^= randomState_ << 13U;
        randomState_ ^= randomState_ >> 17U;
        randomState_ ^= randomState_ << 5U;
        return randomState_;
    };
    const double randomValue = static_cast<double>(drawRandomValue())
        / static_cast<double>(std::numeric_limits<std::uint32_t>::max());
    if (randomValue < settings_.packetLossProbability)
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, sender, receiver,
            "설정된 패킷 손실로 버렸습니다.");
        return;
    }

    std::uint64_t latencyTicks = std::max<std::uint64_t>(
        settings_.linkLatencyTicks, 1U);
    const std::uint64_t maximumTick = std::numeric_limits<std::uint64_t>::max();
    if (settings_.linkLatencyJitterTicks > 0U)
    {
        const std::uint64_t jitterRange =
            static_cast<std::uint64_t>(settings_.linkLatencyJitterTicks) + 1U;
        const std::uint64_t jitterTicks = drawRandomValue() % jitterRange;
        latencyTicks = latencyTicks > maximumTick - jitterTicks
            ? maximumTick : latencyTicks + jitterTicks;
    }
    const std::uint64_t dueTick = currentTick_ > maximumTick - latencyTicks
        ? maximumTick : currentTick_ + latencyTicks;

    packet.sender = sender;
    transmissions_.push_back({
        dueTick,
        nextTransmissionOrder_++, sender, receiver, std::move(packet)});
}

void VirtualMeshNetwork::ProcessTransmission(Transmission transmission)
{
    if (!LinkAvailable(transmission.sender, transmission.receiver))
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped,
            transmission.sender, transmission.receiver, "전송 중 링크가 끊겼습니다.");
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
                const bool queued = QueuePendingMessage(
                    std::move(droppedPacket.message));
                if (queued && !activeRequests_.contains(endpoints))
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
        if (!neighbors_[receiver].contains(transmission.sender))
            RecordEvent(MeshEventKind::NeighborDiscovered,
                receiver, transmission.sender, "새 이웃을 찾았습니다.");
        neighbors_[receiver][transmission.sender] = {currentTick_};
        {
            std::vector<NetworkNodeId> pendingDestinations;
            for (const auto& [endpoints, messages] : pendingMessages_)
            {
                static_cast<void>(messages);
                if (endpoints.first == receiver
                    && !activeRequests_.contains(endpoints))
                    pendingDestinations.push_back(endpoints.second);
            }
            for (const NetworkNodeId destination : pendingDestinations)
                StartRouteDiscovery(receiver, destination);
        }
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
        if (settings_.mode == VirtualMeshMode::Direct)
            return;
        if (settings_.mode == VirtualMeshMode::LeaderCentric
            && receiver != settings_.leaderNodeId)
            return;
        --packet.remainingHops;
        ++packet.hopCount;
        if (settings_.mode == VirtualMeshMode::LeaderCentric)
        {
            if (IsKnownNeighbor(receiver, packet.requestDestination))
                Transmit(receiver, packet.requestDestination, packet);
            return;
        }
        for (const auto& [neighbor, details] : neighbors_[receiver])
        {
            static_cast<void>(details);
            if (neighbor != transmission.sender)
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
            RecordEvent(MeshEventKind::RouteFound, packet.requestOrigin,
                packet.requestDestination,
                "경로를 찾았습니다. 홉 수 " + std::to_string(packet.hopCount) + ".");
            auto pending = pendingMessages_.find(pair);
            if (pending != pendingMessages_.end())
            {
                metrics_.queuedMessages -= pending->second.size();
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
        ForwardRouteReply(receiver, std::move(packet));
        return;
    case Packet::Type::RouteError:
        InvalidateRoute(receiver, packet.requestDestination);
        if (receiver == packet.requestOrigin)
        {
            const NodePair pair{packet.requestOrigin, packet.requestDestination};
            packet.message.route.clear();
            const bool queued = QueuePendingMessage(std::move(packet.message));
            if (queued && !activeRequests_.contains(pair))
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
    case Packet::Type::DataAcknowledgement:
        packet.message.route.push_back(receiver);
        if (receiver == packet.message.destination)
        {
            if (packet.type == Packet::Type::DataAcknowledgement)
            {
                if (pendingReliableMessages_.erase(packet.message.id) > 0U)
                {
                    ++metrics_.reliableAcknowledgements;
                    RecordEvent(MeshEventKind::ReliableAcknowledged,
                        packet.message.source, packet.message.destination,
                        packet.message.kind + " 메시지 확인 응답을 받았습니다.");
                }
                return;
            }

            if (packet.message.reliable)
            {
                auto& received = receivedReliableMessages_[receiver];
                if (received.contains(packet.message.id))
                {
                    ++metrics_.duplicateReliableMessages;
                    AcknowledgeReliableMessage(receiver, packet.message);
                    return;
                }
                const std::uint64_t retryWindow = settings_.reliableRetryIntervalTicks
                    > std::numeric_limits<std::uint64_t>::max()
                        / settings_.maximumReliableAttempts
                    ? std::numeric_limits<std::uint64_t>::max()
                    : settings_.reliableRetryIntervalTicks
                        * settings_.maximumReliableAttempts;
                const std::uint64_t retention = std::max(
                    settings_.routeLifetimeTicks, retryWindow);
                const std::uint64_t maximumTick =
                    std::numeric_limits<std::uint64_t>::max();
                received[packet.message.id] = currentTick_ > maximumTick - retention
                    ? maximumTick : currentTick_ + retention;
            }

            RecordEvent(MeshEventKind::MessageDelivered,
                packet.message.source, packet.message.destination,
                packet.message.kind + " 메시지가 도착했습니다.");
            const bool reliable = packet.message.reliable;
            const VirtualMeshMessage deliveredMessage = packet.message;
            deliveredMessages_.push_back({std::move(packet.message), currentTick_});
            ++metrics_.deliveredMessages;
            if (reliable)
                AcknowledgeReliableMessage(receiver, deliveredMessage);
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
    if (settings_.mode == VirtualMeshMode::MultiHop
        && unavailableRoutes_.contains(pair))
        return;
    const auto knownNeighbors = neighbors_.find(source);
    if (knownNeighbors == neighbors_.end() || knownNeighbors->second.empty())
        return;

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
    RecordEvent(MeshEventKind::RouteSearchStarted,
        source, destination, "새 경로를 찾기 시작했습니다.");
    const auto transmitTo = [this, source, &request](NetworkNodeId neighbor)
    {
        Transmit(source, neighbor, request);
    };
    if (settings_.mode == VirtualMeshMode::Direct)
    {
        if (knownNeighbors->second.contains(destination))
            transmitTo(destination);
        return;
    }
    if (settings_.mode == VirtualMeshMode::LeaderCentric)
    {
        const NetworkNodeId nextNode = source == settings_.leaderNodeId
            || destination == settings_.leaderNodeId
            ? destination : settings_.leaderNodeId;
        if (knownNeighbors->second.contains(nextNode))
            transmitTo(nextNode);
        return;
    }
    for (const auto& [neighbor, details] : knownNeighbors->second)
    {
        static_cast<void>(details);
        transmitTo(neighbor);
    }
}

void VirtualMeshNetwork::ForwardData(NetworkNodeId node, Packet packet)
{
    if (packet.remainingHops == 0U)
    {
        ++metrics_.routeErrors;
        RecordEvent(MeshEventKind::RouteError, node, packet.message.destination,
            "최대 홉 수를 넘었습니다.");
        return;
    }
    const auto routes = routes_.find(node);
    if (routes == routes_.end())
    {
        ++metrics_.routeErrors;
        if (packet.type == Packet::Type::DataAcknowledgement)
        {
            ++metrics_.droppedMessages;
            RecordEvent(MeshEventKind::RouteError, node, packet.message.destination,
                "확인 응답을 보낼 경로가 없어 원본 재전송을 기다립니다.");
        }
        return;
    }
    const auto route = routes->second.find(packet.message.destination);
    if (route == routes->second.end() || route->second.expiresAtTick <= currentTick_
        || !IsKnownNeighbor(node, route->second.nextHop)
        || !IsAllowedNextHop(node, packet.message.destination,
            route->second.nextHop))
    {
        InvalidateRoute(node, packet.message.destination);
        const NodePair pair{node, packet.message.destination};
        if (settings_.mode == VirtualMeshMode::MultiHop)
        {
            unavailableRoutes_.insert(pair);
            ++metrics_.routeErrors;
            ++metrics_.droppedMessages;
            RecordEvent(MeshEventKind::RouteError, node, packet.message.destination,
                "고정 다중 홉 경로가 끊겨 재탐색하지 않았습니다.");
            return;
        }
        if (packet.type == Packet::Type::DataAcknowledgement)
        {
            // 확인 응답은 원본이 다시 도착하면 재발급하므로 업무 메시지 큐에 넣지 않는다.
            ++metrics_.routeErrors;
            ++metrics_.droppedMessages;
            RecordEvent(MeshEventKind::RouteError, node, packet.message.destination,
                "확인 응답을 보낼 경로가 없어 원본 재전송을 기다립니다.");
            return;
        }
        const NetworkNodeId destination = packet.message.destination;
        const bool queued = QueuePendingMessage(std::move(packet.message));
        ++metrics_.routeErrors;
        if (queued)
            RecordEvent(MeshEventKind::RouteError, node, destination,
                "사용할 경로가 없어 다시 찾습니다.");
        if (queued && !activeRequests_.contains(pair))
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
        RecordEvent(MeshEventKind::RouteError, node, packet.requestOrigin,
            "경로 응답이 최대 홉 수를 넘었습니다.");
        return;
    }
    const auto route = routes_[node].find(packet.requestOrigin);
    if (route == routes_[node].end() || route->second.expiresAtTick <= currentTick_)
    {
        ++metrics_.routeErrors;
        RecordEvent(MeshEventKind::RouteError, node, packet.requestOrigin,
            "경로 응답을 보낼 역방향 경로가 없습니다.");
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
    for (auto& [node, receivedMessages] : receivedReliableMessages_)
    {
        static_cast<void>(node);
        std::erase_if(receivedMessages, [this](const auto& entry)
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

void VirtualMeshNetwork::RecordEvent(MeshEventKind kind,
    NetworkNodeId source, NetworkNodeId destination, std::string detail)
{
    if (events_.size() == maximumRecordedEvents)
        events_.erase(events_.begin());
    events_.push_back({currentTick_, kind, source, destination, std::move(detail)});
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

bool VirtualMeshNetwork::IsAllowedNextHop(NetworkNodeId node,
    NetworkNodeId destination, NetworkNodeId nextHop) const noexcept
{
    switch (settings_.mode)
    {
    case VirtualMeshMode::Direct:
        return nextHop == destination;
    case VirtualMeshMode::LeaderCentric:
        if (node == settings_.leaderNodeId || destination == settings_.leaderNodeId)
            return nextHop == destination;
        return nextHop == settings_.leaderNodeId;
    case VirtualMeshMode::MultiHop:
    case VirtualMeshMode::AdaptiveMesh:
        return true;
    }
    return false;
}
}
