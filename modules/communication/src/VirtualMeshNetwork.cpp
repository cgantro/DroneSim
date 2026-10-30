#include "communication/VirtualMeshNetwork.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <glm/geometric.hpp>

namespace dronesim::communication
{
namespace
{
constexpr double geometryTolerance = 1.0e-9;
constexpr std::size_t maximumRecordedEvents = 64U;

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

VirtualMeshNetwork::VirtualMeshNetwork(
    DroneSocketTransport& transport, VirtualMeshSettings settings)
    : transport_(transport), settings_(settings)
{
    if (!std::isfinite(settings_.maximumLinkRangeMeters)
        || settings_.maximumLinkRangeMeters <= 0.0)
        throw std::invalid_argument("가상 통신망 설정값을 확인해 주세요.");
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

    // 노드 집합이 바뀌면 전송 계층이 해당 드론의 소켓도 함께 맞춘다.
    std::vector<NetworkNodeId> nodeIds;
    nodeIds.reserve(updated.size());
    for (const auto& [id, node] : updated)
    {
        static_cast<void>(node);
        nodeIds.push_back(id);
    }
    transport_.UpdateNodes(nodeIds);

    nodes_ = std::move(updated);
    obstacles_.assign(obstacles.begin(), obstacles.end());
    std::erase_if(radioDisabledNodes_, [this](NetworkNodeId id)
    {
        return !nodes_.contains(id);
    });
    std::erase_if(pendingMessages_, [this](const auto& entry)
    {
        return !nodes_.contains(entry.second.sender)
            || !nodes_.contains(entry.second.receiver);
    });
    std::erase_if(receivedMessages_, [this](const ReceivedMessage& entry)
    {
        return !nodes_.contains(entry.sender) || !nodes_.contains(entry.receiver);
    });
}

void VirtualMeshNetwork::SetNodeRadioEnabled(NetworkNodeId node, bool enabled)
{
    if (!nodes_.contains(node) || IsNodeRadioEnabled(node) == enabled)
        return;
    if (enabled)
        radioDisabledNodes_.erase(node);
    else
        radioDisabledNodes_.insert(node);
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
    // UDP로 보내기 전에 가상 환경의 거리·가림·라디오 조건을 먼저 확인한다.
    const auto sourceNode = nodes_.find(source);
    const auto destinationNode = nodes_.find(destination);
    if (source == destination || kind.empty()
        || sourceNode == nodes_.end() || destinationNode == nodes_.end()
        || !LinkAvailable(source, destination))
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, source, destination,
            "직접 연결이 없어 메시지를 보내지 못했습니다.");
        return std::nullopt;
    }

    VirtualMeshMessage message{
        .id = nextMessageId_++,
        .source = source,
        .destination = destination,
        .sequence = sequence,
        .kind = std::move(kind),
        .payload = std::move(payload)};
    const std::uint64_t id = message.id;
    const std::uint64_t maximumTick = std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t dueTick = currentTick_ == maximumTick
        ? maximumTick : currentTick_ + 1U;
    if (!transport_.Send({message.id, source, destination, sequence,
            message.kind, message.payload}))
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, source, destination,
            "UDP 메시지가 데이터그램 크기 제한을 넘었습니다.");
        return std::nullopt;
    }
    pendingMessages_.emplace(id, PendingMessage{dueTick, source, destination});
    return id;
}

void VirtualMeshNetwork::Advance(std::uint64_t tick)
{
    if (tick < currentTick_)
        throw std::invalid_argument("가상 통신망 시간은 뒤로 갈 수 없습니다.");
    currentTick_ = tick;
    deliveredMessages_.clear();
    ReceiveMessages();

    std::vector<ReceivedMessage> waiting;
    for (ReceivedMessage& received : receivedMessages_)
    {
        if (received.dueTick > tick)
        {
            waiting.push_back(std::move(received));
            continue;
        }

        if (!LinkAvailable(received.sender, received.receiver))
        {
            ++metrics_.droppedMessages;
            RecordEvent(MeshEventKind::PacketDropped, received.sender, received.receiver,
                "UDP 전송 중 직접 연결이 끊겼습니다.");
            continue;
        }
        RecordEvent(MeshEventKind::MessageDelivered,
            received.message.source, received.message.destination,
            received.message.kind + " 메시지가 도착했습니다.");
        deliveredMessages_.push_back({std::move(received.message), currentTick_});
        ++metrics_.deliveredMessages;
    }
    receivedMessages_ = std::move(waiting);

    std::erase_if(pendingMessages_, [this](const auto& entry)
    {
        const PendingMessage& pending = entry.second;
        if (currentTick_ <= pending.dueTick
            || currentTick_ - pending.dueTick <= 4U)
            return false;
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, pending.sender, pending.receiver,
            "UDP 데이터그램을 제한 시간 안에 받지 못했습니다.");
        return true;
    });
}

void VirtualMeshNetwork::ReceiveMessages()
{
    // 수신한 봉투가 실제 송신 노드와 대기 중인 메시지 기록에 맞는지 확인한다.
    SocketReceiveBatch batch = transport_.ReceiveAll();
    for (std::size_t index = 0; index < batch.droppedDatagrams; ++index)
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, 0U, 0U,
            "형식이 맞지 않는 UDP 메시지를 버렸습니다.");
    }

    for (SocketPacket& packet : batch.packets)
    {
        const auto pending = pendingMessages_.find(packet.id);
        const NetworkNodeId senderId = packet.source;
        const NetworkNodeId receiverId = packet.destination;
        const auto sender = nodes_.find(senderId);
        const auto receiver = nodes_.find(receiverId);
        if (pending == pendingMessages_.end() || sender == nodes_.end()
            || receiver == nodes_.end()
            || pending->second.sender != senderId
            || pending->second.receiver != receiverId)
        {
            ++metrics_.droppedMessages;
            RecordEvent(MeshEventKind::PacketDropped, senderId, receiverId,
                "등록된 드론 사이에서 보낸 메시지가 아닙니다.");
            if (pending != pendingMessages_.end())
                pendingMessages_.erase(pending);
            continue;
        }

        VirtualMeshMessage message{
            .id = packet.id,
            .source = senderId,
            .destination = receiverId,
            .sequence = packet.sequence,
            .kind = std::move(packet.kind),
            .payload = std::move(packet.payload)};

        receivedMessages_.push_back({pending->second.dueTick,
            senderId, receiverId, std::move(message)});
        pendingMessages_.erase(pending);
    }
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

bool VirtualMeshNetwork::HasDirectLink(
    NetworkNodeId node, NetworkNodeId neighbor) const noexcept
{
    return LinkAvailable(node, neighbor);
}

bool VirtualMeshNetwork::CanExchangeAtPositions(
    NetworkNodeId first, NetworkNodeId second,
    const glm::dvec3& firstPositionMeters,
    const glm::dvec3& secondPositionMeters) const
{
    if (!nodes_.contains(first) || !nodes_.contains(second))
        return false;
    return LinkAvailableAt(first, second, firstPositionMeters, secondPositionMeters);
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

bool VirtualMeshNetwork::LinkAvailableAt(NetworkNodeId sender,
    NetworkNodeId receiver, const glm::dvec3& senderPositionMeters,
    const glm::dvec3& receiverPositionMeters) const
{
    if (radioDisabledNodes_.contains(sender) || radioDisabledNodes_.contains(receiver))
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

void VirtualMeshNetwork::RecordEvent(MeshEventKind kind,
    NetworkNodeId source, NetworkNodeId destination, std::string detail)
{
    if (events_.size() == maximumRecordedEvents)
        events_.erase(events_.begin());
    events_.push_back({currentTick_, kind, source, destination, std::move(detail)});
}
}
