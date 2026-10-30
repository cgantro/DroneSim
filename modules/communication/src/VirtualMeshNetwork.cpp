#include "communication/VirtualMeshNetwork.h"

#include <nlohmann/json.hpp>

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
using Json = nlohmann::json;
constexpr double geometryTolerance = 1.0e-9;
constexpr std::size_t maximumRecordedEvents = 64U;
constexpr std::size_t maximumDatagramBytes = 65507U;

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

    std::map<NetworkNodeId, DroneSocket> newSockets;
    for (const auto& [id, node] : updated)
    {
        static_cast<void>(node);
        if (!sockets_.contains(id))
            newSockets.try_emplace(id);
    }

    std::erase_if(sockets_, [&updated](const auto& entry)
    {
        return !updated.contains(entry.first);
    });
    for (auto& [id, socket] : newSockets)
        sockets_.emplace(id, std::move(socket));

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

std::optional<SocketPort> VirtualMeshNetwork::PortFor(
    NetworkNodeId node) const noexcept
{
    const auto found = sockets_.find(node);
    if (found == sockets_.end())
        return std::nullopt;
    return found->second.LocalPort();
}

std::optional<std::uint64_t> VirtualMeshNetwork::Send(
    DroneEndpoint source, DroneEndpoint destination,
    std::uint32_t sequence, std::string kind, std::string payload)
{
    const auto sourceNode = nodes_.find(source.node);
    const auto destinationNode = nodes_.find(destination.node);
    const auto sourceSocket = sockets_.find(source.node);
    const auto destinationSocket = sockets_.find(destination.node);
    if (source.node == destination.node || kind.empty()
        || sourceNode == nodes_.end() || destinationNode == nodes_.end()
        || sourceSocket == sockets_.end() || destinationSocket == sockets_.end()
        || sourceSocket->second.LocalPort() != source.port
        || destinationSocket->second.LocalPort() != destination.port
        || !LinkAvailable(source.node, destination.node))
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, source.node, destination.node,
            "드론 소켓 주소가 맞지 않거나 직접 연결이 없어 메시지를 보내지 못했습니다.");
        return std::nullopt;
    }

    VirtualMeshMessage message{
        .id = nextMessageId_++,
        .source = source.node,
        .destination = destination.node,
        .sourcePort = source.port,
        .destinationPort = destination.port,
        .sequence = sequence,
        .kind = std::move(kind),
        .payload = std::move(payload)};
    const std::uint64_t id = message.id;
    const std::uint64_t maximumTick = std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t dueTick = currentTick_ == maximumTick
        ? maximumTick : currentTick_ + 1U;
    const Json packet{
        {"id", message.id},
        {"source", message.source},
        {"destination", message.destination},
        {"sourcePort", message.sourcePort},
        {"destinationPort", message.destinationPort},
        {"sequence", message.sequence},
        {"kind", message.kind},
        {"payload", message.payload}};
    const std::string datagram = packet.dump();
    if (datagram.size() > maximumDatagramBytes)
    {
        ++metrics_.droppedMessages;
        RecordEvent(MeshEventKind::PacketDropped, source.node, destination.node,
            "UDP 메시지가 데이터그램 크기 제한을 넘었습니다.");
        return std::nullopt;
    }

    sourceSocket->second.SendTo(destination.port, datagram);
    pendingMessages_.emplace(id, PendingMessage{dueTick, source.node, destination.node});
    return id;
}

void VirtualMeshNetwork::Advance(std::uint64_t tick)
{
    if (tick < currentTick_)
        throw std::invalid_argument("가상 통신망 시간은 뒤로 갈 수 없습니다.");
    currentTick_ = tick;
    deliveredMessages_.clear();
    ReceiveDatagrams();

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

void VirtualMeshNetwork::ReceiveDatagrams()
{
    for (auto& [receiverId, socket] : sockets_)
    {
        while (std::optional<ReceivedDatagram> datagram = socket.Receive())
        {
            Json packet = Json::parse(datagram->payload, nullptr, false);
            if (packet.is_discarded() || !packet.is_object()
                || !packet.contains("id") || !packet["id"].is_number_unsigned()
                || !packet.contains("source") || !packet["source"].is_number_unsigned()
                || !packet.contains("destination")
                || !packet["destination"].is_number_unsigned()
                || !packet.contains("sourcePort")
                || !packet["sourcePort"].is_number_unsigned()
                || !packet.contains("destinationPort")
                || !packet["destinationPort"].is_number_unsigned()
                || !packet.contains("sequence") || !packet["sequence"].is_number_unsigned()
                || !packet.contains("kind") || !packet["kind"].is_string()
                || !packet.contains("payload") || !packet["payload"].is_string())
            {
                ++metrics_.droppedMessages;
                RecordEvent(MeshEventKind::PacketDropped, 0U, receiverId,
                    "형식이 맞지 않는 UDP 데이터그램을 버렸습니다.");
                continue;
            }

            const std::uint64_t id = packet["id"].get<std::uint64_t>();
            const auto pending = pendingMessages_.find(id);
            const auto sender = nodes_.find(packet["source"].get<NetworkNodeId>());
            const auto receiver = nodes_.find(receiverId);
            if (pending == pendingMessages_.end() || sender == nodes_.end()
                || receiver == nodes_.end()
                || pending->second.sender != sender->first
                || pending->second.receiver != receiverId
                || packet["destination"].get<NetworkNodeId>() != receiverId
                || packet["sourcePort"].get<SocketPort>() != datagram->sourcePort
                || packet["destinationPort"].get<SocketPort>()
                    != sockets_.at(receiverId).LocalPort())
            {
                ++metrics_.droppedMessages;
                RecordEvent(MeshEventKind::PacketDropped,
                    sender == nodes_.end() ? 0U : sender->first, receiverId,
                    "보낸 드론이나 받은 드론이 맞지 않는 UDP 데이터그램을 버렸습니다.");
                if (pending != pendingMessages_.end())
                    pendingMessages_.erase(pending);
                continue;
            }

            const std::uint64_t sourcePort = packet["sourcePort"].get<std::uint64_t>();
            const std::uint64_t destinationPort = packet["destinationPort"].get<std::uint64_t>();
            const std::uint64_t sequence = packet["sequence"].get<std::uint64_t>();
            if (sourcePort != sockets_.at(sender->first).LocalPort()
                || destinationPort > std::numeric_limits<SocketPort>::max()
                || sequence > std::numeric_limits<std::uint32_t>::max())
            {
                ++metrics_.droppedMessages;
                RecordEvent(MeshEventKind::PacketDropped, sender->first, receiverId,
                    "UDP 데이터그램의 포트나 순번이 허용 범위를 벗어났습니다.");
                pendingMessages_.erase(pending);
                continue;
            }

            VirtualMeshMessage message{
                .id = id,
                .source = sender->first,
                .destination = receiverId,
                .sourcePort = static_cast<SocketPort>(sourcePort),
                .destinationPort = static_cast<SocketPort>(destinationPort),
                .sequence = static_cast<std::uint32_t>(sequence),
                .kind = packet["kind"].get<std::string>(),
                .payload = packet["payload"].get<std::string>()};
            if (message.kind.empty())
            {
                ++metrics_.droppedMessages;
                RecordEvent(MeshEventKind::PacketDropped, sender->first, receiverId,
                    "종류가 비어 있는 UDP 데이터그램을 버렸습니다.");
                pendingMessages_.erase(pending);
                continue;
            }

            receivedMessages_.push_back({pending->second.dueTick,
                sender->first, receiverId, std::move(message)});
            pendingMessages_.erase(pending);
        }
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
