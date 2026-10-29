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
    nodes_ = std::move(updated);
    obstacles_.assign(obstacles.begin(), obstacles.end());
    std::erase_if(radioDisabledNodes_, [this](NetworkNodeId id)
    {
        return !nodes_.contains(id);
    });
    std::erase_if(disabledLinks_, [this](const NodePair& pair)
    {
        return !nodes_.contains(pair.first) || !nodes_.contains(pair.second);
    });
}

void VirtualMeshNetwork::SetLinkDisabled(
    NetworkNodeId first, NetworkNodeId second, bool disabled)
{
    if (first == second)
        throw std::invalid_argument("한 노드와 자기 자신 사이에는 링크 장애를 설정할 수 없습니다.");
    const NodePair pair = MakePair(first, second);
    const bool changed = disabled
        ? disabledLinks_.insert(pair).second
        : disabledLinks_.erase(pair) != 0U;
    if (!changed)
        return;
    RecordEvent(disabled ? MeshEventKind::LinkDisabled : MeshEventKind::LinkRestored,
        first, second, disabled ? "직접 연결을 끊었습니다." : "직접 연결을 복구했습니다.");
}

bool VirtualMeshNetwork::IsLinkDisabled(
    NetworkNodeId first, NetworkNodeId second) const noexcept
{
    return first != second && disabledLinks_.contains(MakePair(first, second));
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
    if (source == destination || kind.empty()
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
    Transmit(source, destination, std::move(message));
    return id;
}

void VirtualMeshNetwork::Advance(std::uint64_t tick)
{
    if (tick < currentTick_)
        throw std::invalid_argument("가상 통신망 시간은 뒤로 갈 수 없습니다.");
    currentTick_ = tick;
    deliveredMessages_.clear();
    std::sort(transmissions_.begin(), transmissions_.end(),
        [](const Transmission& first, const Transmission& second)
        {
            return first.dueTick < second.dueTick
                || (first.dueTick == second.dueTick && first.order < second.order);
        });
    auto split = std::upper_bound(transmissions_.begin(), transmissions_.end(), tick,
        [](std::uint64_t current, const Transmission& transmission)
        {
            return current < transmission.dueTick;
        });
    for (auto item = transmissions_.begin(); item != split; ++item)
    {
        if (!LinkAvailable(item->sender, item->receiver))
        {
            ++metrics_.droppedMessages;
            RecordEvent(MeshEventKind::PacketDropped, item->sender, item->receiver,
                "전송 중 직접 연결이 끊겼습니다.");
            continue;
        }
        RecordEvent(MeshEventKind::MessageDelivered,
            item->message.source, item->message.destination,
            item->message.kind + " 메시지가 도착했습니다.");
        deliveredMessages_.push_back({std::move(item->message), currentTick_});
        ++metrics_.deliveredMessages;
    }
    transmissions_.erase(transmissions_.begin(), split);
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

bool VirtualMeshNetwork::LinkAvailableAt(NetworkNodeId sender,
    NetworkNodeId receiver, const glm::dvec3& senderPositionMeters,
    const glm::dvec3& receiverPositionMeters) const
{
    const auto senderNode = nodes_.find(sender);
    const auto receiverNode = nodes_.find(receiver);
    if (senderNode == nodes_.end() || receiverNode == nodes_.end()
        || !senderNode->second.powered || !receiverNode->second.powered
        || !senderNode->second.canTransmit || !receiverNode->second.canReceive
        || radioDisabledNodes_.contains(sender) || radioDisabledNodes_.contains(receiver)
        || disabledLinks_.contains(MakePair(sender, receiver)))
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

void VirtualMeshNetwork::Transmit(NetworkNodeId sender, NetworkNodeId receiver,
    VirtualMeshMessage message)
{
    const std::uint64_t delay = std::max<std::uint64_t>(settings_.linkLatencyTicks, 1U);
    const std::uint64_t maximumTick = std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t dueTick = currentTick_ > maximumTick - delay
        ? maximumTick : currentTick_ + delay;
    transmissions_.push_back({dueTick, nextTransmissionOrder_++,
        sender, receiver, std::move(message)});
}

void VirtualMeshNetwork::RecordEvent(MeshEventKind kind,
    NetworkNodeId source, NetworkNodeId destination, std::string detail)
{
    if (events_.size() == maximumRecordedEvents)
        events_.erase(events_.begin());
    events_.push_back({currentTick_, kind, source, destination, std::move(detail)});
}
}
