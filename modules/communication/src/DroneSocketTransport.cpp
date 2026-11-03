#include "communication/DroneSocketTransport.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace dronesim::communication
{
namespace
{
using Json = nlohmann::json;

/**
 * @brief 데이터그램 안의 JSON을 통신 패킷으로 해석한다.
 * @details JSON 형식과 자료형만 검사한다. 송신자가 등록된 드론인지,
 *          실제 송신 포트와 일치하는지는 소켓을 아는 DroneSocketTransport가 확인한다.
 */
std::optional<SocketPacket> DecodePacket(std::string_view payload)
{
    const Json fields = Json::parse(payload, nullptr, false);
    if (fields.is_discarded() || !fields.is_object()
        || !fields.contains("id") || !fields["id"].is_number_unsigned()
        || !fields.contains("source") || !fields["source"].is_number_unsigned()
        || !fields.contains("destination")
        || !fields["destination"].is_number_unsigned()
        || !fields.contains("sequence")
        || !fields["sequence"].is_number_unsigned()
        || !fields.contains("kind") || !fields["kind"].is_string()
        || !fields.contains("payload") || !fields["payload"].is_string())
        return std::nullopt;

    const std::uint64_t sequence = fields["sequence"].get<std::uint64_t>();
    std::string kind = fields["kind"].get<std::string>();
    if (sequence > std::numeric_limits<std::uint32_t>::max() || kind.empty())
        return std::nullopt;

    return SocketPacket{
        fields["id"].get<std::uint64_t>(),
        fields["source"].get<NetworkNodeId>(),
        fields["destination"].get<NetworkNodeId>(),
        static_cast<std::uint32_t>(sequence),
        std::move(kind),
        fields["payload"].get<std::string>()};
}
}

void DroneSocketTransport::UpdateNodes(std::span<const NetworkNodeId> nodeIds)
{
    std::set<NetworkNodeId> currentNodes(nodeIds.begin(), nodeIds.end());
    std::map<NetworkNodeId, DroneSocket> newSockets;
    for (const NetworkNodeId nodeId : currentNodes)
    {
        if (!sockets_.contains(nodeId))
            newSockets.try_emplace(nodeId);
    }

    std::erase_if(sockets_, [&currentNodes](const auto& entry)
    {
        return !currentNodes.contains(entry.first);
    });
    for (auto& [nodeId, socket] : newSockets)
        sockets_.emplace(nodeId, std::move(socket));
}

std::optional<SocketPort> DroneSocketTransport::PortFor(
    NetworkNodeId node) const noexcept
{
    const auto found = sockets_.find(node);
    if (found == sockets_.end())
        return std::nullopt;
    return found->second.LocalPort();
}

bool DroneSocketTransport::Send(const SocketPacket& packet) const
{
    const auto sender = sockets_.find(packet.source);
    const auto receiver = sockets_.find(packet.destination);
    if (sender == sockets_.end() || receiver == sockets_.end())
        throw std::logic_error("등록되지 않은 드론 소켓으로 보내려 했습니다.");

    // UDP에는 바이트만 보낼 수 있어 메시지 필드를 JSON 한 덩어리로 묶는다.
    const std::string datagram = Json{
        {"id", packet.id},
        {"source", packet.source},
        {"destination", packet.destination},
        {"sequence", packet.sequence},
        {"kind", packet.kind},
        {"payload", packet.payload}}.dump();
    if (datagram.size() > maximumDatagramPayloadBytes)
        return false;

    sender->second.SendTo(receiver->second.LocalPort(), datagram);
    return true;
}

SocketReceiveBatch DroneSocketTransport::ReceiveAll()
{
    SocketReceiveBatch result;
    for (auto& [destination, socket] : sockets_)
    {
        // UDP 수신함은 드론마다 따로 있으므로, 이번 Tick에 도착한 데이터그램을 모두 비운다.
        while (std::optional<ReceivedDatagram> datagram = socket.Receive())
        {
            std::optional<SocketPacket> packet = DecodePacket(datagram->payload);
            if (!packet)
            {
                ++result.droppedDatagrams;
                continue;
            }

            const auto sender = sockets_.find(packet->source);
            // 본문에 적힌 출발 드론과 운영체제가 알려 준 포트가 맞는지 확인한다.
            // 목적지 ID도 현재 수신함의 주인과 같아야 다른 드론의 패킷을 잘못 받지 않는다.
            if (sender == sockets_.end()
                || sender->second.LocalPort() != datagram->sourcePort
                || destination != packet->destination)
            {
                ++result.droppedDatagrams;
                continue;
            }

            result.packets.push_back(std::move(*packet));
        }
    }
    return result;
}
}
