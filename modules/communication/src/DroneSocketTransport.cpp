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
constexpr std::size_t maximumDatagramBytes = 65507U;
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
    if (datagram.size() > maximumDatagramBytes)
        return false;

    sender->second.SendTo(receiver->second.LocalPort(), datagram);
    return true;
}

SocketReceiveBatch DroneSocketTransport::ReceiveAll()
{
    SocketReceiveBatch result;
    for (auto& [destination, socket] : sockets_)
    {
        while (std::optional<ReceivedDatagram> datagram = socket.Receive())
        {
            Json fields = Json::parse(datagram->payload, nullptr, false);
            if (fields.is_discarded() || !fields.is_object()
                || !fields.contains("id") || !fields["id"].is_number_unsigned()
                || !fields.contains("source") || !fields["source"].is_number_unsigned()
                || !fields.contains("destination")
                || !fields["destination"].is_number_unsigned()
                || !fields.contains("sequence")
                || !fields["sequence"].is_number_unsigned()
                || !fields.contains("kind") || !fields["kind"].is_string()
                || !fields.contains("payload") || !fields["payload"].is_string())
            {
                ++result.droppedDatagrams;
                continue;
            }

            const auto sender = std::find_if(sockets_.begin(), sockets_.end(),
                [&datagram](const auto& entry)
                {
                    return entry.second.LocalPort() == datagram->sourcePort;
                });
            const NetworkNodeId claimedSource = fields["source"].get<NetworkNodeId>();
            const NetworkNodeId claimedDestination =
                fields["destination"].get<NetworkNodeId>();
            const std::uint64_t sequence = fields["sequence"].get<std::uint64_t>();
            if (sender == sockets_.end() || sender->first != claimedSource
                || destination != claimedDestination
                || sequence > std::numeric_limits<std::uint32_t>::max()
                || fields["kind"].get<std::string>().empty())
            {
                ++result.droppedDatagrams;
                continue;
            }

            result.packets.push_back({
                fields["id"].get<std::uint64_t>(), sender->first, destination,
                static_cast<std::uint32_t>(sequence),
                fields["kind"].get<std::string>(),
                fields["payload"].get<std::string>()});
        }
    }
    return result;
}
}
