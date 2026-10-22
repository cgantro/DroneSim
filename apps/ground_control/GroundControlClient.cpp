#include "GroundControlClient.h"

#include "communication/UdpSocket.h"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace aerosimlab::ground_control
{
CommandResult SendCommand(
    std::string_view host,
    std::uint16_t port,
    const communication::CommandPacket& packet,
    communication::UdpSocket& socket,
    std::chrono::milliseconds timeout,
    std::stop_token stopToken)
{
    if (timeout <= std::chrono::milliseconds::zero())
        throw std::invalid_argument("응답 대기 시간은 0보다 커야 합니다.");

    const auto destination = communication::UdpSocket::Resolve(host, port);
    const std::string datagram = communication::EncodeCommand(packet);
    const auto requestStarted = std::chrono::steady_clock::now();
    const auto deadline = requestStarted + timeout;
    auto nextRetry = requestStarted;

    while (!stopToken.stop_requested()
        && std::chrono::steady_clock::now() < deadline)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= nextRetry)
        {
            socket.Send(datagram, destination);
            nextRetry = now + std::chrono::milliseconds(400);
        }

        communication::UdpPeer source;
        if (const auto response = socket.Receive(source); response &&
            source.addressNetworkOrder == destination.addressNetworkOrder &&
            source.portNetworkOrder == destination.portNetworkOrder)
        {
            const auto reply = communication::DecodeReply(*response);
            if (reply.commandId != packet.commandId || reply.sequence != packet.sequence)
                continue;

            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - requestStarted).count();
            return {reply, static_cast<std::uint64_t>(elapsed)};
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (stopToken.stop_requested())
        throw CommandCancelled("시뮬레이터 요청이 취소되었습니다.");
    throw CommandTimeout("시뮬레이터 응답 시간이 초과되었습니다.");
}
}
