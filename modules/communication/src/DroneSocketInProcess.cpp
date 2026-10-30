#include "communication/DroneSocket.h"

#include <deque>
#include <map>
#include <stdexcept>
#include <utility>

namespace aerosimlab::communication
{
namespace
{
struct InProcessDatagram
{
    SocketPort sourcePort = 0;
    std::string payload;
};

struct InProcessBus
{
    std::map<SocketPort, std::deque<InProcessDatagram>> queues;
    SocketPort nextPort = 30000U;
};

InProcessBus& Bus()
{
    static InProcessBus bus;
    return bus;
}
}

class DroneSocket::Impl
{
public:
    Impl()
    {
        InProcessBus& bus = Bus();
        const SocketPort firstCandidate = bus.nextPort;
        while (bus.queues.contains(bus.nextPort))
        {
            bus.nextPort = bus.nextPort == 65535U
                ? 30000U : static_cast<SocketPort>(bus.nextPort + 1U);
            if (bus.nextPort == firstCandidate)
                throw std::runtime_error("브라우저에서 사용할 통신 번호를 더 만들 수 없습니다.");
        }
        localPort_ = bus.nextPort;
        bus.queues.emplace(localPort_, std::deque<InProcessDatagram>{});
        bus.nextPort = bus.nextPort == 65535U
            ? 30000U : static_cast<SocketPort>(bus.nextPort + 1U);
    }

    ~Impl() { Bus().queues.erase(localPort_); }

    SocketPort localPort_ = 0;
};

DroneSocket::DroneSocket() : impl_(std::make_unique<Impl>()) {}
DroneSocket::~DroneSocket() = default;
DroneSocket::DroneSocket(DroneSocket&& other) noexcept = default;
DroneSocket& DroneSocket::operator=(DroneSocket&& other) noexcept = default;

SocketPort DroneSocket::LocalPort() const noexcept
{
    return impl_->localPort_;
}

void DroneSocket::SendTo(SocketPort destinationPort, std::string_view payload) const
{
    if (destinationPort == 0U || payload.empty() || payload.size() > 65507U)
        throw std::invalid_argument("드론 통신 메시지의 포트나 크기를 확인해 주세요.");
    auto& queues = Bus().queues;
    const auto destination = queues.find(destinationPort);
    if (destination == queues.end())
        return;
    destination->second.push_back({impl_->localPort_, std::string(payload)});
}

std::optional<ReceivedDatagram> DroneSocket::Receive()
{
    auto& queue = Bus().queues.at(impl_->localPort_);
    if (queue.empty())
        return std::nullopt;
    InProcessDatagram datagram = std::move(queue.front());
    queue.pop_front();
    return ReceivedDatagram{datagram.sourcePort, std::move(datagram.payload)};
}
}
