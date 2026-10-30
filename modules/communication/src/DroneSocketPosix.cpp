#include "communication/DroneSocket.h"

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace aerosimlab::communication
{
namespace
{
constexpr std::size_t maximumDatagramBytes = 65507U;

class SocketHandle
{
public:
    SocketHandle() = default;
    ~SocketHandle()
    {
        if (value_ >= 0)
            close(value_);
    }
    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;
    [[nodiscard]] int Get() const noexcept { return value_; }
    void Reset(int value) noexcept { value_ = value; }

private:
    int value_ = -1;
};

std::string SocketError(const char* operation, int error)
{
    return std::string(operation) + "에 실패했습니다. 오류: "
        + std::to_string(error);
}
}

class DroneSocket::Impl
{
public:
    Impl()
    {
        const int created = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (created < 0)
            throw std::runtime_error(SocketError("드론 UDP 소켓 생성", errno));
        socket_.Reset(created);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (::bind(socket_.Get(), reinterpret_cast<const sockaddr*>(&address),
                sizeof(address)) != 0)
            throw std::runtime_error(SocketError("드론 UDP 포트 연결", errno));

        const int flags = fcntl(socket_.Get(), F_GETFL, 0);
        if (flags < 0 || fcntl(socket_.Get(), F_SETFL, flags | O_NONBLOCK) < 0)
            throw std::runtime_error(SocketError("드론 UDP 비동기 설정", errno));

        socklen_t addressLength = sizeof(address);
        if (getsockname(socket_.Get(), reinterpret_cast<sockaddr*>(&address),
                &addressLength) != 0)
            throw std::runtime_error(SocketError("드론 UDP 포트 확인", errno));
        localPort_ = ntohs(address.sin_port);
    }

    SocketHandle socket_;
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
    if (destinationPort == 0U || payload.empty() || payload.size() > maximumDatagramBytes)
        throw std::invalid_argument("드론 UDP 메시지의 포트나 크기를 확인해 주세요.");

    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    destination.sin_port = htons(destinationPort);
    const ssize_t sent = sendto(impl_->socket_.Get(), payload.data(), payload.size(), 0,
        reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
    if (sent < 0 || static_cast<std::size_t>(sent) != payload.size())
        throw std::runtime_error(SocketError("드론 UDP 전송", errno));
}

std::optional<ReceivedDatagram> DroneSocket::Receive()
{
    std::array<char, maximumDatagramBytes + 1U> buffer{};
    sockaddr_in source{};
    socklen_t sourceLength = sizeof(source);
    const ssize_t received = recvfrom(impl_->socket_.Get(), buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr*>(&source), &sourceLength);
    if (received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            return std::nullopt;
        throw std::runtime_error(SocketError("드론 UDP 수신", errno));
    }
    return ReceivedDatagram{ntohs(source.sin_port),
        std::string(buffer.data(), static_cast<std::size_t>(received))};
}
}
