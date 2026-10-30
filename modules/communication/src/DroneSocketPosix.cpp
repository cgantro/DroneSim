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
// UDP 데이터그램 하나가 담을 수 있는 최대 사용자 데이터 크기다.
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
        // 각 드론은 프로세스 안에서 독립된 IPv4 UDP 소켓을 소유한다.
        const int created = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (created < 0)
            throw std::runtime_error(SocketError("드론 UDP 소켓 생성", errno));
        socket_.Reset(created);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        // 외부 네트워크와 통신하지 않고 같은 PC의 드론 소켓끼리만 주고받는다.
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        // 0번 포트를 요청하면 운영체제가 아직 사용되지 않는 포트를 골라 준다.
        address.sin_port = 0;
        if (::bind(socket_.Get(), reinterpret_cast<const sockaddr*>(&address),
                sizeof(address)) != 0)
            throw std::runtime_error(SocketError("드론 UDP 포트 연결", errno));

        // 수신 데이터가 없을 때 Fixed Tick이 멈추지 않도록 소켓을 논블로킹으로 둔다.
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
    // 네트워크 통신은 가상 무선 규칙을 통과한 뒤 로컬 UDP로만 전달한다.
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    destination.sin_port = htons(destinationPort);
    const ssize_t sent = sendto(impl_->socket_.Get(), payload.data(), payload.size(), 0,
        reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
    if (sent < 0 || static_cast<std::size_t>(sent) != payload.size())
        throw std::runtime_error(SocketError("드론 UDP 전송", errno));
}

std::optional<ReceivedDatagram> DroneSocket::Receive()
{
    // 최대 UDP 크기보다 한 바이트 크게 받아 비정상적으로 큰 데이터도 잘라내지 않는다.
    std::array<char, maximumDatagramBytes + 1U> buffer{};
    sockaddr_in source{};
    socklen_t sourceLength = sizeof(source);
    const ssize_t received = recvfrom(impl_->socket_.Get(), buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr*>(&source), &sourceLength);
    if (received < 0)
    {
        // 논블로킹 소켓에서 아직 데이터가 없다는 것은 정상적인 상태다.
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            return std::nullopt;
        throw std::runtime_error(SocketError("드론 UDP 수신", errno));
    }
    return ReceivedDatagram{ntohs(source.sin_port),
        std::string(buffer.data(), static_cast<std::size_t>(received))};
}
}
