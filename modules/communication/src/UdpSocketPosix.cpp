#include "UdpSocketPlatform.h"

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "communication/Protocol.h"

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace aerosimlab::communication::detail
{
namespace
{
class SocketHandle
{
public:
    explicit SocketHandle(int value = -1) noexcept : value_(value) {}
    ~SocketHandle()
    {
        if (value_ >= 0)
            close(value_);
    }

    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;
    SocketHandle(SocketHandle&& other) noexcept
        : value_(std::exchange(other.value_, -1))
    {
    }
    SocketHandle& operator=(SocketHandle&& other) noexcept
    {
        if (this == &other)
            return *this;
        if (value_ >= 0)
            close(value_);
        value_ = std::exchange(other.value_, -1);
        return *this;
    }

    [[nodiscard]] int Get() const noexcept { return value_; }

private:
    int value_ = -1;
};

std::string SocketError(const char* operation, int errorCode)
{
    return std::string(operation) + " 실패 (오류 "
        + std::to_string(errorCode) + ")";
}

void ValidateDatagramSize(std::string_view datagram)
{
    if (datagram.empty() || datagram.size() > maximumDatagramBytes)
        throw std::invalid_argument("UDP 전송 메시지 크기가 허용 범위를 벗어났습니다.");
}

class PosixUdpSocket final : public UdpSocketPlatform
{
public:
    explicit PosixUdpSocket(std::uint16_t localPort)
    {
        const int created = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (created < 0)
            throw std::runtime_error(SocketError("UDP 소켓 생성", errno));
        socket_ = SocketHandle(created);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(localPort);
        if (::bind(socket_.Get(), reinterpret_cast<const sockaddr*>(&address),
                sizeof(address)) != 0)
            throw std::runtime_error(SocketError("UDP 포트 연결", errno));

        const int flags = fcntl(socket_.Get(), F_GETFL, 0);
        if (flags < 0 || fcntl(socket_.Get(), F_SETFL, flags | O_NONBLOCK) < 0)
            throw std::runtime_error(SocketError("UDP non-blocking 설정", errno));
    }

    [[nodiscard]] std::optional<std::string> Receive(UdpPeer& peer) override
    {
        // 크기 제한보다 한 바이트 더 받아 큰 데이터그램을 정상 패킷과 구분한다.
        std::array<char, maximumDatagramBytes + 1U> buffer{};
        sockaddr_in source{};
        socklen_t sourceLength = sizeof(source);
        const ssize_t received = recvfrom(socket_.Get(), buffer.data(), buffer.size(), 0,
            reinterpret_cast<sockaddr*>(&source), &sourceLength);
        if (received < 0)
        {
            const int errorCode = errno;
            if (errorCode == EAGAIN || errorCode == EWOULDBLOCK
                || errorCode == EINTR)
                return std::nullopt;
            throw std::runtime_error(SocketError("UDP 수신", errorCode));
        }

        peer = {source.sin_addr.s_addr, source.sin_port};
        return std::string(buffer.data(), static_cast<std::size_t>(received));
    }

    void Send(std::string_view datagram, const UdpPeer& peer) const override
    {
        ValidateDatagramSize(datagram);
        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_addr.s_addr = peer.addressNetworkOrder;
        destination.sin_port = peer.portNetworkOrder;
        const ssize_t sent = sendto(socket_.Get(), datagram.data(), datagram.size(), 0,
            reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
        if (sent < 0 || static_cast<std::size_t>(sent) != datagram.size())
            throw std::runtime_error(SocketError("UDP 전송", errno));
    }

    [[nodiscard]] std::uint16_t LocalPort() const override
    {
        sockaddr_in address{};
        socklen_t addressLength = sizeof(address);
        if (getsockname(socket_.Get(), reinterpret_cast<sockaddr*>(&address),
                &addressLength) != 0)
            throw std::runtime_error(SocketError("UDP 로컬 포트 확인", errno));
        return ntohs(address.sin_port);
    }

private:
    SocketHandle socket_;
};
}

std::unique_ptr<UdpSocketPlatform> CreateUdpSocketPlatform(std::uint16_t localPort)
{
    return std::make_unique<PosixUdpSocket>(localPort);
}

UdpPeer ResolveIpv4(std::string_view host, std::uint16_t port)
{
    const std::string hostText(host);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, hostText.c_str(), &address.sin_addr) != 1)
        throw std::invalid_argument("IPv4 주소는 숫자 형식으로 입력해야 합니다.");
    return {address.sin_addr.s_addr, address.sin_port};
}

std::string FormatIpv4Address(std::uint32_t addressNetworkOrder)
{
    std::array<char, INET_ADDRSTRLEN> text{};
    in_addr address{};
    address.s_addr = addressNetworkOrder;
    if (!inet_ntop(AF_INET, &address, text.data(),
            static_cast<socklen_t>(text.size())))
        throw std::runtime_error("UDP 상대방 IPv4 주소를 문자열로 바꾸지 못했습니다.");
    return text.data();
}
}
