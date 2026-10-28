#include "communication/UdpSocketPlatform.h"

#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>

#include "communication/Protocol.h"

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace aerosimlab::communication::detail
{
namespace
{
class WinsockSession
{
public:
    WinsockSession()
    {
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
            throw std::runtime_error("Winsock을 초기화하지 못했습니다.");
    }

    ~WinsockSession() { WSACleanup(); }
    WinsockSession(const WinsockSession&) = delete;
    WinsockSession& operator=(const WinsockSession&) = delete;
};

class SocketHandle
{
public:
    explicit SocketHandle(SOCKET value = INVALID_SOCKET) noexcept : value_(value) {}
    ~SocketHandle()
    {
        if (value_ != INVALID_SOCKET)
            closesocket(value_);
    }

    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;
    SocketHandle(SocketHandle&& other) noexcept
        : value_(std::exchange(other.value_, INVALID_SOCKET))
    {
    }
    SocketHandle& operator=(SocketHandle&& other) noexcept
    {
        if (this == &other)
            return *this;
        if (value_ != INVALID_SOCKET)
            closesocket(value_);
        value_ = std::exchange(other.value_, INVALID_SOCKET);
        return *this;
    }

    [[nodiscard]] SOCKET Get() const noexcept { return value_; }

private:
    SOCKET value_ = INVALID_SOCKET;
};

std::string SocketError(const char* operation, int errorCode)
{
    return std::string(operation) + " 실패 (Winsock 오류 "
        + std::to_string(errorCode) + ")";
}

void ValidateDatagramSize(std::string_view datagram)
{
    if (datagram.empty() || datagram.size() > maximumDatagramBytes)
        throw std::invalid_argument("UDP 전송 메시지 크기가 허용 범위를 벗어났습니다.");
}

class WindowsUdpSocket final : public UdpSocketPlatform
{
public:
    explicit WindowsUdpSocket(std::uint16_t localPort)
    {
        const SOCKET created = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (created == INVALID_SOCKET)
            throw std::runtime_error(SocketError("UDP 소켓 생성", WSAGetLastError()));
        socket_ = SocketHandle(created);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(localPort);
        if (::bind(socket_.Get(), reinterpret_cast<const sockaddr*>(&address),
                sizeof(address)) == SOCKET_ERROR)
            throw std::runtime_error(SocketError("UDP 포트 연결", WSAGetLastError()));

        u_long nonBlocking = 1;
        if (ioctlsocket(socket_.Get(), FIONBIO, &nonBlocking) == SOCKET_ERROR)
            throw std::runtime_error(SocketError("UDP non-blocking 설정", WSAGetLastError()));
    }

    [[nodiscard]] std::optional<std::string> Receive(UdpPeer& peer) override
    {
        // 크기 제한보다 한 바이트 더 받아 큰 데이터그램을 정상 패킷과 구분한다.
        std::array<char, maximumDatagramBytes + 1U> buffer{};
        sockaddr_in source{};
        int sourceLength = sizeof(source);
        const int received = recvfrom(socket_.Get(), buffer.data(),
            static_cast<int>(buffer.size()), 0,
            reinterpret_cast<sockaddr*>(&source), &sourceLength);
        if (received == SOCKET_ERROR)
        {
            const int error = WSAGetLastError();
            // 응답이 없는 포트에서 온 ICMP 오류는 다음 UDP 요청을 막지 않는다.
            if (error == WSAEWOULDBLOCK || error == WSAEINTR
                || error == WSAECONNRESET)
                return std::nullopt;
            if (error == WSAEMSGSIZE)
            {
                peer = {source.sin_addr.s_addr, source.sin_port};
                return std::string(maximumDatagramBytes + 1U, ' ');
            }
            throw std::runtime_error(SocketError("UDP 수신", error));
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
        const int sent = sendto(socket_.Get(), datagram.data(),
            static_cast<int>(datagram.size()), 0,
            reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
        if (sent == SOCKET_ERROR
            || static_cast<std::size_t>(sent) != datagram.size())
            throw std::runtime_error(SocketError("UDP 전송", WSAGetLastError()));
    }

    [[nodiscard]] std::uint16_t LocalPort() const override
    {
        sockaddr_in address{};
        int addressLength = sizeof(address);
        if (getsockname(socket_.Get(), reinterpret_cast<sockaddr*>(&address),
                &addressLength) == SOCKET_ERROR)
            throw std::runtime_error(SocketError("UDP 로컬 포트 확인", WSAGetLastError()));
        return ntohs(address.sin_port);
    }

private:
    // 소켓이 먼저 닫힌 뒤 Winsock 초기화 참조 수를 내려가도록 순서를 둔다.
    WinsockSession winsock_;
    SocketHandle socket_;
};
}

std::unique_ptr<UdpSocketPlatform> CreateUdpSocketPlatform(std::uint16_t localPort)
{
    return std::make_unique<WindowsUdpSocket>(localPort);
}

UdpPeer ResolveIpv4(std::string_view host, std::uint16_t port)
{
    WinsockSession winsock;
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
    if (!inet_ntop(AF_INET, &address, text.data(), text.size()))
        throw std::runtime_error("UDP 상대방 IPv4 주소를 문자열로 바꾸지 못했습니다.");
    return text.data();
}
}
