#include "communication/DroneSocket.h"

#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace dronesim::communication
{
namespace
{
// IPv4 UDP 데이터그램 하나가 담을 수 있는 최대 사용자 데이터 크기다.
constexpr std::size_t maximumDatagramBytes = 65507U;

class WinsockSession
{
public:
    WinsockSession()
    {
        // 프로세스에서 Winsock을 먼저 초기화해야 소켓을 만들 수 있다.
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
            throw std::runtime_error("UDP 통신을 시작하지 못했습니다.");
    }

    // Impl의 멤버 순서상 UDP 소켓이 닫힌 다음 Winsock을 종료한다.
    ~WinsockSession() { WSACleanup(); }
    WinsockSession(const WinsockSession&) = delete;
    WinsockSession& operator=(const WinsockSession&) = delete;
};

class SocketHandle
{
public:
    SocketHandle() = default;
    ~SocketHandle()
    {
        if (value_ != INVALID_SOCKET)
            closesocket(value_);
    }
    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;
    [[nodiscard]] SOCKET Get() const noexcept { return value_; }
    void Reset(SOCKET value) noexcept { value_ = value; }

private:
    SOCKET value_ = INVALID_SOCKET;
};

std::string SocketError(const char* operation, int error)
{
    return std::string(operation) + "에 실패했습니다. Winsock 오류: "
        + std::to_string(error);
}
}

class DroneSocket::Impl
{
public:
    Impl()
    {
        // 각 드론은 독립된 IPv4 UDP 소켓을 가진다.
        const SOCKET created = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (created == INVALID_SOCKET)
            throw std::runtime_error(SocketError("드론 UDP 소켓 생성", WSAGetLastError()));
        socket_.Reset(created);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        // 같은 PC 안에서만 주고받도록 루프백 주소에 바인딩한다.
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        // 포트 0은 사용 가능한 포트를 운영체제가 선택하게 한다.
        address.sin_port = 0;
        if (::bind(socket_.Get(), reinterpret_cast<const sockaddr*>(&address),
                sizeof(address)) == SOCKET_ERROR)
            throw std::runtime_error(SocketError("드론 UDP 포트 연결", WSAGetLastError()));

        // 수신을 기다리며 시뮬레이션 Tick이 멈추지 않도록 논블로킹으로 설정한다.
        u_long nonBlocking = 1;
        if (ioctlsocket(socket_.Get(), FIONBIO, &nonBlocking) == SOCKET_ERROR)
            throw std::runtime_error(SocketError("드론 UDP 비동기 설정", WSAGetLastError()));

        int addressLength = sizeof(address);
        if (getsockname(socket_.Get(), reinterpret_cast<sockaddr*>(&address),
                &addressLength) == SOCKET_ERROR)
            throw std::runtime_error(SocketError("드론 UDP 포트 확인", WSAGetLastError()));
        localPort_ = ntohs(address.sin_port);
    }

    WinsockSession winsock_;
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
    // 가상 거리·가림 검사를 통과한 데이터만 로컬 UDP 포트로 보낸다.
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    destination.sin_port = htons(destinationPort);
    const int sent = sendto(impl_->socket_.Get(), payload.data(),
        static_cast<int>(payload.size()), 0,
        reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
    if (sent == SOCKET_ERROR || static_cast<std::size_t>(sent) != payload.size())
        throw std::runtime_error(SocketError("드론 UDP 전송", WSAGetLastError()));
}

std::optional<ReceivedDatagram> DroneSocket::Receive()
{
    // 허용 크기보다 한 바이트 크게 받아 초과 데이터그램을 식별한다.
    std::array<char, maximumDatagramBytes + 1U> buffer{};
    sockaddr_in source{};
    int sourceLength = sizeof(source);
    const int received = recvfrom(impl_->socket_.Get(), buffer.data(),
        static_cast<int>(buffer.size()), 0,
        reinterpret_cast<sockaddr*>(&source), &sourceLength);
    if (received == SOCKET_ERROR)
    {
        const int error = WSAGetLastError();
        // 아직 데이터가 없거나 일시적인 네트워크 오류면 다음 Tick에 다시 확인한다.
        if (error == WSAEWOULDBLOCK || error == WSAEINTR || error == WSAECONNRESET)
            return std::nullopt;
        if (error != WSAEMSGSIZE)
            throw std::runtime_error(SocketError("드론 UDP 수신", error));
    }
    const std::size_t receivedBytes = received == SOCKET_ERROR
        ? buffer.size() : static_cast<std::size_t>(received);
    return ReceivedDatagram{ntohs(source.sin_port),
        std::string(buffer.data(), receivedBytes)};
}
}
