#include "communication/UdpSocket.h"

#include "communication/Protocol.h"

#include <array>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace aerosimlab::communication
{
namespace
{
#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket invalidSocket = INVALID_SOCKET;
NativeSocket Native(std::uintptr_t handle) noexcept { return static_cast<NativeSocket>(handle); }
int LastSocketError() noexcept { return WSAGetLastError(); }
std::string SocketError(const char* operation, int errorCode)
{
    return std::string(operation) + " 실패 (Winsock 오류 " + std::to_string(errorCode) + ")";
}
#else
using NativeSocket = int;
constexpr NativeSocket invalidSocket = -1;
NativeSocket Native(std::uintptr_t handle) noexcept { return static_cast<NativeSocket>(handle); }
int LastSocketError() noexcept { return errno; }
std::string SocketError(const char* operation, int errorCode)
{
    return std::string(operation) + " 실패 (오류 " + std::to_string(errorCode) + ")";
}
#endif
}

UdpSocket::UdpSocket(std::uint16_t localPort)
{
#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
        throw std::runtime_error("Winsock을 초기화하지 못했습니다.");
    winsockStarted_ = true;
#endif

    const NativeSocket socketHandle = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socketHandle == invalidSocket)
    {
        const int errorCode = LastSocketError();
#ifdef _WIN32
        if (winsockStarted_)
            WSACleanup();
#endif
        throw std::runtime_error(SocketError("UDP 소켓 생성", errorCode));
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(localPort);
    if (::bind(socketHandle, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0)
    {
        const int errorCode = LastSocketError();
#ifdef _WIN32
        closesocket(socketHandle);
        if (winsockStarted_)
            WSACleanup();
        winsockStarted_ = false;
#else
        close(socketHandle);
#endif
        throw std::runtime_error(SocketError("UDP 포트 연결", errorCode));
    }

#ifdef _WIN32
    u_long nonBlocking = 1;
    if (ioctlsocket(socketHandle, FIONBIO, &nonBlocking) != 0)
    {
        const int errorCode = LastSocketError();
        closesocket(socketHandle);
        if (winsockStarted_)
            WSACleanup();
        winsockStarted_ = false;
        throw std::runtime_error(SocketError("UDP non-blocking 설정", errorCode));
    }
#else
    const int flags = fcntl(socketHandle, F_GETFL, 0);
    if (flags < 0 || fcntl(socketHandle, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        const int errorCode = LastSocketError();
        close(socketHandle);
        throw std::runtime_error(SocketError("UDP non-blocking 설정", errorCode));
    }
#endif
    handle_ = static_cast<std::uintptr_t>(socketHandle);
}

UdpSocket::~UdpSocket()
{
    const NativeSocket socketHandle = Native(handle_);
    if (socketHandle != invalidSocket)
    {
#ifdef _WIN32
        closesocket(socketHandle);
#else
        close(socketHandle);
#endif
    }
#ifdef _WIN32
    if (winsockStarted_)
        WSACleanup();
#endif
}

std::optional<std::string> UdpSocket::Receive(UdpPeer& peer)
{
    std::array<char, maximumDatagramBytes + 1> buffer{};
    sockaddr_in source{};
#ifdef _WIN32
    int sourceLength = sizeof(source);
    const int received = recvfrom(Native(handle_), buffer.data(), static_cast<int>(buffer.size()), 0,
        reinterpret_cast<sockaddr*>(&source), &sourceLength);
    if (received == SOCKET_ERROR)
    {
        const int error = WSAGetLastError();
        // UDP 응답이 없는 포트에 보낸 직후 Windows가 ICMP 오류를 한 번 전달할 수 있다.
        // 이는 소켓 자체의 고장이 아니므로 다음 재전송과 응답을 계속 기다린다.
        if (error == WSAEWOULDBLOCK || error == WSAEINTR || error == WSAECONNRESET)
            return std::nullopt;
        if (error == WSAEMSGSIZE)
        {
            peer.addressNetworkOrder = source.sin_addr.s_addr;
            peer.portNetworkOrder = source.sin_port;
            return std::string(maximumDatagramBytes + 1, ' ');
        }
        throw std::runtime_error(SocketError("UDP 수신", error));
    }
#else
    socklen_t sourceLength = sizeof(source);
    const ssize_t received = recvfrom(Native(handle_), buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr*>(&source), &sourceLength);
    if (received < 0)
    {
        const int errorCode = LastSocketError();
        if (errorCode == EAGAIN || errorCode == EWOULDBLOCK || errorCode == EINTR)
            return std::nullopt;
        throw std::runtime_error(SocketError("UDP 수신", errorCode));
    }
#endif
    peer.addressNetworkOrder = source.sin_addr.s_addr;
    peer.portNetworkOrder = source.sin_port;
    return std::string(buffer.data(), static_cast<std::size_t>(received));
}

void UdpSocket::Send(std::string_view datagram, const UdpPeer& peer) const
{
    if (datagram.empty() || datagram.size() > maximumDatagramBytes)
        throw std::invalid_argument("UDP 전송 메시지 크기가 허용 범위를 벗어났습니다.");
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_addr.s_addr = peer.addressNetworkOrder;
    destination.sin_port = peer.portNetworkOrder;
#ifdef _WIN32
    const int sent = sendto(Native(handle_), datagram.data(), static_cast<int>(datagram.size()), 0,
        reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
    if (sent == SOCKET_ERROR || static_cast<std::size_t>(sent) != datagram.size())
#else
    const ssize_t sent = sendto(Native(handle_), datagram.data(), datagram.size(), 0,
        reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
    if (sent < 0 || static_cast<std::size_t>(sent) != datagram.size())
#endif
    {
        const int errorCode = LastSocketError();
        throw std::runtime_error(SocketError("UDP 전송", errorCode));
    }
}

std::uint16_t UdpSocket::LocalPort() const
{
    sockaddr_in address{};
#ifdef _WIN32
    int addressLength = sizeof(address);
    if (getsockname(Native(handle_), reinterpret_cast<sockaddr*>(&address), &addressLength) != 0)
#else
    socklen_t addressLength = sizeof(address);
    if (getsockname(Native(handle_), reinterpret_cast<sockaddr*>(&address), &addressLength) != 0)
#endif
    {
        const int errorCode = LastSocketError();
        throw std::runtime_error(SocketError("UDP 로컬 포트 확인", errorCode));
    }
    return ntohs(address.sin_port);
}

UdpPeer UdpSocket::Resolve(std::string_view host, std::uint16_t port)
{
    const std::string hostString(host);
#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
        throw std::runtime_error("Winsock을 초기화하지 못했습니다.");
#endif
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, hostString.c_str(), &address.sin_addr) != 1)
    {
#ifdef _WIN32
        WSACleanup();
#endif
        throw std::invalid_argument("IPv4 주소는 숫자 형식으로 입력해야 합니다.");
    }
#ifdef _WIN32
    WSACleanup();
#endif
    return {address.sin_addr.s_addr, address.sin_port};
}
}
