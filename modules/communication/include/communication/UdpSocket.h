#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace aerosimlab::communication
{
/** @brief IPv4 발신자의 주소를 응답 전송에 사용한다. */
struct UdpPeer
{
    std::uint32_t addressNetworkOrder = 0;
    std::uint16_t portNetworkOrder = 0;

    /** @brief IPv4 주소를 영상 전송처럼 별도 연결에 쓸 문자열로 돌려준다. */
    [[nodiscard]] std::string AddressIpv4() const;
};

/**
 * @brief UDP 소켓 하나의 수명과 송수신을 관리한다.
 * @details 소켓은 non-blocking으로 동작한다. 받을 데이터가 없으면 Receive가 빈 값을 돌려주므로
 * 시뮬레이션 반복문이 네트워크를 기다리며 멈추지 않는다.
 */
class UdpSocket
{
public:
    explicit UdpSocket(std::uint16_t localPort);
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    [[nodiscard]] std::optional<std::string> Receive(UdpPeer& peer);
    void Send(std::string_view datagram, const UdpPeer& peer) const;
    [[nodiscard]] std::uint16_t LocalPort() const;
    [[nodiscard]] static UdpPeer Resolve(std::string_view host, std::uint16_t port);

private:
    std::uintptr_t handle_ = static_cast<std::uintptr_t>(-1);
    bool winsockStarted_ = false;
};
}
