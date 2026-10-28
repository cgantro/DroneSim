#pragma once

#include "communication/UdpSocket.h"

#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace aerosimlab::communication::detail
{
/** @brief 운영체제별 구현이 공통 UDP 소켓 기능을 제공한다. */
class UdpSocketPlatform
{
public:
    virtual ~UdpSocketPlatform() = default;

    [[nodiscard]] virtual std::optional<std::string> Receive(UdpPeer& peer) = 0;
    virtual void Send(std::string_view datagram, const UdpPeer& peer) const = 0;
    [[nodiscard]] virtual std::uint16_t LocalPort() const = 0;
};

/** @brief 빌드에서 선택한 운영체제 소켓을 만들고 IPv4 주소를 해석한다. */
[[nodiscard]] std::unique_ptr<UdpSocketPlatform> CreateUdpSocketPlatform(
    std::uint16_t localPort);
[[nodiscard]] UdpPeer ResolveIpv4(std::string_view host, std::uint16_t port);
[[nodiscard]] std::string FormatIpv4Address(std::uint32_t addressNetworkOrder);
}
