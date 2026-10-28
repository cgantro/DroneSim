#include "communication/UdpSocket.h"

#include "communication/UdpSocketPlatform.h"

namespace aerosimlab::communication
{
class UdpSocket::Impl
{
public:
    explicit Impl(std::uint16_t localPort)
        : platform(detail::CreateUdpSocketPlatform(localPort))
    {
    }

    std::unique_ptr<detail::UdpSocketPlatform> platform;
};

std::string UdpPeer::AddressIpv4() const
{
    return detail::FormatIpv4Address(addressNetworkOrder);
}

UdpSocket::UdpSocket(std::uint16_t localPort)
    : impl_(std::make_unique<Impl>(localPort))
{
}

UdpSocket::~UdpSocket() = default;

std::optional<std::string> UdpSocket::Receive(UdpPeer& peer)
{
    return impl_->platform->Receive(peer);
}

void UdpSocket::Send(std::string_view datagram, const UdpPeer& peer) const
{
    impl_->platform->Send(datagram, peer);
}

std::uint16_t UdpSocket::LocalPort() const
{
    return impl_->platform->LocalPort();
}

UdpPeer UdpSocket::Resolve(std::string_view host, std::uint16_t port)
{
    return detail::ResolveIpv4(host, port);
}
}
