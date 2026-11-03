#include "communication/DroneSocket.h"

#include <stdexcept>

namespace dronesim::communication
{
void DroneSocket::ValidateDatagram(
    SocketPort destinationPort, std::string_view payload)
{
    if (destinationPort == 0U || payload.empty()
        || payload.size() > maximumDatagramPayloadBytes)
    {
        throw std::invalid_argument(
            "드론 UDP 메시지의 포트나 크기를 확인해 주세요.");
    }
}
}
