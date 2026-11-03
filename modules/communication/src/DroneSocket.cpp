#include "communication/DroneSocket.h"

#include <stdexcept>

namespace dronesim::communication
{
void DroneSocket::ValidateDatagram(
    SocketPort destinationPort, std::string_view payload)
{
    // 포트 0은 운영체제가 로컬 포트를 고를 때 쓰는 값이라 목적지로 사용할 수 없다.
    // 이 통신 규약은 빈 메시지를 허용하지 않으며 UDP 사용자 자료의 크기 제한도 따른다.
    if (destinationPort == 0U || payload.empty()
        || payload.size() > maximumDatagramPayloadBytes)
    {
        throw std::invalid_argument(
            "드론 UDP 메시지의 포트나 크기를 확인해 주세요.");
    }
}
}
