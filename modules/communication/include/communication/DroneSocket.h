#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace aerosimlab::communication
{
using SocketPort = std::uint16_t;

/** @brief 한 드론의 데이터그램 소켓에서 받은 메시지다. */
struct ReceivedDatagram
{
    SocketPort sourcePort = 0;
    std::string payload;
};

/**
 * @brief 드론 하나가 소유하는 데이터그램 소켓이다.
 * @details 네이티브에서는 127.0.0.1에 UDP 소켓을 열고, 브라우저에서는 같은
 *          인터페이스를 쓰는 메모리 전송기를 사용한다.
 */
class DroneSocket
{
public:
    DroneSocket();
    ~DroneSocket();

    DroneSocket(const DroneSocket&) = delete;
    DroneSocket& operator=(const DroneSocket&) = delete;
    DroneSocket(DroneSocket&& other) noexcept;
    DroneSocket& operator=(DroneSocket&& other) noexcept;

    /** @brief 운영체제가 이 드론 소켓에 배정한 로컬 포트를 반환한다. */
    [[nodiscard]] SocketPort LocalPort() const noexcept;

    /** @brief 지정한 드론 포트로 데이터그램을 보낸다. */
    void SendTo(SocketPort destinationPort, std::string_view payload) const;

    /** @brief 대기 중인 데이터그램 하나를 읽는다. 데이터가 없으면 빈 값을 돌려준다. */
    [[nodiscard]] std::optional<ReceivedDatagram> Receive();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}
