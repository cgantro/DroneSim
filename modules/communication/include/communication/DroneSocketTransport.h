#pragma once

#include "communication/DroneSocket.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace aerosimlab::communication
{
using NetworkNodeId = std::uint64_t;

/** @brief 소켓 전송 계층이 데이터그램에 담아 주고받는 메시지다. */
struct SocketPacket
{
    std::uint64_t id = 0;
    NetworkNodeId source = 0;
    NetworkNodeId destination = 0;
    std::uint32_t sequence = 0;
    std::string kind;
    std::string payload;
};

/** @brief 수신된 패킷과 형식 오류로 버린 데이터그램 수다. */
struct SocketReceiveBatch
{
    std::vector<SocketPacket> packets;
    std::size_t droppedDatagrams = 0;
};

/**
 * @brief 드론별 UDP 소켓을 열고 데이터그램 송수신을 맡는다.
 * @details 실제 거리·장애물·라디오 상태는 판단하지 않는다. 그 판단은
 *          VirtualMeshNetwork가 맡고, 이 클래스는 허용된 데이터를 소켓으로 옮긴다.
 */
class DroneSocketTransport
{
public:
    /** @brief 현재 드론마다 소켓을 만들고 사라진 드론의 소켓을 닫는다. */
    void UpdateNodes(std::span<const NetworkNodeId> nodeIds);

    /** @brief 드론 소켓에 운영체제가 배정한 포트를 반환한다. */
    [[nodiscard]] std::optional<SocketPort> PortFor(NetworkNodeId node) const noexcept;

    /** @brief 메시지를 전송 형식으로 포장해 출발 드론 소켓에서 보낸다. */
    [[nodiscard]] bool Send(const SocketPacket& packet) const;

    /** @brief 모든 드론 소켓을 확인해 데이터그램을 메시지로 풀어낸다. */
    [[nodiscard]] SocketReceiveBatch ReceiveAll();

private:
    std::map<NetworkNodeId, DroneSocket> sockets_;
};
}
