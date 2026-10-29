#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <vector>

namespace aerosimlab::communication
{
using NetworkNodeId = std::uint64_t;

/** @brief 한 드론 또는 지상통제소의 현재 통신 상태다. */
struct NetworkNodeSnapshot
{
    NetworkNodeId id = 0;
    glm::dvec3 positionMeters{0.0};
};

/** @brief 무선 신호를 가리는 건물이나 지형의 단순한 부피다. */
struct RadioObstacle
{
    std::vector<glm::dvec2> footprintMeters;
    double bottomMeters = 0.0;
    double topMeters = 0.0;
};

struct VirtualMeshSettings
{
    double maximumLinkRangeMeters = 90.0;
};

/** @brief 직접 연결을 통해 보낼 임무 또는 상태 메시지다. */
struct VirtualMeshMessage
{
    std::uint64_t id = 0;
    NetworkNodeId source = 0;
    NetworkNodeId destination = 0;
    std::uint32_t sequence = 0;
    std::string kind;
    std::string payload;
};

struct DeliveredMeshMessage
{
    VirtualMeshMessage message;
    std::uint64_t deliveredAtTick = 0;
};

struct VirtualMeshMetrics
{
    std::uint64_t deliveredMessages = 0;
    std::uint64_t droppedMessages = 0;
};

enum class MeshEventKind
{
    NodeRadioDisabled,
    NodeRadioRestored,
    MessageDelivered,
    PacketDropped
};

/** @brief 화면과 시험에서 확인할 최근 통신 변화다. */
struct MeshEvent
{
    std::uint64_t tick = 0;
    MeshEventKind kind = MeshEventKind::PacketDropped;
    NetworkNodeId source = 0;
    NetworkNodeId destination = 0;
    std::string detail;
};

/**
 * @brief 거리, 건물 가림, 장애 주입을 반영해 노드 사이 직접 전송만 처리한다.
 * @details 메시지는 중간 노드를 거치지 않는다. 신뢰성이 필요한 명령의 재전송은
 *          통신망이 아닌 임무 계층에서 맡는다.
 */
class VirtualMeshNetwork
{
public:
    explicit VirtualMeshNetwork(VirtualMeshSettings settings = {});

    /** @brief 매 Tick 노드 위치와 전파를 가리는 장애물을 갱신한다. */
    void UpdateNodes(std::span<const NetworkNodeSnapshot> nodes,
        std::span<const RadioObstacle> obstacles);

    /** @brief 한 노드의 송수신을 끊거나 복구한다. */
    void SetNodeRadioEnabled(NetworkNodeId node, bool enabled);
    [[nodiscard]] bool IsNodeRadioEnabled(NetworkNodeId node) const noexcept;

    /** @brief 직접 연결이 가능한 메시지를 보내고 메시지 ID를 돌려준다. */
    [[nodiscard]] std::optional<std::uint64_t> Send(
        NetworkNodeId source, NetworkNodeId destination,
        std::uint32_t sequence, std::string kind, std::string payload);

    /** @brief 모의 시간을 진행하고 도착한 메시지를 처리한다. */
    void Advance(std::uint64_t tick);

    [[nodiscard]] std::span<const DeliveredMeshMessage> DeliveredMessages() const noexcept;
    [[nodiscard]] std::span<const MeshEvent> Events() const noexcept;
    [[nodiscard]] const VirtualMeshMetrics& Metrics() const noexcept;
    [[nodiscard]] bool HasDirectLink(
        NetworkNodeId node, NetworkNodeId neighbor) const noexcept;
    [[nodiscard]] bool CanExchangeAtPositions(
        NetworkNodeId first, NetworkNodeId second,
        const glm::dvec3& firstPositionMeters,
        const glm::dvec3& secondPositionMeters) const;

private:
    struct Transmission
    {
        std::uint64_t dueTick = 0;
        NetworkNodeId sender = 0;
        NetworkNodeId receiver = 0;
        VirtualMeshMessage message;
    };

    [[nodiscard]] bool LinkAvailable(NetworkNodeId sender,
        NetworkNodeId receiver) const;
    [[nodiscard]] bool LinkAvailableAt(NetworkNodeId sender,
        NetworkNodeId receiver, const glm::dvec3& senderPositionMeters,
        const glm::dvec3& receiverPositionMeters) const;
    [[nodiscard]] bool HasLineOfSight(
        const glm::dvec3& start, const glm::dvec3& end) const;
    void Transmit(NetworkNodeId sender, NetworkNodeId receiver,
        VirtualMeshMessage message);
    void RecordEvent(MeshEventKind kind, NetworkNodeId source,
        NetworkNodeId destination, std::string detail);

    VirtualMeshSettings settings_;
    std::map<NetworkNodeId, NetworkNodeSnapshot> nodes_;
    std::vector<RadioObstacle> obstacles_;
    std::set<NetworkNodeId> radioDisabledNodes_;
    std::vector<Transmission> transmissions_;
    std::vector<DeliveredMeshMessage> deliveredMessages_;
    std::vector<MeshEvent> events_;
    VirtualMeshMetrics metrics_;
    std::uint64_t currentTick_ = 0;
    std::uint64_t nextMessageId_ = 1;
};
}
