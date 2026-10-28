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

/** @brief 한 드론이나 지상통제 노드의 이번 Tick 상태다. */
struct NetworkNodeSnapshot
{
    NetworkNodeId id = 0;
    glm::dvec3 positionMeters{0.0};
    bool powered = true;
    bool canTransmit = true;
    bool canReceive = true;
};

/** @brief 전파를 가리는 건물이나 지형 장애물의 단순한 부피다. */
struct RadioObstacle
{
    std::vector<glm::dvec2> footprintMeters;
    double bottomMeters = 0.0;
    double topMeters = 0.0;
};

struct VirtualMeshSettings
{
    double maximumLinkRangeMeters = 90.0;
    std::uint64_t helloIntervalTicks = 60;
    std::uint64_t neighborTimeoutTicks = 180;
    std::uint64_t linkLatencyTicks = 1;
    std::uint64_t routeLifetimeTicks = 300;
    std::uint32_t maximumHops = 5;
    double packetLossProbability = 0.0;
    std::uint32_t randomSeed = 42;
};

/** @brief 모의 통신망을 통해 보낼 임무·상태 메시지다. */
struct VirtualMeshMessage
{
    std::uint64_t id = 0;
    NetworkNodeId source = 0;
    NetworkNodeId destination = 0;
    std::uint32_t sequence = 0;
    std::string kind;
    std::string payload;
    std::vector<NetworkNodeId> route;
};

/** @brief 목적지에 실제로 도착한 메시지와 모의 도착 Tick이다. */
struct DeliveredMeshMessage
{
    VirtualMeshMessage message;
    std::uint64_t deliveredAtTick = 0;
};

struct VirtualMeshMetrics
{
    std::uint64_t helloMessages = 0;
    std::uint64_t routeRequests = 0;
    std::uint64_t routeReplies = 0;
    std::uint64_t forwardedMessages = 0;
    std::uint64_t deliveredMessages = 0;
    std::uint64_t droppedMessages = 0;
    std::uint64_t routeErrors = 0;
};

enum class MeshEventKind
{
    LinkDisabled,
    LinkRestored,
    NeighborDiscovered,
    RouteSearchStarted,
    RouteFound,
    MessageDelivered,
    PacketDropped,
    RouteError
};

/** @brief 화면과 시험 기록에 보여 줄 최근 통신 사건이다. */
struct MeshEvent
{
    std::uint64_t tick = 0;
    MeshEventKind kind = MeshEventKind::PacketDropped;
    NetworkNodeId source = 0;
    NetworkNodeId destination = 0;
    std::string detail;
};

/**
 * @brief 거리와 장애물을 고려해 이웃을 찾고, AODV 원리를 따른 다중 홉 전달을 모의한다.
 * @details 노드의 실제 위치는 링크 가능성을 계산할 때만 사용합니다. 노드는 HELLO를 받고
 *          이웃을 기록하며, 경로 요청·응답으로 만든 경로를 사용해 메시지를 한 홉씩 전달합니다.
 */
class VirtualMeshNetwork
{
public:
    explicit VirtualMeshNetwork(VirtualMeshSettings settings = {});

    /** @brief 현재 위치와 전파 장애물을 갱신한다. */
    void UpdateNodes(std::span<const NetworkNodeSnapshot> nodes,
        std::span<const RadioObstacle> obstacles);

    /** @brief 지정한 두 노드 사이의 통신을 즉시 끊거나 복구한다. */
    void SetLinkDisabled(NetworkNodeId first, NetworkNodeId second, bool disabled);

    /** @brief 전송할 때 적용할 패킷 손실 확률을 바꾼다. */
    void SetPacketLossProbability(double probability);

    /** @brief 두 노드 사이에 장애가 주입되어 있는지 확인한다. */
    [[nodiscard]] bool IsLinkDisabled(
        NetworkNodeId first, NetworkNodeId second) const noexcept;

    /** @brief 지정한 송수신 노드의 라디오 상태를 바꾼다. */
    void SetNodeRadioEnabled(NetworkNodeId node, bool enabled);

    /** @brief 새 메시지를 보내고, 경로가 없으면 경로 탐색을 시작한다. */
    [[nodiscard]] std::optional<std::uint64_t> Send(
        NetworkNodeId source, NetworkNodeId destination,
        std::uint32_t sequence, std::string kind, std::string payload);

    /** @brief 경로가 없을 때 같은 종류의 오래된 대기 메시지를 새 내용으로 바꿔 보낸다. */
    [[nodiscard]] std::optional<std::uint64_t> SendLatest(
        NetworkNodeId source, NetworkNodeId destination,
        std::uint32_t sequence, std::string kind, std::string payload);

    /** @brief 시뮬레이션 시간을 진행하고 도착한 패킷을 처리한다. */
    void Advance(std::uint64_t tick);

    [[nodiscard]] std::span<const DeliveredMeshMessage> DeliveredMessages() const noexcept;
    [[nodiscard]] std::span<const MeshEvent> Events() const noexcept;
    [[nodiscard]] const VirtualMeshMetrics& Metrics() const noexcept;
    [[nodiscard]] std::optional<NetworkNodeId> NextHop(
        NetworkNodeId source, NetworkNodeId destination) const noexcept;

private:
    struct RouteEntry
    {
        NetworkNodeId nextHop = 0;
        std::uint32_t hopCount = 0;
        std::uint64_t expiresAtTick = 0;
    };

    struct Packet
    {
        enum class Type { Hello, Data, RouteRequest, RouteReply, RouteError };
        Type type = Type::Data;
        VirtualMeshMessage message;
        std::uint64_t requestId = 0;
        NetworkNodeId requestOrigin = 0;
        NetworkNodeId requestDestination = 0;
        NetworkNodeId sender = 0;
        std::uint32_t remainingHops = 0;
        std::uint32_t hopCount = 0;
    };

    struct Transmission
    {
        std::uint64_t dueTick = 0;
        std::uint64_t order = 0;
        NetworkNodeId sender = 0;
        NetworkNodeId receiver = 0;
        Packet packet;
    };

    struct Neighbor
    {
        std::uint64_t lastHeardTick = 0;
    };

    using NodePair = std::pair<NetworkNodeId, NetworkNodeId>;

    [[nodiscard]] bool LinkAvailable(NetworkNodeId sender,
        NetworkNodeId receiver) const;
    [[nodiscard]] bool HasLineOfSight(const glm::dvec3& start,
        const glm::dvec3& end) const;
    void SendHelloMessages();
    void Transmit(NetworkNodeId sender, NetworkNodeId receiver, Packet packet);
    void ProcessTransmission(Transmission transmission);
    void StartRouteDiscovery(NetworkNodeId source, NetworkNodeId destination);
    void ForwardData(NetworkNodeId node, Packet packet);
    void ForwardRouteReply(NetworkNodeId node, Packet packet);
    void InvalidateRoute(NetworkNodeId node, NetworkNodeId destination);
    void ExpireOldState();
    void RecordEvent(MeshEventKind kind, NetworkNodeId source,
        NetworkNodeId destination, std::string detail);
    [[nodiscard]] bool IsKnownNeighbor(NetworkNodeId node,
        NetworkNodeId neighbor) const noexcept;

    VirtualMeshSettings settings_;
    std::map<NetworkNodeId, NetworkNodeSnapshot> nodes_;
    std::vector<RadioObstacle> obstacles_;
    std::map<NetworkNodeId, std::map<NetworkNodeId, Neighbor>> neighbors_;
    std::map<NetworkNodeId, std::map<NetworkNodeId, RouteEntry>> routes_;
    std::map<NetworkNodeId, std::map<std::uint64_t, std::uint64_t>> seenRequests_;
    std::map<NodePair, std::vector<VirtualMeshMessage>> pendingMessages_;
    std::map<NodePair, std::uint64_t> activeRequests_;
    std::map<NodePair, std::uint64_t> activeRequestExpiryTicks_;
    std::set<NodePair> disabledLinks_;
    std::set<NetworkNodeId> radioDisabledNodes_;
    std::vector<Transmission> transmissions_;
    std::vector<DeliveredMeshMessage> deliveredMessages_;
    std::vector<MeshEvent> events_;
    VirtualMeshMetrics metrics_;
    std::uint64_t currentTick_ = 0;
    std::uint64_t nextHelloTick_ = 0;
    std::uint64_t nextMessageId_ = 1;
    std::uint64_t nextRequestId_ = 1;
    std::uint64_t nextTransmissionOrder_ = 0;
    std::uint32_t randomState_ = 0;
};
}
