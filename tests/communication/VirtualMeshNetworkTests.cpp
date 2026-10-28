#include "communication/VirtualMeshNetwork.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

namespace aerosimlab::communication
{
namespace
{
VirtualMeshSettings QuickSettings()
{
    VirtualMeshSettings settings;
    settings.maximumLinkRangeMeters = 75.0;
    settings.helloIntervalTicks = 2;
    settings.neighborTimeoutTicks = 8;
    settings.linkLatencyTicks = 1;
    settings.routeLifetimeTicks = 80;
    settings.maximumHops = 6;
    return settings;
}

std::vector<DeliveredMeshMessage> AdvanceTicks(VirtualMeshNetwork& network,
    std::uint64_t firstTick, std::uint64_t lastTick)
{
    std::vector<DeliveredMeshMessage> delivered;
    for (std::uint64_t tick = firstTick; tick <= lastTick; ++tick)
    {
        network.Advance(tick);
        delivered.insert(delivered.end(), network.DeliveredMessages().begin(),
            network.DeliveredMessages().end());
    }
    return delivered;
}
}

TEST(VirtualMeshNetworkTests, RoutesProbeAndAcknowledgementAcrossMesh)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array nodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3, {120.0, 0.0, 10.0}},
        NetworkNodeSnapshot{4, {180.0, 0.0, 10.0}},
        NetworkNodeSnapshot{std::numeric_limits<NetworkNodeId>::max(),
            {-60.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});
    const NetworkNodeId groundControlId =
        std::numeric_limits<NetworkNodeId>::max();
    // 이웃 발견이 끝난 뒤 시험 메시지를 보내 실제 경로 탐색을 확인합니다.
    static_cast<void>(AdvanceTicks(network, 0, 3));
    const auto messageId = network.Send(groundControlId, 4,
        7, "leader_probe", "epoch=1");
    ASSERT_TRUE(messageId.has_value());
    const auto deliveredMessages = AdvanceTicks(network, 4, 30);

    ASSERT_EQ(deliveredMessages.size(), 1U)
        << "RREQ=" << network.Metrics().routeRequests
        << " RREP=" << network.Metrics().routeReplies
        << " drop=" << network.Metrics().droppedMessages
        << " error=" << network.Metrics().routeErrors
        << " events=" << network.Events().size();
    const auto& delivered = deliveredMessages.front();
    EXPECT_EQ(delivered.message.id, *messageId);
    EXPECT_EQ(delivered.message.route, (std::vector<NetworkNodeId>{
        groundControlId, 1, 2, 3, 4}));
    EXPECT_EQ(delivered.message.payload, "epoch=1");
    EXPECT_GE(network.Metrics().forwardedMessages, 2U);

    ASSERT_TRUE(network.Send(4, groundControlId, 7,
        "leader_probe_ack", delivered.message.payload));
    const auto acknowledgements = AdvanceTicks(network, 31, 50);
    ASSERT_EQ(acknowledgements.size(), 1U);
    EXPECT_EQ(acknowledgements.front().message.kind, "leader_probe_ack");
    EXPECT_EQ(acknowledgements.front().message.route, (std::vector<NetworkNodeId>{
        4, 3, 2, 1, groundControlId}));

    const auto events = network.Events();
    EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const MeshEvent& event)
    {
        return event.kind == MeshEventKind::RouteFound;
    }));
    EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const MeshEvent& event)
    {
        return event.kind == MeshEventKind::MessageDelivered;
    }));
}

TEST(VirtualMeshNetworkTests, FindsAlternateRouteAfterLinkFailure)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array nodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {70.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3, {35.0, 60.0, 10.0}},
        NetworkNodeSnapshot{4, {105.0, 60.0, 10.0}}};
    network.UpdateNodes(nodes, {});
    static_cast<void>(AdvanceTicks(network, 0, 3));

    ASSERT_TRUE(network.Send(1, 4, 1, "waypoint", "first"));
    const auto firstDelivery = AdvanceTicks(network, 4, 20);
    ASSERT_EQ(firstDelivery.size(), 1U);
    const auto firstHop = network.NextHop(1, 4);
    ASSERT_TRUE(firstHop.has_value());

    network.SetLinkDisabled(1, *firstHop, true);
    EXPECT_TRUE(network.IsLinkDisabled(1, *firstHop));
    EXPECT_FALSE(network.NextHop(1, 4).has_value());
    ASSERT_TRUE(network.Send(1, 4, 2, "waypoint", "after-failure"));
    const auto secondDelivery = AdvanceTicks(network, 21, 45);

    ASSERT_EQ(secondDelivery.size(), 1U);
    const auto& rerouted = secondDelivery.front().message;
    EXPECT_EQ(rerouted.payload, "after-failure");
    EXPECT_EQ(rerouted.route.front(), 1U);
    EXPECT_EQ(rerouted.route.back(), 4U);
    EXPECT_EQ(rerouted.route.size(), 3U);
    for (std::size_t index = 1; index < rerouted.route.size(); ++index)
    {
        const NetworkNodeId first = rerouted.route[index - 1U];
        const NetworkNodeId second = rerouted.route[index];
        const NetworkNodeId linkStart = std::min(first, second);
        const NetworkNodeId linkEnd = std::max(first, second);
        const NetworkNodeId failedStart = std::min(NetworkNodeId{1}, *firstHop);
        const NetworkNodeId failedEnd = std::max(NetworkNodeId{1}, *firstHop);
        EXPECT_FALSE(linkStart == failedStart && linkEnd == failedEnd);
    }
    network.SetLinkDisabled(1, *firstHop, false);
    EXPECT_FALSE(network.IsLinkDisabled(1, *firstHop));
    const auto events = network.Events();
    EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const MeshEvent& event)
    {
        return event.kind == MeshEventKind::LinkDisabled;
    }));
    EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const MeshEvent& event)
    {
        return event.kind == MeshEventKind::RouteFound;
    }));
}

TEST(VirtualMeshNetworkTests, BuildingBlocksRadioLink)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array nodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {100.0, 0.0, 10.0}}};
    const std::array obstacles{
        RadioObstacle{{{40.0, -10.0}, {60.0, -10.0},
                          {60.0, 10.0}, {40.0, 10.0}}, 0.0, 20.0}};
    network.UpdateNodes(nodes, obstacles);
    static_cast<void>(AdvanceTicks(network, 0, 20));

    ASSERT_TRUE(network.Send(1, 2, 1, "status", "blocked"));
    static_cast<void>(AdvanceTicks(network, 21, 30));
    EXPECT_TRUE(network.DeliveredMessages().empty());

    network.SetNodeRadioEnabled(1, false);
    EXPECT_FALSE(network.IsNodeRadioEnabled(1));
    EXPECT_TRUE(network.IsNodeRadioEnabled(2));
    network.SetNodeRadioEnabled(1, true);
    EXPECT_TRUE(network.IsNodeRadioEnabled(1));

    VirtualMeshNetwork radioNetwork{QuickSettings()};
    const std::array radioNodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}}};
    radioNetwork.UpdateNodes(radioNodes, {});
    static_cast<void>(AdvanceTicks(radioNetwork, 0, 3));
    ASSERT_TRUE(radioNetwork.Send(1, 2, 1, "status", "before-radio-failure"));
    ASSERT_EQ(AdvanceTicks(radioNetwork, 4, 12).size(), 1U);

    radioNetwork.SetNodeRadioEnabled(1, false);
    ASSERT_TRUE(radioNetwork.Send(1, 2, 2, "status", "radio-disabled"));
    EXPECT_TRUE(AdvanceTicks(radioNetwork, 13, 20).empty());
    EXPECT_FALSE(radioNetwork.IsNodeRadioEnabled(1));
    EXPECT_TRUE(std::any_of(radioNetwork.Events().begin(), radioNetwork.Events().end(),
        [](const MeshEvent& event)
        {
            return event.kind == MeshEventKind::NodeRadioDisabled;
        }));

    radioNetwork.SetNodeRadioEnabled(1, true);
    static_cast<void>(AdvanceTicks(radioNetwork, 21, 25));
    ASSERT_TRUE(radioNetwork.Send(1, 2, 3, "status", "radio-restored"));
    const auto recovered = AdvanceTicks(radioNetwork, 26, 40);
    EXPECT_TRUE(std::any_of(recovered.begin(), recovered.end(), [](const auto& delivery)
    {
        return delivery.message.payload == "radio-restored";
    }));
}

TEST(VirtualMeshNetworkTests, KeepsOnlyNewestStateWhileRouteIsMissing)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array separatedNodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {200.0, 0.0, 10.0}}};
    network.UpdateNodes(separatedNodes, {});

    ASSERT_TRUE(network.SendLatest(1, 2, 1, "drone_telemetry", "old"));
    ASSERT_TRUE(network.SendLatest(1, 2, 2, "drone_telemetry", "new"));
    ASSERT_TRUE(network.SendLatest(1, 2, 3, "drone_telemetry", "newest"));

    const std::array connectedNodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}}};
    network.UpdateNodes(connectedNodes, {});
    const auto delivered = AdvanceTicks(network, 0, 20);

    ASSERT_EQ(delivered.size(), 1U);
    EXPECT_EQ(delivered.front().message.sequence, 3U);
    EXPECT_EQ(delivered.front().message.payload, "newest");
}

TEST(VirtualMeshNetworkTests, RuntimePacketLossDropsMessages)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array nodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});
    static_cast<void>(AdvanceTicks(network, 0, 3));
    ASSERT_TRUE(network.Send(1, 2, 1, "status", "route-setup"));
    ASSERT_EQ(AdvanceTicks(network, 4, 12).size(), 1U);

    network.SetPacketLossProbability(1.0);
    ASSERT_TRUE(network.Send(1, 2, 2, "fleet_route", "route"));
    static_cast<void>(AdvanceTicks(network, 13, 20));

    EXPECT_TRUE(network.DeliveredMessages().empty());
    EXPECT_GT(network.Metrics().droppedMessages, 0U);
    const auto events = network.Events();
    EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const MeshEvent& event)
    {
        return event.kind == MeshEventKind::PacketDropped
            && event.detail == "설정된 패킷 손실로 버렸습니다.";
    }));
}

TEST(VirtualMeshNetworkTests, RetriesRouteUntilDestinationAcknowledges)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array nodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});
    static_cast<void>(AdvanceTicks(network, 0, 5));
    network.SetPacketLossProbability(1.0);

    constexpr std::uint32_t routeSequence = 7U;
    constexpr std::uint64_t retryIntervalTicks = 60U;
    ASSERT_TRUE(network.Send(1, 2, routeSequence, "fleet_route", "route-data"));
    network.Advance(6U);
    EXPECT_TRUE(network.DeliveredMessages().empty());
    EXPECT_GT(network.Metrics().droppedMessages, 0U);
    network.SetPacketLossProbability(0.0);

    std::uint64_t lastTransmissionTick = 5U;
    std::uint32_t transmissions = 1U;
    bool acknowledged = false;
    std::uint32_t receivedRouteCount = 0U;

    for (std::uint64_t tick = 7U; tick <= 100U && !acknowledged; ++tick)
    {
        network.Advance(tick);
        for (const DeliveredMeshMessage& delivered : network.DeliveredMessages())
        {
            if (delivered.message.kind == "fleet_route"
                && delivered.message.sequence == routeSequence)
            {
                ++receivedRouteCount;
                static_cast<void>(network.Send(delivered.message.destination,
                    delivered.message.source, routeSequence, "fleet_route_ack", {}));
            }
            else if (delivered.message.kind == "fleet_route_ack"
                && delivered.message.sequence == routeSequence)
            {
                acknowledged = true;
            }
        }

        if (!acknowledged && transmissions < 10U
            && tick >= lastTransmissionTick + retryIntervalTicks)
        {
            ASSERT_TRUE(network.Send(1, 2, routeSequence,
                "fleet_route", "route-data"));
            lastTransmissionTick = tick;
            ++transmissions;
        }
    }

    EXPECT_TRUE(acknowledged);
    EXPECT_GE(receivedRouteCount, 1U);
    EXPECT_GT(transmissions, 1U);
}
}
