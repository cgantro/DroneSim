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

TEST(VirtualMeshNetworkTests, AppliesSeededLatencyJitterAndRepeatsIt)
{
    const auto collectLatencies = []
    {
        auto settings = QuickSettings();
        settings.maximumLinkRangeMeters = 100.0;
        settings.linkLatencyTicks = 2U;
        settings.linkLatencyJitterTicks = 4U;
        settings.randomSeed = 17U;

        VirtualMeshNetwork network{settings};
        const std::array nodes{
            NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
            NetworkNodeSnapshot{2, {50.0, 0.0, 10.0}}};
        network.UpdateNodes(nodes, {});
        static_cast<void>(AdvanceTicks(network, 0, 15));
        EXPECT_TRUE(network.Send(1, 2, 0U, "status", "prime"));
        const auto primingDelivery = AdvanceTicks(network, 16U, 40U);
        EXPECT_EQ(primingDelivery.size(), 1U);
        if (primingDelivery.size() != 1U)
            return std::vector<std::uint64_t>{};

        std::vector<std::uint64_t> latencies;
        std::uint64_t currentTick = 40U;
        for (std::uint32_t sequence = 1U; sequence <= 16U; ++sequence)
        {
            network.Advance(currentTick);
            const std::uint64_t sentAtTick = currentTick;
            EXPECT_TRUE(network.Send(1, 2, sequence, "status", "sample"));

            bool delivered = false;
            for (std::uint64_t tick = sentAtTick + 1U;
                 tick <= sentAtTick + 16U; ++tick)
            {
                network.Advance(tick);
                if (!network.DeliveredMessages().empty())
                {
                    EXPECT_EQ(network.DeliveredMessages().size(), 1U);
                    if (network.DeliveredMessages().size() != 1U)
                        return std::vector<std::uint64_t>{};
                    latencies.push_back(
                        network.DeliveredMessages().front().deliveredAtTick - sentAtTick);
                    currentTick = tick;
                    delivered = true;
                    break;
                }
            }
            EXPECT_TRUE(delivered) << "메시지가 제한 시간 안에 도착하지 않았습니다.";
            if (!delivered)
                return std::vector<std::uint64_t>{};
        }
        return latencies;
    };

    const auto firstRun = collectLatencies();
    const auto repeatedRun = collectLatencies();
    ASSERT_EQ(firstRun.size(), 16U);
    EXPECT_EQ(firstRun, repeatedRun);
    EXPECT_TRUE(std::all_of(firstRun.begin(), firstRun.end(),
        [](std::uint64_t latency)
        {
            return latency >= 2U && latency <= 6U;
        })) << ::testing::PrintToString(firstRun);
    EXPECT_LT(*std::min_element(firstRun.begin(), firstRun.end()),
        *std::max_element(firstRun.begin(), firstRun.end()));
}

TEST(VirtualMeshNetworkTests, ComparesDirectLeaderAndAdaptiveMeshRouting)
{
    const std::array lineNodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3, {120.0, 0.0, 10.0}}};

    auto directSettings = QuickSettings();
    directSettings.mode = VirtualMeshMode::Direct;
    VirtualMeshNetwork directNetwork{directSettings};
    directNetwork.UpdateNodes(lineNodes, {});
    static_cast<void>(AdvanceTicks(directNetwork, 0, 3));
    ASSERT_TRUE(directNetwork.Send(1, 3, 1, "status", "direct-only"));
    EXPECT_TRUE(AdvanceTicks(directNetwork, 4, 24).empty());

    auto leaderSettings = QuickSettings();
    leaderSettings.mode = VirtualMeshMode::LeaderCentric;
    leaderSettings.leaderNodeId = 1;
    VirtualMeshNetwork leaderNetwork{leaderSettings};
    const std::array leaderCentricNodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {-60.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3, {60.0, 0.0, 10.0}}};
    leaderNetwork.UpdateNodes(leaderCentricNodes, {});
    static_cast<void>(AdvanceTicks(leaderNetwork, 0, 3));
    ASSERT_TRUE(leaderNetwork.Send(2, 3, 1, "status", "through-leader"));
    const auto leaderDelivery = AdvanceTicks(leaderNetwork, 4, 24);
    ASSERT_EQ(leaderDelivery.size(), 1U);
    EXPECT_EQ(leaderDelivery.front().message.route,
        (std::vector<NetworkNodeId>{2, 1, 3}));

    const std::array leaderChangeNodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {30.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3, {0.0, 30.0, 10.0}},
        NetworkNodeSnapshot{4, {30.0, 30.0, 10.0}}};
    VirtualMeshNetwork leaderChangeNetwork{leaderSettings};
    leaderChangeNetwork.UpdateNodes(leaderChangeNodes, {});
    static_cast<void>(AdvanceTicks(leaderChangeNetwork, 0, 3));
    ASSERT_TRUE(leaderChangeNetwork.Send(2, 3, 1,
        "status", "before-leader-change"));
    ASSERT_EQ(AdvanceTicks(leaderChangeNetwork, 4, 18).size(), 1U);
    leaderChangeNetwork.SetLeaderNodeId(2);
    ASSERT_TRUE(leaderChangeNetwork.Send(3, 4, 2,
        "status", "after-leader-change"));
    const auto afterLeaderChange = AdvanceTicks(leaderChangeNetwork, 19, 35);
    ASSERT_EQ(afterLeaderChange.size(), 1U);
    EXPECT_EQ(afterLeaderChange.front().message.route,
        (std::vector<NetworkNodeId>{3, 2, 4}));

    const std::array alternateRouteNodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {50.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3, {100.0, 0.0, 10.0}},
        NetworkNodeSnapshot{4, {50.0, 50.0, 10.0}}};
    auto checkLinkRecovery = [&alternateRouteNodes](VirtualMeshMode mode,
                                 bool shouldRecover)
    {
        auto settings = QuickSettings();
        settings.mode = mode;
        VirtualMeshNetwork network{settings};
        network.UpdateNodes(alternateRouteNodes, {});
        static_cast<void>(AdvanceTicks(network, 0, 3));
        ASSERT_TRUE(network.Send(1, 3, 1, "status", "before-failure"));
        ASSERT_EQ(AdvanceTicks(network, 4, 24).size(), 1U);
        const auto firstHop = network.NextHop(1, 3);
        ASSERT_TRUE(firstHop.has_value());

        network.SetLinkDisabled(1, *firstHop, true);
        const auto messageId = network.Send(1, 3, 2, "status", "after-failure");
        EXPECT_EQ(messageId.has_value(), shouldRecover);
        if (!shouldRecover)
        {
            network.SetLinkDisabled(1, *firstHop, false);
            static_cast<void>(AdvanceTicks(network, 25, 30));
            ASSERT_TRUE(network.Send(1, 3, 3, "status", "after-restoration"));
            const auto restored = AdvanceTicks(network, 31, 50);
            ASSERT_EQ(restored.size(), 1U);
            EXPECT_EQ(restored.front().message.payload, "after-restoration");
            return;
        }

        const auto recovered = AdvanceTicks(network, 25, 60);
        ASSERT_EQ(recovered.size(), 1U);
        EXPECT_EQ(recovered.front().message.payload, "after-failure");
        EXPECT_NE(recovered.front().message.route[1], *firstHop);
    };

    checkLinkRecovery(VirtualMeshMode::MultiHop, false);
    checkLinkRecovery(VirtualMeshMode::AdaptiveMesh, true);
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

TEST(VirtualMeshNetworkTests, BlocksOnlyTheSelectedDirection)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array nodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});
    static_cast<void>(AdvanceTicks(network, 0, 3));

    ASSERT_TRUE(network.Send(1, 2, 1, "status", "prime-forward"));
    ASSERT_TRUE(network.Send(2, 1, 1, "status", "prime-reverse"));
    ASSERT_EQ(AdvanceTicks(network, 4, 20).size(), 2U);

    network.SetDirectedLinkDisabled(1, 2, true);
    EXPECT_TRUE(network.IsDirectedLinkDisabled(1, 2));
    EXPECT_FALSE(network.IsDirectedLinkDisabled(2, 1));

    ASSERT_TRUE(network.Send(1, 2, 2, "status", "blocked-forward"));
    ASSERT_TRUE(network.Send(2, 1, 2, "status", "open-reverse"));
    const auto delivered = AdvanceTicks(network, 21, 38);
    ASSERT_EQ(delivered.size(), 1U);
    EXPECT_EQ(delivered.front().message.source, 2U);
    EXPECT_EQ(delivered.front().message.destination, 1U);
    EXPECT_EQ(delivered.front().message.payload, "open-reverse");

    network.SetDirectedLinkDisabled(1, 2, false);
    EXPECT_FALSE(network.IsDirectedLinkDisabled(1, 2));
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
