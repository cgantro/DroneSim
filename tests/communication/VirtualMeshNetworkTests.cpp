#include "communication/VirtualMeshNetwork.h"

#include <gtest/gtest.h>

#include <array>
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

TEST(VirtualMeshNetworkTests, RoutesMessageAcrossFourNodes)
{
    VirtualMeshNetwork network{QuickSettings()};
    const std::array nodes{
        NetworkNodeSnapshot{1, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2, {60.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3, {120.0, 0.0, 10.0}},
        NetworkNodeSnapshot{4, {180.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});
    const auto messageId = network.Send(1, 4, 7, "waypoint", "x=180");
    ASSERT_TRUE(messageId.has_value());
    const auto deliveredMessages = AdvanceTicks(network, 0, 20);

    ASSERT_EQ(deliveredMessages.size(), 1U);
    const auto& delivered = deliveredMessages.front();
    EXPECT_EQ(delivered.message.id, *messageId);
    EXPECT_EQ(delivered.message.route, (std::vector<NetworkNodeId>{1, 2, 3, 4}));
    EXPECT_EQ(delivered.message.payload, "x=180");
    EXPECT_GE(network.Metrics().forwardedMessages, 2U);
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
}
}
