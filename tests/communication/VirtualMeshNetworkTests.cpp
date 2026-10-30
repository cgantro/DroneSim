#include "communication/VirtualMeshNetwork.h"

#include <gtest/gtest.h>

#include <array>

namespace aerosimlab::communication
{
namespace
{
VirtualMeshNetwork MakeNetwork()
{
    VirtualMeshSettings settings;
    settings.maximumLinkRangeMeters = 100.0;
    return VirtualMeshNetwork(settings);
}

void AddTwoNodes(VirtualMeshNetwork& network)
{
    const std::array nodes{
        NetworkNodeSnapshot{1U, 1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, 2U, {30.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});
}

VirtualEndpoint Endpoint(NetworkNodeId node)
{
    return {node, static_cast<VirtualPort>(node)};
}
}

TEST(VirtualMeshNetworkTests, DeliversMessageOnTheNextNetworkUpdate)
{
    auto network = MakeNetwork();
    AddTwoNodes(network);

    ASSERT_TRUE(network.Send(Endpoint(1U), Endpoint(2U), 7U, "state", "position"));
    network.Advance(1U);
    ASSERT_EQ(network.DeliveredMessages().size(), 1U);
    const auto& message = network.DeliveredMessages().front().message;
    EXPECT_EQ(message.source, 1U);
    EXPECT_EQ(message.destination, 2U);
    EXPECT_EQ(message.sequence, 7U);
    EXPECT_EQ(message.kind, "state");
}

TEST(VirtualMeshNetworkTests, DoesNotForwardThroughAnIntermediateDrone)
{
    auto network = MakeNetwork();
    const std::array nodes{
        NetworkNodeSnapshot{1U, 1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, 2U, {80.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3U, 3U, {160.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});

    EXPECT_FALSE(network.Send(Endpoint(1U), Endpoint(3U), 1U, "state", "unreachable"));
    EXPECT_TRUE(network.DeliveredMessages().empty());
}

TEST(VirtualMeshNetworkTests, RadioOutageStopsQueuedAndNewMessages)
{
    auto network = MakeNetwork();
    AddTwoNodes(network);
    ASSERT_TRUE(network.Send(Endpoint(1U), Endpoint(2U), 1U, "state", "in flight"));
    network.SetNodeRadioEnabled(2U, false);
    network.Advance(1U);
    EXPECT_TRUE(network.DeliveredMessages().empty());
    EXPECT_FALSE(network.Send(Endpoint(1U), Endpoint(2U), 2U, "state", "radio off"));
    network.SetNodeRadioEnabled(2U, true);
    ASSERT_TRUE(network.Send(Endpoint(1U), Endpoint(2U), 3U, "state", "restored"));
    network.Advance(2U);
    EXPECT_EQ(network.DeliveredMessages().size(), 1U);
    EXPECT_EQ(network.DeliveredMessages().front().message.payload, "restored");
}

TEST(VirtualMeshNetworkTests, BuildingBlocksDirectConnection)
{
    auto network = MakeNetwork();
    const std::array nodes{
        NetworkNodeSnapshot{1U, 1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, 2U, {60.0, 0.0, 10.0}}};
    const std::array obstacles{
        RadioObstacle{{{-10.0, -10.0}, {10.0, -10.0},
            {10.0, 10.0}, {-10.0, 10.0}}, 0.0, 20.0}};
    network.UpdateNodes(nodes, obstacles);
    EXPECT_FALSE(network.Send(Endpoint(1U), Endpoint(2U), 1U, "state", "blocked by building"));
    EXPECT_TRUE(network.CanExchangeAtPositions(1U, 2U,
        {0.0, 0.0, 30.0}, {60.0, 0.0, 30.0}));
    EXPECT_FALSE(network.CanExchangeAtPositions(1U, 3U,
        {0.0, 0.0, 30.0}, {60.0, 0.0, 30.0}));
}

TEST(VirtualMeshNetworkTests, RejectsUnknownVirtualPorts)
{
    auto network = MakeNetwork();
    AddTwoNodes(network);

    EXPECT_FALSE(network.Send({1U, 99U}, Endpoint(2U), 1U, "state", "wrong port"));
    EXPECT_FALSE(network.Send(Endpoint(1U), {2U, 99U}, 2U, "state", "wrong port"));
    EXPECT_EQ(network.Metrics().droppedMessages, 2U);
}

TEST(VirtualMeshNetworkTests, ReportsDirectConnectionAndLosesItWhenBlocked)
{
    auto network = MakeNetwork();
    AddTwoNodes(network);
    EXPECT_TRUE(network.HasDirectLink(1U, 2U));
    network.SetNodeRadioEnabled(2U, false);
    EXPECT_FALSE(network.HasDirectLink(1U, 2U));
}
}
