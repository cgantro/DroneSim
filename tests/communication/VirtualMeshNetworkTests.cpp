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
    settings.linkLatencyTicks = 2U;
    return VirtualMeshNetwork(settings);
}

void AddTwoNodes(VirtualMeshNetwork& network)
{
    const std::array nodes{
        NetworkNodeSnapshot{1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, {30.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});
}
}

TEST(VirtualMeshNetworkTests, DeliversMessageDirectlyAfterConfiguredDelay)
{
    auto network = MakeNetwork();
    AddTwoNodes(network);

    ASSERT_TRUE(network.Send(1U, 2U, 7U, "state", "position"));
    network.Advance(1U);
    EXPECT_TRUE(network.DeliveredMessages().empty());
    network.Advance(2U);
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
        NetworkNodeSnapshot{1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, {80.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3U, {160.0, 0.0, 10.0}}};
    network.UpdateNodes(nodes, {});

    EXPECT_FALSE(network.Send(1U, 3U, 1U, "state", "unreachable"));
    EXPECT_TRUE(network.DeliveredMessages().empty());
}

TEST(VirtualMeshNetworkTests, LinkAndRadioOutagesStopDirectDelivery)
{
    auto network = MakeNetwork();
    AddTwoNodes(network);
    network.SetLinkDisabled(1U, 2U, true);
    EXPECT_FALSE(network.Send(1U, 2U, 1U, "state", "blocked"));
    network.SetLinkDisabled(1U, 2U, false);
    network.SetNodeRadioEnabled(2U, false);
    EXPECT_FALSE(network.Send(1U, 2U, 2U, "state", "radio off"));
    network.SetNodeRadioEnabled(2U, true);
    ASSERT_TRUE(network.Send(1U, 2U, 3U, "state", "restored"));
    network.Advance(2U);
    EXPECT_EQ(network.DeliveredMessages().size(), 1U);
    EXPECT_EQ(network.DeliveredMessages().front().message.payload, "restored");
}

TEST(VirtualMeshNetworkTests, BuildingBlocksDirectConnection)
{
    auto network = MakeNetwork();
    const std::array nodes{
        NetworkNodeSnapshot{1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, {60.0, 0.0, 10.0}}};
    const std::array obstacles{
        RadioObstacle{{{-10.0, -10.0}, {10.0, -10.0},
            {10.0, 10.0}, {-10.0, 10.0}}, 0.0, 20.0}};
    network.UpdateNodes(nodes, obstacles);
    EXPECT_FALSE(network.Send(1U, 2U, 1U, "state", "blocked by building"));
    EXPECT_TRUE(network.CanExchangeAtPositions(1U, 2U,
        {0.0, 0.0, 30.0}, {60.0, 0.0, 30.0}));
}

TEST(VirtualMeshNetworkTests, ReportsDirectConnectionAndLosesItWhenBlocked)
{
    auto network = MakeNetwork();
    AddTwoNodes(network);
    EXPECT_TRUE(network.HasDirectLink(1U, 2U));
    network.SetLinkDisabled(1U, 2U, true);
    EXPECT_FALSE(network.HasDirectLink(1U, 2U));
}
}
