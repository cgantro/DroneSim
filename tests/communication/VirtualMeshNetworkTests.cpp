#include "communication/VirtualMeshNetwork.h"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace aerosimlab::communication
{
namespace
{
struct NetworkFixture
{
    DroneSocketTransport transport;
    VirtualMeshNetwork network;

    NetworkFixture()
        : network(transport, VirtualMeshSettings{.maximumLinkRangeMeters = 100.0})
    {
    }

    void AddNodes(std::span<const NetworkNodeSnapshot> nodes,
        std::span<const RadioObstacle> obstacles = {})
    {
        // 네트워크에 등록하면 전송 계층도 같은 드론 ID의 소켓을 준비한다.
        network.UpdateNodes(nodes, obstacles);
    }

    void AddTwoNodes()
    {
        const std::array nodes{
        NetworkNodeSnapshot{1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, {30.0, 0.0, 10.0}}};
        AddNodes(nodes);
    }
};
}

TEST(VirtualMeshNetworkTests, OpensOneSocketForEveryDrone)
{
    NetworkFixture fixture;
    fixture.AddTwoNodes();

    const SocketPort firstPort = fixture.transport.PortFor(1U).value();
    const SocketPort secondPort = fixture.transport.PortFor(2U).value();
    EXPECT_NE(firstPort, 0U);
    EXPECT_NE(secondPort, 0U);
    EXPECT_NE(firstPort, secondPort);
}

TEST(VirtualMeshNetworkTests, DeliversMessageOnTheNextNetworkUpdate)
{
    NetworkFixture fixture;
    fixture.AddTwoNodes();

    ASSERT_TRUE(fixture.network.Send(1U, 2U,
        7U, "state", "position"));
    fixture.network.Advance(1U);
    ASSERT_EQ(fixture.network.DeliveredMessages().size(), 1U);
    const auto& message = fixture.network.DeliveredMessages().front().message;
    EXPECT_EQ(message.source, 1U);
    EXPECT_EQ(message.destination, 2U);
    EXPECT_EQ(message.sequence, 7U);
    EXPECT_EQ(message.kind, "state");
}

TEST(VirtualMeshNetworkTests, DoesNotForwardThroughAnIntermediateDrone)
{
    NetworkFixture fixture;
    const std::array nodes{
        NetworkNodeSnapshot{1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, {80.0, 0.0, 10.0}},
        NetworkNodeSnapshot{3U, {160.0, 0.0, 10.0}}};
    fixture.AddNodes(nodes);

    EXPECT_FALSE(fixture.network.Send(1U, 3U,
        1U, "state", "unreachable"));
    EXPECT_TRUE(fixture.network.DeliveredMessages().empty());
}

TEST(VirtualMeshNetworkTests, RadioOutageStopsQueuedAndNewMessages)
{
    NetworkFixture fixture;
    fixture.AddTwoNodes();
    ASSERT_TRUE(fixture.network.Send(1U, 2U,
        1U, "state", "in flight"));
    fixture.network.SetNodeRadioEnabled(2U, false);
    // 차단 전에 보낸 데이터도 도착 시점에 링크가 끊겼으면 버려야 한다.
    fixture.network.Advance(1U);
    EXPECT_TRUE(fixture.network.DeliveredMessages().empty());
    EXPECT_FALSE(fixture.network.Send(1U, 2U,
        2U, "state", "radio off"));
    fixture.network.SetNodeRadioEnabled(2U, true);
    ASSERT_TRUE(fixture.network.Send(1U, 2U,
        3U, "state", "restored"));
    fixture.network.Advance(2U);
    EXPECT_EQ(fixture.network.DeliveredMessages().size(), 1U);
    EXPECT_EQ(fixture.network.DeliveredMessages().front().message.payload, "restored");
}

TEST(VirtualMeshNetworkTests, BuildingBlocksDirectConnection)
{
    NetworkFixture fixture;
    const std::array nodes{
        NetworkNodeSnapshot{1U, {0.0, 0.0, 10.0}},
        NetworkNodeSnapshot{2U, {60.0, 0.0, 10.0}}};
    const std::array obstacles{
        RadioObstacle{{{-10.0, -10.0}, {10.0, -10.0},
            {10.0, 10.0}, {-10.0, 10.0}}, 0.0, 20.0}};
    fixture.AddNodes(nodes, obstacles);
    EXPECT_FALSE(fixture.network.Send(1U, 2U,
        1U, "state", "blocked by building"));
    EXPECT_TRUE(fixture.network.CanExchangeAtPositions(1U, 2U,
        {0.0, 0.0, 30.0}, {60.0, 0.0, 30.0}));
    EXPECT_FALSE(fixture.network.CanExchangeAtPositions(1U, 3U,
        {0.0, 0.0, 30.0}, {60.0, 0.0, 30.0}));
}

TEST(VirtualMeshNetworkTests, RejectsUnregisteredNodes)
{
    NetworkFixture fixture;
    fixture.AddTwoNodes();

    EXPECT_FALSE(fixture.network.Send(1U, 99U,
        1U, "state", "wrong port"));
    EXPECT_EQ(fixture.network.Metrics().droppedMessages, 1U);
}

TEST(VirtualMeshNetworkTests, ReportsDirectConnectionAndLosesItWhenBlocked)
{
    NetworkFixture fixture;
    fixture.AddTwoNodes();
    EXPECT_TRUE(fixture.network.HasDirectLink(1U, 2U));
    fixture.network.SetNodeRadioEnabled(2U, false);
    EXPECT_FALSE(fixture.network.HasDirectLink(1U, 2U));
}
}
