#include "media/RtpVideoStream.h"

#include "communication/UdpSocket.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace
{
TEST(RtpVideoStreamTests, SendsH264FrameAndReceivesDecodedRgb)
{
    std::array<std::unique_ptr<aerosimlab::communication::UdpSocket>, 3> reservations;
    std::array<std::uint16_t, 3> ports{};
    for (std::size_t index = 0; index < reservations.size(); ++index)
    {
        reservations[index] = std::make_unique<aerosimlab::communication::UdpSocket>(0);
        ports[index] = reservations[index]->LocalPort();
    }
    ASSERT_NE(ports[0], ports[1]);
    ASSERT_NE(ports[0], ports[2]);
    ASSERT_NE(ports[1], ports[2]);
    reservations = {};

    aerosimlab::media::RtpVideoReceiver receiver;
    std::string error;
    ASSERT_TRUE(receiver.Start("127.0.0.1", ports[0], ports[1], ports[2], error)) << error;

    aerosimlab::media::RtpVideoSender sender;
    ASSERT_TRUE(sender.Start(320, 180, 30, "127.0.0.1", ports[0], ports[1],
        ports[2], error)) << error;

    aerosimlab::sensor::RgbFrame source;
    source.width = 320;
    source.height = 180;
    source.simulationTick = 42;
    source.rowOrder = aerosimlab::sensor::RgbFrameRowOrder::BottomToTop;
    source.pixels.resize(static_cast<std::size_t>(source.width * source.height * 3));
    for (int row = 0; row < source.height; ++row)
    {
        const bool sourceRowIsBottomHalf = row < source.height / 2;
        for (int column = 0; column < source.width; ++column)
        {
            const auto pixel = (static_cast<std::size_t>(row) * source.width + column) * 3U;
            source.pixels[pixel] = sourceRowIsBottomHalf ? 30 : 220;
            source.pixels[pixel + 1] = 30;
            source.pixels[pixel + 2] = sourceRowIsBottomHalf ? 220 : 30;
        }
    }

    ASSERT_TRUE(sender.PushFrame(source, error)) << error;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    std::optional<aerosimlab::sensor::RgbFrame> decoded;
    while (std::chrono::steady_clock::now() < deadline && !decoded)
    {
        decoded = receiver.TakeLatestFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_TRUE(decoded.has_value());
    EXPECT_TRUE(decoded->IsValid());
    EXPECT_EQ(decoded->width, source.width);
    EXPECT_EQ(decoded->height, source.height);
    EXPECT_EQ(decoded->rowOrder, aerosimlab::sensor::RgbFrameRowOrder::TopToBottom);
    const auto samplePixel = [&decoded](int row)
    {
        const auto pixel = (static_cast<std::size_t>(row) * decoded->width
            + decoded->width / 2) * 3U;
        return std::array{decoded->pixels[pixel], decoded->pixels[pixel + 1],
            decoded->pixels[pixel + 2]};
    };
    const auto topHalf = samplePixel(decoded->height / 4);
    const auto bottomHalf = samplePixel(decoded->height * 3 / 4);
    EXPECT_GT(topHalf[0], topHalf[2]);
    EXPECT_GT(bottomHalf[2], bottomHalf[0]);
    EXPECT_GT(sender.Stats().framesSubmitted, 0U);
    EXPECT_GT(receiver.Stats().framesDecoded, 0U);
}

}
