#include "media/RtpVideoStream.h"
#include "media/RecordedCaptureTransfer.h"

#include "communication/UdpSocket.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
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

TEST(RecordedCaptureTransferTests, ReceivesAndVerifiesCaptureFiles)
{
    struct TemporaryTree
    {
        std::filesystem::path path;
        ~TemporaryTree()
        {
            std::error_code ignored;
            std::filesystem::remove_all(path, ignored);
        }
    };

    const auto uniqueSuffix = std::chrono::steady_clock::now()
        .time_since_epoch().count();
    const auto root = std::filesystem::temp_directory_path()
        / ("dronesim-capture-transfer-" + std::to_string(uniqueSuffix));
    const TemporaryTree cleanup{root};
    const auto senderDirectory = root / "sender";
    const auto receiverDirectory = root / "receiver";
    std::filesystem::create_directories(senderDirectory);

    const aerosimlab::media::RecordedCaptureFiles files{
        .video = senderDirectory / "capture.mp4",
        .metadata = senderDirectory / "capture.jsonl",
        .manifest = senderDirectory / "capture.manifest.json"};
    const std::array<std::pair<std::filesystem::path, std::string>, 3> contents{{
        {files.video, std::string(64U * 1024U + 9U, 'v')},
        {files.metadata, "{\"simulation_tick\":42}\n"},
        {files.manifest, "{\"collection_state\":\"SAVED\"}\n"}}};
    for (const auto& [path, content] : contents)
    {
        std::ofstream output(path, std::ios::binary);
        ASSERT_TRUE(output);
        output.write(content.data(), static_cast<std::streamsize>(content.size()));
        ASSERT_TRUE(output);
    }

    aerosimlab::media::RecordedCaptureReceiver receiver;
    std::string error;
    ASSERT_TRUE(receiver.Start(0, receiverDirectory, error)) << error;
    ASSERT_TRUE(aerosimlab::media::SendRecordedCapture("127.0.0.1",
        receiver.LocalPort(), files, error)) << error;

    std::optional<aerosimlab::media::RecordedCaptureTransferResult> received;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline && !received)
    {
        received = receiver.TakeLatestResult();
        if (!received)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(received);
    EXPECT_TRUE(received->verified) << received->message;
    EXPECT_EQ(received->manifestPath.filename(), files.manifest.filename());
    for (const auto& [path, expected] : contents)
    {
        const std::filesystem::path deliveredPath = receiverDirectory / path.filename();
        ASSERT_TRUE(std::filesystem::exists(deliveredPath));
        std::ifstream input(deliveredPath, std::ios::binary);
        const std::string actual((std::istreambuf_iterator<char>(input)),
            std::istreambuf_iterator<char>());
        EXPECT_EQ(actual, expected);
    }
}
}
