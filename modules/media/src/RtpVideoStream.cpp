#include "media/RtpVideoStream.h"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace aerosimlab::media
{
namespace
{
constexpr std::size_t maximumQueuedFrames = 2U;
constexpr int maximumVideoWidth = 4096;
constexpr int maximumVideoHeight = 2160;
constexpr guint encodedBitrateBitsPerSecond = 2'000'000U;

void InitializeGStreamer()
{
    static std::once_flag initialized;
    std::call_once(initialized, []
    {
#if defined(_WIN32) && defined(AEROSIMLAB_GSTREAMER_BIN_DIR)
        const char* currentPath = std::getenv("PATH");
        std::string runtimePath = AEROSIMLAB_GSTREAMER_BIN_DIR;
        if (currentPath && *currentPath)
        {
            runtimePath.push_back(';');
            runtimePath.append(currentPath);
        }
        static_cast<void>(_putenv_s("PATH", runtimePath.c_str()));
#endif
        gst_init(nullptr, nullptr);
#ifdef AEROSIMLAB_GSTREAMER_PLUGIN_DIR
        static_cast<void>(gst_registry_scan_path(gst_registry_get(),
            AEROSIMLAB_GSTREAMER_PLUGIN_DIR));
#endif
    });
}

GstElement* CreateElement(const char* factory, const char* name)
{
    GstElement* element = gst_element_factory_make(factory, name);
    if (!element)
        throw std::runtime_error(std::string("GStreamer 요소를 찾지 못했습니다: ") + factory);
    return element;
}

GstElement* AddElement(GstElement* pipeline, const char* factory, const char* name)
{
    GstElement* element = CreateElement(factory, name);
    if (gst_bin_add(GST_BIN(pipeline), element))
        return element;

    gst_object_unref(element);
    throw std::runtime_error(std::string("GStreamer 파이프라인에 요소를 추가하지 못했습니다: ")
        + name);
}

std::string ReadBusError(GstElement* pipeline)
{
    GstBus* bus = gst_element_get_bus(pipeline);
    if (!bus)
        return {};

    GstMessage* message = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
    gst_object_unref(bus);
    if (!message)
        return {};

    GError* error = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    std::string result = error && error->message
        ? error->message : "GStreamer 파이프라인에서 알 수 없는 오류가 났습니다.";
    if (debug && *debug)
        result += std::string(" (") + debug + ")";
    if (error)
        g_error_free(error);
    g_free(debug);
    gst_message_unref(message);
    return result;
}

void SaveError(std::mutex& mutex, std::string& destination, std::string error)
{
    std::lock_guard lock(mutex);
    if (destination.empty())
        destination = std::move(error);
}

bool CheckAddressAndPorts(const std::string& host,
    std::uint16_t firstPort, std::uint16_t secondPort)
{
    return !host.empty() && firstPort != 0 && secondPort != 0;
}
}

struct RtpVideoSender::Impl
{
    struct QueuedFrame
    {
        sensor::RgbFrame frame;
        std::uint64_t sequence = 0;
    };

    GstElement* pipeline = nullptr;
    GstAppSrc* source = nullptr;
    std::jthread worker;
    mutable std::mutex mutex;
    std::condition_variable queueChanged;
    std::deque<QueuedFrame> queue;
    std::string lastError;
    std::atomic_bool running = false;
    bool acceptingFrames = false;
    std::atomic_bool stopRequested = false;
    int width = 0;
    int height = 0;
    int framesPerSecond = 0;
    std::uint64_t nextSequence = 0;
    std::atomic<std::uint64_t> acceptedFrames = 0;
    std::atomic<std::uint64_t> droppedFrames = 0;
    std::atomic<std::uint64_t> submittedFrames = 0;

    void SetError(std::string error)
    {
        {
            std::lock_guard lock(mutex);
            if (lastError.empty())
                lastError = std::move(error);
            acceptingFrames = false;
            stopRequested = true;
            queue.clear();
        }
        running = false;
        queueChanged.notify_all();
    }

    void EncodeFrames(std::stop_token stopToken)
    {
        try
        {
            for (;;)
            {
                QueuedFrame queued;
                {
                    std::unique_lock lock(mutex);
                    queueChanged.wait(lock, [this, stopToken]
                    {
                        return stopRequested || stopToken.stop_requested() || !queue.empty();
                    });
                    if (queue.empty())
                    {
                        if (stopRequested || stopToken.stop_requested())
                            break;
                        continue;
                    }
                    queued = std::move(queue.front());
                    queue.pop_front();
                }

                const gsize byteCount = queued.frame.pixels.size();
                if (queued.frame.rowOrder == sensor::RgbFrameRowOrder::BottomToTop)
                    queued.frame.FlipVertical();
                GstBuffer* buffer = gst_buffer_new_allocate(nullptr, byteCount, nullptr);
                if (!buffer)
                    throw std::runtime_error("GStreamer 영상 버퍼를 만들지 못했습니다.");
                const gsize copied = gst_buffer_fill(buffer, 0,
                    queued.frame.pixels.data(), byteCount);
                if (copied != byteCount)
                {
                    gst_buffer_unref(buffer);
                    throw std::runtime_error("GStreamer 영상 버퍼에 픽셀을 복사하지 못했습니다.");
                }

                const GstClockTime frameDuration = GST_SECOND
                    / static_cast<GstClockTime>(framesPerSecond);
                GST_BUFFER_PTS(buffer) = queued.sequence * frameDuration;
                GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
                GST_BUFFER_DURATION(buffer) = frameDuration;
                GST_BUFFER_OFFSET(buffer) = queued.frame.simulationTick;
                GST_BUFFER_OFFSET_END(buffer) = queued.frame.simulationTick + 1U;

                const GstFlowReturn result = gst_app_src_push_buffer(source, buffer);
                if (result != GST_FLOW_OK)
                {
                    if (stopRequested)
                        break;
                    const std::string pipelineError = ReadBusError(pipeline);
                    throw std::runtime_error(pipelineError.empty()
                        ? "GStreamer가 영상 프레임을 받지 않았습니다."
                        : pipelineError);
                }
                ++submittedFrames;

                const std::string pipelineError = ReadBusError(pipeline);
                if (!pipelineError.empty())
                    throw std::runtime_error(pipelineError);
            }

            static_cast<void>(gst_app_src_end_of_stream(source));
        }
        catch (const std::exception& error)
        {
            SetError(error.what());
        }
    }

    void ReleasePipeline()
    {
        if (pipeline)
            static_cast<void>(gst_element_set_state(pipeline, GST_STATE_NULL));
        if (source)
        {
            gst_object_unref(source);
            source = nullptr;
        }
        if (pipeline)
        {
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        running = false;
    }
};

RtpVideoSender::RtpVideoSender() : impl_(std::make_unique<Impl>())
{
}

RtpVideoSender::~RtpVideoSender()
{
    Stop();
}

bool RtpVideoSender::Start(int width, int height, int framesPerSecond,
    const std::string& destinationHost, std::uint16_t destinationRtpPort,
    std::uint16_t destinationRtcpPort, std::uint16_t localRtcpPort,
    std::string& error)
{
    Stop();
    if (width <= 0 || width > maximumVideoWidth || width % 2 != 0
        || height <= 0 || height > maximumVideoHeight || height % 2 != 0
        || framesPerSecond <= 0 || framesPerSecond > 120
        || !CheckAddressAndPorts(destinationHost, destinationRtpPort,
            destinationRtcpPort) || localRtcpPort == 0)
    {
        error = "영상 크기, 프레임 속도, IPv4 주소와 RTP·RTCP 포트를 확인해 주세요.";
        return false;
    }

    try
    {
        InitializeGStreamer();
        auto& state = *impl_;
        state.lastError.clear();
        state.acceptedFrames = 0;
        state.droppedFrames = 0;
        state.submittedFrames = 0;
        state.nextSequence = 0;
        state.width = width;
        state.height = height;
        state.framesPerSecond = framesPerSecond;

        state.pipeline = gst_pipeline_new("dronesim-eo-sender");
        if (!state.pipeline)
            throw std::runtime_error("GStreamer 전송 파이프라인을 만들지 못했습니다.");

        GstElement* source = AddElement(state.pipeline, "appsrc", "eo-frames");
        GstElement* converter = AddElement(state.pipeline,
            "videoconvert", "eo-color-converter");
        GstElement* encoder = AddElement(state.pipeline,
            "openh264enc", "eo-h264-encoder");
        GstElement* parser = AddElement(state.pipeline, "h264parse", "eo-h264-parser");
        GstElement* payloader = AddElement(state.pipeline,
            "rtph264pay", "eo-rtp-payloader");
        GstElement* rtpbin = AddElement(state.pipeline, "rtpbin", "eo-rtp-session");
        GstElement* rtpSink = AddElement(state.pipeline, "udpsink", "eo-rtp-output");
        GstElement* rtcpSink = AddElement(state.pipeline, "udpsink", "eo-rtcp-output");
        GstElement* rtcpSource = AddElement(state.pipeline, "udpsrc", "eo-rtcp-input");

        const guint sourceQueueBytes = static_cast<guint>(
            static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height) * 3U
            * maximumQueuedFrames);
        GstCaps* rawCaps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "RGB",
            "width", G_TYPE_INT, width,
            "height", G_TYPE_INT, height,
            "framerate", GST_TYPE_FRACTION, framesPerSecond, 1,
            nullptr);
        if (!rawCaps)
            throw std::runtime_error("GStreamer 영상 형식을 만들지 못했습니다.");
        g_object_set(source,
            "caps", rawCaps,
            "is-live", TRUE,
            "format", GST_FORMAT_TIME,
            "block", TRUE,
            "max-bytes", static_cast<guint64>(sourceQueueBytes),
            nullptr);
        gst_caps_unref(rawCaps);

        g_object_set(encoder,
            "bitrate", encodedBitrateBitsPerSecond,
            "gop-size", static_cast<guint>(framesPerSecond),
            "multi-thread", 2U,
            nullptr);
        g_object_set(payloader, "pt", 96U, "config-interval", 1, nullptr);
        g_object_set(rtpSink,
            "host", destinationHost.c_str(),
            "port", static_cast<gint>(destinationRtpPort),
            "sync", FALSE, "async", FALSE,
            nullptr);
        g_object_set(rtcpSink,
            "host", destinationHost.c_str(),
            "port", static_cast<gint>(destinationRtcpPort),
            "sync", FALSE, "async", FALSE,
            nullptr);
        GstCaps* rtcpCaps = gst_caps_new_empty_simple("application/x-rtcp");
        g_object_set(rtcpSource,
            "port", static_cast<gint>(localRtcpPort),
            "caps", rtcpCaps,
            nullptr);
        gst_caps_unref(rtcpCaps);

        if (!gst_element_link_many(source, converter, encoder, parser, payloader, nullptr))
            throw std::runtime_error("RGB 영상에서 H.264 RTP를 만드는 경로를 연결하지 못했습니다.");

        GstPad* payloadPad = gst_element_get_static_pad(payloader, "src");
        GstPad* rtpInputPad = gst_element_request_pad_simple(rtpbin, "send_rtp_sink_0");
        if (!payloadPad || !rtpInputPad
            || gst_pad_link(payloadPad, rtpInputPad) != GST_PAD_LINK_OK)
        {
            if (payloadPad)
                gst_object_unref(payloadPad);
            if (rtpInputPad)
            {
                gst_element_release_request_pad(rtpbin, rtpInputPad);
                gst_object_unref(rtpInputPad);
            }
            throw std::runtime_error("H.264 영상을 RTP 세션에 연결하지 못했습니다.");
        }
        gst_object_unref(payloadPad);
        gst_object_unref(rtpInputPad);

        if (!gst_element_link_pads(rtpbin, "send_rtp_src_0", rtpSink, "sink")
            || !gst_element_link_pads(rtpbin, "send_rtcp_src_0", rtcpSink, "sink")
            || !gst_element_link_pads(rtcpSource, "src", rtpbin, "recv_rtcp_sink_0"))
            throw std::runtime_error("RTP 영상 또는 RTCP 제어 경로를 연결하지 못했습니다.");

        state.source = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(state.pipeline), "eo-frames"));
        if (!state.source)
            throw std::runtime_error("GStreamer 영상 입력을 찾지 못했습니다.");
        if (gst_element_set_state(state.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error("GStreamer 전송 파이프라인을 시작하지 못했습니다.");

        {
            std::lock_guard lock(state.mutex);
            state.acceptingFrames = true;
            state.stopRequested = false;
        }
        state.running = true;
        state.worker = std::jthread([&state](std::stop_token stopToken)
        {
            state.EncodeFrames(stopToken);
        });
        return true;
    }
    catch (const std::exception& exception)
    {
        error = exception.what();
        impl_->ReleasePipeline();
        return false;
    }
}

bool RtpVideoSender::PushFrame(const sensor::RgbFrame& frame, std::string& error)
{
    auto& state = *impl_;
    if (!state.running || !frame.IsValid()
        || frame.width != state.width || frame.height != state.height)
    {
        error = "전송 중인 EO 영상과 프레임 크기가 다릅니다.";
        return false;
    }

    std::lock_guard lock(state.mutex);
    if (!state.acceptingFrames || state.stopRequested)
    {
        error = state.lastError.empty()
            ? "EO 영상 전송이 종료됐습니다." : state.lastError;
        return false;
    }
    if (state.queue.size() == maximumQueuedFrames)
    {
        state.queue.pop_front();
        ++state.droppedFrames;
    }
    state.queue.push_back({frame, state.nextSequence++});
    ++state.acceptedFrames;
    state.queueChanged.notify_one();
    return true;
}

void RtpVideoSender::Stop()
{
    if (!impl_)
        return;
    auto& state = *impl_;
    {
        std::lock_guard lock(state.mutex);
        state.acceptingFrames = false;
        state.stopRequested = true;
        state.queue.clear();
    }
    state.queueChanged.notify_all();
    if (state.pipeline)
        static_cast<void>(gst_element_set_state(state.pipeline, GST_STATE_NULL));
    if (state.worker.joinable())
    {
        state.worker.request_stop();
        state.worker.join();
    }
    state.ReleasePipeline();
}

bool RtpVideoSender::IsRunning() const noexcept
{
    return impl_ && impl_->running.load();
}

RtpVideoStreamStats RtpVideoSender::Stats() const noexcept
{
    if (!impl_)
        return {};
    return {impl_->acceptedFrames.load(), impl_->droppedFrames.load(),
        impl_->submittedFrames.load(), 0};
}

std::string RtpVideoSender::LastError() const
{
    if (!impl_)
        return {};
    std::lock_guard lock(impl_->mutex);
    return impl_->lastError;
}

struct RtpVideoReceiver::Impl
{
    GstElement* pipeline = nullptr;
    GstElement* rtpbin = nullptr;
    GstAppSink* sink = nullptr;
    GstElement* depayloader = nullptr;
    std::jthread worker;
    mutable std::mutex mutex;
    std::optional<sensor::RgbFrame> latestFrame;
    std::string lastError;
    std::atomic_bool running = false;
    std::atomic_bool stopRequested = false;
    std::atomic<std::uint64_t> decodedFrames = 0;

    static void LinkDecodedPad(GstElement*, GstPad* sourcePad, gpointer userData)
    {
        auto* depayloader = static_cast<GstElement*>(userData);
        GstPad* sinkPad = gst_element_get_static_pad(depayloader, "sink");
        if (!sinkPad)
            return;
        if (!gst_pad_is_linked(sinkPad))
            static_cast<void>(gst_pad_link(sourcePad, sinkPad));
        gst_object_unref(sinkPad);
    }

    void ReceiveFrames(std::stop_token stopToken)
    {
        while (!stopToken.stop_requested() && !stopRequested.load())
        {
            GstSample* sample = gst_app_sink_try_pull_sample(sink, 50 * GST_MSECOND);
            if (!sample)
            {
                const std::string pipelineError = ReadBusError(pipeline);
                if (!pipelineError.empty())
                {
                    SaveError(mutex, lastError, pipelineError);
                    break;
                }
                continue;
            }

            try
            {
                GstCaps* caps = gst_sample_get_caps(sample);
                GstBuffer* buffer = gst_sample_get_buffer(sample);
                GstVideoInfo info{};
                if (!caps || !buffer || !gst_video_info_from_caps(&info, caps)
                    || GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB)
                    throw std::runtime_error("받은 EO 영상의 RGB 형식을 해석하지 못했습니다.");

                const int width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&info));
                const int height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info));
                if (width <= 0 || width > maximumVideoWidth
                    || height <= 0 || height > maximumVideoHeight)
                    throw std::runtime_error("받은 EO 영상 크기가 허용 범위를 벗어났습니다.");

                const std::size_t rowBytes = static_cast<std::size_t>(width) * 3U;
                const std::size_t expectedBytes = rowBytes * static_cast<std::size_t>(height);
                sensor::RgbFrame frame;
                frame.width = width;
                frame.height = height;
                frame.rowOrder = sensor::RgbFrameRowOrder::TopToBottom;
                if (GST_BUFFER_OFFSET_IS_VALID(buffer))
                    frame.simulationTick = GST_BUFFER_OFFSET(buffer);
                frame.pixels.resize(expectedBytes);

                GstVideoFrame mappedFrame{};
                if (!gst_video_frame_map(&mappedFrame, &info, buffer, GST_MAP_READ))
                    throw std::runtime_error("받은 EO 영상 버퍼를 읽지 못했습니다.");
                const auto* pixels = static_cast<const std::uint8_t*>(
                    GST_VIDEO_FRAME_PLANE_DATA(&mappedFrame, 0));
                const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(&mappedFrame, 0);
                if (std::abs(stride) < static_cast<gint>(rowBytes))
                {
                    gst_video_frame_unmap(&mappedFrame);
                    throw std::runtime_error("받은 EO 영상의 행 간격이 픽셀 수보다 작습니다.");
                }
                for (int row = 0; row < height; ++row)
                {
                    const auto* sourceRow = pixels + static_cast<std::ptrdiff_t>(row) * stride;
                    std::memcpy(frame.pixels.data() + static_cast<std::size_t>(row) * rowBytes,
                        sourceRow, rowBytes);
                }
                gst_video_frame_unmap(&mappedFrame);

                {
                    std::lock_guard lock(mutex);
                    latestFrame = std::move(frame);
                }
                ++decodedFrames;
            }
            catch (const std::exception& error)
            {
                SaveError(mutex, lastError, error.what());
            }
            gst_sample_unref(sample);
        }
        running = false;
    }

    void ReleasePipeline()
    {
        if (pipeline)
            static_cast<void>(gst_element_set_state(pipeline, GST_STATE_NULL));
        if (sink)
        {
            gst_object_unref(sink);
            sink = nullptr;
        }
        rtpbin = nullptr;
        if (pipeline)
        {
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        depayloader = nullptr;
        running = false;
    }
};

RtpVideoReceiver::RtpVideoReceiver() : impl_(std::make_unique<Impl>())
{
}

RtpVideoReceiver::~RtpVideoReceiver()
{
    Stop();
}

bool RtpVideoReceiver::Start(const std::string& senderHost,
    std::uint16_t localRtpPort, std::uint16_t localRtcpPort,
    std::uint16_t senderRtcpPort, std::string& error)
{
    Stop();
    if (!CheckAddressAndPorts(senderHost, localRtpPort, localRtcpPort)
        || senderRtcpPort == 0 || localRtpPort == localRtcpPort)
    {
        error = "송신기 IPv4 주소와 서로 다른 RTP·RTCP 수신 포트를 확인해 주세요.";
        return false;
    }

    try
    {
        InitializeGStreamer();
        auto& state = *impl_;
        {
            std::lock_guard lock(state.mutex);
            state.latestFrame.reset();
            state.lastError.clear();
        }
        state.decodedFrames = 0;
        state.stopRequested = false;
        state.pipeline = gst_pipeline_new("dronesim-eo-receiver");
        if (!state.pipeline)
            throw std::runtime_error("GStreamer 수신 파이프라인을 만들지 못했습니다.");

        GstElement* rtpSource = AddElement(state.pipeline, "udpsrc", "eo-rtp-input");
        GstElement* rtcpSource = AddElement(state.pipeline, "udpsrc", "eo-rtcp-input");
        state.rtpbin = AddElement(state.pipeline, "rtpbin", "eo-rtp-session");
        state.depayloader = AddElement(state.pipeline,
            "rtph264depay", "eo-h264-depayloader");
        GstElement* parser = AddElement(state.pipeline, "h264parse", "eo-h264-parser");
        GstElement* decoder = AddElement(state.pipeline, "openh264dec", "eo-h264-decoder");
        GstElement* converter = AddElement(state.pipeline,
            "videoconvert", "eo-rgb-converter");
        GstElement* format = AddElement(state.pipeline, "capsfilter", "eo-rgb-format");
        GstElement* rtcpSink = AddElement(state.pipeline, "udpsink", "eo-rtcp-output");
        GstElement* sink = AddElement(state.pipeline, "appsink", "eo-decoded-frames");

        GstCaps* rtpCaps = gst_caps_new_simple("application/x-rtp",
            "media", G_TYPE_STRING, "video",
            "clock-rate", G_TYPE_INT, 90000,
            "encoding-name", G_TYPE_STRING, "H264",
            "payload", G_TYPE_INT, 96,
            nullptr);
        GstCaps* rtcpCaps = gst_caps_new_empty_simple("application/x-rtcp");
        GstCaps* rawCaps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "RGB", nullptr);
        if (!rtpCaps || !rtcpCaps || !rawCaps)
        {
            if (rtpCaps)
                gst_caps_unref(rtpCaps);
            if (rtcpCaps)
                gst_caps_unref(rtcpCaps);
            if (rawCaps)
                gst_caps_unref(rawCaps);
            throw std::runtime_error("GStreamer 수신 영상 형식을 만들지 못했습니다.");
        }
        g_object_set(rtpSource,
            "port", static_cast<gint>(localRtpPort), "caps", rtpCaps,
            "buffer-size", 1'048'576, nullptr);
        g_object_set(rtcpSource,
            "port", static_cast<gint>(localRtcpPort), "caps", rtcpCaps,
            nullptr);
        g_object_set(state.rtpbin, "latency", 80U, "drop-on-latency", TRUE, nullptr);
        g_object_set(state.depayloader,
            "request-keyframe", TRUE, "wait-for-keyframe", TRUE, nullptr);
        g_object_set(format, "caps", rawCaps, nullptr);
        g_object_set(rtcpSink,
            "host", senderHost.c_str(),
            "port", static_cast<gint>(senderRtcpPort),
            "sync", FALSE, "async", FALSE,
            nullptr);
        g_object_set(sink,
            "emit-signals", FALSE,
            "sync", FALSE,
            "max-buffers", 1U,
            "drop", TRUE,
            "wait-on-eos", FALSE,
            nullptr);
        gst_caps_unref(rtpCaps);
        gst_caps_unref(rtcpCaps);
        gst_caps_unref(rawCaps);

        if (!gst_element_link_pads(rtpSource, "src", state.rtpbin,
                "recv_rtp_sink_0")
            || !gst_element_link_pads(rtcpSource, "src", state.rtpbin,
                "recv_rtcp_sink_0")
            || !gst_element_link_pads(state.rtpbin, "send_rtcp_src_0", rtcpSink,
                "sink")
            || !gst_element_link_many(state.depayloader, parser, decoder,
                converter, format, sink, nullptr))
            throw std::runtime_error("RTP 수신, H.264 디코딩 또는 RTCP 경로를 연결하지 못했습니다.");

        g_signal_connect(state.rtpbin, "pad-added",
            G_CALLBACK(Impl::LinkDecodedPad), state.depayloader);
        state.sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(state.pipeline),
            "eo-decoded-frames"));
        if (!state.sink)
            throw std::runtime_error("GStreamer 디코딩 영상을 찾지 못했습니다.");
        if (gst_element_set_state(state.pipeline, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error("GStreamer 수신 파이프라인을 시작하지 못했습니다.");

        state.running = true;
        state.worker = std::jthread([&state](std::stop_token stopToken)
        {
            state.ReceiveFrames(stopToken);
        });
        return true;
    }
    catch (const std::exception& exception)
    {
        error = exception.what();
        impl_->ReleasePipeline();
        return false;
    }
}

std::optional<sensor::RgbFrame> RtpVideoReceiver::TakeLatestFrame()
{
    if (!impl_)
        return std::nullopt;
    std::lock_guard lock(impl_->mutex);
    std::optional<sensor::RgbFrame> frame = std::move(impl_->latestFrame);
    impl_->latestFrame.reset();
    return frame;
}

void RtpVideoReceiver::Stop()
{
    if (!impl_)
        return;
    auto& state = *impl_;
    state.stopRequested = true;
    if (state.pipeline)
        static_cast<void>(gst_element_set_state(state.pipeline, GST_STATE_NULL));
    if (state.worker.joinable())
    {
        state.worker.request_stop();
        state.worker.join();
    }
    state.ReleasePipeline();
}

bool RtpVideoReceiver::IsRunning() const noexcept
{
    return impl_ && impl_->running.load();
}

RtpVideoStreamStats RtpVideoReceiver::Stats() const noexcept
{
    if (!impl_)
        return {};
    return {0, 0, 0, impl_->decodedFrames.load()};
}

std::string RtpVideoReceiver::LastError() const
{
    if (!impl_)
        return {};
    std::lock_guard lock(impl_->mutex);
    return impl_->lastError;
}
}
