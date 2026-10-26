#pragma once

#include "sensor/RgbFrame.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace aerosimlab::media
{
/** @brief 한 대의 시뮬레이터와 지상통제소가 EO 영상을 주고받을 포트다. */
struct EoVideoStreamPorts final
{
    static constexpr std::uint16_t receiverRtp = 5600;
    static constexpr std::uint16_t receiverRtcp = 5601;
    static constexpr std::uint16_t senderRtcp = 5602;
};

/** @brief GStreamer가 처리한 RTP/RTCP 영상 전송 통계다. */
struct RtpVideoStreamStats
{
    std::uint64_t framesAccepted = 0;
    std::uint64_t framesDropped = 0;
    std::uint64_t framesSubmitted = 0;
    std::uint64_t framesDecoded = 0;
};

/**
 * @brief RGB EO 프레임을 H.264 RTP로 보내고, RTCP 수신 보고서를 받는다.
 * @details PushFrame은 픽셀을 제한된 대기열에 복사한 뒤 바로 반환한다. 인코딩과 UDP 전송은
 * 별도 작업자에서 진행하므로 렌더링 반복문이 코덱이나 네트워크를 기다리지 않는다.
 */
class RtpVideoSender final
{
public:
    RtpVideoSender();
    ~RtpVideoSender();

    RtpVideoSender(const RtpVideoSender&) = delete;
    RtpVideoSender& operator=(const RtpVideoSender&) = delete;

    /** @brief H.264 인코더와 RTP/RTCP 전송 경로를 연다. */
    [[nodiscard]] bool Start(int width, int height, int framesPerSecond,
        const std::string& destinationHost, std::uint16_t destinationRtpPort,
        std::uint16_t destinationRtcpPort, std::uint16_t localRtcpPort,
        std::string& error);

    /** @brief 영상 프레임을 최신 프레임 우선 대기열에 복사한다. */
    [[nodiscard]] bool PushFrame(const sensor::RgbFrame& frame,
        std::string& error);

    /** @brief 전송을 중단하고 대기 프레임을 비운 뒤 파이프라인을 닫는다. */
    void Stop();

    [[nodiscard]] bool IsRunning() const noexcept;
    [[nodiscard]] RtpVideoStreamStats Stats() const noexcept;
    [[nodiscard]] std::string LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * @brief RTP로 받은 H.264 영상을 디코딩해 최신 RGB 프레임을 보관한다.
 * @details 수신 작업자는 오래된 디코딩 프레임을 덮어쓴다. 화면 갱신이 늦어도 수신 대기열이
 * 계속 쌓이지 않으며, 호출자는 TakeLatestFrame으로 프레임 소유권을 가져간다.
 */
class RtpVideoReceiver final
{
public:
    RtpVideoReceiver();
    ~RtpVideoReceiver();

    RtpVideoReceiver(const RtpVideoReceiver&) = delete;
    RtpVideoReceiver& operator=(const RtpVideoReceiver&) = delete;

    /** @brief 지정한 RTP·RTCP 포트에서 영상을 받고 RTCP 보고서를 돌려보낸다. */
    [[nodiscard]] bool Start(const std::string& senderHost,
        std::uint16_t localRtpPort, std::uint16_t localRtcpPort,
        std::uint16_t senderRtcpPort, std::string& error);

    /** @brief 아직 화면에 전달하지 않은 최신 프레임의 소유권을 가져간다. */
    [[nodiscard]] std::optional<sensor::RgbFrame> TakeLatestFrame();

    /** @brief 네트워크 수신과 디코딩 작업을 끝낸다. */
    void Stop();

    [[nodiscard]] bool IsRunning() const noexcept;
    [[nodiscard]] RtpVideoStreamStats Stats() const noexcept;
    [[nodiscard]] std::string LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
