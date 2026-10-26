#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aerosimlab::media
{
inline constexpr std::uint16_t defaultCaptureTransferPort = 5603;

/** @brief 녹화 영상·촬영 정보·저장 목록을 한 묶음으로 보관한다. */
struct RecordedCaptureFiles
{
    std::filesystem::path video;
    std::filesystem::path metadata;
    std::filesystem::path manifest;
};

/** @brief 지상통제소가 받은 자료 묶음과 무결성 확인 결과다. */
struct RecordedCaptureTransferResult
{
    bool verified = false;
    std::filesystem::path manifestPath;
    std::string message;
};

/** @brief 녹화 파일 세 개를 TCP로 보내고 수신 측 확인 응답을 기다린다. */
[[nodiscard]] bool SendRecordedCapture(
    const std::string& destinationIpv4, std::uint16_t destinationPort,
    const RecordedCaptureFiles& files, std::string& error);

/** @brief 녹화 자료를 받아 체크섬을 확인한 뒤 지상통제소 폴더에 저장한다. */
class RecordedCaptureReceiver final
{
public:
    RecordedCaptureReceiver();
    ~RecordedCaptureReceiver();

    RecordedCaptureReceiver(const RecordedCaptureReceiver&) = delete;
    RecordedCaptureReceiver& operator=(const RecordedCaptureReceiver&) = delete;

    /** @brief 지정한 포트에서 수신을 시작한다. 0번 포트는 시험용 임시 포트를 고른다. */
    [[nodiscard]] bool Start(std::uint16_t port,
        const std::filesystem::path& destinationDirectory, std::string& error);

    /** @brief 수신을 멈추고 진행 중인 연결을 닫는다. */
    void Stop();

    [[nodiscard]] bool IsRunning() const noexcept;
    [[nodiscard]] std::uint16_t LocalPort() const noexcept;

    /** @brief 새로 받은 자료 결과를 가져온다. 결과가 없으면 빈 값을 돌려준다. */
    [[nodiscard]] std::optional<RecordedCaptureTransferResult> TakeLatestResult();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/** @brief 전송 중 바뀐 파일을 수신 측에서 걸러내도록 CRC-32/ISO-HDLC를 계산한다. */
[[nodiscard]] bool CalculateFileCrc32(const std::filesystem::path& path,
    std::uint32_t& checksum, std::string& error);
}
