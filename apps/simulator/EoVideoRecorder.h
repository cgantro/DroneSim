#pragma once

#include "sensor/RgbFrame.h"
#include "simulation_core/EoCameraModel.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace aerosimlab::simulator
{
/** @brief EO 프레임을 H.264 영상과 촬영 시점 정보로 저장한다. */
class EoVideoRecorder final
{
public:
    enum class MissionOutcome
    {
        Completed,
        Failed
    };

    EoVideoRecorder();
    ~EoVideoRecorder();

    EoVideoRecorder(const EoVideoRecorder&) = delete;
    EoVideoRecorder& operator=(const EoVideoRecorder&) = delete;

    /** @brief H.264 MP4와 프레임별 JSON 기록을 시작한다. */
    [[nodiscard]] bool Start(const std::filesystem::path& videoPath,
        int width, int height, double framesPerSecond,
        const std::string& facilityId, const std::string& inspectionZoneId,
        std::string& error);

    /**
     * @brief 프레임과 촬영 정보를 제한된 대기열에 복사해 인코더 작업자에게 넘긴다.
     * @return 저장 대기열이 처리 속도를 따라가지 못하면 빠짐없이 저장할 수 있도록 false를 돌려준다.
     */
    [[nodiscard]] bool WriteFrame(const sensor::RgbFrame& frame,
        const simulation::CameraState& camera, double simulationTimeSeconds,
        simulation::CaptureAssessment captureAssessment,
        std::string& error);

    /** @brief 자동 점검 결과를 저장 목록에 붙인다. 수동 녹화에는 설정하지 않는다. */
    void SetMissionOutcome(MissionOutcome outcome, std::string failureReason = {});

    /** @brief 파일을 마무리하고 저장 상태·파일 크기를 담은 목록을 만든다. */
    [[nodiscard]] bool Stop(std::uint64_t& savedFrameCount, std::string& error);

    [[nodiscard]] bool IsRecording() const noexcept;
    [[nodiscard]] const std::filesystem::path& VideoPath() const noexcept;

    /** @brief 녹화 파일을 전송한 뒤 받을 쪽에서 대조할 CRC-32 값을 계산한다. */
    [[nodiscard]] static bool CalculateFileChecksum(
        const std::filesystem::path& path, std::string& checksum,
        std::string& error);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::filesystem::path videoPath_;
    std::filesystem::path metadataPath_;
    std::filesystem::path metadataTemporaryPath_;
    std::uint64_t savedFrameCount_ = 0;
    std::uint64_t queuedFrameCount_ = 0;
    int width_ = 0;
    int height_ = 0;
    double framesPerSecond_ = 0.0;
    std::string facilityId_;
    std::string inspectionZoneId_;
    std::uint64_t geometryReadyFrameCount_ = 0;
    std::optional<MissionOutcome> missionOutcome_;
    std::string missionFailureReason_;
};

/** @brief 운영체제의 사용자 데이터 폴더 아래에 새 녹화 파일 경로를 만든다. */
[[nodiscard]] std::filesystem::path NewEoRecordingPath();
}
