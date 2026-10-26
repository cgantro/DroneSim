#include "EoVideoRecorder.h"

#include <nlohmann/json.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>

namespace aerosimlab::simulator
{
struct EoVideoRecorder::Impl
{
    struct QueuedFrame
    {
        sensor::RgbFrame frame;
        simulation::CameraState camera;
        double simulationTimeSeconds = 0.0;
        simulation::CaptureAssessment captureAssessment = simulation::CaptureAssessment::TooFar;
        std::uint64_t frameIndex = 0;
    };

    cv::VideoWriter writer;
    std::ofstream metadata;
    std::mutex queueMutex;
    std::condition_variable queueChanged;
    std::deque<QueuedFrame> frames;
    std::thread encoderThread;
    std::atomic_bool sessionOpen = false;
    bool acceptingFrames = false;
    bool stopRequested = false;
    std::string workerError;

    void Fail(std::string error)
    {
        {
            std::lock_guard lock(queueMutex);
            if (workerError.empty())
                workerError = std::move(error);
            acceptingFrames = false;
            stopRequested = true;
            frames.clear();
        }
        queueChanged.notify_all();
    }
};

EoVideoRecorder::EoVideoRecorder() = default;

namespace
{
using Json = nlohmann::json;
constexpr std::size_t maximumQueuedFrames = 8U;

std::optional<std::uint32_t> CalculateCrc32(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
        return std::nullopt;

    std::array<unsigned char, 64U * 1024U> buffer{};
    std::uint32_t checksum = 0xFFFFFFFFU;
    while (input)
    {
        input.read(reinterpret_cast<char*>(buffer.data()),
            static_cast<std::streamsize>(buffer.size()));
        const std::streamsize bytesRead = input.gcount();
        for (std::streamsize index = 0; index < bytesRead; ++index)
        {
            checksum ^= buffer[static_cast<std::size_t>(index)];
            for (int bit = 0; bit < 8; ++bit)
            {
                const std::uint32_t lowBitMask = 0U - (checksum & 1U);
                checksum = (checksum >> 1U) ^ (0xEDB88320U & lowBitMask);
            }
        }
    }
    if (!input.eof())
        return std::nullopt;
    return checksum ^ 0xFFFFFFFFU;
}

std::string FormatCrc32(std::uint32_t checksum)
{
    std::ostringstream formatted;
    formatted << std::uppercase << std::hex << std::setw(8)
        << std::setfill('0') << checksum;
    return formatted.str();
}

const char* MissionOutcomeName(EoVideoRecorder::MissionOutcome outcome)
{
    switch (outcome)
    {
    case EoVideoRecorder::MissionOutcome::Completed: return "COMPLETED";
    case EoVideoRecorder::MissionOutcome::Failed: return "FAILED";
    }
    return "UNKNOWN";
}

std::filesystem::path UserDataDirectory()
{
#ifdef _WIN32
    if (const char* localAppData = std::getenv("LOCALAPPDATA");
        localAppData && *localAppData)
        return std::filesystem::path{localAppData} / "DroneSim";
#else
    if (const char* xdgDataHome = std::getenv("XDG_DATA_HOME");
        xdgDataHome && *xdgDataHome)
        return std::filesystem::path{xdgDataHome} / "dronesim";
    if (const char* home = std::getenv("HOME"); home && *home)
        return std::filesystem::path{home} / ".local" / "share" / "dronesim";
#endif
    return std::filesystem::temp_directory_path() / "DroneSim";
}
}

EoVideoRecorder::~EoVideoRecorder()
{
    std::uint64_t savedFrameCount = 0;
    std::string error;
    static_cast<void>(Stop(savedFrameCount, error));
}

bool EoVideoRecorder::Start(const std::filesystem::path& videoPath,
    int width, int height, double framesPerSecond,
    const std::string& facilityId, const std::string& inspectionZoneId,
    std::string& error)
{
    if (IsRecording())
    {
        error = "이미 영상 녹화 중입니다.";
        return false;
    }
    if (videoPath.empty() || width <= 0 || height <= 0
        || facilityId.empty() || inspectionZoneId.empty()
        || !(framesPerSecond > 0.0))
    {
        error = "영상 설정과 시설물·점검 구역 식별값이 필요합니다.";
        return false;
    }

    impl_ = std::make_unique<Impl>();
    try
    {
        std::filesystem::create_directories(videoPath.parent_path());
        metadataPath_ = videoPath;
        metadataPath_.replace_extension(".jsonl");
        metadataTemporaryPath_ = metadataPath_;
        metadataTemporaryPath_ += ".partial";
        impl_->metadata.open(metadataTemporaryPath_, std::ios::out | std::ios::trunc);
        if (!impl_->metadata)
        {
            error = "영상 메타데이터 파일을 만들지 못했습니다.";
            return false;
        }
        videoPath_ = videoPath;
        width_ = width;
        height_ = height;
        framesPerSecond_ = framesPerSecond;
        savedFrameCount_ = 0;
        queuedFrameCount_ = 0;
        geometryReadyFrameCount_ = 0;
        facilityId_ = facilityId;
        inspectionZoneId_ = inspectionZoneId;

        const int h264 = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
#ifdef _WIN32
        constexpr int videoBackend = cv::CAP_MSMF;
#else
        constexpr int videoBackend = cv::CAP_FFMPEG;
#endif
        if (!impl_->writer.open(videoPath_.string(), videoBackend, h264,
            framesPerSecond, cv::Size{width_, height_}, true))
        {
            impl_->metadata.close();
            std::error_code ignored;
            std::filesystem::remove(metadataTemporaryPath_, ignored);
            std::filesystem::remove(videoPath_, ignored);
            error = "현재 OpenCV 영상 장치에서 H.264 MP4 인코더를 열지 못했습니다.";
            return false;
        }

        impl_->sessionOpen = true;
        impl_->acceptingFrames = true;
        impl_->encoderThread = std::thread([this]
        {
            try
            {
                for (;;)
                {
                    Impl::QueuedFrame queued;
                    {
                        std::unique_lock lock(impl_->queueMutex);
                        impl_->queueChanged.wait(lock, [this]
                        {
                            return impl_->stopRequested || !impl_->frames.empty();
                        });
                        if (impl_->frames.empty())
                        {
                            if (impl_->stopRequested)
                                return;
                            continue;
                        }
                        queued = std::move(impl_->frames.front());
                        impl_->frames.pop_front();
                    }

                    cv::Mat rgb(queued.frame.height, queued.frame.width, CV_8UC3,
                        queued.frame.pixels.data());
                    cv::Mat topToBottom;
                    if (queued.frame.rowOrder == sensor::RgbFrameRowOrder::BottomToTop)
                        cv::flip(rgb, topToBottom, 0);
                    else
                        topToBottom = rgb;
                    cv::Mat bgr;
                    cv::cvtColor(topToBottom, bgr, cv::COLOR_RGB2BGR);
                    impl_->writer.write(bgr);
                    if (!impl_->writer.isOpened())
                        throw std::runtime_error("H.264 영상 인코더가 프레임 저장 중 종료됐습니다.");

                    const glm::dvec3& position = queued.camera.positionMeters;
                    const glm::dquat& orientation = queued.camera.orientation;
                    const Json record{
                        {"capture_state", "CAPTURED"},
                        {"facility_id", facilityId_},
                        {"inspection_zone_id", inspectionZoneId_},
                        {"capture_assessment", simulation::ToString(queued.captureAssessment)},
                        {"frame_index", queued.frameIndex},
                        {"simulation_tick", queued.frame.simulationTick},
                        {"simulation_time_seconds", queued.simulationTimeSeconds},
                        {"width_pixels", queued.frame.width},
                        {"height_pixels", queued.frame.height},
                        {"position_meters", {position.x, position.y, position.z}},
                        {"orientation_quaternion", {
                            {"w", orientation.w}, {"x", orientation.x},
                            {"y", orientation.y}, {"z", orientation.z}}}};
                    impl_->metadata << record.dump() << '\n';
                    impl_->metadata.flush();
                    if (!impl_->metadata)
                        throw std::runtime_error("영상 프레임 정보를 저장하지 못했습니다.");

                    ++savedFrameCount_;
                    if (queued.captureAssessment == simulation::CaptureAssessment::Ready)
                        ++geometryReadyFrameCount_;
                }
            }
            catch (const std::exception& exception)
            {
                impl_->Fail(exception.what());
            }
            catch (...)
            {
                impl_->Fail("영상 인코더에서 알 수 없는 오류가 발생했습니다.");
            }
        });
    }
    catch (const std::exception& exception)
    {
        if (impl_)
        {
            impl_->writer.release();
            impl_->metadata.close();
        }
        impl_.reset();
        error = exception.what();
        return false;
    }
    return true;
}

bool EoVideoRecorder::WriteFrame(const sensor::RgbFrame& frame,
    const simulation::CameraState& camera, double simulationTimeSeconds,
    simulation::CaptureAssessment captureAssessment,
    std::string& error)
{
    if (!IsRecording() || !frame.IsValid()
        || frame.width != width_ || frame.height != height_)
    {
        error = "녹화 중인 영상과 프레임 크기가 맞지 않습니다.";
        return false;
    }

    try
    {
        Impl::QueuedFrame queued{
            .frame = frame,
            .camera = camera,
            .simulationTimeSeconds = simulationTimeSeconds,
            .captureAssessment = captureAssessment,
        };
        {
            std::lock_guard lock(impl_->queueMutex);
            if (!impl_->acceptingFrames)
            {
                error = impl_->workerError.empty()
                    ? "영상 인코더가 새 프레임을 받지 않습니다."
                    : impl_->workerError;
                return false;
            }
            if (impl_->frames.size() >= maximumQueuedFrames)
            {
                impl_->workerError = "영상 인코더가 느려 저장 대기열이 가득 찼습니다.";
                impl_->acceptingFrames = false;
                impl_->stopRequested = true;
                impl_->frames.clear();
                error = impl_->workerError;
                impl_->queueChanged.notify_all();
                return false;
            }
            queued.frameIndex = queuedFrameCount_++;
            impl_->frames.push_back(std::move(queued));
        }
        impl_->queueChanged.notify_one();
    }
    catch (const std::exception& exception)
    {
        impl_->Fail(exception.what());
        error = exception.what();
        return false;
    }
    return true;
}

void EoVideoRecorder::SetMissionOutcome(
    MissionOutcome outcome, std::string failureReason)
{
    missionOutcome_ = outcome;
    missionFailureReason_ = outcome == MissionOutcome::Failed
        ? std::move(failureReason) : std::string{};
}

bool EoVideoRecorder::Stop(std::uint64_t& savedFrameCount, std::string& error)
{
    savedFrameCount = savedFrameCount_;
    if (!impl_)
    {
        error = "진행 중인 영상 녹화가 없습니다.";
        return false;
    }

    {
        std::lock_guard lock(impl_->queueMutex);
        impl_->acceptingFrames = false;
        impl_->stopRequested = true;
    }
    impl_->queueChanged.notify_all();
    if (impl_->encoderThread.joinable())
        impl_->encoderThread.join();

    impl_->writer.release();
    impl_->metadata.flush();
    const bool metadataFlushed = static_cast<bool>(impl_->metadata);
    impl_->metadata.close();
    impl_->sessionOpen = false;
    savedFrameCount = savedFrameCount_;

    bool saved = metadataFlushed;
    {
        std::lock_guard lock(impl_->queueMutex);
        if (!impl_->workerError.empty())
        {
            saved = false;
            error = impl_->workerError;
        }
    }
    std::error_code fileError;
    const std::uintmax_t videoBytes = std::filesystem::file_size(videoPath_, fileError);
    if (fileError || videoBytes == 0)
    {
        saved = false;
        if (error.empty())
            error = "녹화 영상을 마무리하지 못했습니다.";
    }
    if (saved)
    {
        std::filesystem::rename(metadataTemporaryPath_, metadataPath_, fileError);
        if (fileError)
        {
            saved = false;
            error = "프레임 정보를 저장 파일로 확정하지 못했습니다.";
        }
    }

    if (saved)
    {
        std::string videoChecksum;
        std::string metadataChecksum;
        if (!CalculateFileChecksum(videoPath_, videoChecksum, error)
            || !CalculateFileChecksum(metadataPath_, metadataChecksum, error))
        {
            saved = false;
        }
        if (!saved)
        {
            impl_.reset();
            width_ = 0;
            height_ = 0;
            framesPerSecond_ = 0.0;
            return false;
        }

        std::filesystem::path manifestPath = videoPath_;
        manifestPath.replace_extension(".manifest.json");
        std::filesystem::path temporaryManifestPath = manifestPath;
        temporaryManifestPath += ".partial";
        std::ofstream manifestFile(temporaryManifestPath,
            std::ios::out | std::ios::trunc);
        Json manifest{
            {"collection_state", "SAVED"},
            {"facility_id", facilityId_},
            {"inspection_zone_id", inspectionZoneId_},
            {"video_file", videoPath_.filename().string()},
            {"metadata_file", metadataPath_.filename().string()},
            {"saved_frame_count", savedFrameCount_},
            {"geometry_ready_frame_count", geometryReadyFrameCount_},
            {"width_pixels", width_},
            {"height_pixels", height_},
            {"frames_per_second", framesPerSecond_},
            {"video_bytes", videoBytes},
            {"metadata_bytes", std::filesystem::file_size(metadataPath_)},
            {"checksum_algorithm", "CRC-32/ISO-HDLC"},
            {"video_checksum", videoChecksum},
            {"metadata_checksum", metadataChecksum}};
        if (missionOutcome_)
        {
            manifest["mission_outcome"] = MissionOutcomeName(*missionOutcome_);
            if (*missionOutcome_ == MissionOutcome::Failed)
                manifest["mission_failure_reason"] = missionFailureReason_;
        }
        manifestFile << manifest.dump(2) << '\n';
        manifestFile.flush();
        saved = static_cast<bool>(manifestFile);
        manifestFile.close();
        if (saved)
        {
            std::filesystem::rename(temporaryManifestPath, manifestPath, fileError);
            saved = !fileError;
        }
        if (!saved)
        {
            error = "녹화 상태 목록을 저장하지 못했습니다.";
            std::filesystem::remove(temporaryManifestPath, fileError);
        }
    }
    if (!metadataFlushed && error.empty())
        error = "프레임 메타데이터를 디스크에 기록하지 못했습니다.";

    impl_.reset();
    width_ = 0;
    height_ = 0;
    framesPerSecond_ = 0.0;
    return saved;
}

bool EoVideoRecorder::IsRecording() const noexcept
{
    return impl_ && impl_->sessionOpen.load();
}

const std::filesystem::path& EoVideoRecorder::VideoPath() const noexcept
{
    return videoPath_;
}

bool EoVideoRecorder::CalculateFileChecksum(
    const std::filesystem::path& path, std::string& checksum, std::string& error)
{
    const auto value = CalculateCrc32(path);
    if (!value)
    {
        error = "파일 체크섬을 계산하지 못했습니다.";
        return false;
    }
    checksum = FormatCrc32(*value);
    return true;
}

std::filesystem::path NewEoRecordingPath()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    std::tm localTime{};
#ifdef _WIN32
    localtime_s(&localTime, &time);
#else
    localtime_r(&time, &localTime);
#endif
    std::ostringstream name;
    name << "inspection_" << std::put_time(&localTime, "%Y%m%d_%H%M%S")
         << '_' << std::setw(3) << std::setfill('0') << milliseconds << ".mp4";
    return UserDataDirectory() / "recordings" / name.str();
}
}
