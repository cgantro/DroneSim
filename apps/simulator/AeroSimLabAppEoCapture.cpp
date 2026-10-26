#include "AeroSimLabApp.h"

#include "graphics/Camera.h"
#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Renderer.h"
#include "graphics/Window.h"
#include "simulation_core/ScopedDuration.h"
#include "vision/RgbFrameQuality.h"
#include "vision/RgbFrameView.h"

#include <cstdint>

namespace aerosimlab::simulator
{
std::uint32_t AeroSimLabApp::RenderScenePanel(int width, int height)
{
    aerosimlab::simulation::ScopedDuration measureScenePanel(
        benchmarkMode_ ? &frameMeasurements_.scenePanelRenderMicroseconds : nullptr);
    sceneTarget_->Resize(width, height);
    UpdateObserverCamera(width, height);
    return RenderWorldToTarget(*sceneTarget_, *observerCamera_);
}

std::uint32_t AeroSimLabApp::RenderCameraPanel()
{
    const auto& camera = simulation_.Camera();
    const int imageWidth = camera.model.imageWidthPixels;
    const int imageHeight = camera.model.imageHeightPixels;
    const bool targetSizeChanged = cameraTarget_->GetWidth() != imageWidth
        || cameraTarget_->GetHeight() != imageHeight;
    cameraTarget_->Resize(imageWidth, imageHeight);
    if (targetSizeChanged || !lastRenderedEoFrameTick_
        || *lastRenderedEoFrameTick_ != camera.lastFrameTick)
    {
        aerosimlab::simulation::ScopedDuration measureCameraScene(
            benchmarkMode_ ? &frameMeasurements_.cameraSceneRenderMicroseconds : nullptr);
        UpdateEoCamera(imageWidth, imageHeight);
        static_cast<void>(RenderWorldToTarget(*cameraTarget_, *eoCamera_));
        lastRenderedEoFrameTick_ = camera.lastFrameTick;
    }
    aerosimlab::simulation::ScopedDuration measureEoReadbackHandling(
        benchmarkMode_ ? &frameMeasurements_.eoReadbackHandlingMicroseconds : nullptr);
    CaptureEoFrame();
    return cameraTarget_->GetTextureId();
}

std::uint32_t AeroSimLabApp::RenderWorldToTarget(
    aerosimlab::graphics::MultisampleFramebuffer& target,
    const aerosimlab::graphics::Camera& camera)
{
    renderer_->BeginView(target);
    DrawWorld(camera.GetViewMatrix(), camera.GetProjectionMatrix(), camera.GetPosition());
    renderer_->EndView(target);
    return target.GetTextureId();
}

void AeroSimLabApp::CaptureEoFrame()
{
    ProcessCompletedEoReadback();
    QueueEoReadback();
}

void AeroSimLabApp::ProcessCompletedEoReadback()
{
    std::uint64_t simulationTick = 0;
    double pollDurationMicroseconds = 0.0;
    bool readCompleted = false;
    aerosimlab::graphics::RgbReadbackPollTimings pollTimings;
    {
        aerosimlab::simulation::ScopedDuration measurePoll(
            benchmarkMode_ ? &pollDurationMicroseconds : nullptr);
        readCompleted = cameraTarget_->TryReadRgbPixels(eoFrame_.pixels, simulationTick,
            benchmarkMode_ ? &pollTimings : nullptr);
    }
    if (benchmarkMode_)
    {
        frameMeasurements_.eoFenceWaitMicroseconds = pollTimings.fenceWaitCallMicroseconds;
        frameMeasurements_.eoBufferMapMicroseconds = pollTimings.bufferMapCallMicroseconds;
        frameMeasurements_.eoPixelCopyMicroseconds = pollTimings.pixelCopyMicroseconds;
        benchmarkMeasurements_.eoReadbackPollDurationsMicroseconds.push_back(
            pollDurationMicroseconds);
        benchmarkMeasurements_.eoFenceWaitCallDurationsMicroseconds.push_back(
            pollTimings.fenceWaitCallMicroseconds);
        if (pollTimings.fenceWaitFailed)
        {
            benchmarkMeasurements_.eoReadbackFallbackFinishDurationsMicroseconds.push_back(
                pollTimings.fallbackFinishMicroseconds);
            benchmarkMeasurements_.eoReadbackCleanupDurationsMicroseconds.push_back(
                pollTimings.cleanupMicroseconds);
        }
        if (pollTimings.pixelsReady)
        {
            benchmarkMeasurements_.eoPixelBufferResizeDurationsMicroseconds.push_back(
                pollTimings.pixelBufferResizeMicroseconds);
            benchmarkMeasurements_.eoPixelPackBufferBindDurationsMicroseconds.push_back(
                pollTimings.pixelPackBufferBindMicroseconds);
            benchmarkMeasurements_.eoPixelBufferMapCallDurationsMicroseconds.push_back(
                pollTimings.bufferMapCallMicroseconds);
            if (pollTimings.bufferMapped)
            {
                benchmarkMeasurements_.eoPixelCopyDurationsMicroseconds.push_back(
                    pollTimings.pixelCopyMicroseconds);
                benchmarkMeasurements_.eoPixelBufferUnmapCallDurationsMicroseconds.push_back(
                    pollTimings.bufferUnmapCallMicroseconds);
            }
            benchmarkMeasurements_.eoReadbackCleanupDurationsMicroseconds.push_back(
                pollTimings.cleanupMicroseconds);
        }
    }
    if (readCompleted)
    {
        eoReadbackHappenedThisFrame_ = true;
        ProcessEoPixels(simulationTick);
    }
}

void AeroSimLabApp::QueueEoReadback()
{
    const auto& camera = simulation_.Camera();
    if (camera.status != aerosimlab::simulation::CameraStatus::Active
        || camera.lastFrameTick <= lastQueuedEoFrameTick_)
        return;

    double enqueueDurationMicroseconds = 0.0;
    bool queued = false;
    aerosimlab::graphics::RgbReadbackEnqueueTimings readbackTimings;
    {
        aerosimlab::simulation::ScopedDuration measureEnqueue(
            benchmarkMode_ ? &enqueueDurationMicroseconds : nullptr);
        queued = cameraTarget_->QueueRgbReadback(camera.lastFrameTick,
            benchmarkMode_ ? &readbackTimings : nullptr);
    }
    if (queued)
    {
        lastQueuedEoFrameTick_ = camera.lastFrameTick;
        queuedEoCameraStates_.insert_or_assign(camera.lastFrameTick, camera);
        if (benchmarkMode_)
        {
            eoReadbackHappenedThisFrame_ = true;
            benchmarkMeasurements_.eoReadbackEnqueueDurationsMicroseconds.push_back(
                enqueueDurationMicroseconds);
            benchmarkMeasurements_.eoReadbackSlotSetupDurationsMicroseconds.push_back(
                readbackTimings.slotAndBufferCreationMicroseconds);
            benchmarkMeasurements_.eoPixelBufferSetupDurationsMicroseconds.push_back(
                readbackTimings.bufferBindingAndAllocationMicroseconds);
            benchmarkMeasurements_.eoReadTargetSetupDurationsMicroseconds.push_back(
                readbackTimings.readTargetSetupMicroseconds);
            benchmarkMeasurements_.eoReadPixelsCallDurationsMicroseconds.push_back(
                readbackTimings.readPixelsCallMicroseconds);
            benchmarkMeasurements_.eoFenceCreationCallDurationsMicroseconds.push_back(
                readbackTimings.fenceCreationCallMicroseconds);
            benchmarkMeasurements_.eoGlStateRestoreDurationsMicroseconds.push_back(
                readbackTimings.glStateRestoreMicroseconds);
        }
    }
}

void AeroSimLabApp::ProcessEoPixels(std::uint64_t simulationTick)
{
    eoFrame_.width = cameraTarget_->GetWidth();
    eoFrame_.height = cameraTarget_->GetHeight();
    eoFrame_.simulationTick = simulationTick;
    // 밝기·대비·선명도는 행 방향과 무관하므로 OpenGL 픽셀을 그대로 분석한다.
    eoFrame_.rowOrder = aerosimlab::sensor::RgbFrameRowOrder::BottomToTop;
    const auto cameraState = queuedEoCameraStates_.find(simulationTick);
    if (cameraState != queuedEoCameraStates_.end())
    {
        if (eoVideoRecorder_ && eoVideoRecorder_->IsRecording())
        {
            std::string error;
            const auto captureAssessment =
                aerosimlab::simulation::EoCameraModel::AssessCapture(
                    cameraState->second, inspectionSurface_, {}, inspectionOccluders_);
            if (!eoVideoRecorder_->WriteFrame(eoFrame_, cameraState->second,
                static_cast<double>(simulationTick) / 60.0,
                captureAssessment, error))
            {
                const bool missionIsActive =
                    inspectionMissionStage_ != InspectionMissionStage::Ready
                    && inspectionMissionStage_ != InspectionMissionStage::Saved
                    && inspectionMissionStage_ != InspectionMissionStage::GeometryOnly
                    && inspectionMissionStage_ != InspectionMissionStage::Failed;
                if (missionIsActive)
                {
                    FailInspectionMission("점검 영상을 저장하지 못했습니다. " + error);
                }
                else
                {
                    std::uint64_t savedFrames = 0;
                    std::string finishError;
                    static_cast<void>(eoVideoRecorder_->Stop(savedFrames, finishError));
                    eoVideoRecorder_.reset();
                    const std::string detail = finishError.empty()
                        ? error : error + " " + finishError;
                    window_->SetTitle("DroneSim | EO 녹화 중단: " + detail);
                    static_cast<void>(savedFrames);
                }
            }
#ifndef __EMSCRIPTEN__
            else if (inspectionMissionStage_ == InspectionMissionStage::Capturing
                && captureAssessment
                    == aerosimlab::simulation::CaptureAssessment::Ready)
            {
                inspectionReadyFrameSaved_ = true;
            }
#endif
        }
        queuedEoCameraStates_.erase(cameraState);
    }
    double analysisDurationMicroseconds = 0.0;
    // 분석기가 프레임 메모리를 빌려 쓰므로 분석이 끝날 때까지 픽셀을 유지한다.
    aerosimlab::vision::RgbFrameQuality quality;
    {
        aerosimlab::simulation::ScopedDuration measureAnalysis(
            benchmarkMode_ ? &analysisDurationMicroseconds : nullptr);
        quality = visionAnalyzer_->Analyze(
            aerosimlab::vision::MakeRgbFrameView(eoFrame_));
    }
    totalMeanBrightness_ += quality.meanBrightness;
    totalMeanContrast_ += quality.meanContrast;
    totalSharpness_ += quality.sharpness;
    ++benchmarkMeasurements_.visionSampleCount;
    if (benchmarkMode_)
        benchmarkMeasurements_.visionDurationsMicroseconds.push_back(
            analysisDurationMicroseconds);
}

void AeroSimLabApp::DrainEoReadbacks()
{
    const auto& camera = simulation_.Camera();
    while (true)
    {
        cameraTarget_->FinishRgbReadbacks();
        std::uint64_t simulationTick = 0;
        while (cameraTarget_->TryReadRgbPixels(eoFrame_.pixels, simulationTick))
            ProcessEoPixels(simulationTick);

        if (camera.status != aerosimlab::simulation::CameraStatus::Active
            || camera.lastFrameTick <= lastQueuedEoFrameTick_)
            return;
        if (!cameraTarget_->QueueRgbReadback(camera.lastFrameTick))
            return;
        lastQueuedEoFrameTick_ = camera.lastFrameTick;
    }
}
}
