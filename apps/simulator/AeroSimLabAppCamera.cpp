#include "AeroSimLabApp.h"

#include "graphics/Camera.h"
#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Renderer.h"
#include "graphics/Window.h"
#include "simulation_core/ScopedDuration.h"

#include <cstdint>

namespace aerosimlab::simulator
{
/** @brief 장면 패널 크기에 맞춰 관찰 카메라로 그린 결과를 돌려준다. */
std::uint32_t AeroSimLabApp::RenderScenePanel(int width, int height)
{
    aerosimlab::simulation::ScopedDuration measureScenePanel(
        benchmarkMode_ ? &frameMeasurements_.scenePanelRenderMicroseconds : nullptr);
    sceneTarget_->Resize(width, height);
    UpdateObserverCamera(width, height);
    return RenderWorldToTarget(*sceneTarget_, *observerCamera_, true);
}

std::uint32_t AeroSimLabApp::RenderCameraPanel(
    aerosimlab::simulation::DroneId droneId)
{
    // EO 영상의 실제 픽셀 크기를 렌더 타깃 크기로 사용한다.
    const auto& camera = simulation_.Camera(droneId);
    const int imageWidth = camera.model.imageWidthPixels;
    const int imageHeight = camera.model.imageHeightPixels;

    if (droneId != simulation_.PrimaryDroneId())
    {
        auto& view = auxiliaryCameraViews_[droneId];
        if (!view.camera)
        {
            const float aspectRatio = static_cast<float>(imageWidth)
                / static_cast<float>(imageHeight);
            view.camera = std::make_unique<aerosimlab::graphics::Camera>(
                glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, aspectRatio,
                static_cast<float>(camera.model.verticalFieldOfViewDegrees),
                0.05F, 200.0F);
        }
        if (!view.target)
        {
            view.target = std::make_unique<
                aerosimlab::graphics::MultisampleFramebuffer>(imageWidth, imageHeight);
        }

        const bool targetSizeChanged = view.target->GetWidth() != imageWidth
            || view.target->GetHeight() != imageHeight;
        view.target->Resize(imageWidth, imageHeight);
        // 카메라 프레임이 바뀌거나 패널 크기가 달라진 때에만 장면을 다시 그린다.
        if (targetSizeChanged || !view.lastRenderedTick
            || *view.lastRenderedTick != camera.lastFrameTick)
        {
            aerosimlab::simulation::ScopedDuration measureCameraScene(
                benchmarkMode_
                    ? &frameMeasurements_.cameraSceneRenderMicroseconds : nullptr);
            UpdateEoCamera(droneId, *view.camera, imageWidth, imageHeight);
            static_cast<void>(RenderWorldToTarget(*view.target, *view.camera,
                false, droneId));
            view.lastRenderedTick = camera.lastFrameTick;
        }
        return view.target->GetTextureId();
    }

    const bool targetSizeChanged = cameraTarget_->GetWidth() != imageWidth
        || cameraTarget_->GetHeight() != imageHeight;
    cameraTarget_->Resize(imageWidth, imageHeight);
    // 주 드론도 보조 카메라와 같은 기준으로 중복 렌더링을 건너뛴다.
    if (targetSizeChanged || !lastRenderedEoFrameTick_
        || *lastRenderedEoFrameTick_ != camera.lastFrameTick)
    {
        aerosimlab::simulation::ScopedDuration measureCameraScene(
            benchmarkMode_ ? &frameMeasurements_.cameraSceneRenderMicroseconds : nullptr);
        UpdateEoCamera(imageWidth, imageHeight);
        static_cast<void>(RenderWorldToTarget(*cameraTarget_, *eoCamera_, false,
            simulation_.PrimaryDroneId()));
        lastRenderedEoFrameTick_ = camera.lastFrameTick;
    }
    return cameraTarget_->GetTextureId();
}

std::uint32_t AeroSimLabApp::RenderWorldToTarget(
    aerosimlab::graphics::MultisampleFramebuffer& target,
    const aerosimlab::graphics::Camera& camera,
    bool showMissionGuides,
    std::optional<aerosimlab::simulation::DroneId> hiddenDroneId)
{
    // 지정한 카메라로 월드를 프레임버퍼에 그린 뒤 패널이 쓸 텍스처 ID를 돌려준다.
    renderer_->BeginView(target);
    DrawWorld(camera.GetViewMatrix(), camera.GetProjectionMatrix(),
        camera.GetPosition(), showMissionGuides, hiddenDroneId);
    renderer_->EndView(target);
    return target.GetTextureId();
}

}
