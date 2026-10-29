#include "AeroSimLabApp.h"

#include "graphics/Camera.h"
#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Renderer.h"
#include "graphics/Window.h"
#include "simulation_core/ScopedDuration.h"

#include <cstdint>

namespace aerosimlab::simulator
{
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
    renderer_->BeginView(target);
    DrawWorld(camera.GetViewMatrix(), camera.GetProjectionMatrix(),
        camera.GetPosition(), showMissionGuides, hiddenDroneId);
    renderer_->EndView(target);
    return target.GetTextureId();
}

}
