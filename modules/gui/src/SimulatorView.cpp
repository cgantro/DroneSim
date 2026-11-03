#include "gui/SimulatorView.h"

#include "graphics/Camera.h"
#include "graphics/CoordinateTransform.h"
#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Renderer.h"
#include "simulation_core/EoCameraModel.h"
#include "simulation_core/ScopedDuration.h"

namespace dronesim::gui
{
SimulatorView::SimulatorView(graphics::Renderer& renderer, int width, int height,
    float cameraFieldOfViewDegrees)
    : renderer_(renderer),
      sceneCamera_(width, height),
      primaryCamera_(std::make_unique<graphics::Camera>(
          glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F,
          cameraFieldOfViewDegrees, 0.05F, 200.0F)),
      sceneTarget_(std::make_unique<graphics::MultisampleFramebuffer>(width, height)),
      primaryCameraTarget_(
          std::make_unique<graphics::MultisampleFramebuffer>(width, height))
{
}

SimulatorView::~SimulatorView() = default;

ObserverCameraController& SimulatorView::SceneCamera() noexcept
{
    return sceneCamera_;
}

std::uint32_t SimulatorView::RenderScenePanel(
    const simulation::SimulationCore& simulation,
    simulation::DroneId observerDroneId,
    const std::vector<flight_planning::BuildingObstacle>& buildings,
    int width, int height,
    const DrawWorld& drawWorld,
    double* renderMicroseconds)
{
    simulation::ScopedDuration measure(renderMicroseconds);
    sceneTarget_->Resize(width, height);
    sceneCamera_.Update(simulation, observerDroneId, buildings, width, height);
    DrawToTarget(*sceneTarget_, sceneCamera_.ViewCamera(), true, std::nullopt, drawWorld);
    return sceneTarget_->GetTextureId();
}

std::uint32_t SimulatorView::RenderCameraPanel(
    const simulation::SimulationCore& simulation,
    simulation::DroneId droneId, const DrawWorld& drawWorld,
    double* renderMicroseconds)
{
    const auto& cameraState = simulation.Camera(droneId);
    const int width = cameraState.model.imageWidthPixels;
    const int height = cameraState.model.imageHeightPixels;

    if (droneId != simulation.PrimaryDroneId())
    {
        auto& view = otherCameraViews_[droneId];
        if (!view.camera)
        {
            const float aspectRatio = static_cast<float>(width)
                / static_cast<float>(height);
            view.camera = std::make_unique<graphics::Camera>(
                glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, aspectRatio,
                static_cast<float>(cameraState.model.verticalFieldOfViewDegrees),
                0.05F, 200.0F);
        }
        if (!view.target)
            view.target = std::make_unique<graphics::MultisampleFramebuffer>(width, height);

        const bool targetSizeChanged = view.target->GetWidth() != width
            || view.target->GetHeight() != height;
        view.target->Resize(width, height);
        if (targetSizeChanged || !view.lastRenderedTick
            || *view.lastRenderedTick != cameraState.lastFrameTick)
        {
            simulation::ScopedDuration measure(renderMicroseconds);
            UpdateEoCamera(simulation, droneId, *view.camera, width, height);
            DrawToTarget(*view.target, *view.camera, false, droneId, drawWorld);
            view.lastRenderedTick = cameraState.lastFrameTick;
        }
        return view.target->GetTextureId();
    }

    const bool targetSizeChanged = primaryCameraTarget_->GetWidth() != width
        || primaryCameraTarget_->GetHeight() != height;
    primaryCameraTarget_->Resize(width, height);
    if (targetSizeChanged || !lastPrimaryCameraTick_
        || *lastPrimaryCameraTick_ != cameraState.lastFrameTick)
    {
        simulation::ScopedDuration measure(renderMicroseconds);
        UpdateEoCamera(simulation, droneId, *primaryCamera_, width, height);
        DrawToTarget(*primaryCameraTarget_, *primaryCamera_, false, droneId, drawWorld);
        lastPrimaryCameraTick_ = cameraState.lastFrameTick;
    }
    return primaryCameraTarget_->GetTextureId();
}

void SimulatorView::UpdateEoCamera(const simulation::SimulationCore& simulation,
    simulation::DroneId droneId, graphics::Camera& camera, int width, int height)
{
    const auto& cameraState = simulation.Camera(droneId);
    const glm::vec3 cameraPosition =
        graphics::EnuPositionToRender(cameraState.positionMeters);
    const glm::vec3 viewDirection = graphics::EnuDirectionToRender(
        simulation::EoCameraModel::OpticalAxis(cameraState));
    const glm::vec3 cameraUp = graphics::EnuDirectionToRender(
        simulation::EoCameraModel::UpAxis(cameraState));
    camera.SetPosition(cameraPosition);
    camera.SetTarget(cameraPosition + viewDirection);
    camera.SetUp(cameraUp);
    camera.SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    camera.SetFieldOfViewDegrees(
        static_cast<float>(cameraState.model.verticalFieldOfViewDegrees));
}

void SimulatorView::DrawToTarget(graphics::MultisampleFramebuffer& target,
    const graphics::Camera& camera, bool showMissionGuides,
    std::optional<simulation::DroneId> hiddenDroneId,
    const DrawWorld& drawWorld)
{
    renderer_.BeginView(target);
    drawWorld(camera.GetViewMatrix(), camera.GetProjectionMatrix(),
        camera.GetPosition(), showMissionGuides, hiddenDroneId);
    renderer_.EndView(target);
}
}
