#include "gui/SimulatorView.h"

#include "graphics/Camera.h"
#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Renderer.h"
#include "simulation_core/EoCameraModel.h"
#include "simulation_core/ScopedDuration.h"

#include <algorithm>
#include <cmath>

namespace dronesim::gui
{
namespace
{
glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    return {static_cast<float>(positionMeters.x),
        static_cast<float>(positionMeters.z), static_cast<float>(-positionMeters.y)};
}

glm::vec3 ToRenderDirection(const glm::dvec3& direction)
{
    return {static_cast<float>(direction.x),
        static_cast<float>(direction.z), static_cast<float>(-direction.y)};
}
}

SimulatorView::SimulatorView(graphics::Renderer& renderer, int width, int height,
    float cameraFieldOfViewDegrees)
    : renderer_(renderer),
      observerCamera_(std::make_unique<graphics::Camera>(
          glm::vec3{0.0F, 145.0F, 285.0F}, glm::vec3{0.0F, 12.0F, 0.0F},
          1.0F, 45.0F, 0.1F, 2000.0F)),
      primaryCamera_(std::make_unique<graphics::Camera>(
          glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F,
          cameraFieldOfViewDegrees, 0.05F, 200.0F)),
      sceneTarget_(std::make_unique<graphics::MultisampleFramebuffer>(width, height)),
      primaryCameraTarget_(
          std::make_unique<graphics::MultisampleFramebuffer>(width, height))
{
}

SimulatorView::~SimulatorView() = default;

std::uint32_t SimulatorView::RenderScenePanel(
    const simulation::SimulationCore& simulation,
    simulation::DroneId observerDroneId,
    const std::vector<flight_planning::BuildingObstacle>& buildings,
    int width, int height, const DrawWorld& drawWorld,
    double* renderMicroseconds)
{
    simulation::ScopedDuration measure(renderMicroseconds);
    sceneTarget_->Resize(width, height);
    UpdateObserverCamera(simulation, observerDroneId, buildings, width, height);
    DrawToTarget(*sceneTarget_, *observerCamera_, true, std::nullopt, drawWorld);
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

void SimulatorView::UpdateObserverCamera(
    const simulation::SimulationCore& simulation,
    simulation::DroneId observerDroneId,
    const std::vector<flight_planning::BuildingObstacle>& buildings,
    int width, int height)
{
    const auto& state = observerDroneId == 0U
        ? simulation.State() : simulation.State(observerDroneId);
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const glm::dvec3 forwardMeters{
        std::cos(state.yawRadians), std::sin(state.yawRadians), 0.0};
    constexpr double followDistanceMeters = 12.0;
    constexpr double followHeightMeters = 3.5;
    glm::dvec3 cameraPositionMeters = state.positionMeters
        - forwardMeters * followDistanceMeters
        + glm::dvec3{0.0, 0.0, followHeightMeters};

    const auto isInsideBuilding = [&buildings](const glm::dvec3& position)
    {
        return std::any_of(buildings.begin(), buildings.end(),
            [&position](const flight_planning::BuildingObstacle& building)
            {
                return flight_planning::ContainsBuildingVolume(building, position);
            });
    };
    if (isInsideBuilding(cameraPositionMeters))
    {
        for (double retreatMeters = 1.0; retreatMeters <= 24.0; retreatMeters += 1.0)
        {
            const glm::dvec3 candidate = cameraPositionMeters
                - forwardMeters * retreatMeters;
            if (!isInsideBuilding(candidate))
            {
                cameraPositionMeters = candidate;
                break;
            }
        }
    }

    observerCamera_->SetPosition(ToRenderPosition(cameraPositionMeters));
    observerCamera_->SetTarget(dronePosition + glm::vec3{0.0F, 0.04F, 0.0F});
    observerCamera_->SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    observerCamera_->SetFieldOfViewDegrees(45.0F);
}

void SimulatorView::UpdateEoCamera(const simulation::SimulationCore& simulation,
    simulation::DroneId droneId, graphics::Camera& camera, int width, int height)
{
    const auto& cameraState = simulation.Camera(droneId);
    const glm::vec3 cameraPosition = ToRenderPosition(cameraState.positionMeters);
    const glm::vec3 viewDirection = ToRenderDirection(
        simulation::EoCameraModel::OpticalAxis(cameraState));
    const glm::vec3 cameraUp = ToRenderDirection(
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
