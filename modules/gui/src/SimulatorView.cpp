#include "gui/SimulatorView.h"

#include "graphics/Camera.h"
#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Renderer.h"
#include "simulation_core/EoCameraModel.h"
#include "simulation_core/ScopedDuration.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace dronesim::gui
{
namespace
{
constexpr double initialObserverDistanceMeters = 12.5;
constexpr double minimumObserverDistanceMeters = 2.5;
constexpr double maximumObserverDistanceMeters = 100.0;
constexpr double observerZoomStep = 1.18;
constexpr double cameraRadiansPerPixel = 0.006;
constexpr double minimumObserverPitchRadians = 0.14;
constexpr double maximumObserverPitchRadians = 1.30;
constexpr double observerVerticalFieldOfViewRadians = std::numbers::pi_v<double> / 4.0;

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
      observerDistanceMeters_(initialObserverDistanceMeters),
      observerOrbitPitchRadians_(std::atan2(3.5, 12.0)),
      primaryCamera_(std::make_unique<graphics::Camera>(
          glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F,
          cameraFieldOfViewDegrees, 0.05F, 200.0F)),
      sceneTarget_(std::make_unique<graphics::MultisampleFramebuffer>(width, height)),
      primaryCameraTarget_(
          std::make_unique<graphics::MultisampleFramebuffer>(width, height))
{
}

SimulatorView::~SimulatorView() = default;

void SimulatorView::ZoomObserverCamera(double wheelSteps) noexcept
{
    if (!std::isfinite(wheelSteps) || wheelSteps == 0.0)
        return;

    observerDistanceMeters_ = std::clamp(
        observerDistanceMeters_ * std::pow(observerZoomStep, -wheelSteps),
        minimumObserverDistanceMeters, maximumObserverDistanceMeters);
}

void SimulatorView::RotateObserverCamera(double deltaX, double deltaY) noexcept
{
    if (!std::isfinite(deltaX) || !std::isfinite(deltaY))
        return;

    // ImGui 이동량은 화면 픽셀이다. 픽셀마다 일정한 각도로 바꿔 드론을 중심으로 시점을 돈다.
    observerOrbitYawRadians_ = std::remainder(
        observerOrbitYawRadians_ + deltaX * cameraRadiansPerPixel,
        2.0 * std::numbers::pi_v<double>);
    // 위를 지나 카메라가 뒤집히거나 정확히 위를 바라볼 때 생기는 방향 계산 불안정을 막는다.
    observerOrbitPitchRadians_ = std::clamp(
        observerOrbitPitchRadians_ - deltaY * cameraRadiansPerPixel,
        minimumObserverPitchRadians, maximumObserverPitchRadians);
}

void SimulatorView::PanObserverCamera(
    double deltaX, double deltaY, double viewportHeightPixels) noexcept
{
    if (!std::isfinite(deltaX) || !std::isfinite(deltaY)
        || !std::isfinite(viewportHeightPixels) || viewportHeightPixels <= 0.0)
        return;

    const double cameraYaw = observerOrbitYawRadians_ + std::numbers::pi_v<double>;
    const double sinYaw = std::sin(cameraYaw);
    const double cosYaw = std::cos(cameraYaw);
    const double sinPitch = std::sin(observerOrbitPitchRadians_);
    const double cosPitch = std::cos(observerOrbitPitchRadians_);
    const glm::dvec3 screenRight{-sinYaw, cosYaw, 0.0};
    const glm::dvec3 screenUp{
        -sinPitch * cosYaw, -sinPitch * sinYaw, cosPitch};
    // 세로 시야각에서 화면 높이에 대응하는 실제 길이(2·거리·tan(FOV/2))를 구한다.
    const double metersPerPixel = 2.0 * observerDistanceMeters_
        * std::tan(observerVerticalFieldOfViewRadians * 0.5)
        / viewportHeightPixels;

    // 드래그한 화면이 커서와 함께 움직이도록 관찰 중심은 반대 방향으로 옮긴다.
    observerTargetOffsetMeters_ += screenRight * (-deltaX * metersPerPixel)
        + screenUp * (deltaY * metersPerPixel);
}

void SimulatorView::ResetObserverCamera() noexcept
{
    observerDistanceMeters_ = initialObserverDistanceMeters;
    observerOrbitYawRadians_ = 0.0;
    observerOrbitPitchRadians_ = std::atan2(3.5, 12.0);
    observerTargetOffsetMeters_ = glm::dvec3{0.0};
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
    const double cameraYaw = state.yawRadians + std::numbers::pi_v<double>
        + observerOrbitYawRadians_;
    // 거리와 위아래 각도로 구면 위치를 만들고, ENU의 동·북·위 좌표에 더한다.
    const double horizontalDistanceMeters = observerDistanceMeters_
        * std::cos(observerOrbitPitchRadians_);
    const glm::dvec3 targetPositionMeters = state.positionMeters
        + observerTargetOffsetMeters_;
    glm::dvec3 cameraPositionMeters = targetPositionMeters
        + glm::dvec3{
            horizontalDistanceMeters * std::cos(cameraYaw),
            horizontalDistanceMeters * std::sin(cameraYaw),
            observerDistanceMeters_ * std::sin(observerOrbitPitchRadians_)};

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
        const glm::dvec3 awayFromTarget = glm::normalize(
            cameraPositionMeters - targetPositionMeters);
        for (double retreatMeters = 1.0; retreatMeters <= 24.0; retreatMeters += 1.0)
        {
            const glm::dvec3 candidate = cameraPositionMeters
                + awayFromTarget * retreatMeters;
            if (!isInsideBuilding(candidate))
            {
                cameraPositionMeters = candidate;
                break;
            }
        }
    }

    observerCamera_->SetPosition(ToRenderPosition(cameraPositionMeters));
    observerCamera_->SetTarget(ToRenderPosition(targetPositionMeters)
        + glm::vec3{0.0F, 0.04F, 0.0F});
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
