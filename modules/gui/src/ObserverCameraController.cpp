#include "gui/ObserverCameraController.h"

#include "graphics/CoordinateTransform.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace dronesim::gui
{
namespace
{
constexpr double initialDistanceMeters = 12.5;
constexpr double minimumDistanceMeters = 2.5;
constexpr double maximumDistanceMeters = 100.0;
constexpr double zoomStep = 1.18;
constexpr double radiansPerPixel = 0.006;
constexpr double minimumPitchRadians = 0.14;
constexpr double maximumPitchRadians = 1.30;
constexpr double verticalFieldOfViewRadians = std::numbers::pi_v<double> / 4.0;
}

ObserverCameraController::ObserverCameraController(int width, int height)
    : camera_(glm::vec3{0.0F, 145.0F, 285.0F}, glm::vec3{0.0F, 12.0F, 0.0F},
          static_cast<float>(width) / static_cast<float>(height),
          45.0F, 0.1F, 2000.0F),
      distanceMeters_(initialDistanceMeters),
      orbitPitchRadians_(std::atan2(3.5, 12.0))
{
}

void ObserverCameraController::Update(
    const simulation::SimulationCore& simulation,
    simulation::DroneId observerDroneId,
    const std::vector<flight_planning::BuildingObstacle>& buildings,
    int width, int height)
{
    const auto& state = observerDroneId == 0U
        ? simulation.State() : simulation.State(observerDroneId);
    const double cameraYaw = state.yawRadians + std::numbers::pi_v<double>
        + orbitYawRadians_;
    const double horizontalDistanceMeters = distanceMeters_
        * std::cos(orbitPitchRadians_);
    const glm::dvec3 targetPositionMeters = state.positionMeters
        + targetOffsetMeters_;
    glm::dvec3 cameraPositionMeters = targetPositionMeters
        + glm::dvec3{
            horizontalDistanceMeters * std::cos(cameraYaw),
            horizontalDistanceMeters * std::sin(cameraYaw),
            distanceMeters_ * std::sin(orbitPitchRadians_)};

    if (flight_planning::ContainsAnyBuildingVolume(buildings, cameraPositionMeters))
    {
        const glm::dvec3 awayFromTarget = glm::normalize(
            cameraPositionMeters - targetPositionMeters);
        for (double retreatMeters = 1.0; retreatMeters <= 24.0; retreatMeters += 1.0)
        {
            const glm::dvec3 candidate = cameraPositionMeters
                + awayFromTarget * retreatMeters;
            if (!flight_planning::ContainsAnyBuildingVolume(buildings, candidate))
            {
                cameraPositionMeters = candidate;
                break;
            }
        }
    }

    camera_.SetPosition(graphics::EnuPositionToRender(cameraPositionMeters));
    camera_.SetTarget(graphics::EnuPositionToRender(targetPositionMeters)
        + glm::vec3{0.0F, 0.04F, 0.0F});
    camera_.SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
}

void ObserverCameraController::Zoom(double wheelSteps) noexcept
{
    if (!std::isfinite(wheelSteps) || wheelSteps == 0.0)
        return;

    distanceMeters_ = std::clamp(distanceMeters_ * std::pow(zoomStep, -wheelSteps),
        minimumDistanceMeters, maximumDistanceMeters);
}

void ObserverCameraController::Rotate(double deltaX, double deltaY) noexcept
{
    if (!std::isfinite(deltaX) || !std::isfinite(deltaY))
        return;

    // 마우스 픽셀 이동량을 각도로 바꿔 카메라의 수평·수직 궤도 각도를 조정한다.
    orbitYawRadians_ = std::remainder(
        orbitYawRadians_ + deltaX * radiansPerPixel,
        2.0 * std::numbers::pi_v<double>);
    // 카메라가 위아래로 뒤집히지 않도록 수직 각도를 안전한 범위에 둔다.
    orbitPitchRadians_ = std::clamp(orbitPitchRadians_ - deltaY * radiansPerPixel,
        minimumPitchRadians, maximumPitchRadians);
}

void ObserverCameraController::Pan(
    double deltaX, double deltaY, double viewportHeightPixels) noexcept
{
    if (!std::isfinite(deltaX) || !std::isfinite(deltaY)
        || !std::isfinite(viewportHeightPixels) || viewportHeightPixels <= 0.0)
        return;

    const double cameraYaw = orbitYawRadians_ + std::numbers::pi_v<double>;
    const double sinYaw = std::sin(cameraYaw);
    const double cosYaw = std::cos(cameraYaw);
    const double sinPitch = std::sin(orbitPitchRadians_);
    const double cosPitch = std::cos(orbitPitchRadians_);
    const glm::dvec3 screenRight{-sinYaw, cosYaw, 0.0};
    const glm::dvec3 screenUp{
        -sinPitch * cosYaw, -sinPitch * sinYaw, cosPitch};
    // 시야각과 카메라 거리로 화면 픽셀 하나가 나타내는 월드 길이를 환산한다.
    const double metersPerPixel = 2.0 * distanceMeters_
        * std::tan(verticalFieldOfViewRadians * 0.5)
        / viewportHeightPixels;

    // 드래그한 손의 방향과 장면 이동 방향을 같게 보이도록 목표점을 반대로 옮긴다.
    targetOffsetMeters_ += screenRight * (-deltaX * metersPerPixel)
        + screenUp * (deltaY * metersPerPixel);
}

void ObserverCameraController::Reset() noexcept
{
    distanceMeters_ = initialDistanceMeters;
    orbitYawRadians_ = 0.0;
    orbitPitchRadians_ = std::atan2(3.5, 12.0);
    targetOffsetMeters_ = glm::dvec3{0.0};
}

const graphics::Camera& ObserverCameraController::ViewCamera() const noexcept
{
    return camera_;
}

}
