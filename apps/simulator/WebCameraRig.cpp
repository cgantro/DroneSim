#include "WebCameraRig.h"

#include "graphics/CoordinateTransform.h"
#include "simulation_core/EoCameraModel.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>

namespace dronesim::simulator
{
WebCameraRig::WebCameraRig(float cameraFieldOfViewDegrees)
    : observerCamera_(glm::vec3{0.0F, 145.0F, 285.0F},
        glm::vec3{0.0F, 12.0F, 0.0F}, 1.0F, 45.0F, 0.1F, 2000.0F)
    , eoCamera_(glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F},
        1.0F, cameraFieldOfViewDegrees, 0.05F, 200.0F)
{
}

void WebCameraRig::UpdateObserver(
    const simulation::SimulationCore& simulation,
    simulation::DroneId observerDroneId,
    const std::vector<flight_planning::BuildingObstacle>& buildings,
    int width, int height)
{
    const auto& state = observerDroneId == 0U
        ? simulation.State()
        : simulation.State(observerDroneId);
    const glm::vec3 dronePosition =
        graphics::EnuPositionToRender(state.positionMeters);
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    const glm::dvec3 forwardMeters{
        std::cos(state.yawRadians), std::sin(state.yawRadians), 0.0};
    constexpr double followDistanceMeters = 12.0;
    constexpr double followHeightMeters = 3.5;
    glm::dvec3 observerPositionMeters = state.positionMeters
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

    // 기본 추적 위치가 건물 안이면 같은 높이에서 뒤로 물러나 시야를 확보한다.
    if (isInsideBuilding(observerPositionMeters))
    {
        for (double retreatMeters = 1.0; retreatMeters <= 24.0; retreatMeters += 1.0)
        {
            const glm::dvec3 candidate = observerPositionMeters
                - forwardMeters * retreatMeters;
            if (!isInsideBuilding(candidate))
            {
                observerPositionMeters = candidate;
                break;
            }
        }
    }

    observerCamera_.SetPosition(graphics::EnuPositionToRender(observerPositionMeters));
    observerCamera_.SetTarget(dronePosition + up * 0.04F);
    observerCamera_.SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    observerCamera_.SetFieldOfViewDegrees(45.0F);
}

void WebCameraRig::UpdateEo(
    const simulation::SimulationCore& simulation,
    simulation::DroneId droneId, int width, int height)
{
    const auto& cameraState = simulation.Camera(droneId);
    const glm::vec3 cameraPosition =
        graphics::EnuPositionToRender(cameraState.positionMeters);
    const glm::vec3 viewDirection = graphics::EnuDirectionToRender(
        simulation::EoCameraModel::OpticalAxis(cameraState));
    const glm::vec3 cameraUp = graphics::EnuDirectionToRender(
        simulation::EoCameraModel::UpAxis(cameraState));

    eoCamera_.SetPosition(cameraPosition);
    eoCamera_.SetTarget(cameraPosition + viewDirection);
    eoCamera_.SetUp(cameraUp);
    eoCamera_.SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    eoCamera_.SetFieldOfViewDegrees(
        static_cast<float>(cameraState.model.verticalFieldOfViewDegrees));
}

const graphics::Camera& WebCameraRig::ObserverCamera() const noexcept
{
    return observerCamera_;
}

const graphics::Camera& WebCameraRig::EoCamera() const noexcept
{
    return eoCamera_;
}
}
