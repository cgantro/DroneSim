#pragma once

#include "flight_planning/BuildingObstacle.h"
#include "simulation_core/SimulationCore.h"

#include <span>
#include <vector>

namespace dronesim::gui
{
/** @brief 미니맵에 표시할 드론 한 대의 현재 위치와 방향이다. */
struct MiniMapDrone
{
    simulation::DroneId droneId = 0;
    glm::dvec3 positionMeters{0.0};
    double yawRadians = 0.0;
};

/** @brief 미니맵에 표시할 드론 한 대의 계획 경로다. */
struct MiniMapRoute
{
    simulation::DroneId droneId = 0;
    glm::dvec3 startMeters{0.0};
    std::vector<glm::dvec3> waypointsMeters;
};

/** @brief 건물 높이, 드론 위치, 경로를 평면 지도에 그린다. */
class MiniMapPanel
{
public:
    void Render(
        std::span<const flight_planning::BuildingObstacle> buildings,
        std::span<const MiniMapDrone> drones,
        std::span<const MiniMapRoute> routes,
        const glm::dvec2& groundSizeMeters);

private:
    double zoom_ = 1.0;
    glm::dvec2 centerMeters_{0.0};
};
}
