#pragma once

#include "gui/DroneStatusView.h"
#include "flight_planning/BuildingObstacle.h"

#include <span>
#include <vector>

namespace dronesim::gui
{
/** @brief 미니맵에 표시할 드론 한 대의 계획 경로다. */
struct MiniMapRoute
{
    simulation::DroneId droneId = 0;
    glm::dvec3 startMeters{0.0};
    std::vector<glm::dvec3> waypointsMeters;
};

/** @brief 한 프레임에 미니맵이 그릴 구조물·드론·경로 자료다. */
struct MiniMapPanelFrame
{
    std::span<const flight_planning::BuildingObstacle> buildings;
    std::span<const DroneStatusView> drones;
    std::span<const MiniMapRoute> routes;
    glm::dvec2 groundSizeMeters{0.0};
};

/** @brief 건물 높이, 드론 위치, 경로를 평면 지도에 그린다. */
class MiniMapPanel
{
public:
    void Render(const MiniMapPanelFrame& frame);

private:
    double zoom_ = 1.0;
    glm::dvec2 centerMeters_{0.0};
};
}
