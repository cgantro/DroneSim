#pragma once

#include "flight_planning/BuildingObstacle.h"
#include "graphics/Camera.h"
#include "simulation_core/SimulationCore.h"

#include <vector>

namespace dronesim::simulator
{
/** @brief 웹 화면의 관찰 시점과 드론 EO 시점을 계산한다. */
class WebCameraRig final
{
public:
    explicit WebCameraRig(float cameraFieldOfViewDegrees);

    /** @brief 선택한 드론을 따라가며 건물 안에 들어가지 않는 관찰 시점을 갱신한다. */
    void UpdateObserver(const simulation::SimulationCore& simulation,
        simulation::DroneId observerDroneId,
        const std::vector<flight_planning::BuildingObstacle>& buildings,
        int width, int height);

    /** @brief 드론 카메라의 위치·방향·화면 비율을 EO 센서 상태와 맞춘다. */
    void UpdateEo(const simulation::SimulationCore& simulation,
        simulation::DroneId droneId, int width, int height);

    [[nodiscard]] const graphics::Camera& ObserverCamera() const noexcept;
    [[nodiscard]] const graphics::Camera& EoCamera() const noexcept;

private:
    graphics::Camera observerCamera_;
    graphics::Camera eoCamera_;
};
}
