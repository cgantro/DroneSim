#pragma once

#include "flight_planning/BuildingObstacle.h"
#include "graphics/Camera.h"
#include "simulation_core/SimulationCore.h"

#include <vector>

namespace dronesim::gui
{
/** @brief 드론을 따라가는 관찰 카메라의 시점과 조작을 맡는다. */
class ObserverCameraController final
{
public:
    ObserverCameraController(int width, int height);

    /** @brief 장면 카메라가 바라볼 드론과 장애물에 맞춰 위치와 화면 비율을 갱신한다. */
    void Update(const simulation::SimulationCore& simulation,
        simulation::DroneId observerDroneId,
        const std::vector<flight_planning::BuildingObstacle>& buildings,
        int width, int height);

    /** @brief 휠 한 칸 단위로 드론과 카메라 사이 거리를 바꾼다. 양수는 가까워진다. */
    void Zoom(double wheelSteps) noexcept;

    /** @brief 마우스 이동량에 따라 드론 둘레로 카메라를 돌린다. */
    void Rotate(double deltaX, double deltaY) noexcept;

    /** @brief 화면 기준으로 카메라가 바라보는 점을 옮긴다. 이동량은 화면 픽셀이다. */
    void Pan(double deltaX, double deltaY, double viewportHeightPixels) noexcept;

    /** @brief 거리, 방향, 이동량을 처음 시점으로 되돌린다. */
    void Reset() noexcept;

    [[nodiscard]] const graphics::Camera& ViewCamera() const noexcept;

private:
    graphics::Camera camera_;
    double distanceMeters_;
    double orbitYawRadians_ = 0.0;
    double orbitPitchRadians_ = 0.0;
    glm::dvec3 targetOffsetMeters_{0.0};
};
}
