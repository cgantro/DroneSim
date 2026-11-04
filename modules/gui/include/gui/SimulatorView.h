#pragma once

#include "flight_planning/BuildingObstacle.h"
#include "gui/ObserverCameraController.h"
#include "simulation_core/SimulationCore.h"

#include <functional>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

namespace dronesim::graphics
{
class Camera;
class MultisampleFramebuffer;
class Renderer;
}

namespace dronesim::simulator
{
class DroneSimAppTestAccess;
}

namespace dronesim::gui
{
/** @brief 장면·기체 카메라와 패널용 렌더 타깃의 생명주기를 맡는다. */
class SimulatorView final
{
public:
    using DrawWorld = std::function<void(const glm::mat4&, const glm::mat4&,
        const glm::vec3&, bool, std::optional<simulation::DroneId>)>;

    SimulatorView(graphics::Renderer& renderer, int width, int height);
    ~SimulatorView();

    /** @brief 관찰 패널 크기에 맞춰 카메라를 갱신하고 장면 텍스처를 돌려준다. */
    [[nodiscard]] std::uint32_t RenderScenePanel(
        const simulation::SimulationCore& simulation,
        const std::vector<flight_planning::BuildingObstacle>& buildings,
        int width, int height,
        const DrawWorld& drawWorld,
        double* renderMicroseconds);

    /** @brief 선택한 드론의 최신 EO 화면을 렌더링하고 텍스처를 돌려준다. */
    [[nodiscard]] std::uint32_t RenderCameraPanel(
        const simulation::SimulationCore& simulation,
        simulation::DroneId droneId,
        const DrawWorld& drawWorld,
        double* renderMicroseconds);

    /** @brief 장면 패널에서 조작하는 관찰 카메라 제어기를 돌려준다. */
    [[nodiscard]] ObserverCameraController& SceneCamera() noexcept;

    /** @brief 장면·카메라 패널이 함께 사용할 드론을 반환한다. 0은 주 드론을 뜻한다. */
    [[nodiscard]] simulation::DroneId ObserverDroneId() const noexcept;
    /** @brief 장면과 카메라 패널의 선택 드론을 함께 바꾼다. */
    void SetObserverDroneId(simulation::DroneId droneId) noexcept;

private:
    friend class simulator::DroneSimAppTestAccess;

    struct CameraView
    {
        std::unique_ptr<graphics::Camera> camera;
        std::unique_ptr<graphics::MultisampleFramebuffer> target;
        std::optional<std::uint64_t> lastRenderedTick;
    };

    static void UpdateEoCamera(const simulation::SimulationCore& simulation,
        simulation::DroneId droneId, graphics::Camera& camera,
        int width, int height);
    void DrawToTarget(graphics::MultisampleFramebuffer& target,
        const graphics::Camera& camera, bool showMissionGuides,
        std::optional<simulation::DroneId> hiddenDroneId,
        const DrawWorld& drawWorld);

    graphics::Renderer& renderer_;
    ObserverCameraController sceneCamera_;
    simulation::DroneId observerDroneId_ = 0;
    std::unique_ptr<graphics::MultisampleFramebuffer> sceneTarget_;
    std::unordered_map<simulation::DroneId, CameraView> cameraViews_;
};
}
