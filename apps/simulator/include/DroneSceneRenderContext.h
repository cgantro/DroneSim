#pragma once

#include "simulation_core/SimulationCore.h"

#include <glm/glm.hpp>

#include <optional>

namespace dronesim::graphics
{
class Renderer;
class Shader;
}

namespace dronesim::simulator
{
/** @brief 한 시점에서 3D 장면을 그리는 데 필요한 화면·카메라 정보다. */
struct DroneSceneRenderContext
{
    dronesim::graphics::Renderer& renderer;
    dronesim::graphics::Shader& shader;
    const glm::mat4& view;
    const glm::mat4& projection;
    const glm::vec3& cameraPosition;
    bool showMissionGuides = false;
    std::optional<dronesim::simulation::DroneId> hiddenDroneId;
};
}
