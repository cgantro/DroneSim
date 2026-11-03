#pragma once

#include "FleetMission.h"
#include "FleetMissionPlanner.h"
#include "TrainingRangeScene.h"
#include "graphics/Material.h"
#include "simulation_core/SimulationCore.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace dronesim::graphics
{
class GlbModel;
class Mesh;
class Renderer;
class Shader;
class Texture2D;
class X500GimbalAssembly;
}

namespace dronesim::simulator
{
/** @brief 한 화면 시점에서 3D 장면을 그리는 데 필요한 렌더 상태다. */
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

/**
 * @brief 훈련장, 임무 경로와 드론 모델을 그리는 역할을 맡는다.
 * @details 비행 상태나 임무를 변경하지 않고 현재 상태를 화면 형상으로 바꾼다.
 */
class SimulatorWorldRenderer final
{
public:
    SimulatorWorldRenderer();
    ~SimulatorWorldRenderer();

    /** @brief 드론과 경로를 그리는 데 필요한 메시와 재질을 읽는다. */
    void LoadModels();

    /** @brief 주어진 시뮬레이션 상태를 지정한 카메라 시점으로 그린다. */
    void Draw(
        const scene::TrainingRangeScene& trainingScene,
        const simulation::SimulationCore& simulation,
        const mission::FleetMissionState& mission,
        std::span<const mission::FleetReconnaissanceTarget> targets,
        const DroneSceneRenderContext& context) const;

private:
    void DrawMissionGuides(
        const mission::FleetMissionState& mission,
        std::span<const mission::FleetReconnaissanceTarget> targets,
        const DroneSceneRenderContext& context) const;
    void DrawDrones(
        const simulation::SimulationCore& simulation,
        const DroneSceneRenderContext& context) const;
    void DrawRouteSegment(
        const glm::dvec3& startMeters,
        const glm::dvec3& endMeters,
        const DroneSceneRenderContext& context) const;
    void DrawCube(
        const glm::mat4& transform,
        const dronesim::graphics::Material& material,
        const DroneSceneRenderContext& context) const;

    std::unique_ptr<dronesim::graphics::Mesh> routeMarkerMesh_;
    std::unique_ptr<dronesim::graphics::GlbModel> droneModel_;
    std::unique_ptr<dronesim::graphics::X500GimbalAssembly> droneGimbalAssembly_;
    std::unique_ptr<dronesim::graphics::Texture2D> droneSurfaceTexture_;
    dronesim::graphics::Material droneMaterial_{glm::vec4{1.0F}, 0.16F, 0.40F};
    dronesim::graphics::Material missionRouteMaterial_{
        glm::vec4{0.12F, 0.82F, 0.92F, 1.0F}, 0.0F, 0.48F};
};
}
