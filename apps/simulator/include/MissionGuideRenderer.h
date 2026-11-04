#pragma once

#include "FleetMission.h"
#include "FleetMissionPlanner.h"
#include "DroneSceneRenderContext.h"
#include "graphics/Material.h"

#include <glm/glm.hpp>

#include <memory>
#include <span>

namespace dronesim::graphics
{
class Mesh;
class Renderer;
class Shader;
}

namespace dronesim::simulator
{
/**
 * @brief 현재 임무의 이동 경로와 정찰 목표를 장면에 그린다.
 * @details 임무 상태는 읽기만 하며, 경로 모양과 표식의 렌더링만 맡는다.
 */
class MissionGuideRenderer final
{
public:
    MissionGuideRenderer();
    ~MissionGuideRenderer();

    /** @brief 경로 선분과 목표 표식에 사용할 메시를 준비한다. */
    void LoadModels();

    /** @brief 비행 중인 드론의 경로와 도착 목표를 현재 카메라 시점에 맞춰 그린다. */
    void Draw(
        const mission::FleetMissionState& mission,
        std::span<const mission::FleetReconnaissanceTarget> targets,
        const DroneSceneRenderContext& context) const;

private:
    void DrawRouteSegment(
        const glm::dvec3& startMeters,
        const glm::dvec3& endMeters,
        const DroneSceneRenderContext& context) const;
    void DrawMarker(
        const glm::dvec3& positionMeters,
        const DroneSceneRenderContext& context) const;

    std::unique_ptr<dronesim::graphics::Mesh> markerMesh_;
    dronesim::graphics::Material routeMaterial_{
        glm::vec4{0.12F, 0.82F, 0.92F, 1.0F}, 0.0F, 0.48F};
};
}
