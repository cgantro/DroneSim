#include "MissionGuideRenderer.h"

#include "SimulatorWorldRenderer.h"

#include "graphics/CoordinateTransform.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace dronesim::simulator
{
MissionGuideRenderer::MissionGuideRenderer() = default;
MissionGuideRenderer::~MissionGuideRenderer() = default;

void MissionGuideRenderer::LoadModels()
{
    markerMesh_ = dronesim::graphics::Mesh::CreateCube(1.0F);
}

void MissionGuideRenderer::Draw(
    const mission::FleetMissionState& mission,
    std::span<const mission::FleetReconnaissanceTarget> targets,
    const DroneSceneRenderContext& context) const
{
    mission.ForEachAssignment([&](const auto, const auto& assignment) {
        const auto status = assignment.Status();
        if (status != mission::FleetDroneStatus::TakeoffReady
            && status != mission::FleetDroneStatus::Reconnaissance
            && status != mission::FleetDroneStatus::Landing)
            return;

        glm::dvec3 previousPosition = assignment.StartPositionMeters();
        for (const glm::dvec3& waypoint : assignment.RouteWaypoints())
        {
            DrawRouteSegment(previousPosition, waypoint, context);
            previousPosition = waypoint;
        }

        const auto& target = targets[assignment.TargetIndex()];
        DrawRouteSegment(previousPosition, target.approachPositionMeters, context);
        DrawMarker(target.approachPositionMeters, context);
    });
}

void MissionGuideRenderer::DrawRouteSegment(
    const glm::dvec3& startMeters,
    const glm::dvec3& endMeters,
    const DroneSceneRenderContext& context) const
{
    const glm::vec3 start = dronesim::graphics::EnuPositionToRender(startMeters);
    const glm::vec3 end = dronesim::graphics::EnuPositionToRender(endMeters);
    const glm::vec3 direction = end - start;
    const float length = glm::length(direction);
    if (length < 0.1F)
        return;

    // 큐브의 Y축을 경로 방향으로 놓고, 나머지 두 축은 이에 수직인 기저로 만든다.
    const glm::vec3 localY = direction / length;
    // 경로가 월드 Y축과 거의 나란하면 외적이 0에 가까워지므로 X축을 대신 쓴다.
    const glm::vec3 referenceAxis = std::abs(localY.y) < 0.99F
        ? glm::vec3{0.0F, 1.0F, 0.0F}
        : glm::vec3{1.0F, 0.0F, 0.0F};
    const glm::vec3 localX = glm::normalize(glm::cross(referenceAxis, localY));
    const glm::vec3 localZ = glm::cross(localX, localY);
    glm::mat4 orientation{1.0F};
    orientation[0] = glm::vec4{localX, 0.0F};
    orientation[1] = glm::vec4{localY, 0.0F};
    orientation[2] = glm::vec4{localZ, 0.0F};

    constexpr float routeThicknessMeters = 0.12F;
    const glm::mat4 transform = glm::translate(
        glm::mat4{1.0F}, (start + end) * 0.5F)
        * orientation
        * glm::scale(glm::mat4{1.0F},
            glm::vec3{routeThicknessMeters, length, routeThicknessMeters});
    context.renderer.Draw(transform, *markerMesh_, routeMaterial_, context.shader,
        context.view, context.projection, context.cameraPosition);
}

void MissionGuideRenderer::DrawMarker(
    const glm::dvec3& positionMeters,
    const DroneSceneRenderContext& context) const
{
    constexpr float markerSizeMeters = 0.65F;
    const glm::vec3 renderPosition =
        dronesim::graphics::EnuPositionToRender(positionMeters);
    const glm::mat4 transform = glm::translate(glm::mat4{1.0F}, renderPosition)
        * glm::scale(glm::mat4{1.0F}, glm::vec3{markerSizeMeters});
    context.renderer.Draw(transform, *markerMesh_, routeMaterial_, context.shader,
        context.view, context.projection, context.cameraPosition);
}
}
