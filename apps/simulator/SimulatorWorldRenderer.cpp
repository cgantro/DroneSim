#include "SimulatorWorldRenderer.h"

#include "graphics/CoordinateTransform.h"
#include "graphics/GlbModel.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Texture2D.h"
#include "graphics/X500GimbalAssembly.h"
#include "simulation_core/SimulationCore.h"

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

namespace dronesim::simulator
{
SimulatorWorldRenderer::SimulatorWorldRenderer() = default;
SimulatorWorldRenderer::~SimulatorWorldRenderer() = default;

void SimulatorWorldRenderer::LoadModels()
{
    routeMarkerMesh_ = dronesim::graphics::Mesh::CreateCube(1.0F);
    droneModel_ = dronesim::graphics::GlbModel::Load(
        "models/drones/x500/x500_CAM.glb");
    droneGimbalAssembly_ = std::make_unique<dronesim::graphics::X500GimbalAssembly>(
        *droneModel_);
    droneSurfaceTexture_ = dronesim::graphics::Texture2D::LoadFromFile(
        "models/drones/x500/carbon-weave.ppm");
}

void SimulatorWorldRenderer::Draw(
    const scene::TrainingRangeScene& trainingScene,
    const simulation::SimulationCore& simulation,
    const mission::FleetMissionState& mission,
    std::span<const mission::FleetReconnaissanceTarget> targets,
    const DroneSceneRenderContext& context) const
{
    context.renderer.SetAnimationTimeSeconds(
        static_cast<float>(simulation.SimulationTimeSeconds()));
    trainingScene.Draw(context.renderer, context.shader, context.view,
        context.projection, context.cameraPosition);

    if (context.showMissionGuides)
        DrawMissionGuides(mission, targets, context);

    DrawDrones(simulation, context);
}

void SimulatorWorldRenderer::DrawMissionGuides(
    const mission::FleetMissionState& mission,
    std::span<const mission::FleetReconnaissanceTarget> targets,
    const DroneSceneRenderContext& context) const
{
    mission.ForEachAssignment([&](const auto droneId, const auto& assignment) {
        static_cast<void>(droneId);
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

        const glm::dvec3 targetPosition =
            targets[assignment.TargetIndex()].approachPositionMeters;
        DrawRouteSegment(previousPosition, targetPosition, context);
        const glm::vec3 marker = dronesim::graphics::EnuPositionToRender(targetPosition);
        DrawCube(glm::translate(glm::mat4{1.0F}, marker)
            * glm::scale(glm::mat4{1.0F}, glm::vec3{0.65F}),
            missionRouteMaterial_, context);
    });
}

void SimulatorWorldRenderer::DrawDrones(
    const simulation::SimulationCore& simulation,
    const DroneSceneRenderContext& context) const
{
    for (const simulation::DroneId droneId : simulation.DroneIds())
    {
        if (context.hiddenDroneId == droneId)
            continue;

        const auto& drone = simulation.State(droneId);
        const glm::vec3 position =
            dronesim::graphics::EnuPositionToRender(drone.positionMeters);
        const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position)
            * glm::mat4_cast(
                dronesim::graphics::EnuOrientationToRender(drone.orientation));

        // 짐벌 관절은 기체 GLB의 축점을 기준으로 돌리고, 기체 자세는 바깥 변환으로 적용한다.
        const auto& gimbal = simulation.Camera(droneId).gimbalAngles;
        const auto partTransforms = droneGimbalAssembly_->PartTransforms({
            gimbal.yawDegrees, gimbal.rollDegrees, gimbal.pitchUpDegrees});
        droneModel_->Draw(heading, context.renderer, context.shader,
            context.view, context.projection, context.cameraPosition,
            &droneMaterial_, droneSurfaceTexture_.get(), true, partTransforms);
    }
}

void SimulatorWorldRenderer::DrawRouteSegment(
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

    const glm::vec3 localY = direction / length;
    const glm::vec3 referenceAxis = std::abs(localY.y) < 0.99F
        ? glm::vec3{0.0F, 1.0F, 0.0F}
        : glm::vec3{1.0F, 0.0F, 0.0F};
    const glm::vec3 localX = glm::normalize(glm::cross(referenceAxis, localY));
    const glm::vec3 localZ = glm::cross(localX, localY);
    glm::mat4 orientation{1.0F};
    orientation[0] = glm::vec4{localX, 0.0F};
    orientation[1] = glm::vec4{localY, 0.0F};
    orientation[2] = glm::vec4{localZ, 0.0F};

    // 기본 큐브의 Y축을 경로 방향에 맞추고 두 점의 중간에 놓아 선분을 표현한다.
    constexpr float routeThicknessMeters = 0.12F;
    const glm::mat4 transform = glm::translate(
        glm::mat4{1.0F}, (start + end) * 0.5F)
        * orientation
        * glm::scale(glm::mat4{1.0F},
            glm::vec3{routeThicknessMeters, length, routeThicknessMeters});
    DrawCube(transform, missionRouteMaterial_, context);
}

void SimulatorWorldRenderer::DrawCube(
    const glm::mat4& transform,
    const dronesim::graphics::Material& material,
    const DroneSceneRenderContext& context) const
{
    context.renderer.Draw(transform, *routeMarkerMesh_, material,
        context.shader, context.view, context.projection, context.cameraPosition);
}
}
