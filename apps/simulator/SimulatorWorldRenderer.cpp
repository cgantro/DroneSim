#include "SimulatorWorldRenderer.h"

#include "graphics/CoordinateTransform.h"
#include "graphics/GlbModel.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Texture2D.h"
#include "graphics/X500GimbalAssembly.h"
#include "simulation_core/SimulationCore.h"

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

namespace dronesim::simulator
{
SimulatorWorldRenderer::SimulatorWorldRenderer() = default;
SimulatorWorldRenderer::~SimulatorWorldRenderer() = default;

void SimulatorWorldRenderer::LoadModels()
{
    missionGuideRenderer_.LoadModels();
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
        missionGuideRenderer_.Draw(mission, targets, context);

    DrawDrones(simulation, context);
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

}
