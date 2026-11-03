#include "gui/SimulatorHud.h"
#include "gui/SimulatorView.h"
#include "gui/GuiText.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <algorithm>
#include <string>

namespace dronesim::gui
{
namespace
{
void CreateDefaultDockspace(ImGuiViewport* viewport)
{
    const ImGuiID dockspaceId = ImGui::GetID("DroneSimDockspaceV5");
    if (ImGui::DockBuilderGetNode(dockspaceId) == nullptr)
    {
        ImGui::DockBuilderAddNode(dockspaceId,
            ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->WorkSize);
        ImGuiID sceneNode = 0;
        ImGuiID sideNode = 0;
        ImGui::DockBuilderSplitNode(
            dockspaceId, ImGuiDir_Right, 0.34F, &sideNode, &sceneNode);

        // 장면을 크게 남기고, 나머지 창은 탭으로 전환하거나 바깥으로 떼어 쓴다.
        ImGui::DockBuilderDockWindow("장면##SceneViewV5", sceneNode);
        ImGui::DockBuilderDockWindow("운용##OperationsV5", sideNode);
        ImGui::DockBuilderDockWindow("카메라##CameraViewV5", sideNode);
        ImGui::DockBuilderDockWindow("지도##MapViewV5", sideNode);
        ImGui::DockBuilderFinish(dockspaceId);
    }
    ImGui::DockSpaceOverViewport(dockspaceId, viewport);
}

void DrawScenePanel(const simulation::SimulationCore& simulation,
    simulation::DroneId& observerDroneId,
    SimulatorView& sceneView,
    const std::function<std::uint32_t(int, int)>& renderScene)
{
    if (ImGui::Begin("장면##SceneViewV5"))
    {
        const auto droneIds = simulation.DroneIds();
        if (!droneIds.empty())
        {
            if (observerDroneId == 0U)
                observerDroneId = simulation.PrimaryDroneId();

            const std::string selectedName = DroneLabel(observerDroneId);
            if (ImGui::BeginCombo("장면 시점 드론", selectedName.c_str()))
            {
                for (const simulation::DroneId droneId : droneIds)
                {
                    const std::string name = DroneLabel(droneId);
                    const bool selected = droneId == observerDroneId;
                    if (ImGui::Selectable(name.c_str(), selected))
                        observerDroneId = droneId;
                    if (selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        }

        ImGui::TextDisabled("왼쪽 끌기 회전 · 가운데 끌기 이동 · 휠 확대");
        ImGui::SameLine();
        if (ImGui::SmallButton("-"))
            sceneView.ZoomObserverCamera(-1.0);
        ImGui::SameLine();
        if (ImGui::SmallButton("+"))
            sceneView.ZoomObserverCamera(1.0);
        ImGui::SameLine();
        if (ImGui::SmallButton("시점 초기화"))
            sceneView.ResetObserverCamera();

        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)
            && !ImGui::GetIO().WantTextInput)
        {
            if (ImGui::IsKeyPressed(ImGuiKey_Equal)
                || ImGui::IsKeyPressed(ImGuiKey_KeypadAdd))
                sceneView.ZoomObserverCamera(1.0);
            if (ImGui::IsKeyPressed(ImGuiKey_Minus)
                || ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract))
                sceneView.ZoomObserverCamera(-1.0);
            if (ImGui::IsKeyPressed(ImGuiKey_0)
                || ImGui::IsKeyPressed(ImGuiKey_Keypad0))
                sceneView.ResetObserverCamera();
        }

        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 imageMinimum = ImGui::GetCursorScreenPos();
        const ImVec2 imageMaximum{
            imageMinimum.x + available.x, imageMinimum.y + available.y};
        const ImGuiIO& io = ImGui::GetIO();
        const bool pointerOverScene = ImGui::IsWindowHovered()
            && ImGui::IsMouseHoveringRect(imageMinimum, imageMaximum);
        if (pointerOverScene && io.MouseWheel != 0.0F)
            sceneView.ZoomObserverCamera(static_cast<double>(io.MouseWheel));
        if (pointerOverScene && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
            sceneView.RotateObserverCamera(io.MouseDelta.x, io.MouseDelta.y);
        if (pointerOverScene && ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
        {
            sceneView.PanObserverCamera(
                io.MouseDelta.x, io.MouseDelta.y, available.y);
        }

        const ImVec2 imageSize{
            std::max(1.0F, available.x), std::max(1.0F, available.y)};
        const std::uint32_t texture = renderScene(
            static_cast<int>(imageSize.x), static_cast<int>(imageSize.y));
        ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
            ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
    }
    ImGui::End();
}

void DrawCameraPanel(
    const simulation::SimulationCore& simulation,
    simulation::DroneId& observerDroneId,
    const std::function<std::uint32_t(simulation::DroneId)>& renderCamera,
    const std::function<void(simulation::DroneId,
        const simulation::CameraGimbalAngles&)>& setCameraGimbalAngles)
{
    if (ImGui::Begin("카메라##CameraViewV5"))
    {
        const auto droneIds = simulation.DroneIds();
        if (droneIds.empty())
        {
            ImGui::TextUnformatted("드론이 없습니다.");
        }
        else
        {
            if (std::find(droneIds.begin(), droneIds.end(), observerDroneId)
                == droneIds.end())
                observerDroneId = simulation.PrimaryDroneId();

            const std::string selectedLabel = DroneLabel(observerDroneId);
            if (ImGui::BeginCombo("선택한 드론", selectedLabel.c_str()))
            {
                for (const simulation::DroneId droneId : droneIds)
                {
                    const std::string label = DroneLabel(droneId);
                    const bool selected = droneId == observerDroneId;
                    if (ImGui::Selectable(label.c_str(), selected))
                        observerDroneId = droneId;
                    if (selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            const auto& camera = simulation.Camera(observerDroneId);
            ImGui::SameLine();
            ImGui::TextDisabled("%s", simulation::ToString(camera.status).data());

            float yawDegrees = static_cast<float>(camera.gimbalAngles.yawDegrees);
            float pitchUpDegrees = static_cast<float>(camera.gimbalAngles.pitchUpDegrees);
            float rollDegrees = static_cast<float>(camera.gimbalAngles.rollDegrees);
            bool anglesChanged = false;
            anglesChanged |= ImGui::SliderFloat("좌우", &yawDegrees,
                -180.0F, 180.0F, "%.0f°");
            anglesChanged |= ImGui::SliderFloat("상하", &pitchUpDegrees,
                -45.0F, 135.0F, "%.0f°");
            anglesChanged |= ImGui::SliderFloat("기울기", &rollDegrees,
                -45.0F, 45.0F, "%.0f°");
            if (ImGui::Button("중립 위치"))
            {
                yawDegrees = 0.0F;
                pitchUpDegrees = 0.0F;
                rollDegrees = 0.0F;
                anglesChanged = true;
            }
            if (anglesChanged)
            {
                setCameraGimbalAngles(observerDroneId,
                    {yawDegrees, pitchUpDegrees, rollDegrees});
            }

            if (camera.status != simulation::CameraStatus::Active)
            {
                ImGui::TextDisabled("카메라 영상을 표시할 수 없습니다.");
            }
            else
            {
                const int imageWidth = camera.model.imageWidthPixels;
                const int imageHeight = camera.model.imageHeightPixels;
                const ImVec2 available = ImGui::GetContentRegionAvail();
                const float scale = std::min(
                    available.x / static_cast<float>(imageWidth),
                    available.y / static_cast<float>(imageHeight));
                const ImVec2 imageSize{
                    static_cast<float>(imageWidth) * scale,
                    static_cast<float>(imageHeight) * scale};
                ImGui::SetCursorPosX(ImGui::GetCursorPosX()
                    + std::max(0.0F, (available.x - imageSize.x) * 0.5F));
                const std::uint32_t texture = renderCamera(observerDroneId);
                ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
                    ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
                ImGui::TextDisabled("영상 크기 %d × %d", imageWidth, imageHeight);
            }
        }
    }
    ImGui::End();
}

}

SimulatorHud::SimulatorHud(GLFWwindow* window)
    : frameBackend_(window)
{
}

void SimulatorHud::Render(const SimulatorHudFrame& frame)
{
    const simulation::SimulationCore& simulation = frame.simulation;
    simulation::DroneId& observerDroneId = frame.observerDroneId;
    SimulatorView& sceneView = frame.sceneView;
    const bool gamepadConnected = frame.gamepadConnected;
    const bool fleetReturnCanStart = frame.fleetReturnCanStart;
    const auto& startFleetReturn = frame.startFleetReturn;
    const auto& completeFleetMissionStatus = frame.completeFleetMissionStatus;
    const bool completeFleetMissionCanStart = frame.completeFleetMissionCanStart;
    const auto& startCompleteFleetMission = frame.startCompleteFleetMission;
    const FleetNetworkStatus* fleetNetworkStatus = frame.fleetNetworkStatus;
    const auto& setFormationNodeRadioEnabled = frame.setFormationNodeRadioEnabled;
    const auto buildings = frame.buildings;
    const auto drones = frame.drones;
    const auto routes = frame.routes;
    const glm::dvec2& groundSizeMeters = frame.groundSizeMeters;
    const auto& renderScene = frame.renderScene;
    const auto& renderCamera = frame.renderCamera;
    const auto& setCameraGimbalAngles = frame.setCameraGimbalAngles;
    HudRenderTimings* timings = frame.timings;

    frameBackend_.BeginFrame(timings);
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    CreateDefaultDockspace(viewport);
    DrawScenePanel(simulation, observerDroneId, sceneView, renderScene);
    miniMapPanel_.Render(buildings, drones, routes, groundSizeMeters);
    DrawCameraPanel(simulation, observerDroneId, renderCamera, setCameraGimbalAngles);
    const OperationsPanelFrame operationsFrame{
        .simulation = simulation,
        .gamepadConnected = gamepadConnected,
        .fleetReturnCanStart = fleetReturnCanStart,
        .startFleetReturn = startFleetReturn,
        .completeFleetMissionStatus = completeFleetMissionStatus,
        .completeFleetMissionCanStart = completeFleetMissionCanStart,
        .startCompleteFleetMission = startCompleteFleetMission,
        .fleetNetworkStatus = fleetNetworkStatus,
        .setRadioEnabled = setFormationNodeRadioEnabled};
    operationsPanel_.Render(operationsFrame);

    frameBackend_.Draw(timings);
}
}
