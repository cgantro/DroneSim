#include "gui/SimulatorHud.h"
#include "graphics/GlApi.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include "simulation_core/QuadrotorModel.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <chrono>
#include <vector>

namespace dronesim::gui
{
namespace
{
void LoadWindowsKoreanFont()
{
    const char* windowsDirectory = std::getenv("WINDIR");
    if (windowsDirectory == nullptr)
        return;

    const std::filesystem::path fontPath =
        std::filesystem::path(windowsDirectory) / "Fonts" / "malgun.ttf";
    if (!std::filesystem::exists(fontPath))
        return;

    const std::string fontPathText = fontPath.string();
    ImGui::GetIO().Fonts->AddFontFromFileTTF(
        fontPathText.c_str(), 18.0F, nullptr, ImGui::GetIO().Fonts->GetGlyphRangesKorean());
}

std::string DroneName(simulation::DroneId id)
{
    return "드론 " + std::to_string(id);
}

std::string NetworkNodeName(communication::NetworkNodeId id)
{
    return DroneName(id);
}

const char* MeshEventLabel(communication::MeshEventKind kind)
{
    switch (kind)
    {
    case communication::MeshEventKind::NodeRadioDisabled: return "드론 라디오 차단";
    case communication::MeshEventKind::NodeRadioRestored: return "드론 라디오 복구";
    case communication::MeshEventKind::MessageDelivered: return "메시지 전달";
    case communication::MeshEventKind::PacketDropped: return "패킷 폐기";
    }
    return "통신 사건";
}

void CreateDefaultDockspace(ImGuiViewport* viewport)
{
    const ImGuiID dockspaceId = ImGui::GetID("DroneSimDockspaceV3");
    if (ImGui::DockBuilderGetNode(dockspaceId) == nullptr)
    {
        ImGui::DockBuilderAddNode(dockspaceId,
            ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->WorkSize);
        ImGuiID sceneNode = 0;
        ImGuiID sideNode = 0;
        ImGui::DockBuilderSplitNode(
            dockspaceId, ImGuiDir_Right, 0.30F, &sideNode, &sceneNode);

        ImGuiID cameraNode = 0;
        ImGuiID statusNode = 0;
        ImGui::DockBuilderSplitNode(
            sideNode, ImGuiDir_Down, 0.52F, &cameraNode, &statusNode);

        ImGui::DockBuilderDockWindow("장면##SceneViewV3", sceneNode);
        ImGui::DockBuilderDockWindow("카메라##CameraViewV3", cameraNode);
        ImGui::DockBuilderDockWindow("DroneSim | 비행 상태##FlightStatusV3", statusNode);
        ImGui::DockBuilderFinish(dockspaceId);
    }
    ImGui::DockSpaceOverViewport(dockspaceId, viewport);
}

ImU32 BuildingHeightColor(double heightMeters, double maximumHeightMeters)
{
    const float ratio = maximumHeightMeters > 0.0
        ? static_cast<float>(std::clamp(heightMeters / maximumHeightMeters, 0.0, 1.0))
        : 0.0F;
    const ImVec4 low{0.22F, 0.63F, 0.39F, 1.0F};
    const ImVec4 high{0.91F, 0.44F, 0.19F, 1.0F};
    return ImGui::ColorConvertFloat4ToU32(ImVec4{
        low.x + (high.x - low.x) * ratio,
        low.y + (high.y - low.y) * ratio,
        low.z + (high.z - low.z) * ratio, 1.0F});
}

void DrawBuildingMiniMap(
    std::span<const flight_planning::BuildingObstacle> buildings,
    const glm::dvec2& groundSizeMeters,
    ImDrawList* drawList, const ImVec2& sceneMinimum, const ImVec2& sceneMaximum)
{
    if (buildings.empty())
        return;

    const float mapWidth = std::min(330.0F, (sceneMaximum.x - sceneMinimum.x) * 0.40F);
    const float mapHeight = std::min(240.0F, (sceneMaximum.y - sceneMinimum.y) * 0.42F);
    if (mapWidth < 150.0F || mapHeight < 115.0F)
        return;

    const ImVec2 mapMinimum{sceneMaximum.x - mapWidth - 12.0F,
        sceneMaximum.y - mapHeight - 12.0F};
    const ImVec2 mapMaximum{mapMinimum.x + mapWidth, mapMinimum.y + mapHeight};
    drawList->AddRectFilled(mapMinimum, mapMaximum,
        IM_COL32(14, 20, 25, 232), 7.0F);
    drawList->AddRect(mapMinimum, mapMaximum,
        IM_COL32(210, 220, 225, 190), 7.0F, 0, 1.0F);
    drawList->AddText({mapMinimum.x + 10.0F, mapMinimum.y + 7.0F},
        IM_COL32(245, 245, 245, 255), "구조물 위치 · 높이 미니맵");

    const float drawingHeight = mapHeight - 58.0F;
    const ImVec2 mapCenter{(mapMinimum.x + mapMaximum.x) * 0.5F,
        mapMinimum.y + 28.0F + drawingHeight * 0.5F};
    const double halfEast = std::max(groundSizeMeters.x * 0.5, 1.0);
    const double halfNorth = std::max(groundSizeMeters.y * 0.5, 1.0);
    double maximumHeightMeters = 0.0;
    for (const auto& building : buildings)
        maximumHeightMeters = std::max(maximumHeightMeters, building.heightMeters);

    constexpr double horizontalProjection = 0.70710678118;
    constexpr double verticalProjection = 0.35355339059;
    constexpr double heightProjection = 0.80;
    const double projectedWidth = horizontalProjection
        * (halfEast + halfNorth) * 2.0;
    const double projectedHeight = verticalProjection
        * (halfEast + halfNorth) * 2.0
        + heightProjection * maximumHeightMeters;
    const float scale = static_cast<float>(std::min(
        (mapWidth - 28.0F) / std::max(projectedWidth, 1.0),
        drawingHeight / std::max(projectedHeight, 1.0)));
    const double minimumProjectedY = -verticalProjection * (halfEast + halfNorth)
        - heightProjection * maximumHeightMeters;
    const double maximumProjectedY = verticalProjection * (halfEast + halfNorth);
    const float verticalOffset = static_cast<float>(
        -(minimumProjectedY + maximumProjectedY) * 0.5) * scale;
    drawList->AddText({mapMinimum.x + 10.0F, mapMaximum.y - 18.0F},
        IM_COL32(190, 200, 205, 255), "동 ↗  북 ↖   색과 돌출은 높이 표시");

    const auto project = [&](const glm::dvec2& position, double heightMeters)
    {
        const double projectedX = horizontalProjection * (position.x - position.y);
        const double projectedY = -verticalProjection * (position.x + position.y)
            - heightProjection * heightMeters;
        return ImVec2{
            mapCenter.x + static_cast<float>(projectedX) * scale,
            mapCenter.y + static_cast<float>(projectedY) * scale + verticalOffset};
    };

    std::vector<std::size_t> drawOrder(buildings.size());
    for (std::size_t index = 0; index < drawOrder.size(); ++index)
        drawOrder[index] = index;
    const auto outlineCenter = [](const flight_planning::BuildingObstacle& building)
    {
        glm::dvec2 center{0.0};
        for (const auto& point : building.outlineMeters)
            center += point;
        return building.outlineMeters.empty()
            ? center : center / static_cast<double>(building.outlineMeters.size());
    };
    std::sort(drawOrder.begin(), drawOrder.end(), [&](std::size_t left, std::size_t right)
    {
        const auto leftCenter = outlineCenter(buildings[left]);
        const auto rightCenter = outlineCenter(buildings[right]);
        return leftCenter.x + leftCenter.y < rightCenter.x + rightCenter.y;
    });

    drawList->PushClipRect(mapMinimum, mapMaximum, true);
    for (const std::size_t index : drawOrder)
    {
        const auto& building = buildings[index];
        if (building.outlineMeters.size() < 3U)
            continue;

        std::vector<ImVec2> roof;
        roof.reserve(building.outlineMeters.size());
        for (const auto& point : building.outlineMeters)
            roof.push_back(project(point, building.heightMeters));

        const ImU32 roofColor = BuildingHeightColor(
            building.heightMeters, maximumHeightMeters);
        const ImVec4 roofColorF = ImGui::ColorConvertU32ToFloat4(roofColor);
        const ImU32 sideColor = ImGui::ColorConvertFloat4ToU32(ImVec4{
            roofColorF.x * 0.58F, roofColorF.y * 0.58F,
            roofColorF.z * 0.58F, 1.0F});
        for (std::size_t pointIndex = 0; pointIndex < building.outlineMeters.size(); ++pointIndex)
        {
            const std::size_t nextIndex = (pointIndex + 1U)
                % building.outlineMeters.size();
            const ImVec2 lowerFirst = project(building.outlineMeters[pointIndex], 0.0);
            const ImVec2 lowerSecond = project(building.outlineMeters[nextIndex], 0.0);
            const ImVec2 upperFirst = roof[pointIndex];
            const ImVec2 upperSecond = roof[nextIndex];
            const ImVec2 side[]{lowerFirst, lowerSecond, upperSecond, upperFirst};
            drawList->AddConvexPolyFilled(side, 4, sideColor);
            drawList->AddLine(lowerFirst, upperFirst,
                IM_COL32(30, 35, 38, 210), 1.0F);
        }
        drawList->AddConvexPolyFilled(roof.data(),
            static_cast<int>(roof.size()), roofColor);
        drawList->AddPolyline(roof.data(), static_cast<int>(roof.size()),
            IM_COL32(24, 29, 31, 255), ImDrawFlags_Closed, 1.0F);

        const glm::dvec2 center = outlineCenter(building);
        const ImVec2 labelPosition = project(center, building.heightMeters);
        const std::string label = std::to_string(index + 1U);
        const ImVec2 labelSize = ImGui::CalcTextSize(label.c_str());
        drawList->AddText({labelPosition.x - labelSize.x * 0.5F,
            labelPosition.y - labelSize.y * 0.5F},
            IM_COL32(255, 255, 255, 255), label.c_str());

        ImGui::SetCursorScreenPos({labelPosition.x - 14.0F,
            labelPosition.y - 9.0F});
        const std::string itemId = "##구조물" + std::to_string(index + 1U);
        ImGui::InvisibleButton(itemId.c_str(), ImVec2{28.0F, 18.0F});
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s\n동 %.1f m · 북 %.1f m\n높이 %.1f m",
                building.name.c_str(), center.x, center.y, building.heightMeters);
        }
    }
    drawList->PopClipRect();
}

void DrawScenePanel(const simulation::SimulationCore& simulation,
    simulation::DroneId& observerDroneId,
    std::span<const flight_planning::BuildingObstacle> buildings,
    const glm::dvec2& groundSizeMeters,
    const std::function<std::uint32_t(int, int)>& renderScene)
{
    if (ImGui::Begin("장면##SceneViewV3"))
    {
        const auto droneIds = simulation.DroneIds();
        if (!droneIds.empty())
        {
            if (observerDroneId == 0U)
                observerDroneId = simulation.PrimaryDroneId();

            const std::string selectedName = DroneName(observerDroneId);
            if (ImGui::BeginCombo("장면 시점 드론", selectedName.c_str()))
            {
                for (const simulation::DroneId droneId : droneIds)
                {
                    const std::string name = DroneName(droneId);
                    const bool selected = droneId == observerDroneId;
                    if (ImGui::Selectable(name.c_str(), selected))
                        observerDroneId = droneId;
                    if (selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        }

        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 imageSize{
            std::max(1.0F, available.x), std::max(1.0F, available.y)};
        const std::uint32_t texture = renderScene(
            static_cast<int>(imageSize.x), static_cast<int>(imageSize.y));
        ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
            ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
        DrawBuildingMiniMap(buildings, groundSizeMeters,
            ImGui::GetWindowDrawList(), ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
    }
    ImGui::End();
}

void DrawCameraPanel(
    const simulation::SimulationCore& simulation,
    simulation::DroneId& observerDroneId,
    const std::function<std::uint32_t(simulation::DroneId)>& renderCamera)
{
    if (ImGui::Begin("카메라##CameraViewV3"))
    {
        ImGui::TextDisabled("드론 이름을 누르면 장면 시점이 바뀝니다.");
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const auto droneIds = simulation.DroneIds();
        if (droneIds.empty())
        {
            ImGui::TextUnformatted("드론이 없습니다.");
        }
        else
        {
            constexpr int cameraColumns = 2;
            const int cameraRows = static_cast<int>(
                (droneIds.size() + cameraColumns - 1U) / cameraColumns);
            const float rowSpacing = ImGui::GetStyle().ItemSpacing.y;
            const float cellHeight = std::max(1.0F,
                (available.y - rowSpacing * static_cast<float>(cameraRows - 1))
                    / static_cast<float>(cameraRows));

            if (ImGui::BeginTable("드론별 카메라", cameraColumns,
                    ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_BordersOuterH
                        | ImGuiTableFlags_SizingStretchSame,
                    available))
            {
                for (const simulation::DroneId droneId : droneIds)
                {
                    ImGui::TableNextColumn();
                    const auto& camera = simulation.Camera(droneId);
                    const std::string label = DroneName(droneId);
                    const bool selectedForScene = droneId == observerDroneId;
                    const std::string selectableLabel = selectedForScene
                        ? "> " + label : label;
                    if (ImGui::Selectable(selectableLabel.c_str(), selectedForScene,
                            ImGuiSelectableFlags_SpanAvailWidth))
                    {
                        observerDroneId = droneId;
                    }

                    const int imageWidth = camera.model.imageWidthPixels;
                    const int imageHeight = camera.model.imageHeightPixels;
                    if (camera.status != simulation::CameraStatus::Active
                        || imageWidth <= 0 || imageHeight <= 0)
                    {
                        ImGui::TextDisabled("%s",
                            simulation::ToString(camera.status).data());
                        continue;
                    }

                    const ImVec2 cell = ImGui::GetContentRegionAvail();
                    const float imageBoxHeight = std::max(1.0F,
                        cellHeight - ImGui::GetTextLineHeightWithSpacing()
                            - ImGui::GetStyle().ItemSpacing.y);
                    const float scale = std::min(
                        cell.x / static_cast<float>(imageWidth),
                        imageBoxHeight / static_cast<float>(imageHeight));
                    const ImVec2 imageSize{
                        static_cast<float>(imageWidth) * scale,
                        static_cast<float>(imageHeight) * scale};
                    const float imageX = ImGui::GetCursorPosX()
                        + std::max(0.0F, (cell.x - imageSize.x) * 0.5F);
                    ImGui::SetCursorPosX(imageX);
                    const std::uint32_t texture = renderCamera(droneId);
                    ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
                        ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
                    ImGui::TextDisabled("%d × %d", imageWidth, imageHeight);
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
}

void DrawFlightStatusPanel(
    const simulation::SimulationCore& simulation,
    bool gamepadConnected,
    bool fleetReturnCanStart,
    const std::function<void()>& startFleetReturn,
    const std::string& completeFleetMissionStatus,
    bool completeFleetMissionCanStart,
    const std::function<void()>& startCompleteFleetMission,
    const communication::VirtualMeshNetwork* formationNetwork,
    const communication::DroneSocketTransport* socketTransport,
    int& selectedFaultSource,
    const std::function<void(simulation::DroneId, bool)>&
        setFormationNodeRadioEnabled)
{
    if (ImGui::Begin("DroneSim | 비행 상태##FlightStatusV3"))
    {
        const auto& primary = simulation.State();
        ImGui::Text("주 드론: %s", DroneName(simulation.PrimaryDroneId()).c_str());
        ImGui::Text("비행 상태: %s", simulation::ToString(primary.mode).data());
        ImGui::Text("위치 [m]: X %.2f   Y %.2f   높이 %.2f",
            primary.positionMeters.x, primary.positionMeters.y, primary.positionMeters.z);
        ImGui::Text("모의 시간: %.2f초   진행 Tick: %llu",
            simulation.SimulationTimeSeconds(),
            static_cast<unsigned long long>(simulation.TickCount()));
        ImGui::Separator();
        ImGui::Text("임무: %s", completeFleetMissionStatus.c_str());
        if (completeFleetMissionCanStart
            && ImGui::Button("편대 정찰 임무 시작"))
            startCompleteFleetMission();
        else if (!completeFleetMissionCanStart)
            ImGui::TextDisabled("모든 드론이 지상에 있고 다른 임무가 없어야 합니다.");
        if (fleetReturnCanStart && ImGui::Button("출발 지점으로 복귀"))
            startFleetReturn();

        if (formationNetwork != nullptr)
        {
            ImGui::Separator();
            ImGui::Text("드론 간 UDP 통신: 직접 연결");
            const auto droneIds = simulation.DroneIds();
            if (!droneIds.empty())
            {
                selectedFaultSource = std::clamp(selectedFaultSource, 0,
                    static_cast<int>(droneIds.size() - 1U));
                std::vector<std::string> names(droneIds.size());
                std::vector<const char*> namePointers(droneIds.size());
                for (std::size_t index = 0; index < droneIds.size(); ++index)
                {
                    names[index] = DroneName(droneIds[index]);
                    namePointers[index] = names[index].c_str();
                }
                ImGui::SetNextItemWidth(-1.0F);
                ImGui::Combo("시험할 드론", &selectedFaultSource,
                    namePointers.data(), static_cast<int>(droneIds.size()));
                const simulation::DroneId droneId = droneIds[
                    static_cast<std::size_t>(selectedFaultSource)];
                const bool radioEnabled = formationNetwork->IsNodeRadioEnabled(droneId);
                const simulation::DroneId leaderId = simulation.PrimaryDroneId();
                const bool isLeader = droneId == leaderId;
                const bool linked = isLeader
                    || formationNetwork->HasDirectLink(droneId, leaderId);
                const auto port = socketTransport != nullptr
                    ? socketTransport->PortFor(droneId) : std::nullopt;
                ImGui::Text("소켓 포트: %u · 편대 연결: %s",
                    port ? static_cast<unsigned int>(*port) : 0U,
                    isLeader ? "리더" : (radioEnabled && linked ? "연결됨" : "끊김"));
                if (radioEnabled)
                {
                    if (ImGui::Button("이 드론 통신 차단"))
                        setFormationNodeRadioEnabled(droneId, false);
                }
                else if (ImGui::Button("이 드론 통신 복구"))
                {
                    setFormationNodeRadioEnabled(droneId, true);
                }
            }
            const auto& metrics = formationNetwork->Metrics();
            ImGui::Text("전달 %llu · 폐기 %llu",
                static_cast<unsigned long long>(metrics.deliveredMessages),
                static_cast<unsigned long long>(metrics.droppedMessages));
            const auto events = formationNetwork->Events();
            const std::size_t visibleEvents = std::min<std::size_t>(events.size(), 4U);
            for (std::size_t offset = 0; offset < visibleEvents; ++offset)
            {
                const auto& event = events[events.size() - 1U - offset];
                ImGui::Text("%.2f초  %s  %s → %s",
                    static_cast<double>(event.tick) / 60.0,
                    MeshEventLabel(event.kind), NetworkNodeName(event.source).c_str(),
                    NetworkNodeName(event.destination).c_str());
            }
        }
        ImGui::Separator();
        ImGui::TextDisabled("조작: A 이륙 · 왼쪽 스틱 이동 · RT/LT 높이 · B 착륙");
        ImGui::Separator();
        ImGui::Text("게임패드: %s",
            gamepadConnected ? "연결됨" : "연결 안 됨");
        ImGui::Separator();
        ImGui::Text("편대 상태 (%zu대)", simulation.DroneCount());
        for (const simulation::DroneId id : simulation.DroneIds())
        {
            const auto& drone = simulation.State(id);
            ImGui::Text("%s  %-16s  (%.1f, %.1f, %.1f) m",
                DroneName(id).c_str(), simulation::ToString(drone.mode).data(),
                drone.positionMeters.x, drone.positionMeters.y, drone.positionMeters.z);
        }
    }
    ImGui::End();
}
}

SimulatorHud::SimulatorHud(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    LoadWindowsKoreanFont();

    if (const char* localAppData = std::getenv("LOCALAPPDATA"))
    {
        std::error_code error;
        const auto settingsDirectory = std::filesystem::path(localAppData) / "DroneSim";
        std::filesystem::create_directories(settingsDirectory, error);
        if (!error)
        {
            layoutFile_ = (settingsDirectory / "imgui.ini").string();
            ImGui::GetIO().IniFilename = layoutFile_.c_str();
        }
    }
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
    {
        ImGui::DestroyContext();
        throw std::runtime_error("시뮬레이터 HUD 입력을 준비하지 못했습니다.");
    }
    if (!ImGui_ImplOpenGL3_Init("#version 330"))
    {
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        throw std::runtime_error("시뮬레이터 HUD 화면을 준비하지 못했습니다.");
    }
    initialized_ = true;
}

SimulatorHud::~SimulatorHud()
{
    if (!initialized_)
        return;

#ifndef __EMSCRIPTEN__
    for (const PendingGpuQuery& query : backendGpuQueries_)
    {
        if (query.query != 0)
            glDeleteQueries(1, &query.query);
    }
#endif
    if (!layoutFile_.empty())
        ImGui::SaveIniSettingsToDisk(layoutFile_.c_str());
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void SimulatorHud::CollectCompletedGpuTimings(HudRenderTimings* timings)
{
#ifndef __EMSCRIPTEN__
    if (timings == nullptr || !timings->collectGpuTimings || !GLAD_GL_VERSION_3_3)
        return;

    for (PendingGpuQuery& query : backendGpuQueries_)
    {
        if (!query.pending)
            continue;

        GLint available = GL_FALSE;
        glGetQueryObjectiv(query.query, GL_QUERY_RESULT_AVAILABLE, &available);
        if (available != GL_TRUE)
            continue;

        GLuint64 durationNanoseconds = 0;
        glGetQueryObjectui64v(query.query, GL_QUERY_RESULT, &durationNanoseconds);
        const std::size_t resultIndex = timings->completedGpuFrameCount++;
        if (resultIndex < timings->completedGpuFrames.size())
        {
            timings->completedGpuFrames[resultIndex] = HudGpuTiming{
                query.frameIndex,
                static_cast<double>(durationNanoseconds) / 1000.0};
        }
        query.pending = false;
    }
#else
    static_cast<void>(timings);
#endif
}

void SimulatorHud::Render(const simulation::SimulationCore& simulation,
    simulation::DroneId& observerDroneId,
    bool gamepadConnected,
    bool fleetReturnCanStart,
    const std::function<void()>& startFleetReturn,
    const std::string& completeFleetMissionStatus,
    bool completeFleetMissionCanStart,
    const std::function<void()>& startCompleteFleetMission,
    const communication::VirtualMeshNetwork* formationNetwork,
    const communication::DroneSocketTransport* socketTransport,
    const std::function<void(simulation::DroneId, bool)>&
        setFormationNodeRadioEnabled,
    std::span<const flight_planning::BuildingObstacle> buildings,
    const glm::dvec2& groundSizeMeters,
    const std::function<std::uint32_t(int, int)>& renderScene,
    const std::function<std::uint32_t(simulation::DroneId)>& renderCamera,
    HudRenderTimings* timings)
{
    CollectCompletedGpuTimings(timings);

    // 패널을 만드는 동안 장면·카메라 그리기 콜백도 실행되므로 순수 UI 시간은 아니다.
    const auto interfaceBuildStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    CreateDefaultDockspace(viewport);
    DrawScenePanel(simulation, observerDroneId, buildings, groundSizeMeters,
        renderScene);
    DrawCameraPanel(simulation, observerDroneId, renderCamera);
    DrawFlightStatusPanel(simulation, gamepadConnected,
        fleetReturnCanStart, startFleetReturn,
        completeFleetMissionStatus, completeFleetMissionCanStart,
        startCompleteFleetMission,
        formationNetwork, socketTransport, selectedFaultSource_,
        setFormationNodeRadioEnabled);

    if (timings != nullptr)
    {
        timings->interfaceBuildMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - interfaceBuildStart).count();
    }
    const auto frameFinalizeStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    ImGui::Render();
    if (timings != nullptr)
    {
        timings->frameFinalizeMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - frameFinalizeStart).count();
    }
    const auto backendDrawStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    ImDrawData* drawData = ImGui::GetDrawData();
    if (timings != nullptr)
    {
        timings->drawListCount = static_cast<std::size_t>(drawData->CmdListsCount);
        timings->vertexCount = static_cast<std::size_t>(drawData->TotalVtxCount);
        timings->indexCount = static_cast<std::size_t>(drawData->TotalIdxCount);
        for (const ImDrawList* list : drawData->CmdLists)
            timings->drawCommandCount += static_cast<std::size_t>(list->CmdBuffer.Size);
    }
#ifndef __EMSCRIPTEN__
    PendingGpuQuery* gpuQuery = nullptr;
    if (timings != nullptr && timings->collectGpuTimings && GLAD_GL_VERSION_3_3)
    {
        for (PendingGpuQuery& query : backendGpuQueries_)
        {
            if (query.pending)
                continue;
            if (query.query == 0)
                glGenQueries(1, &query.query);
            gpuQuery = &query;
            glBeginQuery(GL_TIME_ELAPSED, gpuQuery->query);
            break;
        }
    }
#endif
    ImGui_ImplOpenGL3_RenderDrawData(drawData);
#ifndef __EMSCRIPTEN__
    if (gpuQuery != nullptr)
    {
        glEndQuery(GL_TIME_ELAPSED);
        gpuQuery->frameIndex = timings->frameIndex;
        gpuQuery->pending = true;
    }
#endif
    if (timings != nullptr)
    {
        timings->backendDrawMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - backendDrawStart).count();
    }
}
}
