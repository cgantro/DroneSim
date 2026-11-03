#include "gui/SimulatorHud.h"
#include "gui/SimulatorView.h"
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

            const std::string selectedLabel = DroneName(observerDroneId);
            if (ImGui::BeginCombo("선택한 드론", selectedLabel.c_str()))
            {
                for (const simulation::DroneId droneId : droneIds)
                {
                    const std::string label = DroneName(droneId);
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

void DrawFlightStatusPanel(
    const simulation::SimulationCore& simulation,
    bool gamepadConnected,
    bool fleetReturnCanStart,
    const std::function<void()>& startFleetReturn,
    const std::string& completeFleetMissionStatus,
    bool completeFleetMissionCanStart,
    const std::function<void()>& startCompleteFleetMission,
    const FleetNetworkStatus* fleetNetworkStatus,
    NetworkFaultPanel& networkFaultPanel,
    const std::function<void(simulation::DroneId, bool)>&
        setFormationNodeRadioEnabled)
{
    if (ImGui::Begin("운용##OperationsV5"))
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

        if (fleetNetworkStatus != nullptr)
            networkFaultPanel.Render(*fleetNetworkStatus,
                setFormationNodeRadioEnabled);
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
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#ifndef __EMSCRIPTEN__
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
#endif

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
    DrawScenePanel(simulation, observerDroneId, sceneView, renderScene);
    miniMapPanel_.Render(buildings, drones, routes, groundSizeMeters);
    DrawCameraPanel(simulation, observerDroneId, renderCamera, setCameraGimbalAngles);
    DrawFlightStatusPanel(simulation, gamepadConnected,
        fleetReturnCanStart, startFleetReturn,
        completeFleetMissionStatus, completeFleetMissionCanStart,
        startCompleteFleetMission,
        fleetNetworkStatus, networkFaultPanel_,
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

#ifndef __EMSCRIPTEN__
    // 화면 밖으로 분리한 ImGui 창을 각각의 네이티브 창에 그린 뒤 주 창 context로 돌아온다.
    if ((ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0)
    {
        GLFWwindow* mainContext = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(mainContext);
    }
#endif
}
}
