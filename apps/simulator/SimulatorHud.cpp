#include "SimulatorHud.h"
#include "graphics/GlApi.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include "simulation_core/ScenarioRunner.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <chrono>

namespace aerosimlab::simulator
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

const char* CaptureStatusLabel(simulation::CaptureAssessment assessment)
{
    switch (assessment)
    {
    case simulation::CaptureAssessment::Ready: return "목표가 카메라 화면에 보임";
    case simulation::CaptureAssessment::CameraUnavailable: return "카메라 꺼짐 또는 장애";
    case simulation::CaptureAssessment::NoNewFrame: return "새 영상 프레임 없음";
    case simulation::CaptureAssessment::TooNear: return "목표에 너무 가까움";
    case simulation::CaptureAssessment::TooFar: return "목표에서 너무 멂";
    case simulation::CaptureAssessment::SurfaceFacingAway: return "카메라가 목표를 향하지 않음";
    case simulation::CaptureAssessment::OutsideFrame: return "카메라 화면 밖";
    case simulation::CaptureAssessment::ResolutionTooLow: return "화면에서 목표가 너무 작음";
    case simulation::CaptureAssessment::Occluded: return "목표가 구조물에 가려짐";
    }
    return "판정할 수 없음";
}

const char* MeshEventLabel(communication::MeshEventKind kind)
{
    switch (kind)
    {
    case communication::MeshEventKind::LinkDisabled: return "링크 장애";
    case communication::MeshEventKind::LinkRestored: return "링크 복구";
    case communication::MeshEventKind::NeighborDiscovered: return "이웃 발견";
    case communication::MeshEventKind::RouteSearchStarted: return "경로 탐색";
    case communication::MeshEventKind::RouteFound: return "경로 복구";
    case communication::MeshEventKind::MessageDelivered: return "메시지 전달";
    case communication::MeshEventKind::PacketDropped: return "패킷 폐기";
    case communication::MeshEventKind::RouteError: return "경로 오류";
    }
    return "통신 사건";
}

void CreateDefaultDockspace(ImGuiViewport* viewport)
{
    const ImGuiID dockspaceId = ImGui::GetID("AeroSimLabDockspaceV3");
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
        ImGui::DockBuilderDockWindow("AeroSimLab | 비행 상태##FlightStatusV3", statusNode);
        ImGui::DockBuilderFinish(dockspaceId);
    }
    ImGui::DockSpaceOverViewport(dockspaceId, viewport);
}

void DrawScenePanel(const std::function<std::uint32_t(int, int)>& renderScene)
{
    if (ImGui::Begin("장면##SceneViewV3"))
    {
        const ImVec2 available = ImGui::GetContentRegionAvail();
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
    bool cameraActive,
    int cameraImageWidth,
    int cameraImageHeight,
    const std::function<std::uint32_t()>& renderCamera)
{
    if (ImGui::Begin("카메라##CameraViewV3"))
    {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 imageOrigin = ImGui::GetCursorPos();
        const ImVec2 contentOrigin = ImGui::GetCursorScreenPos();
        ImVec2 imageMaximum{contentOrigin.x + available.x,
            contentOrigin.y + available.y};

        if (cameraActive && cameraImageWidth > 0 && cameraImageHeight > 0)
        {
            const float scale = std::min(
                available.x / static_cast<float>(cameraImageWidth),
                available.y / static_cast<float>(cameraImageHeight));
            const ImVec2 imageSize{
                static_cast<float>(cameraImageWidth) * scale,
                static_cast<float>(cameraImageHeight) * scale};
            ImGui::SetCursorPos(ImVec2{
                imageOrigin.x + (available.x - imageSize.x) * 0.5F,
                imageOrigin.y + (available.y - imageSize.y) * 0.5F});
            const std::uint32_t texture = renderCamera();
            ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
                ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
            imageMaximum = ImGui::GetItemRectMax();
        }
        else
        {
            ImGui::TextUnformatted("카메라가 꺼져 있습니다.");
        }

        const std::string resolution = std::to_string(cameraImageWidth)
            + " × " + std::to_string(cameraImageHeight);
        const ImVec2 textSize = ImGui::CalcTextSize(resolution.c_str());
        const ImVec2 padding{8.0F, 4.0F};
        const ImVec2 badgeMinimum{
            imageMaximum.x - textSize.x - padding.x * 2.0F - 8.0F,
            imageMaximum.y - textSize.y - padding.y * 2.0F - 8.0F};
        const ImVec2 badgeMaximum{imageMaximum.x - 8.0F, imageMaximum.y - 8.0F};
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(badgeMinimum, badgeMaximum,
            IM_COL32(0, 0, 0, 190), 4.0F);
        drawList->AddText(ImVec2{badgeMinimum.x + padding.x,
            badgeMinimum.y + padding.y}, IM_COL32(255, 255, 255, 255), resolution.c_str());
    }
    ImGui::End();
}

void DrawFlightStatusPanel(
    const simulation::SimulationCore& simulation,
    bool gamepadConnected,
    bool groundControlConnected,
    double missionDistanceMeters,
    const std::string& inspectionTargetName,
    const std::string& inspectionMissionStatus,
    bool missionCanStart,
    const std::function<void()>& startInspectionMission,
    const std::string& formationMissionStatus,
    bool formationMissionCanStart,
    bool formationMissionActive,
    simulation::FormationShape& selectedFormationShape,
    const std::function<void(simulation::FormationShape)>& startFormationTransit,
    const communication::VirtualMeshNetwork* formationNetwork,
    int& selectedFaultSource,
    int& selectedFaultDestination,
    const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
        setFormationLinkDisabled,
    simulation::CaptureAssessment captureAssessment)
{
    if (ImGui::Begin("AeroSimLab | 비행 상태##FlightStatusV3"))
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
        ImGui::Text("정찰 대상: %s", inspectionTargetName.c_str());
        ImGui::Text("화면 확인: %s", CaptureStatusLabel(captureAssessment));
        ImGui::TextDisabled("목표가 카메라 화면에 잡히는지 보여 줍니다. 파손 여부는 사람이 영상으로 확인합니다.");
        ImGui::Text("정찰 임무: %s", inspectionMissionStatus.c_str());
        ImGui::Text("촬영 위치까지 %.1f m", missionDistanceMeters);
        if (missionCanStart && ImGui::Button("정찰 임무 시작"))
            startInspectionMission();
        ImGui::Separator();
        ImGui::Text("편대 이동: %s", formationMissionStatus.c_str());
        const char* formationLabels[]{"V자", "일렬", "횡대"};
        int selectedShape = static_cast<int>(selectedFormationShape);
        if (ImGui::Combo("편대 대형", &selectedShape,
                formationLabels, IM_ARRAYSIZE(formationLabels)))
            selectedFormationShape = static_cast<simulation::FormationShape>(selectedShape);
        if (formationMissionCanStart && ImGui::Button("편대 이동 시작"))
            startFormationTransit(selectedFormationShape);
        else if (!formationMissionCanStart)
            ImGui::TextDisabled("모든 드론이 지상에 있을 때 시작할 수 있습니다.");
        if (formationNetwork != nullptr)
        {
            ImGui::Separator();
            if (formationMissionActive)
            {
                ImGui::Text("통신 장애 시험");
                const auto droneIds = simulation.DroneIds();
                if (droneIds.size() >= 2U)
                {
                    const int lastDroneIndex = static_cast<int>(droneIds.size() - 1U);
                    selectedFaultSource = std::clamp(selectedFaultSource, 0, lastDroneIndex);
                    selectedFaultDestination = std::clamp(
                        selectedFaultDestination, 0, lastDroneIndex);
                    std::array<std::string, communication::maximumFleetDroneCount> names;
                    std::array<const char*, communication::maximumFleetDroneCount> namePointers{};
                    for (std::size_t index = 0; index < droneIds.size(); ++index)
                    {
                        names[index] = DroneName(droneIds[index]);
                        namePointers[index] = names[index].c_str();
                    }

                    ImGui::SetNextItemWidth(-1.0F);
                    ImGui::Combo("송신 드론", &selectedFaultSource,
                        namePointers.data(), static_cast<int>(droneIds.size()));
                    ImGui::SetNextItemWidth(-1.0F);
                    ImGui::Combo("상대 드론", &selectedFaultDestination,
                        namePointers.data(), static_cast<int>(droneIds.size()));
                    const simulation::DroneId source = droneIds[
                        static_cast<std::size_t>(selectedFaultSource)];
                    const simulation::DroneId destination = droneIds[
                        static_cast<std::size_t>(selectedFaultDestination)];
                    if (source == destination)
                    {
                        ImGui::TextDisabled("서로 다른 드론을 골라 주세요.");
                    }
                    else
                    {
                        const bool disabled = formationNetwork->IsLinkDisabled(source, destination);
                        ImGui::Text("강제 장애: %s", disabled ? "설정됨" : "해제됨");
                        const auto nextHop = formationNetwork->NextHop(source, destination);
                        if (nextHop)
                            ImGui::Text("현재 다음 드론: %s", DroneName(*nextHop).c_str());
                        else
                            ImGui::TextDisabled("현재 경로: 찾는 중 또는 연결 안 됨");
                        if (disabled)
                        {
                            if (ImGui::Button("링크 복구"))
                                setFormationLinkDisabled(source, destination, false);
                        }
                        else if (ImGui::Button("링크 끊기"))
                        {
                            setFormationLinkDisabled(source, destination, true);
                        }
                    }
                }
            }
            const auto& metrics = formationNetwork->Metrics();
            ImGui::Text("전달 %llu · 중계 %llu · 폐기 %llu",
                static_cast<unsigned long long>(metrics.deliveredMessages),
                static_cast<unsigned long long>(metrics.forwardedMessages),
                static_cast<unsigned long long>(metrics.droppedMessages));
            const auto events = formationNetwork->Events();
            const std::size_t visibleEvents = std::min<std::size_t>(events.size(), 4U);
            for (std::size_t offset = 0; offset < visibleEvents; ++offset)
            {
                const auto& event = events[events.size() - 1U - offset];
                ImGui::Text("%.2f초  %s  %s → %s",
                    static_cast<double>(event.tick) / 60.0,
                    MeshEventLabel(event.kind), DroneName(event.source).c_str(),
                    DroneName(event.destination).c_str());
                ImGui::TextDisabled("  %s", event.detail.c_str());
            }
        }
        ImGui::TextDisabled("조작: A 이륙 · 왼쪽 스틱 이동 · RT/LT 높이 · B 착륙");
        ImGui::Separator();
        ImGui::Text("게임패드: %s   지상통제: %s",
            gamepadConnected ? "연결됨" : "연결 안 됨",
            groundControlConnected ? "최근 입력 수신" : "최근 입력 없음");
        ImGui::Separator();
        ImGui::Text("편대 상태 (%zu / %zu)",
            simulation.DroneCount(), communication::maximumFleetDroneCount);
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
        const auto settingsDirectory = std::filesystem::path(localAppData) / "AeroSimLab";
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
    bool gamepadConnected, bool groundControlConnected, bool cameraActive,
    int cameraImageWidth, int cameraImageHeight,
    double missionDistanceMeters,
    const std::string& inspectionTargetName,
    const std::string& inspectionMissionStatus, bool missionCanStart,
    const std::function<void()>& startInspectionMission,
    const std::string& formationMissionStatus, bool formationMissionCanStart,
    bool formationMissionActive,
    const std::function<void(simulation::FormationShape)>& startFormationTransit,
    const communication::VirtualMeshNetwork* formationNetwork,
    const std::function<void(simulation::DroneId, simulation::DroneId, bool)>&
        setFormationLinkDisabled,
    simulation::CaptureAssessment captureAssessment,
    const std::function<std::uint32_t(int, int)>& renderScene,
    const std::function<std::uint32_t()>& renderCamera,
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
    DrawScenePanel(renderScene);
    DrawCameraPanel(cameraActive, cameraImageWidth, cameraImageHeight, renderCamera);
    DrawFlightStatusPanel(simulation, gamepadConnected, groundControlConnected,
        missionDistanceMeters, inspectionTargetName,
        inspectionMissionStatus,
        missionCanStart, startInspectionMission, formationMissionStatus,
        formationMissionCanStart, formationMissionActive, selectedFormationShape_,
        startFormationTransit, formationNetwork, selectedFaultSource_,
        selectedFaultDestination_, setFormationLinkDisabled,
        captureAssessment);

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
