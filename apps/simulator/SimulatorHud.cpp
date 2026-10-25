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
    case simulation::CaptureAssessment::Ready: return "촬영 구도 조건 충족";
    case simulation::CaptureAssessment::CameraUnavailable: return "카메라 꺼짐 또는 장애";
    case simulation::CaptureAssessment::NoNewFrame: return "새 영상 프레임 없음";
    case simulation::CaptureAssessment::TooNear: return "점검면에 너무 가까움";
    case simulation::CaptureAssessment::TooFar: return "점검면에 너무 멂";
    case simulation::CaptureAssessment::SurfaceFacingAway: return "점검면을 비스듬히 바라봄";
    case simulation::CaptureAssessment::OutsideFrame: return "카메라 화면 밖";
    case simulation::CaptureAssessment::ResolutionTooLow: return "화면에 담긴 크기가 부족함";
    case simulation::CaptureAssessment::Occluded: return "점검면이 가려짐";
    }
    return "판정할 수 없음";
}

void CreateDefaultDockspace(ImGuiViewport* viewport)
{
    const ImGuiID dockspaceId = ImGui::GetID("AeroSimLabDockspaceV2");
    if (ImGui::DockBuilderGetNode(dockspaceId) == nullptr)
    {
        ImGui::DockBuilderAddNode(dockspaceId,
            ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->WorkSize);
        ImGui::DockBuilderDockWindow("장면##SceneViewV2", dockspaceId);
        ImGui::DockBuilderFinish(dockspaceId);
    }
    ImGui::DockSpaceOverViewport(dockspaceId, viewport);
}

void DrawScenePanel(const std::function<std::uint32_t(int, int)>& renderScene)
{
    if (ImGui::Begin("장면##SceneViewV2"))
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
    ImGuiViewport* viewport,
    bool cameraActive,
    int cameraImageWidth,
    int cameraImageHeight,
    const std::function<std::uint32_t()>& renderCamera)
{
    const float cameraPanelWidth = std::min(448.0F, viewport->WorkSize.x * 0.45F);
    const float cameraPanelHeight = cameraPanelWidth * 0.82F;
    ImGui::SetNextWindowSize(
        ImVec2{cameraPanelWidth, cameraPanelHeight}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2{
        viewport->WorkPos.x + viewport->WorkSize.x - cameraPanelWidth - 12.0F,
        viewport->WorkPos.y + viewport->WorkSize.y - cameraPanelHeight - 12.0F},
        ImGuiCond_FirstUseEver);
    if (ImGui::Begin("카메라##CameraViewV2"))
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
    bool missionComplete,
    const std::string& inspectionFacilityId,
    simulation::CaptureAssessment captureAssessment)
{
    ImGui::SetNextWindowPos(ImVec2{14.0F, 14.0F}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.86F);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize
        | ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("AeroSimLab | 비행 상태", nullptr, flags))
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
        ImGui::Text("OSM 건물 %s 외벽 시험 구역: %s",
            inspectionFacilityId.c_str(), CaptureStatusLabel(captureAssessment));
        ImGui::TextDisabled("건물 외곽 상자로 만든 면입니다. 실제 구조물 형상이나 촬영 완료를 뜻하지 않습니다.");
        if (missionComplete)
            ImGui::Text("연습 비행 완료: 노란 표식에 착륙했습니다.");
        else if (primary.mode == simulation::FlightMode::Grounded)
            ImGui::Text("연습 비행: 이륙 후 노란 표식까지 이동하세요.");
        else if (missionDistanceMeters <= 1.5)
            ImGui::Text("표식에 도착했습니다. B를 눌러 착륙하세요.");
        else
            ImGui::Text("노란 표식까지 %.1f m 남았습니다.", missionDistanceMeters);
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
    double missionDistanceMeters, bool missionComplete,
    const std::string& inspectionFacilityId,
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
    DrawCameraPanel(viewport, cameraActive, cameraImageWidth, cameraImageHeight, renderCamera);
    DrawFlightStatusPanel(simulation, gamepadConnected, groundControlConnected,
        missionDistanceMeters, missionComplete, inspectionFacilityId, captureAssessment);

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
