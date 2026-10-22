#include "SimulatorHud.h"

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

    if (!layoutFile_.empty())
        ImGui::SaveIniSettingsToDisk(layoutFile_.c_str());
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void SimulatorHud::Render(const simulation::SimulationCore& simulation,
    bool gamepadConnected, bool groundControlConnected, bool cameraActive,
    int cameraImageWidth, int cameraImageHeight,
    const std::function<std::uint32_t(int, int)>& renderScene,
    const std::function<std::uint32_t()>& renderCamera)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGuiViewport* viewport = ImGui::GetMainViewport();
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

    const auto drawScenePanel = [&renderScene]
    {
        if (ImGui::Begin("장면##SceneViewV2"))
        {
            const ImVec2 available = ImGui::GetContentRegionAvail();
            const ImVec2 imageSize{std::max(1.0F, available.x),
                std::max(1.0F, available.y)};
            const std::uint32_t texture = renderScene(
                static_cast<int>(imageSize.x), static_cast<int>(imageSize.y));
            ImGui::Image(static_cast<ImTextureID>(texture), imageSize,
                ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F});
        }
        ImGui::End();
    };

    drawScenePanel();

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
        const ImVec2 badgeMaximum{
            imageMaximum.x - 8.0F,
            imageMaximum.y - 8.0F};
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(badgeMinimum, badgeMaximum,
            IM_COL32(0, 0, 0, 190), 4.0F);
        drawList->AddText(ImVec2{badgeMinimum.x + padding.x,
            badgeMinimum.y + padding.y}, IM_COL32(255, 255, 255, 255), resolution.c_str());
    }
    ImGui::End();

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

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
}
