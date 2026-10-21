#include "gui/GuiModule.h"

#include "graphics/Window.h"

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <stdexcept>

namespace grasplink::gui
{

struct GuiModule::Impl
{
    explicit Impl(grasplink::graphics::Window& window)
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 12.0F;
        style.ChildRounding = 9.0F;
        style.FrameRounding = 7.0F;
        style.PopupRounding = 9.0F;
        style.ScrollbarRounding = 8.0F;
        style.GrabRounding = 7.0F;
        style.WindowPadding = ImVec2(18.0F, 16.0F);
        style.FramePadding = ImVec2(11.0F, 8.0F);
        style.ItemSpacing = ImVec2(10.0F, 9.0F);
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.055F, 0.071F, 0.094F, 1.0F);
        style.Colors[ImGuiCol_ChildBg] = ImVec4(0.075F, 0.094F, 0.122F, 1.0F);
        style.Colors[ImGuiCol_Border] = ImVec4(0.18F, 0.22F, 0.28F, 0.8F);
        style.Colors[ImGuiCol_FrameBg] = ImVec4(0.12F, 0.15F, 0.19F, 1.0F);
        style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.17F, 0.21F, 0.27F, 1.0F);
        style.Colors[ImGuiCol_Button] = ImVec4(0.12F, 0.38F, 0.48F, 1.0F);
        style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.16F, 0.52F, 0.63F, 1.0F);
        style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.10F, 0.31F, 0.40F, 1.0F);
        style.Colors[ImGuiCol_Header] = ImVec4(0.11F, 0.28F, 0.34F, 0.85F);
        style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.15F, 0.38F, 0.45F, 0.95F);
        if (!ImGui_ImplGlfw_InitForOpenGL(window.GetNativeHandle(), true))
        {
            ImGui::DestroyContext();
            throw std::runtime_error("ImGui GLFW 백엔드를 시작하지 못했습니다.");
        }
#ifdef __EMSCRIPTEN__
        ImGui_ImplGlfw_InstallEmscriptenCallbacks(window.GetNativeHandle(), "#canvas");
        const bool rendererInitialized = ImGui_ImplOpenGL3_Init("#version 300 es");
#else
        const bool rendererInitialized = ImGui_ImplOpenGL3_Init("#version 330");
#endif
        if (!rendererInitialized)
        {
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
            throw std::runtime_error("ImGui OpenGL 백엔드를 시작하지 못했습니다.");
        }
    }

    ~Impl()
    {
        // UI의 OpenGL backend가 GPU 자원을 해제하므로, 이 호출이 끝날 때까지 Window의 OpenGL context가 유효해야 한다.
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    void BeginFrame()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    void EndFrame()
    {
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
};

GuiModule::GuiModule(grasplink::graphics::Window& window)
    : m_Impl(std::make_unique<Impl>(window))
{
}

GuiModule::~GuiModule() = default;

void GuiModule::BeginFrame()
{
    m_Impl->BeginFrame();
}

void GuiModule::EndFrame()
{
    m_Impl->EndFrame();
}

bool GuiModule::WantsMouse() const
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool GuiModule::WantsKeyboard() const
{
    return ImGui::GetIO().WantCaptureKeyboard;
}

}
