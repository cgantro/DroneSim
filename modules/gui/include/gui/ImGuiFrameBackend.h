#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

struct GLFWwindow;

namespace dronesim::gui
{
/** @brief 한 프레임에서 얻은 GPU 렌더링 시간이다. */
struct HudGpuTiming
{
    std::uint64_t frameIndex = 0;
    double executionMicroseconds = 0.0;
};

/** @brief ImGui 프레임 구성·그리기 과정에서 모은 측정값이다. */
struct HudRenderTimings
{
    double interfaceBuildMicroseconds = 0.0;
    double frameFinalizeMicroseconds = 0.0;
    double backendDrawMicroseconds = 0.0;
    std::size_t drawListCount = 0;
    std::size_t drawCommandCount = 0;
    std::size_t vertexCount = 0;
    std::size_t indexCount = 0;
    bool collectGpuTimings = false;
    std::uint64_t frameIndex = 0;
    std::array<HudGpuTiming, 4> completedGpuFrames{};
    std::size_t completedGpuFrameCount = 0;
};

/** @brief ImGui와 GLFW·OpenGL 백엔드의 수명과 프레임 그리기를 관리한다. */
class ImGuiFrameBackend
{
public:
    explicit ImGuiFrameBackend(GLFWwindow* window);
    ~ImGuiFrameBackend();

    ImGuiFrameBackend(const ImGuiFrameBackend&) = delete;
    ImGuiFrameBackend& operator=(const ImGuiFrameBackend&) = delete;

    /** @brief 이전 프레임 측정값을 확인하고 새 ImGui 프레임을 시작한다. */
    void BeginFrame(HudRenderTimings* timings);

    /** @brief 완성된 UI를 그래픽 백엔드에 전달해 화면에 그린다. */
    void Draw(HudRenderTimings* timings);

private:
    struct PendingGpuQuery
    {
        std::uint32_t query = 0;
        std::uint64_t frameIndex = 0;
        bool pending = false;
    };

    void CollectCompletedGpuTimings(HudRenderTimings* timings);

    bool initialized_ = false;
    std::string layoutFile_;
    std::chrono::steady_clock::time_point interfaceBuildStarted_{};
    std::array<PendingGpuQuery, 4> backendGpuQueries_{};
};
}
