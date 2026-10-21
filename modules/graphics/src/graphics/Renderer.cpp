#include "graphics/Renderer.h"

#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "graphics/Shader.h"
#ifndef __EMSCRIPTEN__
#include "graphics/MultisampleFramebuffer.h"
#endif

#include "graphics/GlApi.h"

#include <cstdint>
#include <algorithm>
#ifndef NDEBUG
#include <iostream>
#include <mutex>
#include <unordered_set>
#endif

namespace aerosimlab::graphics
{
namespace
{
struct RendererDefaults final
{
    inline static constexpr int multisampleCount = 4;
    inline static constexpr float clearRed = 0.14F;
    inline static constexpr float clearGreen = 0.15F;
    inline static constexpr float clearBlue = 0.16F;
    inline static constexpr float clearAlpha = 1.0F;
};

// 이 이름은 셰이더 변수와 같아야 하므로 C++ 변수 이름과 별도로 모아 둔다.
struct RendererUniformNames final
{
    inline static constexpr char model[] = "u_Model";
    inline static constexpr char view[] = "u_View";
    inline static constexpr char projection[] = "u_Projection";
    inline static constexpr char baseColorFactor[] = "u_BaseColorFactor";
    inline static constexpr char metallicFactor[] = "u_MetallicFactor";
    inline static constexpr char roughnessFactor[] = "u_RoughnessFactor";
    inline static constexpr char cameraPosition[] = "u_CameraPosition";
    inline static constexpr char lightDirection[] = "u_LightDirection";
};

#ifndef NDEBUG
const char* OpenGlErrorName(GLenum error)
{
    switch (error)
    {
    case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
    case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
#ifdef GL_INVALID_FRAMEBUFFER_OPERATION
    case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
#endif
    default: return "unknown OpenGL error";
    }
}

void ReportOpenGlErrors()
{
    static std::mutex reportedErrorsMutex;
    static std::unordered_set<GLenum> reportedErrors;

    for (GLenum error = glGetError(); error != GL_NO_ERROR; error = glGetError())
    {
        std::lock_guard<std::mutex> lock(reportedErrorsMutex);
        if (reportedErrors.insert(error).second)
            std::cerr << "Renderer OpenGL error: " << OpenGlErrorName(error)
                << " (0x" << std::hex << error << std::dec << ")\n";
    }
}
#endif
}

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init(int framebufferWidth, int framebufferHeight)
{
    // 깊이 검사를 끄면 멀리 있는 삼각형도 그린 순서가 나중이라는 이유만으로 가까운 물체 앞에 표시된다.
    glEnable(GL_DEPTH_TEST);
#ifndef __EMSCRIPTEN__
    glEnable(GL_MULTISAMPLE);

    // 화면 그리기 단계(Main pass)는 픽셀마다 네 표본을 가진 대상을 사용한다. 프레임 끝에 표본을 합쳐 Window 기본 framebuffer로 복사한다.
    m_MSAAFramebuffer = std::make_unique<MultisampleFramebuffer>(
        framebufferWidth,
        framebufferHeight,
        RendererDefaults::multisampleCount);
#else
    m_FramebufferWidth = framebufferWidth;
    m_FramebufferHeight = framebufferHeight;
#endif

    m_LightDirection = glm::normalize(m_LightDirection);
}

void Renderer::BeginFrame()
{
    glDisable(GL_SCISSOR_TEST);
#ifdef __EMSCRIPTEN__
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_FramebufferWidth, m_FramebufferHeight);
#else
    m_MSAAFramebuffer->Bind();
#endif

    glClearColor(
        RendererDefaults::clearRed,
        RendererDefaults::clearGreen,
        RendererDefaults::clearBlue,
        RendererDefaults::clearAlpha);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::EndFrame()
{
    glDisable(GL_SCISSOR_TEST);
#ifdef __EMSCRIPTEN__
    return;
#else
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_MSAAFramebuffer->GetWidth(), m_MSAAFramebuffer->GetHeight());
    glClearColor(
        RendererDefaults::clearRed,
        RendererDefaults::clearGreen,
        RendererDefaults::clearBlue,
        RendererDefaults::clearAlpha);
    glClear(GL_COLOR_BUFFER_BIT);
    m_MSAAFramebuffer->ResolveToDefault(
        m_SceneViewport.x, m_SceneViewport.y,
        m_SceneViewport.z, m_SceneViewport.w);
#endif
}

void Renderer::Resize(int width, int height)
{
#ifdef __EMSCRIPTEN__
    m_FramebufferWidth = width;
    m_FramebufferHeight = height;
    m_SceneViewport = {0, 0, width, height};
#else
    if (m_MSAAFramebuffer)
    {
        m_MSAAFramebuffer->Resize(width, height);
        m_SceneViewport = {0, 0, width, height};
    }
#endif
}

void Renderer::SetSceneViewport(int x, int y, int width, int height)
{
#ifdef __EMSCRIPTEN__
    if (width <= 0 || height <= 0)
        return;
    const int viewportWidth = m_FramebufferWidth;
    const int viewportHeight = m_FramebufferHeight;
#else
    if (!m_MSAAFramebuffer || width <= 0 || height <= 0)
        return;
    const int viewportWidth = m_MSAAFramebuffer->GetWidth();
    const int viewportHeight = m_MSAAFramebuffer->GetHeight();
#endif

    const int left = std::clamp(x, 0, viewportWidth);
    const int bottom = std::clamp(y, 0, viewportHeight);
    const int right = static_cast<int>(std::clamp<std::int64_t>(
        static_cast<std::int64_t>(x) + width, left, viewportWidth));
    const int top = static_cast<int>(std::clamp<std::int64_t>(
        static_cast<std::int64_t>(y) + height, bottom, viewportHeight));
    m_SceneViewport = {left, bottom, right - left, top - bottom};
}

void Renderer::Draw(
    const glm::mat4& model,
    const Mesh& mesh,
    const Material& material,
    Shader& shader,
    const glm::mat4& view,
    const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    if (m_SceneViewport.z > 0 && m_SceneViewport.w > 0)
    {
        glViewport(m_SceneViewport.x, m_SceneViewport.y, m_SceneViewport.z, m_SceneViewport.w);
        glEnable(GL_SCISSOR_TEST);
        glScissor(m_SceneViewport.x, m_SceneViewport.y, m_SceneViewport.z, m_SceneViewport.w);
    }

    shader.Bind();
    shader.SetMat4(RendererUniformNames::model, model);
    shader.SetMat4(RendererUniformNames::view, view);
    shader.SetMat4(RendererUniformNames::projection, projection);

    shader.SetFloat4(RendererUniformNames::baseColorFactor, material.BaseColorFactor());
    shader.SetFloat(RendererUniformNames::metallicFactor, material.MetallicFactor());
    shader.SetFloat(RendererUniformNames::roughnessFactor, material.RoughnessFactor());

    shader.SetFloat3(RendererUniformNames::cameraPosition, cameraPosition);
    shader.SetFloat3(RendererUniformNames::lightDirection, m_LightDirection);

    mesh.Bind();

    glDrawElements(
        GL_TRIANGLES,
        static_cast<GLsizei>(mesh.GetIndexCount()),
        GL_UNSIGNED_INT,
        nullptr);

    mesh.UnBind();
    shader.UnBind();

#ifndef NDEBUG
    ReportOpenGlErrors();
#endif

}

} // namespace aerosimlab::graphics
