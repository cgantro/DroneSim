#include "graphics/Renderer.h"

#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "graphics/Shader.h"
#include "graphics/Texture2D.h"
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
    inline static constexpr float clearRed = 0.50F;
    inline static constexpr float clearGreen = 0.68F;
    inline static constexpr float clearBlue = 0.79F;
    inline static constexpr float clearAlpha = 1.0F;
};

// 이 이름은 셰이더 변수와 같아야 하므로 C++ 변수 이름과 별도로 모아 둔다.
struct RendererUniformNames final
{
    inline static constexpr char model[] = "u_Model";
    inline static constexpr char view[] = "u_View";
    inline static constexpr char projection[] = "u_Projection";
    inline static constexpr char baseColorFactor[] = "u_BaseColorFactor";
    inline static constexpr char baseTexture[] = "u_BaseTexture";
    inline static constexpr char useBaseTexture[] = "u_UseBaseTexture";
    inline static constexpr char triplanarTexture[] = "u_TriplanarTexture";
    inline static constexpr char terrainSurface[] = "u_TerrainSurface";
    inline static constexpr char flowingWater[] = "u_FlowingWater";
    inline static constexpr char timeSeconds[] = "u_TimeSeconds";
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
#ifdef __EMSCRIPTEN__
    m_FramebufferWidth = framebufferWidth;
    m_FramebufferHeight = framebufferHeight;
#else
    (void)framebufferWidth;
    (void)framebufferHeight;
    glEnable(GL_MULTISAMPLE);
#endif

    m_LightDirection = glm::normalize(m_LightDirection);
}

#ifdef __EMSCRIPTEN__
void Renderer::BeginFrame()
{
    glDisable(GL_SCISSOR_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_FramebufferWidth, m_FramebufferHeight);

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
    return;
}
#endif

#ifndef __EMSCRIPTEN__
void Renderer::BeginView(MultisampleFramebuffer& target)
{
    target.Bind();
    m_SceneViewport = {0, 0, target.GetWidth(), target.GetHeight()};
    glDisable(GL_SCISSOR_TEST);
    glClearColor(RendererDefaults::clearRed, RendererDefaults::clearGreen,
        RendererDefaults::clearBlue, RendererDefaults::clearAlpha);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::EndView(MultisampleFramebuffer& target)
{
    glDisable(GL_SCISSOR_TEST);
    target.ResolveToTexture();
    m_SceneViewport = {0, 0, 0, 0};
}
#endif

#ifdef __EMSCRIPTEN__
void Renderer::ReadRgbPixels(int x, int y, int width, int height,
    std::vector<std::uint8_t>& pixels) const
{
    if (width <= 0 || height <= 0)
    {
        pixels.clear();
        return;
    }

    pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U);
    GLint previousPackAlignment = 4;
    glGetIntegerv(GL_PACK_ALIGNMENT, &previousPackAlignment);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glPixelStorei(GL_PACK_ALIGNMENT, previousPackAlignment);
}

void Renderer::Resize(int width, int height)
{
    m_FramebufferWidth = width;
    m_FramebufferHeight = height;
    m_SceneViewport = {0, 0, width, height};
}
#endif

void Renderer::SetSceneViewport(int x, int y, int width, int height)
{
#ifdef __EMSCRIPTEN__
    if (width <= 0 || height <= 0)
        return;
    const int viewportWidth = m_FramebufferWidth;
    const int viewportHeight = m_FramebufferHeight;
    const int left = std::clamp(x, 0, viewportWidth);
    const int bottom = std::clamp(y, 0, viewportHeight);
    const int right = static_cast<int>(std::clamp<std::int64_t>(
        static_cast<std::int64_t>(x) + width, left, viewportWidth));
    const int top = static_cast<int>(std::clamp<std::int64_t>(
        static_cast<std::int64_t>(y) + height, bottom, viewportHeight));
    m_SceneViewport = {left, bottom, right - left, top - bottom};
#else
    (void)x;
    (void)y;
    (void)width;
    (void)height;
#endif
}

void Renderer::SetAnimationTimeSeconds(float seconds) noexcept
{
    m_AnimationTimeSeconds = seconds;
}

void Renderer::Draw(
    const glm::mat4& model,
    const Mesh& mesh,
    const Material& material,
    Shader& shader,
    const glm::mat4& view,
    const glm::mat4& projection,
    const glm::vec3& cameraPosition,
    const Texture2D* baseTexture,
    bool triplanarTexture,
    bool terrainSurface)
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
    shader.SetInt(RendererUniformNames::baseTexture, 0);
    shader.SetInt(RendererUniformNames::useBaseTexture, baseTexture != nullptr ? 1 : 0);
    shader.SetInt(RendererUniformNames::triplanarTexture, triplanarTexture ? 1 : 0);
    shader.SetInt(RendererUniformNames::terrainSurface, terrainSurface ? 1 : 0);
    shader.SetInt(RendererUniformNames::flowingWater,
        material.Effect() == Material::SurfaceEffect::FlowingWater ? 1 : 0);
    shader.SetFloat(RendererUniformNames::timeSeconds, m_AnimationTimeSeconds);
    shader.SetFloat(RendererUniformNames::metallicFactor, material.MetallicFactor());
    shader.SetFloat(RendererUniformNames::roughnessFactor, material.RoughnessFactor());

    shader.SetFloat3(RendererUniformNames::cameraPosition, cameraPosition);
    shader.SetFloat3(RendererUniformNames::lightDirection, m_LightDirection);

    if (baseTexture != nullptr)
        baseTexture->Bind();

    mesh.Bind();

    glDrawElements(
        GL_TRIANGLES,
        static_cast<GLsizei>(mesh.GetIndexCount()),
        GL_UNSIGNED_INT,
        nullptr);

    mesh.UnBind();
    if (baseTexture != nullptr)
        baseTexture->Unbind();
    shader.UnBind();

#ifndef NDEBUG
    ReportOpenGlErrors();
#endif

}

} // namespace aerosimlab::graphics
