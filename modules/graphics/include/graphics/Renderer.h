#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <memory>
#include <vector>

namespace aerosimlab::graphics
{

class MultisampleFramebuffer;
class Material;
class Mesh;
class Shader;
class Texture2D;

class Renderer
{
public:
    Renderer();
    ~Renderer();

    void Init(int framebufferWidth, int framebufferHeight);
    void BeginFrame();
    void EndFrame();
    /** @brief 화면에 합성된 RGB 픽셀을 지정한 배열에 저장한다. OpenGL 좌표 기준으로 아래 행부터 기록한다. */
    void ReadRgbPixels(int x, int y, int width, int height, std::vector<std::uint8_t>& pixels) const;
    void Resize(int width, int height);
    void SetSceneViewport(int x, int y, int width, int height);
    void Draw(
        const glm::mat4& model,
        const Mesh& mesh,
        const Material& material,
        Shader& shader,
        const glm::mat4& view,
        const glm::mat4& projection,
        const glm::vec3& cameraPosition,
        const Texture2D* baseTexture = nullptr,
        bool triplanarTexture = false,
        bool terrainSurface = false);

private:
#ifndef __EMSCRIPTEN__
    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;
#else
    int m_FramebufferWidth = 0;
    int m_FramebufferHeight = 0;
#endif
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};
    glm::ivec4 m_SceneViewport{0, 0, 0, 0};
};

} // namespace aerosimlab::graphics
