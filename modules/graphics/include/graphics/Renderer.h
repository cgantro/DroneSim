#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace aerosimlab::graphics
{

#ifndef __EMSCRIPTEN__
class MultisampleFramebuffer;
#endif
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
#ifndef __EMSCRIPTEN__
    void BeginView(MultisampleFramebuffer& target);
    void EndView(MultisampleFramebuffer& target);
#else
    void BeginFrame();
    void EndFrame();
    void ReadRgbPixels(int x, int y, int width, int height, std::vector<std::uint8_t>& pixels) const;
    void Resize(int width, int height);
#endif
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

#ifdef __EMSCRIPTEN__
private:
    int m_FramebufferWidth = 0;
    int m_FramebufferHeight = 0;
#endif
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};
    glm::ivec4 m_SceneViewport{0, 0, 0, 0};
};

} // namespace aerosimlab::graphics
