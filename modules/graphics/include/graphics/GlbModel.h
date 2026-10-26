#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace aerosimlab::graphics
{
class Material;
class Mesh;
class Renderer;
class Shader;
class Texture2D;

/** @brief GLB 장면의 노드 배치와 각 메시의 PBR 재질·이미지를 함께 보관한다. */
class GlbModel final
{
public:
    [[nodiscard]] static std::unique_ptr<GlbModel> Load(const std::string& path);

    GlbModel(const GlbModel&) = delete;
    GlbModel& operator=(const GlbModel&) = delete;
    GlbModel(GlbModel&&) = delete;
    GlbModel& operator=(GlbModel&&) = delete;
    ~GlbModel();

    /**
     * @brief 장면에 있는 모든 메시를 각 노드 변환과 재질로 그린다.
     * @param modelTransform 모델 전체에 적용할 바깥 변환.
     * @param fallbackMaterial GLB 메시가 재질을 지정하지 않았을 때 사용할 재질.
     * @param fallbackTexture GLB 메시가 재질을 지정하지 않았을 때 사용할 이미지.
     * @param fallbackTriplanar GLB 재질이 없는 메시에서 기본 이미지를 삼면 투영할지 여부.
     * @details GLB 재질이 있으면 그 재질과 GLB 이미지 설정을 우선한다. 재질은 있지만 기본색 이미지가 없으면 이미지 없이 그린다.
     */
    void Draw(const glm::mat4& modelTransform, Renderer& renderer, Shader& shader,
        const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition,
        const Material* fallbackMaterial = nullptr,
        const Texture2D* fallbackTexture = nullptr,
        bool fallbackTriplanar = false) const;

    [[nodiscard]] std::size_t PartCount() const noexcept;

private:
    GlbModel() = default;
    struct Part;
    std::vector<std::unique_ptr<Part>> parts_;
};
} // namespace aerosimlab::graphics
