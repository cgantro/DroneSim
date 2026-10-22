#pragma once

#include <glm/glm.hpp>
namespace aerosimlab::graphics
{

/**
 * @brief 표면에 칠할 기본색과 빛 반사 모양을 정하는 값을 보관한다.
 * @details
 * 재질은 Mesh 표면의 색과 빛 반응을 정한다. Renderer가 이 값을 GPU Shader에 전달한다.
 * 기본색은 RGBA 값으로 저장한다. Renderer는 이 색과 금속성·거칠기 값을 간단한 조명 계산에 전달한다.
 * GPU는 금속성과 거칠기를 사용해 표면의 직접광과 반사를 계산한다. 주변 빛과 환경 반사광은 간단히 근사한다.
 */
class Material final
{
public:
    /**
     * @brief 표면 기본색과 반사 특성을 정한다.
     * @param baseColorFactor 표면색·투명도 배율. RGB는 빛 계산용 선형 색이고 alpha는 투명도 값이다.
     * @param metallicFactor 금속처럼 반사하는 정도 [0..1], 단위 없음.
     * @param roughnessFactor 반사 경계가 흐려지는 정도 [0..1], 단위 없음.
     * @details 이 생성자는 값을 그대로 보관한다. Scene.glsl에서 금속성은 0~1, 거칠기는 0.05~1로 제한해 조명 계산에 쓴다.
     */
    Material(const glm::vec4& baseColorFactor, float metallicFactor, float roughnessFactor);

    const glm::vec4& BaseColorFactor() const;

    float MetallicFactor() const;

    float RoughnessFactor() const;

private:
    // 기본값 (1,1,1,1)은 화면 색을 바꾸지 않는다.
    glm::vec4 m_BaseColorFactor{1.0F};

    // 0은 비금속, 1은 금속이다. Renderer가 Scene.glsl에 전달해 반사색 계산에 사용한다.
    float m_MetallicFactor = 1.0F;

    // 0에 가까우면 작은 밝은 반사, 1에 가까우면 넓고 흐린 반사가 된다. Scene.glsl은 0.05보다 작은 값을 막는다.
    float m_RoughnessFactor = 1.0F;

};

} // namespace aerosimlab::graphics
