#include "graphics/Material.h"

namespace aerosimlab::graphics
{

// Material은 전달된 Factor 값을 그대로 저장한다. 값의 유효 범위 제한은 Metallic/Roughness를 해석하는 Shader가 적용한다.
Material::Material(
    const glm::vec4& baseColorFactor,
    float metallicFactor,
    float roughnessFactor)
    : m_BaseColorFactor(baseColorFactor),
      m_MetallicFactor(metallicFactor),
      m_RoughnessFactor(roughnessFactor)
{
}

const glm::vec4& Material::BaseColorFactor() const
{
    return m_BaseColorFactor;
}

float Material::MetallicFactor() const
{
    return m_MetallicFactor;
}

float Material::RoughnessFactor() const
{
    return m_RoughnessFactor;
}

} // namespace aerosimlab::graphics
