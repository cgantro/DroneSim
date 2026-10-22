#include "graphics/Material.h"

namespace aerosimlab::graphics
{

// 재질은 색과 금속성·거칠기 값을 보관한다. 실제로 쓸 수 있는 범위는 셰이더가 정한다.
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
