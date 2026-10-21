#include "graphics/VertexArray.h"

#include "graphics/GlApi.h"

namespace aerosimlab::graphics
{

VertexArray::VertexArray()
{
    // Vertex Array Object(VAO) ID만 만든다. 정점 속성 형식과 index buffer 연결은 Mesh가 이 VAO를 선택한 뒤 기록한다.
    glGenVertexArrays(1, &m_RendererID);
}

VertexArray::~VertexArray()
{
    if (m_RendererID != 0)
    {
        glDeleteVertexArrays(1, &m_RendererID);
        m_RendererID = 0;
    }
}

void VertexArray::Bind() const
{
    glBindVertexArray(m_RendererID);
}

void VertexArray::UnBind() const
{
    // 현재 VAO 선택만 해제한다. 그 안에 저장된 정점 속성 설명과 index buffer 연결은 그대로 남는다.
    glBindVertexArray(0);
}

} // namespace aerosimlab::graphics
