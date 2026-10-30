#include "graphics/VertexArray.h"

#include "graphics/GlApi.h"

namespace dronesim::graphics
{

VertexArray::VertexArray()
{
    // VAO(정점 읽기 설정을 저장하는 GPU 객체)를 만든다. 정점 형식과 연결 번호는 Mesh가 선택한 뒤 등록한다.
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
    // 현재 VAO 선택만 해제한다. 저장된 정점 읽기 방법과 삼각형 연결 번호는 유지된다.
    glBindVertexArray(0);
}

} // namespace dronesim::graphics
