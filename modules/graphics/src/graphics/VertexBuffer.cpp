#include "graphics/VertexBuffer.h"

#include "graphics/GlApi.h"

#include <stdexcept>

namespace dronesim::graphics
{

VertexBuffer::VertexBuffer(const void* data, GLsizeiptr size)
{
    if (size < 0)
        throw std::invalid_argument("GPU buffer size must not be negative.");

    // 지정된 CPU 메모리 구간을 GPU 메모리에 복사한다. 이후 그리기 명령은 이 GPU 사본에서 정점 데이터를 읽는다.
    glGenBuffers(1, &m_RendererID);
    glBindBuffer(GL_ARRAY_BUFFER, m_RendererID);

    // GL_STATIC_DRAW는 초기 업로드 뒤 내용을 자주 바꾸지 않는다는 사용 패턴을 GPU에 알려 준다.
    glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
}

VertexBuffer::~VertexBuffer()
{
    if (m_RendererID != 0)
    {
        glDeleteBuffers(1, &m_RendererID);
        m_RendererID = 0;
    }
}

void VertexBuffer::Bind() const
{
    glBindBuffer(GL_ARRAY_BUFFER, m_RendererID);
}

void VertexBuffer::UnBind() const
{
    // 현재 GL_ARRAY_BUFFER 선택만 해제한다. GPU에 복사된 데이터와 VAO에 기록한 정점 속성 설명은 유지된다.
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

} // namespace dronesim::graphics
