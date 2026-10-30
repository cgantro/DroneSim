#include "graphics/IndexBuffer.h"

#include "graphics/GlApi.h"

#include <cstddef>
#include <limits>
#include <stdexcept>

namespace dronesim::graphics
{

IndexBuffer::IndexBuffer(const std::uint32_t* indices, std::uint32_t count)
    : m_Count(count)
{
    if (indices == nullptr)
        throw std::runtime_error("IndexBuffer indices must not be null");

    if (count == 0)
        throw std::runtime_error("IndexBuffer count  must be Greater than 0");

    if (count > static_cast<std::uint32_t>(std::numeric_limits<GLsizei>::max()))
        throw std::runtime_error("IndexBuffer count exceeds the OpenGL draw limit");
    if (static_cast<std::size_t>(count) >
        static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(std::uint32_t))
        throw std::runtime_error("IndexBuffer data exceeds the GPU buffer limit");

    const GLsizeiptr byteCount = static_cast<GLsizeiptr>(count) *
        static_cast<GLsizeiptr>(sizeof(std::uint32_t));

    glGenBuffers(1, &m_RendererID);

    // 삼각형을 만들 정점 번호는 현재 VAO에 저장된다. Mesh가 사용할 VAO를 먼저 선택해야 한다.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RendererID);

    // GPU 저장 공간은 byte 단위로 만들지만, 그리기 함수에는 정점 번호의 개수를 전달한다.
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        byteCount,
        indices,
        GL_STATIC_DRAW
    );

    // 연결을 0으로 바꾸면 현재 VAO의 정점 번호 설정도 지워지므로 그대로 둔다.
}

IndexBuffer::~IndexBuffer()
{
    // GPU 자원은 생성 때 사용한 OpenGL context가 살아 있고 현재 스레드에서 활성화되어 있을 때 해제해야 한다.
    if (m_RendererID != 0)
    {
        glDeleteBuffers(1, &m_RendererID);
        m_RendererID = 0;
    }
}

void IndexBuffer::Bind() const
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RendererID);
}

void IndexBuffer::UnBind() const
{
    // 연결 정보는 VAO에 저장된다. 여기에 0을 연결하면 VAO의 정점 번호 설정도 지워진다.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

} // namespace dronesim::graphics
