#include "graphics/IndexBuffer.h"

#include "graphics/GlApi.h"

#include <cstddef>
#include <limits>
#include <stdexcept>

namespace grasplink::graphics
{


IndexBuffer::IndexBuffer(const uint32_t* indices, uint32_t cnt) : m_Count(cnt){
    if(indices == nullptr) throw std::runtime_error(
        "IndexBuffer indices must not be null"
    );

    if(cnt == 0) throw std::runtime_error(
        "IndexBuffer count  must be Greater than 0"
    );

    if (cnt > static_cast<uint32_t>(std::numeric_limits<GLsizei>::max()))
        throw std::runtime_error("IndexBuffer count exceeds the OpenGL draw limit");
    if (static_cast<std::size_t>(cnt) >
        static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(uint32_t))
        throw std::runtime_error("IndexBuffer data exceeds the GPU buffer limit");

    const GLsizeiptr byteCount = static_cast<GLsizeiptr>(cnt) *
        static_cast<GLsizeiptr>(sizeof(uint32_t));

    // CPU index 배열과 별도로 GPU가 draw 때 읽을 index buffer 객체를 만든다.
   glGenBuffers(1,&m_RendererID);

   // Element Buffer Object(EBO) 연결은 현재 VAO의 상태로 저장되므로 Mesh가 먼저 VAO를 선택해야 한다.
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_RendererID);

   // OpenGL 메모리 할당 크기는 byte로 전달하지만 count와 그리기 함수 인수는 index 원소 수를 뜻한다.
   glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        byteCount,
        indices,
        GL_STATIC_DRAW
   );

    // VAO가 선택된 상태에서 EBO ID 0을 연결하면 그 VAO에 기록된 EBO 연결도 해제된다.
}

IndexBuffer::~IndexBuffer(){
    // GPU 자원은 생성 때 사용한 OpenGL context가 살아 있고 현재 스레드에서 활성화되어 있을 때 해제해야 한다.
    if(m_RendererID != 0){
        glDeleteBuffers(1,&m_RendererID);
        m_RendererID = 0;
    }
}

void IndexBuffer::Bind() const {glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_RendererID);}

void IndexBuffer::UnBind() const{
    // GL_ELEMENT_ARRAY_BUFFER 연결은 VAO에 저장된다. 따라서 0을 연결하면 현재 VAO가 기억하던 EBO도 지워진다.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
}

} // namespace grasplink::graphics
