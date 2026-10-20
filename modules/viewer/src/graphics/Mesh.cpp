#include "graphics/Mesh.h"

#include "graphics/IndexBuffer.h"
#include "graphics/VertexArray.h"
#include "graphics/VertexBuffer.h"

#include "graphics/GlApi.h"

#include <cstddef>
#include <limits>
#include <stdexcept>

namespace grasplink::graphics
{


Mesh::Mesh(const ::grasplink::model::Vertex* vertices, std::uint32_t vertexCount,
           const std::uint32_t* indices, std::uint32_t indexCount)
{
    if (vertices == nullptr || vertexCount == 0U) { throw std::invalid_argument("Mesh has no vertices"); }
    if (indices == nullptr || indexCount == 0U) { throw std::invalid_argument("Mesh has no indices"); }

    // Vertex Array Object(VAO)를 먼저 선택한다. 이후 지정하는 정점 속성 형식과 index buffer 연결이 이 Mesh의 상태로 저장된다.
    vertexArray_ = std::make_unique<VertexArray>();
    vertexArray_->Bind();

    // GPU buffer 업로드 함수는 byte 수를 받으므로 Vertex 개수에 Vertex 구조체 크기를 곱해 전달한다.
    const std::size_t maximumBufferBytes = std::numeric_limits<std::uint32_t>::max();
    if (vertexCount > maximumBufferBytes / sizeof(::grasplink::model::Vertex))
    {
        throw std::runtime_error("Mesh vertex data exceeds the GPU buffer limit");
    }
    const auto byteCount = static_cast<std::uint32_t>(
        sizeof(::grasplink::model::Vertex) * static_cast<std::size_t>(vertexCount));
    vertexBuffer_ = std::make_unique<VertexBuffer>(vertices, byteCount);
    vertexBuffer_->Bind();

    // index buffer 연결은 현재 VAO의 일부로 저장된다. VAO를 선택한 상태에서 buffer를 만들고 올려 이후 그리기에 쓰도록 한다.
    indexBuffer_ = std::make_unique<IndexBuffer>(indices, indexCount);

    // Shader 입력 위치 0, 1, 2에 정점 위치(vec3), 표면 법선(vec3), Texture 좌표(vec2)를 연결한다.
    // stride는 다음 정점까지의 byte 간격이고 offsetof는 한 Vertex 안에서 각 값이 시작하는 byte 위치다.
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(::grasplink::model::Vertex), reinterpret_cast<void*>(offsetof(::grasplink::model::Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(::grasplink::model::Vertex), reinterpret_cast<void*>(offsetof(::grasplink::model::Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(::grasplink::model::Vertex), reinterpret_cast<void*>(offsetof(::grasplink::model::Vertex, texCoord)));

    // Vertex Buffer Object(VBO)의 현재 선택을 해제해도 VAO에 기록된 정점 속성 설명은 남는다.
    // index buffer 연결은 VAO 내부 상태이므로 여기서 해제하지 않는다.
    vertexBuffer_->UnBind();
    vertexArray_->UnBind();
}

Mesh::~Mesh() = default;

// 그리기 명령은 index buffer의 uint32 원소를 읽어 VAO가 참조하는 정점 buffer의 원소를 선택한다.
void Mesh::Bind() const { vertexArray_->Bind(); }
void Mesh::UnBind() const { vertexArray_->UnBind(); }
std::uint32_t Mesh::GetIndexCount() const { return indexBuffer_->GetCount(); }

std::unique_ptr<Mesh> Mesh::CreateCube(float sideLengthMeters)
{
    if (!std::isfinite(sideLengthMeters) || sideLengthMeters <= 0.0F)
        throw std::invalid_argument("Cube side length must be finite and > 0");

    // 면마다 네 정점을 따로 둔다. 그래서 맞닿은 면도 서로 다른 표면 법선과 Texture 사각형 좌표를 가질 수 있다.
    ::grasplink::model::Vertex vertices[] = {
        {{-0.5F, -0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F}},
        {{ 0.5F, -0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F}},
        {{ 0.5F,  0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {1.0F, 1.0F}},
        {{-0.5F,  0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F}},
        {{ 0.5F, -0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {0.0F, 0.0F}},
        {{-0.5F, -0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {1.0F, 0.0F}},
        {{-0.5F,  0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {1.0F, 1.0F}},
        {{ 0.5F,  0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {0.0F, 1.0F}},
        {{-0.5F, -0.5F, -0.5F}, {-1.0F, 0.0F, 0.0F}, {0.0F, 0.0F}},
        {{-0.5F, -0.5F,  0.5F}, {-1.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
        {{-0.5F,  0.5F,  0.5F}, {-1.0F, 0.0F, 0.0F}, {1.0F, 1.0F}},
        {{-0.5F,  0.5F, -0.5F}, {-1.0F, 0.0F, 0.0F}, {0.0F, 1.0F}},
        {{ 0.5F, -0.5F,  0.5F}, {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F}},
        {{ 0.5F, -0.5F, -0.5F}, {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
        {{ 0.5F,  0.5F, -0.5F}, {1.0F, 0.0F, 0.0F}, {1.0F, 1.0F}},
        {{ 0.5F,  0.5F,  0.5F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F}},
        {{-0.5F,  0.5F,  0.5F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F}},
        {{ 0.5F,  0.5F,  0.5F}, {0.0F, 1.0F, 0.0F}, {1.0F, 0.0F}},
        {{ 0.5F,  0.5F, -0.5F}, {0.0F, 1.0F, 0.0F}, {1.0F, 1.0F}},
        {{-0.5F,  0.5F, -0.5F}, {0.0F, 1.0F, 0.0F}, {0.0F, 1.0F}},
        {{-0.5F, -0.5F, -0.5F}, {0.0F,-1.0F, 0.0F}, {0.0F, 0.0F}},
        {{ 0.5F, -0.5F, -0.5F}, {0.0F,-1.0F, 0.0F}, {1.0F, 0.0F}},
        {{ 0.5F, -0.5F,  0.5F}, {0.0F,-1.0F, 0.0F}, {1.0F, 1.0F}},
        {{-0.5F, -0.5F,  0.5F}, {0.0F,-1.0F, 0.0F}, {0.0F, 1.0F}}
    };
    // 각 사각형 면을 삼각형 두 개로 나눈다. index 값은 byte 위치가 아니라 vertices 배열의 정점 번호다.
    const std::uint32_t indices[] = {
        0,1,2, 2,3,0, 4,5,6, 6,7,4, 8,9,10, 10,11,8,
        12,13,14, 14,15,12, 16,17,18, 18,19,16, 20,21,22, 22,23,20
    };
    // 원점 중심의 단위 정육면체 좌표를 각 축 방향으로 sideLengthMeters / 2만큼 늘려 요청한 변 길이를 만든다.
    for (::grasplink::model::Vertex& vertex : vertices)
        vertex.position *= sideLengthMeters;

    return std::make_unique<Mesh>(vertices, 24U, indices, 36U);
}

} // namespace grasplink::graphics
