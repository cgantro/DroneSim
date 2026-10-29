#include "graphics/Mesh.h"

#include "graphics/IndexBuffer.h"
#include "graphics/VertexArray.h"
#include "graphics/VertexBuffer.h"

#include "graphics/GlApi.h"

#include <cstddef>
#include <limits>
#include <stdexcept>

namespace aerosimlab::graphics
{
Mesh::Mesh(const Vertex* vertices, std::uint32_t vertexCount,
           const std::uint32_t* indices, std::uint32_t indexCount)
{
    if (vertices == nullptr || vertexCount == 0U) { throw std::invalid_argument("Mesh has no vertices"); }
    if (indices == nullptr || indexCount == 0U) { throw std::invalid_argument("Mesh has no indices"); }

    const std::size_t maximumBufferBytes = static_cast<std::size_t>(
        std::numeric_limits<GLsizeiptr>::max());
    if (static_cast<std::size_t>(vertexCount) > maximumBufferBytes / sizeof(Vertex))
    {
        throw std::runtime_error("Mesh vertex data exceeds the GPU buffer limit");
    }
    const GLsizeiptr byteCount = static_cast<GLsizeiptr>(
        sizeof(Vertex) * static_cast<std::size_t>(vertexCount));
    if (indexCount > static_cast<std::uint32_t>(std::numeric_limits<GLsizei>::max()))
        throw std::runtime_error("Mesh index count exceeds the OpenGL draw limit");
    if (static_cast<std::size_t>(indexCount) >
        static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(std::uint32_t))
        throw std::runtime_error("Mesh index data exceeds the GPU buffer limit");

    // 정점 읽기 설정을 저장할 VAO를 먼저 선택한다. 이후 지정하는 정점 형식과 연결 번호가 여기에 저장된다.
    vertexArray_ = std::make_unique<VertexArray>();
    vertexArray_->Bind();

    // GPU buffer 업로드 함수는 byte 수를 받으므로 Vertex 개수에 Vertex 구조체 크기를 곱해 전달한다.
    vertexBuffer_ = std::make_unique<VertexBuffer>(vertices, byteCount);
    vertexBuffer_->Bind();

    // 삼각형 연결 번호는 VAO에 저장된다. VAO를 선택한 상태에서 번호 버퍼를 만들고 데이터를 올린다.
    indexBuffer_ = std::make_unique<IndexBuffer>(indices, indexCount);

    // 셰이더 입력 0과 1에 정점 위치와 표면 방향을 연결한다. stride는 다음 정점까지의 byte 간격이다.
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, textureCoordinate)));

    // 정점 버퍼의 현재 선택만 해제한다. VAO에 저장된 정점 읽기 방법은 그대로 남는다.
    // 삼각형 연결 번호도 VAO에 저장되므로 여기서 별도로 해제하지 않는다.
    vertexBuffer_->UnBind();
    vertexArray_->UnBind();
}

Mesh::~Mesh() = default;

// 그리기 명령은 삼각형 연결 번호를 따라 VAO가 참조하는 정점들을 읽는다.
void Mesh::Bind() const { vertexArray_->Bind(); }
void Mesh::UnBind() const { vertexArray_->UnBind(); }
std::uint32_t Mesh::GetIndexCount() const { return indexBuffer_->GetCount(); }

void Mesh::SetInstances(std::span<const glm::mat4> transforms)
{
    if (transforms.empty())
    {
        instanceBuffer_.reset();
        instanceCount_ = 0;
        return;
    }
    if (transforms.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())
        || transforms.size() > static_cast<std::size_t>(
            std::numeric_limits<GLsizeiptr>::max()) / sizeof(glm::mat4))
        throw std::runtime_error("Mesh instance data exceeds the OpenGL buffer limit");

    instanceBuffer_ = std::make_unique<VertexBuffer>(transforms.data(),
        static_cast<GLsizeiptr>(transforms.size() * sizeof(glm::mat4)));
    vertexArray_->Bind();
    instanceBuffer_->Bind();
    for (GLuint column = 0; column < 4U; ++column)
    {
        const GLuint location = 3U + column;
        glEnableVertexAttribArray(location);
        glVertexAttribPointer(location, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4),
            reinterpret_cast<void*>(static_cast<std::uintptr_t>(
                column * sizeof(glm::vec4))));
        glVertexAttribDivisor(location, 1);
    }
    instanceBuffer_->UnBind();
    vertexArray_->UnBind();
    instanceCount_ = static_cast<std::uint32_t>(transforms.size());
}

std::uint32_t Mesh::InstanceCount() const noexcept
{
    return instanceCount_;
}

std::unique_ptr<Mesh> Mesh::CreateCube(float sideLengthMeters)
{
    if (!std::isfinite(sideLengthMeters) || sideLengthMeters <= 0.0F)
        throw std::invalid_argument("Cube side length must be finite and > 0");

    // 면마다 네 정점을 따로 둬서 서로 다른 표면 방향을 보존한다.
    constexpr glm::vec3 corners[6][4] = {
        {{-0.5F, -0.5F, 0.5F}, {0.5F, -0.5F, 0.5F}, {0.5F, 0.5F, 0.5F}, {-0.5F, 0.5F, 0.5F}},
        {{0.5F, -0.5F, -0.5F}, {-0.5F, -0.5F, -0.5F}, {-0.5F, 0.5F, -0.5F}, {0.5F, 0.5F, -0.5F}},
        {{-0.5F, -0.5F, -0.5F}, {-0.5F, -0.5F, 0.5F}, {-0.5F, 0.5F, 0.5F}, {-0.5F, 0.5F, -0.5F}},
        {{0.5F, -0.5F, 0.5F}, {0.5F, -0.5F, -0.5F}, {0.5F, 0.5F, -0.5F}, {0.5F, 0.5F, 0.5F}},
        {{-0.5F, 0.5F, 0.5F}, {0.5F, 0.5F, 0.5F}, {0.5F, 0.5F, -0.5F}, {-0.5F, 0.5F, -0.5F}},
        {{-0.5F, -0.5F, -0.5F}, {0.5F, -0.5F, -0.5F}, {0.5F, -0.5F, 0.5F}, {-0.5F, -0.5F, 0.5F}}
    };
    constexpr glm::vec3 normals[6] = {
        {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, -1.0F}, {-1.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, -1.0F, 0.0F}
    };
    Vertex vertices[24];
    for (std::size_t face = 0; face < 6; ++face)
        for (std::size_t corner = 0; corner < 4; ++corner)
            vertices[face * 4 + corner] = {corners[face][corner], normals[face]};
    // 각 사각형 면을 삼각형 두 개로 나눈다. index 값은 byte 위치가 아니라 vertices 배열의 정점 번호다.
    const std::uint32_t indices[] = {
        0,1,2, 2,3,0, 4,5,6, 6,7,4, 8,9,10, 10,11,8,
        12,13,14, 14,15,12, 16,17,18, 18,19,16, 20,21,22, 22,23,20
    };
    // 정점 좌표를 각 축 방향으로 늘려 요청한 변 길이를 만든다.
    for (Vertex& vertex : vertices)
        vertex.position *= sideLengthMeters;

    return std::make_unique<Mesh>(vertices, 24U, indices, 36U);
}

} // namespace aerosimlab::graphics
