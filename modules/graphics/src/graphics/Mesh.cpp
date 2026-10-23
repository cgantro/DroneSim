#include "graphics/Mesh.h"

#include "graphics/IndexBuffer.h"
#include "graphics/VertexArray.h"
#include "graphics/VertexBuffer.h"

#include "graphics/GlApi.h"

#include <nlohmann/json.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aerosimlab::graphics
{
namespace
{
using Json = nlohmann::json;

std::uint32_t ReadUint32(const std::uint8_t* bytes)
{
    // GLB의 정수 필드는 항상 가장 낮은 자리의 바이트부터 기록된다.
    return static_cast<std::uint32_t>(bytes[0])
        | (static_cast<std::uint32_t>(bytes[1]) << 8U)
        | (static_cast<std::uint32_t>(bytes[2]) << 16U)
        | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

struct GlbAccessor
{
    const std::uint8_t* bytes = nullptr;
    std::size_t count = 0;
    std::size_t stride = 0;
    std::uint32_t componentType = 0;
};

GlbAccessor ReadAccessor(
    const Json& document,
    std::size_t accessorIndex,
    const std::vector<std::uint8_t>& binary,
    const std::string& expectedType,
    std::size_t elementSize)
{
    const Json& accessor = document.at("accessors").at(accessorIndex);
    if (accessor.at("type").get<std::string>() != expectedType
        || accessor.contains("sparse"))
        throw std::runtime_error("GLB 모델의 정점 자료 형식을 지원하지 않습니다.");

    const Json& view = document.at("bufferViews").at(
        accessor.at("bufferView").get<std::size_t>());
    if (view.value("buffer", 0U) != 0U)
        throw std::runtime_error("GLB 모델은 하나의 내부 버퍼만 사용할 수 있습니다.");

    const std::size_t viewOffset = view.value("byteOffset", std::size_t{0});
    const std::size_t viewLength = view.at("byteLength").get<std::size_t>();
    const std::size_t accessorOffset = accessor.value("byteOffset", std::size_t{0});
    const std::size_t count = accessor.at("count").get<std::size_t>();
    const std::size_t stride = view.value("byteStride", elementSize);
    if (viewOffset > binary.size()
        || viewLength > binary.size() - viewOffset
        || accessorOffset > viewLength
        || count == 0U
        || stride < elementSize)
        throw std::runtime_error("GLB 모델의 정점 자료 범위를 확인해 주세요.");

    const std::size_t offset = viewOffset + accessorOffset;
    const std::size_t available = viewLength - accessorOffset;
    if (available < elementSize || (count - 1U) > (available - elementSize) / stride)
        throw std::runtime_error("GLB 모델의 정점 자료가 파일 범위를 벗어났습니다.");

    return {binary.data() + offset, count, stride,
        accessor.at("componentType").get<std::uint32_t>()};
}

float ReadFloat(const std::uint8_t* bytes)
{
    const std::uint32_t bits = ReadUint32(bytes);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::uint32_t ReadIndex(const std::uint8_t* bytes, std::uint32_t componentType)
{
    if (componentType == 5123U)
        return static_cast<std::uint32_t>(bytes[0])
            | (static_cast<std::uint32_t>(bytes[1]) << 8U);
    if (componentType == 5125U)
        return ReadUint32(bytes);
    throw std::runtime_error("GLB 모델의 삼각형 번호 형식을 지원하지 않습니다.");
}

std::vector<std::uint8_t> ReadGlbFile(const std::string& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file)
        throw std::runtime_error("GLB 모델을 열지 못했습니다: " + path);

    const std::streamsize fileSize = static_cast<std::streamsize>(file.tellg());
    if (fileSize < 20)
        throw std::runtime_error("GLB 파일이 너무 작습니다: " + path);

    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(fileSize));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(bytes.data()), fileSize);
    if (!file || ReadUint32(bytes.data()) != 0x46546C67U
        || ReadUint32(bytes.data() + 4U) != 2U
        || ReadUint32(bytes.data() + 8U) != bytes.size())
        throw std::runtime_error("GLB 헤더를 읽을 수 없습니다: " + path);
    return bytes;
}

struct GlbDocument
{
    Json json;
    std::vector<std::uint8_t> binary;
};

GlbDocument ReadGlbDocument(const std::string& path)
{
    const std::vector<std::uint8_t> file = ReadGlbFile(path);
    std::vector<std::uint8_t> jsonBytes;
    std::vector<std::uint8_t> binary;
    std::size_t chunkOffset = 12U;
    while (chunkOffset + 8U <= file.size())
    {
        const std::size_t chunkSize = ReadUint32(file.data() + chunkOffset);
        const std::uint32_t chunkType = ReadUint32(file.data() + chunkOffset + 4U);
        chunkOffset += 8U;
        if (chunkSize > file.size() - chunkOffset)
            throw std::runtime_error("GLB 조각이 파일 범위를 벗어났습니다: " + path);

        const auto begin = file.begin() + static_cast<std::ptrdiff_t>(chunkOffset);
        const auto end = begin + static_cast<std::ptrdiff_t>(chunkSize);
        if (chunkType == 0x4E4F534AU)
            jsonBytes.assign(begin, end);
        else if (chunkType == 0x004E4942U)
            binary.assign(begin, end);
        chunkOffset += chunkSize;
    }
    if (jsonBytes.empty() || binary.empty())
        throw std::runtime_error("GLB에 JSON 또는 내부 정점 자료가 없습니다: " + path);

    return {Json::parse(jsonBytes.begin(), jsonBytes.end()), std::move(binary)};
}

const Json& ReadSingleTrianglePrimitive(const Json& document)
{
    const Json& meshes = document.at("meshes");
    const Json& nodes = document.at("nodes");
    if (meshes.size() != 1U || meshes.at(0).at("primitives").size() != 1U
        || nodes.size() != 1U || nodes.at(0).value("mesh", 1U) != 0U
        || nodes.at(0).contains("matrix") || nodes.at(0).contains("translation")
        || nodes.at(0).contains("rotation") || nodes.at(0).contains("scale"))
        throw std::runtime_error("GLB 모델은 메시 하나와 삼각형 묶음 하나로 저장해야 합니다.");

    const Json& primitive = meshes.at(0).at("primitives").at(0);
    if (primitive.value("mode", 4U) != 4U)
        throw std::runtime_error("GLB 모델은 삼각형으로 저장해야 합니다.");
    return primitive;
}

std::vector<Vertex> ReadGlbVertices(
    const Json& document, const Json& primitive,
    const std::vector<std::uint8_t>& binary)
{
    const Json& attributes = primitive.at("attributes");
    const GlbAccessor positions = ReadAccessor(
        document, attributes.at("POSITION").get<std::size_t>(), binary, "VEC3", 12U);
    const GlbAccessor normals = ReadAccessor(
        document, attributes.at("NORMAL").get<std::size_t>(), binary, "VEC3", 12U);
    if (positions.componentType != 5126U || normals.componentType != 5126U
        || positions.count != normals.count
        || positions.count > std::numeric_limits<std::uint32_t>::max())
        throw std::runtime_error("GLB 위치·법선 자료 형식이 올바르지 않습니다.");

    std::vector<Vertex> vertices(positions.count);
    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        const std::uint8_t* position = positions.bytes + index * positions.stride;
        const std::uint8_t* normal = normals.bytes + index * normals.stride;
        vertices[index] = {
            {ReadFloat(position), ReadFloat(position + 4U), ReadFloat(position + 8U)},
            {ReadFloat(normal), ReadFloat(normal + 4U), ReadFloat(normal + 8U)},
            {0.0F, 0.0F}};
    }

    if (attributes.contains("TEXCOORD_0"))
    {
        const GlbAccessor textureCoordinates = ReadAccessor(
            document, attributes.at("TEXCOORD_0").get<std::size_t>(),
            binary, "VEC2", 8U);
        if (textureCoordinates.componentType != 5126U
            || textureCoordinates.count != positions.count)
            throw std::runtime_error("GLB 이미지 좌표 자료 형식이 올바르지 않습니다.");
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const std::uint8_t* coordinate = textureCoordinates.bytes
                + index * textureCoordinates.stride;
            vertices[index].textureCoordinate = {
                ReadFloat(coordinate), ReadFloat(coordinate + 4U)};
        }
    }
    return vertices;
}

std::vector<std::uint32_t> ReadGlbTriangleIndices(
    const Json& document, const Json& primitive,
    const std::vector<std::uint8_t>& binary, std::size_t vertexCount)
{
    const std::size_t accessorIndex = primitive.at("indices").get<std::size_t>();
    const std::uint32_t indexType = document.at("accessors")
        .at(accessorIndex).at("componentType").get<std::uint32_t>();
    const std::size_t indexSize = indexType == 5123U ? 2U : 4U;
    const GlbAccessor indices = ReadAccessor(
        document, accessorIndex, binary, "SCALAR", indexSize);
    if ((indexType != 5123U && indexType != 5125U)
        || indices.count > std::numeric_limits<std::uint32_t>::max()
        || indices.count % 3U != 0U)
        throw std::runtime_error("GLB 삼각형 번호 자료 형식이 올바르지 않습니다.");

    std::vector<std::uint32_t> triangles(indices.count);
    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        triangles[index] = ReadIndex(indices.bytes + index * indices.stride,
            indices.componentType);
        if (triangles[index] >= vertexCount)
            throw std::runtime_error("GLB 삼각형이 존재하지 않는 정점을 가리킵니다.");
    }
    return triangles;
}
}


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

std::unique_ptr<Mesh> Mesh::LoadGlb(const std::string& path)
{
    GlbDocument source = ReadGlbDocument(path);
    const Json& primitive = ReadSingleTrianglePrimitive(source.json);
    std::vector<Vertex> vertices = ReadGlbVertices(source.json, primitive, source.binary);
    std::vector<std::uint32_t> triangles = ReadGlbTriangleIndices(
        source.json, primitive, source.binary, vertices.size());
    return std::make_unique<Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        triangles.data(), static_cast<std::uint32_t>(triangles.size()));
}

} // namespace aerosimlab::graphics
