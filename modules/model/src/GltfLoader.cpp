#include "model/GltfLoader.h"

#include <tiny_gltf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace grasplink::model
{
namespace
{
// GLB 위치·회전·크기 계산에서 허용하는 작은 수치 오차다.
namespace transform_limits
{
inline constexpr float tolerance = 1.0e-5F;
}

// 형상 연결 번호는 uint32_t로 저장하므로 배열 크기도 이 범위를 넘을 수 없다.
namespace mesh_limits
{
inline constexpr std::size_t indicesPerTriangle = 3;
inline constexpr std::size_t maximumMeshValue =
    std::numeric_limits<std::uint32_t>::max();
}

// 자원 종류를 ID에 넣어 형상 번호와 재질 번호가 서로 겹치지 않게 한다.
namespace resource_id_suffixes
{
inline constexpr char texture[] = "#texture/";
inline constexpr char material[] = "#material/";
inline constexpr char mesh[] = "#mesh/";
}

// GLB 정점에서 읽는 속성 이름을 한곳에 모아 파일 규격과 코드의 연결을 분명히 한다.
namespace attribute_names
{
inline constexpr char position[] = "POSITION";
inline constexpr char normal[] = "NORMAL";
inline constexpr char textureCoordinates[] = "TEXCOORD_0";
}

// glTF에서 mode를 생략하면 삼각형으로 처리한다는 규칙을 이름으로 드러낸다.
namespace primitive_modes
{
inline constexpr int unspecified = -1;
inline constexpr int triangles = TINYGLTF_MODE_TRIANGLES;
}

enum class NodeVisitState : std::uint8_t
{
    Unvisited,
    Visiting,
    Visited
};

namespace error_messages
{
    namespace accessor
    {
        inline constexpr char expectedFloatVec2[] = "Expected FLOAT VEC2 ACCESSOR";
        inline constexpr char expectedFloatVec3[] = "Expected FLOAT VEC3 ACCESSOR";
        inline constexpr char invalidIndex[] = "Invalid glTF accessor Index";
        inline constexpr char sparseUnsupported[] = "Sparse glTF accessors are not supported";
        inline constexpr char missingBufferView[] = "Accessor without BufferView is not supported";
        inline constexpr char invalidBufferIndex[] = "Invalid glTF buffer index";
        inline constexpr char invalidStride[] = "Invalid glTF accessor stride";
        inline constexpr char bufferViewOutOfRange[] = "glTF bufferView exceeds buffer size";
        inline constexpr char offsetOutOfRange[] = "glTF accessor offset exceeds bufferView";
        inline constexpr char offsetOverflow[] = "glTF accessor offset overflows";
        inline constexpr char rangeOverflow[] = "glTF accessor range overflows";
        inline constexpr char exceedsBufferView[] = "glTF accessor exceeds bufferView";
    }

    namespace index
    {
        inline constexpr char invalidAccessorIndex[] =
            "Invalid glTF index accessor index";
        inline constexpr char unsupportedComponentType[] =
            "Unsupported glTF index component type";
        inline constexpr char expectedScalar[] = "Index Accessor must be SCALAR";
        inline constexpr char exceedsVertexCount[] = "glTF index exceeds vertex Count";
    }

    namespace vertex
    {
        inline constexpr char attributeNotFinite[] =
            "glTF vertex attribute contains a non-finite value";
        inline constexpr char valueNotFinite[] =
            "glTF vertex or transform contains a non-finite value";
    }

    namespace texture
    {
        inline constexpr char invalidSource[] = "Invalid glTF texture source";
        inline constexpr char invalidSize[] = "Invalid glTF image size";
        inline constexpr char unsupportedChannels[] = "Only RGB/RGBA glTF images are supported";
        inline constexpr char unsupportedBitDepth[] = "Only 8-bit glTF images are supported";
        inline constexpr char noDecodedPixels[] = "glTF image contains no decoded pixels";
    }

    namespace material
    {
        inline constexpr char invalidFactor[] =
            "glTF material factors must be finite and between 0 and 1";
        inline constexpr char invalidBaseColorFactor[] =
            "glTF base color factor must contain four values";
        inline constexpr char invalidBaseColorTextureIndex[] =
            "Invalid glTF base color texture index";
    }

    namespace mesh
    {
        inline constexpr char unsupportedPrimitiveMode[] =
            "Only TRIANGLES glTF primitives are supported";
        inline constexpr char missingPosition[] = "glTF primitive has no POSITION";
        inline constexpr char noVertices[] = "glTF Primitives has zero vertices";
        inline constexpr char vertexRangeTooLarge[] =
            "glTF mesh vertex range exceeds uint32_t";
        inline constexpr char normalCountMismatch[] =
            "NORMAL count does not match POSITION count";
        inline constexpr char textureCoordinateCountMismatch[] =
            "TEXCOORD_0 count does not match POSITION count";
        inline constexpr char invalidTriangleIndexCount[] =
            "glTF triangle primitive index count must be a positive multiple of three";
        inline constexpr char indexRangeTooLarge[] =
            "glTF mesh index range exceeds uint32_t";
    }

    namespace node
    {
        inline constexpr char translationNotFinite[] =
            "glTF node translation contains a non-finite value";
        inline constexpr char scaleNotFinite[] =
            "glTF node scale contains a non-finite value";
        inline constexpr char rotationNotFinite[] =
            "glTF node rotation contains a non-finite value";
        inline constexpr char transformNotFinite[] =
            "glTF transform contains a non-finite value";
        inline constexpr char rotationValueNotFinite[] =
            "glTF rotation contains a non-finite value";
        inline constexpr char invalidTransformFormat[] =
            "glTF node must use either matrix or TRS";
        inline constexpr char perspectiveMatrixUnsupported[] =
            "Perspective node matrices are not supported";
        inline constexpr char matrixDecompositionFailed[] =
            "Failed to decompose glTF node matrix";
        inline constexpr char matrixNotTrs[] =
            "glTF node matrix must contain non-degenerate TRS only";
        inline constexpr char invalidTrsSize[] = "Invalid glTF node TRS component size";
        inline constexpr char zeroRotation[] = "glTF node rotation quaternion is zero";
        inline constexpr char degenerateScale[] = "glTF node scale must be non-degenerate";
    }

    namespace hierarchy
    {
        inline constexpr char invalidMeshIndex[] = "Invalid glTF node mesh index";
        inline constexpr char invalidMaterialIndex[] = "Invalid glTF material index";
        inline constexpr char invalidChildIndex[] = "Invalid child node index";
        inline constexpr char multipleParents[] = "glTF node has multiple parents";
        inline constexpr char cycle[] = "glTF node hierarchy contains a cycle";
    }

    namespace scene
    {
        inline constexpr char multipleRoots[] =
            "Multiple root nodes in glTF scenes are not supported";
        inline constexpr char invalidDefaultIndex[] = "Invalid glTF default scene index";
        inline constexpr char invalidRootIndex[] = "Invalid glTF scene root node";
        inline constexpr char disconnectedHierarchy[] =
            "glTF nodes must form a single tree under the scene root";
    }

    namespace file
    {
        inline constexpr char loadFailedPrefix[] = "Failed to load GLB: ";
    }
}

// Buffer는 glTF 파일의 원본 byte 묶음이고 BufferView는 그중 일부 구간이다. Accessor는 그 구간에서 읽을 값의 자료형·개수·간격을 적은 기록이다.
// 세 기록의 연결과 범위를 확인한 뒤 첫 값의 주소와 다음 값까지의 간격을 돌려준다.
struct AccessorView
{
    const unsigned char* data = nullptr;
    std::size_t stride = 0;
    std::size_t count = 0;
};

bool IsIndexInRange(int index, std::size_t count)
{
    return index >= 0 && static_cast<std::size_t>(index) < count;
}

// Accessor의 마지막 값이 끝나는 byte 위치를 계산한다. count와 stride는 호출자가 양수인지 확인한다.
std::optional<std::size_t> AccessorRangeEnd(
    std::size_t offset,
    std::size_t count,
    std::size_t stride,
    std::size_t elementSize)
{
    const std::size_t maximum = std::numeric_limits<std::size_t>::max();
    if (offset > maximum - elementSize)
        return std::nullopt;

    const std::size_t lastElement = count - 1U;
    if (lastElement > (maximum - offset - elementSize) / stride)
        return std::nullopt;

    return offset + lastElement * stride + elementSize;
}

// 잘못된 번호를 vector 접근 전에 검사해 파일 오류를 일관된 예외로 바꾼다.
const tinygltf::Accessor& GetAccessor(const tinygltf::Model& model, int accessorIndex)
{
    if (!IsIndexInRange(accessorIndex, model.accessors.size()))
        throw std::runtime_error(error_messages::accessor::invalidIndex);

    return model.accessors[accessorIndex];
}

// Accessor가 가리키는 byte가 실제 Buffer 범위 안에 있는지 확인해 읽을 주소를 만든다. 기존 값 일부를 덮어쓰는 Sparse 형식은 현재 지원하지 않는다.
AccessorView GetAccessorView(
    const tinygltf::Model& model,
    const tinygltf::Accessor& accessor,
    std::size_t elementSize)
{
    if (accessor.sparse.isSparse)
        throw std::runtime_error(error_messages::accessor::sparseUnsupported);

    if (!IsIndexInRange(accessor.bufferView, model.bufferViews.size()))
    {
        throw std::runtime_error(error_messages::accessor::missingBufferView);
    }

    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    if (!IsIndexInRange(bufferView.buffer, model.buffers.size()))
    {
        throw std::runtime_error(error_messages::accessor::invalidBufferIndex);
    }

    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];

    /*
        byteStride는 한 값의 첫 byte에서 다음 같은 값의 첫 byte까지 간격이다.
        예를 들어 정점마다 [위치][법선][UV]를 붙여 저장하면 다음 위치는 위치 값만큼이 아니라 법선과 UV까지 건너뛴 뒤 시작한다.
        마지막 값의 끝까지 BufferView 안에 드는지 확인하므로 잘못된 파일이 범위 밖 주소를 만들기 전에 거부된다.
    */
    const int byteStride = accessor.ByteStride(bufferView);
    if (byteStride <= 0 || static_cast<std::size_t>(byteStride) < elementSize)
        throw std::runtime_error(error_messages::accessor::invalidStride);

    const std::size_t bufferSize = buffer.data.size();
    const std::size_t viewOffset = static_cast<std::size_t>(bufferView.byteOffset);
    const std::size_t viewLength = static_cast<std::size_t>(bufferView.byteLength);
    if (viewOffset > bufferSize || viewLength > bufferSize - viewOffset)
        throw std::runtime_error(error_messages::accessor::bufferViewOutOfRange);

    const std::size_t accessorOffset = static_cast<std::size_t>(accessor.byteOffset);
    if (accessorOffset > viewLength)
        throw std::runtime_error(error_messages::accessor::offsetOutOfRange);
    if (accessorOffset > std::numeric_limits<std::size_t>::max() - viewOffset)
        throw std::runtime_error(error_messages::accessor::offsetOverflow);
    const std::size_t offset = viewOffset + accessorOffset;

    if (accessor.count > 0)
    {
        const std::size_t count = static_cast<std::size_t>(accessor.count);
        const std::size_t stride = static_cast<std::size_t>(byteStride);
        const auto accessorEnd = AccessorRangeEnd(accessorOffset, count, stride, elementSize);
        if (!accessorEnd)
            throw std::runtime_error(error_messages::accessor::rangeOverflow);
        if (*accessorEnd > viewLength)
            throw std::runtime_error(error_messages::accessor::exceedsBufferView);
    }

    return AccessorView{
        accessor.count == 0 ? nullptr : buffer.data.data() + offset,
        static_cast<std::size_t>(byteStride),
        static_cast<std::size_t>(accessor.count)};
}

void RequireFinite(const glm::vec2& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y))
        throw std::runtime_error(error_messages::vertex::attributeNotFinite);
}

void RequireFinite(const glm::vec3& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
        throw std::runtime_error(error_messages::vertex::valueNotFinite);
}

void RequireFinite(const glm::vec4& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y) ||
        !std::isfinite(value.z) || !std::isfinite(value.w))
    {
        throw std::runtime_error(error_messages::node::transformNotFinite);
    }
}

void RequireFinite(const glm::quat& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y) ||
        !std::isfinite(value.z) || !std::isfinite(value.w))
    {
        throw std::runtime_error(error_messages::node::rotationValueNotFinite);
    }
}

bool HasDegenerateScale(const glm::vec3& scale)
{
    return std::abs(scale.x) <= transform_limits::tolerance ||
        std::abs(scale.y) <= transform_limits::tolerance ||
        std::abs(scale.z) <= transform_limits::tolerance;
}

float CheckedFloat(double value, const char* error)
{
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::runtime_error(error);
    return static_cast<float>(value);
}

// FLOAT VEC2와 VEC3 속성은 동일한 stride 검사로 읽는다.
template<typename Vector, std::size_t ComponentCount, typename Convert>
std::vector<Vector> ReadFloatVectorAccessor(
    const tinygltf::Model& model,
    int accessorIndex,
    int expectedType,
    const char* expectedTypeName,
    Convert convert)
{
    const tinygltf::Accessor& accessor = GetAccessor(model, accessorIndex);
    if (accessor.type != expectedType ||
        accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
        throw std::runtime_error(expectedTypeName);

    const AccessorView view = GetAccessorView(model, accessor, sizeof(float) * ComponentCount);
    std::vector<Vector> result(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        std::array<float, ComponentCount> values{};

        // 파일 byte를 정렬에 의존하지 않고 float 배열로 복사한다.
        std::memcpy(values.data(), view.data + i * view.stride, sizeof(float) * ComponentCount);
        result[i] = convert(values.data());
        RequireFinite(result[i]);
    }

    return result;
}

// 파일에서 읽을 값 주소가 C++ 타입 T의 정렬 경계에 맞는다고 가정하지 않고 복사한다.
// glTF 파일이 요구하는 별도의 정렬 조건은 여기서 검사하지 않는다.
template<typename T>
T ReadScalar(const unsigned char* data)
{
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

std::size_t GetIndexComponentSize(int componentType)
{
    switch (componentType)
    {
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
        return sizeof(std::uint8_t);
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
        return sizeof(std::uint16_t);
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
        return sizeof(std::uint32_t);
    default:
        throw std::runtime_error(error_messages::index::unsupportedComponentType);
    }
}

std::uint32_t ReadIndex(const unsigned char* source, int componentType)
{
    switch (componentType)
    {
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
        return ReadScalar<std::uint8_t>(source);
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
        return ReadScalar<std::uint16_t>(source);
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
        return ReadScalar<std::uint32_t>(source);
    default:
        throw std::runtime_error(error_messages::index::unsupportedComponentType);
    }
}

// 삼각형 연결 번호는 정점 배열에서 사용할 꼭짓점을 고르는 값이다. 이를 32-bit 정수로 통일하고 파일에 번호가 없으면 0부터 정점 순서대로 만든다.
std::vector<std::uint32_t> ReadIndices(
    const tinygltf::Model& model,
    int accessorIndex,
    std::size_t vertexCount)
{
    if (accessorIndex == -1)
    {
        std::vector<std::uint32_t> indices(vertexCount);
        std::iota(indices.begin(), indices.end(), 0U);
        return indices;
    }
    if (accessorIndex < -1)
        throw std::runtime_error(error_messages::index::invalidAccessorIndex);

    const tinygltf::Accessor& accessor = GetAccessor(model, accessorIndex);
    if (accessor.type != TINYGLTF_TYPE_SCALAR)
        throw std::runtime_error(error_messages::index::expectedScalar);

    const AccessorView view = GetAccessorView(
        model,
        accessor,
        GetIndexComponentSize(accessor.componentType));
    std::vector<std::uint32_t> result;
    result.reserve(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;
        const std::uint32_t value = ReadIndex(source, accessor.componentType);

        if (value >= vertexCount)
            throw std::runtime_error(error_messages::index::exceedsVertexCount);

        result.push_back(value);
    }

    return result;
}

// 디코드된 이미지의 RGB/RGBA 채널은 각 8 bit다. Texture 번호는 이미지 번호가 아니라 glTF 재질에서 참조할 glTF texture 항목 번호다.
TextureData ConvertTexture(
    const tinygltf::Model& model,
    const tinygltf::Texture& source,
    const std::string& resourcePrefix,
    std::size_t textureIndex)
{
    TextureData result;
    result.uniqueID = ResourceID{
        resourcePrefix + resource_id_suffixes::texture + std::to_string(textureIndex)};

    if (!IsIndexInRange(source.source, model.images.size()))
    {
        throw std::runtime_error(error_messages::texture::invalidSource);
    }

    const tinygltf::Image& image = model.images[source.source];

    if (image.width <= 0 || image.height <= 0)
        throw std::runtime_error(error_messages::texture::invalidSize);

    if (image.component != 3 && image.component != 4)
        throw std::runtime_error(error_messages::texture::unsupportedChannels);

    if (image.bits != 8)
        throw std::runtime_error(error_messages::texture::unsupportedBitDepth);

    if (image.image.empty())
        throw std::runtime_error(error_messages::texture::noDecodedPixels);

    result.width = image.width;
    result.height = image.height;
    result.channels = image.component;
    result.pixels = image.image;

    return result;
}

float MaterialFactor(double value)
{
    if (!std::isfinite(value) || value < 0.0 || value > 1.0)
        throw std::runtime_error(error_messages::material::invalidFactor);
    return static_cast<float>(value);
}

// 재질은 표면 색과 빛 반응을 정하는 데이터다. 파일의 기본색·금속성·거칠기 계수와 기본색 이미지 참조를 내부 자료로 옮긴다.
MaterialData ConvertMaterial(
    const tinygltf::Model& model,
    const tinygltf::Material& source,
    const std::string& resourcePrefix,
    std::size_t materialIndex)
{
    MaterialData result;
    result.name = source.name;
    result.uniqueID = ResourceID{
        resourcePrefix + resource_id_suffixes::material + std::to_string(materialIndex)};

    const auto& pbr = source.pbrMetallicRoughness;

    if (pbr.baseColorFactor.size() != 4)
        throw std::runtime_error(error_messages::material::invalidBaseColorFactor);
    result.baseColorFactor = glm::vec4{
        MaterialFactor(pbr.baseColorFactor[0]),
        MaterialFactor(pbr.baseColorFactor[1]),
        MaterialFactor(pbr.baseColorFactor[2]),
        MaterialFactor(pbr.baseColorFactor[3])};

    result.metallicFactor = MaterialFactor(pbr.metallicFactor);
    result.roughnessFactor = MaterialFactor(pbr.roughnessFactor);

    // glTF 재질은 texture 항목 번호를 가리키지만 내부 캐시는 모델 경로와 자원 종류, 해당 texture 항목 번호를 합친 ID로 찾는다.
    const int textureIndex = pbr.baseColorTexture.index;
    if (textureIndex < -1 ||
        (textureIndex >= 0 && static_cast<std::size_t>(textureIndex) >= model.textures.size()))
    {
        throw std::runtime_error(error_messages::material::invalidBaseColorTextureIndex);
    }
    if (textureIndex >= 0)
    {
        result.baseColorTexture = ResourceID{
            resourcePrefix + resource_id_suffixes::texture +
            std::to_string(textureIndex)};
    }

    return result;
}

std::uint32_t AppendPrimitiveVertices(
    MeshData& mesh,
    const tinygltf::Model& model,
    const tinygltf::Primitive& primitive)
{
    // POSITION은 표면 삼각형의 각 꼭짓점 위치다. 위치가 없으면 화면에 표면을 놓을 수 없다.
    const auto positionIterator = primitive.attributes.find(attribute_names::position);
    if (positionIterator == primitive.attributes.end())
        throw std::runtime_error(error_messages::mesh::missingPosition);

    const std::vector<glm::vec3> positions = ReadFloatVectorAccessor<glm::vec3, 3>(
        model, positionIterator->second, TINYGLTF_TYPE_VEC3,
        error_messages::accessor::expectedFloatVec3, [](const float* value) {
            return glm::vec3{value[0], value[1], value[2]};
        });

    const std::size_t vertexCount = positions.size();
    if (vertexCount == 0)
        throw std::runtime_error(error_messages::mesh::noVertices);
    if (vertexCount > mesh_limits::maximumMeshValue ||
        mesh.vertices.size() > mesh_limits::maximumMeshValue - vertexCount)
        throw std::runtime_error(error_messages::mesh::vertexRangeTooLarge);

    // NORMAL은 표면 방향, TEXCOORD_0은 기본 색 이미지 좌표다. 선택 속성이 없으면 Vertex 기본값을 유지한다.
    std::vector<glm::vec3> normals;
    const auto normalIterator = primitive.attributes.find(attribute_names::normal);
    if (normalIterator != primitive.attributes.end())
    {
        normals = ReadFloatVectorAccessor<glm::vec3, 3>(
            model, normalIterator->second, TINYGLTF_TYPE_VEC3,
            error_messages::accessor::expectedFloatVec3, [](const float* value) {
                return glm::vec3{value[0], value[1], value[2]};
            });
        if (normals.size() != vertexCount)
            throw std::runtime_error(error_messages::mesh::normalCountMismatch);
    }

    std::vector<glm::vec2> texCoords;
    const auto texCoordIterator = primitive.attributes.find(attribute_names::textureCoordinates);
    if (texCoordIterator != primitive.attributes.end())
    {
        texCoords = ReadFloatVectorAccessor<glm::vec2, 2>(
            model, texCoordIterator->second, TINYGLTF_TYPE_VEC2,
            error_messages::accessor::expectedFloatVec2, [](const float* value) {
                return glm::vec2{value[0], value[1]};
            });
        if (texCoords.size() != vertexCount)
            throw std::runtime_error(
                error_messages::mesh::textureCoordinateCountMismatch);
    }

    const auto baseVertex = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.reserve(mesh.vertices.size() + vertexCount);

    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        Vertex vertex;
        vertex.position = positions[i];
        if (!normals.empty()) vertex.normal = normals[i];
        if (!texCoords.empty()) vertex.texCoord = texCoords[i];
        mesh.vertices.push_back(vertex);
    }

    return baseVertex;
}

void AppendPrimitiveIndicesAndSubmesh(
    MeshData& mesh,
    const tinygltf::Model& model,
    const tinygltf::Primitive& primitive,
    std::size_t vertexCount,
    std::uint32_t baseVertex)
{
    const std::vector<std::uint32_t> localIndices =
        ReadIndices(model, primitive.indices, vertexCount);
    if (localIndices.size() < mesh_limits::indicesPerTriangle ||
        localIndices.size() % mesh_limits::indicesPerTriangle != 0)
    {
        throw std::runtime_error(error_messages::mesh::invalidTriangleIndexCount);
    }

    if (mesh.indices.size() > mesh_limits::maximumMeshValue ||
        localIndices.size() > mesh_limits::maximumMeshValue - mesh.indices.size())
    {
        throw std::runtime_error(error_messages::mesh::indexRangeTooLarge);
    }

    const auto indexStart = static_cast<std::uint32_t>(mesh.indices.size());
    mesh.indices.reserve(mesh.indices.size() + localIndices.size());
    for (const std::uint32_t localIndex : localIndices)
        mesh.indices.push_back(baseVertex + localIndex);

    SubMeshInfo subMesh;
    subMesh.indexStart = indexStart;
    subMesh.indexCount = static_cast<std::uint32_t>(localIndices.size());
    if (primitive.material < -1 ||
        (primitive.material >= 0 &&
            static_cast<std::size_t>(primitive.material) >= model.materials.size()))
        throw std::runtime_error(error_messages::hierarchy::invalidMaterialIndex);
    if (primitive.material >= 0)
        subMesh.defaultMaterialIndex = static_cast<std::size_t>(primitive.material);
    mesh.subMeshes.push_back(subMesh);
}

MeshData ConvertMesh(
    const tinygltf::Model& model,
    const tinygltf::Mesh& source,
    const std::string& resourcePrefix,
    std::size_t meshIndex)
{
    // Primitive는 재질 하나로 그리는 삼각형 묶음이다. 각 묶음의 연결 번호에 앞서 추가한 정점 수를 더해 하나의 Mesh 배열로 합친다.
    MeshData result;
    result.name = source.name;
    result.uniqueID = ResourceID{
        resourcePrefix + resource_id_suffixes::mesh + std::to_string(meshIndex)};

    for (const tinygltf::Primitive& primitive : source.primitives)
    {
        if (primitive.mode != primitive_modes::triangles &&
            primitive.mode != primitive_modes::unspecified)
            throw std::runtime_error(error_messages::mesh::unsupportedPrimitiveMode);

        const std::uint32_t baseVertex =
            AppendPrimitiveVertices(result, model, primitive);
        const std::size_t vertexCount = result.vertices.size() - baseVertex;
        AppendPrimitiveIndicesAndSubmesh(
            result, model, primitive, vertexCount, baseVertex);
    }

    return result;
}

void ReadMatrixTransform(
    const tinygltf::Node& source,
    NodeData& destination)
{
    if (source.matrix.size() != 16 ||
        !source.translation.empty() ||
        !source.rotation.empty() ||
        !source.scale.empty())
    {
        throw std::runtime_error(error_messages::node::invalidTransformFormat);
    }

    glm::mat4 matrix{1.0F};

    // 파일과 GLM은 행렬 원소를 열 기준으로 나열하므로 같은 열·행 위치에 옮긴다.
    for (int column = 0; column < 4; ++column)
    {
        for (int row = 0; row < 4; ++row)
        {
            matrix[column][row] = CheckedFloat(source.matrix[column * 4 + row],
                "glTF node matrix contains a non-finite value");
        }
    }

    // 원근 성분은 위치·회전·크기만으로 보존할 수 없으므로 거부한다.
    if (std::abs(matrix[0][3]) > transform_limits::tolerance ||
        std::abs(matrix[1][3]) > transform_limits::tolerance ||
        std::abs(matrix[2][3]) > transform_limits::tolerance ||
        std::abs(matrix[3][3] - 1.0F) > transform_limits::tolerance)
    {
        throw std::runtime_error(error_messages::node::perspectiveMatrixUnsupported);
    }

    glm::vec3 scale{1.0F};
    glm::quat orientation{};
    glm::vec3 translation{0.0F};
    glm::vec3 skew{0.0F};
    glm::vec4 perspective{0.0F};

    if (!glm::decompose(
            matrix,
            scale,
            orientation,
            translation,
            skew,
            perspective))
    {
        throw std::runtime_error(error_messages::node::matrixDecompositionFailed);
    }

    RequireFinite(scale);
    RequireFinite(translation);
    RequireFinite(orientation);
    RequireFinite(skew);
    RequireFinite(perspective);

    // 전단 변환은 한 축이 다른 축 방향으로 기울어진 상태다. TRS만 저장할 수 있어 이런 행렬은 거부한다.
    if (HasDegenerateScale(scale) ||
        glm::length(orientation) <= transform_limits::tolerance ||
        glm::length(skew) > transform_limits::tolerance ||
        std::abs(perspective.x) > transform_limits::tolerance ||
        std::abs(perspective.y) > transform_limits::tolerance ||
        std::abs(perspective.z) > transform_limits::tolerance ||
        std::abs(perspective.w - 1.0F) > transform_limits::tolerance)
    {
        throw std::runtime_error(error_messages::node::matrixNotTrs);
    }

    destination.translation = translation;
    destination.rotation = glm::normalize(orientation);
    destination.scale = scale;
    RequireFinite(destination.rotation);
}

void ReadTrsTransform(
    const tinygltf::Node& source,
    NodeData& destination)
{
    if ((!source.translation.empty() && source.translation.size() != 3) ||
        (!source.rotation.empty() && source.rotation.size() != 4) ||
        (!source.scale.empty() && source.scale.size() != 3))
    {
        throw std::runtime_error(error_messages::node::invalidTrsSize);
    }

    if (source.translation.size() == 3)
    {
        destination.translation = glm::vec3{
            CheckedFloat(source.translation[0], error_messages::node::translationNotFinite),
            CheckedFloat(source.translation[1], error_messages::node::translationNotFinite),
            CheckedFloat(source.translation[2], error_messages::node::translationNotFinite)};
    }

    if (source.scale.size() == 3)
    {
        destination.scale = glm::vec3{
            CheckedFloat(source.scale[0], error_messages::node::scaleNotFinite),
            CheckedFloat(source.scale[1], error_messages::node::scaleNotFinite),
            CheckedFloat(source.scale[2], error_messages::node::scaleNotFinite)};
    }

    if (source.rotation.size() == 4)
    {
        // Quaternion은 회전을 네 숫자로 나타내며 yaw/pitch 같은 세 축 회전각과 다른 표현이다. 파일의 [x,y,z,w] 순서를 GLM 생성자의 (w,x,y,z)로 옮긴다.
        const glm::quat quaternion{
        CheckedFloat(source.rotation[3], error_messages::node::rotationNotFinite),
        CheckedFloat(source.rotation[0], error_messages::node::rotationNotFinite),
        CheckedFloat(source.rotation[1], error_messages::node::rotationNotFinite),
        CheckedFloat(source.rotation[2], error_messages::node::rotationNotFinite)};

        const double quaternionLength = std::sqrt(
            static_cast<double>(quaternion.x) * quaternion.x +
            static_cast<double>(quaternion.y) * quaternion.y +
            static_cast<double>(quaternion.z) * quaternion.z +
            static_cast<double>(quaternion.w) * quaternion.w);
        if (!std::isfinite(quaternionLength) ||
            quaternionLength <= transform_limits::tolerance)
            throw std::runtime_error(error_messages::node::zeroRotation);

        const glm::quat normalizedQuaternion{
            static_cast<float>(quaternion.w / quaternionLength),
            static_cast<float>(quaternion.x / quaternionLength),
            static_cast<float>(quaternion.y / quaternionLength),
            static_cast<float>(quaternion.z / quaternionLength)};
        // 길이가 0인 quaternion은 회전축과 회전량을 정할 수 없으므로 거부한다. 나머지는 회전 방향을 유지하며 길이를 1로 맞춘다.
        destination.rotation = normalizedQuaternion;
    }

    RequireFinite(destination.translation);
    RequireFinite(destination.rotation);
    RequireFinite(destination.scale);
    if (HasDegenerateScale(destination.scale))
    {
        throw std::runtime_error(error_messages::node::degenerateScale);
    }
}

// Node는 모델의 한 부품과 부모·자식 관계를 나타낸다. glTF의 두 변환 형식을 각각 읽어 부모 기준 위치·회전·크기를 저장한다.
void ReadNodeTransform(
    const tinygltf::Node& source,
    NodeData& destination)
{
    if (!source.matrix.empty())
    {
        ReadMatrixTransform(source, destination);
        return;
    }

    ReadTrsTransform(source, destination);
}

void ReadVisualResources(
    const tinygltf::Model& source,
    const std::string& resourcePrefix,
    ModelResource& destination)
{
    destination.textures.reserve(source.textures.size());
    for (std::size_t i = 0; i < source.textures.size(); ++i)
    {
        destination.textures.push_back(
            ConvertTexture(source, source.textures[i], resourcePrefix, i));
    }

    destination.materials.reserve(source.materials.size());
    for (std::size_t i = 0; i < source.materials.size(); ++i)
    {
        destination.materials.push_back(
            ConvertMaterial(source, source.materials[i], resourcePrefix, i));
    }

    destination.meshes.reserve(source.meshes.size());
    for (std::size_t i = 0; i < source.meshes.size(); ++i)
    {
        destination.meshes.push_back(
            ConvertMesh(source, source.meshes[i], resourcePrefix, i));
    }
}

void ReadNodeRecords(const tinygltf::Model& source, ModelResource& destination)
{
    destination.nodes.resize(source.nodes.size());
    for (std::size_t i = 0; i < source.nodes.size(); ++i)
    {
        const tinygltf::Node& sourceNode = source.nodes[i];
        NodeData& node = destination.nodes[i];

        node.name = sourceNode.name.empty()
            ? "Node_" + std::to_string(i)
            : sourceNode.name;

        if (sourceNode.mesh < -1 ||
            (sourceNode.mesh >= 0 && !IsIndexInRange(sourceNode.mesh, destination.meshes.size())))
            throw std::runtime_error(error_messages::hierarchy::invalidMeshIndex);
        if (sourceNode.mesh >= 0)
            node.meshIndex = static_cast<std::size_t>(sourceNode.mesh);
        ReadNodeTransform(sourceNode, node);
    }

    for (std::size_t parentIndex = 0; parentIndex < source.nodes.size(); ++parentIndex)
    {
        for (const int childIndex : source.nodes[parentIndex].children)
        {
            if (!IsIndexInRange(childIndex, destination.nodes.size()))
                throw std::runtime_error(error_messages::hierarchy::invalidChildIndex);

            NodeData& child = destination.nodes[static_cast<std::size_t>(childIndex)];
            if (child.parentIndex)
                throw std::runtime_error(error_messages::hierarchy::multipleParents);

            child.parentIndex = parentIndex;
        }
    }
}

void BuildNodeHierarchy(ModelResource& model)
{
    std::vector<NodeVisitState> nodeState(
        model.nodes.size(), NodeVisitState::Unvisited);
    for (std::size_t start = 0; start < model.nodes.size(); ++start)
    {
        std::optional<std::size_t> current = start;
        while (current && nodeState[*current] == NodeVisitState::Unvisited)
        {
            nodeState[*current] = NodeVisitState::Visiting;
            current = model.nodes[*current].parentIndex;
        }

        if (current && nodeState[*current] == NodeVisitState::Visiting)
            throw std::runtime_error(error_messages::hierarchy::cycle);

        current = start;
        while (current && nodeState[*current] == NodeVisitState::Visiting)
        {
            nodeState[*current] = NodeVisitState::Visited;
            current = model.nodes[*current].parentIndex;
        }
    }
}

void SelectSceneRoot(const tinygltf::Model& source, ModelResource& model)
{
    int sceneIndex = source.defaultScene;
    if (sceneIndex < -1)
        throw std::runtime_error(error_messages::scene::invalidDefaultIndex);
    if (sceneIndex < 0 && !source.scenes.empty())
        sceneIndex = 0;

    if (sceneIndex >= 0 && !IsIndexInRange(sceneIndex, source.scenes.size()))
        throw std::runtime_error(error_messages::scene::invalidDefaultIndex);

    if (sceneIndex >= 0)
    {
        const tinygltf::Scene& scene = source.scenes[sceneIndex];
        if (scene.nodes.size() > 1)
            throw std::runtime_error(error_messages::scene::multipleRoots);
        if (scene.nodes.size() == 1)
        {
            const int rootIndex = scene.nodes.front();
            if (!IsIndexInRange(rootIndex, model.nodes.size()) ||
                model.nodes[static_cast<std::size_t>(rootIndex)].parentIndex.has_value())
            {
                throw std::runtime_error(error_messages::scene::invalidRootIndex);
            }
            model.rootNodeIndex = static_cast<std::size_t>(rootIndex);
        }
    }

    // 장면에 시작 부품이 없으면 부모가 없는 부품 하나를 찾는다.
    if (!model.rootNodeIndex)
    {
        for (std::size_t i = 0; i < model.nodes.size(); ++i)
        {
            if (model.nodes[i].parentIndex.has_value())
                continue;
            if (model.rootNodeIndex.has_value())
                throw std::runtime_error(error_messages::scene::multipleRoots);
            model.rootNodeIndex = i;
        }
    }

    // 선택한 부품에서 시작하지 않는 별도 계층이 있으면 PrefabFactory가 한 장면으로 만들 수 없다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        if (!model.nodes[i].parentIndex && i != *model.rootNodeIndex)
        {
            throw std::runtime_error(error_messages::scene::disconnectedHierarchy);
        }
    }
}
} // namespace

ModelResource LoadGlb(const std::filesystem::path& path)
{
    tinygltf::TinyGLTF loader;
    tinygltf::Model gltfModel;

    std::string error;
    std::string warning;

    const bool loaded = loader.LoadBinaryFromFile(
        &gltfModel,
        &error,
        &warning,
        path.string());

    if (!loaded)
        throw std::runtime_error(
            error_messages::file::loadFailedPrefix + path.string() + "\n" + error);

    /*
        TinyGLTF가 남긴 경고는 파일 읽기 실패와는 다르다.
        현재는 경고를 저장하거나 출력하는 기능이 없어 버린다. 진단 로그가 추가되면 Logger에 전달해야 한다.
    */
    (void)warning;

    ModelResource result;

    /*
        ResourceID는 캐시에서 자원을 다시 찾는 식별자다. 파일 경로와 자원 종류, 항목 번호를 조합하므로 서로 다른 GLB의 같은 번호도 구분된다.
        예: robot.glb#mesh/0은 첫 형상, robot.glb#material/0은 첫 재질, robot.glb#texture/0은 첫 glTF texture 항목이다.
    */
    const std::string resourcePrefix = path.generic_string();
    ReadVisualResources(gltfModel, resourcePrefix, result);
    ReadNodeRecords(gltfModel, result);
    BuildNodeHierarchy(result);
    SelectSceneRoot(gltfModel, result);

    return result;
}

} // namespace grasplink::model
