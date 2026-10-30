#pragma once

#include "graphics/Mesh.h"

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

namespace dronesim::graphics::glb_detail
{
using Json = nlohmann::json;

inline std::uint32_t ReadUint32(const std::uint8_t* bytes)
{
    return static_cast<std::uint32_t>(bytes[0])
        | (static_cast<std::uint32_t>(bytes[1]) << 8U)
        | (static_cast<std::uint32_t>(bytes[2]) << 16U)
        | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

struct Document
{
    Json json;
    std::vector<std::uint8_t> binary;
};

inline Document ReadDocument(const std::string& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file)
        throw std::runtime_error("GLB 모델을 열지 못했습니다: " + path);

    const std::streampos endPosition = file.tellg();
    if (endPosition < std::streampos{20})
        throw std::runtime_error("GLB 파일이 너무 작습니다: " + path);
    const auto fileSize = static_cast<std::size_t>(endPosition);
    std::vector<std::uint8_t> bytes(fileSize);
    file.seekg(0);
    file.read(reinterpret_cast<char*>(bytes.data()),
        static_cast<std::streamsize>(bytes.size()));
    if (!file || ReadUint32(bytes.data()) != 0x46546C67U
        || ReadUint32(bytes.data() + 4U) != 2U
        || ReadUint32(bytes.data() + 8U) != bytes.size())
        throw std::runtime_error("GLB 헤더를 읽을 수 없습니다: " + path);

    std::vector<std::uint8_t> jsonBytes;
    std::vector<std::uint8_t> binary;
    std::size_t offset = 12U;
    while (offset <= bytes.size() && bytes.size() - offset >= 8U)
    {
        const std::size_t chunkSize = ReadUint32(bytes.data() + offset);
        const std::uint32_t chunkType = ReadUint32(bytes.data() + offset + 4U);
        offset += 8U;
        if (chunkSize > bytes.size() - offset)
            throw std::runtime_error("GLB 조각이 파일 범위를 벗어났습니다: " + path);
        const auto begin = bytes.begin() + static_cast<std::ptrdiff_t>(offset);
        const auto end = begin + static_cast<std::ptrdiff_t>(chunkSize);
        if (chunkType == 0x4E4F534AU)
        {
            if (!jsonBytes.empty())
                throw std::runtime_error("GLB JSON 조각이 중복되었습니다: " + path);
            jsonBytes.assign(begin, end);
        }
        else if (chunkType == 0x004E4942U)
        {
            if (!binary.empty())
                throw std::runtime_error("GLB 이진 조각이 중복되었습니다: " + path);
            binary.assign(begin, end);
        }
        offset += chunkSize;
    }
    if (jsonBytes.empty() || binary.empty())
        throw std::runtime_error("GLB에 JSON 또는 내부 정점 자료가 없습니다: " + path);
    return {Json::parse(jsonBytes.begin(), jsonBytes.end()), std::move(binary)};
}

struct Accessor
{
    const std::uint8_t* bytes = nullptr;
    std::size_t count = 0;
    std::size_t stride = 0;
    std::uint32_t componentType = 0;
    bool normalized = false;
};

inline Accessor ReadAccessor(const Json& document, std::size_t accessorIndex,
    const std::vector<std::uint8_t>& binary, const std::string& expectedType,
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
    if (viewOffset > binary.size() || viewLength > binary.size() - viewOffset
        || accessorOffset > viewLength || count == 0U || stride < elementSize)
        throw std::runtime_error("GLB 모델의 정점 자료 범위를 확인해 주세요.");
    const std::size_t available = viewLength - accessorOffset;
    if (available < elementSize || (count - 1U) > (available - elementSize) / stride)
        throw std::runtime_error("GLB 모델의 정점 자료가 파일 범위를 벗어났습니다.");
    return {binary.data() + viewOffset + accessorOffset, count, stride,
        accessor.at("componentType").get<std::uint32_t>(),
        accessor.value("normalized", false)};
}

inline float ReadFloat(const std::uint8_t* bytes)
{
    const std::uint32_t bits = ReadUint32(bytes);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline float ReadVertexComponent(const std::uint8_t* bytes,
    std::uint32_t componentType)
{
    if (componentType == 5126U)
        return ReadFloat(bytes);
    if (componentType == 5122U)
    {
        const std::uint32_t raw = static_cast<std::uint32_t>(bytes[0])
            | (static_cast<std::uint32_t>(bytes[1]) << 8U);
        const std::int32_t signedValue = raw < 0x8000U
            ? static_cast<std::int32_t>(raw)
            : static_cast<std::int32_t>(raw) - 0x10000;
        return std::max(static_cast<float>(signedValue) / 32767.0F, -1.0F);
    }
    throw std::runtime_error("GLB 위치 자료 형식이 올바르지 않습니다.");
}

inline std::uint32_t ReadIndex(const std::uint8_t* bytes, std::uint32_t componentType)
{
    if (componentType == 5121U)
        return bytes[0];
    if (componentType == 5123U)
        return static_cast<std::uint32_t>(bytes[0])
            | (static_cast<std::uint32_t>(bytes[1]) << 8U);
    if (componentType == 5125U)
        return ReadUint32(bytes);
    throw std::runtime_error("GLB 모델의 삼각형 번호 형식을 지원하지 않습니다.");
}

inline std::vector<Vertex> ReadVertices(const Json& document, const Json& primitive,
    const std::vector<std::uint8_t>& binary)
{
    const Json& attributes = primitive.at("attributes");
    const std::uint32_t positionType = document.at("accessors")
        .at(attributes.at("POSITION").get<std::size_t>())
        .at("componentType").get<std::uint32_t>();
    const bool normalizedPositions = document.at("accessors")
        .at(attributes.at("POSITION").get<std::size_t>())
        .value("normalized", false);
    const std::size_t positionComponentSize = positionType == 5122U ? 2U : 4U;
    const Accessor positions = ReadAccessor(document,
        attributes.at("POSITION").get<std::size_t>(), binary, "VEC3",
        positionComponentSize * 3U);
    if ((positions.componentType != 5126U
            && !(positions.componentType == 5122U && normalizedPositions))
        || positions.count > std::numeric_limits<std::uint32_t>::max())
        throw std::runtime_error("GLB 위치 자료 형식이 올바르지 않습니다.");

    Accessor normals;
    if (attributes.contains("NORMAL"))
    {
        const std::size_t normalAccessorIndex =
            attributes.at("NORMAL").get<std::size_t>();
        const std::uint32_t normalType = document.at("accessors")
            .at(normalAccessorIndex).at("componentType").get<std::uint32_t>();
        const std::size_t normalComponentSize = normalType == 5122U ? 2U : 4U;
        normals = ReadAccessor(document, normalAccessorIndex, binary, "VEC3",
            normalComponentSize * 3U);
        if ((normals.componentType != 5126U
                && !(normals.componentType == 5122U && normals.normalized))
            || normals.count != positions.count)
            throw std::runtime_error("GLB 표면 방향 자료 형식이 올바르지 않습니다.");
    }
    std::vector<Vertex> vertices(positions.count);
    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        const auto* position = positions.bytes + index * positions.stride;
        const std::size_t componentSize = positions.componentType == 5122U ? 2U : 4U;
        vertices[index].position = {
            ReadVertexComponent(position, positions.componentType),
            ReadVertexComponent(position + componentSize, positions.componentType),
            ReadVertexComponent(position + componentSize * 2U, positions.componentType)};
        if (attributes.contains("NORMAL"))
        {
            const auto* normal = normals.bytes + index * normals.stride;
            const std::size_t componentSize = normals.componentType == 5122U ? 2U : 4U;
            vertices[index].normal = {
                ReadVertexComponent(normal, normals.componentType),
                ReadVertexComponent(normal + componentSize, normals.componentType),
                ReadVertexComponent(normal + componentSize * 2U, normals.componentType)};
        }
        vertices[index].textureCoordinate = {0.0F, 0.0F};
    }
    if (attributes.contains("TEXCOORD_0"))
    {
        const std::size_t accessorIndex = attributes.at("TEXCOORD_0").get<std::size_t>();
        const std::uint32_t componentType = document.at("accessors")
            .at(accessorIndex).at("componentType").get<std::uint32_t>();
        const std::size_t componentSize = componentType == 5121U ? 1U
            : componentType == 5123U ? 2U : 4U;
        const Accessor coordinates = ReadAccessor(document,
            accessorIndex, binary, "VEC2", componentSize * 2U);
        const bool supportedType = coordinates.componentType == 5126U
            || (coordinates.normalized && (coordinates.componentType == 5121U
                || coordinates.componentType == 5123U));
        if (!supportedType || coordinates.count != positions.count)
            throw std::runtime_error("GLB 이미지 좌표 자료 형식이 올바르지 않습니다.");
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const auto* coordinate = coordinates.bytes + index * coordinates.stride;
            if (coordinates.componentType == 5126U)
            {
                vertices[index].textureCoordinate = {
                    ReadFloat(coordinate), ReadFloat(coordinate + 4U)};
            }
            else if (coordinates.componentType == 5121U)
            {
                vertices[index].textureCoordinate = {
                    static_cast<float>(coordinate[0]) / 255.0F,
                    static_cast<float>(coordinate[1]) / 255.0F};
            }
            else
            {
                const auto readNormalizedU16 = [](const std::uint8_t* value)
                {
                    const std::uint32_t decoded = static_cast<std::uint32_t>(value[0])
                        | (static_cast<std::uint32_t>(value[1]) << 8U);
                    return static_cast<float>(decoded) / 65535.0F;
                };
                vertices[index].textureCoordinate = {
                    readNormalizedU16(coordinate), readNormalizedU16(coordinate + 2U)};
            }
        }
    }
    return vertices;
}

inline std::vector<std::uint32_t> ReadTriangleIndices(const Json& document,
    const Json& primitive, const std::vector<std::uint8_t>& binary,
    std::size_t vertexCount)
{
    if (!primitive.contains("indices"))
    {
        if (vertexCount % 3U != 0U)
            throw std::runtime_error("인덱스 없는 GLB 정점 수가 삼각형을 이루지 않습니다.");
        std::vector<std::uint32_t> indices(vertexCount);
        for (std::size_t index = 0; index < vertexCount; ++index)
            indices[index] = static_cast<std::uint32_t>(index);
        return indices;
    }

    const std::size_t accessorIndex = primitive.at("indices").get<std::size_t>();
    const std::uint32_t type = document.at("accessors").at(accessorIndex)
        .at("componentType").get<std::uint32_t>();
    const std::size_t elementSize = type == 5121U ? 1U : type == 5123U ? 2U : 4U;
    const Accessor source = ReadAccessor(document, accessorIndex, binary,
        "SCALAR", elementSize);
    if ((type != 5121U && type != 5123U && type != 5125U)
        || source.count > std::numeric_limits<std::uint32_t>::max()
        || source.count % 3U != 0U)
        throw std::runtime_error("GLB 삼각형 번호 자료 형식이 올바르지 않습니다.");
    std::vector<std::uint32_t> indices(source.count);
    for (std::size_t index = 0; index < indices.size(); ++index)
    {
        indices[index] = ReadIndex(source.bytes + index * source.stride,
            source.componentType);
        if (indices[index] >= vertexCount)
            throw std::runtime_error("GLB 삼각형이 존재하지 않는 정점을 가리킵니다.");
    }
    return indices;
}
} // namespace dronesim::graphics::glb_detail
