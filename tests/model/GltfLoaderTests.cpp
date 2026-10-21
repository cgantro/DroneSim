#include "model/GltfLoader.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Bytes = std::vector<unsigned char>;
constexpr int gltfUnsignedByteComponentType = 5121;

void AppendUInt32(Bytes& bytes, std::uint32_t value)
{
    for (unsigned int shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<unsigned char>((value >> shift) & 0xffU));
}

class TriangleGlb final
{
public:
    explicit TriangleGlb(std::size_t viewLength, int materialIndex = -1,
        std::size_t vertexCount = 3U, std::string material = {}, bool includeSceneNode = true,
        std::optional<int> indicesReference = std::nullopt, int sceneIndex = 0,
        bool includeByteIndices = false)
    {
        constexpr std::array<std::array<float, 3>, 4> positions{{
            {0.0F, 0.0F, 0.0F},
            {1.0F, 0.0F, 0.0F},
            {0.0F, 1.0F, 0.0F},
            {1.0F, 1.0F, 0.0F}}};
        if (vertexCount > positions.size())
            throw std::invalid_argument("triangle GLB test supports at most four vertices");
        const std::size_t positionByteLength = vertexCount * sizeof(positions[0]);
        Bytes indexBytes;
        if (includeByteIndices)
        {
            for (const std::uint32_t index : {0U, 1U, 2U})
                indexBytes.push_back(static_cast<unsigned char>(index));
        }
        const std::size_t bufferLength = positionByteLength + indexBytes.size();
        path_ = std::filesystem::temp_directory_path() /
            ("grasplink-triangle-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) + ".glb");

        const std::string primitiveMaterial = materialIndex < 0
            ? std::string{}
            : ",\"material\":" + std::to_string(materialIndex);
        const std::string materialArray = material.empty()
            ? std::string{}
            : "\"materials\":[" + material + "],";
        const std::string sceneNodes = includeSceneNode
            ? "\"nodes\":[{\"mesh\":0}],\"scenes\":[{\"nodes\":[0]}],\"scene\":" +
                std::to_string(sceneIndex)
            : "\"nodes\":[]";
        const std::string primitiveIndices = indicesReference.has_value()
            ? ",\"indices\":" + std::to_string(*indicesReference)
            : includeByteIndices ? ",\"indices\":1" : std::string{};
        const std::string bufferViews = includeByteIndices
            ? "\"bufferViews\":[{\"buffer\":0,\"byteLength\":" +
                std::to_string(viewLength) + "},{\"buffer\":0,\"byteOffset\":" +
                std::to_string(positionByteLength) + ",\"byteLength\":" +
                std::to_string(indexBytes.size()) + "}]"
            : "\"bufferViews\":[{\"buffer\":0,\"byteLength\":" +
                std::to_string(viewLength) + "}]";
        const std::string accessors = includeByteIndices
            ? "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":" +
                std::to_string(vertexCount) + ",\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":" +
                std::to_string(gltfUnsignedByteComponentType) + ",\"count\":3,\"type\":\"SCALAR\"}]"
            : "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":" +
                std::to_string(vertexCount) + ",\"type\":\"VEC3\"}]";
        std::string json =
            "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":" +
            std::to_string(bufferLength) + "}],"
            + bufferViews + "," + accessors + ","
            + materialArray +
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}" +
            primitiveIndices + primitiveMaterial + "}]}],"
            + sceneNodes + "}";

        Bytes binary(bufferLength);
        std::memcpy(binary.data(), positions.data(), positionByteLength);
        std::copy(indexBytes.begin(), indexBytes.end(), binary.begin() + positionByteLength);
        while (json.size() % 4U != 0U) json.push_back(' ');
        while (binary.size() % 4U != 0U) binary.push_back(0U);

        Bytes file;
        AppendUInt32(file, 0x46546c67U);
        AppendUInt32(file, 2U);
        AppendUInt32(file, static_cast<std::uint32_t>(12U + 8U + json.size() + 8U + binary.size()));
        AppendUInt32(file, static_cast<std::uint32_t>(json.size()));
        AppendUInt32(file, 0x4e4f534aU);
        file.insert(file.end(), json.begin(), json.end());
        AppendUInt32(file, static_cast<std::uint32_t>(binary.size()));
        AppendUInt32(file, 0x004e4942U);
        file.insert(file.end(), binary.begin(), binary.end());

        std::ofstream output(path_, std::ios::binary);
        output.write(reinterpret_cast<const char*>(file.data()),
            static_cast<std::streamsize>(file.size()));
        if (!output)
            throw std::runtime_error("could not write temporary GLB");
    }

    ~TriangleGlb()
    {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    [[nodiscard]] const std::filesystem::path& Path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};
}

/**
 * @brief GLB 정점 자료를 읽고 Accessor가 지정한 구역 밖의 읽기를 거부하는지 확인한다.
 * @details Accessor는 파일 안의 정점 배열에서 읽을 범위를 정한다. 지정 구역이 실제 정점보다 짧으면 로더가 범위를 벗어나 읽지 않고 오류를 알려야 한다.
 */
TEST(GltfLoader, ReadsPositionsAndRejectsAccessorOutsideBufferView)
{
    TriangleGlb validFile(36U);
    const auto model = grasplink::model::GltfLoader::LoadGLB(validFile.Path());
    ASSERT_EQ(model.meshes.size(), 1U);
    ASSERT_EQ(model.meshes.front().vertices.size(), 3U);
    EXPECT_FLOAT_EQ(model.meshes.front().vertices[1].position.x, 1.0F);
    EXPECT_FLOAT_EQ(model.meshes.front().vertices[2].position.y, 1.0F);

    TriangleGlb shortView(24U);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(shortView.Path()), std::runtime_error);

    TriangleGlb missingMaterial(36U, 0);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(missingMaterial.Path()), std::runtime_error);

    TriangleGlb validMaterial(36U, 0, 3U,
        R"({"pbrMetallicRoughness":{"baseColorFactor":[0.1,0.2,0.3,0.4],"metallicFactor":0.5,"roughnessFactor":0.75}})");
    const auto materialModel = grasplink::model::GltfLoader::LoadGLB(validMaterial.Path());
    ASSERT_EQ(materialModel.materials.size(), 1U);
    EXPECT_FLOAT_EQ(materialModel.materials[0].baseColorFactor.g, 0.2F);
    EXPECT_FLOAT_EQ(materialModel.materials[0].metallicFactor, 0.5F);
    EXPECT_FLOAT_EQ(materialModel.materials[0].roughnessFactor, 0.75F);

    TriangleGlb invalidMaterialFactor(36U, 0, 3U,
        R"({"pbrMetallicRoughness":{"metallicFactor":1.5}})");
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(invalidMaterialFactor.Path()),
        std::runtime_error);

    TriangleGlb invalidTextureReference(36U, 0, 3U,
        R"({"pbrMetallicRoughness":{"baseColorTexture":{"index":0}}})");
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(invalidTextureReference.Path()),
        std::runtime_error);

    TriangleGlb incompleteTriangle(48U, -1, 4U);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(incompleteTriangle.Path()), std::runtime_error);

    TriangleGlb missingSceneNode(36U, -1, 3U, {}, false);
    const auto meshOnlyModel = grasplink::model::GltfLoader::LoadGLB(missingSceneNode.Path());
    EXPECT_TRUE(meshOnlyModel.nodes.empty());
    EXPECT_FALSE(meshOnlyModel.rootNodeIndex.has_value());

    TriangleGlb invalidIndicesReference(36U, -1, 3U, {}, true, -2);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(invalidIndicesReference.Path()), std::runtime_error);

    TriangleGlb invalidNegativeScene(36U, -1, 3U, {}, true, std::nullopt, -2);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(invalidNegativeScene.Path()),
        std::runtime_error);

    TriangleGlb byteIndices(36U, -1, 3U, {}, true, std::nullopt, 0, true);
    const auto indexedModel = grasplink::model::GltfLoader::LoadGLB(byteIndices.Path());
    ASSERT_EQ(indexedModel.meshes.size(), 1U);
    EXPECT_EQ(indexedModel.meshes.front().indices, (std::vector<std::uint32_t>{0U, 1U, 2U}));
}
