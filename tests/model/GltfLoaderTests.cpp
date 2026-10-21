#include "model/GltfLoader.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Bytes = std::vector<unsigned char>;
constexpr int gltfUnsignedByteComponentType = 5121;

struct TriangleGlbOptions
{
    std::size_t positionBufferViewLength = 36U;
    int positionAccessorIndex = 0;
    int indexAccessorIndex = 1;
    int materialIndex = -1;
    std::string materialJson;
    bool includeByteIndices = false;
};

void AppendUInt32(Bytes& bytes, std::uint32_t value)
{
    for (unsigned int shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<unsigned char>((value >> shift) & 0xffU));
}

class TriangleGlb final
{
public:
    explicit TriangleGlb(const TriangleGlbOptions& options = {})
    {
        constexpr std::array<std::array<float, 3>, 3> positions{{
            {0.0F, 0.0F, 0.0F},
            {1.0F, 0.0F, 0.0F},
            {0.0F, 1.0F, 0.0F}}};
        const std::size_t positionByteLength = positions.size() * sizeof(positions[0]);
        Bytes indexBytes;
        if (options.includeByteIndices)
        {
            for (const std::uint32_t index : {0U, 1U, 2U})
                indexBytes.push_back(static_cast<unsigned char>(index));
        }
        const std::size_t bufferLength = positionByteLength + indexBytes.size();
        path_ = std::filesystem::temp_directory_path() /
            ("grasplink-triangle-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) + ".glb");

        const std::string primitiveMaterial = options.materialIndex < 0
            ? std::string{}
            : ",\"material\":" + std::to_string(options.materialIndex);
        const std::string materialArray = options.materialJson.empty()
            ? std::string{}
            : "\"materials\":[" + options.materialJson + "],";
        const std::string primitiveIndices = options.includeByteIndices
            ? ",\"indices\":" + std::to_string(options.indexAccessorIndex)
            : "";
        const std::string bufferViews = options.includeByteIndices
            ? "\"bufferViews\":[{\"buffer\":0,\"byteLength\":" +
                std::to_string(options.positionBufferViewLength) + "},{\"buffer\":0,\"byteOffset\":" +
                std::to_string(positionByteLength) + ",\"byteLength\":" +
                std::to_string(indexBytes.size()) + "}]"
            : "\"bufferViews\":[{\"buffer\":0,\"byteLength\":" +
                std::to_string(options.positionBufferViewLength) + "}]";
        const std::string accessors = options.includeByteIndices
            ? "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":" +
                std::to_string(gltfUnsignedByteComponentType) + ",\"count\":3,\"type\":\"SCALAR\"}]"
            : "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}]";
        std::string json =
            "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":" +
            std::to_string(bufferLength) + "}],"
            + bufferViews + "," + accessors + ","
            + materialArray +
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":" +
                std::to_string(options.positionAccessorIndex) + "}" +
            primitiveIndices + primitiveMaterial + "}]}],"
            + "\"nodes\":[{\"mesh\":0}],\"scenes\":[{\"nodes\":[0]}],\"scene\":0}";

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
 * @brief GLB 메시, 재질, 인덱스를 읽고 잘못된 입력을 거부하는지 확인한다.
 * @details 버퍼 범위와 재질 참조를 검사한다. 삼각형 구성, 장면 연결, 인덱스 형식도 지원 범위에 맞게 처리하는지 확인한다.
 */
TEST(GltfLoader, ReadsMeshMaterialAndIndexDataAndRejectsInvalidInputs)
{
    TriangleGlb validFile;
    const auto model = grasplink::model::GltfLoader::LoadGLB(validFile.Path());
    ASSERT_EQ(model.meshes.size(), 1U);
    ASSERT_EQ(model.meshes.front().vertices.size(), 3U);
    EXPECT_FLOAT_EQ(model.meshes.front().vertices[1].position.x, 1.0F);
    EXPECT_FLOAT_EQ(model.meshes.front().vertices[2].position.y, 1.0F);

    TriangleGlbOptions shortViewOptions;
    shortViewOptions.positionBufferViewLength = 24U;
    TriangleGlb shortView(shortViewOptions);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(shortView.Path()), std::runtime_error);

    TriangleGlbOptions invalidAccessorOptions;
    invalidAccessorOptions.positionAccessorIndex = 1;
    TriangleGlb invalidAccessor(invalidAccessorOptions);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(invalidAccessor.Path()), std::runtime_error);

    TriangleGlbOptions invalidIndexAccessorOptions;
    invalidIndexAccessorOptions.includeByteIndices = true;
    invalidIndexAccessorOptions.indexAccessorIndex = 2;
    TriangleGlb invalidIndexAccessor(invalidIndexAccessorOptions);
    EXPECT_THROW(
        grasplink::model::GltfLoader::LoadGLB(invalidIndexAccessor.Path()),
        std::runtime_error);

    TriangleGlbOptions missingMaterialOptions;
    missingMaterialOptions.materialIndex = 0;
    TriangleGlb missingMaterial(missingMaterialOptions);
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(missingMaterial.Path()), std::runtime_error);

    TriangleGlbOptions validMaterialOptions;
    validMaterialOptions.materialIndex = 0;
    validMaterialOptions.materialJson =
        R"({"pbrMetallicRoughness":{"baseColorFactor":[0.1,0.2,0.3,0.4],"metallicFactor":0.5,"roughnessFactor":0.75}})";
    TriangleGlb validMaterial(validMaterialOptions);
    const auto materialModel = grasplink::model::GltfLoader::LoadGLB(validMaterial.Path());
    ASSERT_EQ(materialModel.materials.size(), 1U);
    EXPECT_FLOAT_EQ(materialModel.materials[0].baseColorFactor.g, 0.2F);
    EXPECT_FLOAT_EQ(materialModel.materials[0].metallicFactor, 0.5F);
    EXPECT_FLOAT_EQ(materialModel.materials[0].roughnessFactor, 0.75F);

    TriangleGlbOptions byteIndicesOptions;
    byteIndicesOptions.includeByteIndices = true;
    TriangleGlb byteIndices(byteIndicesOptions);
    const auto indexedModel = grasplink::model::GltfLoader::LoadGLB(byteIndices.Path());
    ASSERT_EQ(indexedModel.meshes.size(), 1U);
    EXPECT_EQ(indexedModel.meshes.front().indices, (std::vector<std::uint32_t>{0U, 1U, 2U}));
}
