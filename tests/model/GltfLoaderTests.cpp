#include "model/GltfLoader.h"

#include <gtest/gtest.h>

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

void AppendUInt32(Bytes& bytes, std::uint32_t value)
{
    for (unsigned int shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<unsigned char>((value >> shift) & 0xffU));
}

class TriangleGlb final
{
public:
    explicit TriangleGlb(std::size_t viewLength, int materialIndex = -1)
    {
        path_ = std::filesystem::temp_directory_path() /
            ("grasplink-triangle-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) + ".glb");

        const std::string primitiveMaterial = materialIndex < 0
            ? std::string{}
            : ",\"material\":" + std::to_string(materialIndex);
        std::string json =
            "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":36}],"
            "\"bufferViews\":[{\"buffer\":0,\"byteLength\":" + std::to_string(viewLength) +
            "}],\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}],"
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}" +
            primitiveMaterial + "}]}],"
            "\"nodes\":[{\"mesh\":0}],\"scenes\":[{\"nodes\":[0]}],\"scene\":0}";

        Bytes binary(36U);
        constexpr std::array<float, 9> positions{
            0.0F, 0.0F, 0.0F,
            1.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F};
        std::memcpy(binary.data(), positions.data(), sizeof(positions));
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
}
