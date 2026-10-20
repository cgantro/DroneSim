#include "rendering/assets/AssetManager.h"

#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "graphics/Texture.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace grasplink::rendering
{
namespace
{
void ValidatePixelBuffer(const ::grasplink::model::TextureData& textureData)
{
    if (textureData.width <= 0 || textureData.height <= 0 ||
        (textureData.channels != 1 && textureData.channels != 3 && textureData.channels != 4))
        throw std::runtime_error("Invalid texture dimensions or channel count");

    const std::size_t width = static_cast<std::size_t>(textureData.width);
    const std::size_t height = static_cast<std::size_t>(textureData.height);
    const std::size_t channels = static_cast<std::size_t>(textureData.channels);
    const std::size_t maximum = std::numeric_limits<std::size_t>::max();
    if (width > maximum / height || width * height > maximum / channels)
        throw std::runtime_error("Texture pixel buffer size overflows");

    const std::size_t requiredBytes = width * height * channels;
    if (textureData.pixels.size() < requiredBytes)
        throw std::runtime_error("Texture pixel buffer is smaller than its dimensions");
}

void ValidateMeshData(const ::grasplink::model::MeshData& meshData)
{
    if (meshData.vertices.empty())
        throw std::runtime_error("Cannot upload mesh without vertices: " + meshData.name);
    if (meshData.indices.empty())
        throw std::runtime_error("Cannot upload mesh without indices: " + meshData.name);

    const std::size_t maximum = std::numeric_limits<std::uint32_t>::max();
    if (meshData.vertices.size() > maximum)
        throw std::runtime_error("Mesh vertex count exceeds uint32_t range");
    if (meshData.indices.size() > maximum)
        throw std::runtime_error("Mesh index count exceeds uint32_t range");
}
}

AssetManager::AssetManager()
{
    defaultMaterial_ = std::make_shared<::grasplink::graphics::Material>(
        glm::vec4{0.7F, 0.7F, 0.7F, 1.0F},
        0.0F,
        0.8F);
}

void AssetManager::UploadModel(const ::grasplink::model::ModelResource& model)
{
    // 잘못된 데이터 때문에 업로드가 중간에 멈춰도 일부 자원이 캐시에 남지 않도록 먼저 확인한다.
    for (const ::grasplink::model::TextureData& textureData : model.textures)
    {
        if (textures_.find(textureData.uniqueID) == textures_.end() &&
            !textureData.pixels.empty())
            ValidatePixelBuffer(textureData);
    }

    for (const ::grasplink::model::MaterialData& materialData : model.materials)
    {
        if (materials_.find(materialData.uniqueID) != materials_.end() ||
            !materialData.baseColorTexture.IsValid() ||
            textures_.find(materialData.baseColorTexture) != textures_.end())
            continue;

        const auto textureData = std::find_if(model.textures.begin(), model.textures.end(),
            [&](const ::grasplink::model::TextureData& texture) {
                return texture.uniqueID == materialData.baseColorTexture &&
                    !texture.pixels.empty();
            });
        if (textureData == model.textures.end())
            throw std::runtime_error("Base color texture was not uploaded");
    }

    for (const ::grasplink::model::MeshData& meshData : model.meshes)
        if (meshes_.find(meshData.uniqueID) == meshes_.end())
            ValidateMeshData(meshData);

    // 이미지, 재질, 형상 순서로 올린다. 뒤에 처리할 데이터가 앞서 만든 자원을 번호로 찾기 때문이다.

    for (const ::grasplink::model::TextureData& textureData : model.textures)
    {
        if (textures_.find(textureData.uniqueID) != textures_.end()) continue;
        if (textureData.pixels.empty()) continue;
        // 현재 업로드 경로는 모델의 모든 texture 항목을 sRGB 형식으로 만든다. 화면용 재질에는 기본색 이미지만 연결한다.
        // 다른 종류의 이미지를 실제 재질 입력으로 연결하려면 용도에 맞는 색 공간 처리가 필요하다.
        auto texture = ::grasplink::graphics::Texture::Create2D(
            textureData.width,
            textureData.height,
            textureData.channels,
            textureData.pixels.data(),
            true);

        textures_.emplace(textureData.uniqueID, texture);
    }

    for (const ::grasplink::model::MaterialData& materialData : model.materials)
    {
        if (materials_.find(materialData.uniqueID) != materials_.end()) continue;

        auto material = std::make_shared<::grasplink::graphics::Material>(
            materialData.baseColorFactor,
            materialData.metallicFactor,
            materialData.roughnessFactor);

        if (materialData.baseColorTexture.IsValid())
        {
            // 재질 번호 자체가 없는 표면은 PrefabFactory가 기본 Material을 선택한다. 여기서는 재질이 기본색 Texture를 지정했으므로 업로드 결과가 없으면 잘못된 참조로 처리한다.
            const auto textureIterator = textures_.find(materialData.baseColorTexture);
            if (textureIterator == textures_.end())
                throw std::runtime_error("Base color texture was not uploaded");

            material->SetBaseColorTexture(textureIterator->second);
        }

        materials_.emplace(materialData.uniqueID, std::move(material));
    }

    for (const ::grasplink::model::MeshData& meshData : model.meshes)
    {
        const auto existing = meshes_.find(meshData.uniqueID);
        if (existing != meshes_.end())
        {
            continue;
        }

        auto gpuMesh = std::make_shared<::grasplink::graphics::Mesh>(
            meshData.vertices.data(),
            static_cast<std::uint32_t>(meshData.vertices.size()),
            meshData.indices.data(),
            static_cast<std::uint32_t>(meshData.indices.size()));

        meshes_.emplace(meshData.uniqueID, std::move(gpuMesh));
    }
}

std::shared_ptr<::grasplink::graphics::Mesh> AssetManager::GetMesh(::grasplink::model::ResourceID id) const
{
    const auto iterator = meshes_.find(id);
    if (iterator == meshes_.end()) return nullptr;
    return iterator->second;
}

std::shared_ptr<::grasplink::graphics::Material> AssetManager::GetMaterial(::grasplink::model::ResourceID id) const
{
    const auto iterator = materials_.find(id);
    if (iterator == materials_.end()) return nullptr;
    return iterator->second;
}

std::shared_ptr<::grasplink::graphics::Material> AssetManager::GetDefaultMaterial() const
{
    return defaultMaterial_;
}

} // namespace grasplink::rendering
