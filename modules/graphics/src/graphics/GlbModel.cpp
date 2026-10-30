#include "graphics/GlbModel.h"

#include "graphics/GlbData.h"
#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Texture2D.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aerosimlab::graphics
{
namespace
{
using glb_detail::Json;

glm::vec3 ReadVec3(const Json& values, const char* name,
    const glm::vec3& defaultValue)
{
    if (!values.contains(name))
        return defaultValue;
    const Json& array = values.at(name);
    if (!array.is_array() || array.size() != 3U)
        throw std::runtime_error(std::string("GLB 노드의 ") + name + " 값이 올바르지 않습니다.");
    const glm::vec3 result(array.at(0).get<float>(), array.at(1).get<float>(),
        array.at(2).get<float>());
    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z))
        throw std::runtime_error(std::string("GLB 노드의 ") + name + " 값이 유한하지 않습니다.");
    return result;
}

glm::mat4 ReadNodeTransform(const Json& node)
{
    // GLB는 행렬 하나 또는 이동·회전·크기 값을 저장한다. 둘 중 실제로 있는 형식을 읽는다.
    if (node.contains("matrix"))
    {
        const Json& values = node.at("matrix");
        if (!values.is_array() || values.size() != 16U)
            throw std::runtime_error("GLB 노드 변환 행렬이 올바르지 않습니다.");
        glm::mat4 result(1.0F);
        for (std::size_t index = 0; index < 16U; ++index)
        {
            const float value = values.at(index).get<float>();
            if (!std::isfinite(value))
                throw std::runtime_error("GLB 노드 변환 행렬에 유한하지 않은 값이 있습니다.");
            result[index / 4U][index % 4U] = value;
        }
        return result;
    }

    const glm::vec3 translation = ReadVec3(node, "translation", glm::vec3(0.0F));
    const glm::vec3 scale = ReadVec3(node, "scale", glm::vec3(1.0F));
    glm::quat rotation(1.0F, 0.0F, 0.0F, 0.0F);
    if (node.contains("rotation"))
    {
        const Json& values = node.at("rotation");
        if (!values.is_array() || values.size() != 4U)
            throw std::runtime_error("GLB 노드 회전 값이 올바르지 않습니다.");
        const float x = values.at(0).get<float>();
        const float y = values.at(1).get<float>();
        const float z = values.at(2).get<float>();
        const float w = values.at(3).get<float>();
        rotation = glm::quat(w, x, y, z);
        const float length = glm::length(rotation);
        if (!std::isfinite(length) || length < 1.0e-6F)
            throw std::runtime_error("GLB 노드 회전 값이 올바르지 않습니다.");
        rotation = glm::normalize(rotation);
    }
    // glTF 회전 순서와 쿼터니언 값(x,y,z,w)을 GLM 행렬로 옮긴다.
    glm::mat4 result = glm::translate(glm::mat4(1.0F), translation);
    const float x = rotation.x;
    const float y = rotation.y;
    const float z = rotation.z;
    const float w = rotation.w;
    glm::mat4 rotationMatrix(1.0F);
    rotationMatrix[0][0] = 1.0F - 2.0F * (y * y + z * z);
    rotationMatrix[0][1] = 2.0F * (x * y + z * w);
    rotationMatrix[0][2] = 2.0F * (x * z - y * w);
    rotationMatrix[1][0] = 2.0F * (x * y - z * w);
    rotationMatrix[1][1] = 1.0F - 2.0F * (x * x + z * z);
    rotationMatrix[1][2] = 2.0F * (y * z + x * w);
    rotationMatrix[2][0] = 2.0F * (x * z + y * w);
    rotationMatrix[2][1] = 2.0F * (y * z - x * w);
    rotationMatrix[2][2] = 1.0F - 2.0F * (x * x + y * y);
    result *= rotationMatrix;
    return glm::scale(result, scale);
}

float ReadFactor(const Json& values, const char* name, float defaultValue)
{
    const float value = values.value(name, defaultValue);
    if (!std::isfinite(value) || value < 0.0F || value > 1.0F)
        throw std::runtime_error(std::string("GLB 재질의 ") + name + " 값은 0부터 1 사이여야 합니다.");
    return value;
}

glm::vec4 ReadBaseColorFactor(const Json& pbr)
{
    if (!pbr.contains("baseColorFactor"))
        return glm::vec4(1.0F);
    const Json& values = pbr.at("baseColorFactor");
    if (!values.is_array() || values.size() != 4U)
        throw std::runtime_error("GLB 기본색 값이 올바르지 않습니다.");
    glm::vec4 result;
    for (std::size_t index = 0; index < 4U; ++index)
    {
        result[static_cast<glm::length_t>(index)] = values.at(index).get<float>();
        if (!std::isfinite(result[static_cast<glm::length_t>(index)])
            || result[static_cast<glm::length_t>(index)] < 0.0F
            || result[static_cast<glm::length_t>(index)] > 1.0F)
            throw std::runtime_error("GLB 기본색 값은 0부터 1 사이여야 합니다.");
    }
    return result;
}

std::unique_ptr<Texture2D> LoadBaseColorTexture(const Json& document,
    const Json& pbr, const std::vector<std::uint8_t>& binary,
    bool hasTextureCoordinates, const std::filesystem::path& modelDirectory)
{
    // 재질의 기본색 이미지를 읽어 메시의 UV 좌표와 연결한다.
    if (!pbr.contains("baseColorTexture"))
        return nullptr;
    const Json& textureInfo = pbr.at("baseColorTexture");
    if (textureInfo.value("texCoord", 0U) != 0U)
        throw std::runtime_error("GLB 기본색 이미지는 TEXCOORD_0 좌표만 지원합니다.");
    if (!hasTextureCoordinates)
        throw std::runtime_error("GLB 기본색 이미지를 쓰려면 TEXCOORD_0 정점 자료가 필요합니다.");

    const std::size_t textureIndex = textureInfo.at("index").get<std::size_t>();
    const Json& texture = document.at("textures").at(textureIndex);
    bool repeat = true;
    if (texture.contains("sampler"))
    {
        const Json& sampler = document.at("samplers").at(
            texture.at("sampler").get<std::size_t>());
        const auto readWrap = [&sampler](const char* name)
        {
            const std::uint32_t mode = sampler.value(name, 10497U);
            if (mode != 10497U && mode != 33071U)
                throw std::runtime_error("GLB 텍스처의 mirrored-repeat 설정은 지원하지 않습니다.");
            return mode;
        };
        const std::uint32_t wrapS = readWrap("wrapS");
        const std::uint32_t wrapT = readWrap("wrapT");
        if ((wrapS == 10497U) != (wrapT == 10497U))
            throw std::runtime_error("가로·세로 반복 설정이 다른 GLB 텍스처는 지원하지 않습니다.");
        repeat = wrapS == 10497U;
    }
    const std::size_t imageIndex = texture.at("source").get<std::size_t>();
    const Json& image = document.at("images").at(imageIndex);
    if (image.contains("uri"))
    {
        const std::string uri = image.at("uri").get<std::string>();
        if (uri.starts_with("data:"))
            throw std::runtime_error("GLB data URI 이미지는 지원하지 않습니다.");
        const std::filesystem::path imagePath{uri};
        if (imagePath.is_absolute())
            throw std::runtime_error("GLB 이미지는 모델 폴더 안의 상대 경로여야 합니다.");
        return Texture2D::LoadFromFile(modelDirectory / imagePath, repeat);
    }
    if (!image.contains("bufferView") || !image.contains("mimeType"))
        throw std::runtime_error("GLB 이미지에는 내부 bufferView와 PNG/JPEG 형식 정보가 필요합니다.");

    const std::string mimeType = image.at("mimeType").get<std::string>();
    if (mimeType != "image/png" && mimeType != "image/jpeg")
        throw std::runtime_error("GLB 기본색 이미지는 PNG 또는 JPEG만 지원합니다.");

    const Json& view = document.at("bufferViews").at(
        image.at("bufferView").get<std::size_t>());
    if (view.value("buffer", 0U) != 0U)
        throw std::runtime_error("GLB 이미지는 내부 BIN 버퍼에 저장해야 합니다.");
    const std::size_t offset = view.value("byteOffset", std::size_t{0});
    const std::size_t length = view.at("byteLength").get<std::size_t>();
    if (offset > binary.size() || length > binary.size() - offset)
        throw std::runtime_error("GLB 이미지 자료가 파일 범위를 벗어났습니다.");
    return Texture2D::LoadFromMemory(
        std::span<const std::uint8_t>(binary.data() + offset, length), repeat);
}

void BuildNormalsIfMissing(std::vector<Vertex>& vertices,
    const std::vector<std::uint32_t>& indices, const Json& primitive)
{
    // 법선이 빠진 메시만 삼각형 면적을 가중치로 삼아 정점 법선을 계산한다.
    if (primitive.at("attributes").contains("NORMAL"))
        return;
    std::vector<glm::vec3> sums(vertices.size(), glm::vec3(0.0F));
    for (std::size_t index = 0; index < indices.size(); index += 3U)
    {
        const std::uint32_t a = indices[index];
        const std::uint32_t b = indices[index + 1U];
        const std::uint32_t c = indices[index + 2U];
        const glm::vec3 normal = glm::cross(
            vertices[b].position - vertices[a].position,
            vertices[c].position - vertices[a].position);
        sums[a] += normal;
        sums[b] += normal;
        sums[c] += normal;
    }
    for (std::size_t index = 0; index < vertices.size(); ++index)
        vertices[index].normal = glm::length(sums[index]) > 1.0e-8F
            ? glm::normalize(sums[index]) : glm::vec3(0.0F, 1.0F, 0.0F);
}

} // namespace

struct GlbModel::Part
{
    std::unique_ptr<Mesh> mesh;
    std::unique_ptr<Texture2D> texture;
    Material material{glm::vec4(1.0F), 0.0F, 1.0F};
    glm::mat4 nodeTransform{1.0F};
    bool hasMaterial = false;
};

GlbModel::~GlbModel() = default;

std::unique_ptr<GlbModel> GlbModel::Load(const std::string& path)
{
    // GLB 씬의 루트부터 자식 노드를 따라가며 메시, 재질, 충돌 자료를 만든다.
    glb_detail::Document source = glb_detail::ReadDocument(path);
    const std::filesystem::path modelDirectory = std::filesystem::path{path}.parent_path();
    const Json& document = source.json;
    const Json& nodes = document.at("nodes");
    const Json& meshes = document.at("meshes");
    const Json& scenes = document.at("scenes");
    if (!nodes.is_array() || nodes.empty() || !meshes.is_array() || meshes.empty()
        || !scenes.is_array() || scenes.empty())
        throw std::runtime_error("GLB 장면에 노드·메시·씬이 필요합니다.");

    const std::size_t sceneIndex = document.value("scene", std::size_t{0});
    const Json& roots = scenes.at(sceneIndex).at("nodes");
    auto model = std::unique_ptr<GlbModel>(new GlbModel());
    std::vector<bool> recursionPath(nodes.size(), false);
    const auto addPrimitive = [&](const Json& node, const glm::mat4& nodeTransform)
    {
        const std::size_t meshIndex = node.at("mesh").get<std::size_t>();
        const Json& primitives = meshes.at(meshIndex).at("primitives");
        for (std::size_t primitiveIndex = 0;
            primitiveIndex < primitives.size(); ++primitiveIndex)
        {
            const Json& primitive = primitives.at(primitiveIndex);
            if (primitive.value("mode", 4U) != 4U)
                throw std::runtime_error("GLB는 삼각형 메시만 지원합니다.");
            std::vector<Vertex> vertices = glb_detail::ReadVertices(
                document, primitive, source.binary);
            std::vector<std::uint32_t> indices = glb_detail::ReadTriangleIndices(
                document, primitive, source.binary, vertices.size());
            if (indices.size() % 3U != 0U)
                throw std::runtime_error("GLB 삼각형 번호 수가 올바르지 않습니다.");
            BuildNormalsIfMissing(vertices, indices, primitive);

            // 경계는 월드 변환까지 반영해 두고, 충돌 정점은 로컬 좌표와 변환을 함께 보관한다.
            GlbPartBounds bounds{
                glm::vec3(std::numeric_limits<float>::max()),
                glm::vec3(std::numeric_limits<float>::lowest())};
            for (const Vertex& vertex : vertices)
            {
                const glm::vec3 position = glm::vec3(
                    nodeTransform * glm::vec4(vertex.position, 1.0F));
                bounds.minimum = glm::min(bounds.minimum, position);
                bounds.maximum = glm::max(bounds.maximum, position);
            }
            model->partBounds_.push_back(bounds);

            GlbCollisionPart collisionPart;
            collisionPart.vertices.reserve(vertices.size());
            for (const Vertex& vertex : vertices)
                collisionPart.vertices.push_back(vertex.position);
            collisionPart.triangleIndices = indices;
            collisionPart.nodeTransform = nodeTransform;
            model->collisionParts_.push_back(std::move(collisionPart));

            auto part = std::make_unique<Part>();
            part->mesh = std::make_unique<Mesh>(vertices.data(),
                static_cast<std::uint32_t>(vertices.size()), indices.data(),
                static_cast<std::uint32_t>(indices.size()));
            part->nodeTransform = nodeTransform;
            if (primitive.contains("material"))
            {
                const Json& material = document.at("materials").at(
                    primitive.at("material").get<std::size_t>());
                const Json pbr = material.value("pbrMetallicRoughness", Json::object());
                part->material = Material(ReadBaseColorFactor(pbr),
                    ReadFactor(pbr, "metallicFactor", 1.0F),
                    ReadFactor(pbr, "roughnessFactor", 1.0F));
                part->texture = LoadBaseColorTexture(document, pbr, source.binary,
                    primitive.at("attributes").contains("TEXCOORD_0"), modelDirectory);
                part->hasMaterial = true;
            }
            model->parts_.push_back(std::move(part));
        }
    };

    // 부모 변환을 자식에게 누적해 계층형 모델의 위치·방향·크기를 보존한다.
    std::function<void(std::size_t, const glm::mat4&)> visitNode;
    visitNode = [&](std::size_t nodeIndex, const glm::mat4& parentTransform)
    {
        if (nodeIndex >= nodes.size())
            throw std::runtime_error("GLB 씬이 존재하지 않는 노드를 가리킵니다.");
        if (recursionPath[nodeIndex])
            throw std::runtime_error("GLB 노드 연결에 순환 참조가 있습니다.");
        recursionPath[nodeIndex] = true;
        const Json& node = nodes.at(nodeIndex);
        const glm::mat4 worldTransform = parentTransform * ReadNodeTransform(node);
        if (node.contains("mesh"))
            addPrimitive(node, worldTransform);
        if (node.contains("children"))
        {
            for (const Json& child : node.at("children"))
                visitNode(child.get<std::size_t>(), worldTransform);
        }
        recursionPath[nodeIndex] = false;
    };

    for (const Json& root : roots)
        visitNode(root.get<std::size_t>(), glm::mat4(1.0F));
    if (model->parts_.empty())
        throw std::runtime_error("GLB 씬에 그릴 삼각형 메시가 없습니다.");
    return model;
}

void GlbModel::Draw(const glm::mat4& modelTransform, Renderer& renderer,
    Shader& shader, const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition, const Material* fallbackMaterial,
    const Texture2D* fallbackTexture, bool fallbackTriplanar) const
{
    // GLB 재질이 있으면 이를 쓰고, 없을 때만 호출자가 준 기본 재질과 질감을 사용한다.
    static const Material defaultMaterial(glm::vec4(1.0F), 0.0F, 1.0F);
    for (const auto& part : parts_)
    {
        const Material& material = part->hasMaterial
            ? part->material : (fallbackMaterial != nullptr ? *fallbackMaterial : defaultMaterial);
        const Texture2D* texture = part->hasMaterial
            ? part->texture.get() : fallbackTexture;
        const bool triplanar = !part->hasMaterial && fallbackTriplanar;
        renderer.Draw(modelTransform * part->nodeTransform, *part->mesh,
            material, shader, view, projection, cameraPosition, texture, triplanar);
    }
}

void GlbModel::SetInstances(std::span<const glm::mat4> modelTransforms)
{
    for (const auto& part : parts_)
    {
        std::vector<glm::mat4> partTransforms;
        partTransforms.reserve(modelTransforms.size());
        for (const glm::mat4& transform : modelTransforms)
            partTransforms.push_back(transform * part->nodeTransform);
        part->mesh->SetInstances(partTransforms);
    }
}

std::size_t GlbModel::PartCount() const noexcept
{
    return parts_.size();
}

std::span<const GlbPartBounds> GlbModel::PartBounds() const noexcept
{
    return partBounds_;
}

std::span<const GlbCollisionPart> GlbModel::CollisionParts() const noexcept
{
    return collisionParts_;
}
} // namespace aerosimlab::graphics
