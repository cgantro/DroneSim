#include "rendering/assets/PrefabFactory.h"

#include "rendering/assets/AssetManager.h"
#include "model/ModelResource.h"

#include "graphics/Material.h"
#include "graphics/Mesh.h"
#include "graphics/Shader.h"

#include "rendering/components/RenderComponents.h"
#include "scene/Scene.h"

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace grasplink::rendering
{

namespace
{
enum class NodeVisitState
{
    Unvisited,
    Visiting,
    Visited
};

/**
 * @brief 표면 묶음에 지정된 재질을 모델의 재질 목록과 자원 캐시에서 찾는다.
 * @details -1은 glTF 표면에 재질 번호가 없다는 뜻이므로 이 경우만 기본 Material을 쓴다. 목록 범위를 벗어난 번호나 GPU에 올리지 않은 재질은 잘못된 모델 연결이므로 예외를 던진다.
 */
std::shared_ptr<::grasplink::graphics::Material> ResolveMaterial(
    const ::grasplink::model::ModelResource& model,
    const AssetManager& assets,
    std::optional<std::size_t> materialIndex)
{
    if (!materialIndex) return assets.GetDefaultMaterial();

    if (*materialIndex >= model.materials.size())
        throw std::runtime_error("Invalid material index");

    const ::grasplink::model::MaterialData& materialData =
        model.materials[*materialIndex];

    std::shared_ptr<::grasplink::graphics::Material> material = assets.GetMaterial(materialData.uniqueID);
    if (!material)
        throw std::runtime_error("Material has not been uploaded");

    return material;
}

void ValidateNodeHierarchy(const ::grasplink::model::ModelResource& model)
{
    if (model.nodes.empty())
        throw std::runtime_error("Model has no nodes");
    if (!model.rootNodeIndex || *model.rootNodeIndex >= model.nodes.size())
        throw std::runtime_error("Model has invalid root node");

    std::vector<NodeVisitState> parentState(model.nodes.size(), NodeVisitState::Unvisited);
    for (std::size_t start = 0; start < model.nodes.size(); ++start)
    {
        std::vector<std::size_t> path;
        std::size_t current = start;
        while (parentState[current] == NodeVisitState::Unvisited)
        {
            parentState[current] = NodeVisitState::Visiting;
            path.push_back(current);
            const auto parent = model.nodes[current].parentIndex;
            if (parent && *parent >= model.nodes.size())
                throw std::runtime_error("Invalid parent index");
            if (!parent)
                break;
            current = *parent;
        }
        if (parentState[current] == NodeVisitState::Visiting && model.nodes[current].parentIndex)
            throw std::runtime_error("Model contains a parent cycle");
        for (const std::size_t index : path)
            parentState[index] = NodeVisitState::Visited;
    }
}

void ValidateRenderReferences(
    const ::grasplink::model::ModelResource& model,
    const AssetManager& assets)
{
    for (const ::grasplink::model::NodeData& node : model.nodes)
    {
        if (node.meshIndex && *node.meshIndex >= model.meshes.size())
            throw std::runtime_error("Invalid mesh index in model node");
        if (!node.meshIndex)
            continue;

        const ::grasplink::model::MeshData& mesh = model.meshes[*node.meshIndex];
        if (!assets.GetMesh(mesh.uniqueID))
            throw std::runtime_error("Mesh has not been uploaded: " + mesh.name);
        for (const ::grasplink::model::SubMeshInfo& subMesh : mesh.subMeshes)
        {
            const std::size_t begin = subMesh.indexStart;
            const std::size_t count = subMesh.indexCount;
            if (count == 0 || count % 3 != 0 || begin > mesh.indices.size() ||
                count > mesh.indices.size() - begin)
                throw std::runtime_error("Invalid primitive index range");
            (void)ResolveMaterial(model, assets, subMesh.defaultMaterialIndex);
            for (std::size_t index = begin; index < begin + count; ++index)
                if (mesh.indices[index] >= mesh.vertices.size())
                    throw std::runtime_error("Invalid vertex index in mesh " + mesh.name + " at " +
                        std::to_string(index) + ": " + std::to_string(mesh.indices[index]) + " >= " +
                        std::to_string(mesh.vertices.size()));
        }
    }
}

void ValidateModelBeforeSceneChanges(
    const ::grasplink::model::ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<::grasplink::graphics::Shader>& shader)
{
    if (!model.rootNodeIndex || *model.rootNodeIndex >= model.nodes.size() ||
        model.nodes[*model.rootNodeIndex].parentIndex)
    {
        throw std::runtime_error("PrefabFactory requires a valid scene root");
    }

    if (!shader)
        throw std::runtime_error("PrefabFactory requires a shader");

    ValidateNodeHierarchy(model);
    ValidateRenderReferences(model, assets);
}

/**
 * @brief 부품이 가리키는 형상과 그 표면별 그리기 범위를 Entity에 연결한다.
 * @details AssetManager::UploadModel은 삼각형 형상을 GPU에 올리고, PrefabFactory는 형상 식별자로 이를 찾아 Entity에 공유한다. 한 형상의 표면 묶음들은 같은 GPU Mesh를 공유하고 indexStart/indexCount로 각자 사용할 연결 번호 구간을 고른다.
 * 시작 위치는 번호 배열의 원소 개수이며 Renderer가 GPU 명령을 만들 때 byte 위치로 바꾼다. 표면이 하나면 부품 Entity가 직접 그린다.
 * 여러 표면은 재질이나 연결 번호 범위가 다를 수 있어 자식 Entity로 나눈다.
 */
void CreateRenderEntities(
    ::grasplink::scene::Scene& scene,
    const ::grasplink::model::ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<::grasplink::graphics::Shader>& shader,
    const ::grasplink::model::NodeData& node,
    std::size_t nodeIndex,
    ::grasplink::scene::Entity& nodeEntity)
{
    if (!node.meshIndex) return;

    const ::grasplink::model::MeshData& meshData = model.meshes[*node.meshIndex];
    const std::shared_ptr<::grasplink::graphics::Mesh> gpuMesh = assets.GetMesh(meshData.uniqueID);

    if (meshData.subMeshes.size() == 1)
    {
        const ::grasplink::model::SubMeshInfo& subMesh = meshData.subMeshes.front();
        nodeEntity
            .set<MeshFilter>(MeshFilter{gpuMesh, subMesh.indexStart, subMesh.indexCount})
            .set<MeshRenderer>(MeshRenderer{
                shader,
                ResolveMaterial(model, assets, subMesh.defaultMaterialIndex),
                true});
        return;
    }

    for (std::size_t primitiveIndex = 0;
         primitiveIndex < meshData.subMeshes.size();
         ++primitiveIndex)
    {
        const ::grasplink::model::SubMeshInfo& subMesh = meshData.subMeshes[primitiveIndex];

        const std::string renderEntityName =
            node.name + "_Primitive_" +
            std::to_string(nodeIndex) + "_" +
            std::to_string(primitiveIndex);

        // 새 자식의 부모 기준 위치와 회전은 0, 크기는 1이다. 별도 이동 없이 부모 부품의 위치·회전을 물려받게 한다.
        ::grasplink::scene::Entity renderEntity = scene.CreateEntity(renderEntityName);
        renderEntity.SetParent(nodeEntity);

        // 재질은 자원 번호로 찾아 공유한다. 파일에 재질을 지정하지 않은 표면만 기본 재질을 쓴다.
        std::shared_ptr<::grasplink::graphics::Material> material =
            ResolveMaterial(model, assets, subMesh.defaultMaterialIndex);

        renderEntity
            .set<MeshFilter>(MeshFilter{
                gpuMesh,
                subMesh.indexStart,
                subMesh.indexCount})
            .set<MeshRenderer>(MeshRenderer{
                shader,
                material,
                true});
    }
}
} // namespace

::grasplink::scene::Entity prefab_factory::CreateModel(
    ::grasplink::scene::Scene& scene,
    const ::grasplink::model::ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<::grasplink::graphics::Shader>& shader)
{
    // 참조와 계층을 먼저 검증해 잘못된 Asset 때문에 Scene에 일부 Entity만 남는 일을 막는다.
    ValidateModelBeforeSceneChanges(model, assets, shader);

    // 첫 단계: 부모 부품이 배열에서 자식 뒤에 있어도 되도록 모든 Entity를 먼저 만든다.
    std::vector<::grasplink::scene::Entity> entities(model.nodes.size());

    // 둘째 단계: 파일의 부모 기준 위치 [m], 길이 1인 회전값, 크기 배율을 Entity에 복사한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const ::grasplink::model::NodeData& node = model.nodes[i];
        ::grasplink::scene::Entity entity = scene.CreateEntity(node.name);

        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);

        entities[i] = entity;
    }

    // 셋째 단계: 미리 만든 Entity 번호로 부모 관계를 설정한다. SetParent는 부모 기준 위치·회전·크기를 유지한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const ::grasplink::model::NodeData& node = model.nodes[i];

        // 부모가 없는 최상위 Entity는 CreateEntity가 장면의 최상위 기준에 이미 연결했다.
        if (!node.parentIndex) continue;

        entities[i].SetParent(
            entities[*node.parentIndex]);
    }

    // 마지막 단계: 계층을 만든 뒤 형상 번호와 GPU 업로드, 재질 참조가 유효한지 확인해 화면 그리기 정보를 붙인다.
    // 중간에 실패해도 앞서 만든 Entity를 장면에서 지우는 되돌리기는 하지 않는다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        CreateRenderEntities(
            scene,
            model,
            assets,
            shader,
            model.nodes[i],
            i,
            entities[i]);
    }

    return entities[*model.rootNodeIndex];
}

} // namespace grasplink::rendering
