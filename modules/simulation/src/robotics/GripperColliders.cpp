#include "simulation/robotics/GripperColliders.h"

#include "scene/Entity.h"
#include "model/ModelResource.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "CollisionGeometry.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <array>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasplink::simulation
{
using grasplink::model::MeshData;
using grasplink::model::ModelResource;
using grasplink::model::NodeData;
using grasplink::model::SubMeshInfo;
using grasplink::scene::Entity;
using grasplink::scene::Scene;

namespace
{
struct ProxySpec
{
    const char* root;
    std::array<const char*, 2> meshes;
    std::size_t meshCount;
    GripperCollisionPart part;
};

// 모델에서 지정한 Body/joint 좌표계마다 7개 강체 부품을 구성한다. 현재 자산의 메시 9개는 메시별로 볼록 충돌 외피 하나씩 만든다.
constexpr std::array<ProxySpec, 7> ProxySpecs{{
    {"Gripper", {"GripperMesh", nullptr}, 1, GripperCollisionPart::Body},
    {"LeftOuterKnuckleJoint", {"LeftOuterKnuckleMesh", "LeftFingerMesh"}, 2, GripperCollisionPart::Other},
    {"RightOuterKnuckleJoint", {"RightOuterKnuckleMesh", "RightFingerMesh"}, 2, GripperCollisionPart::Other},
    {"LeftInnerKnuckleJoint", {"LeftInnerKnuckleMesh", nullptr}, 1, GripperCollisionPart::Other},
    {"RightInnerKnuckleJoint", {"RightInnerKnuckleMesh", nullptr}, 1, GripperCollisionPart::Other},
    {"LeftFingerTipJoint", {"LeftFingerTipMesh", nullptr}, 1, GripperCollisionPart::LeftFingerTip},
    {"RightFingerTipJoint", {"RightFingerTipMesh", nullptr}, 1, GripperCollisionPart::RightFingerTip}}};

bool Finite(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool Finite(const glm::quat& value)
{
    const float lengthSquared = glm::dot(value, value);
    return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) &&
        std::isfinite(value.z) && std::isfinite(lengthSquared) && lengthSquared > 0.0F;
}

bool IsUnitScale(const glm::vec3& scale)
{
    return Finite(scale) && std::abs(scale.x - 1.0F) <= 1.0e-5F &&
        std::abs(scale.y - 1.0F) <= 1.0e-5F && std::abs(scale.z - 1.0F) <= 1.0e-5F;
}

bool HasName(const Entity& entity, const std::string& expected)
{
    const char* name = entity.GetHandle().name();
    return name != nullptr && expected == name;
}

std::vector<physics::CollisionShapeDescription> MeshShapes(const ModelResource& model,
    const std::vector<glm::mat4>& nodeTransforms, std::size_t meshNodeIndex, std::size_t rootIndex)
{
    const NodeData& node = model.nodes[meshNodeIndex];
    if (!node.meshIndex || *node.meshIndex >= model.meshes.size())
        throw std::invalid_argument("TwoF85 collider mesh node has no valid mesh.");
    // GLB 메시 node에서 소유 joint/body까지의 내부 변환만 합성해 정점 [m]을 해당 부품 좌표계로 옮긴다.
    // 모델 작성자가 지정한 소유 root 변환은 proxy의 부모 계층에 남겨 둔다. 실행 중 Scene이 이 변환을 한 번 적용한다.
    const glm::mat4 meshToRoot = glm::inverse(nodeTransforms[rootIndex]) * nodeTransforms[meshNodeIndex];
    const MeshData& mesh = model.meshes[*node.meshIndex];
    std::vector<physics::CollisionShapeDescription> shapes;
    for (const SubMeshInfo& primitive : mesh.subMeshes)
    {
        const std::size_t end = static_cast<std::size_t>(primitive.indexStart) + primitive.indexCount;
        if (end > mesh.indices.size())
            throw std::invalid_argument("TwoF85 mesh primitive has an invalid index range.");

        std::vector<bool> usedVertices(mesh.vertices.size(), false);
        std::vector<glm::vec3> vertices;
        vertices.reserve(primitive.indexCount);
        for (std::size_t index = primitive.indexStart; index < end; ++index)
        {
            const std::uint32_t vertexIndex = mesh.indices[index];
            if (vertexIndex >= mesh.vertices.size())
                throw std::invalid_argument("TwoF85 mesh primitive has an invalid vertex index.");
            if (usedVertices[vertexIndex])
                continue;
            usedVertices[vertexIndex] = true;
            const glm::vec3& position = mesh.vertices[vertexIndex].position;
            if (!Finite(position)) throw std::invalid_argument("TwoF85 mesh has a non-finite vertex.");
            const glm::vec3 point(meshToRoot * glm::vec4(position, 1.0F));
            if (!Finite(point)) throw std::invalid_argument("TwoF85 mesh transform is not finite.");
            vertices.push_back(point);
        }

        auto support = detail::BuildConvexSupportPoints(vertices);
        if (!detail::HasHullVolume(support))
            continue;
        physics::CollisionShapeDescription shape;
        shape.type = physics::CollisionShapeType::ConvexHull;
        shape.pointsMeters = std::move(support);
        shapes.push_back(std::move(shape));
    }
    if (shapes.empty()) throw std::invalid_argument("TwoF85 mesh has no non-degenerate collision primitive.");
    return shapes;
}
}

void ConfigureTwoF85Colliders(Scene& scene, const Entity& robotRoot, const ModelResource& model)
{
    if (!robotRoot) throw std::invalid_argument("TwoF85 collider robot root is invalid.");
    const flecs::entity sceneRoot = scene.GetSceneRoot();
    bool belongsToScene = false;
    for (Entity current = robotRoot; current; current = current.GetParent())
        if (current.GetHandle() == sceneRoot) { belongsToScene = true; break; }
    if (!belongsToScene) throw std::invalid_argument("TwoF85 robot root does not belong to the supplied scene.");
    for (Entity current = robotRoot; current && current.GetHandle() != sceneRoot; current = current.GetParent())
        if (!IsUnitScale(current.GetLocalScale()))
            throw std::invalid_argument("TwoF85 scene ancestry has non-unit scale.");
    std::unordered_map<std::string, std::size_t> modelIndices;
    std::unordered_set<std::string> expectedNodes;
    for (const ProxySpec& spec : ProxySpecs)
    {
        expectedNodes.emplace(spec.root);
        for (std::size_t i = 0; i < spec.meshCount; ++i) expectedNodes.emplace(spec.meshes[i]);
    }
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
        if (expectedNodes.count(model.nodes[i].name) != 0 && !modelIndices.emplace(model.nodes[i].name, i).second)
            throw std::invalid_argument("TwoF85 model contains duplicate node names.");

    // 충돌 메시만 보더라도 잘못된 부모 index나 순환 참조가 모델 다른 곳에 있으면 계층 계산이 안전하지 않으므로 GLB 전체를 먼저 검사한다. 단위 scale 조건은 실제 충돌 형상 경로에만 요구한다.
    const auto nodeTransforms = detail::BuildNodeWorldTransforms(model);

    auto validateAncestry = [&](std::size_t index)
    {
        while (true)
        {
            const NodeData& node = model.nodes[index];
            if (!Finite(node.translation) || !Finite(node.rotation) || !Finite(node.scale))
                throw std::invalid_argument("TwoF85 collider ancestry has invalid TRS.");
            if (!IsUnitScale(node.scale))
                throw std::invalid_argument("TwoF85 collider ancestry has non-unit scale.");
            if (!node.parentIndex) break;
            index = *node.parentIndex;
        }
    };

    struct Prepared
    {
        Entity parent;
        std::vector<physics::CollisionShapeDescription> shapes;
        std::string name;
        GripperCollisionPart part = GripperCollisionPart::Other;
    };
    std::array<Prepared, ProxySpecs.size()> prepared;
    for (std::size_t p = 0; p < ProxySpecs.size(); ++p)
    {
        const ProxySpec& spec = ProxySpecs[p];
        const auto rootIt = modelIndices.find(spec.root);
        if (rootIt == modelIndices.end()) throw std::invalid_argument("TwoF85 required model node is missing.");
        const std::size_t rootIndex = rootIt->second;
        validateAncestry(rootIndex);
        Entity rootEntity = robotRoot.FindChildByNameRecursive(spec.root);
        if (!rootEntity) throw std::invalid_argument("TwoF85 required scene entity is missing.");
        Entity liveAncestor = rootEntity;
        while (liveAncestor && liveAncestor != robotRoot)
        {
            if (!IsUnitScale(liveAncestor.GetLocalScale()))
                throw std::invalid_argument("TwoF85 scene joint ancestry has non-unit scale.");
            liveAncestor = liveAncestor.GetParent();
        }
        if (liveAncestor != robotRoot)
            throw std::invalid_argument("TwoF85 joint is outside the robot root hierarchy.");
        const std::size_t gripperIndex = modelIndices.at("Gripper");
        std::size_t modelAncestor = rootIndex;
        Entity sceneAncestor = rootEntity;
        while (modelAncestor != gripperIndex)
        {
            const auto parentIndex = model.nodes[modelAncestor].parentIndex;
            if (!parentIndex || !sceneAncestor)
                throw std::invalid_argument("TwoF85 joint is outside the Gripper hierarchy.");
            modelAncestor = *parentIndex;
            sceneAncestor = sceneAncestor.GetParent();
            if (!sceneAncestor || !HasName(sceneAncestor, model.nodes[modelAncestor].name))
                throw std::invalid_argument("TwoF85 scene joint hierarchy does not match the model.");
        }
        prepared[p].parent = rootEntity;
        prepared[p].name = std::string(spec.root) + "_CollisionProxy";
        prepared[p].part = spec.part;
        if (robotRoot.FindChildByNameRecursive(prepared[p].name))
            throw std::invalid_argument("TwoF85 collision proxy already exists.");

        for (std::size_t m = 0; m < spec.meshCount; ++m)
        {
            const auto meshIt = modelIndices.find(spec.meshes[m]);
            if (meshIt == modelIndices.end()) throw std::invalid_argument("TwoF85 required mesh node is missing.");
            const std::size_t meshIndex = meshIt->second;
            validateAncestry(meshIndex);
            std::size_t ancestor = meshIndex;
            while (ancestor != rootIndex && model.nodes[ancestor].parentIndex)
                ancestor = *model.nodes[ancestor].parentIndex;
            if (ancestor != rootIndex) throw std::invalid_argument("TwoF85 mesh is outside its owning joint hierarchy.");
            Entity meshEntity = robotRoot.FindChildByNameRecursive(spec.meshes[m]);
            if (!meshEntity) throw std::invalid_argument("TwoF85 required scene mesh entity is missing.");
            // Scene Prefab에서 메시부터 소유 joint까지 이어지는 부모 경로가 GLB와 같은지 확인한다. 경로가 다르면 다른 부품의 메시를 잘못 붙일 수 있어 거부한다.
            Entity sceneAncestor = meshEntity;
            std::size_t modelMeshAncestor = meshIndex;
            while (true)
            {
                if (!sceneAncestor || !HasName(sceneAncestor, model.nodes[modelMeshAncestor].name))
                    throw std::invalid_argument("TwoF85 scene mesh hierarchy does not match the model.");
                if (!IsUnitScale(sceneAncestor.GetLocalScale()))
                    throw std::invalid_argument("TwoF85 scene mesh ancestry has non-unit scale.");
                if (modelMeshAncestor == rootIndex) break;
                const auto parentIndex = model.nodes[modelMeshAncestor].parentIndex;
                if (!parentIndex) throw std::invalid_argument("TwoF85 mesh is outside its owning joint hierarchy.");
                modelMeshAncestor = *parentIndex;
                sceneAncestor = sceneAncestor.GetParent();
            }
            auto shapes = MeshShapes(model, nodeTransforms, meshIndex, rootIndex);
            prepared[p].shapes.insert(prepared[p].shapes.end(),
                std::make_move_iterator(shapes.begin()), std::make_move_iterator(shapes.end()));
        }
    }

    // 충돌 proxy를 모델에 지정된 Body/joint Entity의 자식으로 두고 Local 위치·회전을 항등값으로 둔다. 그러면 원본 관절의 Local 회전이 부모 계층을 따라 proxy에도 반영된다.
    // 앱이 GripperState에서 계산한 관절 회전을 원본 Entity에 적용하면, 같은 부모 계층에 붙은 본체와 여섯 관절의 일곱 proxy가 함께 이동한다.
    // Fixed Update에서 Scene World 변환을 먼저 다시 계산해야 한다. 그 다음 PhysicsSystemModule이 이 변환을 Kinematic Body의 목표 자세로 전달한다.
    for (Prepared& item : prepared)
    {
        Entity proxy = scene.CreateEntity(item.name);
        proxy.SetParent(item.parent);
        proxy.SetLocalPosition(glm::vec3(0.0F));
        proxy.SetLocalRotation(glm::quat{1.0F, 0.0F, 0.0F, 0.0F});
        proxy.SetLocalScale(glm::vec3(1.0F));
        proxy.set<RigidBody>(RigidBody{physics::BodyMotionType::Kinematic, physics::CollisionLayer::Gripper})
            .set<GripperCollisionProxy>(GripperCollisionProxy{item.part})
            .set<Colliders>(Colliders{std::move(item.shapes)});
    }
}
}
