#include "scene/Entity.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#include "rendering/assets/AssetManager.h"
#include "model/GltfLoader.h"
#include "model/ModelResource.h"
#include "rendering/assets/PrefabFactory.h"
#include "rendering/components/RenderComponents.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "scene/Scene.h"
#include "scene/Scene.h"
#include "scene/TransformSystemModule.h"
#include "simulation/robotics/RobotTransformAdapter.h"
#include "TestSupport.h"

#include <glm/gtc/quaternion.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace grasplink::robotics;

namespace
{
using grasplink::graphics::Shader;
using grasplink::graphics::Window;
using grasplink::model::GltfLoader;
using grasplink::model::MeshData;
using grasplink::model::ModelResource;
using grasplink::model::NodeData;
using grasplink::model::ResourceID;
using grasplink::scene::Entity;
using grasplink::scene::Scene;
using grasplink::scene::TransformSystemModule;
using grasplink::rendering::AssetManager;
using grasplink::rendering::MeshFilter;
using grasplink::rendering::prefab_factory::CreateModel;

struct TestCheckFailure {};

void CheckImpl(bool condition, const std::string& label, const char* file, int line)
{
    if (condition) return;
    ADD_FAILURE_AT(file, line) << label;
    throw TestCheckFailure{};
}

void CheckNearImpl(double actual, double expected, double tolerance, const std::string& label,
    const char* file, int line)
{
    if (std::isfinite(actual) && std::abs(actual - expected) <= tolerance) return;
    const std::string detail = label + ": actual=" + std::to_string(actual) +
        ", expected=" + std::to_string(expected) + ", tolerance=" + std::to_string(tolerance);
    ADD_FAILURE_AT(file, line) << detail;
    throw TestCheckFailure{};
}

#define Check(...) CheckImpl(__VA_ARGS__, __FILE__, __LINE__)
#define CheckNear(...) CheckNearImpl(__VA_ARGS__, __FILE__, __LINE__)

template<typename Exception, typename Function>
void CheckThrows(Function&& function, const std::string& label)
{
    try
    {
        function();
        Check(false, label);
    }
    catch (const TestCheckFailure&)
    {
        throw;
    }
    catch (const Exception&)
    {
    }
}

void CheckPositionNear(const glm::vec3& actual, const glm::vec3& expected, const std::string& label)
{
    // Scene 기준 위치를 [m] 단위로 비교하고 축마다 0.2 mm 오차를 허용한다. GLB Entity 행렬은 float, 정기구학 계산은 double을 쓰므로 생기는 반올림 차이를 포함한다.
    const auto coordinateLabel = [&](const char* axis, float actualValue, float expectedValue)
    {
        return label + " " + axis + ": actual=" + std::to_string(actualValue) +
            ", expected=" + std::to_string(expectedValue);
    };
    CheckNear(actual.x, expected.x, 2e-4, coordinateLabel("x", actual.x, expected.x));
    CheckNear(actual.y, expected.y, 2e-4, coordinateLabel("y", actual.y, expected.y));
    CheckNear(actual.z, expected.z, 2e-4, coordinateLabel("z", actual.z, expected.z));
}

void ApplyAndCompare(
    flecs::world& world,
    Entity& robotRoot,
    grasplink::robotics::kinematics::RobotKinematics& kinematics,
    grasplink::simulation::robotics::RobotTransformAdapter& adapter,
    const std::array<double, 6>& jointRadians,
    const glm::vec3& rootPosition,
    const glm::quat& rootRotation,
    const std::string& label)
{
    // FK는 관절 각도 [rad]에서 각 링크와 도구 끝의 위치·방향을 계산한다. 여기서는 로봇 root의 Scene 위치와 회전까지 적용한 결과가 GLB 장면 계층의 World 자세와 같은지 비교한다.
    RobotState state;
    state.valid = true;
    state.jointPositionRadians.assign(jointRadians.begin(), jointRadians.end());
    const auto& fk = kinematics.Update(state);
    adapter.Apply(fk);

    robotRoot.SetLocalPosition(rootPosition);
    robotRoot.SetLocalRotation(rootRotation);
    TransformSystemModule::UpdateWorldTransforms(world);

    const Entity toolFrame = robotRoot.FindChildByNameRecursive("ToolFrame");
    Check(static_cast<bool>(toolFrame), label + ": GLB ToolFrame exists");
    const glm::mat4 rootWorld = robotRoot.GetWorldMatrix();
    const glm::mat4 toolWorld = toolFrame.GetWorldMatrix();
    // ToolFrame은 로봇이 작업할 때 기준으로 삼는 도구 끝 위치다. GLB 장면 계층에서 읽은 Scene 위치와 CPU FK가 계산한 위치를 비교한다.
    const glm::vec3 actualWorld = glm::vec3(toolWorld[3]);
    const glm::vec3 expectedWorld = glm::vec3(rootWorld * glm::vec4(
        static_cast<float>(fk.toolFrameInBaseFrame.positionMeters.x),
        static_cast<float>(fk.toolFrameInBaseFrame.positionMeters.y),
        static_cast<float>(fk.toolFrameInBaseFrame.positionMeters.z),
        1.0F));
    CheckPositionNear(actualWorld, expectedWorld, label + ": ToolFrame world position");

    const glm::quat actualOrientation = glm::quat_cast(glm::mat3(toolWorld));
    const auto& expectedRotation = fk.toolFrameInBaseFrame.rotation;
    const glm::quat expectedOrientation = glm::quat_cast(glm::mat3(rootWorld)) * glm::quat{
        static_cast<float>(expectedRotation.w), static_cast<float>(expectedRotation.x),
        static_cast<float>(expectedRotation.y), static_cast<float>(expectedRotation.z)};
    // quaternion과 그 음수는 같은 회전이므로, 두 값을 정규화한 내적의 절댓값으로 부호와 무관하게 회전 방향을 비교한다.
    Check(std::abs(glm::dot(glm::normalize(actualOrientation), glm::normalize(expectedOrientation))) > 0.9999F,
        label + ": ToolFrame world orientation");
}
}

/**
 * @brief 관절 각도에서 계산한 도구 끝 위치와 GLB 모델 계층에서 얻은 위치가 같은지 확인한다.
 * @details Forward Kinematics(FK)는 각 관절의 회전값으로 팔 링크와 도구 끝의 자세를 계산한다. 기본 자세와 개별·복합 관절 이동에서 이 결과를 실제 GLB 관절 계층과 비교한다.
 * 로봇 root를 옮기거나 돌린 경우도 확인하며, 모델의 관절 이름·부모·기준 위치가 사양과 다르면 연결을 거부해야 한다.
 * 숨겨진 Window가 OpenGL context를 제공하고 AssetManager와 Shader가 만든 GPU 자원을 해제한 뒤 마지막에 context를 닫는다.
 */
TEST(RobotPoseIntegration, MatchesKinematicsAndPreservesCachedMeshOwnership)
{
    try
    {
        // 보이지 않는 시험용 Window도 OpenGL context를 소유한다. AssetManager와 Shader가 만든 GPU 자원을 해제할 때까지 이 context를 유지해야 한다.
        Window window(Window::Properties{320, 240, "RobotPoseIntegrationTests", false, false});
        AssetManager assets;
        ModelResource model = GltfLoader::LoadGLB(
            std::filesystem::path("assets") / "HCR12A_2F-85.glb");
        assets.UploadModel(model);
        const auto shader = Shader::CreateFromSource(
            "RobotPoseTest",
            "#version 330 core\nlayout(location=0) in vec3 position; void main(){ gl_Position=vec4(position,1.0); }\n",
            "#version 330 core\nout vec4 color; void main(){ color=vec4(1.0); }\n");

        flecs::world world;
        world.import<TransformSystemModule>();
        Scene scene(world);
        Check(scene.GetSceneRoot().is_alive(), "Scene owns a live hierarchy root");

        ModelResource invalidHierarchy = model;
        invalidHierarchy.nodes.front().name = "PreflightFailureRoot";
        invalidHierarchy.nodes.front().parentIndex = 0;
        CheckThrows<std::runtime_error>(
            [&] { CreateModel(scene, invalidHierarchy, assets, shader); },
            "cyclic model hierarchy is rejected before Scene mutation");
        Check(!world.lookup("PreflightFailureRoot").is_valid(),
            "preflight failure leaves no partial model Entity in the Scene");

        Entity robotRoot = CreateModel(scene, model, assets, shader);
        const auto meshNode = std::find_if(model.nodes.begin(), model.nodes.end(),
            [](const NodeData& node) { return node.meshIndex >= 0; });
        Check(meshNode != model.nodes.end(), "robot model contains a renderable node");
        const std::size_t meshNodeIndex = static_cast<std::size_t>(meshNode - model.nodes.begin());
        const MeshData& renderableMesh = model.meshes[static_cast<std::size_t>(meshNode->meshIndex)];
        const ResourceID meshId = renderableMesh.uniqueID;
        const auto cachedMesh = assets.GetMesh(meshId);
        Check(cachedMesh != nullptr, "uploaded GPU mesh is available from AssetManager");
        const bool isRootMesh = meshNodeIndex == static_cast<std::size_t>(model.rootNodeIndex);
        const Entity meshEntity = renderableMesh.subMeshes.size() == 1
            ? (isRootMesh ? robotRoot : robotRoot.FindChildByNameRecursive(meshNode->name))
            : robotRoot.FindChildByNameRecursive(
                meshNode->name + "_Primitive_" + std::to_string(meshNodeIndex) + "_0");
        Check(meshEntity.IsValid(), "renderable model node has a Scene Entity");
        Check(meshEntity.Has<MeshFilter>(), "renderable node has a MeshFilter");
        Check(meshEntity.GetHandle().get<MeshFilter>().mesh == cachedMesh,
            "PrefabFactory attaches the cached GPU mesh to the Entity");

        robotRoot.SetLocalPosition({1.2F, -0.4F, 0.7F});
        robotRoot.SetLocalRotation(glm::quat{glm::vec3{0.3F, 0.7F, -0.4F}});
        const auto& specification = models::hanwha::kHcr12a;
        grasplink::simulation::robotics::RobotTransformAdapter adapter(robotRoot, specification);
        grasplink::robotics::kinematics::RobotKinematics kinematics(specification);

        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0, 0, 0, 0, 0, 0}, {}, {1, 0, 0, 0}, "zero pose");

        const double quarterTurn = std::acos(-1.0) * 0.5;
        for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
        {
            std::array<double, 6> angles{};
            angles[joint] = quarterTurn;
            ApplyAndCompare(world, robotRoot, kinematics, adapter,
                angles, {}, {1, 0, 0, 0}, "single joint " + std::to_string(joint + 1));
        }

        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0.45, -0.4, 0.3, 0.5, -0.2, 0.7}, {}, {1, 0, 0, 0}, "mixed pose");
        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0.45, -0.4, 0.3, 0.5, -0.2, 0.7},
            {1.2F, -0.4F, 0.7F}, glm::quat{glm::vec3{0.3F, 0.7F, -0.4F}}, "root translation and rotation");

        // 정상 기준 자세에서 시작해 관절 이름, 부모 계층과 기준 변환의 각 오류를 따로 만들어 연결 검사가 거부하는지 확인한다.
        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0, 0, 0, 0, 0, 0}, {}, {1, 0, 0, 0}, "restore bind pose");
        grasplink::simulation::robotics::RobotTransformAdapter validBind(robotRoot, specification);

        Entity j1 = robotRoot.FindChildByNameRecursive("J1");
        // 항등 quaternion의 음수도 같은 항등 회전이다. 성분 부호만 반대라는 이유로 정상 GLB 기준 자세를 거부하면 안 된다.
        j1.SetLocalRotation(glm::quat{-1.0F, 0.0F, 0.0F, 0.0F});
        grasplink::simulation::robotics::RobotTransformAdapter negativeIdentityBind(robotRoot, specification);
        const glm::vec3 originalJ1Position = j1.GetLocalPosition();
        j1.SetLocalPosition(originalJ1Position + glm::vec3{0.01F, 0.0F, 0.0F});
        CheckThrows<std::invalid_argument>(
            [&] { grasplink::simulation::robotics::RobotTransformAdapter bad(robotRoot, specification); },
            "changed GLB bind pivot rejected");
        j1.SetLocalPosition(originalJ1Position);

        Entity j2 = robotRoot.FindChildByNameRecursive("J2");
        Entity originalJ2Parent = j2.GetParent();
        j2.SetParent(robotRoot);
        CheckThrows<std::invalid_argument>(
            [&] { grasplink::simulation::robotics::RobotTransformAdapter bad(robotRoot, specification); },
            "broken joint ancestor hierarchy rejected");
        j2.SetParent(originalJ2Parent);

        assets.UploadModel(model);
        Check(assets.GetMesh(meshId) == cachedMesh,
            "uploading the same model reuses its cached GPU mesh");
        model = {};
        Check(meshEntity.GetHandle().get<MeshFilter>().mesh == cachedMesh,
            "Entity keeps the GPU mesh after the CPU model is released");

    }
    catch (const TestCheckFailure&)
    {
    }
    catch (const std::exception& error)
    {
        ADD_FAILURE() << error.what();
    }
}
