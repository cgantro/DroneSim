#include "scene/Entity.h"
#include "scene/TransformComponents.h"
#include "scene/Scene.h"
#include "scene/Scene.h"
#include "scene/TransformSystemModule.h"
#include "TestSupport.h"

#include <gtest/gtest.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace
{
using namespace grasplink::scene;
using grasplink::scene::Entity;
using grasplink::scene::Scene;
using grasplink::scene::TransformSystemModule;

constexpr float kTolerance = 1.0e-5F;
const glm::quat kIdentity{1.0F, 0.0F, 0.0F, 0.0F};

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

void CheckMatrix(const glm::mat4& actual, const glm::mat4& expected, const std::string& label)
{
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            CheckNear(actual[column][row], expected[column][row], kTolerance,
                label + " [" + std::to_string(column) + "][" + std::to_string(row) + "]");
}

void CheckQuaternion(const glm::quat& actual, const glm::quat& expected, const std::string& label)
{
    CheckNear(glm::length(actual), 1.0, kTolerance, label + " unit length");
    // q와 -q는 성분 부호만 다르고 같은 공간 회전을 나타낸다. 저장된 부호가 아니라 실제 회전 방향이 같은지 확인한다.
    CheckNear(std::abs(glm::dot(actual, glm::normalize(expected))), 1.0, kTolerance,
        label + " orientation");
}

void CheckQuaternionComponents(const glm::quat& actual, const glm::quat& expected, const std::string& label)
{
    CheckNear(actual.w, expected.w, 0.0, label + " w");
    CheckNear(actual.x, expected.x, 0.0, label + " x");
    CheckNear(actual.y, expected.y, 0.0, label + " y");
    CheckNear(actual.z, expected.z, 0.0, label + " z");
}

glm::mat4 ReferenceMatrix(const glm::vec3& position, const glm::quat& rotation, const glm::vec3& scale)
{
    return glm::translate(glm::mat4(1.0F), position) * glm::mat4_cast(glm::normalize(rotation)) *
        glm::scale(glm::mat4(1.0F), scale);
}

std::array<glm::quat, 3> InvalidRotations()
{
    return {
        glm::quat{0.0F, 0.0F, 0.0F, 0.0F},
        glm::quat{1.0F, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F},
        glm::quat{1.0F, 0.0F, std::numeric_limits<float>::infinity(), 0.0F}};
}

void CheckDefaultsAndNormalization(Scene& scene)
{
    CheckQuaternionComponents(Rotation{}, kIdentity, "default Rotation");
    CheckQuaternionComponents(Entity{}.GetLocalRotation(), kIdentity, "invalid Entity rotation");
    Entity withoutRotation{scene.GetWorld().entity()};
    CheckQuaternionComponents(withoutRotation.GetLocalRotation(), kIdentity, "missing rotation pair");
    Entity entity = scene.CreateEntity("Normalization");
    CheckQuaternionComponents(entity.GetLocalRotation(), kIdentity, "Scene rotation default");

    const glm::quat expected = glm::quat(glm::vec3{0.3F, -0.7F, 0.5F});
    // 성분 제곱합을 float로 바로 구하면 큰 값은 overflow, 작은 값은 underflow할 수 있다.
    // 그런 입력도 원래 방향을 유지한 길이 1의 quaternion으로 정규화되어야 한다.
    for (const float magnitude : {7.0F, 1.0e30F, 1.0e-30F})
    {
        const glm::quat input = expected * magnitude;
        CheckQuaternion(Rotation{input}, expected, "Rotation normalizes finite input");
        entity.SetLocalRotation(input);
        CheckQuaternion(entity.GetLocalRotation(), expected, "setter normalizes finite input");
    }
    const glm::quat tinyIdentity{std::numeric_limits<float>::denorm_min(), 0.0F, 0.0F, 0.0F};
    CheckQuaternion(Rotation{tinyIdentity}, kIdentity, "smallest nonzero input");

    TransformSystemModule::UpdateWorldTransforms(scene.GetWorld());
    const glm::quat before = entity.GetLocalRotation();
    const glm::mat4 worldBefore = entity.GetWorldMatrix();
    for (const glm::quat& invalid : InvalidRotations())
    {
        CheckThrows<std::invalid_argument>([&] { (void)Rotation{invalid}; }, "invalid Rotation rejected");
        CheckThrows<std::invalid_argument>([&] { entity.SetLocalRotation(invalid); }, "invalid setter rejected");
        CheckQuaternionComponents(entity.GetLocalRotation(), before, "rejected setter preserves Local state");
        CheckMatrix(entity.GetWorldMatrix(), worldBefore, "rejected setter preserves World cache");
        TransformSystemModule::UpdateWorldTransforms(scene.GetWorld());
        CheckMatrix(entity.GetWorldMatrix(), worldBefore, "rejected setter preserves subsequent World pose");
    }
    entity.Destroy();
    CheckQuaternionComponents(entity.GetLocalRotation(), kIdentity, "destroyed Entity rotation");
}

void CheckComposition()
{
    const glm::vec3 position{0.8F, -1.2F, 0.4F};
    const glm::vec3 scale{1.5F, 0.7F, 2.0F};
    const float halfPi = std::acos(-1.0F) * 0.5F;
    // Euler 각은 pitch가 특정 자세에 가까우면 축 표현이 불안정해질 수 있다. 그 양쪽 자세에서도 quaternion이 나타내는 회전을 행렬로 바꿀 때 방향이 유지되는지 확인한다.
    for (const float pitch : {halfPi - 1.0e-5F, halfPi, halfPi + 1.0e-5F})
    {
        const glm::quat rotation = glm::quat(glm::vec3{0.4F, pitch, -0.8F});
        const glm::mat4 expected = ReferenceMatrix(position, rotation, scale);
        CheckMatrix(TransformSystemModule::ComposeLocalMatrix(position, rotation, scale), expected,
            "near singular pitch preserves matrix");
        CheckMatrix(TransformSystemModule::ComposeLocalMatrix(position, -rotation, scale), expected,
            "opposite quaternion sign preserves matrix");
        CheckMatrix(TransformSystemModule::ComposeLocalMatrix(position, rotation * 4.0F, scale), expected,
            "nonunit composition preserves matrix");
    }
    for (const glm::quat& invalid : InvalidRotations())
        CheckThrows<std::invalid_argument>(
            [&] { (void)TransformSystemModule::ComposeLocalMatrix(position, invalid, scale); },
            "invalid composition rejected");
}

void CheckHierarchy(Scene& scene)
{
    Entity root = scene.CreateEntity("HierarchyRoot");
    Entity child = scene.CreateEntity("HierarchyChild");
    Entity grandchild = scene.CreateEntity("HierarchyGrandchild");
    Entity grouping{scene.GetWorld().entity("Grouping").child_of(root.GetHandle())};
    child.SetParent(grouping);
    grandchild.SetParent(child);

    const glm::vec3 rootPosition{0.2F, -0.3F, 0.7F};
    const glm::vec3 childPosition{0.8F, 0.1F, -0.4F};
    const glm::vec3 grandchildPosition{-0.2F, 0.4F, 0.1F};
    const glm::quat childRotation = glm::quat(glm::vec3{-0.6F, 0.2F, 0.3F});
    const glm::quat grandchildRotation = glm::quat(glm::vec3{0.1F, -0.5F, 0.8F});
    root.SetLocalPosition(rootPosition);
    child.SetLocalPosition(childPosition);
    child.SetLocalRotation(childRotation);
    grandchild.SetLocalPosition(grandchildPosition);
    grandchild.SetLocalRotation(grandchildRotation);
    const float halfPi = std::acos(-1.0F) * 0.5F;
    glm::mat4 previousWorld = grandchild.GetWorldMatrix();
    for (const float pitch : {halfPi - 1.0e-5F, halfPi, halfPi + 1.0e-5F})
    {
        const glm::quat rootRotation = glm::quat(glm::vec3{0.3F, pitch, -0.4F});
        root.SetLocalRotation(rootRotation);
        CheckMatrix(grandchild.GetWorldMatrix(), previousWorld, "setter waits for World update");
        TransformSystemModule::UpdateWorldTransforms(scene.GetWorld());
        const glm::mat4 rootExpected = ReferenceMatrix(rootPosition, rootRotation, glm::vec3{1.0F});
        const glm::mat4 childExpected = rootExpected * ReferenceMatrix(childPosition, childRotation, glm::vec3{1.0F});
        const glm::mat4 grandchildExpected = childExpected *
            ReferenceMatrix(grandchildPosition, grandchildRotation, glm::vec3{1.0F});
        CheckMatrix(root.GetWorldMatrix(), rootExpected, "root World rotation");
        CheckMatrix(grouping.GetWorldMatrix(), rootExpected, "grouping propagates parent World");
        CheckMatrix(child.GetWorldMatrix(), childExpected, "child World pose");
        CheckMatrix(grandchild.GetWorldMatrix(), grandchildExpected, "grandchild World pose");
        previousWorld = grandchildExpected;
    }
}

void CheckCorruptedComponent()
{
    flecs::world world;
    Entity entity{world.entity()
        .set<Position, Local>(Position{})
        .set<Rotation, Local>(Rotation{})
        .set<Scale, Local>(Scale{})
        .set<TransformMatrix, Local>(TransformMatrix{})
        .set<TransformMatrix, World>(TransformMatrix{})};
    const glm::quat valid = glm::quat(glm::vec3{0.2F, 0.4F, -0.6F});
    entity.SetLocalRotation(valid);
    TransformSystemModule::UpdateWorldTransforms(world);
    const glm::mat4 worldBefore = entity.GetWorldMatrix();
    const glm::mat4 localBefore = entity.GetHandle().get<TransformMatrix, Local>();
    for (const glm::quat& invalid : InvalidRotations())
    {
        // 수정 가능한 ECS Component에 직접 쓰면 Rotation 생성자의 입력 검사를 거치지 않는다.
        // 행렬 합성 단계가 잘못된 값을 거부해 World 행렬에 NaN이 저장되지 않는지 확인한다.
        Rotation& stored = entity.GetHandle().get_mut<Rotation, Local>();
        stored.w = invalid.w;
        stored.x = invalid.x;
        stored.y = invalid.y;
        stored.z = invalid.z;
        CheckThrows<std::invalid_argument>([&] { TransformSystemModule::UpdateWorldTransforms(world); },
            "corrupted ECS rotation rejected before matrix generation");
        CheckMatrix(entity.GetWorldMatrix(), worldBefore, "corrupted rotation preserves World cache");
        CheckMatrix(entity.GetHandle().get<TransformMatrix, Local>(), localBefore,
            "corrupted rotation preserves Local matrix cache");
        entity.SetLocalRotation(valid);
    }
}
}

/**
 * @brief quaternion Local 회전의 저장 계약과 계층 World 행렬 전파를 확인한다.
 * @details 정규화, 잘못된 입력 거부, q와 -q의 방향 동등성 및 pitch 90도 부근의 자세 보존을 검사한다.
 * Scene은 World보다 먼저 파괴돼 root 아래의 물체를 정리하며, 이 테스트는 OpenGL context를 만들지 않는다.
 */
TEST(QuaternionTransform, DefaultsAndNormalization)
{
    flecs::world world;
    world.import<TransformSystemModule>();
    Scene scene(world);
    Check(scene.GetSceneRoot().is_alive(), "Scene owns a live hierarchy root");
    CheckDefaultsAndNormalization(scene);
}

TEST(QuaternionTransform, Composition)
{
    CheckComposition();
}

TEST(QuaternionTransform, HierarchyPropagation)
{
    flecs::world world;
    world.import<TransformSystemModule>();
    Scene scene(world);
    CheckHierarchy(scene);
}

TEST(QuaternionTransform, CorruptedComponentIsRejected)
{
    CheckCorruptedComponent();
}
