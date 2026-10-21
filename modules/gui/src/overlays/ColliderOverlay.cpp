#include "gui/overlays/ColliderOverlay.h"

#include "graphics/Camera.h"
#include "simulation/components/PhysicsComponents.h"
#include "scene/TransformComponents.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <imgui.h>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <array>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace grasplink::gui
{
namespace
{
/** @brief Scene 좌표의 Collider 형상을 화면에 그릴 두 픽셀 점과 충돌 그룹 색을 저장한다. */
struct ScreenLine
{
    ImVec2 from;
    ImVec2 to;
    ImU32 color = IM_COL32_WHITE;
};

/** @brief Body 원점에서 측정한 설정 Shape의 위치와 회전을 행렬로 바꾼다. 형상의 크기는 별도의 Shape 치수 [m]에서 읽는다. */
glm::mat4 LocalShapeMatrix(const grasplink::physics::Transform& transform)
{
    return glm::translate(glm::mat4{1.0F}, transform.position) *
        glm::mat4_cast(glm::normalize(transform.rotation));
}

/** @brief Scene 좌표의 점을 ImGui 화면 픽셀로 옮기고 카메라 뒤에 있는 점은 그릴 수 없어 제외한다. */
bool Project(const glm::vec3& position, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImVec2& result)
{
    const glm::vec4 clip = viewProjection * glm::vec4{position, 1.0F};
    // clip.w가 0에 가깝거나 음수면 화면 좌표로 안정적으로 바꿀 수 없으므로 제외한다. 가까운 평면(near plane)을 가로지르는 선 자체는 잘라내지 않는다.
    // 따라서 이 경계 주변의 선 일부는 생략되거나 화면 바깥으로 길게 늘어날 수 있다.
    if (clip.w <= 0.001F)
        return false;

    // 원근 나눗셈으로 clip 좌표를 정규화 장치 좌표(NDC, 축별 -1..1)로 바꾼다. ImGui 화면은 y가 아래로 증가하므로 y축 방향을 뒤집어 픽셀 좌표로 변환한다.
    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    result = {((ndc.x + 1.0F) * 0.5F) * displaySize.x,
        ((1.0F - ndc.y) * 0.5F) * displaySize.y};
    return true;
}

/** @brief Shape 자체 좌표로 주어진 선의 양 끝점을 Scene 좌표와 화면 픽셀로 변환해 캐시에 추가한다. */
void DrawLine(std::vector<ScreenLine>& lines, const glm::vec3& from, const glm::vec3& to,
    const glm::mat4& shapeWorld, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImU32 color)
{
    ImVec2 screenFrom;
    ImVec2 screenTo;
    const glm::vec3 worldFrom = glm::vec3{shapeWorld * glm::vec4{from, 1.0F}};
    const glm::vec3 worldTo = glm::vec3{shapeWorld * glm::vec4{to, 1.0F}};
    if (Project(worldFrom, viewProjection, displaySize, screenFrom) &&
        Project(worldTo, viewProjection, displaySize, screenTo))
        lines.push_back({screenFrom, screenTo, color});
}

/** @brief ECS에 설정한 Box의 여덟 꼭짓점을 만들고 열두 모서리를 화면 선으로 그린다. */
void DrawBox(std::vector<ScreenLine>& lines, const grasplink::physics::CollisionShapeDescription& shape,
    const glm::mat4& entityWorld, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImU32 color)
{
    // 중심에서 각 축으로 뻗는 반쪽 크기로 여덟 꼭짓점을 만든다. 각 모서리는 Entity와 Shape 변환, 카메라 투영을 거쳐 화면 선이 된다.
    const glm::mat4 world = entityWorld * LocalShapeMatrix(shape.localTransform);
    std::array<glm::vec3, 8> corners;
    for (unsigned int i = 0; i < corners.size(); ++i)
        corners[i] = {
            (i & 1U) ? shape.halfExtentsMeters.x : -shape.halfExtentsMeters.x,
            (i & 2U) ? shape.halfExtentsMeters.y : -shape.halfExtentsMeters.y,
            (i & 4U) ? shape.halfExtentsMeters.z : -shape.halfExtentsMeters.z};

    for (unsigned int i = 0; i < corners.size(); ++i)
        for (unsigned int axis = 0; axis < 3; ++axis)
            if ((i & (1U << axis)) == 0)
                DrawLine(lines, corners[i], corners[i | (1U << axis)], world,
                    viewProjection, displaySize, color);
}

/** @brief Jolt가 구성한 3D 볼록 껍질의 실제 모서리를 저장한다. */
struct HullWireframe
{
    std::vector<glm::vec3> sourcePoints;
    std::vector<std::pair<glm::vec3, glm::vec3>> edges;
};

bool HasSamePoints(const HullWireframe& wireframe,
    const grasplink::physics::CollisionShapeDescription& shape)
{
    if (wireframe.sourcePoints.size() != shape.pointsMeters.size())
        return false;
    return std::equal(wireframe.sourcePoints.begin(), wireframe.sourcePoints.end(),
        shape.pointsMeters.begin(), [](const glm::vec3& left, const glm::vec3& right)
        {
            return left.x == right.x && left.y == right.y && left.z == right.z;
        });
}

HullWireframe BuildHullWireframe(const grasplink::physics::CollisionShapeDescription& shape)
{
    HullWireframe wireframe;
    wireframe.sourcePoints = shape.pointsMeters;
    if (shape.pointsMeters.size() < 4)
        return wireframe;

    JPH::Array<JPH::Vec3> points;
    points.reserve(shape.pointsMeters.size());
    for (const glm::vec3& point : shape.pointsMeters)
        points.emplace_back(point.x, point.y, point.z);
    const JPH::ConvexHullShapeSettings settings(points, 0.0F);
    const auto result = settings.Create();
    if (result.HasError())
        return wireframe;

    const auto* hull = static_cast<const JPH::ConvexHullShape*>(result.Get().GetPtr());
    const JPH::Vec3 centerOfMass = hull->GetCenterOfMass();
    std::unordered_set<std::uint64_t> seenEdges;
    std::vector<JPH::uint> faceVertices(hull->GetNumPoints());
    for (JPH::uint face = 0; face < hull->GetNumFaces(); ++face)
    {
        const JPH::uint vertexCount = hull->GetFaceVertices(face,
            static_cast<JPH::uint>(faceVertices.size()), faceVertices.data());
        for (JPH::uint index = 0; index < vertexCount; ++index)
        {
            const JPH::uint first = faceVertices[index];
            const JPH::uint second = faceVertices[(index + 1) % vertexCount];
            const auto low = std::min(first, second);
            const auto high = std::max(first, second);
            const std::uint64_t key = (static_cast<std::uint64_t>(low) << 32U) | high;
            if (!seenEdges.insert(key).second)
                continue;

            const JPH::Vec3 firstPoint = hull->GetPoint(first) + centerOfMass;
            const JPH::Vec3 secondPoint = hull->GetPoint(second) + centerOfMass;
            wireframe.edges.emplace_back(
                glm::vec3{firstPoint.GetX(), firstPoint.GetY(), firstPoint.GetZ()},
                glm::vec3{secondPoint.GetX(), secondPoint.GetY(), secondPoint.GetZ()});
        }
    }
    return wireframe;
}

void DrawConvexHull(std::vector<ScreenLine>& lines, const grasplink::physics::CollisionShapeDescription& shape,
    const HullWireframe& wireframe, const glm::mat4& entityWorld,
    const glm::mat4& viewProjection, const ImVec2& displaySize, ImU32 color)
{
    const glm::mat4 world = entityWorld * LocalShapeMatrix(shape.localTransform);
    for (const auto& [from, to] : wireframe.edges)
        DrawLine(lines, from, to, world, viewProjection, displaySize, color);
}
ImU32 LayerColor(grasplink::physics::CollisionLayer layer)
{
    switch (layer)
    {
    case grasplink::physics::CollisionLayer::Environment: return IM_COL32(70, 230, 100, 255);
    case grasplink::physics::CollisionLayer::Robot: return IM_COL32(255, 110, 55, 255);
    case grasplink::physics::CollisionLayer::Gripper: return IM_COL32(210, 90, 255, 255);
    case grasplink::physics::CollisionLayer::DynamicObject: return IM_COL32(70, 210, 255, 255);
    case grasplink::physics::CollisionLayer::Count: return IM_COL32_WHITE;
    }
    return IM_COL32_WHITE;
}

/** @brief Entity의 누적 World 행렬에서 물리 Body 원점의 Scene 위치와 회전만 추출한다. */
glm::mat4 PhysicsTransform(const glm::mat4& world)
{
    // 물리 Body 변환에는 위치와 회전만 반영한다. Collider 치수는 이미 [m]로 설정되어 있어 Entity 계층의 크기 배율은 적용하지 않는다.
    glm::vec3 scale{1.0F};
    glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 position{0.0F};
    glm::vec3 skew{0.0F};
    glm::vec4 perspective{0.0F};
    if (!glm::decompose(world, scale, rotation, position, skew, perspective))
        return glm::mat4{1.0F};
    return glm::translate(glm::mat4{1.0F}, position) * glm::mat4_cast(glm::normalize(rotation));
}
}

struct ColliderOverlay::Impl
{
    // ECS에 지정된 Collider 형상과 최신 World 행렬을 읽어 선을 만든다. Jolt가 내부에서 최적화하거나 바꾼 실제 충돌 형상은 읽지 않는다.
    flecs::query<const RigidBody, const Colliders, const grasplink::scene::TransformMatrix> colliderQuery;
    std::vector<ScreenLine> collisionLines;
    std::unordered_map<const grasplink::physics::CollisionShapeDescription*, HullWireframe> hullWireframes;
    std::chrono::steady_clock::time_point nextCollisionRefresh{};
    ImVec2 cachedDisplaySize{};
    bool wasVisible = false;

    explicit Impl(flecs::world& world)
        : colliderQuery(world.query_builder<const RigidBody, const Colliders, const grasplink::scene::TransformMatrix>()
            .term_at(2).second<grasplink::scene::World>().build())
    {
    }

    void Draw(const grasplink::graphics::Camera& camera, bool visible, const ImVec2& viewportSize)
    {
        if (visible)
        {
            const auto now = std::chrono::steady_clock::now();
            const bool resized = viewportSize.x != cachedDisplaySize.x || viewportSize.y != cachedDisplaySize.y;
            if (!wasVisible || resized || now >= nextCollisionRefresh)
            {
                RefreshCollisionLines(camera, viewportSize);
                cachedDisplaySize = viewportSize;
                nextCollisionRefresh = now + std::chrono::milliseconds(50);
            }
            // 픽셀 좌표를 캐시하므로 카메라가 움직여도 다음 갱신 전까지는 이전 위치의 선을 표시한다.
            // 전경 draw list는 깊이 버퍼를 확인하지 않으므로 다른 물체 뒤에 가려진 Collider 선도 화면에 나타난다.
            ImDrawList* drawList = ImGui::GetForegroundDrawList();
            for (const ScreenLine& line : collisionLines)
                drawList->AddLine(line.from, line.to, line.color, 1.5F);
        }
        else
        {
            collisionLines.clear();
        }
        wasVisible = visible;
    }

    void RefreshCollisionLines(const grasplink::graphics::Camera& camera, const ImVec2& displaySize)
    {
        collisionLines.clear();
        std::unordered_set<const grasplink::physics::CollisionShapeDescription*> visibleHulls;
        // View와 Projection 행렬을 차례로 적용해 Scene 좌표를 화면 투영 전 좌표로 옮긴다.
        // PhysicsTransform에서 제외한 Entity 크기 배율은 이 선에도 적용하지 않는다.
        const glm::mat4 viewProjection = camera.GetProjectionMatrix() * camera.GetViewMatrix();
        colliderQuery.each([&](flecs::entity, const RigidBody& rigidBody,
            const Colliders& colliders, const grasplink::scene::TransformMatrix& matrix)
        {
            const glm::mat4 entityWorld = PhysicsTransform(static_cast<const glm::mat4&>(matrix));
            const ImU32 color = LayerColor(rigidBody.collisionLayer);
            for (const auto& shape : colliders.shapes)
            {
                switch (shape.type)
                {
                case grasplink::physics::CollisionShapeType::Box:
                    DrawBox(collisionLines, shape, entityWorld, viewProjection, displaySize, color);
                    break;
                case grasplink::physics::CollisionShapeType::ConvexHull:
                {
                    visibleHulls.insert(&shape);
                    auto [wireframe, inserted] = hullWireframes.try_emplace(&shape);
                    if (inserted || !HasSamePoints(wireframe->second, shape))
                        wireframe->second = BuildHullWireframe(shape);
                    DrawConvexHull(collisionLines, shape, wireframe->second,
                        entityWorld, viewProjection, displaySize, color);
                    break;
                }
                }
            }
        });
        for (auto wireframe = hullWireframes.begin(); wireframe != hullWireframes.end();)
        {
            if (visibleHulls.count(wireframe->first) == 0)
                wireframe = hullWireframes.erase(wireframe);
            else
                ++wireframe;
        }
    }
};

ColliderOverlay::ColliderOverlay(flecs::world& world)
    : m_Impl(std::make_unique<Impl>(world))
{
}

ColliderOverlay::~ColliderOverlay() = default;

void ColliderOverlay::Draw(const grasplink::graphics::Camera& camera, bool visible, const ImVec2& viewportSize)
{
    m_Impl->Draw(camera, visible, viewportSize);
}

}
