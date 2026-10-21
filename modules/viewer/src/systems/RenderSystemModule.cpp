#include "rendering/systems/RenderSystemModule.h"

#include "rendering/components/RenderComponents.h"
#include "scene/TransformComponents.h"
#include "graphics/Camera.h"
#include "graphics/Renderer.h"
#include "rendering/RenderContext.h"

namespace grasplink::rendering
{

RenderSystemModule::RenderSystemModule(flecs::world& world)
{
    world.module<RenderSystemModule>();
    // Flecs의 PreStore 단계에서 Scene OnUpdate 이후에 그린다. 변환 계산은 자동 단계가 아니므로 ViewerApp이 World 진행 전에 명시적으로 끝낸다.
    world
        .system<const MeshFilter, const MeshRenderer, const ::grasplink::scene::TransformMatrix>(
            "RenderSystem")
        .kind(flecs::PreStore)
        .term_at(2)
        .second<::grasplink::scene::World>()
        .run(
            [](flecs::iter& it)
            {
                // system 등록 때 받은 임시 World wrapper를 저장하지 않는다. 실제 실행 중인 Flecs World에서 RenderContext를 빌려 읽는다.
                const RenderContext* context =
                    it.world().try_get<RenderContext>();

                if (!context || !context->renderer || !context->camera)
                    return;

                const glm::mat4 view = context->camera->GetViewMatrix();
                const glm::mat4 projection = context->camera->GetProjectionMatrix();
                const glm::vec3 cameraPosition = context->camera->GetPosition();

                while (it.next())
                {
                    auto meshFilters = it.field<const MeshFilter>(0);
                    auto meshRenderers = it.field<const MeshRenderer>(1);
                    auto worldMatrices = it.field<const ::grasplink::scene::TransformMatrix>(2);

                    for (auto i : it)
                    {
                        if (!meshRenderers[i].visible) continue;

                        // Renderer는 ECS를 바꾸지 않으므로 Component를 읽는 동안 World 변환으로 바로 그린다.
                        context->renderer->Draw(
                            static_cast<const glm::mat4&>(worldMatrices[i]),
                            meshFilters[i],
                            meshRenderers[i],
                            view,
                            projection,
                            cameraPosition);
                    }
                }
            });
}

} // namespace grasplink::rendering
