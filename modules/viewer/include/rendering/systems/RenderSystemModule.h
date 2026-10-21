#pragma once

#include <flecs.h>

namespace grasplink::rendering
{

/**
 * @brief 장면 물체의 모양·표면 설정과 위치를 읽어 Renderer가 화면에 그리도록 요청한다.
 * @details 각 물체의 World 행렬은 Scene 전체 위치와 방향이고 MeshFilter는 그릴 꼭짓점·삼각형 자료, MeshRenderer는 색과 표시 여부를 제공한다.
 * 이 모듈은 표시 대상과 값을 골라 빛에 가리는 물체를 먼저 기록하고 카메라에서 보이는 색을 그리도록 전달한다.
 * 화면 buffer와 GPU 프로그램·Texture 상태를 바꾸고 실제 삼각형 명령을 내리는 일은 Renderer가 맡는다.
 */
struct RenderSystemModule
{
public:
    /**
     * @brief Flecs World에 렌더링 대상을 모아 Renderer로 전달하는 System을 등록한다.
     * @param world Entity와 렌더링 Component가 저장된 Flecs World.
     */
    explicit RenderSystemModule(flecs::world& world);
};

} // namespace grasplink::rendering
