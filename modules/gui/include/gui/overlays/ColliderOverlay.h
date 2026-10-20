#pragma once

#include <flecs.h>

#include <memory>

namespace grasplink::graphics { class Camera; }
struct ImVec2;

namespace grasplink::gui
{

/**
 * @brief 물리 계산에 쓰는 단순 물체의 모양을 화면 위 선으로 보여준다.
 * @details 장면 물체에 붙인 Collider 설정은 접촉 판정에 쓸 모양과 크기를 지정한다. Flecs World가 장면 물체와 이 설정을 저장하고 ColliderOverlay는 Box와 Convex Hull을 Scene 위치에 놓인 화면 선으로 바꾼다.
 * 화면 선은 설정된 충돌 모양을 확인하기 위한 근사 표시이며, Jolt가 실제 접촉 계산에 사용하는 세부 형상과 다를 수 있다.
 * Flecs World에서 물체를 조회하므로 이 객체보다 World가 오래 살아야 한다. GUI 상태나 OpenGL 자원은 소유하지 않고 호출자가 시작한 ImGui 프레임에 선을 추가한다.
 */
class ColliderOverlay
{
public:
    /**
     * @brief 물체의 충돌 모양 설정과 Scene 위치를 읽을 World에 연결한다.
     * @param world 물체와 Collider 설정 및 World 행렬을 저장한다. 이 참조를 사용하는 Overlay보다 오래 살아야 한다.
     */
    explicit ColliderOverlay(flecs::world& world);
    ~ColliderOverlay();

    ColliderOverlay(const ColliderOverlay&) = delete;
    ColliderOverlay& operator=(const ColliderOverlay&) = delete;

    /**
     * @brief 활성 ImGui 프레임에 충돌 모양을 화면 선으로 그린다.
     * @param camera Scene 위치를 화면 픽셀로 바꾸는 데 쓰며 호출이 끝나면 보관하지 않는 Camera 참조다.
     * @param visible false이면 저장한 선을 지운다. 다시 켤 때 새 Scene의 물체에서 선을 계산한다.
     * @details 카메라는 원근 투영으로 앞뒤 거리를 화면 크기에 반영한다. 카메라 뒤쪽의 점은 화면 앞에 있는 것처럼 잘못 나타날 수 있어 선을 만들 때 건너뛴다.
     * 화면 선 좌표는 매번 다시 구하는 비용을 줄이려고 최대 50 ms 재사용한다. 그동안 카메라를 움직이면 선이 잠시 이전 위치에 보일 수 있다.
     * 선은 화면 맨 앞에 그리므로 벽이나 로봇에 가려진 충돌 모양도 확인할 수 있다. OpenGL 명령은 직접 실행하지 않는다.
     */
    void Draw(const grasplink::graphics::Camera& camera, bool visible);

    /** @brief 화면의 일부만 Scene viewport로 쓸 때 해당 영역 크기에 맞춰 충돌선을 그린다. */
    void Draw(const grasplink::graphics::Camera& camera, bool visible, const ImVec2& viewportSize);

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
