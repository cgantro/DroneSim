#pragma once

namespace grasplink::gui
{

/**
 * @brief ECS에 등록한 충돌 형상을 화면에 표시할지 선택하는 ImGui 패널이다.
 * @details 이 클래스는 체크박스 상태만 저장한다. ColliderOverlay가 World에 저장된 충돌 모양과 물체 위치를 읽어 화면 선 좌표로 바꾼다.
 * 패널은 이 선이 실제 접촉 계산 결과가 아니라 설정에서 만든 근사이며, 성능을 위해 좌표를 최대 50 ms 재사용한다고 안내한다.
 */
class PhysicsDebugPanel
{
public:
    /** @brief 현재 ImGui 프레임에 충돌 형상 표시 체크박스와 각 충돌 그룹 색의 범례를 그린다. */
    void Draw();

    /** @brief 이미 열린 ImGui 창 안에 충돌 표시 설정과 범례를 그린다. */
    void DrawContents();

    /** @brief 사용자가 체크박스에서 선택한 충돌 형상 표시 여부를 반환한다. */
    [[nodiscard]] bool IsColliderVisible() const;

private:
    bool visible_ = false;
};

}
