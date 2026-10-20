#include "gui/panels/PhysicsDebugPanel.h"

#include <imgui.h>

namespace grasplink::gui
{

void PhysicsDebugPanel::Draw()
{
    ImGui::Begin("Configured colliders");
    DrawContents();
    ImGui::End();
}

void PhysicsDebugPanel::DrawContents()
{
    // 안내 문구는 이 선이 Jolt Body에서 직접 읽은 실제 충돌 형상이 아니라 ECS에 저장된 설정을 그린 근사라는 점을 알린다.
    ImGui::SeparatorText("Physics view");
    ImGui::Checkbox("Show configured colliders", &visible_);
    ImGui::TextUnformatted("ECS approximation; refresh: 50 ms");
    ImGui::TextUnformatted("X-ray: Robot orange, Gripper purple, Dynamic cyan, Floor green");
}

bool PhysicsDebugPanel::IsColliderVisible() const
{
    return visible_;
}

}
