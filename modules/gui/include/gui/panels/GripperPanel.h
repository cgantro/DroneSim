#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics { class IGripperController; }
namespace grasplink::simulation { struct GripperGraspState; }

namespace grasplink::gui
{

/**
 * @brief 그리퍼를 열거나 닫고 Controller와 물리 계산이 보고한 상태를 보여준다.
 * @details Open과 Close는 그리퍼의 양 끝 위치를 요청하고 Stop은 진행 중인 이동을 멈춘다. 실제 손가락 이동과 접촉 상태는 Controller와 물리 계산이 정한다.
 * Controller는 Draw()가 실행되는 동안만 빌려 쓰며 패널이 소유하지 않는다.
 */
class GripperPanel
{
public:
    /**
     * @brief 현재 프레임의 사용자 입력을 Controller 명령으로 보내고 상태를 화면에 그린다.
     * @param gripper 이번 호출 중에만 사용하는 Controller 참조. 패널은 이 참조를 저장하지 않는다.
     * @details GuiModule::BeginFrame()과 EndFrame() 사이에 호출한다. Controller의 개폐 비율이 0에서 1 사이이면 백분율로 보여 준다.
     * graspState가 주어지면 실제 물리 접촉과 양쪽 손가락이 같은 물체를 잡은 상태를 구분해 표시한다. 개별 손가락의 접촉 적응과 실제 파지력은 계산하지 않는다.
     */
    void Draw(robotics::IGripperController& gripper, const simulation::GripperGraspState* graspState = nullptr);

    /** @brief 이미 열린 ImGui 창 안에 그리퍼 조작과 상태 내용을 그린다. */
    void DrawContents(robotics::IGripperController& gripper,
        const simulation::GripperGraspState* graspState = nullptr, bool commandsEnabled = true);

private:
    bool hasGripperResult = false;
    robotics::Result lastGripperResult{};
};

}
