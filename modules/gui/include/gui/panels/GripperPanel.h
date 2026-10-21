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
    /** @brief 이미 열린 ImGui 창 안에 그리퍼 조작과 상태 내용을 그린다. */
    void DrawContents(robotics::IGripperController& gripper,
        const simulation::GripperGraspState* graspState = nullptr, bool commandsEnabled = true);

private:
    bool hasGripperResult = false;
    robotics::Result lastGripperResult{};
};

}
