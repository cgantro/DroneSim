#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <cstdint>
#include <optional>
#include <string_view>

namespace grasplink::gui
{
struct RobotPanelMissionView
{
    std::string_view stageLabel;
    std::string_view lastMessage;
    std::uint64_t completedCount = 0;
    double maxLinearVelocityMetersPerSecond = 0.0;
    double maxAngularVelocityRadiansPerSecond = 0.0;
    bool paused = false;
    bool missionSucceeded = false;
    bool autoRepeat = false;
    bool hasResult = false;
    bool lastRequestAccepted = true;
    bool canStart = false;
};

struct RobotPanelView
{
    const robotics::RobotState& state;
    const robotics::models::RobotSpecification& specification;
    const robotics::CartesianPose& graspBoxPoseInBase;
    const robotics::CartesianPose& placementPoseInBase;
    std::optional<RobotPanelMissionView> mission;
};

struct RobotPanelActions
{
    bool start = false;
    bool resume = false;
    bool stop = false;
};
/**
 * @brief 로봇 상태와 임무 정보를 표시하고 버튼 입력을 반환한다.
 * @details 임무 진행과 로봇 명령은 ViewerApp이 처리한다. 패널은 전달받은 값을 표시하고 사용자 입력을 RobotPanelActions로 돌려준다.
 */
class RobotPanel final
{
public:
    /** @brief 현재 로봇과 임무 상태를 표시하고 이번 프레임의 버튼 입력을 반환한다. */
    [[nodiscard]] RobotPanelActions DrawContents(const RobotPanelView& view);

};

}
