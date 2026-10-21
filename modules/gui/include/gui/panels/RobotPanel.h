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
 * @brief 로봇과 임무 상태를 표시하고 이번 프레임에 누른 버튼을 반환한다.
 * @details 임무 진행과 로봇 명령은 호출한 쪽에서 처리한다. 이 함수는 상태를 보관하지 않으므로 호출 사이에 유지할 객체가 필요 없다.
 */
[[nodiscard]] RobotPanelActions DrawRobotPanelContents(const RobotPanelView& view);

}
