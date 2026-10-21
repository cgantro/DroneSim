#include "gui/panels/RobotPanel.h"

#include <glm/gtc/quaternion.hpp>
#include <imgui.h>

#include <cmath>
#include <string_view>

namespace grasplink::gui
{
namespace
{
const char* RobotModeName(robotics::RobotMode mode)
{
    switch (mode)
    {
    case robotics::RobotMode::Disconnected: return "Disconnected";
    case robotics::RobotMode::Idle: return "Idle";
    case robotics::RobotMode::Moving: return "Moving";
    case robotics::RobotMode::Planning: return "Planning";
    case robotics::RobotMode::Stopped: return "Stopped";
    case robotics::RobotMode::Fault: return "Fault";
    }
    return "Unknown";
}

double YawDegrees(const std::array<double, 4>& xyzw)
{
    const glm::dquat rotation = glm::normalize(glm::dquat{xyzw[3], xyzw[0], xyzw[1], xyzw[2]});
    return glm::degrees(std::atan2(2.0 * (rotation.w * rotation.y + rotation.x * rotation.z),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)));
}
}

RobotPanelActions DrawRobotPanelContents(const RobotPanelView& view)
{
    RobotPanelActions actions;
    const auto& state = view.state;
    const auto& graspBoxPoseInBase = view.graspBoxPoseInBase;
    const auto& placementPoseInBase = view.placementPoseInBase;
    if (view.mission)
    {
        const RobotPanelMissionView& mission = *view.mission;
        ImGui::SeparatorText("Pick and place");
        if (state.valid && state.tcpPoseValid)
            ImGui::Text("TCP: %.3f, %.3f, %.3f m", state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]);
        ImGui::Text("Box X/Z: %.3f, %.3f m", graspBoxPoseInBase.positionMeters[0], graspBoxPoseInBase.positionMeters[2]);
        ImGui::Text("Box yaw: %.1f deg", YawDegrees(graspBoxPoseInBase.orientationXyzw));
        ImGui::Text("Goal X/Z: %.3f, %.3f m", placementPoseInBase.positionMeters[0], placementPoseInBase.positionMeters[2]);
        ImGui::Text("Goal yaw: %.1f deg", YawDegrees(placementPoseInBase.orientationXyzw));
        ImGui::Text("TCP ceilings: %.1f m/s, %.1f rad/s", mission.maxLinearVelocityMetersPerSecond, mission.maxAngularVelocityRadiansPerSecond);
    }
    const auto& specification = view.specification;
    if (ImGui::BeginTable("Joint positions", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
    {
        ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_WidthFixed, 42.0F);
        ImGui::TableSetupColumn("Min / max", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Current", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            const std::string_view jointName = specification.joints[joint].name;
            ImGui::TextUnformatted(jointName.data(), jointName.data() + jointName.size());
            ImGui::TableSetColumnIndex(1);
            const auto& jointSpec = specification.joints[joint];
            if (jointSpec.motionType == robotics::models::JointMotionType::Prismatic)
                ImGui::Text("%.3f / %.3f m", jointSpec.minimumPosition, jointSpec.maximumPosition);
            else
                ImGui::Text("%.1f / %.1f deg",
                    glm::degrees(jointSpec.minimumPosition), glm::degrees(jointSpec.maximumPosition));
            ImGui::TableSetColumnIndex(2);
            if (joint >= state.jointPositions.size())
            {
                ImGui::TextUnformatted("--");
                continue;
            }

            if (jointSpec.motionType == robotics::models::JointMotionType::Prismatic)
                ImGui::Text("%.3f m", state.jointPositions[joint]);
            else
                ImGui::Text("%.1f deg", glm::degrees(state.jointPositions[joint]));
        }
        ImGui::EndTable();
    }
    ImGui::TextUnformatted("Joint values show angles or travel distance, not percentages.");
    if (view.mission)
    {
        const RobotPanelMissionView& mission = *view.mission;
        if (mission.canStart && ImGui::Button("Run random pick and place"))
            actions.start = true;
        if (mission.paused)
        {
            ImGui::SameLine();
            if (ImGui::Button("Resume task"))
                actions.resume = true;
        }
        else
        {
            ImGui::SameLine();
            if (ImGui::Button("Stop robot"))
                actions.stop = true;
        }
        ImGui::Text("Task: %s", mission.paused ? "Paused" : mission.stageLabel.data());
        if (mission.paused)
            ImGui::Text("Paused at: %s", mission.stageLabel.data());
        ImGui::Text("Mission success: %s", mission.missionSucceeded ? "YES" : "NO");
        ImGui::Text("Successful missions: %llu", static_cast<unsigned long long>(mission.completedCount));
        ImGui::Text("Automatic repeat: %s", mission.autoRepeat ? "ON" : "OFF");
        if (mission.hasResult)
        {
            ImGui::Text("Last command: %s", mission.lastRequestAccepted ? "accepted" : "failed");
            ImGui::TextWrapped("Accepted means the controller received the command; the mode above shows whether it is still moving.");
            if (!mission.lastMessage.empty())
                ImGui::TextWrapped("%.*s", static_cast<int>(mission.lastMessage.size()), mission.lastMessage.data());
        }
    }
    ImGui::Text("Controller: %s", RobotModeName(state.mode));
    std::size_t fastestJoint = 0;
    double maximumJointSpeed = 0.0;
    for (std::size_t joint = 0; joint < state.jointVelocities.size() &&
        joint < specification.jointCount; ++joint)
    {
        const double speed = std::abs(state.jointVelocities[joint]);
        if (speed > maximumJointSpeed)
        {
            fastestJoint = joint;
            maximumJointSpeed = speed;
        }
    }
    if (specification.jointCount > 0 && !state.jointVelocities.empty())
    {
        const auto& jointName = specification.joints[fastestJoint].name;
        if (specification.joints[fastestJoint].motionType == robotics::models::JointMotionType::Prismatic)
            ImGui::Text("Joint speed: %.*s %.3f m/s",
                static_cast<int>(jointName.size()), jointName.data(), maximumJointSpeed);
        else
            ImGui::Text("Joint speed: %.*s %.1f deg/s",
                static_cast<int>(jointName.size()), jointName.data(), glm::degrees(maximumJointSpeed));
    }
    if (state.errorCode == robotics::ErrorCode::EnvironmentContact ||
        state.errorCode == robotics::ErrorCode::SelfCollision ||
        state.errorCode == robotics::ErrorCode::AttachedObjectCollision)
        ImGui::TextWrapped("A collision stopped the robot at its last safe pose.");
    return actions;
}
}
