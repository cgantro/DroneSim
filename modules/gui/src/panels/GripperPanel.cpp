#include "gui/panels/GripperPanel.h"

#include "robotics/core/IGripperController.h"
#include "simulation/robotics/GripperGraspAdapter.h"

#include <imgui.h>

#include <cstdint>
#include <utility>

namespace grasplink::gui
{
namespace
{
const char* GripperModeName(grasplink::robotics::GripperMode mode)
{
    using grasplink::robotics::GripperMode;
    switch (mode)
    {
    case GripperMode::Disconnected: return "Disconnected";
    case GripperMode::Inactive: return "Inactive";
    case GripperMode::Idle: return "Idle";
    case GripperMode::Moving: return "Moving";
    case GripperMode::Stopped: return "Stopped";
    case GripperMode::Fault: return "Fault";
    }
    return "Unknown";
}

const char* GripperObjectStatusName(grasplink::robotics::GripperObjectStatus status)
{
    using grasplink::robotics::GripperObjectStatus;
    switch (status)
    {
    case GripperObjectStatus::Moving: return "No contact / target not reached";
    case GripperObjectStatus::ContactWhileOpening: return "Contact while opening";
    case GripperObjectStatus::ContactWhileClosing: return "Contact while closing";
    case GripperObjectStatus::AtRequestedPosition: return "At requested position";
    }
    return "Unknown";
}

const char* ErrorCodeName(grasplink::robotics::ErrorCode code)
{
    using grasplink::robotics::ErrorCode;
    switch (code)
    {
    case ErrorCode::None: return "Success";
    case ErrorCode::NotConnected: return "Not connected";
    case ErrorCode::InvalidCommand: return "Invalid command";
    case ErrorCode::Busy: return "Busy";
    case ErrorCode::Cancelled: return "Cancelled";
    case ErrorCode::Fault: return "Fault";
    case ErrorCode::Unsupported: return "Unsupported";
    case ErrorCode::TransportError: return "Transport error";
    case ErrorCode::Unreachable: return "Unreachable pose";
    case ErrorCode::JointLimitReached: return "Joint limit reached";
    case ErrorCode::IkDidNotConverge: return "IK did not converge";
    case ErrorCode::EnvironmentContact: return "Robot stopped at environment collision";
    case ErrorCode::SelfCollision: return "Robot self collision";
    case ErrorCode::AttachedObjectCollision: return "Attached object collision";
    }
    return "Unknown";
}

}

void GripperPanel::Draw(grasplink::robotics::IGripperController& gripper, const grasplink::simulation::GripperGraspState* graspState)
{
    ImGui::Begin("Gripper control");
    DrawContents(gripper, graspState);
    ImGui::End();
}

void GripperPanel::DrawContents(grasplink::robotics::IGripperController& gripper,
    const grasplink::simulation::GripperGraspState* graspState, bool commandsEnabled)
{
    using grasplink::robotics::GripperCommand;
    ImGui::SeparatorText("Gripper");
    const auto rememberResult = [&](grasplink::robotics::Result result)
    {
        lastGripperResult = std::move(result);
        hasGripperResult = true;
    };
    const auto submitClosure = [&](std::uint8_t position)
    {
        GripperCommand command;
        command.positionRequest = position;
        rememberResult(gripper.Command(command));
    };
    ImGui::Text("Connection: %s", gripper.IsConnected() ? "Connected" : "Disconnected");

    ImGui::BeginDisabled(!commandsEnabled);
    if (ImGui::Button("Open"))
    {
        submitClosure(0);
    }
    ImGui::SameLine();
    if (ImGui::Button("Close"))
    {
        submitClosure(255);
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
    {
        rememberResult(gripper.Stop());
    }
    ImGui::EndDisabled();

    const grasplink::robotics::GripperState state = gripper.GetState();
    if (state.valid)
    {
        ImGui::Text("Mode: %s", GripperModeName(state.mode));
        ImGui::Text("Activated: %s", state.activated ? "Yes" : "No");
        ImGui::Text("Actual position (raw): %u", static_cast<unsigned int>(state.actualPosition));
        if (state.closureFractionValid && std::isfinite(state.closureFraction) &&
            state.closureFraction >= 0.0 && state.closureFraction <= 1.0)
            ImGui::Text("Actual closure: %.1f%%", state.closureFraction * 100.0);
        else
            ImGui::TextUnformatted("Actual closure: unavailable");
        ImGui::Text("Object status: %s", GripperObjectStatusName(state.objectStatus));
        if (state.currentValid)
            ImGui::Text("Current (raw): %u", static_cast<unsigned int>(state.currentRaw));
        else
            ImGui::TextUnformatted("Current: unavailable");
    }
    else
    {
        ImGui::TextUnformatted("Gripper state: unavailable");
        ImGui::TextUnformatted("Actual position: unavailable");
        ImGui::TextUnformatted("Actual closure: unavailable");
        ImGui::TextUnformatted("Object status: unavailable");
        ImGui::TextUnformatted("Current: unavailable");
    }

    if (hasGripperResult)
    {
        ImGui::Text("Last result: %s", ErrorCodeName(lastGripperResult.code));
        if (!lastGripperResult.message.empty())
            ImGui::TextWrapped("%s", lastGripperResult.message.c_str());
    }
    else
    {
        ImGui::TextUnformatted("Last result: no command sent");
    }
    if (graspState != nullptr)
    {
        ImGui::Text("Left contact: %s", graspState->leftContact ? "Yes" : "No");
        ImGui::Text("Right contact: %s", graspState->rightContact ? "Yes" : "No");
        ImGui::Text("Object held: %s", graspState->grasped ? "Yes" : "No");
        ImGui::TextWrapped("Open to release the held object. Holding uses a rigid attachment; grip force and individual finger adaptation are not simulated.");
    }
    else
        ImGui::TextUnformatted("Physical contact feedback: unavailable");
}

}
