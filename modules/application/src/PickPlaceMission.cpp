#include "application/PickPlaceMission.h"
#include "application/PickPlaceConfig.h"
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <string_view>
#include <utility>

namespace grasplink::application
{
namespace
{
namespace config = pick_place::config;

glm::dquat TcpOrientation(const robotics::RobotState& state)
{
    return {state.tcpPose.orientationXyzw[3], state.tcpPose.orientationXyzw[0],
        state.tcpPose.orientationXyzw[1], state.tcpPose.orientationXyzw[2]};
}

glm::dquat Orientation(const std::array<double, 4>& xyzw)
{
    return glm::normalize(glm::dquat{xyzw[3], xyzw[0], xyzw[1], xyzw[2]});
}

std::array<double, 4> QuaternionXyzw(const glm::dquat& orientation)
{
    const auto normalized = glm::normalize(orientation);
    return {normalized.x, normalized.y, normalized.z, normalized.w};
}

double YawDegrees(const std::array<double, 4>& xyzw)
{
    const glm::dquat rotation = Orientation(xyzw);
    return glm::degrees(std::atan2(2.0 * (rotation.w * rotation.y + rotation.x * rotation.z),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)));
}

glm::dquat CaptureAttachedBoxOffset(const robotics::CartesianPose& boxPose,
    const robotics::RobotState& state, std::array<double, 3>& positionOffset,
    std::array<double, 4>& rotationOffset)
{
    const glm::dquat tcpOrientation = TcpOrientation(state);
    const glm::dquat inverseTcp = glm::inverse(tcpOrientation);
    const glm::dvec3 boxPosition{boxPose.positionMeters[0], boxPose.positionMeters[1], boxPose.positionMeters[2]};
    const glm::dvec3 tcpPosition{state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]};
    const glm::dvec3 localPosition = inverseTcp * (boxPosition - tcpPosition);
    positionOffset = {localPosition.x, localPosition.y, localPosition.z};
    rotationOffset = QuaternionXyzw(inverseTcp * Orientation(boxPose.orientationXyzw));
    return tcpOrientation;
}

bool BoxCornersFitPlacement(const robotics::CartesianPose& boxPose,
    const robotics::CartesianPose& placementPose)
{
    const glm::dvec3 boxCenter{boxPose.positionMeters[0], boxPose.positionMeters[1], boxPose.positionMeters[2]};
    const glm::dvec3 targetCenter{placementPose.positionMeters[0], placementPose.positionMeters[1], placementPose.positionMeters[2]};
    const glm::dquat targetOrientation = Orientation(placementPose.orientationXyzw);
    const glm::dvec3 centerInTarget = glm::inverse(targetOrientation) * (boxCenter - targetCenter);
    const glm::dquat boxOrientationInTarget = glm::inverse(targetOrientation) * Orientation(boxPose.orientationXyzw);
    const double boxHalfSide = config::boxSideMeters * 0.5;
    const double targetHalfSide = config::placementAreaSideMeters * 0.5;
    for (const double xSign : {-1.0, 1.0})
        for (const double zSign : {-1.0, 1.0})
        {
            const glm::dvec3 boxCorner = centerInTarget + boxOrientationInTarget *
                glm::dvec3{xSign * boxHalfSide, 0.0, zSign * boxHalfSide};
            if (std::abs(boxCorner.x) > targetHalfSide || std::abs(boxCorner.z) > targetHalfSide)
                return false;
        }
    return true;
}

const char* RuntimeFaultReason(robotics::ErrorCode errorCode)
{
    switch (errorCode)
    {
    case robotics::ErrorCode::IkDidNotConverge: return "역기구학 계산이 목표에 도달하지 못했습니다.";
    case robotics::ErrorCode::JointLimitReached: return "관절 한계에 막혀 목표에 도달하지 못했습니다.";
    case robotics::ErrorCode::EnvironmentContact: return "로봇이 바닥이나 주변 물체에 닿았습니다.";
    case robotics::ErrorCode::SelfCollision: return "로봇 팔의 링크끼리 부딪혔습니다.";
    case robotics::ErrorCode::AttachedObjectCollision: return "로봇에 붙은 물체가 주변 물체와 부딪혔습니다.";
    default: return "로봇 제어기에서 오류를 보고했습니다.";
    }
}

glm::dquat ClosestSquarePlacementOrientation(const glm::dquat& goalBoxOrientation,
    const glm::dquat& boxRotationOffset, const glm::dquat& currentTcpOrientation)
{
    glm::dquat bestTcpOrientation = goalBoxOrientation * glm::inverse(boxRotationOffset);
    double bestAlignment = -1.0;
    for (int quarterTurn = 0;
         quarterTurn < config::PlacementSettings::equivalentSquareOrientations;
         ++quarterTurn)
    {
        const glm::dquat equivalentBoxOrientation = glm::angleAxis(
            static_cast<double>(quarterTurn) * glm::half_pi<double>(), glm::dvec3{0.0, 1.0, 0.0}) * goalBoxOrientation;
        const glm::dquat candidateTcpOrientation = glm::normalize(
            equivalentBoxOrientation * glm::inverse(boxRotationOffset));
        const double alignment = std::abs(glm::dot(candidateTcpOrientation, currentTcpOrientation));
        if (alignment > bestAlignment)
        {
            bestAlignment = alignment;
            bestTcpOrientation = candidateTcpOrientation;
        }
    }
    return bestTcpOrientation;
}

robotics::CartesianPose MakeBoxTarget(const robotics::CartesianPose& boxPose, double heightOffset,
    const std::array<double, 4>& orientation)
{
    robotics::CartesianPose target = boxPose;
    target.orientationXyzw = orientation;
    target.positionMeters[1] += heightOffset;
    return target;
}

robotics::CartesianPose MakeAttachedBoxTarget(const robotics::CartesianPose& boxPose, double heightOffset,
    const std::array<double, 4>& targetBoxOrientation, const std::array<double, 4>& boxRotationOffsetInTool,
    const std::array<double, 3>& boxOffsetInTool)
{
    const glm::dquat boxRotationOffset = Orientation(boxRotationOffsetInTool);
    const glm::dquat tcpOrientation = glm::normalize(
        Orientation(targetBoxOrientation) * glm::inverse(boxRotationOffset));
    const glm::dvec3 boxOffset = tcpOrientation *
        glm::dvec3{boxOffsetInTool[0], boxOffsetInTool[1], boxOffsetInTool[2]};
    robotics::CartesianPose target{};
    target.orientationXyzw = QuaternionXyzw(tcpOrientation);
    target.positionMeters = {boxPose.positionMeters[0] - boxOffset.x,
        boxPose.positionMeters[1] + heightOffset - boxOffset.y,
        boxPose.positionMeters[2] - boxOffset.z};
    return target;
}

robotics::Result MovePoseTo(const robotics::CartesianPose& pose, robotics::IRobotController& controller)
{
    return controller.BeginPosePlanning(pose);
}

robotics::LinearPathMoveCommand LinearCommandTo(const robotics::CartesianPose& pose,
    double linearVelocity = config::linearVelocityMetersPerSecond,
    double linearAcceleration = config::linearAccelerationMetersPerSecondSquared)
{
    robotics::LinearPathMoveCommand command;
    command.targetPoses.push_back(pose);
    command.maxLinearVelocityMetersPerSecond = linearVelocity;
    command.maxAngularVelocityRadiansPerSecond = config::angularVelocityRadiansPerSecond;
    command.maxLinearAccelerationMetersPerSecondSquared = linearAcceleration;
    command.maxAngularAccelerationRadiansPerSecondSquared = config::angularAccelerationRadiansPerSecondSquared;
    return command;
}

robotics::Result MoveLinearTo(const robotics::CartesianPose& pose, robotics::IRobotController& controller,
    double linearVelocity = config::linearVelocityMetersPerSecond,
    double linearAcceleration = config::linearAccelerationMetersPerSecondSquared)
{
    const auto command = LinearCommandTo(pose, linearVelocity, linearAcceleration);
    return controller.BeginLinearPathPlanning(command);
}

robotics::Result CommandGripper(
    config::GripperCommandSettings::PositionRequest position,
    robotics::IGripperController& gripper)
{
    robotics::GripperCommand command;
    command.positionRequest = static_cast<std::uint8_t>(position);
    command.speedRequest = config::GripperCommandSettings::speedRequest;
    command.forceRequest = config::GripperCommandSettings::forceRequest;
    return gripper.Command(command);
}

}


bool PickPlaceMission::SetResult(robotics::Result result)
{
    lastResult_ = std::move(result);
    hasResult_ = true;
    return lastResult_.Ok();
}

void PickPlaceMission::SetStageFromResult(robotics::Result result, Stage next)
{
    stage_ = SetResult(std::move(result)) ? next : Stage::Failed;
}

void PickPlaceMission::SetStageFromMotionResult(
    robotics::Result result, Stage next, robotics::IRobotController& controller)
{
    if (!SetResult(std::move(result)))
    {
        stage_ = Stage::Failed;
        return;
    }
    if (controller.IsMotionPlanning())
    {
        pendingStageAfterPlanning_ = next;
        stage_ = Stage::PlanningMotion;
        return;
    }
    stage_ = next;
}

void PickPlaceMission::BeginPickup(const robotics::CartesianPose& graspBoxPoseInBase,
    robotics::IRobotController& controller)
{
    const double boxYawRadians = glm::radians(YawDegrees(graspBoxPoseInBase.orientationXyzw));
    const glm::dquat desiredPickupOrientation = glm::normalize(
        glm::angleAxis(boxYawRadians, glm::dvec3{0.0, 1.0, 0.0}) * Orientation(graspOrientationXyzw_));
    pickupOrientationXyzw_ = QuaternionXyzw(desiredPickupOrientation);

    const auto approach = MakeBoxTarget(graspBoxPoseInBase, config::approachHeightMeters,
        pickupOrientationXyzw_);
    const auto descent = MakeBoxTarget(graspBoxPoseInBase, config::graspClearanceMeters,
        pickupOrientationXyzw_);
    recoveryPose_ = approach;
    SetStageFromMotionResult(controller.BeginPosePlanningWithLinearContinuation(
        approach, LinearCommandTo(descent)), Stage::MovingAbovePickup, controller);
}

void PickPlaceMission::UpdatePlanningMotion(robotics::IRobotController& controller)
{
    if (controller.IsMotionPlanning())
        return;

    auto result = controller.TakeMotionPlanningResult();
    if (!result)
    {
        SetResult({robotics::ErrorCode::Fault, "이동 경로 계산이 결과 없이 끝났습니다."});
        stage_ = Stage::Failed;
        return;
    }

    SetStageFromResult(std::move(*result), pendingStageAfterPlanning_);
    pendingStageAfterPlanning_ = Stage::Ready;
}

void PickPlaceMission::UpdateGraspOrientation(const robotics::RobotState& state)
{
    if (orientationReady_ || !state.valid || !state.tcpPoseValid)
        return;

    const glm::dquat current = TcpOrientation(state);
    // GLB에서 그리퍼의 Y축은 손가락 면에 수직인 접근 방향이다. X축 둘레로 90도 돌리면 손가락 면이 수직이고 접근 축이 바닥을 향한다.
    const glm::dquat faceDown = glm::angleAxis(glm::half_pi<double>(), glm::dvec3{1.0, 0.0, 0.0}) * current;
    graspOrientationXyzw_ = {faceDown.x, faceDown.y, faceDown.z, faceDown.w};
    orientationReady_ = true;
}

void PickPlaceMission::UpdateStage(const UpdateContext& context)
{
    switch (stage_)
    {
    case Stage::MovingAbovePickup:
    case Stage::MovingDownToPickup:
    case Stage::Closing:
    case Stage::Lifting:
        UpdatePickup(context);
        break;
    case Stage::RaisingAfterResume:
    case Stage::TransitingToPlacement:
    case Stage::MovingToPlacementOverhead:
    case Stage::AligningAbovePlacement:
    case Stage::MovingDownToPlacement:
    case Stage::SettlingAtPlacement:
    case Stage::Opening:
        UpdatePlacement(context);
        break;
    case Stage::Retreating:
        UpdateRetreat(context);
        break;
    case Stage::Ready:
    case Stage::PlanningMotion:
    case Stage::Recovering:
    case Stage::Complete:
    case Stage::Failed:
        break;
    }
}

bool PickPlaceMission::IsMissionActive() const noexcept
{
    return stage_ != Stage::Ready && stage_ != Stage::Complete && stage_ != Stage::Failed;
}

bool PickPlaceMission::IsEnvironmentCollisionStopped(const robotics::RobotState& state) const noexcept
{
    return state.mode == robotics::RobotMode::Idle &&
        state.errorCode == robotics::ErrorCode::EnvironmentContact &&
        IsMissionActive() && stage_ != Stage::Recovering;
}

void PickPlaceMission::Update(const robotics::RobotState& state, robotics::IRobotController& controller,
    robotics::IGripperController& gripper, bool boxGrasped,
    const robotics::CartesianPose& graspBoxPoseInBase,
    const robotics::CartesianPose& placementPoseInBase,
    std::chrono::steady_clock::time_point now)
{
    // 정지 뒤에는 상태 피드백이 Idle이어도 임무 단계를 진행하지 않는다.
    if (taskPaused_)
        return;

    const bool idle = state.mode == robotics::RobotMode::Idle;
    if (IsMissionActive() && state.mode == robotics::RobotMode::Fault)
    {
        const auto errorCode = state.errorCode == robotics::ErrorCode::None
            ? robotics::ErrorCode::Fault : state.errorCode;
        SetResult({errorCode, std::string{"로봇 동작 오류: "} + RuntimeFaultReason(errorCode)});
        taskSucceeded_ = false;
        stage_ = Stage::Failed;
        autoLoopEnabled_ = false;
        return;
    }
    if (stage_ == Stage::PlanningMotion)
    {
        UpdatePlanningMotion(controller);
        return;
    }
    bool gripperCloseFinished = false;
    if (stage_ == Stage::Closing && !boxGrasped)
    {
        const auto gripperState = gripper.GetState();
        gripperCloseFinished = gripperState.mode == robotics::GripperMode::Idle &&
            gripperState.objectStatus == robotics::GripperObjectStatus::AtRequestedPosition;
    }
    // Viewer가 겹친 틱을 취소하면 Controller는 직전 관절각에서 Idle로 멈추고 충돌 원인을 남긴다. 여기서는 하강을 이어가지 않고 저장한 높이로 후퇴한다.
    const bool collisionStopped = IsEnvironmentCollisionStopped(state);

    UpdateGraspOrientation(state);

    if (collisionStopped)
    {
        SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::Recovering, controller);
    }
    else
    {
        const UpdateContext context{state, controller, gripper, boxGrasped, gripperCloseFinished,
            graspBoxPoseInBase, placementPoseInBase, idle, now};
        UpdateStage(context);
    }

    if (!collisionStopped && stage_ == Stage::Recovering && idle)
    {
        stage_ = Stage::Retreating;
    }
    if (stage_ == Stage::Failed)
        autoLoopEnabled_ = false;
}

void PickPlaceMission::UpdatePickup(const UpdateContext& context)
{
    switch (stage_)
    {
    case Stage::MovingAbovePickup:
        if (context.idle)
        {
            const auto descent = MakeBoxTarget(context.graspBoxPoseInBase, config::graspClearanceMeters,
                pickupOrientationXyzw_);
            SetStageFromMotionResult(MoveLinearTo(descent, context.controller), Stage::MovingDownToPickup,
                context.controller);
        }
        break;
    case Stage::MovingDownToPickup:
        if (context.idle)
            SetStageFromResult(CommandGripper(
                config::GripperCommandSettings::PositionRequest::Closed, context.gripper), Stage::Closing);
        break;
    case Stage::Closing:
        if (context.boxGrasped && context.state.tcpPoseValid)
        {
            // 파지 순간의 상자와 공구 사이 위치·방향을 저장해, 들어 올릴 때도 상자 중심을 유지한다.
            CaptureAttachedBoxOffset(context.graspBoxPoseInBase, context.state,
                boxOffsetInTool_, boxRotationOffsetInTool_);
            graspedBoxPoseInBase_ = context.graspBoxPoseInBase;
            recoveryPose_ = MakeAttachedBoxTarget(context.graspBoxPoseInBase, config::approachHeightMeters,
                context.graspBoxPoseInBase.orientationXyzw, boxRotationOffsetInTool_, boxOffsetInTool_);
            SetStageFromMotionResult(MoveLinearTo(recoveryPose_, context.controller), Stage::Lifting,
                context.controller);
        }
        else if (context.gripperCloseFinished)
        {
            taskSucceeded_ = false;
            if (!SetResult(CommandGripper(
                config::GripperCommandSettings::PositionRequest::Open, context.gripper)))
                stage_ = Stage::Failed;
            else
                SetStageFromMotionResult(MoveLinearTo(recoveryPose_, context.controller), Stage::Recovering,
                    context.controller);
        }
        break;
    case Stage::Lifting:
        if (context.idle)
        {
            recoveryPose_ = MakeAttachedBoxTarget(graspedBoxPoseInBase_, config::transitBoxHeightMeters,
                graspedBoxPoseInBase_.orientationXyzw, boxRotationOffsetInTool_, boxOffsetInTool_);
            SetStageFromMotionResult(MoveLinearTo(recoveryPose_, context.controller),
                Stage::TransitingToPlacement, context.controller);
        }
        break;
    default:
        break;
    }
}

void PickPlaceMission::UpdatePlacement(const UpdateContext& context)
{
    switch (stage_)
    {
    case Stage::RaisingAfterResume:
        if (context.idle)
            stage_ = Stage::TransitingToPlacement;
        break;
    case Stage::TransitingToPlacement:
        if (context.idle)
        {
            const glm::dquat boxRotationOffset = Orientation(boxRotationOffsetInTool_);
            const glm::dquat currentTcp = TcpOrientation(context.state);
            const glm::dquat chosenTcp = ClosestSquarePlacementOrientation(
                Orientation(context.placementPoseInBase.orientationXyzw), boxRotationOffset, currentTcp);
            placementBoxOrientationXyzw_ = QuaternionXyzw(chosenTcp * boxRotationOffset);
            recoveryPose_ = MakeAttachedBoxTarget(context.placementPoseInBase, config::transitBoxHeightMeters,
                QuaternionXyzw(currentTcp * boxRotationOffset), boxRotationOffsetInTool_, boxOffsetInTool_);
            SetStageFromMotionResult(MovePoseTo(recoveryPose_, context.controller),
                Stage::MovingToPlacementOverhead, context.controller);
        }
        break;
    case Stage::MovingToPlacementOverhead:
        if (context.idle)
        {
            recoveryPose_ = MakeAttachedBoxTarget(context.placementPoseInBase, config::approachHeightMeters,
                placementBoxOrientationXyzw_, boxRotationOffsetInTool_, boxOffsetInTool_);
            const auto placement = MakeAttachedBoxTarget(context.placementPoseInBase, 0.0,
                placementBoxOrientationXyzw_, boxRotationOffsetInTool_, boxOffsetInTool_);
            SetStageFromMotionResult(context.controller.BeginPosePlanningWithLinearContinuation(
                recoveryPose_, LinearCommandTo(placement)), Stage::AligningAbovePlacement, context.controller);
        }
        break;
    case Stage::AligningAbovePlacement:
        if (context.idle)
        {
            const auto placement = MakeAttachedBoxTarget(context.placementPoseInBase, 0.0,
                placementBoxOrientationXyzw_, boxRotationOffsetInTool_, boxOffsetInTool_);
            SetStageFromMotionResult(MoveLinearTo(placement, context.controller),
                Stage::MovingDownToPlacement, context.controller);
        }
        break;
    case Stage::MovingDownToPlacement:
        if (context.idle)
        {
            placementDwellStarted_ = context.now;
            stage_ = Stage::SettlingAtPlacement;
        }
        break;
    case Stage::SettlingAtPlacement:
        if (context.idle &&
            context.now - placementDwellStarted_ >=
                config::PlacementSettings::settleDuration)
            SetStageFromResult(CommandGripper(
                config::GripperCommandSettings::PositionRequest::Open, context.gripper), Stage::Opening);
        break;
    case Stage::Opening:
    {
        const auto gripperState = context.gripper.GetState();
        const bool gripperSettled = gripperState.mode == robotics::GripperMode::Idle &&
            gripperState.objectStatus == robotics::GripperObjectStatus::AtRequestedPosition;
        const bool openingStoppedAtContact = gripperState.mode == robotics::GripperMode::Stopped &&
            gripperState.objectStatus == robotics::GripperObjectStatus::ContactWhileOpening;
        if (!context.boxGrasped && (gripperSettled || openingStoppedAtContact))
        {
            placementReleased_ = true;
            SetStageFromMotionResult(MoveLinearTo(recoveryPose_, context.controller),
                Stage::Retreating, context.controller);
        }
        break;
    }
    default:
        break;
    }
}

void PickPlaceMission::UpdateRetreat(const UpdateContext& context)
{
    if (!context.idle)
        return;

    const bool missionSucceeded = taskSucceeded_ && placementReleased_ && !context.boxGrasped &&
        BoxCornersFitPlacement(context.graspBoxPoseInBase, context.placementPoseInBase);
    stage_ = missionSucceeded ? Stage::Complete : Stage::Failed;
    if (missionSucceeded)
    {
        ++completedMissionCount_;
        missionSuccessEventPending_ = true;
    }
    else
    {
        autoLoopEnabled_ = false;
    }
}

void PickPlaceMission::ApplyActions(const PickPlaceMissionActions& actions,
    const robotics::RobotState& state, robotics::IRobotController& controller, bool boxGrasped,
    const robotics::CartesianPose& graspBoxPoseInBase)
{
    const bool canStart = !taskPaused_ && !boxGrasped &&
        (stage_ == Stage::Ready || stage_ == Stage::Complete || stage_ == Stage::Failed);
    // 첫 임무는 버튼으로 시작하고 성공 뒤에는 Viewer가 상자를 옮기고 Ready로 돌려놓아 자동으로 다음 임무를 시작한다.
    const bool startRequested = canStart && orientationReady_ &&
        (actions.start || (autoLoopEnabled_ && stage_ == Stage::Ready));
    if (startRequested)
    {
        taskPaused_ = false;
        taskSucceeded_ = true;
        placementReleased_ = false;
        autoLoopEnabled_ = true;
        BeginPickup(graspBoxPoseInBase, controller);
    }
    if (taskPaused_ && actions.resume)
    {
        ResumeFromPause(state, controller, boxGrasped, graspBoxPoseInBase);
    }
    else if (actions.stop)
    {
        StopForPause(controller);
    }

}

void PickPlaceMission::ResumeFromPause(const robotics::RobotState& state,
    robotics::IRobotController& controller, bool boxGrasped,
    const robotics::CartesianPose& graspBoxPoseInBase)
{
    taskPaused_ = false;
    taskSucceeded_ = true;
    if (boxGrasped)
    {
        ResumeWithAttachedBox(state, controller, graspBoxPoseInBase);
        return;
    }

    if (placementReleased_)
    {
        stage_ = Stage::Retreating;
        return;
    }

    if (stage_ == Stage::Opening)
    {
        placementReleased_ = false;
        return;
    }

    placementReleased_ = false;
    BeginPickup(graspBoxPoseInBase, controller);
}

void PickPlaceMission::ResumeWithAttachedBox(const robotics::RobotState& state,
    robotics::IRobotController& controller,
    const robotics::CartesianPose& graspBoxPoseInBase)
{
    placementReleased_ = false;
    if (!state.valid || !state.tcpPoseValid)
    {
        SetResult({robotics::ErrorCode::InvalidCommand,
            "공구 끝 위치 정보가 없어 안전하게 다시 시작할 수 없습니다."});
        stage_ = Stage::Failed;
        return;
    }

    CaptureAttachedBoxOffset(graspBoxPoseInBase, state, boxOffsetInTool_, boxRotationOffsetInTool_);
    graspedBoxPoseInBase_ = graspBoxPoseInBase;
    const double currentBoxHeight = graspedBoxPoseInBase_.positionMeters[1];
    const double resumeTransitHeight = std::max(currentBoxHeight, config::transitBoxHeightMeters);
    const double liftDistance = resumeTransitHeight - currentBoxHeight;
    recoveryPose_ = MakeAttachedBoxTarget(graspedBoxPoseInBase_, liftDistance,
        graspedBoxPoseInBase_.orientationXyzw, boxRotationOffsetInTool_, boxOffsetInTool_);
    graspedBoxPoseInBase_.positionMeters[1] = resumeTransitHeight - config::transitBoxHeightMeters;
    SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::RaisingAfterResume, controller);
}

void PickPlaceMission::StopForPause(robotics::IRobotController& controller)
{
    if (!SetResult(controller.Stop()))
        return;

    if (stage_ == Stage::PlanningMotion)
        controller.TakeMotionPlanningResult();

    taskPaused_ = true;
    autoLoopEnabled_ = false;
}

PickPlaceMissionSnapshot PickPlaceMission::Snapshot() const
{
    const char* stageName = "Unknown";
    switch (stage_)
    {
    case Stage::Ready: stageName = "Ready"; break;
    case Stage::MovingAbovePickup: stageName = "Moving above the box"; break;
    case Stage::MovingDownToPickup: stageName = "Lowering to the box"; break;
    case Stage::Closing: stageName = "Closing gripper"; break;
    case Stage::Lifting: stageName = "Lifting the box"; break;
    case Stage::RaisingAfterResume: stageName = "Lifting to resume height"; break;
    case Stage::TransitingToPlacement: stageName = "Planning path to goal"; break;
    case Stage::PlanningMotion: stageName = "Planning motion"; break;
    case Stage::MovingToPlacementOverhead: stageName = "Moving above the goal"; break;
    case Stage::AligningAbovePlacement: stageName = "Aligning box over goal"; break;
    case Stage::MovingDownToPlacement: stageName = "Lowering box"; break;
    case Stage::SettlingAtPlacement: stageName = "Holding at placement"; break;
    case Stage::Opening: stageName = "Opening gripper"; break;
    case Stage::Retreating: stageName = "Retracting from goal"; break;
    case Stage::Recovering: stageName = "Retracting after collision"; break;
    case Stage::Complete: stageName = "Complete"; break;
    case Stage::Failed: stageName = "Failed"; break;
    }
    return {stageName, lastResult_.message, taskPaused_, stage_ == Stage::Complete, autoLoopEnabled_,
        hasResult_, lastResult_.Ok(), !taskPaused_ &&
            (stage_ == Stage::Ready || stage_ == Stage::Complete || stage_ == Stage::Failed),
        completedMissionCount_};
}

bool PickPlaceMission::ConsumeSuccessEvent() noexcept
{
    const bool result = missionSuccessEventPending_;
    missionSuccessEventPending_ = false;
    return result;
}

void PickPlaceMission::PrepareNextTask() noexcept
{
    stage_ = Stage::Ready;
    taskPaused_ = false;
    placementReleased_ = false;
    taskSucceeded_ = true;
}
}
