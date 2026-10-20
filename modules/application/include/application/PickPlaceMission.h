#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/core/IGripperController.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <string_view>

namespace grasplink::application
{
struct PickPlaceMissionActions
{
    bool start = false;
    bool resume = false;
    bool stop = false;
};

struct PickPlaceMissionSnapshot
{
    // 단계 이름은 정적 문자열이고, 결과 메시지는 다음 명령 전까지 임무 객체가 보관한다.
    std::string_view stageLabel;
    std::string_view lastMessage;
    bool paused = false;
    bool missionSucceeded = false;
    bool autoRepeat = false;
    bool hasResult = false;
    bool lastRequestAccepted = true;
    bool canStart = false;
    std::uint64_t completedCount = 0;
};

/**
 * @brief 상자 집기와 목표 위치에 놓기까지의 임무 단계를 관리한다.
 * @details ViewerApp은 매 프레임 상태를 먼저 갱신하고, GUI에서 받은 버튼 입력을 그 다음 처리한다.
 * Snapshot의 문자열 보기는 다음 임무 갱신 또는 명령 전까지만 유효하므로 즉시 화면에 전달한다.
 */
class PickPlaceMission final
{
public:
    /**
     * @brief 로봇 상태를 읽고 임무의 다음 명령과 단계를 갱신합니다.
     * @param now 현재 시각입니다. 생략하면 실제 시계를 사용합니다.
     */
    void Update(const robotics::RobotState& state, robotics::IRobotController& controller,
        robotics::IGripperController& gripper, bool boxGrasped,
        const robotics::CartesianPose& graspBoxPoseInBase,
        const robotics::CartesianPose& placementPoseInBase,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
    /** @brief 패널에서 받은 시작, 재개, 정지 입력을 임무 명령으로 처리한다. */
    void ApplyActions(const PickPlaceMissionActions& actions,
        const robotics::RobotState& state,
        robotics::IRobotController& controller, bool boxGrasped,
        const robotics::CartesianPose& graspBoxPoseInBase);
    /** @brief 화면과 테스트에 필요한 임무 상태를 복사 없이 읽기 전용으로 제공한다. */
    [[nodiscard]] PickPlaceMissionSnapshot Snapshot() const;

    /** @brief 임무 성공 알림이 대기 중이면 한 번 소비하고 대기 상태를 지운다. */
    [[nodiscard]] bool ConsumeSuccessEvent() noexcept;

    /** @brief 성공 후 다음 무작위 상자 작업을 받을 준비를 한다. */
    void PrepareNextTask() noexcept;

private:
    struct UpdateContext
    {
        const robotics::RobotState& state;
        robotics::IRobotController& controller;
        robotics::IGripperController& gripper;
        bool boxGrasped;
        bool gripperCloseFinished;
        const robotics::CartesianPose& graspBoxPoseInBase;
        const robotics::CartesianPose& placementPoseInBase;
        bool idle;
        std::chrono::steady_clock::time_point now;
    };

    enum class Stage
    {
        Ready, MovingAbovePickup, MovingDownToPickup,
        Closing, Lifting,
        TransitingToPlacement, PlanningMotion, MovingToPlacementOverhead,
        AligningAbovePlacement, MovingDownToPlacement, SettlingAtPlacement, Opening, Retreating,
        Recovering, Complete, Failed, RaisingAfterResume
    };

    bool SetResult(robotics::Result result);
    void SetStageFromResult(robotics::Result result, Stage next);
    void SetStageFromMotionResult(robotics::Result result, Stage next, robotics::IRobotController& controller);
    void UpdatePlanningMotion(robotics::IRobotController& controller);
    void UpdateGraspOrientation(const robotics::RobotState& state);
    void UpdateStage(const UpdateContext& context);
    void UpdatePickup(const UpdateContext& context);
    void UpdatePlacement(const UpdateContext& context);
    void UpdateRetreat(const UpdateContext& context);
    [[nodiscard]] bool IsStartableStage() const noexcept;
    [[nodiscard]] bool IsMissionActive() const noexcept;
    [[nodiscard]] bool IsEnvironmentCollisionStopped(const robotics::RobotState& state) const noexcept;
    void ResumeFromPause(const robotics::RobotState& state, robotics::IRobotController& controller,
        bool boxGrasped, const robotics::CartesianPose& graspBoxPoseInBase);
    // 상자를 든 채 재개할 때는 현재 공구 자세를 기준으로 상자와의 상대 위치를 다시 맞춘다.
    void ResumeWithAttachedBox(const robotics::RobotState& state,
        robotics::IRobotController& controller,
        const robotics::CartesianPose& graspBoxPoseInBase);
    void StopForPause(robotics::IRobotController& controller);
    void BeginPickup(const robotics::CartesianPose& graspBoxPoseInBase,
        robotics::IRobotController& controller);

    std::array<double, 4> graspOrientationXyzw_{};
    std::array<double, 4> pickupOrientationXyzw_{};
    std::array<double, 4> placementBoxOrientationXyzw_{};
    robotics::CartesianPose graspedBoxPoseInBase_{};
    std::array<double, 3> boxOffsetInTool_{};
    std::array<double, 4> boxRotationOffsetInTool_{0.0, 0.0, 0.0, 1.0};
    robotics::CartesianPose recoveryPose_{};
    robotics::Result lastResult_{};
    std::chrono::steady_clock::time_point placementDwellStarted_{};
    Stage stage_ = Stage::Ready;
    Stage pendingStageAfterPlanning_ = Stage::Ready;
    bool orientationReady_ = false;
    bool hasResult_ = false;
    bool taskSucceeded_ = true;
    bool placementReleased_ = false;
    bool missionSuccessEventPending_ = false;
    bool autoLoopEnabled_ = false;
    bool taskPaused_ = false;
    std::uint64_t completedMissionCount_ = 0;
};
}
