#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/planning/PlanningPolicy.h"
#include "robotics/planning/StateValidity.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace grasplink::robotics::planning
{

/** @brief 계획된 관절 자세와 TCP 자세, 이전 표본에서 이동하는 최소 시간을 보관한다. */
struct LinearPathPoint
{
    JointVector joints;
    CartesianPose tcpPose{};
    // 이전 표본부터 이 TCP·관절 변위를 각 속도 상한 안에서 완료하는 최소 시간 [s]다.
    double durationSeconds = 0.0;
};

/** @brief TCP 경로를 실행할 관절 표본과 경로 전체의 계획 속도를 보관한다. */
struct LinearPathPlan
{
    std::vector<LinearPathPoint> points;
    double plannedLinearVelocity = 0.0;
    double plannedAngularVelocity = 0.0;
    bool hasMotion = false;
    std::size_t ikSolveCount = 0;
    std::size_t ikIterationCount = 0;
    std::size_t tcpStraightnessChecks = 0;
    std::size_t jointPathValidityChecks = 0;
    std::size_t validityStateChecks = 0;
    std::size_t tcpRefinementCount = 0;
};

enum class LinearPathPlanningState { Idle, Running, Completed, Failed, Cancelled };

/**
 * @brief 직선 TCP 경로를 여러 번의 짧은 호출로 나누어 계획한다.
 * @details IK 반복과 관절 경로 검사는 Begin()을 호출한 스레드에서 수행한다. Advance()를 다른 스레드에서 호출하면 작업을 실패 처리하며, 같은 작업에 대한 호출을 동시에 실행해서는 안 된다.
 * 상태 검사 함수가 Jolt나 ECS를 참조할 수 있으므로 이 객체와 검사 함수를 소유한 스레드에서 진행해야 한다.
 * Begin()에 넘긴 Robot 사양과 IK 계산기는 작업이 완료·실패·취소될 때까지 살아 있어야 한다.
 * 검사 함수와 진단 함수는 복사하지만, 함수가 참조로 사용하는 객체는 복사하지 않는다.
 */
class LinearPathPlanningJob final
{
public:
    LinearPathPlanningJob();
    ~LinearPathPlanningJob();
    LinearPathPlanningJob(LinearPathPlanningJob&&) noexcept;
    LinearPathPlanningJob& operator=(LinearPathPlanningJob&&) noexcept;
    LinearPathPlanningJob(const LinearPathPlanningJob&) = delete;
    LinearPathPlanningJob& operator=(const LinearPathPlanningJob&) = delete;

    Result Begin(const LinearPathMoveCommand& command,
        const models::RobotSpecification& specification,
        const JointVector& startJoints,
        const CartesianPose& startTcp,
        kinematics::LevenbergMarquardtIk& inverse,
        const StateValidityChecker& stateValidityChecker,
        const PlanningPolicy& policy = kDefaultPlanningPolicy,
        const std::function<std::string()>& validityDiagnosticProvider = {});
    /// 계획 작업이 끝날 때까지 사양을 빌리므로 임시 사양은 받을 수 없다.
    Result Begin(const LinearPathMoveCommand&,
        models::RobotSpecification&&,
        const JointVector&,
        const CartesianPose&,
        kinematics::LevenbergMarquardtIk&,
        const StateValidityChecker&,
        const PlanningPolicy& = kDefaultPlanningPolicy,
        const std::function<std::string()>& = {}) = delete;
    /**
     * @brief 지정한 작업 단위 수만큼 경로 계획을 진행한다.
     * @details IK 반복 1회, TCP 직선성 검사 1회, 관절 경로 표본의 상태 검사 1회를 각각 작업 단위로 센다. 매 프레임에 작은 예산을 주면 계획이 렌더 루프를 오래 막지 않는다.
     * @param workBudget 이번 호출에서 허용하는 최대 작업 단위 수다. 0이면 진행하지 않는다.
     * @return 계획 중, 완료, 실패 또는 취소 상태다.
     */
    LinearPathPlanningState Advance(std::size_t workBudget);
    void Cancel() noexcept;
    [[nodiscard]] LinearPathPlanningState GetState() const noexcept;
    [[nodiscard]] const Result& GetResult() const noexcept;
    [[nodiscard]] const LinearPathPlan& GetPlan() const noexcept;
    [[nodiscard]] std::optional<LinearPathPlan> TakePlan();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * @brief TCP 직선 경로를 IK 표본과 관절 경로 검증으로 바꾼다.
 * @details 경로가 유효한지 전부 확인한 뒤에만 결과 계획을 갱신한다. 샘플 한도를 넘으면 끝점의 도달 가능성과 경로 구간 수 초과를 구분한다.
 */
Result BuildLinearPath(
    const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification,
    const JointVector& startJoints,
    const CartesianPose& startTcp,
    kinematics::LevenbergMarquardtIk& inverse,
    const StateValidityChecker& stateValidityChecker,
    LinearPathPlan& plan,
    const PlanningPolicy& policy = kDefaultPlanningPolicy,
    const std::function<std::string()>& validityDiagnosticProvider = {});

} // namespace grasplink::robotics::planning
