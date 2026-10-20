#pragma once

#include "robotics/planning/PlanningPolicy.h"
#include "robotics/planning/StateValidity.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace grasplink::robotics::planning
{

/**
 * @brief 관절 경로를 확인할 때 사용할 표본 간격을 설정한다.
 */
struct JointPathPlannerOptions final
{
    PlanningPolicy validationPolicy{};
};

struct JointPathPlan final
{
    std::vector<JointVector> points;
    std::size_t iterations = 0;
    std::size_t validityChecks = 0;
};

/**
 * @brief 시작 관절각에서 목표 관절각까지 충돌 없이 이동할 수 있는지 확인한다.
 * @details 허용된 관절 변화 간격으로 중간 자세를 검사하므로, 두 끝점이 모두 안전해도 그 사이에서 충돌하면 실패한다.
 */
[[nodiscard]] JointStateInvalidity ValidateJointPath(
    const JointVector& start,
    const JointVector& end,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const PlanningPolicy& policy = kDefaultPlanningPolicy);

enum class JointPathPlanningState : std::uint8_t
{
    Idle,
    Running,
    Completed,
    Failed,
    Cancelled
};

/**
 * @brief 관절 제한과 충돌을 검사하면서 MoveJ용 직접 경로를 확인한다.
 * @details 경로를 `validationPolicy`의 관절 간격으로 검사하며, 막히면 우회하지 않고 요청을 거부한다.
 * Advance의 작업 예산은 확인할 시작점, 끝점, 경로 표본의 최대 개수다.
 * Begin()에 넘긴 RobotSpecification과 검사 함수가 참조하는 객체는 작업이 완료·실패·취소될 때까지 살아 있어야 한다.
 * 검사 함수는 복사하지만, 함수가 참조로 사용하는 객체는 복사하지 않는다.
 */
class JointPathPlanningJob final
{
public:
    JointPathPlanningJob();
    ~JointPathPlanningJob();
    JointPathPlanningJob(JointPathPlanningJob&&) noexcept;
    JointPathPlanningJob& operator=(JointPathPlanningJob&&) noexcept;
    JointPathPlanningJob(const JointPathPlanningJob&) = delete;
    JointPathPlanningJob& operator=(const JointPathPlanningJob&) = delete;

    Result Begin(const models::RobotSpecification& specification,
        const JointVector& start,
        const JointVector& goal,
        const StateValidityChecker& stateValidityChecker,
        const JointPathPlannerOptions& options = {});
    /// 작업이 여러 호출에 걸쳐 사양을 빌리므로 임시 사양은 받을 수 없다.
    Result Begin(models::RobotSpecification&&,
        const JointVector&,
        const JointVector&,
        const StateValidityChecker&,
        const JointPathPlannerOptions& = {}) = delete;
    JointPathPlanningState Advance(std::size_t workBudget);
    void Cancel() noexcept;
    [[nodiscard]] JointPathPlanningState GetState() const noexcept;
    [[nodiscard]] const Result& GetResult() const noexcept;
    [[nodiscard]] const JointPathPlan& GetPlan() const noexcept;
    [[nodiscard]] std::optional<JointPathPlan> TakePlan();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace grasplink::robotics::planning
