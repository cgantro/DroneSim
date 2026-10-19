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
