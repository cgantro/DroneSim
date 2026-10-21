#include "robotics/planning/JointPathPlanner.h"

#include "robotics/planning/detail/JointPathSampling.h"

#include <algorithm>
#include <cmath>
#include <thread>
#include <utility>

namespace grasplink::robotics::planning
{
namespace
{
JointVector Interpolate(const JointVector& start, const JointVector& goal, double fraction)
{
    JointVector sample(start.size());
    for (std::size_t joint = 0; joint < start.size(); ++joint)
        sample[joint] = start[joint] + (goal[joint] - start[joint]) * fraction;
    return sample;
}
}

struct JointPathPlanningJob::Impl
{
    enum class EndpointCheckStage
    {
        Start,
        Goal,
        IntermediateSamples
    };

    const models::RobotSpecification* specification = nullptr;
    JointVector start;
    JointVector goal;
    StateValidityChecker checker;
    JointPathPlan plan;
    Result result{};
    JointPathPlanningState state = JointPathPlanningState::Idle;
    std::thread::id ownerThread{};
    EndpointCheckStage endpoint = EndpointCheckStage::Start;
    std::size_t intervalCount = 0;
    std::size_t nextInterval = 1;

    void ReleaseBorrowedInputs()
    {
        // 끝난 작업은 모델이나 검사 함수를 다시 부르지 않으므로 빌린 참조를 놓는다.
        specification = nullptr;
        checker = {};
    }

    JointStateInvalidity Check(const JointVector& joints)
    {
        ++plan.validityChecks;
        return ValidateJointState(*specification, joints, checker);
    }

    void Complete()
    {
        if (start == goal)
            plan.points = {start};
        else
            plan.points = {start, goal};
        state = JointPathPlanningState::Completed;
        ReleaseBorrowedInputs();
    }

    void Fail(JointStateInvalidity invalidity)
    {
        result = MapJointStateInvalidity(invalidity);
        result.message = "JointPathPlanner: " + result.message;
        state = JointPathPlanningState::Failed;
        ReleaseBorrowedInputs();
    }

    void AdvanceOne()
    {
        switch (endpoint)
        {
        case EndpointCheckStage::Start:
        {
            endpoint = EndpointCheckStage::Goal;
            const auto invalidity = Check(start);
            if (invalidity != JointStateInvalidity::None)
                Fail(invalidity);
            break;
        }
        case EndpointCheckStage::Goal:
        {
            endpoint = EndpointCheckStage::IntermediateSamples;
            const auto invalidity = Check(goal);
            if (invalidity != JointStateInvalidity::None)
                Fail(invalidity);
            else if (start == goal || intervalCount == 1)
                Complete();
            break;
        }
        case EndpointCheckStage::IntermediateSamples:
        {
            // 시작점과 끝점은 따로 검사했으므로 내부 표본만 확인한다.
            const auto invalidity = Check(Interpolate(start, goal,
                static_cast<double>(nextInterval) / static_cast<double>(intervalCount)));
            if (invalidity != JointStateInvalidity::None)
            {
                Fail(invalidity);
                return;
            }
            ++nextInterval;
            if (nextInterval >= intervalCount)
                Complete();
            break;
        }
        }
    }
};

JointStateInvalidity ValidateJointPath(
    const JointVector& start,
    const JointVector& end,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const PlanningPolicy& policy)
{
    // 먼저 두 끝점의 관절 수와 허용 범위를 확인한다. 문제가 있으면 충돌 검사 함수를 호출하지 않는다.
    const auto startInvalidity = ValidateJointState(specification, start, {});
    if (startInvalidity != JointStateInvalidity::None)
        return startInvalidity;
    const auto endInvalidity = ValidateJointState(specification, end, {});
    if (endInvalidity != JointStateInvalidity::None)
        return endInvalidity;

    const auto intervals = detail::JointPathIntervalCount(start, end, specification, policy);
    if (!intervals)
        return JointStateInvalidity::PathSamplingLimitExceeded;
    JointVector sample(start.size());
    for (std::size_t step = 1; step <= *intervals; ++step)
    {
        const double fraction = static_cast<double>(step) / static_cast<double>(*intervals);
        for (std::size_t joint = 0; joint < start.size(); ++joint)
            sample[joint] = start[joint] + (end[joint] - start[joint]) * fraction;
        // 끝점이 유효하므로 선형 보간한 중간 관절값도 한계 안에 있다. 각 표본에서는 충돌 검사만 추가로 수행한다.
        const auto invalidity = stateValidityChecker ?
            stateValidityChecker(sample) : JointStateInvalidity::None;
        if (invalidity != JointStateInvalidity::None)
            return invalidity;
    }
    return JointStateInvalidity::None;
}

JointPathPlanningJob::JointPathPlanningJob() : impl_(std::make_unique<Impl>()) {}
JointPathPlanningJob::~JointPathPlanningJob() = default;
JointPathPlanningJob::JointPathPlanningJob(JointPathPlanningJob&&) noexcept = default;
JointPathPlanningJob& JointPathPlanningJob::operator=(JointPathPlanningJob&&) noexcept = default;

Result JointPathPlanningJob::Begin(const models::RobotSpecification& specification,
    const JointVector& start, const JointVector& goal,
    const StateValidityChecker& stateValidityChecker, const PlanningPolicy& policy)
{
    auto& job = *impl_;
    job.specification = &specification;
    job.start = start;
    job.goal = goal;
    job.checker = stateValidityChecker;
    job.plan = {};
    job.result = Result::Success();
    job.ownerThread = std::this_thread::get_id();
    job.endpoint = Impl::EndpointCheckStage::Start;
    job.nextInterval = 1;
    job.state = JointPathPlanningState::Running;

    if (specification.joints == nullptr || specification.jointCount == 0 ||
        !std::isfinite(policy.jointCollisionSampleSpacingRadians) ||
        policy.jointCollisionSampleSpacingRadians <= 0.0 ||
        !std::isfinite(policy.prismaticJointCollisionSampleSpacingMeters) ||
        policy.prismaticJointCollisionSampleSpacingMeters <= 0.0 ||
        policy.maximumPathIntervals == 0 ||
        start.size() != specification.jointCount || goal.size() != specification.jointCount)
    {
        job.result = {ErrorCode::InvalidCommand, "JointPathPlanner: invalid robot, endpoints, or sample spacing"};
        job.state = JointPathPlanningState::Failed;
        job.ReleaseBorrowedInputs();
        return job.result;
    }

    const auto startInvalidity = ValidateJointState(specification, start, {});
    const auto goalInvalidity = ValidateJointState(specification, goal, {});
    const auto endpointInvalidity = startInvalidity != JointStateInvalidity::None ?
        startInvalidity : goalInvalidity;
    if (endpointInvalidity != JointStateInvalidity::None)
    {
        job.Fail(endpointInvalidity);
        return job.result;
    }

    const auto intervals = detail::JointPathIntervalCount(
        start, goal, specification, policy);
    if (!intervals)
    {
        job.result = {ErrorCode::InvalidCommand, "JointPathPlanner: path exceeds the sample limit"};
        job.state = JointPathPlanningState::Failed;
        job.ReleaseBorrowedInputs();
        return job.result;
    }
    job.intervalCount = *intervals;
    return Result::Success();
}

JointPathPlanningState JointPathPlanningJob::Advance(std::size_t workBudget)
{
    auto& job = *impl_;
    if (job.state != JointPathPlanningState::Running || workBudget == 0)
        return job.state;
    if (std::this_thread::get_id() != job.ownerThread)
    {
        job.result = {ErrorCode::InvalidCommand,
            "JointPathPlanner: state validity checks must run on the Begin caller thread"};
        job.state = JointPathPlanningState::Failed;
        job.ReleaseBorrowedInputs();
        return job.state;
    }

    while (workBudget-- > 0 && job.state == JointPathPlanningState::Running)
        job.AdvanceOne();
    return job.state;
}

void JointPathPlanningJob::Cancel() noexcept
{
    if (impl_->state == JointPathPlanningState::Running)
    {
        impl_->state = JointPathPlanningState::Cancelled;
        impl_->result = {ErrorCode::Cancelled, "JointPathPlanner: planning cancelled"};
    }
    impl_->ReleaseBorrowedInputs();
}

JointPathPlanningState JointPathPlanningJob::GetState() const noexcept { return impl_->state; }
const Result& JointPathPlanningJob::GetResult() const noexcept { return impl_->result; }
const JointPathPlan& JointPathPlanningJob::GetPlan() const noexcept { return impl_->plan; }

std::optional<JointPathPlan> JointPathPlanningJob::TakePlan()
{
    if (impl_->state != JointPathPlanningState::Completed)
        return std::nullopt;
    auto result = std::move(impl_->plan);
    impl_->plan = {};
    return result;
}

} // namespace grasplink::robotics::planning
