#include "robotics/planning/JointPathPlanner.h"

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
    const models::RobotSpecification* specification = nullptr;
    JointVector start;
    JointVector goal;
    StateValidityChecker checker;
    JointPathPlan plan;
    Result result{};
    JointPathPlanningState state = JointPathPlanningState::Idle;
    std::thread::id ownerThread{};
    std::size_t endpoint = 0;
    std::size_t intervalCount = 0;
    std::size_t nextInterval = 1;

    JointStateInvalidity Check(const JointVector& joints)
    {
        ++plan.validityChecks;
        return ValidateJointState(*specification, joints, checker);
    }

    void Fail(JointStateInvalidity invalidity)
    {
        result = MapJointStateInvalidity(invalidity);
        result.message = "JointPathPlanner: " + result.message;
        state = JointPathPlanningState::Failed;
    }

    void AdvanceOne()
    {
        if (endpoint < 2)
        {
            const JointVector& joints = endpoint++ == 0 ? start : goal;
            const auto invalidity = Check(joints);
            if (invalidity != JointStateInvalidity::None)
                Fail(invalidity);
            else if (endpoint == 2 && start == goal)
            {
                plan.points = {start};
                state = JointPathPlanningState::Completed;
            }
            return;
        }

        const auto invalidity = Check(Interpolate(start, goal,
            static_cast<double>(nextInterval) / static_cast<double>(intervalCount)));
        if (invalidity != JointStateInvalidity::None)
        {
            Fail(invalidity);
            return;
        }
        ++nextInterval;
        if (nextInterval > intervalCount)
        {
            plan.points = {start, goal};
            state = JointPathPlanningState::Completed;
        }
    }
};

JointPathPlanningJob::JointPathPlanningJob() : impl_(std::make_unique<Impl>()) {}
JointPathPlanningJob::~JointPathPlanningJob() = default;
JointPathPlanningJob::JointPathPlanningJob(JointPathPlanningJob&&) noexcept = default;
JointPathPlanningJob& JointPathPlanningJob::operator=(JointPathPlanningJob&&) noexcept = default;

Result JointPathPlanningJob::Begin(const models::RobotSpecification& specification,
    const JointVector& start, const JointVector& goal,
    const StateValidityChecker& stateValidityChecker, const JointPathPlannerOptions& options)
{
    auto& job = *impl_;
    job.specification = &specification;
    job.start = start;
    job.goal = goal;
    job.checker = stateValidityChecker;
    job.plan = {};
    job.result = Result::Success();
    job.ownerThread = std::this_thread::get_id();
    job.endpoint = 0;
    job.nextInterval = 1;
    job.state = JointPathPlanningState::Running;

    const double spacing = options.validationPolicy.jointCollisionSampleSpacingRadians;
    if (specification.joints == nullptr || specification.jointCount == 0 ||
        !std::isfinite(spacing) || spacing <= 0.0 ||
        options.validationPolicy.maximumPathIntervals == 0 ||
        start.size() != specification.jointCount || goal.size() != specification.jointCount)
    {
        job.result = {ErrorCode::InvalidCommand, "JointPathPlanner: invalid robot, endpoints, or sample spacing"};
        job.state = JointPathPlanningState::Failed;
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

    double maximumChange = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
        maximumChange = std::max(maximumChange, std::abs(goal[joint] - start[joint]));
    const double intervals = std::max(1.0, std::ceil(maximumChange / spacing));
    if (!std::isfinite(intervals) || intervals > static_cast<double>(
        options.validationPolicy.maximumPathIntervals))
    {
        job.result = {ErrorCode::InvalidCommand, "JointPathPlanner: path exceeds the sample limit"};
        job.state = JointPathPlanningState::Failed;
        return job.result;
    }
    job.intervalCount = static_cast<std::size_t>(intervals);
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
