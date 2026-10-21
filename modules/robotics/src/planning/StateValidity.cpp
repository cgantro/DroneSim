#include "robotics/planning/StateValidity.h"

#include <cmath>

namespace grasplink::robotics::planning
{
namespace
{
struct StateValidityMessages final
{
    inline static constexpr char invalidJointState[] =
        "StateValidity: invalid joint state in planned path";
    inline static constexpr char pathSamplingLimitExceeded[] =
        "StateValidity: joint collision path exceeds the sample limit";
    inline static constexpr char jointLimitExceeded[] =
        "StateValidity: planned state exceeds a joint limit";
    inline static constexpr char environmentOverlap[] =
        "StateValidity: planned state overlaps the environment";
    inline static constexpr char robotLinksOverlap[] =
        "StateValidity: planned robot links overlap";
    inline static constexpr char attachedObjectOverlap[] =
        "StateValidity: attached object overlaps the scene";
    inline static constexpr char unknownInvalidity[] =
        "StateValidity: unknown joint-state invalidity";
};
}

JointStateInvalidity ValidateJointState(
    const models::RobotSpecification& specification,
    const JointVector& joints,
    const StateValidityChecker& stateValidityChecker)
{
    if (specification.joints == nullptr || joints.size() != specification.jointCount)
        return JointStateInvalidity::JointCountMismatch;
    for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
    {
        if (!std::isfinite(joints[joint]))
            return JointStateInvalidity::NonFinitePosition;
        const auto& limits = specification.joints[joint];
        if (limits.HasPositionLimits() &&
            (joints[joint] < limits.minimumPosition || joints[joint] > limits.maximumPosition))
            return JointStateInvalidity::JointLimitViolation;
    }
    return stateValidityChecker ? stateValidityChecker(joints) : JointStateInvalidity::None;
}

Result MapJointStateInvalidity(JointStateInvalidity invalidity)
{
    switch (invalidity)
    {
    case JointStateInvalidity::None: return Result::Success();
    case JointStateInvalidity::JointCountMismatch:
    case JointStateInvalidity::NonFinitePosition:
        return {ErrorCode::InvalidCommand, StateValidityMessages::invalidJointState};
    case JointStateInvalidity::PathSamplingLimitExceeded:
        return {ErrorCode::InvalidCommand, StateValidityMessages::pathSamplingLimitExceeded};
    case JointStateInvalidity::JointLimitViolation:
        return {ErrorCode::JointLimitReached, StateValidityMessages::jointLimitExceeded};
    case JointStateInvalidity::EnvironmentCollision:
        return {ErrorCode::EnvironmentContact, StateValidityMessages::environmentOverlap};
    case JointStateInvalidity::SelfCollision:
        return {ErrorCode::SelfCollision, StateValidityMessages::robotLinksOverlap};
    case JointStateInvalidity::AttachedObjectCollision:
        return {ErrorCode::AttachedObjectCollision, StateValidityMessages::attachedObjectOverlap};
    }
    return {ErrorCode::Fault, StateValidityMessages::unknownInvalidity};
}

} // namespace grasplink::robotics::planning
