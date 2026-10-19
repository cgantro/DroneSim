#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/backends/simulation/detail/SimulationMotionPolicy.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"
#include "robotics/kinematics/detail/PoseMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace grasplink::robotics::backends::simulation
{
namespace planning = grasplink::robotics::planning;
namespace
{
constexpr double kPositionEpsilon = 1e-8;
constexpr double kJointVelocityRampSeconds = 0.20;
constexpr std::size_t kMaximumPoseIkSeeds = 16;
Result Failure(ErrorCode code, std::string message)
{
    // 잘못된 명령 입력은 예외로 던지지 않고 다른 Controller 구현과 같은 Result 오류 값으로 반환한다.
    return {code, std::move(message)};
}

bool IsScaleValid(double value)
{
    return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

Result ValidateJointPositionLimits(
    const models::RobotSpecification& specification,
    const JointVector& positions)
{
    for (std::size_t jointIndex = 0; jointIndex < specification.jointCount; ++jointIndex)
    {
        const auto& joint = specification.joints[jointIndex];
        const double position = positions[jointIndex];
        if (!std::isfinite(position) || position < joint.minPositionRadians ||
            position > joint.maxPositionRadians)
        {
            return Failure(ErrorCode::InvalidCommand,
                "SimRobotController: target outside joint limit: " + std::string(joint.name));
        }
    }
    return Result::Success();
}

void AppendValidityDiagnostic(Result& result, const std::function<std::string()>& provider)
{
    if (result.code == ErrorCode::SelfCollision && provider)
    {
        const std::string detail = provider();
        if (!detail.empty())
            result.message += " (" + detail + ")";
    }
}

bool JointInterpolationFollowsTcpLine(
    const JointVector& start,
    const JointVector& end,
    const kinematics::detail::Pose3& startTcp,
    const kinematics::detail::Pose3& endTcp,
    kinematics::DampedLeastSquaresIk& inverse)
{
    constexpr std::array<double, 3> kCheckFractions{0.25, 0.5, 0.75};
    JointVector sample(start.size());
    for (const double fraction : kCheckFractions)
    {
        for (std::size_t joint = 0; joint < sample.size(); ++joint)
            sample[joint] = start[joint] + (end[joint] - start[joint]) * fraction;
        const auto actual = kinematics::detail::FromCartesian(inverse.EvaluateTcp(sample));
        const auto expected = kinematics::detail::Interpolate(startTcp, endTcp, fraction);
        if (kinematics::detail::Length(kinematics::detail::Subtract(
                actual.positionMeters, expected.positionMeters)) >
                planning::kDefaultPlanningPolicy.maximumLinearTcpErrorMeters ||
            kinematics::detail::Length(kinematics::detail::RotationError(
                actual.rotation, expected.rotation)) >
                planning::kDefaultPlanningPolicy.maximumAngularTcpErrorRadians)
            return false;
    }
    return true;
}

bool SameLinearPathCommand(const LinearPathMoveCommand& lhs, const LinearPathMoveCommand& rhs)
{
    if (lhs.targetPoses.size() != rhs.targetPoses.size() ||
        lhs.maxLinearVelocityMetersPerSecond != rhs.maxLinearVelocityMetersPerSecond ||
        lhs.maxAngularVelocityRadiansPerSecond != rhs.maxAngularVelocityRadiansPerSecond ||
        lhs.maxLinearAccelerationMetersPerSecondSquared != rhs.maxLinearAccelerationMetersPerSecondSquared ||
        lhs.maxAngularAccelerationRadiansPerSecondSquared != rhs.maxAngularAccelerationRadiansPerSecondSquared)
        return false;

    for (std::size_t index = 0; index < lhs.targetPoses.size(); ++index)
    {
        if (lhs.targetPoses[index].positionMeters != rhs.targetPoses[index].positionMeters ||
            lhs.targetPoses[index].orientationXyzw != rhs.targetPoses[index].orientationXyzw)
            return false;
    }
    return true;
}

bool SameJointPosition(const JointVector& lhs, const JointVector& rhs)
{
    if (lhs.size() != rhs.size())
        return false;
    for (std::size_t index = 0; index < lhs.size(); ++index)
    {
        if (std::abs(lhs[index] - rhs[index]) > 1e-8)
            return false;
    }
    return true;
}

kinematics::IkOptions RuntimeLinearIkOptions()
{
    kinematics::IkOptions options = planning::PathIkOptions();
    // 특이 자세에서 관절을 재배치하는 보조 IK에 완화한 오차를 사용해 반복 횟수를 제한한다.
    options.positionToleranceMeters = 1e-6;
    options.orientationToleranceRadians = 1e-5;
    return options;
}

double TrapezoidDistanceFraction(double timeFraction, double rampTimeFraction)
{
    // 프로파일은 시간의 앞·뒤 rampTimeFraction 비율에서 속도를 선형으로 올리고 내리며, 가운데는 최대 속도를 유지한다.
    // 속도를 적분한 뒤 전체 면적 1-ramp와 비교해 경로 전체가 정확히 0부터 1까지 진행하도록 정규화한다.
    const double ramp = std::clamp(rampTimeFraction, 1e-6, 0.49);
    const double u = std::clamp(timeFraction, 0.0, 1.0);
    double integratedSpeed = 0.0;
    if (u < ramp)
        integratedSpeed = u * u / (2.0 * ramp);
    else if (u <= 1.0 - ramp)
        integratedSpeed = ramp * 0.5 + (u - ramp);
    else
    {
        const double decelerationStart = 1.0 - ramp;
        integratedSpeed = ramp * 0.5 + (1.0 - 2.0 * ramp) +
            ((u - u * u * 0.5) - (decelerationStart - decelerationStart * decelerationStart * 0.5)) / ramp;
    }
    return std::clamp(integratedSpeed / (1.0 - ramp), 0.0, 1.0);
}
} // namespace

SimRobotController::SimRobotController(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame)
    : specification_(&specification), inverse_(specification, tcpInToolFrame)
{
    if (specification_->joints == nullptr || specification_->jointCount == 0)
        throw std::invalid_argument("SimRobotController: empty robot specification");
    // 상태는 q=0에서 시작하므로 모든 관절 범위가 0을 포함하고 제한 속도가 양수여야 한다.
    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        const auto& joint = specification_->joints[i];
        // 초기 상태는 q=0. 이를 허용하지 않는 모델은 별도 초기화 계약이 필요하다.
        if (!std::isfinite(joint.minPositionRadians) || !std::isfinite(joint.maxPositionRadians) ||
            joint.minPositionRadians > 0.0 || joint.maxPositionRadians < 0.0 ||
            !std::isfinite(joint.maxVelocityRadiansPerSecond) || joint.maxVelocityRadiansPerSecond <= 0.0)
            throw std::invalid_argument("SimRobotController: invalid limits, zero pose or maximum velocity");
    }
}

void SimRobotController::SetJointStateValidityChecker(
    planning::StateValidityChecker checker)
{
    if (IsMotionPlanning())
    {
        linearPathPlanningJob_.Cancel();
        if (posePlanningPhase_ != PosePlanningPhase::None)
        {
            inverse_.CancelSingleSeed(poseIkSession_);
            poseJointPathPlanningJob_.Cancel();
        }
        posePlanningPhase_ = PosePlanningPhase::None;
        state_.mode = RobotMode::Idle;
        state_.errorCode = ErrorCode::Cancelled;
        linearPathPlanningResult_.emplace(
            Failure(ErrorCode::Cancelled, "SimRobotController: motion planning cancelled because the validity checker changed"));
        pendingLinearPathCommand_ = {};
    }
    linearPathPlanningJob_ = {};
    poseJointPathPlanningJob_.Cancel();
    posePlanningPhase_ = PosePlanningPhase::None;
    posePathPoints_.clear();
    poseAlternativeSeeds_.clear();
    poseContinuationCommand_.reset();
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    jointStateValidityChecker_ = std::move(checker);
}

void SimRobotController::SetJointStateValidityDiagnosticProvider(std::function<std::string()> provider)
{
    jointStateValidityDiagnosticProvider_ = std::move(provider);
}

Result SimRobotController::Connect()
{
    // 재연결은 이전 명령과 상태를 이어가지 않고 q=0의 새 논리 세션을 만든다.
    linearPathPlanningJob_.Cancel();
    linearPathPlanningJob_ = {};
    if (posePlanningPhase_ != PosePlanningPhase::None)
    {
        inverse_.CancelSingleSeed(poseIkSession_);
        poseJointPathPlanningJob_.Cancel();
    }
    posePlanningPhase_ = PosePlanningPhase::None;
    posePathPoints_.clear();
    poseAlternativeSeeds_.clear();
    poseContinuationCommand_.reset();
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    linearPathPlanningResult_.reset();
    pendingLinearPathCommand_ = {};
    state_ = {};
    state_.jointPositionRadians.assign(specification_->jointCount, 0.0);
    state_.jointVelocityRadiansPerSecond.assign(specification_->jointCount, 0.0);
    state_.mode = RobotMode::Idle;
    state_.valid = true;
    RefreshTcp();

    targetPositionRadians_ = state_.jointPositionRadians;
    jointMoveStartPositionRadians_ = state_.jointPositionRadians;
    jointMoveProgress_ = 0.0;
    jointMoveProgressVelocity_ = 0.0;
    jointMoveProgressAcceleration_ = 0.0;
    jointMoveProgressRate_ = 0.0;
    jointMoveElapsedSeconds_ = 0.0;
    jointMoveAccelerationTimeSeconds_ = 0.0;
    jointMoveCruiseTimeSeconds_ = 0.0;
    jointMoveProfileDurationSeconds_ = 0.0;
    jointMovePeakProgressVelocity_ = 0.0;
    jointMoveBraking_ = false;
    hasPendingJointTarget_ = false;
    linearInterpolationBuffer_.resize(specification_->jointCount);
    connected_ = true;
    linearPath_.clear();
    return Result::Success();
}

void SimRobotController::Disconnect() noexcept
{
    linearPathPlanningJob_.Cancel();
    if (posePlanningPhase_ != PosePlanningPhase::None)
    {
        inverse_.CancelSingleSeed(poseIkSession_);
        poseJointPathPlanningJob_.Cancel();
    }
    posePlanningPhase_ = PosePlanningPhase::None;
    posePathPoints_.clear();
    poseAlternativeSeeds_.clear();
    poseContinuationCommand_.reset();
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    linearPathPlanningResult_.reset();
    posePathSegment_ = 0;
    pendingLinearPathCommand_ = {};
    // 백엔드의 관절 배열은 재사용하되 연결/feedback 유효성을 내리고 속도만 즉시 0으로 만든다.
    connected_ = false;
    state_.mode = RobotMode::Disconnected;
    state_.valid = false;
    state_.tcpPoseValid = false;
    linearPath_.clear();
    std::fill(
        state_.jointVelocityRadiansPerSecond.begin(),
        state_.jointVelocityRadiansPerSecond.end(),
        0.0);
}

bool SimRobotController::IsConnected() const noexcept
{
    return connected_;
}

Result SimRobotController::MoveJoint(const JointMoveCommand& command)
{
    return MoveJointImpl(command, JointMoveOrigin::ExternalJointCommand);
}

Result SimRobotController::MoveJointImpl(const JointMoveCommand& command, JointMoveOrigin origin)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");
    if (IsMotionPlanning())
        return Failure(ErrorCode::Busy, "SimRobotController: linear path planning is in progress");

    if (command.targetPositionRadians.size() != specification_->jointCount)
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: joint count mismatch");

    if (!IsScaleValid(command.velocityScale) || !IsScaleValid(command.accelerationScale))
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: scale must be in (0, 1]");

    const Result validation = ValidateJointPositionLimits(*specification_, command.targetPositionRadians);
    if (!validation)
        return validation;

    // 모든 joint를 먼저 확인한다. 하나라도 범위 밖이면 target이나 기존 진행 상태를 일부만 바꾸지 않는다.
    // 명령을 받는 순간 관절각 q를 바꾸지 않는다. 이후 Update 호출이 정해진 속도 안에서 현재 상태를 목표까지 진행시킨다.
    // 움직이는 도중 새 명령이 들어오면 대기열에 쌓지 않고 현재 목표를 새 목표로 바꾼다.
    JointVector targetPositionRadians = command.targetPositionRadians;
    if (!command.preserveJointTurns)
        planning::AlignEquivalentJointAngles(targetPositionRadians, state_.jointPositionRadians, *specification_);

    const bool canBrakeJointPath = state_.mode == RobotMode::Moving && linearPath_.empty() &&
        jointMoveProgressVelocity_ > kPositionEpsilon && jointMoveProgressAcceleration_ > 0.0;
    const double stoppingProgress = canBrakeJointPath ? std::min(1.0, jointMoveProgress_ +
        jointMoveProgressVelocity_ * jointMoveProgressVelocity_ /
            (2.0 * jointMoveProgressAcceleration_)) : jointMoveProgress_;
    const JointVector* nextPathStart = &state_.jointPositionRadians;
    JointVector brakingStart;
    if (canBrakeJointPath)
    {
        brakingStart = state_.jointPositionRadians;
        for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
            brakingStart[joint] = jointMoveStartPositionRadians_[joint] +
                (targetPositionRadians_[joint] - jointMoveStartPositionRadians_[joint]) * stoppingProgress;
        nextPathStart = &brakingStart;
    }

    if ((origin == JointMoveOrigin::ExternalJointCommand || canBrakeJointPath) && jointStateValidityChecker_)
    {
        const auto invalidity = planning::ValidateJointPath(*nextPathStart, targetPositionRadians,
            *specification_, jointStateValidityChecker_);
        if (invalidity != planning::JointStateInvalidity::None)
        {
            Result result = planning::MapJointStateInvalidity(invalidity);
            AppendValidityDiagnostic(result, jointStateValidityDiagnosticProvider_);
            return result;
        }
    }

    if (origin != JointMoveOrigin::PlannedPoseSegment)
    {
        poseContinuationCommand_.reset();
        poseContinuationPlan_.reset();
        posePathPoints_.clear();
        posePathSegment_ = 0;
    }

    if (canBrakeJointPath)
    {
        pendingJointTargetRadians_ = std::move(targetPositionRadians);
        pendingVelocityScale_ = command.velocityScale;
        pendingAccelerationScale_ = command.accelerationScale;
        hasPendingJointTarget_ = true;
        jointMoveBraking_ = true;
        jointMoveStopProgress_ = stoppingProgress;
        state_.errorCode = ErrorCode::None;
        return Result::Success();
    }

    ConfigureJointMove(std::move(targetPositionRadians), command.velocityScale, command.accelerationScale);
    state_.errorCode = ErrorCode::None;
    return Result::Success();
}

void SimRobotController::ConfigureJointMove(
    JointVector targetPositionRadians, double velocityScale, double accelerationScale)
{
    targetPositionRadians_ = std::move(targetPositionRadians);
    jointMoveStartPositionRadians_ = state_.jointPositionRadians;
    jointMoveProgress_ = 0.0;
    jointMoveProgressVelocity_ = 0.0;
    jointMoveProgressAcceleration_ = 0.0;
    jointMoveElapsedSeconds_ = 0.0;
    jointMoveAccelerationTimeSeconds_ = 0.0;
    jointMoveCruiseTimeSeconds_ = 0.0;
    jointMoveProfileDurationSeconds_ = 0.0;
    jointMovePeakProgressVelocity_ = 0.0;
    jointMoveBraking_ = false;
    hasPendingJointTarget_ = false;
    pendingJointTargetRadians_.clear();
    linearPath_.clear();

    jointMoveProgressRate_ = std::numeric_limits<double>::infinity();
    bool needsMotion = false;
    for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
    {
        const double distance = std::abs(targetPositionRadians_[joint] - jointMoveStartPositionRadians_[joint]);
        if (distance <= kPositionEpsilon)
            continue;
        needsMotion = true;
        const auto& limits = specification_->joints[joint];
        jointMoveProgressRate_ = std::min(jointMoveProgressRate_,
            limits.maxVelocityRadiansPerSecond * velocityScale / distance);
        const double acceleration = limits.maxVelocityRadiansPerSecond /
            kJointVelocityRampSeconds * accelerationScale;
        jointMoveProgressAcceleration_ = std::min(jointMoveProgressAcceleration_ == 0.0 ?
            std::numeric_limits<double>::infinity() : jointMoveProgressAcceleration_, acceleration / distance);
    }

    state_.mode = needsMotion ? RobotMode::Moving : RobotMode::Idle;
    if (!needsMotion)
    {
        jointMoveProgressRate_ = 0.0;
        jointMoveProgressAcceleration_ = 0.0;
        state_.jointPositionRadians = targetPositionRadians_;
        std::fill(state_.jointVelocityRadiansPerSecond.begin(), state_.jointVelocityRadiansPerSecond.end(), 0.0);
        RefreshTcp();
    }
    else
    {
        const double accelerationTimeToMaximum = jointMoveProgressRate_ /
            jointMoveProgressAcceleration_;
        const double accelerationDistanceToMaximum = 0.5 * jointMoveProgressAcceleration_ *
            accelerationTimeToMaximum * accelerationTimeToMaximum;
        if (2.0 * accelerationDistanceToMaximum >= 1.0)
        {
            jointMoveAccelerationTimeSeconds_ = std::sqrt(1.0 / jointMoveProgressAcceleration_);
            jointMovePeakProgressVelocity_ = jointMoveProgressAcceleration_ *
                jointMoveAccelerationTimeSeconds_;
            jointMoveCruiseTimeSeconds_ = 0.0;
        }
        else
        {
            jointMoveAccelerationTimeSeconds_ = accelerationTimeToMaximum;
            jointMovePeakProgressVelocity_ = jointMoveProgressRate_;
            jointMoveCruiseTimeSeconds_ = (1.0 - 2.0 * accelerationDistanceToMaximum) /
                jointMovePeakProgressVelocity_;
        }
        jointMoveProfileDurationSeconds_ = 2.0 * jointMoveAccelerationTimeSeconds_ +
            jointMoveCruiseTimeSeconds_;
    }
}

Result SimRobotController::MovePose(const CartesianPose& targetInBase, double velocityScale, double accelerationScale)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");
    if (IsMotionPlanning())
        return Failure(ErrorCode::Busy, "SimRobotController: linear path planning is in progress");
    if (!IsScaleValid(velocityScale) || !IsScaleValid(accelerationScale))
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: scale must be in (0, 1]");
    planning::JointStateInvalidity invalidity = planning::JointStateInvalidity::None;
    kinematics::IkResult ikFailure;
    auto solution = planning::SolveCollisionFreeIk(inverse_, *specification_, jointStateValidityChecker_,
        targetInBase, state_.jointPositionRadians, {}, planning::kDefaultPlanningPolicy,
        invalidity, ikFailure);
    if (!solution)
    {
        if (invalidity == planning::JointStateInvalidity::EnvironmentCollision)
            return Failure(ErrorCode::EnvironmentContact, "SimRobotController: no collision-free IK solution or joint path");
        if (invalidity != planning::JointStateInvalidity::None)
        {
            Result result = planning::MapJointStateInvalidity(invalidity);
            AppendValidityDiagnostic(result, jointStateValidityDiagnosticProvider_);
            return result;
        }
        return planning::MapIkFailure(ikFailure);
    }
    return MoveJointImpl({solution->jointPositionRadians, velocityScale, accelerationScale},
        JointMoveOrigin::PoseTargetCommand);
}

Result SimRobotController::BeginPosePlanning(
    const CartesianPose& targetInBase, double velocityScale, double accelerationScale)
{
    return BeginPosePlanningImpl(targetInBase, std::nullopt, velocityScale, accelerationScale);
}

Result SimRobotController::BeginPosePlanningWithLinearContinuation(
    const CartesianPose& approach, const LinearPathMoveCommand& continuation,
    double velocityScale, double accelerationScale)
{
    return BeginPosePlanningImpl(approach, continuation, velocityScale, accelerationScale);
}

Result SimRobotController::BeginPosePlanningImpl(
    const CartesianPose& targetInBase, std::optional<LinearPathMoveCommand> continuation,
    double velocityScale, double accelerationScale)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");
    if (IsMotionPlanning())
        return Failure(ErrorCode::Busy, "SimRobotController: motion planning is in progress");
    if (!IsScaleValid(velocityScale) || !IsScaleValid(accelerationScale))
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: scale must be in (0, 1]");
    if (continuation)
    {
        const Result validation = planning::ValidateLinearPathCommand(*continuation);
        if (!validation)
            return validation;
    }
    if (!specification_->hasToolFrame)
        return Failure(ErrorCode::Unsupported, "SimRobotController: missing ToolFrame for TCP motion");

    linearPathPlanningResult_.reset();
    posePathPoints_.clear();
    poseContinuationCommand_ = std::move(continuation);
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    poseAlternativeSeeds_ = kinematics::detail::BuildAlternativeIkSeeds(
        state_.jointPositionRadians, *specification_);
    const std::size_t maximumSeeds = poseContinuationCommand_ ? kMaximumPoseIkSeeds : 64;
    kinematics::detail::AppendIkRestartSeeds(poseAlternativeSeeds_,
        state_.jointPositionRadians, *specification_, maximumSeeds - 1);
    poseAlternativeSeedIndex_ = 0;
    poseIkSeedAttempts_ = 1;
    posePathSegment_ = 0;
    poseTarget_ = targetInBase;
    posePathFailure_ = Result::Success();
    pendingPoseMove_ = {{}, velocityScale, accelerationScale};
    poseIkSession_ = inverse_.BeginSingleSeed(targetInBase, state_.jointPositionRadians);
    posePlanningPhase_ = PosePlanningPhase::Ik;
    state_.mode = RobotMode::Planning;
    state_.errorCode = ErrorCode::None;
    return Result::Success();
}

bool SimRobotController::StartNextPoseIkSeed()
{
    if (poseAlternativeSeedIndex_ >= poseAlternativeSeeds_.size())
        return false;
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    poseIkSession_ = inverse_.BeginSingleSeed(poseTarget_,
        poseAlternativeSeeds_[poseAlternativeSeedIndex_++]);
    ++poseIkSeedAttempts_;
    posePlanningPhase_ = PosePlanningPhase::Ik;
    return true;
}

bool SimRobotController::StartPoseCandidateJointPath()
{
    planning::JointPathPlannerOptions options;
    options.validationPolicy = planning::kDefaultPlanningPolicy;
    options.maximumIterations = 512;
    options.maximumNodesPerTree = 256;
    options.maximumShortcutAttempts = 32;
    const Result begin = poseJointPathPlanningJob_.Begin(*specification_,
        state_.jointPositionRadians, poseCandidateJoints_, jointStateValidityChecker_, options);
    if (!begin)
    {
        posePathFailure_ = begin;
        return false;
    }
    posePlanningPhase_ = PosePlanningPhase::JointPath;
    return true;
}

std::size_t SimRobotController::GetLastPoseIkSeedAttemptCount() const noexcept
{
    return poseIkSeedAttempts_;
}

Result SimRobotController::MoveLinear(const LinearMoveCommand& command)
{
    LinearPathMoveCommand path;
    path.targetPoses.push_back(command.targetPose);
    path.maxLinearVelocityMetersPerSecond = command.maxLinearVelocityMetersPerSecond;
    path.maxAngularVelocityRadiansPerSecond = command.maxAngularVelocityRadiansPerSecond;
    path.maxLinearAccelerationMetersPerSecondSquared = command.maxLinearAccelerationMetersPerSecondSquared;
    path.maxAngularAccelerationRadiansPerSecondSquared = command.maxAngularAccelerationRadiansPerSecondSquared;
    return MoveLinearPath(path);
}

Result SimRobotController::MoveLinearPath(const LinearPathMoveCommand& command)
{
    const Result validation = ValidateLinearPathRequest(command);
    if (!validation)
        return validation;
    if (const auto cached = CommitCachedLinearContinuation(command))
        return *cached;

    planning::LinearPathPlan plan;
    const auto result = planning::BuildLinearPath(
        command, *specification_, state_.jointPositionRadians,
        inverse_.EvaluateTcp(state_.jointPositionRadians), inverse_, jointStateValidityChecker_, plan,
        planning::kDefaultPlanningPolicy, jointStateValidityDiagnosticProvider_);
    if (!result)
        return result;
    return CommitLinearPathPlan(command, std::move(plan));
}

Result SimRobotController::BeginLinearPathPlanning(const LinearPathMoveCommand& command)
{
    const Result validation = ValidateLinearPathRequest(command);
    if (!validation)
        return validation;
    if (const auto cached = CommitCachedLinearContinuation(command))
    {
        linearPathPlanningResult_ = *cached;
        return *cached;
    }

    pendingLinearPathCommand_ = command;
    linearPathPlanningResult_.reset();
    const Result begin = linearPathPlanningJob_.Begin(command, *specification_,
        state_.jointPositionRadians, inverse_.EvaluateTcp(state_.jointPositionRadians),
        inverse_, jointStateValidityChecker_, planning::kInteractivePlanningPolicy,
        jointStateValidityDiagnosticProvider_);
    if (!begin)
    {
        linearPathPlanningResult_ = begin;
        pendingLinearPathCommand_ = {};
        linearPathPlanningJob_ = {};
        return begin;
    }
    if (linearPathPlanningJob_.GetState() == planning::LinearPathPlanningState::Completed)
    {
        auto plan = linearPathPlanningJob_.TakePlan();
        const Result committed = plan ? CommitLinearPathPlan(command, std::move(*plan)) :
            Failure(ErrorCode::Fault, "SimRobotController: completed planner returned no plan");
        linearPathPlanningResult_ = committed;
        pendingLinearPathCommand_ = {};
        return committed;
    }
    state_.mode = RobotMode::Planning;
    state_.errorCode = ErrorCode::None;
    return Result::Success();
}

void SimRobotController::AdvanceMotionPlanning(std::size_t workBudget)
{
    if (!IsMotionPlanning())
        return;

    if (posePlanningPhase_ == PosePlanningPhase::Ik)
    {
        const auto ikState = inverse_.StepSingleSeed(poseIkSession_, workBudget);
        if (ikState == kinematics::IkSessionState::Running)
            return;

        const auto& ikResult = inverse_.GetSingleSeedResult(poseIkSession_);
        if (!ikResult)
        {
            if (StartNextPoseIkSeed())
                return;
            FinishPosePlanning(planning::MapIkFailure(ikResult));
            return;
        }

        poseCandidateJoints_ = ikResult.jointPositionRadians;
        if (poseContinuationCommand_)
        {
            const auto invalidity = jointStateValidityChecker_ ?
                jointStateValidityChecker_(poseCandidateJoints_) : planning::JointStateInvalidity::None;
            if (invalidity != planning::JointStateInvalidity::None)
            {
                posePathFailure_ = planning::MapJointStateInvalidity(invalidity);
                if (StartNextPoseIkSeed())
                    return;
                FinishPosePlanning(posePathFailure_);
                return;
            }

            const Result continuationBegin = linearPathPlanningJob_.Begin(
                *poseContinuationCommand_, *specification_, poseCandidateJoints_,
                inverse_.EvaluateTcp(poseCandidateJoints_), inverse_, jointStateValidityChecker_,
                planning::kInteractivePlanningPolicy, jointStateValidityDiagnosticProvider_);
            if (!continuationBegin)
            {
                posePathFailure_ = continuationBegin;
                if (StartNextPoseIkSeed())
                    return;
                FinishPosePlanning(posePathFailure_);
                return;
            }

            posePlanningPhase_ = PosePlanningPhase::LinearContinuation;
            return;
        }

        if (!StartPoseCandidateJointPath())
        {
            if (StartNextPoseIkSeed())
                return;
            FinishPosePlanning(posePathFailure_);
        }
        return;
    }

    if (posePlanningPhase_ == PosePlanningPhase::LinearContinuation)
    {
        const auto planningState = linearPathPlanningJob_.Advance(workBudget);
        if (planningState == planning::LinearPathPlanningState::Running)
            return;
        if (planningState != planning::LinearPathPlanningState::Completed)
        {
            posePathFailure_ = linearPathPlanningJob_.GetResult();
            linearPathPlanningJob_ = {};
            if (StartNextPoseIkSeed())
                return;
            FinishPosePlanning(posePathFailure_);
            return;
        }

        poseContinuationPlan_ = linearPathPlanningJob_.TakePlan();
        linearPathPlanningJob_ = {};
        if (!poseContinuationPlan_)
        {
            posePathFailure_ = Failure(ErrorCode::Fault,
                "SimRobotController: completed continuation planner returned no plan");
            if (StartNextPoseIkSeed())
                return;
            FinishPosePlanning(posePathFailure_);
            return;
        }
        if (!StartPoseCandidateJointPath())
        {
            if (StartNextPoseIkSeed())
                return;
            FinishPosePlanning(posePathFailure_);
        }
        return;
    }

    if (posePlanningPhase_ == PosePlanningPhase::JointPath)
    {
        const auto planningState = poseJointPathPlanningJob_.Advance(workBudget);
        if (planningState == planning::JointPathPlanningState::Running)
            return;
        if (planningState != planning::JointPathPlanningState::Completed)
        {
            posePathFailure_ = poseJointPathPlanningJob_.GetResult();
            if (StartNextPoseIkSeed())
                return;
            FinishPosePlanning(posePathFailure_);
            return;
        }
        auto plan = poseJointPathPlanningJob_.TakePlan();
        if (!plan)
        {
            FinishPosePlanning(Failure(ErrorCode::Fault,
                "SimRobotController: joint path planner completed without a path"));
            return;
        }
        posePathPoints_ = std::move(plan->points);
        poseAlternativeSeeds_.clear();
        posePathSegment_ = 0;
        posePlanningPhase_ = PosePlanningPhase::None;
        if (!poseContinuationPlan_)
            poseContinuationCommand_.reset();
        const Result execution = StartPosePathSegment();
        if (!execution)
        {
            poseContinuationPlan_.reset();
            poseContinuationCommand_.reset();
        }
        FinishPosePlanning(execution);
        return;
    }

    const auto planningState = linearPathPlanningJob_.Advance(workBudget);
    if (planningState == planning::LinearPathPlanningState::Running)
        return;
    if (planningState == planning::LinearPathPlanningState::Completed)
    {
        auto plan = linearPathPlanningJob_.TakePlan();
        linearPathPlanningResult_ = plan ?
            CommitLinearPathPlan(pendingLinearPathCommand_, std::move(*plan)) :
            Failure(ErrorCode::Fault, "SimRobotController: completed planner returned no plan");
        pendingLinearPathCommand_ = {};
    }
    else
    {
        linearPathPlanningResult_ = linearPathPlanningJob_.GetResult();
        state_.mode = RobotMode::Idle;
        state_.errorCode = linearPathPlanningResult_->code;
        pendingLinearPathCommand_ = {};
        linearPathPlanningJob_ = {};
    }
}

bool SimRobotController::IsMotionPlanning() const noexcept
{
    return posePlanningPhase_ != PosePlanningPhase::None ||
        linearPathPlanningJob_.GetState() == planning::LinearPathPlanningState::Running;
}

void SimRobotController::FinishPosePlanning(Result result)
{
    posePlanningPhase_ = PosePlanningPhase::None;
    poseAlternativeSeeds_.clear();
    if (!result && poseIkSeedAttempts_ > 1)
        result.message += " after " + std::to_string(poseIkSeedAttempts_) + " IK seed attempts";
    linearPathPlanningResult_ = std::move(result);
    state_.mode = *linearPathPlanningResult_ ? RobotMode::Moving : RobotMode::Idle;
    state_.errorCode = linearPathPlanningResult_->code;
    if (!*linearPathPlanningResult_)
    {
        posePathPoints_.clear();
        poseContinuationCommand_.reset();
        poseContinuationPlan_.reset();
        poseCandidateJoints_.clear();
        hasPendingJointTarget_ = false;
        pendingJointTargetRadians_.clear();
    }
}

Result SimRobotController::StartPosePathSegment()
{
    if (posePathPoints_.empty())
        return Failure(ErrorCode::Fault, "SimRobotController: joint path contains no start state");
    if (posePathPoints_.size() == 1)
    {
        posePathPoints_.clear();
        return MoveJointImpl({state_.jointPositionRadians, pendingPoseMove_.velocityScale,
            pendingPoseMove_.accelerationScale}, JointMoveOrigin::PlannedPoseSegment);
    }
    posePathSegment_ = 1;
    return MoveJointImpl({posePathPoints_[posePathSegment_], pendingPoseMove_.velocityScale,
        pendingPoseMove_.accelerationScale}, JointMoveOrigin::PlannedPoseSegment);
}

std::optional<Result> SimRobotController::TakeMotionPlanningResult()
{
    auto result = std::move(linearPathPlanningResult_);
    linearPathPlanningResult_.reset();
    return result;
}

Result SimRobotController::ValidateLinearPathRequest(
    const LinearPathMoveCommand& command) const
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");
    if (IsMotionPlanning())
        return Failure(ErrorCode::Busy, "SimRobotController: linear path planning is in progress");
    const Result validation = planning::ValidateLinearPathCommand(command);
    if (!validation)
        return validation;
    if (!specification_->hasToolFrame)
        return Failure(ErrorCode::Unsupported, "SimRobotController: missing ToolFrame for TCP motion");
    return Result::Success();
}

Result SimRobotController::CommitLinearPathPlan(
    const LinearPathMoveCommand& command, planning::LinearPathPlan plan)
{
    if (!plan.hasMotion)
        return MoveJoint({state_.jointPositionRadians, 1.0, 1.0});

    auto& candidate = plan.points;
    const double plannedLinearVelocity = plan.plannedLinearVelocity;
    const double plannedAngularVelocity = plan.plannedAngularVelocity;
    linearVelocityLimit_ = command.maxLinearVelocityMetersPerSecond;
    angularVelocityLimit_ = command.maxAngularVelocityRadiansPerSecond;
    linearAccelerationLimit_ = command.maxLinearAccelerationMetersPerSecondSquared;
    angularAccelerationLimit_ = command.maxAngularAccelerationRadiansPerSecondSquared;
    linearPlannedDurationSeconds_ = 0.0;
    for (std::size_t i = 1; i < candidate.size(); ++i)
        linearPlannedDurationSeconds_ += candidate[i].durationSeconds;
    // 모든 직선 구간의 계획 시간을 합쳐 가속·감속을 한 번만 적용한다. 웨이포인트마다 명령을 다시 내리면 매 구간마다 멈췄다가 재가속한다.
    const double accelerationTime = std::max(
        plannedLinearVelocity / linearAccelerationLimit_,
        plannedAngularVelocity / angularAccelerationLimit_);
    linearProfileRampFraction_ = accelerationTime /
        (linearPlannedDurationSeconds_ + accelerationTime);
    linearProfileRampFraction_ = std::clamp(linearProfileRampFraction_, 0.005, 0.45);
    linearProfileDurationSeconds_ = linearPlannedDurationSeconds_ / (1.0 - linearProfileRampFraction_);
    linearProfileElapsedSeconds_ = 0.0;
    targetPositionRadians_ = candidate.back().joints;
    posePathPoints_.clear();
    posePathSegment_ = 0;
    poseContinuationCommand_.reset();
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    linearPath_ = std::move(candidate);
    linearSegment_ = 1;
    linearSegmentFraction_ = 0.0;
    state_.errorCode = ErrorCode::None;
    state_.mode = RobotMode::Moving;
    return Result::Success();
}

std::optional<Result> SimRobotController::CommitCachedLinearContinuation(
    const LinearPathMoveCommand& command)
{
    if (!poseContinuationPlan_ || !poseContinuationCommand_)
        return std::nullopt;

    auto plan = std::move(*poseContinuationPlan_);
    const bool matches = SameLinearPathCommand(*poseContinuationCommand_, command) &&
        !plan.points.empty() && SameJointPosition(state_.jointPositionRadians, plan.points.front().joints);
    poseContinuationPlan_.reset();
    poseContinuationCommand_.reset();
    if (!matches)
        return std::nullopt;
    return CommitLinearPathPlan(command, std::move(plan));
}

void SimRobotController::RefreshTcp()
{
    state_.tcpPoseValid = specification_->hasToolFrame && state_.valid;
    if (state_.tcpPoseValid)
        state_.tcpPose = inverse_.EvaluateTcp(state_.jointPositionRadians);
    if (posePlanningPhase_ != PosePlanningPhase::None)
    {
        inverse_.CancelSingleSeed(poseIkSession_);
        FinishPosePlanning(Failure(ErrorCode::Cancelled,
            "SimRobotController: pose planning cancelled because the robot state changed"));
    }
}

Result SimRobotController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    if (IsMotionPlanning())
    {
        linearPathPlanningJob_.Cancel();
        if (posePlanningPhase_ != PosePlanningPhase::None)
        {
            inverse_.CancelSingleSeed(poseIkSession_);
            poseJointPathPlanningJob_.Cancel();
            FinishPosePlanning(Failure(ErrorCode::Cancelled,
                "SimRobotController: pose planning cancelled"));
        }
        else
        {
            linearPathPlanningResult_ = Failure(
                ErrorCode::Cancelled, "SimRobotController: linear path planning cancelled");
            state_.mode = RobotMode::Idle;
            state_.errorCode = ErrorCode::Cancelled;
            pendingLinearPathCommand_ = {};
            linearPathPlanningJob_ = {};
        }
    }

    // 현재 q를 새 목표로 고정해 이후 Update가 남은 동작을 재개하지 않게 한다.
    targetPositionRadians_ = state_.jointPositionRadians;
    jointMoveStartPositionRadians_ = state_.jointPositionRadians;
    jointMoveProgress_ = 0.0;
    jointMoveProgressVelocity_ = 0.0;
    jointMoveProgressAcceleration_ = 0.0;
    jointMoveProgressRate_ = 0.0;
    jointMoveBraking_ = false;
    hasPendingJointTarget_ = false;
    linearPath_.clear();
    posePathPoints_.clear();
    posePathSegment_ = 0;
    poseContinuationCommand_.reset();
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    linearProfileElapsedSeconds_ = 0.0;
    std::fill(
        state_.jointVelocityRadiansPerSecond.begin(),
        state_.jointVelocityRadiansPerSecond.end(),
        0.0);
    state_.mode = RobotMode::Stopped;
    return Result::Success();
}

RobotState SimRobotController::GetState() const
{
    // vector까지 값으로 복사하므로 호출자는 다음 Update와 무관한 snapshot을 받는다.
    return state_;
}

const RobotState& SimRobotController::GetStateView() const noexcept
{
    return state_;
}

void SimRobotController::Update(double dtSeconds)
{
    // FixedControlLoop 같은 호출자가 정한 tick마다 q/dq와 모델 FK의 TCP를 갱신한다. 화면 Entity와 물리 Body 반영은 별도 계층의 책임이다.
    if (!connected_ || state_.mode != RobotMode::Moving ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        return;
    }

    if (!linearPath_.empty())
    {
        UpdateLinear(dtSeconds);
        return;
    }

    double nextProgress = jointMoveProgress_;
    double nextVelocity = 0.0;
    if (jointMoveBraking_)
    {
        nextVelocity = std::max(0.0,
            jointMoveProgressVelocity_ - jointMoveProgressAcceleration_ * dtSeconds);
        nextProgress += (jointMoveProgressVelocity_ * jointMoveProgressVelocity_ -
            nextVelocity * nextVelocity) / (2.0 * jointMoveProgressAcceleration_);
        if (nextProgress >= jointMoveStopProgress_ || nextVelocity == 0.0)
        {
            nextProgress = jointMoveStopProgress_;
            nextVelocity = 0.0;
        }
    }
    else
    {
        jointMoveElapsedSeconds_ = std::min(jointMoveProfileDurationSeconds_,
            jointMoveElapsedSeconds_ + dtSeconds);
        const double elapsed = jointMoveElapsedSeconds_;
        const double accelerationDistance = 0.5 * jointMoveProgressAcceleration_ *
            jointMoveAccelerationTimeSeconds_ * jointMoveAccelerationTimeSeconds_;
        if (elapsed < jointMoveAccelerationTimeSeconds_)
        {
            nextProgress = 0.5 * jointMoveProgressAcceleration_ * elapsed * elapsed;
            nextVelocity = jointMoveProgressAcceleration_ * elapsed;
        }
        else if (elapsed < jointMoveAccelerationTimeSeconds_ + jointMoveCruiseTimeSeconds_)
        {
            nextProgress = accelerationDistance + jointMovePeakProgressVelocity_ *
                (elapsed - jointMoveAccelerationTimeSeconds_);
            nextVelocity = jointMovePeakProgressVelocity_;
        }
        else
        {
            const double decelerationTime = jointMoveProfileDurationSeconds_ - elapsed;
            nextProgress = 1.0 - 0.5 * jointMoveProgressAcceleration_ *
                decelerationTime * decelerationTime;
            nextVelocity = jointMoveProgressAcceleration_ * decelerationTime;
        }
    }
    nextProgress = std::clamp(nextProgress, jointMoveProgress_, jointMoveBraking_ ?
        jointMoveStopProgress_ : 1.0);
    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        const double start = jointMoveStartPositionRadians_[i];
        const double delta = targetPositionRadians_[i] - start;
        state_.jointPositionRadians[i] = start + delta * nextProgress;
        state_.jointVelocityRadiansPerSecond[i] = delta * nextVelocity;
    }
    jointMoveProgress_ = nextProgress;
    jointMoveProgressVelocity_ = nextVelocity;
    if (jointMoveBraking_ && jointMoveProgress_ >= jointMoveStopProgress_ - kPositionEpsilon)
    {
        jointMoveProgressVelocity_ = 0.0;
        std::fill(state_.jointVelocityRadiansPerSecond.begin(),
            state_.jointVelocityRadiansPerSecond.end(), 0.0);
        for (std::size_t i = 0; i < specification_->jointCount; ++i)
            state_.jointPositionRadians[i] = jointMoveStartPositionRadians_[i] +
                (targetPositionRadians_[i] - jointMoveStartPositionRadians_[i]) * jointMoveStopProgress_;
        if (hasPendingJointTarget_)
        {
            JointVector pendingTarget = std::move(pendingJointTargetRadians_);
            const double pendingVelocity = pendingVelocityScale_;
            const double pendingAcceleration = pendingAccelerationScale_;
            ConfigureJointMove(std::move(pendingTarget), pendingVelocity, pendingAcceleration);
        }
    }
    else if (jointMoveProgress_ >= 1.0)
    {
        state_.jointPositionRadians = targetPositionRadians_;
        jointMoveProgressVelocity_ = 0.0;
        std::fill(
            state_.jointVelocityRadiansPerSecond.begin(),
            state_.jointVelocityRadiansPerSecond.end(),
            0.0);
        if (posePathSegment_ + 1 < posePathPoints_.size())
        {
            ++posePathSegment_;
            ConfigureJointMove(posePathPoints_[posePathSegment_], pendingPoseMove_.velocityScale,
                pendingPoseMove_.accelerationScale);
        }
        else
        {
            posePathPoints_.clear();
            posePathSegment_ = 0;
            state_.mode = RobotMode::Idle;
        }
    }
    RefreshTcp();
}

void SimRobotController::UpdateLinear(double dtSeconds)
{
    using namespace kinematics::detail;
    // 이번 tick 시작 관절각은 별도 임시 vector 대신, 끝에서 새 속도로 덮어쓸 속도 배열에 잠시 보관한다.
    std::copy(state_.jointPositionRadians.begin(), state_.jointPositionRadians.end(),
        state_.jointVelocityRadiansPerSecond.begin());
    double currentPathTime = 0.0;
    for (std::size_t i = 1; i < linearSegment_; ++i)
        currentPathTime += linearPath_[i].durationSeconds;
    if (linearSegment_ < linearPath_.size())
        currentPathTime += linearSegmentFraction_ * linearPath_[linearSegment_].durationSeconds;
    linearProfileElapsedSeconds_ = std::min(linearProfileDurationSeconds_, linearProfileElapsedSeconds_ + dtSeconds);
    const double profileEnd = linearProfileElapsedSeconds_ / linearProfileDurationSeconds_;
    const double desiredPathTime = linearPlannedDurationSeconds_ *
        TrapezoidDistanceFraction(profileEnd, linearProfileRampFraction_);
    double pathTimeBudget = std::max(0.0, desiredPathTime - currentPathTime);
    double remaining = dtSeconds;
    bool failed = false;
    ErrorCode failureCode = ErrorCode::IkDidNotConverge;
    JointVector& interpolatedJoints = linearInterpolationBuffer_;
    while (remaining > 0.0 && pathTimeBudget > 0.0 && linearSegment_ < linearPath_.size())
    {
        const auto& begin = linearPath_[linearSegment_ - 1];
        const auto& end = linearPath_[linearSegment_];
        const double segmentRemaining = (1.0 - linearSegmentFraction_) * end.durationSeconds;
        const double available = remaining;
        double fractionStep = std::min(pathTimeBudget, segmentRemaining) / end.durationSeconds;
        double usedSeconds = 0.0;
        // 유한한 양수 시간도 표현 가능한 경로 비율보다 작을 수 있다. 나눗셈의 0 분모나 진행 없는 반복 대신 현재 자세를 유지하고 다음 tick을 기다린다.
        if (!std::isfinite(available) || available <= 0.0 || fractionStep <= 0.0 ||
            linearSegmentFraction_ + fractionStep == linearSegmentFraction_)
            break;
        const Pose3 currentTcp = FromCartesian(inverse_.EvaluateTcp(state_.jointPositionRadians));
        bool accepted = false;
        for (std::size_t attempt = 0;
            attempt < detail::kDefaultSimulationMotionPolicy.linearRuntimeRetryAttempts; ++attempt)
        {
            const double nextSegmentFraction = std::min(1.0, linearSegmentFraction_ + fractionStep);
            if (nextSegmentFraction == linearSegmentFraction_)
            {
                accepted = true;
                usedSeconds = available;
                break;
            }
            // 명령을 받을 때 직선 TCP 목표마다 IK와 충돌 검사를 끝낸 경로 표본 사이를 보간해 매 tick IK 반복을 없앤다.
            for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
                interpolatedJoints[joint] = begin.joints[joint] +
                    (end.joints[joint] - begin.joints[joint]) * nextSegmentFraction;
            double requiredSeconds = 0.0;
            for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
                requiredSeconds = std::max(requiredSeconds, planning::RequiredTimeForVelocity(
                    interpolatedJoints[joint] - state_.jointPositionRadians[joint],
                    specification_->joints[joint].maxVelocityRadiansPerSecond));
            const Pose3 nextTcp = FromCartesian(inverse_.EvaluateTcp(interpolatedJoints));
            requiredSeconds = std::max(requiredSeconds,
                planning::RequiredTimeForVelocity(
                    Length(Subtract(nextTcp.positionMeters, currentTcp.positionMeters)), linearVelocityLimit_));
            requiredSeconds = std::max(requiredSeconds,
                planning::RequiredTimeForVelocity(
                    Length(RotationError(nextTcp.rotation, currentTcp.rotation)), angularVelocityLimit_));
            const double ratio = std::max(1.0, requiredSeconds / available);
            if (ratio <= 1.0 + 1e-8)
            {
                accepted = true;
                state_.jointPositionRadians = interpolatedJoints;
                linearSegmentFraction_ = nextSegmentFraction;
                // 표본 끝에 남은 명목 시간이 아주 작아도 IK 허용 오차를 넘는 보정에는 실제 시간이 필요하다.
                // 남은 실제 tick 전체를 속도 예산으로 쓰고, 명목 진행 시간과 실제 변화에 필요한 시간 중 큰 값을 소비해 같은 시간을 다음 구간에 중복 사용하지 않는다.
                usedSeconds = std::min(available, std::max(fractionStep * end.durationSeconds, requiredSeconds));
                pathTimeBudget -= fractionStep * end.durationSeconds;
                break;
            }
            // 실제 관절 변화가 속도 상한을 넘으면 이번 tick의 경로 진행 비율을 줄여 다시 확인한다.
            fractionStep *= detail::kDefaultSimulationMotionPolicy.runtimeRetryProgressScale / ratio;
        }
        if (!accepted)
        {
            // 특이 자세에서는 TCP를 거의 움직이지 않고 손목 관절끼리 반대 방향으로 돌려야 다음 직선 자세를 만들 수 있다.
            // 경로 표본의 관절각 방향을 참고하되 실제 TCP와 모든 속도 상한을 다시 확인한 작은 보조 이동만 허용한다.
            if (!ReorientForLinear(end.joints, available))
            {
                failed = true;
                break;
            }
            remaining = 0.0;
            continue;
        }
        remaining = usedSeconds >= remaining ? 0.0 : remaining - usedSeconds;
        if (linearSegmentFraction_ >= 1.0 - 1e-12)
        {
            linearSegmentFraction_ = 0.0;
            ++linearSegment_;
        }
        else
        {
            // 속도 때문에 경로 진행을 줄였으면 이번 시간은 모두 소비했다. 남은 경로 시간은 다음 Update가 이어서 처리한다.
            remaining = 0.0;
        }
    }
    if (failed)
    {
        targetPositionRadians_ = state_.jointPositionRadians;
        linearPath_.clear();
        state_.mode = RobotMode::Fault;
        state_.errorCode = failureCode;
    }
    else if (linearSegment_ >= linearPath_.size())
    {
        linearPath_.clear();
        state_.mode = RobotMode::Idle;
    }
    for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
    {
        const double previousPosition = state_.jointVelocityRadiansPerSecond[joint];
        state_.jointVelocityRadiansPerSecond[joint] = state_.mode == RobotMode::Moving ?
            (state_.jointPositionRadians[joint] - previousPosition) / dtSeconds : 0.0;
    }
    RefreshTcp();
}

bool SimRobotController::ReorientForLinear(const JointVector& plannedJoints, double availableSeconds)
{
    using namespace kinematics::detail;
    const JointVector current = state_.jointPositionRadians;
    const Pose3 currentTcp = FromCartesian(inverse_.EvaluateTcp(current));
    const auto& begin = linearPath_[linearSegment_ - 1];
    const auto& end = linearPath_[linearSegment_];
    const Pose3 pathTcp = Interpolate(FromCartesian(begin.tcpPose), FromCartesian(end.tcpPose), linearSegmentFraction_);
    const auto options = RuntimeLinearIkOptions();
    double attraction = 1.0;
    double previousDistanceSquared = 0.0;
    for (std::size_t i = 0; i < current.size(); ++i)
    {
        const double delta = plannedJoints[i] - current[i];
        previousDistanceSquared += delta * delta;
        if (std::abs(delta) > 0.0)
            attraction = std::min(attraction, specification_->joints[i].maxVelocityRadiansPerSecond * availableSeconds / std::abs(delta));
    }
    // 후보 시작각을 다음 표본 방향으로 작은 비율만 옮기고 현재 경로상의 TCP 자세 IK를 다시 푼다.
    // IK가 위치·방향을 되돌리는 동안 남는 관절 변화는 TCP를 거의 바꾸지 않는 여유 방향이며 nullspace라고 부른다.
    // 일반 모델에서 단순 관절 보간이 TCP를 고정한다고 가정하지 않으므로 후보마다 FK/속도 검사를 거친다.
    for (std::size_t attempt = 0;
        attempt < detail::kDefaultSimulationMotionPolicy.reorientationRetryAttempts; ++attempt)
    {
        JointVector seed = current;
        for (std::size_t i = 0; i < current.size(); ++i)
            seed[i] += (plannedJoints[i] - current[i]) * attraction;
        // 현재 FK 값 대신 명목 경로의 같은 진행률 자세로 복원해 반복 보조 이동의 작은 IK 오차가 직선 밖으로 누적되지 않게 한다.
        const auto solution = inverse_.Solve(ToCartesian(pathTcp), seed, options);
        if (solution)
        {
            double ratio = 1.0;
            double nextDistanceSquared = 0.0;
            double movement = 0.0;
            for (std::size_t i = 0; i < current.size(); ++i)
            {
                const double change = solution.jointPositionRadians[i] - current[i];
                ratio = std::max(ratio, planning::VelocityRatio(
                    change, availableSeconds, specification_->joints[i].maxVelocityRadiansPerSecond));
                movement = std::max(movement, std::abs(change));
                const double residual = plannedJoints[i] - solution.jointPositionRadians[i];
                nextDistanceSquared += residual * residual;
            }
            const Pose3 nextTcp = FromCartesian(inverse_.EvaluateTcp(solution.jointPositionRadians));
            const double pathPositionError = Length(Subtract(nextTcp.positionMeters, pathTcp.positionMeters));
            const double pathOrientationError = Length(RotationError(nextTcp.rotation, pathTcp.rotation));
            ratio = std::max(ratio, planning::VelocityRatio(
                Length(Subtract(nextTcp.positionMeters, currentTcp.positionMeters)), availableSeconds, linearVelocityLimit_));
            ratio = std::max(ratio, planning::VelocityRatio(
                Length(RotationError(nextTcp.rotation, currentTcp.rotation)), availableSeconds, angularVelocityLimit_));
            const bool pathValid = planning::ValidateJointPath(
                current, solution.jointPositionRadians, *specification_, jointStateValidityChecker_) ==
                planning::JointStateInvalidity::None;
            const bool reorientationFollowsTcp = JointInterpolationFollowsTcpLine(
                current, solution.jointPositionRadians, pathTcp, pathTcp, inverse_);
            const Pose3 plannedEndTcp = FromCartesian(end.tcpPose);
            const bool remainingLineFollowsTcp = JointInterpolationFollowsTcpLine(
                solution.jointPositionRadians, end.joints, pathTcp, plannedEndTcp, inverse_);
            if (ratio <= 1.0 + 1e-8 && movement > 1e-12 && nextDistanceSquared < previousDistanceSquared &&
                pathPositionError <= options.positionToleranceMeters &&
                pathOrientationError <= options.orientationToleranceRadians && pathValid &&
                reorientationFollowsTcp && remainingLineFollowsTcp)
            {
                state_.jointPositionRadians = solution.jointPositionRadians;
                return true;
            }
        }
        attraction *= 0.5;
    }
    return false;
}

const models::RobotSpecification& SimRobotController::GetSpecification() const noexcept
{
    // 생성 시 받은 사양을 복사하지 않고 참조한다. 따라서 사양 데이터를 가진 저장소는 이 Controller보다 오래 살아 있어야 한다.
    return *specification_;
}

bool SimRobotController::RestoreCollisionSafeState(const JointVector& safePositionRadians)
{
    return RestoreCollisionSafeState(safePositionRadians, ErrorCode::EnvironmentContact);
}

bool SimRobotController::RestoreCollisionSafeState(
    const JointVector& safePositionRadians, ErrorCode collisionReason)
{
    if (safePositionRadians.size() != specification_->jointCount ||
        !ValidateJointPositionLimits(*specification_, safePositionRadians))
        return false;
    if (collisionReason != ErrorCode::EnvironmentContact && collisionReason != ErrorCode::SelfCollision &&
        collisionReason != ErrorCode::AttachedObjectCollision)
        return false;

    linearPathPlanningJob_.Cancel();
    if (posePlanningPhase_ != PosePlanningPhase::None)
        inverse_.CancelSingleSeed(poseIkSession_);
    poseJointPathPlanningJob_.Cancel();
    posePlanningPhase_ = PosePlanningPhase::None;
    linearPathPlanningJob_ = {};
    pendingLinearPathCommand_ = {};
    linearPathPlanningResult_.reset();
    poseAlternativeSeeds_.clear();
    posePathPoints_.clear();
    posePathSegment_ = 0;

    state_.jointPositionRadians = safePositionRadians;
    std::fill(state_.jointVelocityRadiansPerSecond.begin(), state_.jointVelocityRadiansPerSecond.end(), 0.0);
    targetPositionRadians_ = safePositionRadians;
    jointMoveStartPositionRadians_ = safePositionRadians;
    jointMoveProgress_ = 0.0;
    jointMoveProgressVelocity_ = 0.0;
    jointMoveProgressAcceleration_ = 0.0;
    jointMoveProgressRate_ = 0.0;
    jointMoveBraking_ = false;
    hasPendingJointTarget_ = false;
    pendingJointTargetRadians_.clear();
    linearPath_.clear();
    linearSegment_ = 1;
    linearSegmentFraction_ = 0.0;
    linearProfileElapsedSeconds_ = 0.0;
    poseContinuationCommand_.reset();
    poseContinuationPlan_.reset();
    poseCandidateJoints_.clear();
    // unsafe tick은 직전 안전 자세로 되돌린 뒤 멈춘다. Fault로 고정하면 호출자가 위쪽 안전 자세로 물러나는 명령도 보낼 수 없다.
    state_.mode = RobotMode::Idle;
    state_.errorCode = collisionReason;
    RefreshTcp();
    return true;
}

} // namespace
