#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/backends/simulation/detail/SimulationMotionPolicy.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"
#include "robotics/kinematics/detail/PoseMath.h"
#include "planning/detail/LinearPathPlannerSupport.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace grasplink::robotics::backends::simulation
{
namespace planning = grasplink::robotics::planning;
namespace
{
constexpr std::size_t kMaximumPoseIkSeeds = 64;
constexpr std::size_t kMaximumPoseIkSeedsWithContinuation = 16;
constexpr double kJointPositionMatchTolerance = 1e-8;

struct SimRobotControllerMessages final
{
    inline static constexpr char notConnected[] = "SimRobotController: not connected";
    inline static constexpr char motionPlanningBusy[] = "SimRobotController: motion planning is in progress";
    inline static constexpr char invalidScale[] = "SimRobotController: scale must be in (0, 1]";
    inline static constexpr char missingToolFrame[] = "SimRobotController: missing ToolFrame for TCP motion";
    inline static constexpr char plannerReturnedNoPlan[] = "SimRobotController: completed planner returned no plan";
    inline static constexpr char jointLimitFailurePrefix[] =
        "SimRobotController: target outside joint limit: ";
    inline static constexpr char invalidMaximumJointVelocity[] =
        "SimRobotController: maximum joint velocity must be finite and positive";
    inline static constexpr char validityCheckerChangedDuringPlanning[] =
        "SimRobotController: motion planning cancelled because the validity checker changed";
    inline static constexpr char jointCountMismatch[] = "SimRobotController: joint count mismatch";
    inline static constexpr char noCollisionFreePosePath[] =
        "SimRobotController: no collision-free IK solution or joint path";
    inline static constexpr char continuationPlannerReturnedNoPlan[] =
        "SimRobotController: completed continuation planner returned no plan";
    inline static constexpr char jointPathPlannerReturnedNoPath[] =
        "SimRobotController: joint path planner completed without a path";
    inline static constexpr char jointPathHasNoStartState[] =
        "SimRobotController: joint path contains no start state";
    inline static constexpr char posePlanningCancelled[] = "SimRobotController: pose planning cancelled";
    inline static constexpr char linearPlanningCancelled[] =
        "SimRobotController: linear path planning cancelled";
};

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
        if (!std::isfinite(position) || (joint.HasPositionLimits() &&
            (position < joint.minimumPosition || position > joint.maximumPosition)))
        {
            return Failure(ErrorCode::InvalidCommand,
                std::string{SimRobotControllerMessages::jointLimitFailurePrefix} + std::string(joint.name));
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
        if (std::abs(lhs[index] - rhs[index]) > kJointPositionMatchTolerance)
            return false;
    }
    return true;
}

} // namespace

SimRobotController::SimRobotController(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame)
    : specification_(specification), inverse_(specification, tcpInToolFrame)
{
    // IK 계산기가 관절 배열과 위치 범위를 먼저 검사한다. Controller는 속도 제한만 추가로 확인한다.
    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        if (!std::isfinite(joint.maximumVelocity) || joint.maximumVelocity <= 0.0)
            throw std::invalid_argument(SimRobotControllerMessages::invalidMaximumJointVelocity);
    }
}

void SimRobotController::SetJointStateValidityChecker(
    planning::StateValidityChecker checker)
{
    const bool wasPlanning = IsMotionPlanning();
    ResetMotionPlanningWork();
    if (wasPlanning)
    {
        state_.mode = RobotMode::Idle;
        state_.errorCode = ErrorCode::Cancelled;
        motionPlanningResult_.emplace(
            Failure(ErrorCode::Cancelled, SimRobotControllerMessages::validityCheckerChangedDuringPlanning));
    }
    jointStateValidityChecker_ = std::move(checker);
}

void SimRobotController::SetJointStateValidityDiagnosticProvider(std::function<std::string()> provider)
{
    jointStateValidityDiagnosticProvider_ = std::move(provider);
}

Result SimRobotController::Connect()
{
    // 재연결은 이전 명령과 상태를 이어가지 않고 q=0의 새 논리 세션을 만든다.
    ResetMotionPlanningWork();
    motionPlanningResult_.reset();
    state_ = {};
    state_.jointPositions.resize(specification_.jointCount);
    for (std::size_t joint = 0; joint < specification_.jointCount; ++joint)
    {
        const auto& jointSpec = specification_.joints[joint];
        state_.jointPositions[joint] = jointSpec.HasPositionLimits()
            ? std::clamp(0.0, jointSpec.minimumPosition, jointSpec.maximumPosition)
            : 0.0;
    }
    state_.jointVelocities.assign(specification_.jointCount, 0.0);
    state_.mode = RobotMode::Idle;
    state_.valid = true;
    RefreshTcp();

    jointMotion_.Reset(state_.jointPositions);
    linearInterpolationBuffer_.resize(specification_.jointCount);
    previousJointPositions_.resize(specification_.jointCount);
    connected_ = true;
    linearMotion_.ClearPath();
    return Result::Success();
}

void SimRobotController::Disconnect() noexcept
{
    ResetMotionPlanningWork();
    motionPlanningResult_.reset();
    // 백엔드의 관절 배열은 재사용하되 연결/feedback 유효성을 내리고 속도만 즉시 0으로 만든다.
    connected_ = false;
    state_.mode = RobotMode::Disconnected;
    state_.valid = false;
    state_.tcpPoseValid = false;
    linearMotion_.ClearPath();
    std::fill(
        state_.jointVelocities.begin(),
        state_.jointVelocities.end(),
        0.0);
}

bool SimRobotController::IsConnected() const noexcept
{
    return connected_;
}

Result SimRobotController::ValidateCommandAvailability() const
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, SimRobotControllerMessages::notConnected);
    if (IsMotionPlanning())
        return Failure(ErrorCode::Busy, SimRobotControllerMessages::motionPlanningBusy);
    return Result::Success();
}

Result SimRobotController::MoveJoint(const JointMoveCommand& command)
{
    return MoveJointImpl(command, JointMoveOrigin::ExternalJointCommand);
}

Result SimRobotController::MoveJointImpl(const JointMoveCommand& command, JointMoveOrigin origin)
{
    const Result availability = ValidateCommandAvailability();
    if (!availability)
        return availability;

    if (command.targetPositions.size() != specification_.jointCount)
        return Failure(ErrorCode::InvalidCommand, SimRobotControllerMessages::jointCountMismatch);

    if (!IsScaleValid(command.velocityScale) || !IsScaleValid(command.accelerationScale))
        return Failure(ErrorCode::InvalidCommand, SimRobotControllerMessages::invalidScale);

    const Result validation = ValidateJointPositionLimits(specification_, command.targetPositions);
    if (!validation)
        return validation;

    // 모든 joint를 먼저 확인한다. 하나라도 범위 밖이면 target이나 기존 진행 상태를 일부만 바꾸지 않는다.
    // 명령을 받는 순간 관절각 q를 바꾸지 않는다. 이후 Update 호출이 정해진 속도 안에서 현재 상태를 목표까지 진행시킨다.
    // 움직이는 도중 새 명령이 들어오면 대기열에 쌓지 않고 현재 목표를 새 목표로 바꾼다.
    JointVector targetPositions = command.targetPositions;
    planning::detail::AlignEquivalentJointAngles(targetPositions, state_.jointPositions, specification_);

    const bool canBrakeJointPath = state_.mode == RobotMode::Moving && linearMotion_.path.empty() &&
        jointMotion_.progressVelocity > detail::kMotionEpsilon && jointMotion_.progressAcceleration > 0.0;
    const double stoppingProgress = canBrakeJointPath ? std::min(1.0, jointMotion_.progress +
        jointMotion_.progressVelocity * jointMotion_.progressVelocity /
            (2.0 * jointMotion_.progressAcceleration)) : jointMotion_.progress;
    const JointVector* nextPathStart = &state_.jointPositions;
    JointVector brakingStart;
    if (canBrakeJointPath)
    {
        brakingStart = state_.jointPositions;
        for (std::size_t joint = 0; joint < specification_.jointCount; ++joint)
            brakingStart[joint] = jointMotion_.startPositions[joint] +
                (jointMotion_.targetPositions[joint] - jointMotion_.startPositions[joint]) * stoppingProgress;
        nextPathStart = &brakingStart;
    }

    if ((origin == JointMoveOrigin::ExternalJointCommand || canBrakeJointPath) && jointStateValidityChecker_)
    {
        const auto invalidity = planning::ValidateJointPath(*nextPathStart, targetPositions,
            specification_, jointStateValidityChecker_);
        if (invalidity != planning::JointStateInvalidity::None)
        {
            Result result = planning::MapJointStateInvalidity(invalidity);
            AppendValidityDiagnostic(result, jointStateValidityDiagnosticProvider_);
            return result;
        }
    }

    if (origin != JointMoveOrigin::PlannedPoseSegment)
        ClearPosePlanningState();

    if (canBrakeJointPath)
    {
        jointMotion_.pendingMove = PendingJointMove{
            std::move(targetPositions), command.velocityScale, command.accelerationScale};
        jointMotion_.braking = true;
        jointMotion_.stopProgress = stoppingProgress;
        state_.errorCode = ErrorCode::None;
        return Result::Success();
    }

    ConfigureJointMove(std::move(targetPositions), command.velocityScale, command.accelerationScale);
    state_.errorCode = ErrorCode::None;
    return Result::Success();
}

Result SimRobotController::MovePose(const CartesianPose& targetInBase, double velocityScale, double accelerationScale)
{
    const Result availability = ValidateCommandAvailability();
    if (!availability)
        return availability;
    if (!IsScaleValid(velocityScale) || !IsScaleValid(accelerationScale))
        return Failure(ErrorCode::InvalidCommand, SimRobotControllerMessages::invalidScale);
    planning::JointStateInvalidity invalidity = planning::JointStateInvalidity::None;
    kinematics::IkResult ikFailure;
    auto solution = planning::detail::SolveCollisionFreeIk(inverse_, specification_, jointStateValidityChecker_,
        targetInBase, state_.jointPositions,
        kinematics::RobotDefaultIkOptions(specification_), planning::kDefaultPlanningPolicy,
        invalidity, ikFailure);
    if (!solution)
    {
        if (invalidity == planning::JointStateInvalidity::EnvironmentCollision)
        {
            return Failure(ErrorCode::EnvironmentContact,
                SimRobotControllerMessages::noCollisionFreePosePath);
        }
        if (invalidity != planning::JointStateInvalidity::None)
        {
            Result result = planning::MapJointStateInvalidity(invalidity);
            AppendValidityDiagnostic(result, jointStateValidityDiagnosticProvider_);
            return result;
        }
        return planning::detail::MapIkFailure(ikFailure);
    }
    return MoveJointImpl({solution->jointPositions, velocityScale, accelerationScale},
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
    const Result availability = ValidateCommandAvailability();
    if (!availability)
        return availability;
    if (!IsScaleValid(velocityScale) || !IsScaleValid(accelerationScale))
        return Failure(ErrorCode::InvalidCommand, SimRobotControllerMessages::invalidScale);
    if (continuation)
    {
        const Result validation = planning::detail::ValidateLinearPathCommand(*continuation);
        if (!validation)
            return validation;
    }
    if (!specification_.hasToolFrame)
        return Failure(ErrorCode::Unsupported, SimRobotControllerMessages::missingToolFrame);

    motionPlanningResult_.reset();
    posePathPoints_.clear();
    posePlanning_.continuationCommand = std::move(continuation);
    posePlanning_.continuationPlan.reset();
    posePlanning_.candidateJoints.clear();
    posePlanning_.alternativeSeeds = kinematics::detail::BuildAlternativeIkSeeds(
        state_.jointPositions, specification_);
    const std::size_t maximumSeeds = posePlanning_.continuationCommand
        ? kMaximumPoseIkSeedsWithContinuation : kMaximumPoseIkSeeds;
    kinematics::detail::AppendIkRestartSeeds(posePlanning_.alternativeSeeds,
        state_.jointPositions, specification_, maximumSeeds - 1);
    posePlanning_.alternativeSeedIndex = 0;
    posePlanning_.ikSeedAttempts = 1;
    posePathSegment_ = 0;
    posePlanning_.target = targetInBase;
    posePlanning_.pathFailure = Result::Success();
    pendingPoseMove_ = {{}, velocityScale, accelerationScale};
    posePlanning_.ikSession = inverse_.BeginSingleSeed(targetInBase, state_.jointPositions,
        kinematics::RobotDefaultIkOptions(specification_));
    posePlanning_.phase = PosePlanningPhase::Ik;
    state_.mode = RobotMode::Planning;
    state_.errorCode = ErrorCode::None;
    return Result::Success();
}

bool SimRobotController::StartNextPoseIkSeed()
{
    if (posePlanning_.alternativeSeedIndex >= posePlanning_.alternativeSeeds.size())
        return false;
    posePlanning_.continuationPlan.reset();
    posePlanning_.candidateJoints.clear();
    posePlanning_.ikSession = inverse_.BeginSingleSeed(posePlanning_.target,
        posePlanning_.alternativeSeeds[posePlanning_.alternativeSeedIndex++],
        kinematics::RobotDefaultIkOptions(specification_));
    ++posePlanning_.ikSeedAttempts;
    posePlanning_.phase = PosePlanningPhase::Ik;
    return true;
}

bool SimRobotController::StartPoseCandidateJointPath()
{
    const Result begin = posePlanning_.jointPathJob.Begin(specification_,
        state_.jointPositions, posePlanning_.candidateJoints, jointStateValidityChecker_,
        planning::kDefaultPlanningPolicy);
    if (!begin)
    {
        posePlanning_.pathFailure = begin;
        return false;
    }
    posePlanning_.phase = PosePlanningPhase::JointPath;
    return true;
}

std::size_t SimRobotController::GetLastPoseIkSeedAttemptCount() const noexcept
{
    return posePlanning_.ikSeedAttempts;
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
        command, specification_, state_.jointPositions,
        inverse_.EvaluateTcp(state_.jointPositions), inverse_, jointStateValidityChecker_, plan,
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
        motionPlanningResult_ = *cached;
        return *cached;
    }

    pendingLinearPathCommand_ = command;
    motionPlanningResult_.reset();
    const Result begin = linearPathPlanningJob_.Begin(command, specification_,
        state_.jointPositions, inverse_.EvaluateTcp(state_.jointPositions),
        inverse_, jointStateValidityChecker_, planning::kDefaultPlanningPolicy,
        jointStateValidityDiagnosticProvider_);
    if (!begin)
    {
        motionPlanningResult_ = begin;
        pendingLinearPathCommand_ = {};
        linearPathPlanningJob_ = {};
        return begin;
    }
    if (linearPathPlanningJob_.GetState() == planning::LinearPathPlanningState::Completed)
    {
        auto plan = linearPathPlanningJob_.TakePlan();
        const Result committed = plan ? CommitLinearPathPlan(command, std::move(*plan)) :
            Failure(ErrorCode::Fault, SimRobotControllerMessages::plannerReturnedNoPlan);
        motionPlanningResult_ = committed;
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

    switch (posePlanning_.phase)
    {
    case PosePlanningPhase::Ik:
        AdvancePoseIkPlanning(workBudget);
        return;
    case PosePlanningPhase::LinearContinuation:
        AdvancePoseLinearContinuation(workBudget);
        return;
    case PosePlanningPhase::JointPath:
        AdvancePoseJointPath(workBudget);
        return;
    case PosePlanningPhase::Inactive:
        // 포즈 계획이 없을 때는 별도로 시작한 선형 경로 작업을 진행한다.
        AdvanceLinearPath(workBudget);
        return;
    }
}

void SimRobotController::RetryPosePlanningOrFinish(Result failure)
{
    if (!StartNextPoseIkSeed())
        FinishPosePlanning(std::move(failure));
}

void SimRobotController::AdvancePoseIkPlanning(std::size_t workBudget)
{
    const auto ikState = inverse_.StepSingleSeed(posePlanning_.ikSession, workBudget);
    if (ikState == kinematics::IkSessionState::Running)
        return;

    const auto& ikResult = inverse_.GetSingleSeedResult(posePlanning_.ikSession);
    if (!ikResult)
    {
        RetryPosePlanningOrFinish(planning::detail::MapIkFailure(ikResult));
        return;
    }

    posePlanning_.candidateJoints = ikResult.jointPositions;
    if (!posePlanning_.continuationCommand)
    {
        if (!StartPoseCandidateJointPath())
            RetryPosePlanningOrFinish(posePlanning_.pathFailure);
        return;
    }

    StartPoseLinearContinuation();
}

void SimRobotController::StartPoseLinearContinuation()
{
    const auto invalidity = jointStateValidityChecker_
        ? jointStateValidityChecker_(posePlanning_.candidateJoints)
        : planning::JointStateInvalidity::None;
    if (invalidity != planning::JointStateInvalidity::None)
    {
        posePlanning_.pathFailure = planning::MapJointStateInvalidity(invalidity);
        RetryPosePlanningOrFinish(posePlanning_.pathFailure);
        return;
    }

    const Result continuationBegin = linearPathPlanningJob_.Begin(
        *posePlanning_.continuationCommand, specification_, posePlanning_.candidateJoints,
        inverse_.EvaluateTcp(posePlanning_.candidateJoints), inverse_, jointStateValidityChecker_,
        planning::kDefaultPlanningPolicy, jointStateValidityDiagnosticProvider_);
    if (!continuationBegin)
    {
        posePlanning_.pathFailure = continuationBegin;
        RetryPosePlanningOrFinish(posePlanning_.pathFailure);
        return;
    }
    posePlanning_.phase = PosePlanningPhase::LinearContinuation;
}

void SimRobotController::AdvancePoseLinearContinuation(std::size_t workBudget)
{
    const auto planningState = linearPathPlanningJob_.Advance(workBudget);
    if (planningState == planning::LinearPathPlanningState::Running)
        return;

    if (planningState != planning::LinearPathPlanningState::Completed)
    {
        posePlanning_.pathFailure = linearPathPlanningJob_.GetResult();
        linearPathPlanningJob_ = {};
        RetryPosePlanningOrFinish(posePlanning_.pathFailure);
        return;
    }

    posePlanning_.continuationPlan = linearPathPlanningJob_.TakePlan();
    linearPathPlanningJob_ = {};
    if (!posePlanning_.continuationPlan)
    {
        posePlanning_.pathFailure = Failure(ErrorCode::Fault,
            SimRobotControllerMessages::continuationPlannerReturnedNoPlan);
        RetryPosePlanningOrFinish(posePlanning_.pathFailure);
        return;
    }

    if (!StartPoseCandidateJointPath())
        RetryPosePlanningOrFinish(posePlanning_.pathFailure);
}

void SimRobotController::AdvancePoseJointPath(std::size_t workBudget)
{
    const auto planningState = posePlanning_.jointPathJob.Advance(workBudget);
    if (planningState == planning::JointPathPlanningState::Running)
        return;

    if (planningState != planning::JointPathPlanningState::Completed)
    {
        RetryPosePlanningOrFinish(posePlanning_.jointPathJob.GetResult());
        return;
    }

    auto plan = posePlanning_.jointPathJob.TakePlan();
    if (!plan)
    {
        FinishPosePlanning(Failure(ErrorCode::Fault,
            SimRobotControllerMessages::jointPathPlannerReturnedNoPath));
        return;
    }

    posePathPoints_ = std::move(plan->points);
    posePlanning_.alternativeSeeds.clear();
    posePathSegment_ = 0;
    posePlanning_.phase = PosePlanningPhase::Inactive;
    if (!posePlanning_.continuationPlan)
        posePlanning_.continuationCommand.reset();

    const Result execution = StartPosePathSegment();
    if (!execution)
    {
        posePlanning_.continuationPlan.reset();
        posePlanning_.continuationCommand.reset();
    }
    FinishPosePlanning(execution);
}

void SimRobotController::AdvanceLinearPath(std::size_t workBudget)
{
    const auto planningState = linearPathPlanningJob_.Advance(workBudget);
    if (planningState == planning::LinearPathPlanningState::Running)
        return;

    if (planningState == planning::LinearPathPlanningState::Completed)
    {
        auto plan = linearPathPlanningJob_.TakePlan();
        motionPlanningResult_ = plan
            ? CommitLinearPathPlan(pendingLinearPathCommand_, std::move(*plan))
            : Failure(ErrorCode::Fault, SimRobotControllerMessages::plannerReturnedNoPlan);
        pendingLinearPathCommand_ = {};
        return;
    }

    motionPlanningResult_ = linearPathPlanningJob_.GetResult();
    state_.mode = RobotMode::Idle;
    state_.errorCode = motionPlanningResult_->code;
    pendingLinearPathCommand_ = {};
    linearPathPlanningJob_ = {};
}

bool SimRobotController::IsMotionPlanning() const noexcept
{
    return posePlanning_.phase != PosePlanningPhase::Inactive ||
        linearPathPlanningJob_.GetState() == planning::LinearPathPlanningState::Running;
}

void SimRobotController::ClearPosePlanningState()
{
    posePlanning_.phase = PosePlanningPhase::Inactive;
    posePathPoints_.clear();
    posePlanning_.alternativeSeeds.clear();
    posePlanning_.continuationCommand.reset();
    posePlanning_.continuationPlan.reset();
    posePlanning_.candidateJoints.clear();
    posePathSegment_ = 0;
}

void SimRobotController::ResetMotionPlanningWork()
{
    linearPathPlanningJob_.Cancel();
    if (posePlanning_.phase != PosePlanningPhase::Inactive)
        inverse_.CancelSingleSeed(posePlanning_.ikSession);
    posePlanning_.jointPathJob.Cancel();
    ClearPosePlanningState();
    pendingLinearPathCommand_ = {};
}

void SimRobotController::FinishPosePlanning(Result result)
{
    posePlanning_.phase = PosePlanningPhase::Inactive;
    posePlanning_.alternativeSeeds.clear();
    if (!result && posePlanning_.ikSeedAttempts > 1)
        result.message += " after " + std::to_string(posePlanning_.ikSeedAttempts) + " IK seed attempts";
    motionPlanningResult_ = std::move(result);
    state_.errorCode = motionPlanningResult_->code;
    if (!*motionPlanningResult_)
    {
        state_.mode = RobotMode::Idle;
        ClearPosePlanningState();
        jointMotion_.pendingMove.reset();
    }
    // 성공한 경우에는 경로 시작 단계가 Moving 또는 Idle을 이미 정했으므로 그 상태를 유지한다.
}

Result SimRobotController::StartPosePathSegment()
{
    if (posePathPoints_.empty())
        return Failure(ErrorCode::Fault, SimRobotControllerMessages::jointPathHasNoStartState);
    if (posePathPoints_.size() == 1)
    {
        posePathPoints_.clear();
        return MoveJointImpl({state_.jointPositions, pendingPoseMove_.velocityScale,
            pendingPoseMove_.accelerationScale}, JointMoveOrigin::PlannedPoseSegment);
    }
    posePathSegment_ = 1;
    return MoveJointImpl({posePathPoints_[posePathSegment_], pendingPoseMove_.velocityScale,
        pendingPoseMove_.accelerationScale}, JointMoveOrigin::PlannedPoseSegment);
}

std::optional<Result> SimRobotController::TakeMotionPlanningResult()
{
    auto result = std::move(motionPlanningResult_);
    motionPlanningResult_.reset();
    return result;
}

Result SimRobotController::ValidateLinearPathRequest(
    const LinearPathMoveCommand& command) const
{
    const Result availability = ValidateCommandAvailability();
    if (!availability)
        return availability;
    const Result validation = planning::detail::ValidateLinearPathCommand(command);
    if (!validation)
        return validation;
    if (!specification_.hasToolFrame)
        return Failure(ErrorCode::Unsupported, SimRobotControllerMessages::missingToolFrame);
    return Result::Success();
}

Result SimRobotController::CommitLinearPathPlan(
    const LinearPathMoveCommand& command, planning::LinearPathPlan plan)
{
    if (!plan.hasMotion)
        return MoveJoint({state_.jointPositions, 1.0, 1.0});

    auto& candidate = plan.points;
    const double plannedLinearVelocity = plan.plannedLinearVelocity;
    const double plannedAngularVelocity = plan.plannedAngularVelocity;
    linearMotion_.linearVelocityLimit = command.maxLinearVelocityMetersPerSecond;
    linearMotion_.angularVelocityLimit = command.maxAngularVelocityRadiansPerSecond;
    linearMotion_.linearAccelerationLimit = command.maxLinearAccelerationMetersPerSecondSquared;
    linearMotion_.angularAccelerationLimit = command.maxAngularAccelerationRadiansPerSecondSquared;
    linearMotion_.plannedDurationSeconds = 0.0;
    for (std::size_t i = 1; i < candidate.size(); ++i)
        linearMotion_.plannedDurationSeconds += candidate[i].durationSeconds;
    // 모든 직선 구간의 계획 시간을 합쳐 가속·감속을 한 번만 적용한다. 웨이포인트마다 명령을 다시 내리면 매 구간마다 멈췄다가 재가속한다.
    const double accelerationTime = std::max(
        plannedLinearVelocity / linearMotion_.linearAccelerationLimit,
        plannedAngularVelocity / linearMotion_.angularAccelerationLimit);
    linearMotion_.profileRampFraction = accelerationTime /
        (linearMotion_.plannedDurationSeconds + accelerationTime);
    linearMotion_.profileRampFraction = std::clamp(linearMotion_.profileRampFraction,
        detail::kMinimumPathRampFraction, detail::kMaximumPathRampFraction);
    linearMotion_.profileDurationSeconds = linearMotion_.plannedDurationSeconds /
        (1.0 - linearMotion_.profileRampFraction);
    jointMotion_.targetPositions = candidate.back().joints;
    posePathPoints_.clear();
    posePathSegment_ = 0;
    posePlanning_.continuationCommand.reset();
    posePlanning_.continuationPlan.reset();
    posePlanning_.candidateJoints.clear();
    linearMotion_.ClearPath();
    linearMotion_.path = std::move(candidate);
    state_.errorCode = ErrorCode::None;
    state_.mode = RobotMode::Moving;
    return Result::Success();
}

std::optional<Result> SimRobotController::CommitCachedLinearContinuation(
    const LinearPathMoveCommand& command)
{
    if (!posePlanning_.continuationPlan || !posePlanning_.continuationCommand)
        return std::nullopt;

    auto plan = std::move(*posePlanning_.continuationPlan);
    const bool matches = SameLinearPathCommand(*posePlanning_.continuationCommand, command) &&
        !plan.points.empty() && SameJointPosition(state_.jointPositions, plan.points.front().joints);
    posePlanning_.continuationPlan.reset();
    posePlanning_.continuationCommand.reset();
    if (!matches)
        return std::nullopt;
    return CommitLinearPathPlan(command, std::move(plan));
}

void SimRobotController::RefreshTcp()
{
    state_.tcpPoseValid = specification_.hasToolFrame && state_.valid;
    if (state_.tcpPoseValid)
        state_.tcpPose = inverse_.EvaluateTcp(state_.jointPositions);
}

Result SimRobotController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, SimRobotControllerMessages::notConnected);

    if (IsMotionPlanning())
    {
        linearPathPlanningJob_.Cancel();
        if (posePlanning_.phase != PosePlanningPhase::Inactive)
        {
            inverse_.CancelSingleSeed(posePlanning_.ikSession);
            posePlanning_.jointPathJob.Cancel();
            FinishPosePlanning(Failure(ErrorCode::Cancelled,
                SimRobotControllerMessages::posePlanningCancelled));
        }
        else
        {
            motionPlanningResult_ = Failure(
                ErrorCode::Cancelled, SimRobotControllerMessages::linearPlanningCancelled);
            state_.mode = RobotMode::Idle;
            state_.errorCode = ErrorCode::Cancelled;
            pendingLinearPathCommand_ = {};
            linearPathPlanningJob_ = {};
        }
    }

    // 현재 q를 새 목표로 고정해 이후 Update가 남은 동작을 재개하지 않게 한다.
    jointMotion_.Reset(state_.jointPositions);
    linearMotion_.ClearPath();
    ClearPosePlanningState();
    std::fill(
        state_.jointVelocities.begin(),
        state_.jointVelocities.end(),
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

const models::RobotSpecification& SimRobotController::GetSpecification() const noexcept
{
    // 생성 시 받은 사양을 복사하지 않고 참조한다. 따라서 사양 데이터를 가진 저장소는 이 Controller보다 오래 살아 있어야 한다.
    return specification_;
}

bool SimRobotController::RestoreCollisionSafeState(const JointVector& safePositions)
{
    return RestoreCollisionSafeState(safePositions, ErrorCode::EnvironmentContact);
}

bool SimRobotController::RestoreCollisionSafeState(
    const JointVector& safePositions, ErrorCode collisionReason)
{
    if (safePositions.size() != specification_.jointCount ||
        !ValidateJointPositionLimits(specification_, safePositions))
        return false;
    switch (collisionReason)
    {
    case ErrorCode::EnvironmentContact:
    case ErrorCode::SelfCollision:
    case ErrorCode::AttachedObjectCollision:
        break;
    default:
        return false;
    }

    ResetMotionPlanningWork();
    motionPlanningResult_.reset();

    state_.jointPositions = safePositions;
    std::fill(state_.jointVelocities.begin(), state_.jointVelocities.end(), 0.0);
    jointMotion_.Reset(safePositions);
    linearMotion_.ClearPath();
    // unsafe tick은 직전 안전 자세로 되돌린 뒤 멈춘다. Fault로 고정하면 호출자가 위쪽 안전 자세로 물러나는 명령도 보낼 수 없다.
    state_.mode = RobotMode::Idle;
    state_.errorCode = collisionReason;
    RefreshTcp();
    return true;
}

} // namespace
