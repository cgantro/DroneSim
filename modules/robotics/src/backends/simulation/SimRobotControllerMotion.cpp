#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/backends/simulation/detail/SimulationMotionPolicy.h"
#include "robotics/kinematics/detail/PoseMath.h"
#include "planning/detail/LinearPathPlannerSupport.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace grasplink::robotics::backends::simulation
{
namespace planning = grasplink::robotics::planning;
namespace
{
constexpr double kRuntimeLinearIkPositionToleranceMeters = 1e-6;
constexpr double kRuntimeLinearIkOrientationToleranceRadians = 1e-5;
constexpr double kSegmentCompletionFractionTolerance = 1e-12;
constexpr double kMinimumReorientationJointChange = 1e-12;

bool JointInterpolationFollowsTcpLine(
    const JointVector& start,
    const JointVector& end,
    const kinematics::detail::Pose3& startTcp,
    const kinematics::detail::Pose3& endTcp,
    kinematics::LevenbergMarquardtIk& inverse)
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

kinematics::IkOptions RuntimeLinearIkOptions(const models::RobotSpecification& specification)
{
    kinematics::IkOptions options = planning::detail::PathIkOptions(specification);
    // 보조 IK 결과가 명목 TCP 경로에서 벗어나는 정도를 더 작게 제한한다.
    // 같은 허용 오차를 IK 수렴 판정과 후보 자세 검증에 함께 사용한다.
    options.positionToleranceMeters = kRuntimeLinearIkPositionToleranceMeters;
    options.orientationToleranceRadians = kRuntimeLinearIkOrientationToleranceRadians;
    return options;
}

double TrapezoidDistanceFraction(double timeFraction, double rampTimeFraction)
{
    // 프로파일은 시간의 앞·뒤 rampTimeFraction 비율에서 속도를 선형으로 올리고 내리며, 가운데는 최대 속도를 유지한다.
    // 속도를 적분한 뒤 전체 면적 1-ramp와 비교해 경로 전체가 정확히 0부터 1까지 진행하도록 정규화한다.
    const double ramp = std::clamp(rampTimeFraction,
        detail::kMinimumPathRampFraction, detail::kMaximumPathRampFraction);
    const double u = std::clamp(timeFraction, 0.0, 1.0);
    // 가속·등속·감속 구간의 속도 그래프 면적을 더한다. 각 구간의 시간이 0이면 면적도 0이 된다.
    const double accelerationTime = std::min(u, ramp);
    const double accelerationArea = accelerationTime * accelerationTime / (2.0 * ramp);
    const double cruiseTime = std::clamp(u - ramp, 0.0, 1.0 - 2.0 * ramp);
    const double decelerationTime = std::clamp(u - (1.0 - ramp), 0.0, ramp);
    const double decelerationArea = decelerationTime -
        decelerationTime * decelerationTime / (2.0 * ramp);
    const double integratedSpeed = accelerationArea + cruiseTime + decelerationArea;
    return std::clamp(integratedSpeed / (1.0 - ramp), 0.0, 1.0);
}
}

void SimRobotController::ConfigureJointMove(
    JointVector targetPositions, double velocityScale, double accelerationScale)
{
    jointMotion_.ResetProgress();
    jointMotion_.targetPositions = std::move(targetPositions);
    jointMotion_.startPositions = state_.jointPositions;
    jointMotion_.progressAcceleration = std::numeric_limits<double>::infinity();
    linearMotion_.ClearPath();

    jointMotion_.progressRate = std::numeric_limits<double>::infinity();
    bool needsMotion = false;
    for (std::size_t joint = 0; joint < specification_.jointCount; ++joint)
    {
        const double distance = std::abs(jointMotion_.targetPositions[joint] - jointMotion_.startPositions[joint]);
        if (distance <= detail::kMotionEpsilon)
            continue;
        needsMotion = true;
        const auto& limits = specification_.joints[joint];
        jointMotion_.progressRate = std::min(jointMotion_.progressRate,
            limits.maximumVelocity * velocityScale / distance);
        const double acceleration = limits.maximumVelocity /
            detail::kJointVelocityRampSeconds * accelerationScale;
        jointMotion_.progressAcceleration = std::min(
            jointMotion_.progressAcceleration, acceleration / distance);
    }

    state_.mode = needsMotion ? RobotMode::Moving : RobotMode::Idle;
    if (!needsMotion)
    {
        jointMotion_.progressRate = 0.0;
        jointMotion_.progressAcceleration = 0.0;
        state_.jointPositions = jointMotion_.targetPositions;
        std::fill(state_.jointVelocities.begin(), state_.jointVelocities.end(), 0.0);
        RefreshTcp();
    }
    else
    {
        const double accelerationTimeToMaximum = jointMotion_.progressRate /
            jointMotion_.progressAcceleration;
        const double accelerationDistanceToMaximum = 0.5 * jointMotion_.progressAcceleration *
            accelerationTimeToMaximum * accelerationTimeToMaximum;
        if (2.0 * accelerationDistanceToMaximum >= 1.0)
        {
            jointMotion_.accelerationTimeSeconds = std::sqrt(1.0 / jointMotion_.progressAcceleration);
            jointMotion_.peakProgressVelocity = jointMotion_.progressAcceleration *
                jointMotion_.accelerationTimeSeconds;
            jointMotion_.cruiseTimeSeconds = 0.0;
        }
        else
        {
            jointMotion_.accelerationTimeSeconds = accelerationTimeToMaximum;
            jointMotion_.peakProgressVelocity = jointMotion_.progressRate;
            jointMotion_.cruiseTimeSeconds = (1.0 - 2.0 * accelerationDistanceToMaximum) /
                jointMotion_.peakProgressVelocity;
        }
        jointMotion_.profileDurationSeconds = 2.0 * jointMotion_.accelerationTimeSeconds +
            jointMotion_.cruiseTimeSeconds;
    }
}

void SimRobotController::Update(double dtSeconds)
{
    // FixedControlLoop 같은 호출자가 정한 tick마다 q/dq와 모델 FK의 TCP를 갱신한다. 화면 Entity와 물리 Body 반영은 별도 계층의 책임이다.
    if (!connected_ || state_.mode != RobotMode::Moving ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        return;
    }

    if (!linearMotion_.path.empty())
    {
        UpdateLinear(dtSeconds);
        return;
    }

    AdvanceJointMove(dtSeconds);
    FinishJointMoveIfNeeded();
    RefreshTcp();
}

void SimRobotController::AdvanceJointMove(double dtSeconds)
{
    double nextProgress = jointMotion_.progress;
    double nextVelocity = 0.0;
    if (jointMotion_.braking)
    {
        nextVelocity = std::max(0.0,
            jointMotion_.progressVelocity - jointMotion_.progressAcceleration * dtSeconds);
        nextProgress += (jointMotion_.progressVelocity * jointMotion_.progressVelocity -
            nextVelocity * nextVelocity) / (2.0 * jointMotion_.progressAcceleration);
        if (nextProgress >= jointMotion_.stopProgress || nextVelocity == 0.0)
        {
            nextProgress = jointMotion_.stopProgress;
            nextVelocity = 0.0;
        }
    }
    else
    {
        jointMotion_.elapsedSeconds = std::min(jointMotion_.profileDurationSeconds,
            jointMotion_.elapsedSeconds + dtSeconds);
        const double elapsed = jointMotion_.elapsedSeconds;
        const double accelerationDistance = 0.5 * jointMotion_.progressAcceleration *
            jointMotion_.accelerationTimeSeconds * jointMotion_.accelerationTimeSeconds;
        if (elapsed < jointMotion_.accelerationTimeSeconds)
        {
            nextProgress = 0.5 * jointMotion_.progressAcceleration * elapsed * elapsed;
            nextVelocity = jointMotion_.progressAcceleration * elapsed;
        }
        else if (elapsed < jointMotion_.accelerationTimeSeconds + jointMotion_.cruiseTimeSeconds)
        {
            nextProgress = accelerationDistance + jointMotion_.peakProgressVelocity *
                (elapsed - jointMotion_.accelerationTimeSeconds);
            nextVelocity = jointMotion_.peakProgressVelocity;
        }
        else
        {
            const double decelerationTime = jointMotion_.profileDurationSeconds - elapsed;
            nextProgress = 1.0 - 0.5 * jointMotion_.progressAcceleration *
                decelerationTime * decelerationTime;
            nextVelocity = jointMotion_.progressAcceleration * decelerationTime;
        }
    }
    nextProgress = std::clamp(nextProgress, jointMotion_.progress, jointMotion_.braking ?
        jointMotion_.stopProgress : 1.0);
    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const double start = jointMotion_.startPositions[i];
        const double delta = jointMotion_.targetPositions[i] - start;
        state_.jointPositions[i] = start + delta * nextProgress;
        state_.jointVelocities[i] = delta * nextVelocity;
    }
    jointMotion_.progress = nextProgress;
    jointMotion_.progressVelocity = nextVelocity;
}

void SimRobotController::FinishJointMoveIfNeeded()
{
    if (jointMotion_.braking && jointMotion_.progress >= jointMotion_.stopProgress - detail::kMotionEpsilon)
    {
        jointMotion_.progressVelocity = 0.0;
        std::fill(state_.jointVelocities.begin(),
            state_.jointVelocities.end(), 0.0);
        for (std::size_t i = 0; i < specification_.jointCount; ++i)
            state_.jointPositions[i] = jointMotion_.startPositions[i] +
                (jointMotion_.targetPositions[i] - jointMotion_.startPositions[i]) * jointMotion_.stopProgress;
        if (jointMotion_.pendingMove)
        {
            PendingJointMove nextMove = std::move(*jointMotion_.pendingMove);
            jointMotion_.pendingMove.reset();
            ConfigureJointMove(std::move(nextMove.targetPositions),
                nextMove.velocityScale, nextMove.accelerationScale);
        }
        return;
    }

    if (jointMotion_.progress < 1.0)
        return;

    state_.jointPositions = jointMotion_.targetPositions;
    jointMotion_.progressVelocity = 0.0;
    std::fill(state_.jointVelocities.begin(), state_.jointVelocities.end(), 0.0);
    if (posePathSegment_ + 1 < posePathPoints_.size())
    {
        ++posePathSegment_;
        ConfigureJointMove(posePathPoints_[posePathSegment_], pendingPoseMove_.velocityScale,
            pendingPoseMove_.accelerationScale);
        return;
    }

    posePathPoints_.clear();
    posePathSegment_ = 0;
    state_.mode = RobotMode::Idle;
}

void SimRobotController::UpdateLinear(double dtSeconds)
{
    using namespace kinematics::detail;
    // 속도를 계산할 수 있도록 tick 시작 위치를 보관한다.
    std::copy(state_.jointPositions.begin(), state_.jointPositions.end(),
        previousJointPositions_.begin());
    double currentPathTime = linearMotion_.completedPathDurationSeconds;
    if (linearMotion_.segment < linearMotion_.path.size())
        currentPathTime += linearMotion_.segmentFraction * linearMotion_.path[linearMotion_.segment].durationSeconds;
    linearMotion_.profileElapsedSeconds = std::min(linearMotion_.profileDurationSeconds,
        linearMotion_.profileElapsedSeconds + dtSeconds);
    const double profileEnd = linearMotion_.profileElapsedSeconds / linearMotion_.profileDurationSeconds;
    const double desiredPathTime = linearMotion_.plannedDurationSeconds *
        TrapezoidDistanceFraction(profileEnd, linearMotion_.profileRampFraction);
    double pathTimeBudget = std::max(0.0, desiredPathTime - currentPathTime);
    double remaining = dtSeconds;
    bool failed = false;
    while (remaining > 0.0 && pathTimeBudget > 0.0 && linearMotion_.segment < linearMotion_.path.size())
    {
        const double available = remaining;
        double usedSeconds = 0.0;
        // 유한한 양수 시간도 표현 가능한 경로 비율보다 작을 수 있다. 나눗셈의 0 분모나 진행 없는 반복 대신 현재 자세를 유지하고 다음 tick을 기다린다.
        if (!std::isfinite(available) || available <= 0.0)
            break;
        if (!TryAdvanceLinearSegment(available, pathTimeBudget, usedSeconds))
        {
            // 특이 자세에서 일반 관절 보간으로 TCP 직선을 벗어나면, 같은 TCP 자세를 유지하는 보조 관절 이동을 시도한다.
            // 보조 이동도 관절과 TCP의 속도, 충돌, FK 오차를 다시 확인한다.
            if (!TryAdjustJointsAlongLinearPath(
                linearMotion_.path[linearMotion_.segment].joints, available))
            {
                failed = true;
                break;
            }
            remaining = 0.0;
            continue;
        }
        remaining = usedSeconds >= remaining ? 0.0 : remaining - usedSeconds;
        if (linearMotion_.segmentFraction >= 1.0 - kSegmentCompletionFractionTolerance)
        {
            linearMotion_.completedPathDurationSeconds +=
                linearMotion_.path[linearMotion_.segment].durationSeconds;
            linearMotion_.segmentFraction = 0.0;
            ++linearMotion_.segment;
        }
        else
        {
            // 속도 때문에 경로 진행을 줄였으면 이번 시간은 모두 소비했다. 남은 경로 시간은 다음 Update가 이어서 처리한다.
            remaining = 0.0;
        }
    }
    if (failed)
    {
        jointMotion_.targetPositions = state_.jointPositions;
        linearMotion_.ClearPath();
        state_.mode = RobotMode::Fault;
        state_.errorCode = ErrorCode::IkDidNotConverge;
    }
    else if (linearMotion_.segment >= linearMotion_.path.size())
    {
        linearMotion_.ClearPath();
        state_.mode = RobotMode::Idle;
    }
    for (std::size_t joint = 0; joint < specification_.jointCount; ++joint)
    {
        const double previousPosition = previousJointPositions_[joint];
        state_.jointVelocities[joint] = state_.mode == RobotMode::Moving ?
            (state_.jointPositions[joint] - previousPosition) / dtSeconds : 0.0;
    }
    RefreshTcp();
}

bool SimRobotController::TryAdvanceLinearSegment(double availableSeconds,
    double& pathTimeBudget, double& usedSeconds)
{
    using namespace kinematics::detail;

    const auto& begin = linearMotion_.path[linearMotion_.segment - 1];
    const auto& end = linearMotion_.path[linearMotion_.segment];
    const double segmentRemaining = (1.0 - linearMotion_.segmentFraction) * end.durationSeconds;
    double fractionStep = std::min(pathTimeBudget, segmentRemaining) / end.durationSeconds;
    if (fractionStep <= 0.0 || linearMotion_.segmentFraction + fractionStep == linearMotion_.segmentFraction)
        return true;

    const Pose3 currentTcp = FromCartesian(inverse_.EvaluateTcp(state_.jointPositions));
    for (std::size_t attempt = 0;
        attempt < detail::kDefaultSimulationMotionPolicy.linearRuntimeRetryAttempts; ++attempt)
    {
        const double nextSegmentFraction = std::min(1.0, linearMotion_.segmentFraction + fractionStep);
        if (nextSegmentFraction == linearMotion_.segmentFraction)
        {
            usedSeconds = availableSeconds;
            return true;
        }

        // 명령을 받을 때 직선 TCP 목표마다 IK와 충돌 검사를 끝낸 경로 표본 사이를 보간해 매 tick IK 반복을 없앤다.
        for (std::size_t joint = 0; joint < specification_.jointCount; ++joint)
            linearInterpolationBuffer_[joint] = begin.joints[joint] +
                (end.joints[joint] - begin.joints[joint]) * nextSegmentFraction;

        double requiredSeconds = 0.0;
        for (std::size_t joint = 0; joint < specification_.jointCount; ++joint)
            requiredSeconds = std::max(requiredSeconds, planning::detail::RequiredTimeForVelocity(
                linearInterpolationBuffer_[joint] - state_.jointPositions[joint],
                specification_.joints[joint].maximumVelocity));

        const Pose3 nextTcp = FromCartesian(inverse_.EvaluateTcp(linearInterpolationBuffer_));
        requiredSeconds = std::max(requiredSeconds,
            planning::detail::RequiredTimeForVelocity(
                Length(Subtract(nextTcp.positionMeters, currentTcp.positionMeters)), linearMotion_.linearVelocityLimit));
        requiredSeconds = std::max(requiredSeconds,
            planning::detail::RequiredTimeForVelocity(
                Length(RotationError(nextTcp.rotation, currentTcp.rotation)), linearMotion_.angularVelocityLimit));

        const double ratio = std::max(1.0, requiredSeconds / availableSeconds);
        if (ratio <= 1.0 + detail::kVelocityRatioTolerance)
        {
            state_.jointPositions = linearInterpolationBuffer_;
            linearMotion_.segmentFraction = nextSegmentFraction;
            // 표본 끝에 남은 명목 시간이 아주 작아도 IK 허용 오차를 넘는 보정에는 실제 시간이 필요하다.
            // 남은 실제 tick 전체를 속도 예산으로 쓰고, 명목 진행 시간과 실제 변화에 필요한 시간 중 큰 값을 소비한다.
            usedSeconds = std::min(availableSeconds,
                std::max(fractionStep * end.durationSeconds, requiredSeconds));
            pathTimeBudget -= fractionStep * end.durationSeconds;
            return true;
        }

        // 실제 관절 변화가 속도 상한을 넘으면 진행 비율을 줄여 다시 확인한다.
        fractionStep *= detail::kDefaultSimulationMotionPolicy.runtimeRetryProgressScale / ratio;
    }
    return false;
}

bool SimRobotController::TryAdjustJointsAlongLinearPath(const JointVector& plannedJoints, double availableSeconds)
{
    using namespace kinematics::detail;
    const JointVector current = state_.jointPositions;
    const Pose3 currentTcp = FromCartesian(inverse_.EvaluateTcp(current));
    const auto& begin = linearMotion_.path[linearMotion_.segment - 1];
    const auto& end = linearMotion_.path[linearMotion_.segment];
    const Pose3 pathTcp = Interpolate(
        FromCartesian(begin.tcpPose), FromCartesian(end.tcpPose), linearMotion_.segmentFraction);
    const auto options = RuntimeLinearIkOptions(specification_);
    double attraction = 1.0;
    double previousDistanceSquared = 0.0;
    for (std::size_t i = 0; i < current.size(); ++i)
    {
        const double delta = plannedJoints[i] - current[i];
        previousDistanceSquared += delta * delta;
        if (std::abs(delta) > 0.0)
            attraction = std::min(attraction, specification_.joints[i].maximumVelocity * availableSeconds / std::abs(delta));
    }
    // 시작 자세를 계획된 관절 자세 쪽으로 조금 옮겨 IK를 푼다. 이 보정은 다음 경로 표본으로 움직이는 동안 수행한다.
    // IK가 TCP 위치와 방향을 다시 맞추는 동안, TCP를 거의 움직이지 않고 관절 자세를 바꿀 수 있는 여유 방향을 찾는다.
    // 일반 모델에서 단순 관절 보간이 TCP를 고정한다고 가정하지 않으므로 후보마다 FK와 속도를 다시 확인한다.
    for (std::size_t attempt = 0;
        attempt < detail::kDefaultSimulationMotionPolicy.reorientationRetryAttempts; ++attempt)
    {
        JointVector seed = current;
        for (std::size_t i = 0; i < current.size(); ++i)
            seed[i] += (plannedJoints[i] - current[i]) * attraction;
        // 이전 계산의 FK 결과가 아니라 경로에서 정한 TCP를 목표로 삼아 작은 IK 오차가 다음 tick에 쌓이지 않게 한다.
        const auto solution = inverse_.Solve(ToCartesian(pathTcp), seed, options);
        if (!solution)
        {
            attraction *= 0.5;
            continue;
        }

        double ratio = 1.0;
        double nextDistanceSquared = 0.0;
        double movement = 0.0;
        for (std::size_t i = 0; i < current.size(); ++i)
        {
            const double change = solution.jointPositions[i] - current[i];
            ratio = std::max(ratio, planning::detail::VelocityRatio(
                change, availableSeconds, specification_.joints[i].maximumVelocity));
            movement = std::max(movement, std::abs(change));
            const double residual = plannedJoints[i] - solution.jointPositions[i];
            nextDistanceSquared += residual * residual;
        }
        const Pose3 nextTcp = FromCartesian(inverse_.EvaluateTcp(solution.jointPositions));
        const double pathPositionError = Length(Subtract(nextTcp.positionMeters, pathTcp.positionMeters));
        const double pathOrientationError = Length(RotationError(nextTcp.rotation, pathTcp.rotation));
        ratio = std::max(ratio, planning::detail::VelocityRatio(
            Length(Subtract(nextTcp.positionMeters, currentTcp.positionMeters)),
            availableSeconds, linearMotion_.linearVelocityLimit));
        ratio = std::max(ratio, planning::detail::VelocityRatio(
            Length(RotationError(nextTcp.rotation, currentTcp.rotation)),
            availableSeconds, linearMotion_.angularVelocityLimit));
        const bool pathValid = planning::ValidateJointPath(
            current, solution.jointPositions, specification_, jointStateValidityChecker_) ==
            planning::JointStateInvalidity::None;
        const bool reorientationFollowsTcp = JointInterpolationFollowsTcpLine(
            current, solution.jointPositions, pathTcp, pathTcp, inverse_);
        const Pose3 plannedEndTcp = FromCartesian(end.tcpPose);
        const bool remainingLineFollowsTcp = JointInterpolationFollowsTcpLine(
            solution.jointPositions, end.joints, pathTcp, plannedEndTcp, inverse_);
        const bool respectsVelocityLimits = ratio <= 1.0 + detail::kVelocityRatioTolerance;
        const bool makesProgress = movement > kMinimumReorientationJointChange &&
            nextDistanceSquared < previousDistanceSquared;
        const bool staysOnTcpPath = pathPositionError <= options.positionToleranceMeters &&
            pathOrientationError <= options.orientationToleranceRadians;
        if (respectsVelocityLimits && makesProgress && staysOnTcpPath && pathValid &&
            reorientationFollowsTcp && remainingLineFollowsTcp)
        {
            state_.jointPositions = solution.jointPositions;
            return true;
        }
        attraction *= 0.5;
    }
    return false;
}

} // namespace grasplink::robotics::backends::simulation
