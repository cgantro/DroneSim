#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"
#include "robotics/kinematics/detail/PoseMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace grasplink::robotics::kinematics
{
namespace
{
using namespace detail;
using SixVector = std::array<double, 6>;
using SixMatrix = std::array<SixVector, 6>;

constexpr std::size_t kMaximumDampingAttempts = 8;
constexpr std::size_t kMaximumLineSearchSteps = 10;
constexpr double kSingularSystemDampingGrowth = 10.0;
constexpr double kRejectedStepDampingGrowth = 4.0;
constexpr double kAcceptedStepDampingDecay = 0.5;
constexpr double kMinimumDampingRatio = 0.01;
// 계산 중 가장 큰 수가 이 값보다 작으면 거의 0으로 나누는 셈이므로 풀이를 멈춘다.
constexpr double kNormalEquationPivotCutoff = 1e-24;

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

bool SolveSystem(SixMatrix matrix, SixVector right, SixVector& answer)
{
    // 가장 큰 값을 피벗으로 골라 나눗셈 오차를 줄이며 TCP의 6개 오차를 푼다.
    for (std::size_t column = 0; column < 6; ++column)
    {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < 6; ++row)
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column]))
                pivot = row;
        if (!std::isfinite(matrix[pivot][column]) ||
            std::abs(matrix[pivot][column]) <= kNormalEquationPivotCutoff)
            return false;
        std::swap(matrix[pivot], matrix[column]);
        std::swap(right[pivot], right[column]);
        for (std::size_t row = column + 1; row < 6; ++row)
        {
            const double factor = matrix[row][column] / matrix[column][column];
            for (std::size_t entry = column; entry < 6; ++entry)
                matrix[row][entry] -= factor * matrix[column][entry];
            right[row] -= factor * right[column];
        }
    }
    for (int row = 5; row >= 0; --row)
    {
        double residual = right[static_cast<std::size_t>(row)];
        for (std::size_t column = static_cast<std::size_t>(row) + 1; column < 6; ++column)
            residual -= matrix[static_cast<std::size_t>(row)][column] * answer[column];
        answer[static_cast<std::size_t>(row)] = residual / matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
        if (!std::isfinite(answer[static_cast<std::size_t>(row)]))
            return false;
    }
    return true;
}

struct Error
{
    SixVector weighted{};
    double position = 0.0;
    double orientation = 0.0;
    double cost = 0.0;
};

Error Measure(const Pose3& target, const Pose3& current, double weight)
{
    const Vec3 position = Subtract(target.positionMeters, current.positionMeters);
    const Vec3 rotation = RotationError(target.rotation, current.rotation);
    Error error;
    error.position = Length(position);
    error.orientation = Length(rotation);
    error.weighted = {position.x, position.y, position.z, rotation.x * weight, rotation.y * weight, rotation.z * weight};
    for (double entry : error.weighted)
        error.cost += entry * entry;
    return error;
}

std::pair<double, double> JacobianCondition(const std::vector<SixVector>& columns)
{
    SixMatrix gram{};
    for (const auto& column : columns)
        for (std::size_t row = 0; row < 6; ++row)
            for (std::size_t other = 0; other < 6; ++other)
                gram[row][other] += column[row] * column[other];

    for (std::size_t sweep = 0; sweep < 64; ++sweep)
    {
        std::size_t p = 0;
        std::size_t q = 1;
        double largestOffDiagonal = 0.0;
        for (std::size_t row = 0; row < 6; ++row)
            for (std::size_t column = row + 1; column < 6; ++column)
                if (std::abs(gram[row][column]) > largestOffDiagonal)
                {
                    largestOffDiagonal = std::abs(gram[row][column]);
                    p = row;
                    q = column;
                }
        if (largestOffDiagonal <= 1e-14)
            break;

        const double angle = 0.5 * std::atan2(2.0 * gram[p][q], gram[q][q] - gram[p][p]);
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        const double pp = gram[p][p];
        const double qq = gram[q][q];
        const double pq = gram[p][q];
        gram[p][p] = cosine * cosine * pp - 2.0 * sine * cosine * pq + sine * sine * qq;
        gram[q][q] = sine * sine * pp + 2.0 * sine * cosine * pq + cosine * cosine * qq;
        gram[p][q] = gram[q][p] = 0.0;
        for (std::size_t index = 0; index < 6; ++index)
        {
            if (index == p || index == q)
                continue;
            const double ip = gram[index][p];
            const double iq = gram[index][q];
            gram[index][p] = gram[p][index] = cosine * ip - sine * iq;
            gram[index][q] = gram[q][index] = sine * ip + cosine * iq;
        }
    }

    double minimumEigenvalue = std::numeric_limits<double>::infinity();
    double maximumEigenvalue = 0.0;
    for (std::size_t axis = 0; axis < 6; ++axis)
    {
        const double eigenvalue = std::max(0.0, gram[axis][axis]);
        minimumEigenvalue = std::min(minimumEigenvalue, eigenvalue);
        maximumEigenvalue = std::max(maximumEigenvalue, eigenvalue);
    }
    const double minimumSingularValue = std::sqrt(minimumEigenvalue);
    const double maximumSingularValue = std::sqrt(maximumEigenvalue);
    const double conditionNumber = minimumSingularValue > 1e-12
        ? maximumSingularValue / minimumSingularValue
        : std::numeric_limits<double>::infinity();
    return {minimumSingularValue, conditionNumber};
}
}

DampedLeastSquaresIk::DampedLeastSquaresIk(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame)
    : specification_(specification), forward_(specification), tcpInToolFrame_(tcpInToolFrame),
      anglesScratch_(specification.jointCount), jointStepScratch_(specification.jointCount),
      candidateScratch_(specification.jointCount), jacobianColumnsScratch_(specification.jointCount)
{
    if (!Finite(tcpInToolFrame_.positionMeters))
        throw std::invalid_argument("DampedLeastSquaresIk: invalid TCP offset");
    tcpInToolFrame_.rotation = Normalize(tcpInToolFrame_.rotation);
    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        if (!std::isfinite(joint.minPositionRadians) || !std::isfinite(joint.maxPositionRadians) ||
            joint.minPositionRadians > joint.maxPositionRadians)
            throw std::invalid_argument("DampedLeastSquaresIk: invalid joint limits");
        if (i > 0)
            maximumReachMeters_ += Length(Subtract(joint.bindPivotMeters, specification_.joints[i - 1].bindPivotMeters));
    }
    // 첫 관절 중심을 기준으로 링크 사이 길이와 ToolFrame/TCP offset 길이를 모두 더한다. 회전이나 관절 제한을 무시한 넉넉한 구이므로 이 거리 밖은 확실히 도달할 수 없다.
    if (specification_.hasToolFrame)
        maximumReachMeters_ += Length(specification_.toolFrameInLastJoint.positionMeters) + Length(tcpInToolFrame_.positionMeters);
    if (!std::isfinite(maximumReachMeters_))
        throw std::invalid_argument("DampedLeastSquaresIk: non-finite robot reach");
}

CartesianPose DampedLeastSquaresIk::EvaluateTcp(const JointVector& jointPositionRadians)
{
    if (!specification_.hasToolFrame)
        throw std::invalid_argument("DampedLeastSquaresIk: missing ToolFrame");
    return ToCartesian(Compose(forward_.Update(jointPositionRadians).toolFrameInBaseFrame, tcpInToolFrame_));
}

IkResult DampedLeastSquaresIk::SolveSingleSeed(const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options)
{
    const SessionId session = BeginSingleSeed(targetInBase, currentSeed, options);
    while (StepSingleSeed(session, options.maxIterations) == IkSessionState::Running) {}
    return GetSingleSeedResult(session);
}

DampedLeastSquaresIk::SessionId DampedLeastSquaresIk::BeginSingleSeed(
    const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options)
{
    if (++nextSessionId_ == 0)
        ++nextSessionId_;
    activeSessionId_ = nextSessionId_;
    sessionState_ = IkSessionState::Running;
    sessionOptions_ = options;
    sessionIteration_ = 0;
    sessionDamping_ = options.damping;
    sessionActiveJointLimit_ = false;
    sessionStagnantIterations_ = 0;
    sessionResult_ = {};
    sessionResult_.positionErrorMeters = std::numeric_limits<double>::infinity();
    sessionResult_.orientationErrorRadians = std::numeric_limits<double>::infinity();
    const auto fail = [&](IkStatus status, const char* message) {
        sessionResult_.status = status;
        sessionResult_.message = message;
        sessionResult_.terminationReason = status == IkStatus::MissingToolFrame
            ? IkTerminationReason::MissingToolFrame : IkTerminationReason::InvalidInput;
        sessionState_ = IkSessionState::Completed;
    };
    if (currentSeed.size() != specification_.jointCount || options.maxIterations == 0 ||
        !Positive(options.positionToleranceMeters) || !Positive(options.orientationToleranceRadians) ||
        !Positive(options.damping) || !Positive(options.orientationWeightMetersPerRadian) ||
        !Positive(options.maxJointStepRadians) || options.stagnationIterationLimit == 0 ||
        !Positive(options.stagnationRelativeCostTolerance) ||
        !Positive(options.stagnationJointStepToleranceRadians))
    {
        fail(IkStatus::InvalidInput, "IK: invalid seed size or solver options");
        return activeSessionId_;
    }
    for (std::size_t i = 0; i < currentSeed.size(); ++i)
    {
        if (!std::isfinite(currentSeed[i]) || currentSeed[i] < specification_.joints[i].minPositionRadians ||
            currentSeed[i] > specification_.joints[i].maxPositionRadians)
        {
            fail(IkStatus::InvalidInput, "IK: seed outside joint limits or non-finite");
            return activeSessionId_;
        }
    }
    try { sessionTarget_ = FromCartesian(targetInBase); }
    catch (const std::invalid_argument&)
    {
        fail(IkStatus::InvalidInput, "IK: invalid target pose");
        return activeSessionId_;
    }
    sessionResult_.jointPositionRadians = currentSeed;
    if (!specification_.hasToolFrame)
    {
        fail(IkStatus::MissingToolFrame, "IK: robot specification has no ToolFrame");
        return activeSessionId_;
    }
    const double targetRadius = Length(Subtract(sessionTarget_.positionMeters, specification_.joints[0].bindPivotMeters));
    if (!std::isfinite(targetRadius) || targetRadius > maximumReachMeters_ + options.positionToleranceMeters)
    {
        sessionResult_.status = IkStatus::Unreachable;
        sessionResult_.terminationReason = IkTerminationReason::ConservativeReachExceeded;
        sessionResult_.message = "IK: target radius " + std::to_string(targetRadius) +
            " m exceeds conservative robot reach " + std::to_string(maximumReachMeters_) + " m";
        sessionState_ = IkSessionState::Completed;
        return activeSessionId_;
    }

    std::copy(currentSeed.begin(), currentSeed.end(), anglesScratch_.begin());
    return activeSessionId_;
}

IkSessionState DampedLeastSquaresIk::StepSingleSeed(SessionId session, std::size_t iterationBudget)
{
    if (session != activeSessionId_)
        throw std::invalid_argument("IK: session is no longer active");
    if (sessionState_ != IkSessionState::Running || iterationBudget == 0)
        return sessionState_;

    auto& angles = anglesScratch_;
    auto& jointStep = jointStepScratch_;
    auto& candidate = candidateScratch_;
    auto& columns = jacobianColumnsScratch_;
    const auto finish = [&](IkStatus status, const char* message) {
        sessionResult_.status = status;
        sessionResult_.message = message;
        sessionResult_.terminationReason = status == IkStatus::JointLimitReached
            ? IkTerminationReason::ActiveJointLimit
            : (status == IkStatus::DidNotConverge ? IkTerminationReason::Stagnation : IkTerminationReason::InvalidInput);
        if (status != IkStatus::Success && angles.size() == specification_.jointCount)
        {
            const auto& state = forward_.Update(angles);
            const Pose3 current = Compose(state.toolFrameInBaseFrame, tcpInToolFrame_);
            for (std::size_t joint = 0; joint < columns.size(); ++joint)
            {
                const Vec3 axis = state.jointAxesInBaseFrame[joint];
                const Vec3 linear = Cross(axis, Subtract(current.positionMeters,
                    state.linkPosesInBaseFrame[joint].positionMeters));
                columns[joint] = {linear.x, linear.y, linear.z,
                    axis.x * sessionOptions_.orientationWeightMetersPerRadian,
                    axis.y * sessionOptions_.orientationWeightMetersPerRadian,
                    axis.z * sessionOptions_.orientationWeightMetersPerRadian};
            }
            const auto [minimum, condition] = JacobianCondition(columns);
            sessionResult_.weightedJacobianMinimumSingularValue = minimum;
            sessionResult_.weightedJacobianConditionNumber = condition;
        }
        sessionState_ = IkSessionState::Completed;
    };
    std::size_t iterationsUsed = 0;
    while (sessionState_ == IkSessionState::Running && iterationsUsed < iterationBudget)
    {
        const auto& state = forward_.Update(angles);
        const Pose3 current = Compose(state.toolFrameInBaseFrame, tcpInToolFrame_);
        const Error error = Measure(sessionTarget_, current, sessionOptions_.orientationWeightMetersPerRadian);
        sessionResult_.jointPositionRadians = angles;
        sessionResult_.iterations = sessionIteration_;
        sessionResult_.positionErrorMeters = error.position;
        sessionResult_.orientationErrorRadians = error.orientation;
        if (error.position <= sessionOptions_.positionToleranceMeters &&
            error.orientation <= sessionOptions_.orientationToleranceRadians)
        {
            sessionResult_.status = IkStatus::Success;
            sessionResult_.terminationReason = IkTerminationReason::Converged;
            sessionState_ = IkSessionState::Completed;
            break;
        }
        if (!std::isfinite(error.cost))
        {
            finish(IkStatus::InvalidInput, "IK: target or solver scaling exceeds numeric range");
            break;
        }
        if (sessionIteration_ == sessionOptions_.maxIterations)
        {
            finish(sessionActiveJointLimit_ ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
                sessionActiveJointLimit_ ? "IK: iteration limit while constrained by joint limits" : "IK: iteration limit reached");
            if (!sessionActiveJointLimit_)
                sessionResult_.terminationReason = IkTerminationReason::IterationLimit;
            break;
        }

        for (auto& column : columns)
            column.fill(0.0);
    // Jacobian은 관절을 조금 돌렸을 때 TCP의 위치와 방향이 얼마나 바뀌는지 적은 표다.
    // 관절마다 표의 열 하나를 만들므로 관절 수가 달라도 같은 계산을 쓴다. TCP offset도 계산에 포함한다.
        for (std::size_t i = 0; i < columns.size(); ++i)
        {
            const Vec3 axis = state.jointAxesInBaseFrame[i];
            const Vec3 linear = Cross(axis, Subtract(current.positionMeters, state.linkPosesInBaseFrame[i].positionMeters));
            columns[i] = {linear.x, linear.y, linear.z,
                axis.x * sessionOptions_.orientationWeightMetersPerRadian,
                axis.y * sessionOptions_.orientationWeightMetersPerRadian,
                axis.z * sessionOptions_.orientationWeightMetersPerRadian};
        }

        bool improved = false;
        bool activeLimitBlockedThisIteration = false;
        bool solvedDampedSystem = false;
        for (std::size_t attempt = 0; attempt < kMaximumDampingAttempts && !improved; ++attempt)
        {
            std::array<bool, 6> active{};
            bool feasibleDirection = false;
            double stepScale = 1.0;
            for (std::size_t activePass = 0; activePass <= angles.size(); ++activePass)
            {
                SixMatrix normal{};
                for (std::size_t row = 0; row < 6; ++row)
                {
                    for (std::size_t column = 0; column < 6; ++column)
                        for (std::size_t joint = 0; joint < columns.size(); ++joint)
                            if (!active[joint])
                                normal[row][column] += columns[joint][row] * columns[joint][column];
                    normal[row][row] += sessionDamping_ * sessionDamping_;
                }
                SixVector taskStep{};
                if (!SolveSystem(normal, error.weighted, taskStep))
                    break;
                solvedDampedSystem = true;
                std::fill(jointStep.begin(), jointStep.end(), 0.0);
                double largest = 0.0;
                for (std::size_t joint = 0; joint < angles.size(); ++joint)
                {
                    if (active[joint])
                        continue;
                    for (std::size_t row = 0; row < 6; ++row)
                        jointStep[joint] += columns[joint][row] * taskStep[row];
                    largest = std::max(largest, std::abs(jointStep[joint]));
                }
                if (!std::isfinite(largest))
                {
                    finish(IkStatus::DidNotConverge, "IK: non-finite numerical step");
                    break;
                }
                stepScale = largest > sessionOptions_.maxJointStepRadians
                    ? sessionOptions_.maxJointStepRadians / largest : 1.0;

                bool addedConstraint = false;
                for (std::size_t joint = 0; joint < angles.size(); ++joint)
                {
                    if (active[joint])
                        continue;
                    const auto& limits = specification_.joints[joint];
                    const double boundaryTolerance = std::max(1e-10,
                        (limits.maxPositionRadians - limits.minPositionRadians) * 1e-10);
                    const double requested = angles[joint] + jointStep[joint] * stepScale;
                    if ((angles[joint] <= limits.minPositionRadians + boundaryTolerance &&
                            requested < limits.minPositionRadians) ||
                        (angles[joint] >= limits.maxPositionRadians - boundaryTolerance &&
                            requested > limits.maxPositionRadians))
                    {
                        active[joint] = true;
                        addedConstraint = true;
                        activeLimitBlockedThisIteration = true;
                    }
                }
                if (!addedConstraint)
                {
                    feasibleDirection = true;
                    break;
                }
            }
            if (sessionState_ != IkSessionState::Running)
                break;
            if (!feasibleDirection)
            {
                sessionDamping_ *= solvedDampedSystem
                    ? kRejectedStepDampingGrowth : kSingularSystemDampingGrowth;
                continue;
            }

            double feasibleFraction = stepScale;
            for (std::size_t joint = 0; joint < angles.size(); ++joint)
            {
                if (active[joint] || std::abs(jointStep[joint]) <= 1e-15)
                    continue;
                const auto& limits = specification_.joints[joint];
                const double distance = jointStep[joint] > 0.0
                    ? limits.maxPositionRadians - angles[joint]
                    : angles[joint] - limits.minPositionRadians;
                feasibleFraction = std::min(feasibleFraction,
                    std::max(0.0, distance / std::abs(jointStep[joint])));
            }
            for (std::size_t line = 0; line < kMaximumLineSearchSteps && !improved; ++line)
            {
                const double fraction = feasibleFraction * std::ldexp(1.0, -line);
                std::copy(angles.begin(), angles.end(), candidate.begin());
                for (std::size_t i = 0; i < angles.size(); ++i)
                {
                    const double requested = angles[i] + jointStep[i] * fraction;
                    const auto& limits = specification_.joints[i];
                    candidate[i] = std::clamp(requested, limits.minPositionRadians, limits.maxPositionRadians);
                }
                const Error next = Measure(sessionTarget_, Compose(forward_.Update(candidate).toolFrameInBaseFrame, tcpInToolFrame_),
                    sessionOptions_.orientationWeightMetersPerRadian);
                if (next.cost < error.cost)
                {
                    angles.swap(candidate);
    // 오차가 줄면 damping을 낮춰 더 잘 따라가고, 오차가 줄지 않으면 damping을 높여 관절 변화가 튀지 않게 한다.
    // damping을 너무 낮추면 한 번에 움직이는 관절각이 커질 수 있어 별도의 최대 관절 변화량도 적용한다.
                    sessionDamping_ = std::max(sessionOptions_.damping * kMinimumDampingRatio,
                        sessionDamping_ * kAcceptedStepDampingDecay);
                    double largestAcceptedStep = 0.0;
                    for (std::size_t joint = 0; joint < jointStep.size(); ++joint)
                        largestAcceptedStep = std::max(largestAcceptedStep,
                            std::abs(jointStep[joint] * fraction));
                    const double relativeImprovement = (error.cost - next.cost) /
                        std::max(error.cost, std::numeric_limits<double>::min());
                    if (relativeImprovement <= sessionOptions_.stagnationRelativeCostTolerance &&
                        largestAcceptedStep <= sessionOptions_.stagnationJointStepToleranceRadians)
                        ++sessionStagnantIterations_;
                    else
                        sessionStagnantIterations_ = 0;
                    improved = true;
                }
            }
            if (!improved)
                sessionDamping_ *= kRejectedStepDampingGrowth;
        }
        if (sessionState_ != IkSessionState::Running)
            break;
        if (!improved)
        {
            sessionActiveJointLimit_ = activeLimitBlockedThisIteration;
            finish(sessionActiveJointLimit_ ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
                sessionActiveJointLimit_ ? "IK: joint limits block local improvement" : "IK: local iteration stalled");
            break;
        }
        sessionActiveJointLimit_ = activeLimitBlockedThisIteration;
        ++sessionIteration_;
        ++iterationsUsed;
        sessionResult_.iterations = sessionIteration_;
        if (sessionStagnantIterations_ >= sessionOptions_.stagnationIterationLimit)
        {
            finish(sessionActiveJointLimit_ ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
                sessionActiveJointLimit_ ? "IK: progress stalled while constrained by joint limits" : "IK: progress stalled before reaching target");
            break;
        }
    }
    if (sessionState_ == IkSessionState::Running && sessionIteration_ == sessionOptions_.maxIterations)
    {
        const auto& state = forward_.Update(angles);
        const Error error = Measure(sessionTarget_, Compose(state.toolFrameInBaseFrame, tcpInToolFrame_),
            sessionOptions_.orientationWeightMetersPerRadian);
        sessionResult_.jointPositionRadians = angles;
        sessionResult_.iterations = sessionIteration_;
        sessionResult_.positionErrorMeters = error.position;
        sessionResult_.orientationErrorRadians = error.orientation;
        if (error.position <= sessionOptions_.positionToleranceMeters &&
            error.orientation <= sessionOptions_.orientationToleranceRadians)
        {
            sessionResult_.status = IkStatus::Success;
            sessionResult_.terminationReason = IkTerminationReason::Converged;
            sessionState_ = IkSessionState::Completed;
        }
        else
        {
            finish(sessionActiveJointLimit_ ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
                sessionActiveJointLimit_ ? "IK: iteration limit while constrained by joint limits" : "IK: iteration limit reached");
            if (!sessionActiveJointLimit_)
                sessionResult_.terminationReason = IkTerminationReason::IterationLimit;
        }
    }
    return sessionState_;
}

void DampedLeastSquaresIk::CancelSingleSeed(SessionId session)
{
    if (session != activeSessionId_)
        throw std::invalid_argument("IK: session is no longer active");
    if (sessionState_ != IkSessionState::Running)
        return;
    sessionResult_.status = IkStatus::DidNotConverge;
    sessionResult_.terminationReason = IkTerminationReason::Cancelled;
    sessionResult_.message = "IK: cancelled";
    sessionState_ = IkSessionState::Cancelled;
}

const IkResult& DampedLeastSquaresIk::GetSingleSeedResult(SessionId session) const
{
    if (session != activeSessionId_)
        throw std::invalid_argument("IK: session is no longer active");
    return sessionResult_;
}

IkResult DampedLeastSquaresIk::Solve(
    const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options)
{
    IkResult best = SolveSingleSeed(targetInBase, currentSeed, options);
    if (best.Ok() || (best.status != IkStatus::DidNotConverge && best.status != IkStatus::JointLimitReached))
        return best;

    double bestJointDistance = std::numeric_limits<double>::infinity();
    const auto distanceFromSeed = [&](const JointVector& candidate)
    {
        double squaredDistance = 0.0;
        for (std::size_t joint = 0; joint < candidate.size(); ++joint)
        {
            const double delta = candidate[joint] - currentSeed[joint];
            squaredDistance += delta * delta;
        }
        return squaredDistance;
    };
    const auto residual = [&](const IkResult& candidate)
    {
        return candidate.positionErrorMeters +
            options.orientationWeightMetersPerRadian * candidate.orientationErrorRadians;
    };
    // 첫 시드가 막힌 경우에만 HCR의 어깨·팔꿈치·손목 자세를 반사해 제한된 대체 시드를 검사한다.
    for (const auto& alternate : BuildAlternativeIkSeeds(currentSeed, specification_))
    {
        IkResult candidate = SolveSingleSeed(targetInBase, alternate, options);
        if (candidate.Ok())
        {
            const double jointDistance = distanceFromSeed(candidate.jointPositionRadians);
            if (!best.Ok() || jointDistance < bestJointDistance)
            {
                bestJointDistance = jointDistance;
                best = std::move(candidate);
            }
        }
        else if (!best.Ok() && residual(candidate) < residual(best))
            best = std::move(candidate);
    }
    return best;
}

}
