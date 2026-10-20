#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"
#include "robotics/kinematics/detail/PoseMath.h"

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace grasplink::robotics::kinematics
{

IkOptions RobotDefaultIkOptions(const models::RobotSpecification& specification)
{
    IkOptions options;
    if (specification.jointCount < 6)
        options.taskMode = IkTaskMode::PositionOnly;
    return options;
}

namespace
{
using namespace detail;
// LM 계산에 쓰는 고정 기준입니다. 호출자가 조정하는 값은 IkOptions에 둡니다.
struct IkSolverConstants final
{
    static constexpr std::size_t taskDimension = 6;
    static constexpr std::size_t maximumDampingAttempts = 8;
    static constexpr std::size_t maximumLineSearchSteps = 10;
    static constexpr std::size_t maximumJacobiSweeps = 64;
    static constexpr double singularSystemDampingGrowth = 10.0;
    static constexpr double rejectedStepDampingGrowth = 4.0;
    static constexpr double acceptedStepDampingDecay = 0.5;
    static constexpr double minimumDampingRatio = 0.01;
    static constexpr double jointLimitToleranceFloor = 1e-10;
    static constexpr double jointLimitToleranceRatio = 1e-10;
    static constexpr double jacobiConvergenceTolerance = 1e-14;
    static constexpr double minimumSingularValueForCondition = 1e-12;
    static constexpr double negligibleJointStep = 1e-15;
};

struct IkErrorMessages final
{
    inline static constexpr char inactiveSession[] = "IK: session is no longer active";
    inline static constexpr char invalidTcpOffset[] = "LevenbergMarquardtIk: invalid TCP offset";
    inline static constexpr char invalidJointLimits[] = "LevenbergMarquardtIk: invalid joint limits";
    inline static constexpr char nonFiniteRobotReach[] = "LevenbergMarquardtIk: non-finite robot reach";
    inline static constexpr char missingToolFrame[] = "LevenbergMarquardtIk: missing ToolFrame";
    inline static constexpr char invalidSeedOrOptions[] = "IK: invalid seed size or solver options";
    inline static constexpr char seedOutsideJointLimits[] = "IK: seed outside joint limits or non-finite";
    inline static constexpr char invalidTargetPose[] = "IK: invalid target pose";
    inline static constexpr char specificationMissingToolFrame[] = "IK: robot specification has no ToolFrame";
    inline static constexpr char targetRadiusPrefix[] = "IK: target radius ";
    inline static constexpr char targetRadiusExceededReach[] = " m exceeds conservative robot reach ";
    inline static constexpr char metersSuffix[] = " m";
    inline static constexpr char numericRangeExceeded[] = "IK: target or solver scaling exceeds numeric range";
    inline static constexpr char constrainedIterationLimit[] = "IK: iteration limit while constrained by joint limits";
    inline static constexpr char iterationLimitReached[] = "IK: iteration limit reached";
    inline static constexpr char nonFiniteNumericalStep[] = "IK: non-finite numerical step";
    inline static constexpr char jointLimitsBlockImprovement[] = "IK: joint limits block local improvement";
    inline static constexpr char localIterationStalled[] = "IK: local iteration stalled";
    inline static constexpr char progressStalledByJointLimits[] = "IK: progress stalled while constrained by joint limits";
    inline static constexpr char progressStalled[] = "IK: progress stalled before reaching target";
    inline static constexpr char cancelled[] = "IK: cancelled";
};

using SixVector = std::array<double, IkSolverConstants::taskDimension>;
using SixMatrix = std::array<SixVector, IkSolverConstants::taskDimension>;
// 계산 중 가장 큰 수가 이 값보다 작으면 거의 0으로 나누는 셈이므로 풀이를 멈춘다.

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

double OrientationWeight(const IkOptions& options)
{
    return options.taskMode == IkTaskMode::PositionOnly
        ? 0.0 : options.orientationWeightMetersPerRadian;
}

SixVector JacobianColumn(
    const models::JointSpecification& joint,
    const Vec3& axis,
    const Vec3& tcpPosition,
    const Vec3& jointPosition,
    double orientationWeight)
{
    const bool slides = joint.motionType == models::JointMotionType::Prismatic;
    const Vec3 linear = slides ? axis : Cross(axis, Subtract(tcpPosition, jointPosition));
    const double angularScale = slides ? 0.0 : orientationWeight;
    return {linear.x, linear.y, linear.z,
        axis.x * angularScale, axis.y * angularScale, axis.z * angularScale};
}

bool SolveSystem(SixMatrix matrix, SixVector right, SixVector& answer)
{
    Eigen::Matrix<double, IkSolverConstants::taskDimension, IkSolverConstants::taskDimension> system;
    Eigen::Matrix<double, IkSolverConstants::taskDimension, 1> target;
    for (Eigen::Index row = 0; row < IkSolverConstants::taskDimension; ++row)
    {
        target[row] = right[static_cast<std::size_t>(row)];
        for (Eigen::Index column = 0; column < IkSolverConstants::taskDimension; ++column)
            system(row, column) = matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)];
    }

    const auto decomposition = system.ldlt();
    if (decomposition.info() != Eigen::Success)
        return false;
    const Eigen::Matrix<double, IkSolverConstants::taskDimension, 1> solution = decomposition.solve(target);
    if (decomposition.info() != Eigen::Success || !solution.allFinite())
        return false;
    for (Eigen::Index row = 0; row < IkSolverConstants::taskDimension; ++row)
        answer[static_cast<std::size_t>(row)] = solution[row];
    return true;
}

struct Error
{
    SixVector weighted{};
    double position = 0.0;
    double orientation = 0.0;
    double cost = 0.0;
};

enum class FeasibleStepStatus
{
    Feasible,
    NoFeasibleDirection,
    NonFiniteStep
};

/**
 * @brief 관절 한계를 넘지 않는 LM 관절 이동량을 계산한다.
 * @details 한계 바깥을 향하는 관절은 이번 반복에서 고정하고, 나머지 관절로 다시 계산한다.
 * 반환 상태는 호출자가 감쇠값을 조절하거나 수치 오류를 보고하는 데 사용한다.
 */
FeasibleStepStatus FindFeasibleJointStep(
    const models::RobotSpecification& specification,
    const IkOptions& options,
    const std::vector<SixVector>& columns,
    const SixVector& weightedError,
    const JointVector& angles,
    double damping,
    JointVector& jointStep,
    std::vector<bool>& active,
    double& stepScale,
    bool& solvedSystem,
    bool& limitBlocked)
{
    std::fill(active.begin(), active.end(), false);
    for (std::size_t activePass = 0; activePass <= angles.size(); ++activePass)
    {
        // 한계에 묶이지 않은 관절의 Jacobian 열로 J_free J_freeᵀ를 만들고 λ²I를 더해 6×6 식을 푼다.
        // 얻은 작업 공간 변화량을 각 열에 곱해 관절 변화량으로 되돌리므로 관절 수가 달라도 풀이 행렬 크기는 같다.
        SixMatrix normal{};
        for (std::size_t row = 0; row < IkSolverConstants::taskDimension; ++row)
        {
            for (std::size_t column = 0; column < IkSolverConstants::taskDimension; ++column)
                for (std::size_t joint = 0; joint < columns.size(); ++joint)
                    if (!active[joint])
                        normal[row][column] += columns[joint][row] * columns[joint][column];
            normal[row][row] += damping * damping;
        }

        SixVector taskStep{};
        if (!SolveSystem(normal, weightedError, taskStep))
            return FeasibleStepStatus::NoFeasibleDirection;
        solvedSystem = true;
        std::fill(jointStep.begin(), jointStep.end(), 0.0);

        double largestScaledStep = 0.0;
        for (std::size_t joint = 0; joint < angles.size(); ++joint)
        {
            if (active[joint])
                continue;
            for (std::size_t row = 0; row < IkSolverConstants::taskDimension; ++row)
                jointStep[joint] += columns[joint][row] * taskStep[row];

            const bool slides = specification.joints[joint].motionType == models::JointMotionType::Prismatic;
            const double maximumStep = slides ? options.maxPrismaticStepMeters : options.maxJointStep;
            largestScaledStep = std::max(largestScaledStep,
                std::abs(jointStep[joint]) / maximumStep);
        }
        if (!std::isfinite(largestScaledStep))
            return FeasibleStepStatus::NonFiniteStep;

        stepScale = largestScaledStep > 1.0 ? 1.0 / largestScaledStep : 1.0;
        bool addedConstraint = false;
        for (std::size_t joint = 0; joint < angles.size(); ++joint)
        {
            if (active[joint])
                continue;
            const auto& limits = specification.joints[joint];
            const double boundaryTolerance = std::max(IkSolverConstants::jointLimitToleranceFloor,
                (limits.maximumPosition - limits.minimumPosition) * IkSolverConstants::jointLimitToleranceRatio);
            const double requested = angles[joint] + jointStep[joint] * stepScale;
            const bool atLowerLimit = angles[joint] <= limits.minimumPosition + boundaryTolerance;
            const bool atUpperLimit = angles[joint] >= limits.maximumPosition - boundaryTolerance;
            if ((atLowerLimit && requested < limits.minimumPosition) ||
                (atUpperLimit && requested > limits.maximumPosition))
            {
                active[joint] = true;
                addedConstraint = true;
                limitBlocked = true;
            }
        }

        if (!addedConstraint)
            return FeasibleStepStatus::Feasible;
    }
    return FeasibleStepStatus::NoFeasibleDirection;
}

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

bool HasConverged(const Error& error, const IkOptions& options)
{
    return error.position <= options.positionToleranceMeters &&
        (options.taskMode == IkTaskMode::PositionOnly ||
            error.orientation <= options.orientationToleranceRadians);
}

std::pair<double, double> JacobianCondition(const std::vector<SixVector>& columns)
{
    SixMatrix gram{};
    for (const auto& column : columns)
        for (std::size_t row = 0; row < IkSolverConstants::taskDimension; ++row)
            for (std::size_t other = 0; other < IkSolverConstants::taskDimension; ++other)
                gram[row][other] += column[row] * column[other];

    for (std::size_t sweep = 0; sweep < IkSolverConstants::maximumJacobiSweeps; ++sweep)
    {
        std::size_t p = 0;
        std::size_t q = 1;
        double largestOffDiagonal = 0.0;
        for (std::size_t row = 0; row < IkSolverConstants::taskDimension; ++row)
            for (std::size_t column = row + 1; column < IkSolverConstants::taskDimension; ++column)
                if (std::abs(gram[row][column]) > largestOffDiagonal)
                {
                    largestOffDiagonal = std::abs(gram[row][column]);
                    p = row;
                    q = column;
                }
        if (largestOffDiagonal <= IkSolverConstants::jacobiConvergenceTolerance)
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
        for (std::size_t index = 0; index < IkSolverConstants::taskDimension; ++index)
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
    for (std::size_t axis = 0; axis < IkSolverConstants::taskDimension; ++axis)
    {
        const double eigenvalue = std::max(0.0, gram[axis][axis]);
        minimumEigenvalue = std::min(minimumEigenvalue, eigenvalue);
        maximumEigenvalue = std::max(maximumEigenvalue, eigenvalue);
    }
    const double minimumSingularValue = std::sqrt(minimumEigenvalue);
    const double maximumSingularValue = std::sqrt(maximumEigenvalue);
    const double conditionNumber = minimumSingularValue > IkSolverConstants::minimumSingularValueForCondition
        ? maximumSingularValue / minimumSingularValue
        : std::numeric_limits<double>::infinity();
    return {minimumSingularValue, conditionNumber};
}
}

LevenbergMarquardtIk::LevenbergMarquardtIk(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame)
    : specification_(specification), forward_(specification), tcpInToolFrame_(tcpInToolFrame),
      jointPositionsScratch_(specification.jointCount), jointStepScratch_(specification.jointCount),
      candidateScratch_(specification.jointCount), activeJointScratch_(specification.jointCount),
      jacobianColumnsScratch_(specification.jointCount)
{
    if (!Finite(tcpInToolFrame_.positionMeters))
        throw std::invalid_argument(IkErrorMessages::invalidTcpOffset);
    tcpInToolFrame_.rotation = Normalize(tcpInToolFrame_.rotation);
    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        if (!std::isfinite(joint.minimumPosition) || !std::isfinite(joint.maximumPosition) ||
            joint.minimumPosition > joint.maximumPosition)
            throw std::invalid_argument(IkErrorMessages::invalidJointLimits);
        if (i > 0)
            maximumReachMeters_ += Length(Subtract(joint.bindPivotMeters, specification_.joints[i - 1].bindPivotMeters));
        if (joint.motionType == models::JointMotionType::Prismatic)
            maximumReachMeters_ += std::max(std::abs(joint.minimumPosition), std::abs(joint.maximumPosition));
    }
    // 첫 관절 중심을 기준으로 링크 사이 길이와 ToolFrame/TCP offset 길이를 모두 더한다. 회전이나 관절 제한을 무시한 넉넉한 구이므로 이 거리 밖은 확실히 도달할 수 없다.
    if (specification_.hasToolFrame)
        maximumReachMeters_ += Length(specification_.toolFrameInLastJoint.positionMeters) + Length(tcpInToolFrame_.positionMeters);
    if (!std::isfinite(maximumReachMeters_))
        throw std::invalid_argument(IkErrorMessages::nonFiniteRobotReach);
}

CartesianPose LevenbergMarquardtIk::EvaluateTcp(const JointVector& jointPositions)
{
    if (!specification_.hasToolFrame)
        throw std::invalid_argument(IkErrorMessages::missingToolFrame);
    return ToCartesian(Compose(forward_.Update(jointPositions).toolFrameInBaseFrame, tcpInToolFrame_));
}

IkResult LevenbergMarquardtIk::SolveSingleSeed(const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options)
{
    const SessionId session = BeginSingleSeed(targetInBase, currentSeed, options);
    while (StepSingleSeed(session, options.maxIterations) == IkSessionState::Running) {}
    return GetSingleSeedResult(session);
}

LevenbergMarquardtIk::SessionId LevenbergMarquardtIk::BeginSingleSeed(
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
        !Positive(options.maxJointStep) || !Positive(options.maxPrismaticStepMeters) ||
        options.stagnationIterationLimit == 0 ||
        !Positive(options.stagnationRelativeCostTolerance) ||
        !Positive(options.stagnationJointStepTolerance) ||
        !Positive(options.stagnationPrismaticStepToleranceMeters))
    {
        fail(IkStatus::InvalidInput, IkErrorMessages::invalidSeedOrOptions);
        return activeSessionId_;
    }
    for (std::size_t i = 0; i < currentSeed.size(); ++i)
    {
        if (!std::isfinite(currentSeed[i]) || currentSeed[i] < specification_.joints[i].minimumPosition ||
            currentSeed[i] > specification_.joints[i].maximumPosition)
        {
            fail(IkStatus::InvalidInput, IkErrorMessages::seedOutsideJointLimits);
            return activeSessionId_;
        }
    }
    try { sessionTarget_ = FromCartesian(targetInBase); }
    catch (const std::invalid_argument&)
    {
        fail(IkStatus::InvalidInput, IkErrorMessages::invalidTargetPose);
        return activeSessionId_;
    }
    sessionResult_.jointPositions = currentSeed;
    if (!specification_.hasToolFrame)
    {
        fail(IkStatus::MissingToolFrame, IkErrorMessages::specificationMissingToolFrame);
        return activeSessionId_;
    }
    const double targetRadius = Length(Subtract(sessionTarget_.positionMeters, specification_.joints[0].bindPivotMeters));
    if (!std::isfinite(targetRadius) || targetRadius > maximumReachMeters_ + options.positionToleranceMeters)
    {
        sessionResult_.status = IkStatus::Unreachable;
        sessionResult_.terminationReason = IkTerminationReason::ConservativeReachExceeded;
        sessionResult_.message = std::string{IkErrorMessages::targetRadiusPrefix} + std::to_string(targetRadius) +
            IkErrorMessages::targetRadiusExceededReach + std::to_string(maximumReachMeters_) + IkErrorMessages::metersSuffix;
        sessionState_ = IkSessionState::Completed;
        return activeSessionId_;
    }

    std::copy(currentSeed.begin(), currentSeed.end(), jointPositionsScratch_.begin());
    return activeSessionId_;
}

IkSessionState LevenbergMarquardtIk::StepSingleSeed(SessionId session, std::size_t iterationBudget)
{
    if (session != activeSessionId_)
        throw std::invalid_argument(IkErrorMessages::inactiveSession);
    if (sessionState_ != IkSessionState::Running || iterationBudget == 0)
        return sessionState_;

    auto& angles = jointPositionsScratch_;
    auto& jointStep = jointStepScratch_;
    auto& candidate = candidateScratch_;
    auto& active = activeJointScratch_;
    auto& columns = jacobianColumnsScratch_;
    const auto recordCurrentResult = [&](const Error& error)
    {
        sessionResult_.jointPositions = angles;
        sessionResult_.iterations = sessionIteration_;
        sessionResult_.positionErrorMeters = error.position;
        sessionResult_.orientationErrorRadians = error.orientation;
    };
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
                columns[joint] = JacobianColumn(specification_.joints[joint], axis,
                    current.positionMeters, state.linkPosesInBaseFrame[joint].positionMeters,
                    OrientationWeight(sessionOptions_));
            }
            const auto [minimum, condition] = JacobianCondition(columns);
            sessionResult_.weightedJacobianMinimumSingularValue = minimum;
            sessionResult_.weightedJacobianConditionNumber = condition;
        }
        sessionState_ = IkSessionState::Completed;
    };
    const auto finishAtIterationLimit = [&]()
    {
        finish(sessionActiveJointLimit_ ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
            sessionActiveJointLimit_ ? IkErrorMessages::constrainedIterationLimit : IkErrorMessages::iterationLimitReached);
        if (!sessionActiveJointLimit_)
            sessionResult_.terminationReason = IkTerminationReason::IterationLimit;
    };
    std::size_t iterationsUsed = 0;
    while (sessionState_ == IkSessionState::Running && iterationsUsed < iterationBudget)
    {
        const auto& state = forward_.Update(angles);
        const Pose3 current = Compose(state.toolFrameInBaseFrame, tcpInToolFrame_);
        const Error error = Measure(sessionTarget_, current, OrientationWeight(sessionOptions_));
        recordCurrentResult(error);
        if (HasConverged(error, sessionOptions_))
        {
            sessionResult_.status = IkStatus::Success;
            sessionResult_.terminationReason = IkTerminationReason::Converged;
            sessionState_ = IkSessionState::Completed;
            break;
        }
        if (!std::isfinite(error.cost))
        {
            finish(IkStatus::InvalidInput, IkErrorMessages::numericRangeExceeded);
            break;
        }
        if (sessionIteration_ == sessionOptions_.maxIterations)
        {
            finishAtIterationLimit();
            break;
        }

        for (auto& column : columns)
            column.fill(0.0);
        // 야코비안은 관절을 조금 돌렸을 때 공구 끝(TCP)이 얼마나 움직이는지 나타내는 표다.
        // 관절마다 열 하나를 만들기 때문에 관절 수가 달라도 같은 계산을 쓸 수 있다. TCP와 공구 장착점의 차이도 반영한다.
        for (std::size_t i = 0; i < columns.size(); ++i)
        {
            const Vec3 axis = state.jointAxesInBaseFrame[i];
            columns[i] = JacobianColumn(specification_.joints[i], axis,
                current.positionMeters, state.linkPosesInBaseFrame[i].positionMeters,
                OrientationWeight(sessionOptions_));
        }

        bool improved = false;
        bool activeLimitBlockedThisIteration = false;
        bool solvedDampedSystem = false;
        for (std::size_t attempt = 0; attempt < IkSolverConstants::maximumDampingAttempts && !improved; ++attempt)
        {
            double stepScale = 1.0;
            const FeasibleStepStatus stepStatus = FindFeasibleJointStep(
                specification_, sessionOptions_, columns, error.weighted, angles,
                sessionDamping_, jointStep, active, stepScale,
                solvedDampedSystem, activeLimitBlockedThisIteration);
            if (stepStatus == FeasibleStepStatus::NonFiniteStep)
            {
                finish(IkStatus::DidNotConverge, IkErrorMessages::nonFiniteNumericalStep);
                break;
            }
            if (sessionState_ != IkSessionState::Running)
                break;
            if (stepStatus == FeasibleStepStatus::NoFeasibleDirection)
            {
                sessionDamping_ *= solvedDampedSystem
                    ? IkSolverConstants::rejectedStepDampingGrowth : IkSolverConstants::singularSystemDampingGrowth;
                continue;
            }

            double feasibleFraction = stepScale;
            for (std::size_t joint = 0; joint < angles.size(); ++joint)
            {
                if (active[joint] || std::abs(jointStep[joint]) <= IkSolverConstants::negligibleJointStep)
                    continue;
                const auto& limits = specification_.joints[joint];
                const double distance = jointStep[joint] > 0.0
                    ? limits.maximumPosition - angles[joint]
                    : angles[joint] - limits.minimumPosition;
                feasibleFraction = std::min(feasibleFraction,
                    std::max(0.0, distance / std::abs(jointStep[joint])));
            }
            for (std::size_t line = 0; line < IkSolverConstants::maximumLineSearchSteps && !improved; ++line)
            {
                const double fraction = feasibleFraction * std::ldexp(1.0, -line);
                std::copy(angles.begin(), angles.end(), candidate.begin());
                for (std::size_t i = 0; i < angles.size(); ++i)
                {
                    const double requested = angles[i] + jointStep[i] * fraction;
                    const auto& limits = specification_.joints[i];
                    candidate[i] = std::clamp(requested, limits.minimumPosition, limits.maximumPosition);
                }
                const Error next = Measure(sessionTarget_, Compose(forward_.Update(candidate).toolFrameInBaseFrame, tcpInToolFrame_),
                    OrientationWeight(sessionOptions_));
                if (next.cost < error.cost)
                {
                    angles.swap(candidate);
                    // 감쇠값은 한 번에 관절을 얼마나 움직일지 조절한다. 목표 오차가 줄면 낮추고, 줄지 않으면 높인다.
                    // 감쇠값을 낮추면 관절이 크게 움직일 수 있으므로 한 번의 최대 관절 변화량도 제한한다.
                    sessionDamping_ = std::max(sessionOptions_.damping * IkSolverConstants::minimumDampingRatio,
                        sessionDamping_ * IkSolverConstants::acceptedStepDampingDecay);
                    double largestAcceptedStepRatio = 0.0;
                    for (std::size_t joint = 0; joint < jointStep.size(); ++joint)
                    {
                        const bool slides = specification_.joints[joint].motionType == models::JointMotionType::Prismatic;
                        const double tolerance = slides
                            ? sessionOptions_.stagnationPrismaticStepToleranceMeters
                            : sessionOptions_.stagnationJointStepTolerance;
                        largestAcceptedStepRatio = std::max(largestAcceptedStepRatio,
                            std::abs(jointStep[joint] * fraction) / tolerance);
                    }
                    const double relativeImprovement = (error.cost - next.cost) /
                        std::max(error.cost, std::numeric_limits<double>::min());
                    if (relativeImprovement <= sessionOptions_.stagnationRelativeCostTolerance &&
                        largestAcceptedStepRatio <= 1.0)
                        ++sessionStagnantIterations_;
                    else
                        sessionStagnantIterations_ = 0;
                    improved = true;
                }
            }
            if (!improved)
                sessionDamping_ *= IkSolverConstants::rejectedStepDampingGrowth;
        }
        if (sessionState_ != IkSessionState::Running)
            break;
        if (!improved)
        {
            sessionActiveJointLimit_ = activeLimitBlockedThisIteration;
            finish(sessionActiveJointLimit_ ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
                sessionActiveJointLimit_ ? IkErrorMessages::jointLimitsBlockImprovement : IkErrorMessages::localIterationStalled);
            break;
        }
        sessionActiveJointLimit_ = activeLimitBlockedThisIteration;
        ++sessionIteration_;
        ++iterationsUsed;
        sessionResult_.iterations = sessionIteration_;
        if (sessionStagnantIterations_ >= sessionOptions_.stagnationIterationLimit)
        {
            finish(sessionActiveJointLimit_ ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
                sessionActiveJointLimit_ ? IkErrorMessages::progressStalledByJointLimits : IkErrorMessages::progressStalled);
            break;
        }
    }
    if (sessionState_ == IkSessionState::Running && sessionIteration_ == sessionOptions_.maxIterations)
    {
        const auto& state = forward_.Update(angles);
        const Error error = Measure(sessionTarget_, Compose(state.toolFrameInBaseFrame, tcpInToolFrame_),
            OrientationWeight(sessionOptions_));
        recordCurrentResult(error);
        if (HasConverged(error, sessionOptions_))
        {
            sessionResult_.status = IkStatus::Success;
            sessionResult_.terminationReason = IkTerminationReason::Converged;
            sessionState_ = IkSessionState::Completed;
        }
        else
        {
            finishAtIterationLimit();
        }
    }
    return sessionState_;
}

void LevenbergMarquardtIk::CancelSingleSeed(SessionId session)
{
    if (session != activeSessionId_)
        throw std::invalid_argument(IkErrorMessages::inactiveSession);
    if (sessionState_ != IkSessionState::Running)
        return;
    sessionResult_.status = IkStatus::DidNotConverge;
    sessionResult_.terminationReason = IkTerminationReason::Cancelled;
    sessionResult_.message = IkErrorMessages::cancelled;
    sessionState_ = IkSessionState::Cancelled;
}

const IkResult& LevenbergMarquardtIk::GetSingleSeedResult(SessionId session) const
{
    if (session != activeSessionId_)
        throw std::invalid_argument(IkErrorMessages::inactiveSession);
    return sessionResult_;
}

IkResult LevenbergMarquardtIk::Solve(
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
            OrientationWeight(options) * candidate.orientationErrorRadians;
    };
    // 첫 시작 자세에서 해를 찾지 못하면 관절마다 허용 범위의 반대편에서 계산을 다시 시도한다.
    for (const auto& alternate : BuildAlternativeIkSeeds(currentSeed, specification_))
    {
        IkResult candidate = SolveSingleSeed(targetInBase, alternate, options);
        if (candidate.Ok())
        {
            const double jointDistance = distanceFromSeed(candidate.jointPositions);
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
