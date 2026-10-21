#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/planning/JointPathPlanner.h"
#include "planning/detail/LinearPathPlannerSupport.h"

#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"
#include "robotics/planning/detail/JointPathSampling.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#endif

namespace grasplink::robotics::planning
{
using detail::AlignEquivalentJointAngles;
using detail::MapIkFailure;
using detail::PathIkOptions;
using detail::RequiredTimeForVelocity;
using detail::SolveCollisionFreeIk;
using detail::ValidateLinearPathCommand;
using detail::VelocityRatio;

namespace detail
{
kinematics::IkOptions PathIkOptions(const models::RobotSpecification& specification)
{
    // 기본 IK 허용 오차는 경로 직선 오차 예산보다 위치는 100배, 방향은 10배 작다.
    // 따라서 별도의 수렴 기준을 더하지 않고 LM의 기본 IK 설정을 사용한다.
    auto options = kinematics::RobotDefaultIkOptions(specification);
    options.collectFailureJacobianDiagnostics = false;
    return options;
}

Result ValidateLinearPathCommand(const LinearPathMoveCommand& command)
{
    if (!command.targetPoses.empty() &&
        std::isfinite(command.maxLinearVelocityMetersPerSecond) &&
        command.maxLinearVelocityMetersPerSecond > 0.0 &&
        std::isfinite(command.maxAngularVelocityRadiansPerSecond) &&
        command.maxAngularVelocityRadiansPerSecond > 0.0 &&
        std::isfinite(command.maxLinearAccelerationMetersPerSecondSquared) &&
        command.maxLinearAccelerationMetersPerSecondSquared > 0.0 &&
        std::isfinite(command.maxAngularAccelerationRadiansPerSecondSquared) &&
        command.maxAngularAccelerationRadiansPerSecondSquared > 0.0)
        return Result::Success();
    return {ErrorCode::InvalidCommand, "LinearPathPlanner: invalid path or motion limits"};
}

Result MapIkFailure(const kinematics::IkResult& result)
{
    using kinematics::IkStatus;
    ErrorCode code = ErrorCode::IkDidNotConverge;
    switch (result.status)
    {
    case IkStatus::Success: return Result::Success();
    case IkStatus::InvalidInput: code = ErrorCode::InvalidCommand; break;
    case IkStatus::MissingToolFrame: code = ErrorCode::Unsupported; break;
    case IkStatus::Unreachable: code = ErrorCode::Unreachable; break;
    case IkStatus::JointLimitReached: code = ErrorCode::JointLimitReached; break;
    case IkStatus::DidNotConverge: break;
    }
    return {code, result.message};
}
} // namespace detail

namespace
{
using models::Pose3;

namespace limits
{
    static constexpr std::size_t maximumRestartSeedCount = 16;
    static constexpr std::size_t maximumIkCandidatesPerSample = 8;
    static constexpr std::size_t maximumIkSeedAttemptsPerSample = 32;
    static constexpr std::size_t maximumTcpRefinementPasses = 12;
    static constexpr std::size_t maximumPathIntervals = detail::kMaximumJointPathIntervals;
    static constexpr std::size_t initialTcpSampleCapacity = 64;
    static constexpr std::size_t finalTcpStraightnessProbeIndex = 3;
}

namespace math
{
    static constexpr double fullTurnRadians = models::kFullTurnRadians;
    static constexpr double degreesPerFullTurn = 360.0;
    static constexpr double jointLimitSearchMarginFraction = 0.1;
    static constexpr double jointLimitPenaltyWeight = 0.1;
    static constexpr double jointLimitRangeFloor = 1e-12;
    static constexpr double jointLimitComparisonTolerance = 1e-6;
    static constexpr double jointLimitBoundaryToleranceFloor = 1e-8;
    static constexpr double jointLimitBoundaryToleranceRatio = 1e-8;
    static constexpr double minimumSegmentDurationSeconds = 1e-6;
    static constexpr double minimumPathIntervalCount = 1.0;
    static constexpr double tcpStraightnessProbeFraction = 0.25;
    static constexpr double segmentMidpointFraction = 0.5;
    static constexpr double millimetersPerMeter = 1000.0;
    static constexpr double milliradiansPerRadian = 1000.0;
}

namespace messages
{
    inline static constexpr char invalidPlanningPolicy[] =
        "LinearPathPlanner: invalid planning policy";
    inline static constexpr char invalidTcpTarget[] = "LinearPathPlanner: invalid TCP target";
    inline static constexpr char pathDurationOutOfRange[] =
        "LinearPathPlanner: path duration exceeds numeric range";
    inline static constexpr char noCollisionFreeIkSolution[] =
        "LinearPathPlanner: no collision-free IK solution";
    inline static constexpr char wrongThread[] =
        "LinearPathPlanner: planning must advance on the Begin caller thread";
}

void AppendJointLimitContext(std::string& context,
    const kinematics::IkResult& ikFailure,
    const models::RobotSpecification& specification)
{
    if (ikFailure.status != kinematics::IkStatus::JointLimitReached)
        return;

    for (std::size_t joint = 0; joint < ikFailure.jointPositions.size(); ++joint)
    {
        const auto& limits = specification.joints[joint];
        if (!limits.HasPositionLimits())
            continue;

        const double tolerance = std::max(math::jointLimitBoundaryToleranceFloor,
            (limits.maximumPosition - limits.minimumPosition) *
                math::jointLimitBoundaryToleranceRatio);
        const double position = ikFailure.jointPositions[joint];
        if (std::abs(position - limits.minimumPosition) <= tolerance ||
            std::abs(position - limits.maximumPosition) <= tolerance)
        {
            context += "; IK candidate reached " + std::string(limits.name) + " limit";
            break;
        }
    }
}

void AppendIkFailureContext(std::string& context,
    const kinematics::IkResult& ikFailure,
    const models::RobotSpecification& specification)
{
    AppendJointLimitContext(context, ikFailure, specification);
    context += "; IK residual " + std::to_string(ikFailure.positionErrorMeters) +
        " m / " + std::to_string(ikFailure.orientationErrorRadians) +
        " rad after " + std::to_string(ikFailure.iterations) + " iterations; candidate_positions=[";
    for (std::size_t joint = 0; joint < ikFailure.jointPositions.size(); ++joint)
    {
        const auto& jointSpec = specification.joints[joint];
        const bool isPrismatic = jointSpec.motionType == models::JointMotionType::Prismatic;
        const double position = isPrismatic
            ? ikFailure.jointPositions[joint]
            : ikFailure.jointPositions[joint] * math::degreesPerFullTurn /
                math::fullTurnRadians;
        context += (joint == 0 ? "" : ",") + std::string(jointSpec.name) + "=" +
            std::to_string(position) +
            (isPrismatic ? " m" : " deg");
    }
    context += "]";
}

Result AddPathSampleContext(Result failure, std::size_t sample, std::size_t sampleCount,
    const models::RobotSpecification& specification, const CartesianPose& target,
    const kinematics::IkResult* ikFailure = nullptr)
{
    std::string context = "LinearPathPlanner: TCP path sample " + std::to_string(sample) + "/" +
        std::to_string(sampleCount) + " failed at (" +
        std::to_string(target.positionMeters[0]) + ", " +
        std::to_string(target.positionMeters[1]) + ", " +
        std::to_string(target.positionMeters[2]) + ") m";
    if (ikFailure)
        AppendIkFailureContext(context, *ikFailure, specification);
    failure.message = std::move(context) + ": " + failure.message;
    return failure;
}

// 각 TCP 표본에서 찾은 유효한 관절 자세를 한 층에 모은다. 점수는 시작 자세부터 누적하고,
// 이전 층의 후보 번호를 함께 저장해 마지막에 가장 나은 전체 경로를 거꾸로 복원한다.
struct PathCandidate
{
    JointVector joints;
    Pose3 tcpPose{};
    double score = 0.0;
    std::size_t previousCandidate = 0;
};

bool IsPolicyValid(const PlanningPolicy& policy)
{
    return std::isfinite(policy.jointCollisionSampleSpacingRadians) &&
        policy.jointCollisionSampleSpacingRadians > 0.0 &&
        std::isfinite(policy.prismaticJointCollisionSampleSpacingMeters) &&
        policy.prismaticJointCollisionSampleSpacingMeters > 0.0 &&
        std::isfinite(policy.linearPositionSampleSpacingMeters) &&
        policy.linearPositionSampleSpacingMeters > 0.0 &&
        std::isfinite(policy.linearOrientationSampleSpacingRadians) &&
        policy.linearOrientationSampleSpacingRadians > 0.0 &&
        policy.maximumPathIntervals > 0 &&
        policy.maximumPathIntervals <= limits::maximumPathIntervals &&
        policy.maximumIkCandidatesPerSample > 0 &&
        policy.maximumIkCandidatesPerSample <= limits::maximumIkCandidatesPerSample &&
        policy.maximumIkSeedAttemptsPerSample > 0 &&
        policy.maximumIkSeedAttemptsPerSample >= policy.maximumIkCandidatesPerSample &&
        policy.maximumIkSeedAttemptsPerSample <= limits::maximumIkSeedAttemptsPerSample &&
        policy.maximumTcpRefinementPasses <= limits::maximumTcpRefinementPasses &&
        std::isfinite(policy.maximumLinearTcpErrorMeters) && policy.maximumLinearTcpErrorMeters > 0.0 &&
        std::isfinite(policy.maximumAngularTcpErrorRadians) && policy.maximumAngularTcpErrorRadians > 0.0;
}

double JointTransitionCost(const JointVector& start, const JointVector& end,
    const models::RobotSpecification& specification)
{
    double score = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
    {
        const auto& limits = specification.joints[joint];
        if (!limits.HasPositionLimits())
        {
            const double shortestTurn = std::remainder(
                end[joint] - start[joint], math::fullTurnRadians);
            const double normalizedTurn = shortestTurn / math::fullTurnRadians;
            score += normalizedTurn * normalizedTurn;
            continue;
        }
        const double range = std::max(math::jointLimitRangeFloor,
            limits.maximumPosition - limits.minimumPosition);
        const double delta = (end[joint] - start[joint]) / range;
        score += delta * delta;
        const double margin = std::min(end[joint] - limits.minimumPosition,
            limits.maximumPosition - end[joint]) / range;
        // 다음 경로 점에서 움직일 여유를 남기도록 관절 한계에 가까운 자세에 작은 불이익을 준다.
        const double deficit = std::max(0.0,
            math::jointLimitSearchMarginFraction - margin) /
            math::jointLimitSearchMarginFraction;
        score += math::jointLimitPenaltyWeight * deficit * deficit;
    }
    return score;
}

double WeightedIkResidual(const kinematics::IkResult& result, const kinematics::IkOptions& options)
{
    return result.positionErrorMeters +
        options.orientationWeightMetersPerRadian * result.orientationErrorRadians;
}

// 여러 시작 자세에서 나온 실패 가운데 목표 오차가 가장 작은 결과를 진단에 남긴다.
void RecordBetterIkFailure(
    kinematics::IkResult failure,
    const kinematics::IkOptions& options,
    double& bestResidual,
    kinematics::IkResult& bestFailure)
{
    const double residual = WeightedIkResidual(failure, options);
    if (residual < bestResidual)
    {
        bestResidual = residual;
        bestFailure = std::move(failure);
    }
}

bool SameJointCandidate(const JointVector& left, const JointVector& right)
{
    for (std::size_t joint = 0; joint < left.size(); ++joint)
    {
        if (std::abs(left[joint] - right[joint]) >
            math::jointLimitComparisonTolerance)
            return false;
    }
    return true;
}

struct IkSeedCandidate
{
    std::size_t previousCandidate = 0;
    JointVector joints;
    bool continuation = false;
};

struct EvaluatedEdge
{
    std::size_t parent = 0;
    JointVector joints;
    bool followsTcp = false;
};

}

struct LinearPathPlanningJob::Impl
{
    // 긴 계획을 한 번에 실행하지 않도록 IK 풀기, TCP 직선 확인, 관절 경로 검사를 단계별로 나눈다.
    // Advance()는 호출자가 준 작업 예산 안에서 현재 단계만 이어서 진행한다.
    enum class Phase { PrepareSample, SolveSeed, CheckTcp, ValidatePath, FinishSample };
    enum class ValidationPurpose { Candidate, RefinementProbe };

    LinearPathMoveCommand command;
    const models::RobotSpecification* specification = nullptr;
    JointVector startJoints;
    kinematics::LevenbergMarquardtIk* inverse = nullptr;
    StateValidityChecker stateValidityChecker;
    std::function<std::string()> validityDiagnosticProvider;
    PlanningPolicy policy{};
    kinematics::IkOptions options{};
    Pose3 start{};
    std::vector<Pose3> targets;
    std::vector<Pose3> tcpSamples;
    std::vector<std::vector<PathCandidate>> layers;
    std::vector<std::size_t> refinementDepth;
    std::size_t sample = 0;
    LinearPathPlanningState state = LinearPathPlanningState::Idle;
    std::thread::id ownerThread{};
    Result result{};
    LinearPathPlan plan{};

    std::vector<IkSeedCandidate> seeds;
    std::vector<PathCandidate> candidates;
    std::vector<EvaluatedEdge> evaluatedEdges;
    std::vector<bool> parentHasContinuation;
    std::vector<bool> parentNeedsSearch;
    std::vector<bool> parentHasValidChild;
    std::size_t seedIndex = 0;
    std::size_t tcpProbe = 0;
    // 후보 관절 자세를 계산할 때 같은 크기의 임시 벡터를 재사용한다.
    JointVector tcpProbeJoints;
    std::size_t refinementEdge = 0;
    double bestFailureResidual = std::numeric_limits<double>::infinity();
    bool needsRefinement = false;
    bool expandedSeeds = false;
    bool planTaken = false;
    bool foundCollisionFreeRefinementEdge = false;
    double maximumSamplePositionErrorMeters = 0.0;
    double maximumSampleOrientationErrorRadians = 0.0;
    JointStateInvalidity invalidity = JointStateInvalidity::None;
    JointStateInvalidity rejectedRefinementInvalidity = JointStateInvalidity::None;
    std::string invalidityDiagnostic;
    std::string rejectedRefinementDiagnostic;
    kinematics::IkResult ikFailure;
    kinematics::IkResult workingSolution;
    double workingScore = 0.0;
    std::size_t workingParent = 0;
    bool workingContinuation = false;
    kinematics::LevenbergMarquardtIk::SessionId ikSession = 0;
    Phase phase = Phase::PrepareSample;
    ValidationPurpose validationPurpose = ValidationPurpose::Candidate;
    JointVector validationStart;
    JointVector validationEnd;
    JointVector validationSample;
    std::size_t validationIntervals = 0;
    std::size_t validationStep = 0;

    auto FindCandidate(const JointVector& joints)
    {
        return std::find_if(candidates.begin(), candidates.end(), [&](const PathCandidate& candidate)
        {
            return SameJointCandidate(candidate.joints, joints);
        });
    }

    auto FindWorstCandidate()
    {
        return std::max_element(candidates.begin(), candidates.end(),
            [](const PathCandidate& left, const PathCandidate& right)
            {
                return left.score < right.score;
            });
    }

    void AdvanceToNextSeed()
    {
        ++seedIndex;
        phase = Phase::SolveSeed;
    }

    void MarkWorkingContinuation()
    {
        if (workingContinuation)
            parentHasContinuation[workingParent] = true;
    }

    void CancelActiveIkSession() noexcept
    {
        if (ikSession == 0 || inverse == nullptr)
            return;

        try
        {
            inverse->CancelSingleSeed(ikSession);
        }
        catch (...)
        {
        }
        ikSession = 0;
    }

    void ResetForBegin(const LinearPathMoveCommand& moveCommand,
        const models::RobotSpecification& robot, const JointVector& joints,
        kinematics::LevenbergMarquardtIk& ik, const StateValidityChecker& checker,
        const PlanningPolicy& planningPolicy,
        const std::function<std::string()>& diagnosticProvider)
    {
        CancelActiveIkSession();
        stateValidityChecker = {};
        command = moveCommand;
        specification = &robot;
        startJoints = joints;
        tcpProbeJoints.resize(joints.size());
        inverse = &ik;
        stateValidityChecker = checker;
        validityDiagnosticProvider = diagnosticProvider;
        policy = planningPolicy;
        ownerThread = std::this_thread::get_id();
        plan = {};
        planTaken = false;
        result = Result::Success();
        sample = 0;
        targets.clear();
        tcpSamples.clear();
        layers.clear();
        refinementDepth.clear();
        seeds.clear();
        candidates.clear();
        evaluatedEdges.clear();
    }

    Result ReadAndValidateTargets(const CartesianPose& tcp)
    {
        targets.reserve(command.targetPoses.size());
        try
        {
            for (const CartesianPose& target : command.targetPoses)
                targets.push_back(kinematics::detail::FromCartesian(target));
            start = kinematics::detail::FromCartesian(tcp);
        }
        catch (const std::invalid_argument&)
        {
            return {ErrorCode::InvalidCommand, messages::invalidTcpTarget};
        }

        options = PathIkOptions(*specification);
        for (const CartesianPose& endpoint : command.targetPoses)
        {
            const auto endpointSession = inverse->BeginSingleSeed(endpoint, startJoints, options);
            const auto& endpointCheck = inverse->GetSingleSeedResult(endpointSession);
            if (endpointCheck.status == kinematics::IkStatus::Unreachable)
                return MapIkFailure(endpointCheck);
            inverse->CancelSingleSeed(endpointSession);
        }
        return Result::Success();
    }

    bool HasMotion() const
    {
        Pose3 segmentStart = start;
        for (const Pose3& end : targets)
        {
            if (kinematics::detail::Length(kinematics::detail::Subtract(
                    end.positionMeters, segmentStart.positionMeters)) > options.positionToleranceMeters ||
                kinematics::detail::Length(kinematics::detail::RotationError(
                    end.rotation, segmentStart.rotation)) > options.orientationToleranceRadians)
                return true;
            segmentStart = end;
        }
        return false;
    }

    Result BuildTcpSamples()
    {
        tcpSamples.reserve(limits::initialTcpSampleCapacity);
        std::size_t totalIntervals = 0;
        Pose3 segmentStart = start;
        for (const Pose3& end : targets)
        {
            const double distance = kinematics::detail::Length(
                kinematics::detail::Subtract(end.positionMeters, segmentStart.positionMeters));
            const double rotation = kinematics::detail::Length(
                kinematics::detail::RotationError(end.rotation, segmentStart.rotation));
            const double intervals = std::max({math::minimumPathIntervalCount,
                std::ceil(distance / policy.linearPositionSampleSpacingMeters),
                std::ceil(rotation / policy.linearOrientationSampleSpacingRadians)});
            if (!std::isfinite(intervals) || intervals > static_cast<double>(policy.maximumPathIntervals) ||
                totalIntervals > policy.maximumPathIntervals ||
                intervals > static_cast<double>(policy.maximumPathIntervals - totalIntervals))
            {
                return {ErrorCode::InvalidCommand, "LinearPathPlanner: path exceeds " +
                    std::to_string(policy.maximumPathIntervals) + " intervals"};
            }

            const std::size_t count = static_cast<std::size_t>(intervals);
            totalIntervals += count;
            for (std::size_t i = 1; i <= count; ++i)
                tcpSamples.push_back(kinematics::detail::Interpolate(segmentStart, end,
                    static_cast<double>(i) / static_cast<double>(count)));
            segmentStart = end;
        }
        return Result::Success();
    }

    Result Begin(const LinearPathMoveCommand& moveCommand,
        const models::RobotSpecification& robot, const JointVector& joints,
        const CartesianPose& tcp, kinematics::LevenbergMarquardtIk& ik,
        const StateValidityChecker& checker, const PlanningPolicy& planningPolicy,
        const std::function<std::string()>& diagnosticProvider)
    {
        ResetForBegin(moveCommand, robot, joints, ik, checker, planningPolicy,
            diagnosticProvider);

        if (!IsPolicyValid(policy))
            return Fail({ErrorCode::InvalidCommand, messages::invalidPlanningPolicy});
        if (const Result targetsResult = ReadAndValidateTargets(tcp); !targetsResult)
            return Fail(targetsResult);

        plan.hasMotion = HasMotion();
        if (!plan.hasMotion)
        {
            state = LinearPathPlanningState::Completed;
            ReleasePlanningStorage();
            return result;
        }

        if (const Result samplingResult = BuildTcpSamples(); !samplingResult)
            return Fail(samplingResult);
        layers.reserve(policy.maximumPathIntervals + 1);
        layers.push_back({PathCandidate{joints, start, 0.0, 0}});
        refinementDepth.resize(tcpSamples.size(), 0);
        state = LinearPathPlanningState::Running;
        phase = Phase::PrepareSample;
        return Result::Success();
    }

    Result Fail(Result failure)
    {
        CancelActiveIkSession();
        result = std::move(failure);
        state = LinearPathPlanningState::Failed;
        ReleasePlanningStorage();
        return result;
    }

    void ReleasePlanningStorage()
    {
        // 끝난 작업은 모델과 IK 계산기를 다시 쓰지 않으므로 빌린 참조와 임시 표본을 해제한다.
        stateValidityChecker = {};
        validityDiagnosticProvider = {};
        specification = nullptr;
        inverse = nullptr;
        std::vector<Pose3>().swap(targets);
        std::vector<Pose3>().swap(tcpSamples);
        std::vector<std::vector<PathCandidate>>().swap(layers);
        std::vector<std::size_t>().swap(refinementDepth);
        std::vector<IkSeedCandidate>().swap(seeds);
        std::vector<PathCandidate>().swap(candidates);
        std::vector<EvaluatedEdge>().swap(evaluatedEdges);
        std::vector<bool>().swap(parentHasContinuation);
        std::vector<bool>().swap(parentNeedsSearch);
        std::vector<bool>().swap(parentHasValidChild);
        std::vector<double>().swap(validationSample);
        std::vector<double>().swap(validationStart);
        std::vector<double>().swap(validationEnd);
    }

    void Finish()
    {
        using namespace kinematics::detail;
        std::size_t selected = static_cast<std::size_t>(std::min_element(
            layers.back().begin(), layers.back().end(), [](const PathCandidate& left, const PathCandidate& right)
            {
                return left.score < right.score;
            }) - layers.back().begin());
        std::vector<JointVector> selectedJoints(tcpSamples.size());
        for (std::size_t layer = tcpSamples.size(); layer > 0; --layer)
        {
            const auto& node = layers[layer][selected];
            selectedJoints[layer - 1] = node.joints;
            selected = node.previousCandidate;
        }

        plan.points.clear();
        plan.points.reserve(tcpSamples.size() + 1);
        plan.points.push_back({startJoints, ToCartesian(start), 0.0});
        plan.plannedLinearVelocity = 0.0;
        plan.plannedAngularVelocity = 0.0;
        for (std::size_t point = 0; point < tcpSamples.size(); ++point)
        {
            const auto& previousPoint = plan.points.back();
            const double distance = Length(Subtract(tcpSamples[point].positionMeters,
                FromCartesian(previousPoint.tcpPose).positionMeters));
            const double rotation = Length(RotationError(tcpSamples[point].rotation,
                FromCartesian(previousPoint.tcpPose).rotation));
            double duration = std::max(
                RequiredTimeForVelocity(distance, command.maxLinearVelocityMetersPerSecond),
                RequiredTimeForVelocity(rotation, command.maxAngularVelocityRadiansPerSecond));
            for (std::size_t joint = 0; joint < specification->jointCount; ++joint)
                duration = std::max(duration, RequiredTimeForVelocity(
                    selectedJoints[point][joint] - previousPoint.joints[joint],
                    specification->joints[joint].maximumVelocity));
            if (!std::isfinite(duration))
            {
                Fail({ErrorCode::InvalidCommand, messages::pathDurationOutOfRange});
                return;
            }
            plan.points.push_back({std::move(selectedJoints[point]), ToCartesian(tcpSamples[point]),
                std::max(duration, math::minimumSegmentDurationSeconds)});
            plan.plannedLinearVelocity = std::max(plan.plannedLinearVelocity, distance / duration);
            plan.plannedAngularVelocity = std::max(plan.plannedAngularVelocity, rotation / duration);
        }
        state = LinearPathPlanningState::Completed;
        result = Result::Success();
        ReleasePlanningStorage();
    }

    void PrepareSample()
    {
        const auto& previous = layers.back();
        seeds.clear();
        seeds.reserve(std::max(policy.maximumIkSeedAttemptsPerSample, previous.size()));
        for (std::size_t parent = 0; parent < previous.size(); ++parent)
            seeds.push_back({parent, previous[parent].joints, true});
        candidates.clear();
        candidates.reserve(policy.maximumIkCandidatesPerSample);
        evaluatedEdges.clear();
        parentHasContinuation.assign(previous.size(), false);
        parentNeedsSearch.assign(previous.size(), false);
        parentHasValidChild.assign(previous.size(), false);
        seedIndex = 0;
        tcpProbe = 0;
        refinementEdge = 0;
        bestFailureResidual = std::numeric_limits<double>::infinity();
        needsRefinement = false;
        expandedSeeds = false;
        foundCollisionFreeRefinementEdge = false;
        maximumSamplePositionErrorMeters = 0.0;
        maximumSampleOrientationErrorRadians = 0.0;
        invalidity = JointStateInvalidity::None;
        rejectedRefinementInvalidity = JointStateInvalidity::None;
        invalidityDiagnostic.clear();
        rejectedRefinementDiagnostic.clear();
        ikFailure = {};
        phase = Phase::SolveSeed;
    }

    void StartSeed()
    {
        const auto& seed = seeds[seedIndex];
        const auto target = kinematics::detail::ToCartesian(tcpSamples[sample]);
        ikSession = inverse->BeginSingleSeed(target, seed.joints, options);
        ++plan.ikSolveCount;
        workingParent = seed.previousCandidate;
        workingContinuation = seed.continuation;
    }

    void RecordIkFailure(kinematics::IkResult failure)
    {
        RecordBetterIkFailure(std::move(failure), options, bestFailureResidual, ikFailure);
        if (workingContinuation)
            parentNeedsSearch[workingParent] = true;
    }

    void PrepareSolution()
    {
        if (!workingSolution)
        {
            RecordIkFailure(std::move(workingSolution));
            AdvanceToNextSeed();
            return;
        }

        const auto& parent = layers.back()[workingParent];
        AlignEquivalentJointAngles(workingSolution.jointPositions, parent.joints, *specification);
        workingScore = parent.score + JointTransitionCost(
            parent.joints, workingSolution.jointPositions, *specification);
        auto duplicate = FindCandidate(workingSolution.jointPositions);
        if (duplicate != candidates.end() && workingScore >= duplicate->score)
        {
            if (duplicate->previousCandidate == workingParent)
            {
                parentHasValidChild[workingParent] = true;
                MarkWorkingContinuation();
            }
            AdvanceToNextSeed();
            return;
        }
        if (candidates.size() >= policy.maximumIkCandidatesPerSample)
        {
            const auto worst = FindWorstCandidate();
            if (workingScore > worst->score)
            {
                AdvanceToNextSeed();
                return;
            }
        }

        for (const auto& edge : evaluatedEdges)
        {
            if (edge.parent == workingParent &&
                std::equal(workingSolution.jointPositions.begin(),
                    workingSolution.jointPositions.end(), edge.joints.begin()))
            {
                MarkWorkingContinuation();
                AdvanceToNextSeed();
                return;
            }
        }
        evaluatedEdges.push_back({workingParent, workingSolution.jointPositions, false});
        tcpProbe = 0;
        phase = Phase::CheckTcp;
    }

    void FinishTcpCheck()
    {
        auto& edge = evaluatedEdges.back();
        const auto& parent = layers.back()[workingParent];
        if (tcpProbe < limits::finalTcpStraightnessProbeIndex)
            return;
        edge.followsTcp = true;
        BeginPathValidation(parent.joints, workingSolution.jointPositions, ValidationPurpose::Candidate);
    }

    void BeginPathValidation(const JointVector& pathStart, const JointVector& pathEnd,
        ValidationPurpose purpose)
    {
        validationPurpose = purpose;
        validationStart = pathStart;
        validationEnd = pathEnd;
        validationStep = 1;
        validationSample.resize(pathStart.size());
        ++plan.jointPathValidityChecks;
        const auto startInvalidity = ValidateJointState(*specification, validationStart, {});
        const auto endInvalidity = ValidateJointState(*specification, validationEnd, {});
        if (startInvalidity != JointStateInvalidity::None || endInvalidity != JointStateInvalidity::None)
        {
            CompletePathValidation(startInvalidity != JointStateInvalidity::None ? startInvalidity : endInvalidity);
            return;
        }
        const auto intervals = detail::JointPathIntervalCount(
            validationStart, validationEnd, *specification, policy);
        if (!intervals)
        {
            CompletePathValidation(JointStateInvalidity::PathSamplingLimitExceeded);
            return;
        }
        validationIntervals = *intervals;
        phase = Phase::ValidatePath;
    }

    void CompleteRefinementValidation(JointStateInvalidity pathInvalidity, std::string diagnostic)
    {
        if (pathInvalidity == JointStateInvalidity::None)
            foundCollisionFreeRefinementEdge = true;
        else if (rejectedRefinementInvalidity == JointStateInvalidity::None)
        {
            rejectedRefinementInvalidity = pathInvalidity;
            rejectedRefinementDiagnostic = std::move(diagnostic);
        }
        ++refinementEdge;
        phase = Phase::FinishSample;
    }

    void RejectCandidate(JointStateInvalidity pathInvalidity, std::string diagnostic)
    {
        if (invalidity == JointStateInvalidity::None)
        {
            invalidity = pathInvalidity;
            invalidityDiagnostic = std::move(diagnostic);
        }
        parentNeedsSearch[workingParent] = true;
        MarkWorkingContinuation();
        AdvanceToNextSeed();
    }

    void CompleteCandidateValidation(JointStateInvalidity pathInvalidity, std::string diagnostic)
    {
        if (pathInvalidity != JointStateInvalidity::None)
        {
            RejectCandidate(pathInvalidity, std::move(diagnostic));
            return;
        }

        const auto& parent = layers.back()[workingParent];
        parentHasValidChild[workingParent] = true;
        auto duplicate = FindCandidate(workingSolution.jointPositions);
        if (duplicate == candidates.end())
        {
            candidates.push_back({std::move(workingSolution.jointPositions), tcpSamples[sample],
                workingScore, workingParent});
            duplicate = std::prev(candidates.end());
        }
        else if (workingScore < duplicate->score)
        {
            duplicate->joints = std::move(workingSolution.jointPositions);
            duplicate->tcpPose = tcpSamples[sample];
            duplicate->score = workingScore;
            duplicate->previousCandidate = workingParent;
        }
        if (workingContinuation)
            parentHasContinuation[workingParent] = true;
        if (candidates.size() > policy.maximumIkCandidatesPerSample)
        {
            const auto worst = FindWorstCandidate();
            candidates.erase(worst);
        }
        AdvanceToNextSeed();
    }

    void CompletePathValidation(JointStateInvalidity pathInvalidity, std::string diagnostic = {})
    {
        if (validationPurpose == ValidationPurpose::RefinementProbe)
        {
            CompleteRefinementValidation(pathInvalidity, std::move(diagnostic));
            return;
        }
        CompleteCandidateValidation(pathInvalidity, std::move(diagnostic));
    }

    void AppendAlternativeSeeds(const std::vector<PathCandidate>& previous,
        const std::vector<std::vector<JointVector>>& alternativeSeeds)
    {
        for (std::size_t branch = 0; seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++branch)
        {
            bool addedSeed = false;
            for (std::size_t parent = 0; parent < previous.size() &&
                seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++parent)
            {
                if (parentNeedsSearch[parent] && branch < alternativeSeeds[parent].size())
                {
                    seeds.push_back({parent, alternativeSeeds[parent][branch], false});
                    addedSeed = true;
                }
            }
            if (!addedSeed)
                break;
        }
    }

    void AppendRestartSeeds(const std::vector<PathCandidate>& previous)
    {
        for (std::size_t restart = 1; seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++restart)
        {
            bool addedSeed = false;
            for (std::size_t parent = 0; parent < previous.size() &&
                seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++parent)
            {
                if (!parentNeedsSearch[parent])
                    continue;
                JointVector seed(previous[parent].joints.size());
                kinematics::detail::BuildIkRestartSeed(
                    (sample + 1) * limits::maximumRestartSeedCount + restart,
                    previous[parent].joints, *specification, seed);
                seeds.push_back({parent, std::move(seed), false});
                addedSeed = true;
            }
            if (!addedSeed)
                break;
        }
    }

    void FinishContinuationSeeds()
    {
        // 직전 관절 자세를 이어 푼 결과가 부족한 구간만 다른 자세와 재시작 자세를 추가로 탐색한다.
        const auto& previous = layers.back();
        for (std::size_t parent = 0; parent < previous.size(); ++parent)
        {
            if (!parentHasValidChild[parent] || !parentHasContinuation[parent])
                parentNeedsSearch[parent] = true;
        }
        if (candidates.size() < previous.size())
            std::fill(parentNeedsSearch.begin(), parentNeedsSearch.end(), true);

        std::vector<std::vector<JointVector>> reflectedSeeds(previous.size());
        for (std::size_t parent = 0; parent < previous.size(); ++parent)
        {
            if (parentNeedsSearch[parent])
                reflectedSeeds[parent] = kinematics::detail::BuildAlternativeIkSeeds(
                    previous[parent].joints, *specification);
        }
        AppendAlternativeSeeds(previous, reflectedSeeds);
        AppendRestartSeeds(previous);
        seedIndex = previous.size();
        expandedSeeds = true;
        phase = Phase::SolveSeed;
    }

    void FinishCandidateSearch()
    {
        if (candidates.empty() && needsRefinement)
        {
            refinementEdge = 0;
            phase = Phase::FinishSample;
            StartNextRefinementValidation();
            return;
        }
        FinishLayer();
    }

    void StartNextRefinementValidation()
    {
        while (refinementEdge < evaluatedEdges.size() && evaluatedEdges[refinementEdge].followsTcp)
            ++refinementEdge;
        if (refinementEdge < evaluatedEdges.size())
        {
            const auto& edge = evaluatedEdges[refinementEdge];
            BeginPathValidation(layers.back()[edge.parent].joints, edge.joints,
                ValidationPurpose::RefinementProbe);
        }
        else
        {
            needsRefinement = foundCollisionFreeRefinementEdge;
            invalidity = foundCollisionFreeRefinementEdge ? JointStateInvalidity::None : rejectedRefinementInvalidity;
            FinishLayer();
        }
    }

    void FailCurrentSample()
    {
        using namespace kinematics::detail;
        const CartesianPose target = ToCartesian(tcpSamples[sample]);
        const std::size_t sampleNumber = sample + 1;
        const std::size_t sampleCount = tcpSamples.size();
        Result failure;
        if (invalidity == JointStateInvalidity::EnvironmentCollision)
            failure = {ErrorCode::EnvironmentContact,
                messages::noCollisionFreeIkSolution};
        else if (invalidity != JointStateInvalidity::None)
            failure = MapJointStateInvalidity(invalidity);
        else
            failure = MapIkFailure(ikFailure);

        Fail(AddPathSampleContext(std::move(failure), sampleNumber, sampleCount,
            *specification, target, invalidity == JointStateInvalidity::None ? &ikFailure : nullptr));
        const std::string& diagnostic = invalidityDiagnostic.empty() ?
            rejectedRefinementDiagnostic : invalidityDiagnostic;
        if (!diagnostic.empty())
            result.message += " (" + diagnostic + ")";
    }

    void RefineCurrentSample()
    {
        if (refinementDepth[sample] >= policy.maximumTcpRefinementPasses ||
            tcpSamples.size() >= policy.maximumPathIntervals)
        {
            Fail({ErrorCode::IkDidNotConverge,
                "LinearPathPlanner: TCP straightness error remains above tolerance at sample " +
                    std::to_string(sample + 1) + "/" + std::to_string(tcpSamples.size()) +
                    " after " + std::to_string(refinementDepth[sample]) + " refinements; largest rejected-branch probe " +
                    std::to_string(maximumSamplePositionErrorMeters * math::millimetersPerMeter) + " mm / " +
                    std::to_string(maximumSampleOrientationErrorRadians * math::milliradiansPerRadian) + " mrad, tolerance " +
                    std::to_string(policy.maximumLinearTcpErrorMeters * math::millimetersPerMeter) + " mm / " +
                    std::to_string(policy.maximumAngularTcpErrorRadians * math::milliradiansPerRadian) + " mrad" +
                    (rejectedRefinementDiagnostic.empty() ? std::string{} :
                        "; first rejected refinement: " + rejectedRefinementDiagnostic)});
            return;
        }

        {
            ZoneScopedN("PlannerRefinement");
            const Pose3 midpoint = kinematics::detail::Interpolate(
                layers.back().front().tcpPose, tcpSamples[sample],
                math::segmentMidpointFraction);
            ++plan.tcpRefinementCount;
            const std::size_t nextDepth = refinementDepth[sample] + 1;
            tcpSamples.insert(tcpSamples.begin() + static_cast<std::ptrdiff_t>(sample), midpoint);
            refinementDepth[sample] = nextDepth;
            refinementDepth.insert(refinementDepth.begin() + static_cast<std::ptrdiff_t>(sample + 1), nextDepth);
        }
        phase = Phase::PrepareSample;
    }

    void FinishEmptyLayer()
    {
        if (needsRefinement)
        {
            RefineCurrentSample();
            return;
        }
        FailCurrentSample();
    }

    void CommitLayerCandidates()
    {
        std::sort(candidates.begin(), candidates.end(), [](const PathCandidate& left, const PathCandidate& right)
        {
            return left.score < right.score;
        });
        layers.push_back(std::move(candidates));
        ++sample;
        phase = Phase::PrepareSample;
        if (sample == tcpSamples.size())
            Finish();
    }

    void FinishLayer()
    {
        if (candidates.empty())
        {
            FinishEmptyLayer();
            return;
        }
        CommitLayerCandidates();
    }

    void Advance(std::size_t workBudget)
    {
        std::size_t work = 0;
        while (state == LinearPathPlanningState::Running && work < workBudget)
        {
            switch (phase)
            {
            case Phase::PrepareSample:
                AdvanceSamplePreparation();
                break;
            case Phase::SolveSeed:
                AdvanceSeedSolve(workBudget, work);
                break;
            case Phase::CheckTcp:
                AdvanceTcpCheck(work);
                break;
            case Phase::ValidatePath:
                AdvancePathValidation(work);
                break;
            case Phase::FinishSample:
                AdvanceSampleFinish();
                break;
            }
        }
    }

    void AdvanceSamplePreparation()
    {
        if (sample >= tcpSamples.size())
        {
            Finish();
            return;
        }
        PrepareSample();
    }

    void AdvanceSeedSolve(std::size_t workBudget, std::size_t& work)
    {
        if (seedIndex >= seeds.size())
        {
            if (expandedSeeds)
                FinishCandidateSearch();
            else
                FinishContinuationSeeds();
            return;
        }

        if (ikSession == 0)
            StartSeed();

        kinematics::IkSessionState ikState;
        {
            ZoneScopedN("PlannerIK");
            const auto before = inverse->GetSingleSeedResult(ikSession).iterations;
            const std::size_t remainingBudget = workBudget - work;
            ikState = inverse->StepSingleSeed(ikSession, remainingBudget);
            const auto after = inverse->GetSingleSeedResult(ikSession).iterations;
            const std::size_t iterationsUsed = after - before;
            plan.ikIterationCount += iterationsUsed;

            // 반복 없이 바로 끝나는 실패도 한 작업으로 세어, 시드 탐색이 호출 예산을 넘지 않게 한다.
            std::size_t consumedWork = iterationsUsed;
            if (ikState != kinematics::IkSessionState::Running && after < options.maxIterations)
                ++consumedWork;
            work += std::max<std::size_t>(1, consumedWork);

            if (ikState == kinematics::IkSessionState::Running)
                return;
            workingSolution = inverse->GetSingleSeedResult(ikSession);
        }

        ikSession = 0;
        PrepareSolution();
    }

    void AdvanceTcpCheck(std::size_t& work)
    {
        const auto& parent = layers.back()[workingParent];
        JointVector& joints = tcpProbeJoints;
        const double fraction = math::tcpStraightnessProbeFraction *
            static_cast<double>(tcpProbe + 1);
        for (std::size_t joint = 0; joint < joints.size(); ++joint)
            joints[joint] = parent.joints[joint] +
                (workingSolution.jointPositions[joint] - parent.joints[joint]) * fraction;

        ZoneScopedN("PlannerTcpStraightness");
        const Pose3 actual = kinematics::detail::FromCartesian(inverse->EvaluateTcp(joints));
        const Pose3 expected = kinematics::detail::Interpolate(parent.tcpPose, tcpSamples[sample], fraction);
        const double positionError = kinematics::detail::Length(kinematics::detail::Subtract(
            actual.positionMeters, expected.positionMeters));
        const double orientationError = kinematics::detail::Length(kinematics::detail::RotationError(
            actual.rotation, expected.rotation));
        maximumSamplePositionErrorMeters = std::max(maximumSamplePositionErrorMeters, positionError);
        maximumSampleOrientationErrorRadians = std::max(maximumSampleOrientationErrorRadians, orientationError);
        ++plan.tcpStraightnessChecks;
        ++tcpProbe;
        ++work;

        if (positionError > policy.maximumLinearTcpErrorMeters ||
            orientationError > policy.maximumAngularTcpErrorRadians)
        {
            needsRefinement = true;
            parentNeedsSearch[workingParent] = true;
            if (workingContinuation)
                parentHasContinuation[workingParent] = true;
            AdvanceToNextSeed();
            return;
        }

        if (tcpProbe == limits::finalTcpStraightnessProbeIndex)
            FinishTcpCheck();
    }

    void AdvancePathValidation(std::size_t& work)
    {
        if (validationStep > validationIntervals)
        {
            CompletePathValidation(JointStateInvalidity::None);
            if (validationPurpose == ValidationPurpose::RefinementProbe)
                StartNextRefinementValidation();
            return;
        }

        const double fraction = static_cast<double>(validationStep) /
            static_cast<double>(validationIntervals);
        for (std::size_t joint = 0; joint < validationSample.size(); ++joint)
            validationSample[joint] = validationStart[joint] +
                (validationEnd[joint] - validationStart[joint]) * fraction;

        ZoneScopedN("PlannerJointPathValidity");
        const auto pathInvalidity = stateValidityChecker
            ? stateValidityChecker(validationSample)
            : JointStateInvalidity::None;
        std::string diagnostic;
        if (pathInvalidity != JointStateInvalidity::None && validityDiagnosticProvider)
            diagnostic = validityDiagnosticProvider();
        ++plan.validityStateChecks;
        ++validationStep;
        ++work;

        if (pathInvalidity == JointStateInvalidity::None)
            return;

        CompletePathValidation(pathInvalidity, std::move(diagnostic));
        if (validationPurpose == ValidationPurpose::RefinementProbe)
            StartNextRefinementValidation();
    }

    void AdvanceSampleFinish()
    {
        if (refinementEdge < evaluatedEdges.size())
            StartNextRefinementValidation();
        else
            FinishLayer();
    }
};

namespace detail
{
void AlignEquivalentJointAngles(
    JointVector& target,
    const JointVector& reference,
    const models::RobotSpecification& specification)
{
    // 회전 관절은 2π를 더하거나 빼도 같은 자세다. 허용 범위 안에서 현재 각도와 가장 가까운 표현을 골라 불필요한 한 바퀴 회전을 막는다.
    for (std::size_t jointIndex = 0; jointIndex < target.size(); ++jointIndex)
    {
        const auto& joint = specification.joints[jointIndex];
        if (joint.motionType == models::JointMotionType::Prismatic)
            continue;
        if (!joint.HasPositionLimits())
        {
            target[jointIndex] += std::round((reference[jointIndex] - target[jointIndex]) /
                math::fullTurnRadians) * math::fullTurnRadians;
            continue;
        }
        const double minimumTurns = std::ceil((joint.minimumPosition - target[jointIndex]) /
            math::fullTurnRadians);
        const double maximumTurns = std::floor((joint.maximumPosition - target[jointIndex]) /
            math::fullTurnRadians);
        if (minimumTurns > maximumTurns)
            continue;

        const double nearestTurns = std::round((reference[jointIndex] - target[jointIndex]) /
            math::fullTurnRadians);
        const double legalTurns = std::clamp(nearestTurns, minimumTurns, maximumTurns);
        target[jointIndex] += legalTurns * math::fullTurnRadians;
    }
}

std::optional<kinematics::IkResult> SolveCollisionFreeIk(
    kinematics::LevenbergMarquardtIk& inverse,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const CartesianPose& target,
    const JointVector& start,
    const kinematics::IkOptions& options,
    const PlanningPolicy& policy,
    JointStateInvalidity& invalidity,
    kinematics::IkResult& ikFailure)
{
    invalidity = JointStateInvalidity::None;
    const auto startInvalidity = ValidateJointState(specification, start, {});
    if (startInvalidity != JointStateInvalidity::None)
    {
        invalidity = startInvalidity;
        return std::nullopt;
    }
    // 먼저 직전 자세를 seed로 풀어 연속성을 유지한다. 다른 seed가 필요하면 시작 자세에서 관절 범위로 정규화한 변화량이 가장 작은 안전한 해를 고른다.
    auto preferred = inverse.SolveSingleSeed(target, start, options);
    if (preferred)
    {
        AlignEquivalentJointAngles(preferred.jointPositions, start, specification);
        const auto pathInvalidity = ValidateJointPath(
            start, preferred.jointPositions, specification, stateValidityChecker, policy);
        if (pathInvalidity == JointStateInvalidity::None)
            return preferred;
        invalidity = pathInvalidity;
    }
    else
    {
        ikFailure = preferred;
        if (preferred.status != kinematics::IkStatus::DidNotConverge &&
            preferred.status != kinematics::IkStatus::JointLimitReached)
            return std::nullopt;
    }

    // 해는 있지만 모든 해의 경로가 막힌 경우와 목표 자체에 IK 해가 없는 경우를 구분한다.
    std::optional<kinematics::IkResult> bestSolution;
    double bestCandidateScore = std::numeric_limits<double>::infinity();
    double bestFailureResidual = ikFailure.message.empty()
        ? std::numeric_limits<double>::infinity()
        : WeightedIkResidual(ikFailure, options);
    const auto considerSolution = [&](kinematics::IkResult solution)
    {
        if (!solution)
        {
            RecordBetterIkFailure(std::move(solution), options, bestFailureResidual, ikFailure);
            return;
        }

        AlignEquivalentJointAngles(solution.jointPositions, start, specification);
        const double score = JointTransitionCost(start, solution.jointPositions, specification);
        if (score >= bestCandidateScore)
            return;

        const auto pathInvalidity = ValidateJointPath(
            start, solution.jointPositions, specification, stateValidityChecker, policy);
        if (pathInvalidity != JointStateInvalidity::None)
        {
            if (invalidity == JointStateInvalidity::None)
                invalidity = pathInvalidity;
            return;
        }

        bestCandidateScore = score;
        bestSolution = std::move(solution);
    };

    for (const auto& seed : kinematics::detail::BuildAlternativeIkSeeds(start, specification))
        considerSolution(inverse.SolveSingleSeed(target, seed, options));

    if (bestSolution)
        return bestSolution;

    // 기존 자세와 반사 분기에서 해를 찾지 못했을 때만 현재 자세를 기준으로 재시작 seed를 시험한다.
    JointVector restartSeed(start.size());
    for (std::size_t sample = 1;
         sample <= limits::maximumRestartSeedCount; ++sample)
    {
        kinematics::detail::BuildIkRestartSeed(sample, start, specification, restartSeed);
        considerSolution(inverse.SolveSingleSeed(target, restartSeed, options));
    }

    if (bestSolution)
        return bestSolution;
    return std::nullopt;
}
} // namespace detail

LinearPathPlanningJob::LinearPathPlanningJob() : impl_(std::make_unique<Impl>()) {}
LinearPathPlanningJob::~LinearPathPlanningJob() = default;
LinearPathPlanningJob::LinearPathPlanningJob(LinearPathPlanningJob&&) noexcept = default;
LinearPathPlanningJob& LinearPathPlanningJob::operator=(LinearPathPlanningJob&&) noexcept = default;

Result LinearPathPlanningJob::Begin(const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification, const JointVector& startJoints,
    const CartesianPose& startTcp, kinematics::LevenbergMarquardtIk& inverse,
    const StateValidityChecker& stateValidityChecker, const PlanningPolicy& policy,
    const std::function<std::string()>& validityDiagnosticProvider)
{
    return impl_->Begin(command, specification, startJoints, startTcp, inverse,
        stateValidityChecker, policy, validityDiagnosticProvider);
}

LinearPathPlanningState LinearPathPlanningJob::Advance(std::size_t workBudget)
{
    if (impl_->state == LinearPathPlanningState::Running && workBudget > 0 &&
        std::this_thread::get_id() != impl_->ownerThread)
    {
        impl_->Fail({ErrorCode::InvalidCommand, messages::wrongThread});
        return impl_->state;
    }
    impl_->Advance(workBudget);
    return impl_->state;
}

void LinearPathPlanningJob::Cancel() noexcept
{
    impl_->CancelActiveIkSession();
    if (impl_->state == LinearPathPlanningState::Running)
    {
        impl_->state = LinearPathPlanningState::Cancelled;
        impl_->result = {ErrorCode::Cancelled, "LinearPathPlanner: planning cancelled"};
    }
    impl_->ReleasePlanningStorage();
}

LinearPathPlanningState LinearPathPlanningJob::GetState() const noexcept { return impl_->state; }
const Result& LinearPathPlanningJob::GetResult() const noexcept { return impl_->result; }
const LinearPathPlan& LinearPathPlanningJob::GetPlan() const noexcept { return impl_->plan; }

std::optional<LinearPathPlan> LinearPathPlanningJob::TakePlan()
{
    if (impl_->state != LinearPathPlanningState::Completed || impl_->planTaken)
        return std::nullopt;
    impl_->planTaken = true;
    LinearPathPlan plan = std::move(impl_->plan);
    impl_->plan = {};
    return plan;
}

Result BuildLinearPath(const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification, const JointVector& startJoints,
    const CartesianPose& startTcp, kinematics::LevenbergMarquardtIk& inverse,
    const StateValidityChecker& stateValidityChecker, LinearPathPlan& plan,
    const PlanningPolicy& policy,
    const std::function<std::string()>& validityDiagnosticProvider)
{
    LinearPathPlanningJob job;
    const Result begin = job.Begin(command, specification, startJoints, startTcp,
        inverse, stateValidityChecker, policy, validityDiagnosticProvider);
    if (!begin)
        return begin;
    while (job.GetState() == LinearPathPlanningState::Running)
        job.Advance(std::numeric_limits<std::size_t>::max());
    if (job.GetState() == LinearPathPlanningState::Completed)
    {
        auto completedPlan = job.TakePlan();
        if (completedPlan)
            plan = std::move(*completedPlan);
    }
    return job.GetResult();
}

} // namespace grasplink::robotics::planning
