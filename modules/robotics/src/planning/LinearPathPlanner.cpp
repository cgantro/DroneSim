#include "robotics/planning/LinearPathPlanner.h"

#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#endif

namespace grasplink::robotics::planning
{
namespace
{
using models::Pose3;

struct LinearPathPlannerLimits final
{
    static constexpr std::size_t maximumRestartSeedCount = 16;
    static constexpr std::size_t maximumIkCandidatesPerSample = 8;
    static constexpr std::size_t maximumIkSeedAttemptsPerSample = 32;
    static constexpr std::size_t maximumTcpRefinementPasses = 12;
    static constexpr std::size_t maximumPathIntervals = 4096;
    static constexpr std::size_t initialTcpSampleCapacity = 64;
    static constexpr std::size_t finalTcpStraightnessProbeIndex = 3;
};

struct LinearPathPlannerMath final
{
    static constexpr double fullTurnRadians = 2.0 * 3.14159265358979323846;
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
};

struct LinearPathPlannerMessages final
{
    inline static constexpr char invalidPlanningPolicy[] =
        "SimRobotController: invalid linear path planning policy";
    inline static constexpr char invalidTcpTarget[] = "SimRobotController: invalid TCP target";
    inline static constexpr char pathDurationOutOfRange[] =
        "SimRobotController: path duration exceeds numeric range";
    inline static constexpr char noCollisionFreeIkSolution[] =
        "SimRobotController: no collision-free IK solution";
    inline static constexpr char invalidPlannedJointState[] =
        "SimRobotController: invalid joint state in planned path";
    inline static constexpr char plannedStateExceedsJointLimit[] =
        "SimRobotController: planned state exceeds a joint limit";
    inline static constexpr char plannedStateOverlapsEnvironment[] =
        "SimRobotController: planned state overlaps the environment";
    inline static constexpr char plannedRobotLinksOverlap[] =
        "SimRobotController: planned robot links overlap";
    inline static constexpr char attachedObjectOverlapsScene[] =
        "SimRobotController: attached object overlaps the scene";
    inline static constexpr char unknownJointStateInvalidity[] =
        "SimRobotController: unknown joint-state invalidity";
};

Result AddPathSampleContext(Result failure, std::size_t sample, std::size_t sampleCount,
    const models::RobotSpecification& specification, const CartesianPose& target,
    const kinematics::IkResult* ikFailure = nullptr)
{
    std::string context = "SimRobotController: TCP path sample " + std::to_string(sample) + "/" +
        std::to_string(sampleCount) + " failed at (" +
        std::to_string(target.positionMeters[0]) + ", " +
        std::to_string(target.positionMeters[1]) + ", " +
        std::to_string(target.positionMeters[2]) + ") m";
    if (ikFailure)
    {
        if (ikFailure->status == kinematics::IkStatus::JointLimitReached)
        {
            for (std::size_t joint = 0; joint < ikFailure->jointPositions.size(); ++joint)
            {
                const auto& limits = specification.joints[joint];
                const double tolerance = std::max(LinearPathPlannerMath::jointLimitBoundaryToleranceFloor,
                    (limits.maximumPosition - limits.minimumPosition) *
                        LinearPathPlannerMath::jointLimitBoundaryToleranceRatio);
                const double position = ikFailure->jointPositions[joint];
                if (std::abs(position - limits.minimumPosition) <= tolerance ||
                    std::abs(position - limits.maximumPosition) <= tolerance)
                {
                    context += "; IK candidate reached " + std::string(limits.name) + " limit";
                    break;
                }
            }
        }
        context += "; IK residual " + std::to_string(ikFailure->positionErrorMeters) +
            " m / " + std::to_string(ikFailure->orientationErrorRadians) +
            " rad after " + std::to_string(ikFailure->iterations) + " iterations; candidate_positions=[";
        for (std::size_t joint = 0; joint < ikFailure->jointPositions.size(); ++joint)
        {
            const auto& jointSpec = specification.joints[joint];
            const double position = jointSpec.motionType == models::JointMotionType::Prismatic
                ? ikFailure->jointPositions[joint]
                : ikFailure->jointPositions[joint] * LinearPathPlannerMath::degreesPerFullTurn /
                    LinearPathPlannerMath::fullTurnRadians;
            context += (joint == 0 ? "" : ",") + std::string(jointSpec.name) + "=" +
                std::to_string(position) +
                (jointSpec.motionType == models::JointMotionType::Prismatic ? " m" : " deg");
        }
        context += "]";
    }
    failure.message = std::move(context) + ": " + failure.message;
    return failure;
}

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
        policy.maximumPathIntervals <= LinearPathPlannerLimits::maximumPathIntervals &&
        policy.maximumIkCandidatesPerSample > 0 &&
        policy.maximumIkCandidatesPerSample <= LinearPathPlannerLimits::maximumIkCandidatesPerSample &&
        policy.maximumIkSeedAttemptsPerSample > 0 &&
        policy.maximumIkSeedAttemptsPerSample >= policy.maximumIkCandidatesPerSample &&
        policy.maximumIkSeedAttemptsPerSample <= LinearPathPlannerLimits::maximumIkSeedAttemptsPerSample &&
        policy.maximumTcpRefinementPasses <= LinearPathPlannerLimits::maximumTcpRefinementPasses &&
        std::isfinite(policy.maximumLinearTcpErrorMeters) && policy.maximumLinearTcpErrorMeters > 0.0 &&
        std::isfinite(policy.maximumAngularTcpErrorRadians) && policy.maximumAngularTcpErrorRadians > 0.0;
}

std::size_t JointPathIntervals(const JointVector& start, const JointVector& end,
    const models::RobotSpecification& specification, const PlanningPolicy& policy)
{
    double largestSpacingCount = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
    {
        const double spacing = specification.joints[joint].motionType == models::JointMotionType::Prismatic
            ? policy.prismaticJointCollisionSampleSpacingMeters
            : policy.jointCollisionSampleSpacingRadians;
        largestSpacingCount = std::max(largestSpacingCount,
            std::abs(end[joint] - start[joint]) / spacing);
    }
    return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(largestSpacingCount)));
}

double JointTransitionCost(const JointVector& start, const JointVector& end,
    const models::RobotSpecification& specification)
{
    double score = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
    {
        const auto& limits = specification.joints[joint];
        const double range = std::max(LinearPathPlannerMath::jointLimitRangeFloor,
            limits.maximumPosition - limits.minimumPosition);
        const double delta = (end[joint] - start[joint]) / range;
        score += delta * delta;
        const double margin = std::min(end[joint] - limits.minimumPosition,
            limits.maximumPosition - end[joint]) / range;
        // 다음 경로 점에서 움직일 여유를 남기도록 관절 한계에 가까운 자세에 작은 불이익을 준다.
        const double deficit = std::max(0.0,
            LinearPathPlannerMath::jointLimitSearchMarginFraction - margin) /
            LinearPathPlannerMath::jointLimitSearchMarginFraction;
        score += LinearPathPlannerMath::jointLimitPenaltyWeight * deficit * deficit;
    }
    return score;
}

bool SameJointCandidate(const JointVector& left, const JointVector& right)
{
    for (std::size_t joint = 0; joint < left.size(); ++joint)
    {
        if (std::abs(left[joint] - right[joint]) >
            LinearPathPlannerMath::jointLimitComparisonTolerance)
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

    void ResetForBegin(const LinearPathMoveCommand& moveCommand,
        const models::RobotSpecification& robot, const JointVector& joints,
        kinematics::LevenbergMarquardtIk& ik, const StateValidityChecker& checker,
        const PlanningPolicy& planningPolicy,
        const std::function<std::string()>& diagnosticProvider)
    {
        if (ikSession != 0 && inverse != nullptr)
            inverse->CancelSingleSeed(ikSession);
        ikSession = 0;
        stateValidityChecker = {};
        command = moveCommand;
        specification = &robot;
        startJoints = joints;
        inverse = &ik;
        stateValidityChecker = checker;
        validityDiagnosticProvider = diagnosticProvider;
        policy = planningPolicy;
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
            return {ErrorCode::InvalidCommand, LinearPathPlannerMessages::invalidTcpTarget};
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
        tcpSamples.reserve(LinearPathPlannerLimits::initialTcpSampleCapacity);
        std::size_t totalIntervals = 0;
        Pose3 segmentStart = start;
        for (const Pose3& end : targets)
        {
            const double distance = kinematics::detail::Length(
                kinematics::detail::Subtract(end.positionMeters, segmentStart.positionMeters));
            const double rotation = kinematics::detail::Length(
                kinematics::detail::RotationError(end.rotation, segmentStart.rotation));
            const double intervals = std::max({LinearPathPlannerMath::minimumPathIntervalCount,
                std::ceil(distance / policy.linearPositionSampleSpacingMeters),
                std::ceil(rotation / policy.linearOrientationSampleSpacingRadians)});
            if (!std::isfinite(intervals) || intervals > static_cast<double>(policy.maximumPathIntervals) ||
                totalIntervals > policy.maximumPathIntervals ||
                intervals > static_cast<double>(policy.maximumPathIntervals - totalIntervals))
            {
                return {ErrorCode::InvalidCommand, "SimRobotController: linear path exceeds " +
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
            return Fail({ErrorCode::InvalidCommand, LinearPathPlannerMessages::invalidPlanningPolicy});
        if (const Result targetsResult = ReadAndValidateTargets(tcp); !targetsResult)
            return Fail(targetsResult);

        plan.hasMotion = HasMotion();
        if (!plan.hasMotion)
        {
            state = LinearPathPlanningState::Completed;
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
        result = std::move(failure);
        state = LinearPathPlanningState::Failed;
        stateValidityChecker = {};
        validityDiagnosticProvider = {};
        return result;
    }

    void ReleasePlanningStorage()
    {
        stateValidityChecker = {};
        validityDiagnosticProvider = {};
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
                Fail({ErrorCode::InvalidCommand, LinearPathPlannerMessages::pathDurationOutOfRange});
                return;
            }
            plan.points.push_back({std::move(selectedJoints[point]), ToCartesian(tcpSamples[point]),
                std::max(duration, LinearPathPlannerMath::minimumSegmentDurationSeconds)});
            plan.plannedLinearVelocity = std::max(plan.plannedLinearVelocity, distance / duration);
            plan.plannedAngularVelocity = std::max(plan.plannedAngularVelocity, rotation / duration);
        }
        state = LinearPathPlanningState::Completed;
        result = Result::Success();
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
        const double residual = failure.positionErrorMeters +
            options.orientationWeightMetersPerRadian * failure.orientationErrorRadians;
        if (residual < bestFailureResidual)
        {
            bestFailureResidual = residual;
            ikFailure = std::move(failure);
        }
        if (workingContinuation)
            parentNeedsSearch[workingParent] = true;
    }

    void PrepareSolution()
    {
        if (!workingSolution)
        {
            RecordIkFailure(std::move(workingSolution));
            ++seedIndex;
            phase = Phase::SolveSeed;
            return;
        }

        const auto& parent = layers.back()[workingParent];
        AlignEquivalentJointAngles(workingSolution.jointPositions, parent.joints, *specification);
        workingScore = parent.score + JointTransitionCost(
            parent.joints, workingSolution.jointPositions, *specification);
        auto duplicate = std::find_if(candidates.begin(), candidates.end(), [&](const PathCandidate& candidate)
        {
            return SameJointCandidate(candidate.joints, workingSolution.jointPositions);
        });
        if (duplicate != candidates.end() && workingScore >= duplicate->score)
        {
            if (duplicate->previousCandidate == workingParent)
            {
                parentHasValidChild[workingParent] = true;
                if (workingContinuation)
                    parentHasContinuation[workingParent] = true;
            }
            ++seedIndex;
            phase = Phase::SolveSeed;
            return;
        }
        if (candidates.size() >= policy.maximumIkCandidatesPerSample)
        {
            const auto worst = std::max_element(candidates.begin(), candidates.end(),
                [](const PathCandidate& left, const PathCandidate& right)
                {
                    return left.score < right.score;
                });
            if (workingScore > worst->score)
            {
                ++seedIndex;
                phase = Phase::SolveSeed;
                return;
            }
        }

        for (const auto& edge : evaluatedEdges)
        {
            if (edge.parent == workingParent &&
                std::equal(workingSolution.jointPositions.begin(),
                    workingSolution.jointPositions.end(), edge.joints.begin()))
            {
                if (workingContinuation)
                    parentHasContinuation[workingParent] = true;
                ++seedIndex;
                phase = Phase::SolveSeed;
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
        if (tcpProbe < LinearPathPlannerLimits::finalTcpStraightnessProbeIndex)
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
        validationIntervals = JointPathIntervals(validationStart, validationEnd, *specification, policy);
        phase = Phase::ValidatePath;
    }

    void CompletePathValidation(JointStateInvalidity pathInvalidity, std::string diagnostic = {})
    {
        if (validationPurpose == ValidationPurpose::RefinementProbe)
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
            return;
        }

        if (pathInvalidity != JointStateInvalidity::None)
        {
            if (invalidity == JointStateInvalidity::None)
            {
                invalidity = pathInvalidity;
                invalidityDiagnostic = std::move(diagnostic);
            }
            parentNeedsSearch[workingParent] = true;
            if (workingContinuation)
                parentHasContinuation[workingParent] = true;
            ++seedIndex;
            phase = Phase::SolveSeed;
            return;
        }

        const auto& parent = layers.back()[workingParent];
        parentHasValidChild[workingParent] = true;
        auto duplicate = std::find_if(candidates.begin(), candidates.end(), [&](const PathCandidate& candidate)
        {
            return SameJointCandidate(candidate.joints, workingSolution.jointPositions);
        });
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
            const auto worst = std::max_element(candidates.begin(), candidates.end(),
                [](const PathCandidate& left, const PathCandidate& right)
                {
                    return left.score < right.score;
                });
            candidates.erase(worst);
        }
        ++seedIndex;
        phase = Phase::SolveSeed;
    }

    void FinishContinuationSeeds()
    {
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
        for (std::size_t branch = 0; seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++branch)
        {
            bool addedSeed = false;
            for (std::size_t parent = 0; parent < previous.size() &&
                seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++parent)
            {
                if (parentNeedsSearch[parent] && branch < reflectedSeeds[parent].size())
                {
                    seeds.push_back({parent, reflectedSeeds[parent][branch], false});
                    addedSeed = true;
                }
            }
            if (!addedSeed)
                break;
        }
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
                    (sample + 1) * LinearPathPlannerLimits::maximumRestartSeedCount + restart,
                    previous[parent].joints, *specification, seed);
                seeds.push_back({parent, std::move(seed), false});
                addedSeed = true;
            }
            if (!addedSeed)
                break;
        }
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
        if (invalidity == JointStateInvalidity::EnvironmentCollision)
            Fail(AddPathSampleContext(
                {ErrorCode::EnvironmentContact, LinearPathPlannerMessages::noCollisionFreeIkSolution},
                sampleNumber, sampleCount, *specification, target));
        else if (invalidity != JointStateInvalidity::None)
            Fail(AddPathSampleContext(MapJointStateInvalidity(invalidity), sampleNumber,
                sampleCount, *specification, target));
        else
            Fail(AddPathSampleContext(MapIkFailure(ikFailure), sampleNumber, sampleCount,
                *specification, target, &ikFailure));
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
                "SimRobotController: TCP straightness error remains above tolerance at sample " +
                    std::to_string(sample + 1) + "/" + std::to_string(tcpSamples.size()) +
                    " after " + std::to_string(refinementDepth[sample]) + " refinements; largest rejected-branch probe " +
                    std::to_string(maximumSamplePositionErrorMeters * LinearPathPlannerMath::millimetersPerMeter) + " mm / " +
                    std::to_string(maximumSampleOrientationErrorRadians * LinearPathPlannerMath::milliradiansPerRadian) + " mrad, tolerance " +
                    std::to_string(policy.maximumLinearTcpErrorMeters * LinearPathPlannerMath::millimetersPerMeter) + " mm / " +
                    std::to_string(policy.maximumAngularTcpErrorRadians * LinearPathPlannerMath::milliradiansPerRadian) + " mrad" +
                    (rejectedRefinementDiagnostic.empty() ? std::string{} :
                        "; first rejected refinement: " + rejectedRefinementDiagnostic)});
            return;
        }

        {
            ZoneScopedN("PlannerRefinement");
            const Pose3 midpoint = kinematics::detail::Interpolate(
                layers.back().front().tcpPose, tcpSamples[sample],
                LinearPathPlannerMath::segmentMidpointFraction);
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
        JointVector joints(parent.joints.size());
        const double fraction = LinearPathPlannerMath::tcpStraightnessProbeFraction *
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
            ++seedIndex;
            phase = Phase::SolveSeed;
            return;
        }

        if (tcpProbe == LinearPathPlannerLimits::finalTcpStraightnessProbeIndex)
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

JointStateInvalidity ValidateJointPath(
    const JointVector& start,
    const JointVector& end,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const PlanningPolicy& policy)
{
    // 관절 하나의 변화량이 설정한 간격보다 크면 그 사이 자세도 나눠 검사한다. 시작 자세는 이미 검증됐다고 보고 끝 자세까지 확인한다.
    const auto startInvalidity = ValidateJointState(specification, start, {});
    if (startInvalidity != JointStateInvalidity::None)
        return startInvalidity;
    const auto endInvalidity = ValidateJointState(specification, end, {});
    if (endInvalidity != JointStateInvalidity::None)
        return endInvalidity;

    const std::size_t intervals = JointPathIntervals(start, end, specification, policy);
    JointVector sample(start.size());
    for (std::size_t step = 1; step <= intervals; ++step)
    {
        const double fraction = static_cast<double>(step) / static_cast<double>(intervals);
        for (std::size_t joint = 0; joint < start.size(); ++joint)
            sample[joint] = start[joint] + (end[joint] - start[joint]) * fraction;
        // 양 끝 관절각이 유한하고 한계 안에 있으면 선형 보간한 중간 각도도 그 범위 안에 있다. 따라서 매 표본에서는 비용이 큰 충돌 검사만 수행한다.
        const auto invalidity = stateValidityChecker ?
            stateValidityChecker(sample) : JointStateInvalidity::None;
        if (invalidity != JointStateInvalidity::None)
            return invalidity;
    }
    return JointStateInvalidity::None;
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
        if (joints[joint] < limits.minimumPosition || joints[joint] > limits.maximumPosition)
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
        return {ErrorCode::InvalidCommand, LinearPathPlannerMessages::invalidPlannedJointState};
    case JointStateInvalidity::JointLimitViolation:
        return {ErrorCode::JointLimitReached, LinearPathPlannerMessages::plannedStateExceedsJointLimit};
    case JointStateInvalidity::EnvironmentCollision:
        return {ErrorCode::EnvironmentContact, LinearPathPlannerMessages::plannedStateOverlapsEnvironment};
    case JointStateInvalidity::SelfCollision:
        return {ErrorCode::SelfCollision, LinearPathPlannerMessages::plannedRobotLinksOverlap};
    case JointStateInvalidity::AttachedObjectCollision:
        return {ErrorCode::AttachedObjectCollision, LinearPathPlannerMessages::attachedObjectOverlapsScene};
    }
    return {ErrorCode::Fault, LinearPathPlannerMessages::unknownJointStateInvalidity};
}

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
        const double minimumTurns = std::ceil((joint.minimumPosition - target[jointIndex]) /
            LinearPathPlannerMath::fullTurnRadians);
        const double maximumTurns = std::floor((joint.maximumPosition - target[jointIndex]) /
            LinearPathPlannerMath::fullTurnRadians);
        if (minimumTurns > maximumTurns)
            continue;

        const double nearestTurns = std::round((reference[jointIndex] - target[jointIndex]) /
            LinearPathPlannerMath::fullTurnRadians);
        const double legalTurns = std::clamp(nearestTurns, minimumTurns, maximumTurns);
        target[jointIndex] += legalTurns * LinearPathPlannerMath::fullTurnRadians;
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
    const auto residual = [&](const kinematics::IkResult& result)
    {
        return result.positionErrorMeters + options.orientationWeightMetersPerRadian *
            result.orientationErrorRadians;
    };
    double bestFailureResidual = ikFailure.message.empty()
        ? std::numeric_limits<double>::infinity()
        : residual(ikFailure);
    const auto considerSolution = [&](kinematics::IkResult solution)
    {
        if (!solution)
        {
            const double candidateResidual = residual(solution);
            if (candidateResidual < bestFailureResidual)
            {
                bestFailureResidual = candidateResidual;
                ikFailure = std::move(solution);
            }
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
         sample <= LinearPathPlannerLimits::maximumRestartSeedCount; ++sample)
    {
        kinematics::detail::BuildIkRestartSeed(sample, start, specification, restartSeed);
        considerSolution(inverse.SolveSingleSeed(target, restartSeed, options));
    }

    if (bestSolution)
        return bestSolution;
    return std::nullopt;
}

LinearPathPlanningJob::LinearPathPlanningJob() : impl_(std::make_unique<Impl>()) {}
LinearPathPlanningJob::~LinearPathPlanningJob() = default;
LinearPathPlanningJob::LinearPathPlanningJob(LinearPathPlanningJob&&) noexcept = default;
LinearPathPlanningJob& LinearPathPlanningJob::operator=(LinearPathPlanningJob&&) noexcept = default;

Result LinearPathPlanningJob::Begin(const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification, const JointVector& startJoints,
    const CartesianPose& startTcp, kinematics::LevenbergMarquardtIk& inverse,
    const StateValidityChecker& stateValidityChecker, const PlanningPolicy& policy)
{
    return Begin(command, specification, startJoints, startTcp, inverse,
        stateValidityChecker, policy, {});
}

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
    impl_->Advance(workBudget);
    return impl_->state;
}

void LinearPathPlanningJob::Cancel() noexcept
{
    if (impl_->state == LinearPathPlanningState::Running && impl_->ikSession != 0)
    {
        try { impl_->inverse->CancelSingleSeed(impl_->ikSession); }
        catch (...) {}
        impl_->ikSession = 0;
    }
    if (impl_->state == LinearPathPlanningState::Running)
        impl_->state = LinearPathPlanningState::Cancelled;
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
    impl_->ReleasePlanningStorage();
    return plan;
}

Result BuildLinearPath(const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification, const JointVector& startJoints,
    const CartesianPose& startTcp, kinematics::LevenbergMarquardtIk& inverse,
    const StateValidityChecker& stateValidityChecker, LinearPathPlan& plan,
    const PlanningPolicy& policy)
{
    return BuildLinearPath(command, specification, startJoints, startTcp, inverse,
        stateValidityChecker, plan, policy, {});
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
