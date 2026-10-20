#include "BenchmarkSupport.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/planning/JointPathPlanner.h"

#include <random>
#include <utility>

namespace grasplink::benchmarks
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;

const JointVector kTargetJoints{0.04, -0.05, 0.03, 0.02, -0.03, 0.04};

// 벤치마크 결과를 읽을 때 오류 코드도 이해할 수 있도록 한국어로 표시한다.
std::string_view ErrorCodeMessageKorean(ErrorCode code)
{
    switch (code)
    {
    case ErrorCode::None: return "오류 없음";
    case ErrorCode::NotConnected: return "제어기에 연결되지 않았습니다";
    case ErrorCode::InvalidCommand: return "요청 값이 올바르지 않습니다";
    case ErrorCode::Busy: return "제어기가 다른 작업을 처리하고 있습니다";
    case ErrorCode::Fault: return "제어기 또는 시뮬레이터에서 오류가 발생했습니다";
    case ErrorCode::Unsupported: return "지원하지 않는 기능입니다";
    case ErrorCode::TransportError: return "통신 오류가 발생했습니다";
    case ErrorCode::Unreachable: return "목표 자세에 도달할 수 없습니다";
    case ErrorCode::JointLimitReached: return "관절 제한에 걸렸습니다";
    case ErrorCode::IkDidNotConverge: return "역기구학 계산이 수렴하지 않았습니다";
    case ErrorCode::EnvironmentContact: return "환경과 부딪혔습니다";
    case ErrorCode::SelfCollision: return "로봇 링크끼리 부딪혔습니다";
    case ErrorCode::AttachedObjectCollision: return "로봇에 붙인 물체와 부딪혔습니다";
    case ErrorCode::Cancelled: return "작업이 취소되었습니다";
    }
    return "알 수 없는 오류입니다";
}

// 고정된 관절 자세에서 목표 Tool 자세를 한 번 계산한다.
CartesianPose MakeTargetPose()
{
    LevenbergMarquardtIk solver(kHcr12a);
    return solver.EvaluateTcp(kTargetJoints);
}

// 고정 난수 시드로 반복 가능한 경로 계획 입력을 만든다.
std::vector<RuntimePathCase> RuntimePathCases(std::size_t caseCount)
{
    std::mt19937_64 random(0x47524153504C494EULL);
    LevenbergMarquardtIk inverse(kHcr12a);
    std::vector<RuntimePathCase> generated;
    generated.reserve(caseCount);
    for (std::size_t sample = 0; sample < caseCount; ++sample)
    {
        JointVector targetJoints(kHcr12a.jointCount);
        for (std::size_t joint = 0; joint < kHcr12a.jointCount; ++joint)
        {
            const auto& limits = kHcr12a.joints[joint];
            std::uniform_real_distribution<double> position(
                limits.minimumPosition * 0.8, limits.maximumPosition * 0.8);
            targetJoints[joint] = position(random);
        }
        RuntimePathCase testCase;
        testCase.targetPose = inverse.EvaluateTcp(targetJoints);
        testCase.reachableTargetJoints = std::move(targetJoints);
        const IkResult endpoint = inverse.SolveSingleSeed(testCase.targetPose, kSeed);
        testCase.endpointIkIterations = endpoint.iterations;
        testCase.endpointIkConvergedFromStart = endpoint.Ok();
        testCase.directJointPathValid = ValidateJointPath(
            kSeed, testCase.reachableTargetJoints, kHcr12a, {}) == JointStateInvalidity::None;
        generated.push_back(std::move(testCase));
    }
    return generated;
}
}
