#pragma once

#include <benchmark/benchmark.h>

#include "robotics/core/ControlTypes.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <cstddef>
#include <string_view>
#include <vector>

namespace grasplink::benchmarks
{
using grasplink::robotics::JointVector;
using grasplink::robotics::CartesianPose;

inline const JointVector kSeed(6, 0.0);

CartesianPose MakeTargetPose();

// 한 목표에 대해 목표 자세, 원래 관절값, 역기구학 결과와 관절 경로 유효성을 함께 보관합니다.
struct RuntimePathCase
{
    CartesianPose targetPose;
    JointVector reachableTargetJoints;
    std::size_t endpointIkIterations = 0;
    bool endpointIkConvergedFromStart = false;
    bool directJointPathValid = false;
};

std::vector<RuntimePathCase> RuntimePathCases(std::size_t caseCount);

// Result의 오류 코드를 벤치마크 출력에 사용할 한국어 설명으로 바꿉니다.
std::string_view ErrorCodeMessageKorean(grasplink::robotics::ErrorCode code);

void Fk(benchmark::State& state);
void LoadUrdfAndGlb(benchmark::State& state);
void BuildUrdfCollisionGeometry(benchmark::State& state);
void BenchmarkLmIk(benchmark::State& state);
void LinearPathPlanning(benchmark::State& state);
void IncrementalLinearPathPlanning(benchmark::State& state);
void InteractiveLinearPathStress(benchmark::State& state);
void PosePlanEndpointStress(benchmark::State& state);
void PosePlanWithRobotCollisions(benchmark::State& state);
void EndpointIkGeneratedReachable(benchmark::State& state);
void MoveJJointPathValidation(benchmark::State& state);
void MoveJCompletion(benchmark::State& state);
void JoltMoveJDirectPath(benchmark::State& state);
void RuntimeLinearPathStress(benchmark::State& state);
void ControlPhysicsTick250Hz(benchmark::State& state);
}
