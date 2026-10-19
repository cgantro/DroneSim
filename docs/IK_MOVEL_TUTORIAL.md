# HCR-12A TCP 이동: IK와 MoveL

이 문서는 관절각을 직접 지정하지 않고 HCR-12A의 TCP를 이동하는 방법을 설명한다. TCP는 공구 끝에서 작업 위치와 방향을 나타내는 기준점이다. 상자에 수직으로 접근하거나 물체를 든 채 일정 높이로 올리는 작업처럼 공구 끝이 지나야 할 직선이 중요하면 MoveL을 사용한다. 목표 자세 하나만 계산해 관절 공간에서 이동하는 MoveJ(`MovePose`)는 TCP가 직선을 따라가지 않을 수 있다.

## 좌표와 단위

예제의 목표 위치는 모두 Robot base 기준 미터(m)이며, 방향 quaternion은 `[x,y,z,w]` 순서다. 관절각은 라디안(rad), TCP 선속도는 m/s, TCP 각속도는 rad/s로 전달한다. Scene에서 로봇 전체를 옮기거나 회전했다면 World 목표를 먼저 Robot base 좌표로 변환해야 한다. `SimRobotController`는 모델 FK로 목표 TCP를 계산한다. 이 값은 실제 장치에서 측정한 위치가 아니다.

관절각으로 TCP 자세를 계산하는 과정을 정방향 기구학(FK), 원하는 TCP 자세를 만들 관절각을 찾는 과정을 역기구학(IK)이라고 한다. MoveL은 IK를 한 번만 계산하지 않는다. TCP 직선 위 여러 자세에서 IK를 풀고, 관절 자세 사이의 경로도 확인해야 한다.

## 감쇠 최소제곱법(DLS) 계산 원리

Jacobian `J`는 관절을 조금 움직였을 때 TCP 위치와 방향이 얼마나 바뀌는지 나타낸다. 현재 TCP와 목표 TCP 사이의 오차를 `e`라고 하면, Damped Least Squares(DLS)는 다음 관절 변화량을 계산한다.

```text
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

`Δq`는 관절별 변화량 [rad]이며, `e`에는 위치 [m]와 방향 [rad] 오차가 들어간다. 구현에서는 방향 오차에 `orientationWeightMetersPerRadian`을 곱해 위치 오차와 함께 계산한다. 이 가중치는 회전 오차를 길이 단위로 비교하기 위한 값이지, 실제 공구 길이가 아니다. `λ`(damping)는 팔이 펴져 특정 방향으로 움직이기 어려운 자세에서 역행렬 계산이 불안정해지거나 관절 변화량이 커지는 현상을 줄인다. 감쇠는 계산 안정성을 돕지만 모든 목표의 해를 보장하지는 않는다.

단위를 확인하기 위한 간단한 1차원 예를 보자. 회전 관절을 `0.01 rad` 움직였을 때 TCP가 목표 방향으로 약 `0.005 m` 이동하는 자세라면 단순화한 Jacobian은 `J = 0.5 m/rad`다. 목표까지 남은 오차가 `e = 0.01 m`, damping이 `λ = 0.1 m/rad`라고 가정하면 다음과 같다.

```text
Δq = (0.5 × 0.01) / (0.5² + 0.1²)
   ≈ 0.01923 rad
예상 TCP 이동량 = J × Δq ≈ 0.009615 m
남은 오차       ≈ 0.000385 m = 0.385 mm
```

이 축약 예는 방향 성분, 나머지 다섯 관절, 관절 한계와 반복 갱신을 제외한 설명용 계산이다. 실제 HCR-12A IK는 Robot base에서 계산한 6축 Jacobian과 위치·방향 오차를 사용해 반복한다. 매 반복에서 관절 제한과 수렴 상태도 확인한다.

## MoveL이 경로를 계획하는 순서

현재 `SimRobotController::BeginLinearPathPlanning`은 다음 순서로 동작한다.

1. 목표가 있는지, 네 가지 속도·가속도 제한이 모두 유한한 양수인지 확인한다.
2. 현재 관절각과 FK로 계산한 현재 TCP 자세를 시작점으로 삼는다.
3. 각 목표 waypoint까지 TCP 위치를 직선으로 보간하고, 방향은 quaternion의 최단 회전 경로로 보간한다. 기본 표본 간격은 위치 10 mm, 방향 0.05 rad다.
4. 각 TCP 표본에 DLS IK를 적용한다. 직전 표본에서 이어 온 관절 해를 먼저 사용하고, 필요하면 제한된 대체 관절 분기를 확인한다.
5. 연속한 관절 표본 사이의 경로와 FK로 다시 계산한 TCP 직선 오차를 검사한다. 기본 충돌 간격은 관절 변화 0.08 rad, 최대 TCP 직선 오차는 위치 1 mm와 방향 0.001 rad다. 경로 구간은 최대 4096개까지 허용한다.
6. 전체 계획이 성공해야 동작을 시작한다. 증분 계획은 `AdvanceMotionPlanning`으로 나누어 처리하고, 이후 `Update(dtSeconds)`가 실행을 진행한다.

Simulation에 충돌 검사 callback을 연결한 경우에만 계획 중 관절 표본의 충돌을 검사한다. 아래 예제에는 callback이 등록되지 않아 장애물 회피나 실제 Jolt 환경에서의 충돌 검증을 하지 않는다. 표본 기반 검사는 표본 사이 모든 자세에 충돌이 없음을 증명하지도 않는다.

## C++ 예제

아래 코드는 모델의 시작 자세에서 알려진 관절 자세의 FK 결과를 목표 TCP로 사용한다. 로봇을 시뮬레이션하고 계획까지 확인하는 작업 흐름을 보여 준다. `SimRobotController` 자체는 Flecs Entity나 Physics Body를 움직이지 않는다. 앱에서 Controller 갱신과 장면·물리 어댑터를 별도로 연결한다.

```cpp
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <cstddef>
#include <iostream>

int main()
{
    using namespace grasplink::robotics;
    using namespace grasplink::robotics::backends::simulation;

    const auto& model = models::hanwha::kHcr12a;
    SimRobotController controller(model);
    if (!controller.Connect())
        return 1;

    // 목표 관절각을 FK로 바꿔 TCP 목표를 만든다. 관절각은 J1부터 J6까지 [rad] 순서다.
    kinematics::DampedLeastSquaresIk fk(model);
    const JointVector targetJoints{0.12, -0.08, 0.06, 0.12, -0.04, 0.10};

    LinearPathMoveCommand command;
    command.targetPoses.push_back(fk.EvaluateTcp(targetJoints));
    command.maxLinearVelocityMetersPerSecond = 0.05;
    command.maxAngularVelocityRadiansPerSecond = 0.2;

    const Result accepted = controller.BeginLinearPathPlanning(command);
    if (!accepted)
    {
        std::cerr << "MoveL 계획 요청 거부: " << accepted.message << '\n';
        return 2;
    }

    for (std::size_t work = 0; work < 20000 && controller.IsMotionPlanning(); ++work)
        controller.AdvanceMotionPlanning(1);

    if (controller.IsMotionPlanning())
    {
        std::cerr << "MoveL 계획 작업 한도에 도달했습니다.\n";
        return 3;
    }

    const auto planningResult = controller.TakeMotionPlanningResult();
    if (!planningResult || !*planningResult)
    {
        std::cerr << "MoveL 계획 실패\n";
        return 4;
    }

    // 이 독립 Simulation 예제에서는 4 ms tick으로 Controller 상태를 진행한다.
    for (std::size_t tick = 0; tick < 100000 &&
         controller.GetStateView().mode == RobotMode::Moving; ++tick)
        controller.Update(0.004);

    const RobotState finalState = controller.GetState();
    if (!finalState.valid || !finalState.tcpPoseValid || finalState.mode != RobotMode::Idle)
    {
        std::cerr << "동작이 정상 완료되지 않았습니다.\n";
        return 5;
    }

    std::cout << "TCP x [m] = " << finalState.tcpPose.positionMeters[0] << '\n';
    controller.Disconnect();
}
```

이 코드는 현재 API를 사용하는 독립 Simulation 예제다. 목표를 FK로 만들기 때문에 끝점은 모델상 도달 가능한 자세지만, 임의의 실제 작업 경로도 성공한다는 뜻은 아니다. 시뮬레이터에서 사용하려면 기존 앱처럼 `SetJointStateValidityChecker`에 충돌 검사를 연결하고, 각 고정 tick에서 Controller·FK·World 변환·Physics를 정해진 순서로 갱신해야 한다. 동작 요청이 수락됐는지와 목표에 도달했는지는 별개다. 마지막 `GetState()`에서 유효성, `mode`, `tcpPoseValid`를 확인한다.

예제의 마지막 `Idle` 검사는 동작이 끝났는지만 확인하고 목표 TCP 오차는 검사하지 않는다. 오차를 수치로 확인하려면 최종 `jointPositionRadians`를 `DampedLeastSquaresIk::EvaluateTcp`에 넣고, 결과를 `targetPoses`의 목표와 비교한다. 코드의 경로 검증 기준은 위치 1 mm와 방향 0.001 rad이며, 표본에 적용하는 허용치다. 요청이 수락됐거나 Controller가 `Idle`이라는 사실만으로 오차 검사까지 통과했다고 볼 수 없다.

## 확인 기준과 저장소 내 검증 위치

경로가 수락됐다는 사실만으로 끝점까지 확인되지는 않는다. 검증할 때는 시작과 완료 시점의 `RobotState`에서 `valid`, `tcpPoseValid`, `mode`를 확인한다. 최종 관절각을 `DampedLeastSquaresIk::EvaluateTcp`에 다시 넣어 요청한 TCP 끝점과 비교하고, 경로 중간 표본이 요청한 직선에서 얼마나 벗어나는지도 확인한다. 기본 경로 정책은 TCP 위치 오차 1 mm, 방향 오차 0.001 rad 이하를 기준으로 표본을 보정한다. 이는 표본 사이 모든 자세를 증명하거나 모든 환경에서 성공한다는 뜻은 아니다. 충돌을 확인하려면 실제 앱의 Jolt 검사 callback도 연결해야 한다.

관련 테스트는 `tests/robotics/RobotMotionTests.cpp`와 `tests/robotics/LinearPathPlannerTests.cpp`에 있다. 특히 `RobotMotion.IncrementalLinearPlanningDoesNotCommitBeforePathValidation`은 계획이 끝나기 전 관절 상태를 바꾸지 않는다는 계약을 확인한다. Windows Developer PowerShell에서 실행할 명령은 다음과 같다.

```powershell
cmake --preset ninja-debug
cmake --build --preset ninja-debug
ctest --test-dir build-ninja-debug --output-on-failure -R "RobotMotion|LinearPathPlanner"
```

위 명령은 테스트를 실행하는 방법을 안내한다. 이 문서를 작성하면서 빌드나 테스트를 실행하지 않았으며, 현재 통과 결과를 주장하지 않는다.

## 실패를 읽는 법

`Unreachable`은 링크 길이로 계산한 보수적 거리 상한 밖에 목표가 있다는 뜻이다. `JointLimitReached`는 해당 IK 시작각에서 관절 경계 때문에 오차를 더 줄이지 못했다는 뜻이다. `IkDidNotConverge`는 반복 한도에 도달했거나 계산이 국소적으로 정체됐다는 뜻이다. 마지막 두 오류는 다른 IK 시작 자세에서도 해가 없다는 증거가 아니다. IK가 성공해도 관절 경로 충돌 검사나 MoveL 직선 오차 검사에서 요청이 거부될 수 있다. 그때는 목표를 MoveJ로 자동 변경하지 말고 시작 자세, 경로 구간, 장애물과 TCP offset을 다시 확인한다.

<!-- HUMANIZE-SUMMARY -->
경로: light monolith
최종 판정: corrected — 원문에 있던 `## MoveL이 경로를 계획하는 순서` 제목을 복구했다.
글자 수: 원문 6,438자, 최종 본문 6,456자 (summary 제외).
변경률: 본문 4.7% (게이트 통과; 경고 30%, 중단 50%).
검증: 원문과 대조해 제목을 복구했고, 그 외 수치·코드·조건·제한은 보존했다. 과윤문이나 잔존 fidelity 위반은 없다.
<!-- /HUMANIZE-SUMMARY -->
