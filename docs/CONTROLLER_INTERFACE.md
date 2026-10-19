# 로봇 제어 인터페이스

시뮬레이터와 실제 장비가 같은 상위 제어 코드를 사용할 수 있도록, GraspLink는 로봇·그리퍼 제어를 공통 인터페이스 뒤에 둔다. 지금 저장소에는 Simulation backend가 있고, Hardware backend는 아직 구현하지 않았다.

```text
Application / Planner
        |
        v
IRobotController / IGripperController
        |
        +-- backends/simulation
        |
        +-- backends/hardware (future)
```

상위 계층은 Flecs Entity나 GLB Node를 직접 수정하지 않는다. 제조사별 packet과 register도 backend 안에서 처리하도록 경계를 둔다.

## 디렉터리

```text
modules/robotics/
├─ include/robotics/
│  ├─ core/
│  │  ├─ ControlTypes.h
│  │  ├─ IRobotController.h
│  │  └─ IGripperController.h
│  ├─ models/
│  │  ├─ ModelTypes.h
│  │  ├─ RobotSpecification.h
│  │  ├─ GripperSpecification.h
│  │  ├─ hanwha/Hcr12a.h
│  │  └─ robotiq/TwoF85.h
│  ├─ backends/
│  │  └─ simulation/SimRobotController.h, SimGripperController.h
│  ├─ kinematics/RobotKinematics.h, RobotInverseKinematics.h, GripperKinematics.h
│  └─ planning/                    # 경로 계획 계약과 정책
└─ src/                             # backend, kinematics, planning 구현
```

namespace root는 `grasplink::robotics`다.

## 로봇 모델 정보

`models/`는 장치에 고정된 상수의 source of truth다.

HCR-12A:
- J1~J6 name
- bind pivot
- local axis
- min/max angle
- max velocity
- ToolFrame bind 위치

Robotiq 2F-85:
- 0~255 position/speed/force request range
- master closed angle
- outer/inner/tip pivot
- linkage axis
- mimic multiplier

새 로봇은 `models/<manufacturer>/<Model>.h`에 `RobotSpecification`을 추가한다.
Simulation/Hardware backend는 이 specification을 주입받는다.

## Controller가 보장하는 동작

`IRobotController`:

```text
Connect
Disconnect
IsConnected
MoveJoint
MoveLinear
MovePose
BeginPosePlanning
BeginPosePlanningWithLinearContinuation
MoveLinearPath
BeginLinearPathPlanning
AdvanceMotionPlanning
IsMotionPlanning
TakeMotionPlanningResult
Stop
GetState
Update
```

`Begin*Planning`은 계획 시작을 요청한다. 증분 계획을 지원하는 Simulation은 `AdvanceMotionPlanning(workBudget)`로
작업 단위를 진행하고, `IsMotionPlanning()`과 `TakeMotionPlanningResult()`로 진행 중 여부와 완료 결과를 읽는다.
기본 인터페이스 구현은 해당 동기 명령으로 되돌아가거나 `Unsupported`를 반환할 수 있으므로 호출자는 구체 backend의
기능 지원을 고려해야 한다. 비동기 계획의 완료는 로봇 동작 완료와 구분되며, 동작 목표 도달은 `GetState()`로 확인한다.

`IGripperController`:

```text
Connect
Disconnect
IsConnected
Activate
Reset
Command
Stop
GetState
Update
```

공통 단위:

```text
Joint angle     rad
Joint velocity  rad/s
Cartesian pos   m
Quaternion      xyzw
Time            s
```

위 quaternion 순서는 `CartesianPose`의 배열 기준이다. 모델/FK의 `QuaternionWxyz`는 wxyz 순서이며, 어댑터가 타입에 맞춰 명시적으로 변환한다.

## 호출 전제와 상태 확인

`Result`가 성공이면 요청을 받았다는 뜻이다. 관절 또는 TCP 목표 도달은 `GetState()`에서 `valid`, `mode`, `errorCode`를 확인해 판단한다. 비동기 계획의 완료는 계획 결과와 동작 완료를 구분한다.

| 호출 | Simulation의 주요 전제 | 실패 또는 완료 확인 |
|---|---|---|
| `MoveJoint` | 연결됨; 목표 관절 수가 모델과 같고 각도가 한계 안에 있음; `velocityScale`, `accelerationScale`이 유한한 `(0,1]` 값 | 연결되지 않으면 `NotConnected`, 잘못된 입력이면 `InvalidCommand`, 계획 중이면 `Busy`; 성공 뒤 `GetState().mode`가 `Idle`인지 확인 |
| `MovePose` / `BeginPosePlanning` | 연결됨; ToolFrame이 있는 모델; 목표는 Robot base 기준 | 도구 frame이 없으면 `Unsupported`; IK·경로 실패는 `errorCode`/`Result.code`; 계획 완료 후에도 이동 종료를 `GetState()`로 확인 |
| `MoveLinearPath` / `BeginLinearPathPlanning` | 연결됨; 비어 있지 않은 waypoint; 각 속도·가속도 제한이 유한한 양수; ToolFrame이 있는 모델 | 비동기 시작 뒤 `IsMotionPlanning()`이 false가 될 때 `TakeMotionPlanningResult()`를 읽고, 성공 결과와 이후 `GetState()`의 동작 상태를 각각 확인 |
| `IGripperController::Command` (Simulation) | 연결과 활성화가 완료됨; 위치·속도·힘 code가 각 specification 범위 안에 있음 | 연결 없음은 `NotConnected`, 미활성화는 `Busy`, 범위 밖 code는 `InvalidCommand`; 도달·접촉은 `GetState().objectStatus`로 구별 |

Simulation이 사용하는 주요 `ErrorCode` 분류는 다음과 같다. 이 표는 공통 분류를 찾기 위한 것이며, 하드웨어 backend의 장치별 원본 오류 code를 해석하지 않는다.

| 분류 | 확인할 상황 |
|---|---|
| `NotConnected`, `Busy`, `InvalidCommand`, `Unsupported` | 연결, 다른 계획 진행 여부, 요청값, backend 기능 지원 확인 |
| `Unreachable`, `JointLimitReached`, `IkDidNotConverge` | 각각 보수적 도달 거리 한계, 해당 IK seed에서 관절 한계에 의한 정체, 반복 한도 또는 국소 정체. 후자의 두 결과는 다른 seed에서도 해가 없음을 증명하지 않는다. |
| `EnvironmentContact`, `SelfCollision`, `AttachedObjectCollision` | 충돌 검사에서 거부된 환경·자가 충돌·부착 물체 상태. `EnvironmentContact`는 Viewer의 현재 tick 복원도 나타낼 수 있다. |
| `Cancelled`, `Fault`, `TransportError` | 취소된 소프트웨어 작업, 구현 오류, 통신 오류. 공통 `Fault`만으로 장비 보호 정지를 추정하지 않는다. |

`RobotMode::Planning`은 계획 작업 중 관절 자세를 유지하는 상태이고 `Moving`은 실행 중이다. `Stopped`는 소프트웨어 정지를 나타낸다. Gripper에서는 `mode`가 연결·활성화·실행 상태를, `objectStatus`가 이동·접촉·요청 위치 도달을 나타낸다. `Stopped`인데 `objectStatus == AtRequestedPosition`이 아닐 수 있으므로 두 필드를 함께 읽는다.

## 시뮬레이션 backend

`SimRobotController`는 `RobotSpecification`을 생성자에서 받고 모델의 joint count/limit/max velocity를 그대로 사용한다.
Controller는 관절 목표를 검증하고, 관절 속도·Simulation 가속 제한과 TCP 목표 IK, 표본 기반 직선 경로 실행, Stop, 모델 기반 상태 제공을 담당한다. `MoveJoint` 가속도는 모델 사양에 없는 제조사 한계를 가정하지 않도록 최대 관절 속도에서 유도한 Simulation 정책으로 제한한다. 기본 `accelerationScale=1.0`은 최대 속도까지 0.20초 동안 가속하는 설정이다.

`MovePose`는 현재 관절각을 seed로 DLS IK를 풀고 관절 공간의 직접 경로를 검사하는 MoveJ 명령이다. `BeginPosePlanning`은 같은 목표를 프레임별 작업으로 나누어 계산한다. 경로가 충돌하면 우회하지 않고 실패한다. 이 경로는 TCP 직선을 보장하지 않는다. `BeginPosePlanningWithLinearContinuation`은 접근 자세와 그 뒤의 선형 continuation을 함께 검사하고, 둘 다 유효할 때만 접근 동작을 시작한다. 접근 동작이 끝난 뒤 호출자가 continuation을 요청해야 한다.

`MoveLinear`은 목표 하나를 `MoveLinearPath`로 감싼다. MoveL은 직전 표본의 IK 해를 다음 seed로 재사용하고, 이어갈 해가 없을 때만 대체 분기를 찾는다.

경로 표본과 관절 경로의 검사에 실패하면 MoveL을 거부하며 MoveJ로 자동 대체하지 않는다. 고정 갱신에서는 사전 검증한 관절 표본을 보간한다.

TCP의 직선·회전 속도와 가속도 제한은 전체 경로 프로파일에 적용한다. 표본 추종이 속도 제한을 넘는 특이 자세에서만 TCP를 거의 고정하는 재정렬 IK를 시도하며, 이 이동도 관절 경로와 TCP 오차를 검사한다.

목표는 Robot base 좌표 기준이다. 실패와 waypoint 처리 계약은 [로봇 이동과 파지](ROBOT_MOTION_AND_GRASP.md)를 참고한다.

관절 최대 속도는 모델 데이터에서 가져오지만 제조사가 가속도를 제공하지 않는 경우 Simulation 정책으로 제한한다. 문서의 0.20초 기본 가속 시간은 이 정책값이며 장비 사양이나 측정 결과가 아니다.

`SimGripperController`는 `GripperSpecification`을 빌리고 `SimGripperMotionSettings`를 값으로 보관한다. 연결 시 열린 위치로 초기화하며 활성화 뒤 유효한 raw 위치·속도·힘 요청을 수락한다. `GripperState.closureFraction`은 0=열림, 1=nominal closed인 연속 위치다. `valid`와 `closureFractionValid`를 함께 확인하며 `actualPosition` raw feedback은 표시용으로 반올림한다. `GripperMode`는 연결·활성화·이동·정지를 구분하고 `objectStatus`는 이동/목표 도달 분류를 제공한다.

위치는 raw 범위에서 fraction으로 선형 대응한다. raw speed는 기본 master 각속도 0.1..1.0 rad/s에 선형 대응하며 raw 0도 가장 느린 양의 속도다. 이 매핑과 속도 범위는 시뮬레이션 가정이며 제조사 보정식·속도 사양이 아니다. 유한한 양의 dt에서 목표를 추종하고 정확히 멈춘다. Stop은 현재 위치와 요청 echo를 유지한 채 `Stopped`로 전환하며, Reset은 현재 위치를 유지하고 비활성화한다. 중간 위치 Stop을 목표 도달로 보고하지 않는다.

`forceRequest`는 범위만 검사하고 전류와 실제 힘은 계산하지 않으므로 `currentValid=false`다. 물리 접촉은 `GripperGraspAdapter`가 전달하며 Controller는 접촉 시 현재 위치에서 멈추고 `ContactWhileOpening` 또는 `ContactWhileClosing`을 보고한다. 양쪽 손끝이 같은 Dynamic 물체를 서로 반대 방향에서 만지면 adapter가 고정 constraint로 물체를 유지한다. 실제 마찰 파지력과 개별 손가락 적응은 계산하지 않는다. 세부 상태 전이는 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)를 참고한다.

## 시뮬레이션 adapter

```text
modules/simulation/include/simulation/robotics/RobotTransformAdapter.h
modules/simulation/src/robotics/RobotTransformAdapter.cpp
modules/simulation/include/simulation/robotics/GripperTransformAdapter.h
modules/simulation/src/robotics/GripperTransformAdapter.cpp
```

`RobotKinematics`가 `RobotState`를 pose로 바꾸고, `RobotTransformAdapter`는 그 결과를 Flecs/GLB transform으로 표현한다. 두 adapter의 구현은 `modules/simulation`에 있다. Viewer는 FK나 제어 로직을 수행하지 않는다. 그리퍼의 기구학은 robotics 모듈이 계산하고, simulation adapter가 GLB 관절을 갱신한다. 관절 자식인 그리퍼 충돌 프록시는 계층 변환을 따라가므로 별도 pose adapter가 없다.

`GripperKinematics`는 분기형 여섯 관절의 master/mimic Local 회전 변화만 계산한다. `GripperTransformAdapter`는 저장한 bind 회전에 변화량을 오른쪽으로 곱하고 원본 Local 위치·크기·장착 변환을 보존한다.

고정 tick에서는 파지 해제와 안전 자세 저장 뒤 두 Controller를 갱신하고, 팔·그리퍼 자세를 적용한다. World 변환을 갱신한 다음 collision guard가 겹침을 검사하고 필요하면 안전 관절 자세를 복원한다.

이어서 Jolt step과 접촉 피드백·파지 연결을 수행하고 World 변환을 다시 갱신한다. 세부 호출 순서는 [Physics / Flecs Integration](PHYSICS_ECS_INTEGRATION.md)을 참고한다.

기존 일곱 그리퍼 proxy는 관절 자식이므로 같은 World 변환을 따른다. GUI는 Controller에 요청을 보내고 상태 복사본과 실제 접촉·파지 상태를 표시한다.

## 아직 구현하지 않은 Hardware backend

```text
robotics/backends/hardware/hanwha/
robotics/backends/hardware/robotiq/
```

Hanwha backend는 HCR TCP/IP/Modbus TCP/Rodi adapter를 내부에 숨긴다.
Robotiq backend는 Modbus RTU RS-485 register packing/parsing을 내부에 숨긴다.

## 더 읽기와 코드 기준

이 문서는 공통 Controller 계약과 현재 시뮬레이션 backend의 차이를 요약한다. 이동과 waypoint 의미는 [로봇 이동과 파지](ROBOT_MOTION_AND_GRASP.md), TCP 호출 예제는 [IK·MoveL 튜토리얼](IK_MOVEL_TUTORIAL.md), 고정 tick과 물리 연결 순서는 [Physics와 Flecs 연결](PHYSICS_ECS_INTEGRATION.md)을 참고한다.

공통 계약의 기준 코드는 `modules/robotics/include/robotics/core/IRobotController.h`와 `IGripperController.h`다. 시뮬레이션 동작은 `modules/robotics/src/backends/simulation/SimRobotController.cpp`와 `SimGripperController.cpp`에서 확인할 수 있다.
