# 시뮬레이터가 하는 일과 아직 하지 않는 일

이 문서는 처음 세운 목표와 현재 구현을 한데 섞지 않도록 정리한 안내서다. 표의 동작은 코드에서 확인할 수 있는 범위로 적었고, 아직 확인하지 못했거나 구현되지 않은 부분은 따로 표시했다. 테스트가 저장소에 있어도 최근 실행에서 통과했다는 뜻은 아니므로, 결과가 필요하면 직접 실행해야 한다.

## 설계 목표

Simulation과 향후 Hardware backend가 장치별 통신·물리 구현을 감추고 공통 제어 계약을 제공한다. Application은 `IRobotController`와 `IGripperController`를 사용하며, Viewer는 장면 표시와 입력을 연결한다. Controller의 요청 수락, 계획 완료, 실제 목표 도달은 서로 다른 상태이므로 `GetState()`로 진행 결과를 확인한다.

```text
Application / Planner
        ↓
IRobotController / IGripperController
        ↓
Simulation or Hardware backend
        ↓
RobotState / GripperState
        ↓
Kinematics, Physics, Viewer
```

관절각은 rad, 각속도는 rad/s, 위치는 m, 시간은 s로 표현한다. 공개 `CartesianPose` quaternion 배열은 `[x,y,z,w]`이며 모델 내부 `QuaternionWxyz`는 `[w,x,y,z]`다.

## 현재 구현과 범위

| 영역 | 코드에서 확인되는 동작 | 제한 또는 미확인 범위 |
|---|---|---|
| Controller 경계 | 공통 로봇·그리퍼 인터페이스와 Simulation backend가 있다. Controller 계약은 지원하지 않는 기능의 `Unsupported` 반환을 허용한다. | Hardware backend, 장치 안전 인증, 실제 장비 통신 결과는 이 저장소에서 확인되지 않는다. |
| 관절 제어 | 모델별 관절 범위·최대 속도를 적용하고, Simulation 가속도 정책으로 목표까지 시간에 따라 이동한다. `MoveJoint` 명령은 관절 경로 검사를 받는다. 기본 가속 정책은 `accelerationScale=1.0`에서 최대 속도까지 0.20초다. | 가속 제한은 제조사 사양이 아닌 시뮬레이션 정책이다. 토크·질량·관성 동역학은 계산하지 않는다. |
| 고정 갱신 | `FixedControlLoop`가 render frame과 분리된 4 ms tick을 제공한다. Controller, kinematics, transform, physics 순서는 앱이 연결한다. | hard real-time 보장이나 별도 제어 스레드·scheduler는 없다. |
| Physics / ECS | `PhysicsWorld`는 Jolt Body와 물리 계산을 담당하고, `PhysicsSystemModule`이 Flecs 설정과 Body 수명·변환 동기화를 연결한다. | Dynamic Body는 변환이 있는 조상 아래를 지원하지 않는다. Collider 치수는 Entity scale을 자동 반영하지 않는다. |
| 로봇 모델 / FK | HCR-12A bind pivot과 관절 상태에서 base 기준 링크·ToolFrame 자세를 계산하고 고정 TCP offset을 적용할 수 있다. | TCP는 모델 계산값일 수 있으며 실제 장치 측정값이라고 볼 수 없다. |
| IK와 이동 | DLS IK, MoveJ 직접 경로 검사, MoveL Cartesian 표본 검증과 증분 계획 API가 있다. | 막힌 MoveJ 경로는 우회하지 않고 거부한다. 표본 사이는 연속 충돌 검사를 증명하지 않으며 MoveL은 실패 시 MoveJ로 대체되지 않는다. |
| Gripper | 2F-85 자유공간 master/mimic 동작과 GLB 변환, Kinematic proxy, 접촉 정지와 양쪽 손끝 접촉 시 fixed constraint grasp가 있다. raw 위치는 `closureFraction`에 선형 대응하며 기본 master 각속도 범위는 0.1–1.0 rad/s다. | raw 위치·속도 매핑과 속도 범위는 시뮬레이션 가정이다. 실제 힘·전류, 마찰 파지 안정성, 개별 손가락 적응은 계산하지 않는다. |
| 자가 충돌 | 후보 자세의 Base, HCR-12A 여섯 링크와 그리퍼 몸체를 환경 및 허용 충돌 표에 따라 검사한다. | 그리퍼 손가락·내부 너클 쌍은 검사 대상이 아니다. 동작 중 충돌 감시는 매 고정 tick 목표 자세 검사이며 전체 경로를 사전 회피하지 않는다. |
| 안전 기능 | 명령·상태의 유효성 검사와 소프트웨어 `Stop()`이 있다. | Watchdog, E-Stop state model, 영점 보정 및 하드웨어 보호 회로 통합은 구현된 것으로 간주하지 않는다. 소프트웨어 Stop은 실제 E-Stop을 대신하지 않는다. |

## 구현 단계 요약

초기 요구사항은 backend 분리, 관절 한계, 속도 제한, 고정 갱신, 물리 연동, HCR-12A FK, IK/Cartesian 이동, 2F-85 제어, Robot/Gripper collider, 접촉 파지, 안전 확장 순서로 구성됐다. 현재 코드에서 확인되는 진행 상태는 다음과 같다.

| 단계 | 현재 상태 |
|---|---|
| Backend와 공통 controller contract | Simulation 구현 있음. Hardware backend는 계획 범위다. |
| 관절 제한과 속도·가속 제어 | 구현 있음. 가속은 시뮬레이션 정책이다. |
| 고정 제어 주기 | 4 ms callback 기반 loop 구현 있음. hard real-time scheduler는 아니다. |
| Jolt·ECS 기본 연동과 로봇 충돌 proxy | 구현 있음. Entity 설정과 Body 수명·동기화는 별도 모듈이 연결한다. |
| HCR-12A 모델과 FK | 링크 및 ToolFrame 계산 구현 있음. |
| IK, MoveJ, MoveL | 구현 있음. 경로 표본 수와 충돌 간격은 정책으로 제한된다. MoveJ는 직접 경로만 검사한다. |
| 2F-85 자유공간 개폐와 접촉 grasp | 구현 있음. 힘 기반 파지와 손가락 적응은 계획 또는 검토 범위다. |
| Robot self-collision | 후보 자세 검사 구현 있음. Gripper 내부 손가락 쌍은 제외된다. |
| Watchdog, E-Stop 상태, 영점 보정 | 이 저장소에서 구현된 것으로 확인되지 않았다. |

## Physics와 좌표 계약

```text
Flecs Entity: RigidBody + Collider 설정
        ↓
PhysicsSystemModule: 설정 해석, Body 수명, 고정 tick 변환 동기화
        ↓
PhysicsWorld: PhysicsBodyHandle을 통한 Jolt 연동
```

Entity Local 변환은 부모 기준이고 World 변환은 조상 변환을 합성한 장면 기준이다. `PhysicsSystemModule`은 Scene World 자세를 Kinematic Body에 전달하며 Dynamic Body의 계산 결과를 Entity Local 값에 기록한다. 이 때문에 Dynamic Body 자신과 조상은 항등 변환이어야 한다. Dynamic 부모 아래의 물리 자식도 지원하지 않는다. Static Environment의 시각용 scale은 예외이며, Collider 반 크기와 offset은 m 단위로 별도 지정한다.

Static Body는 Scene이 정한 자세를 유지한다. Kinematic Body는 Scene이 제공한 목표 자세로 Jolt에 이동 입력을 보내며, 장애물 앞에서 자동으로 멈추는 경로 계획기는 아니다. Dynamic Body는 Jolt가 계산한 중력·충돌 결과를 반영한다. Robot과 Gripper proxy는 Kinematic으로 움직이고 관절 토크나 연결된 articulated dynamics를 풀지 않는다.

## 검증 근거와 한계

관련 회귀 테스트와 벤치마크 정의는 저장소의 `tests/` 및 `modules/robotics`에 있다. 테스트 정의는 검사 의도를 보여 주지만, 실행 결과나 전체 임무 성공률을 증명하지 않는다. 특히 Planner 벤치마크의 dummy validity callback 결과는 실제 환경 충돌 검증으로 해석하지 않는다. Jolt 단일 proxy 벤치마크도 전체 로봇 collider 조립 및 Viewer 통합 성능을 나타내지 않는다.

Release 벤치마크 숫자를 공유할 때는 실행 날짜, 빌드 설정, seed와 case 수, callback이 실제 충돌 검사인지 dummy인지 함께 기록한다. 측정값은 해당 실행의 표본 결과이며 모든 목표에서 성공한다는 보장은 아니다.

## 역할별 상세 참조

이 문서는 요구사항과 구현 여부의 상태 요약이다. 호출 전제와 오류 확인은 [Controller Interface](CONTROLLER_INTERFACE.md), 모듈 소유권과 fixed tick 순서는 [Architecture](ARCHITECTURE.md) 및 [Physics / Flecs Integration](PHYSICS_ECS_INTEGRATION.md), 이동 동작은 [Robot Motion and Grasp](ROBOT_MOTION_AND_GRASP.md), benchmark counter와 측정 한계는 [Diagnostics](DIAGNOSTICS.md)에서 확인한다.

## 다음 검토 범위

- 그리퍼 손가락의 self-collision과 파지 형상별 회귀 범위를 결정한다.
- 연속 경로 전체의 충돌 보장 수준과 필요한 표본 간격을 평가한다.
- 별도 관절 trajectory 명령·실행 계약이 필요한지 검토한다. 현재 `MoveJoint`와 관절 경로 계획이 그 기능 전체를 대체한다고 가정하지 않는다.
- 접촉력·마찰 또는 관절 동역학 모델이 필요한 사용 사례를 확인한 뒤 모델 범위를 정한다.
- Hardware backend를 추가할 때 연결·오류·정지 계약과 실제 장비별 안전 요구를 별도로 정의한다.
- 계획 계산량과 실제 Viewer의 충돌 검사 비용을 함께 측정할 때는 dummy callback 기반 planner benchmark와 실제 Jolt 통합 측정을 구분한다.
