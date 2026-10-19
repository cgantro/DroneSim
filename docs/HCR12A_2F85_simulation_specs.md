# HCR-12A와 Robotiq 2F-85 시뮬레이션 사양

이 문서는 시뮬레이터가 쓰는 HCR-12A와 2F-85 수치를 한곳에 모은다. 제조사 자료에서 온 값, 공개 기구학 모델을 참고한 값, 현재 GLB에 맞춰 계산한 값을 구별해 적었다. 이 숫자들이 모두 제조사 공식 사양이라고 받아들이지 않도록 출처도 함께 설명한다.

현재 runtime asset은 `assets/HCR12A_2F-85.glb`이며, 문서의 사양 표는 2026-10-05 기준이다.

## 1. 먼저 구분해야 하는 데이터 종류

이 문서의 숫자는 모두 제조사 공식값이 아니다.

| 분류 | 의미 |
|---|---|
| `MANUFACTURER` | 제조사 매뉴얼/제품/프로토콜이 정의한 값 |
| `CAD-DERIVED` | STEP/CAD의 실제 체결 feature에서 구한 값 |
| `KINEMATIC-REFERENCE` | 공개 URDF/Xacro 등 시뮬레이션 기구학 모델 기준 |
| `ASSET-DERIVED` | 현재 controller-ready GLB 좌표계에 맞춰 변환한 값 |

출처 유형과 추적 한계는 [Model Data Provenance](MODEL_DATA_PROVENANCE.md), GLB hierarchy/정규화 계약은 [HCR-12A + 2F-85 GLB Normalization](HCR12A_GLB_NORMALIZATION.md)을 참고한다.

---

## 2. 한화 HCR-12A 기본 규격 — 제조사 자료

| 항목 | 값 |
|---|---:|
| 자유도 | 6 DOF |
| 정격 payload | 12 kg |
| Reach / work radius | 1300 mm |
| 관절 형태 | J1~J6 revolute |

현재 runtime에서 사용하는 관절 limit/속도:

| Joint | Limit [deg] | Limit [rad] | Max speed [deg/s] | Max speed [rad/s] |
|---|---:|---:|---:|---:|
| J1 | -180 ~ 180 | -3.141593 ~ 3.141593 | 130 | 2.268928 |
| J2 | -165 ~ 135 | -2.879793 ~ 2.356194 | 130 | 2.268928 |
| J3 | -85 ~ 245 | -1.483530 ~ 4.276057 | 200 | 3.490659 |
| J4 | -190 ~ 190 | -3.316126 ~ 3.316126 | 200 | 3.490659 |
| J5 | -170 ~ 170 | -2.967060 ~ 2.967060 | 200 | 3.490659 |
| J6 | -360 ~ 360 | -6.283185 ~ 6.283185 | 200 | 3.490659 |

현재 `Hcr12a.h`는 degree/degree-per-second 값을 SI runtime 단위인 rad/rad/s로 저장한다.

> HCR revision/문서에 따라 J2/J3 범위가 다르게 표기된 자료가 있으므로 실제 장비와 1:1 대응할 때는 해당 controller/manual revision을 다시 확인한다.

---

## 3. HCR 기계 pivot — CAD 기반

현재 J1~J6 pivot은 mesh 중심이나 임의 추정점이 아니다.
STEP assembly에서 서로 맞물리는 원통형 체결 feature의 중심이 양쪽 부품에서 일치하는지 확인해 결정했다.

| Joint | Bind pivot [m] | Physical/runtime axis |
|---|---|---|
| J1 | `(0.0000, 0.1985, 0.00000)` | `+Y (0,1,0)` |
| J2 | `(0.1535, 0.3100, 0.10000)` | `+X (1,0,0)` |
| J3 | `(0.1095, 0.9100, 0.10000)` | `+X (1,0,0)` |
| J4 | `(0.0000, 1.0150, 0.23350)` | `+Z (0,0,1)` |
| J5 | `(-0.1385, 1.0150, 0.69100)` | `+X (1,0,0)` |
| J6 | `(0.0000, 1.0150, 0.85475)` | `+Z (0,0,1)` |

ToolFrame bind world position:

```text
(0.0000, 1.0150, 0.9145) m
```

원본 CAD의 mm 좌표를 runtime에서는 meter로 정규화했다.

---

## 4. HCR 제어용 GLB 규칙 — 자산 기반

현재 `assets/HCR12A_2F-85.glb`는 다음 contract를 사용한다.

```text
J1~J6 moving node bind rotation = identity
J1 axis = +Y
J2 axis = +X
J3 axis = +X
J4 axis = +Z
J5 axis = +X
J6 axis = +Z
```

이전 중간 모델에서 사용했던 ±90° bind quaternion 보정은 현재 contract가 아니다.
Arm visual transform은 mesh vertex/normal에 bake해 bind pose 외형을 유지한다.

`RobotKinematics`는 bind pivot 사이 차이를 이전 누적 회전으로 변환하고, 모델 axis와 RobotState로 pose를 계산한다. 화면과 충돌 프록시는 같은 FK 결과를 사용한다. Controller는 ToolFrame에 고정 공구 변환을 더해 Robot base 기준 TCP를 계산하며 ToolFrame이 있으면 `tcpPoseValid=true`다. 이는 모델 기반 시뮬레이션 상태이고 실제 장치 측정값이 아니다.

```text
RobotState q [rad]
→ RobotKinematics
→ joint local rotation / base-frame link pose
→ RobotTransformAdapter
→ Joint local rotation
```

Pivot translation은 GLB hierarchy에 이미 존재하므로 매 frame 코드에서 다시 더하지 않는다.

---

## 5. Robotiq 2F-85 제조사 규격

대표 장비 규격:

| 항목 | 값 |
|---|---:|
| Maximum opening | 85 mm |
| Grasp force | 20~235 N |
| Finger speed | 20~150 mm/s |
| Nominal payload | 5 kg |
| Mass | 약 0.925 kg |
| Position resolution | 약 0.4 mm |
| Supply | 24 V DC 계열 |

2F-85는 rotational adaptive / under-actuated gripper다.
손가락은 단순 prismatic slider처럼 좌우로 평행 이동하지 않고 knuckle/linkage 회전으로 열리고 닫힌다.

### 장치 명령과 상태

| 값 | 의미 | 범위 |
|---|---|---:|
| rPR | Position Request | `0=open`, `255=closed` |
| rSP | Speed Request | `0..255` |
| rFR | Force Request | `0..255` |
| gPO | Actual/encoder position 계열 값 | `0..255` |
| gOBJ | Object/motion status | device status code |

중요:

```text
rPR = raw command
rPR != mm
rPR != rad
```

0~255와 실제 opening은 제조사 문서에서 quasi-linear 관계로 다뤄지므로, 정밀 실기기 일치를 원하면 추후 LUT/측정 보정이 필요하다.

---

## 6. 2F-85 자유 공간 기구학 참고 모델

현재 프로젝트는 공개 2F-85 kinematic model의 master/mimic 개념을 자유 공간 자세 기준으로 사용한다.

```text
nominal master closed q = 0.7929 rad ≈ 45.43°
```

이 값은 **Robotiq 내부 motor shaft angle이 아니다.**
시뮬레이션 linkage의 master joint reference다.

Free-space mimic 관계:

| Joint | Master multiplier | nominal range [rad] |
|---|---:|---:|
| LeftOuterKnuckleJoint | +1 | 0 ~ +0.8 |
| RightOuterKnuckleJoint | -1 | -0.8 ~ 0 |
| LeftInnerKnuckleJoint | +1 | 0 ~ +0.8 |
| RightInnerKnuckleJoint | -1 | -0.8 ~ 0 |
| LeftFingerTipJoint | -1 | -0.8 ~ 0 |
| RightFingerTipJoint | +1 | 0 ~ +0.8 |

고정 mimic은 **물체와 접촉하기 전 자유공간 opening/closing**을 표현한다.
접촉 후 passive adaptive motion까지 이 표만으로 재현하지 않는다.

---

## 7. 현재 GLB의 2F-85 pivot과 회전축 — 자산 기반

현재 `assets/HCR12A_2F-85.glb`에 맞춘 Gripper-local frame 값:

```text
joint axis    = (0, 0, -1)
joint plane Z = 0.0934257339 m
```

| Joint | Gripper-local pivot [m] |
|---|---|
| LeftOuterKnuckleJoint | `(-0.03060114, 0.05490452, 0.0934257339)` |
| RightOuterKnuckleJoint | `(+0.03060114, 0.05490452, 0.0934257339)` |
| LeftInnerKnuckleJoint | `(-0.01270000, 0.06142000, 0.0934257339)` |
| RightInnerKnuckleJoint | `(+0.01270000, 0.06142000, 0.0934257339)` |
| LeftFingerTipJoint | `(-0.06775864, 0.09832620, 0.0934257339)` |
| RightFingerTipJoint | `(+0.06775864, 0.09832620, 0.0934257339)` |

이 값은 공개 kinematic frame을 현재 GLB geometry에 대응한 **프로젝트 asset 좌표**다.
다른 2F-85 CAD/GLB에 그대로 사용할 수 있는 제조사 universal coordinate가 아니다.

---

## 8. 현재 그리퍼 계층 구조

과거의 `LeftFingerJoint / RightFingerJoint` 두-node 모델은 폐기됐다.
현재 구조:

```text
Gripper
├─ GripperMesh
├─ LeftOuterKnuckleJoint
│  └─ LeftOuterKnuckle
│     ├─ LeftOuterKnuckleMesh
│     └─ LeftFinger
│        ├─ LeftFingerMesh
│        └─ LeftFingerTipJoint
│           └─ LeftFingerTip
│              └─ LeftFingerTipMesh
├─ LeftInnerKnuckleJoint
│  └─ LeftInnerKnuckle
│     └─ LeftInnerKnuckleMesh
├─ RightOuterKnuckleJoint
│  └─ RightOuterKnuckle
│     ├─ RightOuterKnuckleMesh
│     └─ RightFinger
│        ├─ RightFingerMesh
│        └─ RightFingerTipJoint
│           └─ RightFingerTip
│              └─ RightFingerTipMesh
└─ RightInnerKnuckleJoint
   └─ RightInnerKnuckle
      └─ RightInnerKnuckleMesh
```

Moving gripper joints의 bind rotation도 identity다.
`Gripper` root 자체는 비항등 장착 `matrix`를 갖는다. 런타임은 이 장착 변환과 중첩 Tip 계층을 보존한다.

---

## 9. 그리퍼 형상과 동작 검증

현재 linkage geometry 기준 자유 공간 검증:

```text
OPEN gap   ≈ 85.00 mm
CLOSED gap ≈ 0.97 mm
```

원본 finger geometry의 triangle 수는 rigid component 분할 후에도 보존됐다.

```text
Left  12,952 -> 12,952
Right 12,952 -> 12,952
```

현재 runtime GLB에는 baked open/close animation이 없다.
GLB는 geometry/hierarchy/pivot만 제공하고 동작은 현재 C++ `SimGripperController`와 `GripperKinematics`가 만든다.

---

## 10. 현재 코드 상태

### 구현됨

```text
IRobotController
IGripperController contract
RobotSpecification / GripperSpecification
Hcr12a model specification
TwoF85 model specification
SimRobotController : IRobotController
SimGripperController : IGripperController (자유공간 개폐와 접촉 피드백)
GripperGraspAdapter (접촉 정지와 양쪽 손끝의 fixed constraint 파지)
RobotTransformAdapter
RobotKinematics (FK / ToolFrame calculation)
GripperKinematics (master/mimic Local 회전)
GripperTransformAdapter (GLB bind 회전 보존)
FixedControlLoop (250 Hz callback loop)
PhysicsWorld + Flecs PhysicsSystemModule
J1~J6 Kinematic collision proxies
Gripper Kinematic collision proxies 7개
GUI 그리퍼 요청 / 상태 표시
Joint angle limit 검증
Joint max velocity 기반 target 추종
관절 공간과 Cartesian 경로의 Simulation 가속도 제한
software Stop
```

### 아직 미구현/후순위

```text
Hardware Robot backend
Hardware Gripper backend
실제 힘·전류 계산
접촉 뒤 개별 손가락 under-actuated adaptive motion
Watchdog / E-Stop state / Zero Offset
```

현재 그리퍼는 자유롭게 열고 닫을 수 있고, 물리 접촉에서 닫힘을 멈춘다. 양쪽 손끝이 같은 물체에 닿으면 constraint로 연결한다. 이 기능은 시뮬레이션에서 상대 자세를 유지하는 연결이며 실제 힘이나 마찰을 계산하지 않는다. 실제 힘·전류 feedback, 접촉 뒤 손가락별 적응과 Hardware backend는 구현하지 않았다. DLS IK와 TCP 이동은 [프로젝트 안내](ARCHITECTURE.md)를 참고한다.

---

## 11. 현재 그리퍼 런타임 흐름

자유 공간:

```text
rPR 0..255
    ↓
SimGripperController
    ↓
GripperState.closureFraction [0,1]
    ↓ GripperKinematics
master q = fraction * nominal closed angle
    ↓
masterMultiplier
    ↓
6 linkage Joint local delta rotation
    ↓ GripperTransformAdapter: bind * delta
GLB 관절 Local 회전 → World 변환 → 기존 7개 Kinematic proxy
```

raw 위치에서 fraction으로의 선형 매핑은 자유공간 시뮬레이션 가정이며 실제 장치 보정식이 아니다. 연속 fraction을 관절 계산에 사용하고 raw actual position은 표시용 반올림만 한다. 기본 master 각속도 0.1..1.0 rad/s와 raw speed의 선형 대응도 프로젝트 설정이며 제조사 finger speed 사양을 환산한 값이 아니다. raw speed 0도 최소 양의 속도로 움직인다. 실제 장비 정밀 대응이 필요하면 LUT 또는 측정 기반 calibration을 검토한다.

4 ms마다 두 Controller를 갱신하고 팔·그리퍼 자세와 World 변환을 반영한 뒤 Jolt를 실행한다. 그 후 World 변환을 다시 계산한다. Stop은 현재 위치와 요청 내용을 보존하고 `Stopped`로 바꾼다. Reset은 현재 위치를 유지하면서 제어를 비활성화한다. GUI는 요청을 보내고 상태를 표시한다.

물리 모터와 힘·마찰 계산으로 개별 손가락을 적응시키는 후속 흐름이다. 현재는 양쪽 접촉 후 고정 constraint로 물체를 유지한다.

```text
rPR / rFR
    ↓
Physics motor / constraints
    ↓
Collision / contact
    ↓
under-actuated passive adaptation
    ↓
object grasp
```

---

## 12. 코드에서 기준이 되는 값

모델 규격 코드:

```text
modules/robotics/include/robotics/models/hanwha/Hcr12a.h
modules/robotics/include/robotics/models/robotiq/TwoF85.h
```

수치 provenance:

```text
docs/MODEL_DATA_PROVENANCE.md
```

GLB 구조:

```text
docs/HCR12A_GLB_NORMALIZATION.md
```

Controller architecture:

```text
docs/ARCHITECTURE.md
```

## 13. 현재 물리 기능 구현 상태

HCR-12A GLB의 link geometry에서 만든 Convex Hull은 별도 Kinematic proxy Entity에 연결한다. Simulation adapter는 이 Entity들의 Local 자세를 World 자세로 바꾸어 Jolt에 전달하며, Robot Base에는 고정 Environment collider를, Link1~Link6에는 Kinematic collider를 둔다. Gripper collider는 별도 구성기에서 만든다.

`ConfigureTwoF85Colliders`는 현재 asset의 메시 정점을 축약해 일곱 Kinematic proxy를 구성한다. 고정 base, outer knuckle와 finger의 compound, inner knuckle, fingertip proxy는 authored `Gripper` 또는 여섯 joint Entity의 자식이므로 fixed-step ECS 계층의 World 변환을 따른다.

각 hull 정점은 소유 body 또는 joint 원점 기준이다. 초기 자유 공간 손끝 간격 약 85 mm를 보존한다.

개폐 자세는 `GripperState.closureFraction`에서 `GripperKinematics`가 계산하고 `GripperTransformAdapter`가 authored hierarchy에 적용한다. 같은 Dynamic 물체의 서로 반대 면에 양쪽 손끝이 닿으면 grasp adapter가 물체와 그리퍼 사이에 fixed constraint를 만든다.

이 동작은 접촉력이나 개별 손가락 적응을 계산하지 않는다.

Convex Hull은 원본 메시 형상의 근사이므로 얇거나 작은 부분이 충돌 형상에서 빠질 수 있다. 이 proxy는 관절 축 제약이나 모터 토크를 계산하지 않는다.

2F-85의 `IGripperController` contract, model specification, 자유공간 Simulation controller·기구학·GLB adapter, 메시 기반 collider proxy와 접촉 파지 adapter가 현재 구성되어 있다. force 요청은 raw 범위만 검사하며 전류와 실제 힘은 계산하지 않는다. `0.7929 rad`는 자유공간 linkage의 nominal master angle이며 실제 motor shaft angle이나 접촉 후 손가락 자세를 뜻하지 않는다.

물리 계산 순서와 좌표 기준은 [프로젝트 안내](ARCHITECTURE.md)에 간단히 정리했다.
