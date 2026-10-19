# HCR-12A와 Robotiq 2F-85 GLB 정규화

이 문서는 `assets/HCR12A_2F-85.glb`의 좌표와 hierarchy를 설명한다. 팔과 그리퍼 관절을 코드에서 구동할 수 있도록 어떤 기준을 적용했는지, asset을 바꿀 때 무엇을 다시 확인해야 하는지 기록해 두었다. 내용은 2026-10-04 기준이다.

## 1. 현재 자산의 규칙

현재 runtime GLB는 다음 원칙을 사용한다.

1. HCR J1~J6 moving node의 **bind rotation은 identity**다.
2. 관절 회전축은 Joint node에 미리 quaternion 보정을 넣는 대신 모델 specification의 local axis로 정의한다.
3. Arm visual geometry는 기존 bind pose가 바뀌지 않도록 vertex에 transform을 bake한다.
4. Gripper는 단순 좌/우 translation finger가 아니라 outer/inner knuckle와 fingertip joint로 분리한다.
5. Runtime GLB에는 **그리퍼 open/close baked animation을 넣지 않는다.** 움직임은 C++ controller와 기구학 계산이 만든다.
6. 물체 접촉 뒤 각 손가락이 독립적으로 적응하는 under-actuated motion은 아직 구현하지 않았다. 현재는 양쪽 손끝 접촉 조건을 만족하면 고정 constraint로 물체와 그리퍼를 연결한다.

과거 문서의 `BindTransform * JointRotation` 전제와 `LeftFingerJoint/RightFingerJoint` 두 개만 사용하는 구조는 현재 자산 기준으로 폐기됐다.

---

## 2. HCR-12A 계층 구조

Arm의 논리 hierarchy는 다음과 같다.

```text
RobotRoot
└─ Base
   ├─ BaseMesh
   └─ J1
      └─ Link1
         ├─ Link1Mesh
         └─ J2
            └─ Link2
               ├─ Link2Mesh
               └─ J3
                  └─ Link3
                     ├─ Link3Mesh
                     └─ J4
                        └─ Link4
                           ├─ Link4Mesh
                           └─ J5
                              └─ Link5
                                 ├─ Link5Mesh
                                 └─ J6
                                    └─ Link6
                                       ├─ Link6Mesh
                                       └─ ToolFrame
                                          └─ Gripper
```

CAD 내부의 Housing/Cover/Bolt 같은 상세 assembly hierarchy는 런타임 로봇 제어 구조에서 제거하거나 visual mesh에 bake했다.

---

## 3. HCR J1~J6 회전 중심과 축

Pivot은 mesh 중심이 아니다. STEP assembly에서 서로 맞물리는 원통형 체결 feature의 중심을 양쪽 부품에서 대조해 얻은 기계 회전 중심이다.
원본 mm 좌표를 runtime에서 meter로 사용한다.

| Joint | Bind pivot [m] | Runtime local axis |
|---|---|---|
| J1 | `(0.0000, 0.1985, 0.00000)` | `+Y (0,1,0)` |
| J2 | `(0.1535, 0.3100, 0.10000)` | `+X (1,0,0)` |
| J3 | `(0.1095, 0.9100, 0.10000)` | `+X (1,0,0)` |
| J4 | `(0.0000, 1.0150, 0.23350)` | `+Z (0,0,1)` |
| J5 | `(-0.1385, 1.0150, 0.69100)` | `+X (1,0,0)` |
| J6 | `(0.0000, 1.0150, 0.85475)` | `+Z (0,0,1)` |

ToolFrame bind world position은 `(0, 1.0150, 0.9145)` m다.

이 값의 출처 분류는 `docs/MODEL_DATA_PROVENANCE.md`를 따른다.

---

## 4. 움직이는 관절의 초기 회전을 항등으로 두는 이유

이전 중간 모델에서는 물리축을 모두 local Z처럼 다루기 위해 J1/J2/J3/J5 등에 약 ±90° bind quaternion을 넣는 방식도 사용했다.
그 방식은 시각적으로는 맞을 수 있지만 런타임에서 다음 두 종류 회전을 동시에 이해해야 한다.

```text
asset bind rotation
×
command joint rotation
```

그 결과 Inspector에서 `w≈-0.707`, `z≈0.707` 같은 값이 보이고, 잘못된 local axis를 전달하면 회전이 사선처럼 보이기 쉬웠다.

현재 자산은 다음처럼 단순화했다.

```text
J1..J6 bind rotation = identity
Joint position       = 실제 기계 pivot
Joint axis           = 모델 specification에 명시
```

따라서 arm 회전은 다음 의미가 된다.

```text
q [rad]
+
joint pivot + local axis
→ RobotKinematics
→ Joint local rotation / LinkPose
→ RobotTransformAdapter / RobotPhysicsAdapter
```

현재 `RobotTransformAdapter`도 이 asset contract를 검사한다. J1~J6 bind quaternion이 identity 회전을 나타내지 않으면 controller-ready GLB가 아니라고 판단한다. `q`와 `-q`는 같은 회전으로 취급한다.

---

## 5. 화면 형상에 변환 적용하기

Joint frame을 identity bind로 정리하면서 외형이 움직이면 안 된다.
따라서 기존 visual node에 들어 있던 CAD 정렬 transform을 mesh vertex/normal에 미리 반영했다.

개념적으로 기존:

```text
ParentWorld * VisualLocal * Vertex
```

를 다음으로 바꾼다.

```text
ParentWorld * Identity * (VisualLocal * Vertex)
```

즉 **world bind pose는 유지하면서 visual node transform만 단순화**한다.

Arm visual bind-pose 검증에서 최대 world-space vertex 오차는 약 `1.192e-7 m`로 floating-point 오차 수준이었다.

---

## 6. Robotiq 2F-85 계층 구조

현재 gripper는 단순 `LeftFingerJoint / RightFingerJoint` 두 개가 아니다.
다음 six-joint free-space linkage 구조를 사용한다.

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

Moving gripper joint도 bind rotation은 identity다.
현재 GLB에 맞춘 local rotation axis는 `-Z`다.

---

## 7. 그리퍼 형상 나누기

기존 runtime asset의 좌/우 finger mesh는 여러 실제 rigid part가 하나의 mesh로 합쳐진 상태였다.
현재 controller-ready 자산에서는 원본 finger geometry의 connected component를 rigid body 단위로 분류해 다음 그룹으로 재구성했다.

```text
outer knuckle
finger
inner knuckle
finger tip
```

Triangle을 임의로 잘라내거나 형상을 단순화하지 않았다.
검증 결과 좌우 각각 원본 triangle 수와 재구성 triangle 수가 동일했다.

```text
Left  : 12,952 -> 12,952
Right : 12,952 -> 12,952
```

공개 2F-85 kinematic frame을 현재 gripper geometry에 매핑한 것이므로 pivot/plane/axis 좌표는 **asset-specific**이다. 제조사 내부 motor 설계 좌표로 해석하면 안 된다.

---

## 8. 그리퍼 애니메이션 제거

검증 단계에서는 `Open -> Close -> Open` animation을 넣어 linkage가 실제로 접히는지 확인했지만, 현재 runtime 자산에서는 제거했다.

현재 contract:

```text
GLB
= geometry + hierarchy + pivot/frame

C++ Controller
= command/state + joint rotation
```

현재 자유 공간 제어 흐름은 다음과 같다.

```text
rPR 0..255
   ↓
SimGripperController
   ↓
GripperState.closureFraction [0,1]
   ↓ GripperKinematics
master linkage q [rad]
   ↓
mimic relation
   ↓
6개 gripper Joint local rotation
   ↓ GripperTransformAdapter
GLB 관절 / 기존 7개 Kinematic proxy
```

`rPR=0`은 fully open, `255`는 fully closed지만, `rPR` 자체는 radian이나 mm가 아니다.

---

## 9. 2F-85 자유 공간 검증

현재 geometry와 linkage frame으로 확인한 nominal free-space 결과:

```text
OPEN   fingertip inner gap ≈ 85.00 mm
CLOSED fingertip inner gap ≈ 0.97 mm
nominal closed master q    ≈ 0.7929 rad
```

`0.7929 rad`는 공개 kinematic reference의 master linkage 기준각이며 실제 내부 motor shaft angle이 아니다.

실제 2F-85는 under-actuated 구조다. 물체가 먼저 한 phalanx에 접촉하면 이후 joint 관계가 자유 공간의 고정 mimic 관계와 달라질 수 있다. 현재 구현은 접촉에 따른 그리퍼 닫힘 정지와 양쪽 손끝이 같은 Dynamic 물체의 반대 면에 닿았을 때 만드는 고정 constraint를 지원한다. 접촉 뒤 개별 손가락의 적응 움직임과 힘·마찰 기반 파지 안정성 계산은 구현하지 않았다.

---

## 10. 런타임 변환 경로

Arm은 다음 순서로 움직인다.

```text
Hcr12a.h
  axis / limit / velocity
        ↓
SimRobotController
  RobotState.q [rad]
        ↓
RobotKinematics
  q + pivot / axis
  → joint rotation + link pose
        ↓
RobotTransformAdapter
  quaternion → ECS unit quaternion
        ↓
Entity::SetLocalRotation
        ↓
TransformSystemModule
  Local = T * R * S
  World = ParentWorld * Local
        ↓
Renderer
```

Pivot translation은 GLB hierarchy에 이미 들어 있다. `RobotTransformAdapter`가 다시 더하지 않고, `RobotKinematics`가 동일한 bind pivot을 사용해 base-frame LinkPose를 계산한다.

glTF `[x,y,z,w]`는 로더에서 GLM 생성자 `(w,x,y,z)`로 옮긴다. `NodeData`·Entity는 quaternion을 보관하고 행렬 합성 전에 검증·정규화한다. 영 quaternion·NaN 등 비유한 입력은 거부하며 Euler로 왕복 변환하지 않는다. 자세 저장과 행렬은 float, 관절각은 rad 스칼라를 유지한다. 화면과 물리 계산 흐름은 [프로젝트 안내](ARCHITECTURE.md)를 참고한다.

---

## 11. 코드에서 기준이 되는 값

아래 인터페이스 헤더와 구현 파일이 현재 런타임 계약의 코드 기준이다. 로봇·그리퍼 상수는 header-only model specification이므로 별도 `.cpp` 구현 파일은 없다.

| 책임 | 인터페이스 또는 상수 | 구현 |
|---|---|---|
| HCR-12A 모델 상수 | `modules/robotics/include/robotics/models/hanwha/Hcr12a.h` | Header-only |
| 2F-85 모델 상수 | `modules/robotics/include/robotics/models/robotiq/TwoF85.h` | Header-only |
| Robot Controller 계약 | `modules/robotics/include/robotics/core/IRobotController.h` | Interface only |
| Simulation Robot Controller | `modules/robotics/include/robotics/backends/simulation/SimRobotController.h` | `modules/robotics/src/backends/simulation/SimRobotController.cpp` |
| Robot FK | `modules/robotics/include/robotics/kinematics/RobotKinematics.h` | `modules/robotics/src/kinematics/RobotKinematics.cpp` |
| Gripper FK | `modules/robotics/include/robotics/kinematics/GripperKinematics.h` | `modules/robotics/src/kinematics/GripperKinematics.cpp` |
| Robot GLB transform adapter | `modules/simulation/include/simulation/robotics/RobotTransformAdapter.h` | `modules/simulation/src/robotics/RobotTransformAdapter.cpp` |
| Robot physics proxy adapter | `modules/simulation/include/simulation/robotics/RobotPhysicsAdapter.h` | `modules/simulation/src/robotics/RobotPhysicsAdapter.cpp` |
| Gripper GLB transform adapter | `modules/simulation/include/simulation/robotics/GripperTransformAdapter.h` | `modules/simulation/src/robotics/GripperTransformAdapter.cpp` |
| Gripper contact / fixed-constraint adapter | `modules/simulation/include/simulation/robotics/GripperGraspAdapter.h` | `modules/simulation/src/robotics/GripperGraspAdapter.cpp` |
| Gripper collision proxy setup | `modules/simulation/include/simulation/robotics/GripperColliders.h` | `modules/simulation/src/robotics/GripperColliders.cpp` |
| Robot collision geometry builder | `modules/simulation/include/simulation/robotics/RobotCollisionGeometryBuilder.h` | `modules/simulation/src/robotics/RobotCollisionGeometryBuilder.cpp` |
| Physics ECS synchronization | `modules/simulation/include/simulation/systems/PhysicsSystemModule.h` | `modules/simulation/src/systems/PhysicsSystemModule.cpp` |
| Numeric provenance and transform classification | [Model Data Provenance](MODEL_DATA_PROVENANCE.md) | Documentation |

이 문서보다 코드/자산이 변경되면 위 source-of-truth를 먼저 확인하고 문서를 함께 갱신한다.
