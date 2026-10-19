# 모델 데이터 출처

## 왜 출처를 나누는가?

모델 코드에는 제조사 규격뿐 아니라 CAD에서 읽은 치수, 공개 기구학 모델을 참고한 값, 현재 GLB에 맞춘 좌표도 들어 있다. 예를 들어 `TwoF85.h`의 pivot과 `0.7929 rad`는 Robotiq의 공식 모터 사양이 아니다. 아래 분류를 보면 각 값이 어디서 왔고 어떤 의미까지 뒷받침하는지 알 수 있다.

## 분류

| 분류 | 의미 | 예시 |
|---|---|---|
| `MANUFACTURER` | 제조사 매뉴얼/제품 명세/장치 protocol이 직접 정의한 값 | HCR 관절 limit·속도, 2F-85 rPR/rSP/rFR 범위 |
| `CAD-DERIVED` | STEP/CAD의 실제 체결 형상과 축에서 계산한 값 | HCR J1~J6 기계 pivot 중심 |
| `KINEMATIC-REFERENCE` | 공개 URDF/Xacro 등 기구학 모델에서 가져온 시뮬레이션 기준 | 2F-85 master closed angle, mimic 관계 |
| `ASSET-DERIVED` | 위 데이터를 현재 controller-ready GLB 좌표계에 맞춰 변환한 값 | 2F-85 Gripper-local pivot, local -Z axis, joint plane Z, HCR Link collider 기준 |

`KINEMATIC-REFERENCE`와 `ASSET-DERIVED`는 제조사의 내부 모터 설계값을 의미하지 않는다.

## 출처 추적 범위

이 표는 각 값이 어떤 종류의 근거에서 왔다고 코드와 문서가 분류하는지 보여 준다. 분류와 repository 내 상수 경로만으로 원자료를 독립 재현할 수 있는 것은 아니다.

| 값 분류 | 현재 저장소에서 따라갈 수 있는 내용 | 아직 기록되지 않은 추적 정보 |
|---|---|---|
| HCR 제조사 limit·속도, Robotiq raw code | 모델 header에 정규화한 값과 일반 출처 설명이 있다. 아래 runtime 요약에서 상수 header를 연결한다. | 확인한 제조사 문서의 정확한 제목·revision·페이지·URL과 각 숫자의 직접 인용 위치가 없다. |
| HCR pivot | STEP assembly의 체결 원통 feature에서 읽은 mm 값을 meter로 저장했다고 기록돼 있다. | 원본 CAD 파일 revision, 측정 feature 목록과 좌표를 다시 산출한 절차/도구가 함께 보존돼 있지 않다. |
| 2F-85 kinematic reference | 공개 기구 모델을 바탕으로 한 자유공간 기준으로 분류하고 `0.7929 rad`와 mimic 관계를 model header에 둔다. | 사용한 upstream model의 revision/URL 및 값별 원본 식별자가 없다. 이 값은 제조사 내부 motor specification이 아니다. |
| GLB-derived 좌표·geometry | asset 이름, hierarchy 기준과 주요 normalization 계약을 문서화했다. | 일부 pivot, triangle 재구성 수와 vertex error 수치의 재계산 명령 또는 변환 script가 기록돼 있지 않아 독립 재현성이 제한된다. |

따라서 이 문서의 provenance tag는 알려진 출처 유형을 구분하지만, primary-source citation이나 재현 패키지를 대신하지 않는다. 위 자료가 확보되지 않은 항목에는 문서명·URL·측정 절차를 추정해 채우지 않는다. 모델 좌표 변환은 [GLB 정규화](HCR12A_GLB_NORMALIZATION.md), 현재 runtime 사양과 접촉 동작은 [HCR-12A + 2F-85 Simulation Specification](HCR12A_2F85_simulation_specs.md)를 참고한다.

---

## HCR-12A

코드 위치:

```text
modules/robotics/include/robotics/models/hanwha/Hcr12a.h
```

### 관절 한계와 최대 속도 — 제조사 자료

현재 runtime specification은 HCR-A(2G) 계열에 사용한 다음 값을 SI 단위로 정규화한다.

| Joint | limit [deg] | runtime limit [rad] | max [deg/s] | runtime [rad/s] |
|---|---:|---:|---:|---:|
| J1 | -180 ~ 180 | -3.141593 ~ 3.141593 | 130 | 2.268928 |
| J2 | -165 ~ 135 | -2.879793 ~ 2.356194 | 130 | 2.268928 |
| J3 | -85 ~ 245 | -1.483530 ~ 4.276057 | 200 | 3.490659 |
| J4 | -190 ~ 190 | -3.316126 ~ 3.316126 | 200 | 3.490659 |
| J5 | -170 ~ 170 | -2.967060 ~ 2.967060 | 200 | 3.490659 |
| J6 | -360 ~ 360 | -6.283185 ~ 6.283185 | 200 | 3.490659 |

변환은 개념적으로 다음과 같다.

```text
rad   = deg * pi / 180
rad/s = deg/s * pi / 180
```

### J1~J6 회전 중심 — CAD 기반

Pivot은 mesh bounding-box 중심이나 임의의 회전점이 아니다.
HCR-12A STEP assembly에서 서로 맞물리는 원통형 체결 feature의 중심을 양쪽 부품에서 대조해 결정했다.
원본 CAD mm 좌표를 runtime에서는 meter로 저장한다.

| Joint | bind pivot [m] | physical axis |
|---|---|---|
| J1 | `(0.0000, 0.1985, 0.00000)` | `+Y` |
| J2 | `(0.1535, 0.3100, 0.10000)` | `+X` |
| J3 | `(0.1095, 0.9100, 0.10000)` | `+X` |
| J4 | `(0.0000, 1.0150, 0.23350)` | `+Z` |
| J5 | `(-0.1385, 1.0150, 0.69100)` | `+X` |
| J6 | `(0.0000, 1.0150, 0.85475)` | `+Z` |

ToolFrame bind 위치는 `(0, 1.0150, 0.9145)` m다.

### 제어기에 쓰는 축 — CAD 및 자산 기반

물리 회전축 방향은 CAD에서 확인했지만, 코드의 `axis`는 현재 controller-ready GLB의 Joint local frame에서 사용하는 단위축이다.
현재 GLB는 J1~J6 moving node의 bind rotation을 identity로 만들었으므로 physical axis와 runtime local axis가 직접 일치한다.

### 링크 충돌 형상 — 자산 기반

HCR-12A Link collider는 `HCR12A_2F-85.glb`의 재질 메시에서 삼각형 연결별로 생성한다. 같은 위치의 mesh seam 정점도 이어 분리 부품을 찾는다. 4 cm 미만 부품은 제외하고, 나머지 삼각형은 중심점 기준 관절 좌표계의 16 cm 셀에 묶는다. 삼각형은 셀 경계에서 자르지 않으며 1 cm 미만 셀과 부피가 없는 hull 입력도 제외한다. 방향별 극점으로 축약한 Convex Hull은 오목한 부분을 메울 수 있다. 다음 가동 관절 아래와 Gripper geometry는 `RobotPhysicsAdapter`의 대상에서 제외한다.

2F-85의 별도 collider 구성기는 현재 GLB의 이름 있는 rigid-part mesh마다 방향별 극점으로 정점을 축약해 Convex Hull을 만든다. 고정 `GripperMesh`, 좌우 outer knuckle과 각 attached finger mesh의 compound, 좌우 inner knuckle, 좌우 fingertip으로 총 일곱 Kinematic proxy다.

Mesh 정점을 owning authored Gripper/joint 원점 기준으로 바꿔 shape를 저장하므로 collider 중심은 각 body 원점과 다를 수 있다. 모델 및 Scene 계층의 scale은 unit이어야 한다.

이는 현재 `HCR12A_2F-85.glb`에 맞춘 asset-derived 형상이며 다른 GLB에 그대로 적용할 수 없다. 초기 open pose는 약 85 mm gap이다.

개폐 자세는 `GripperState.closureFraction`에서 계산한 Local 회전을 authored GLB 관절에 적용해 공급하며, 기존 proxy는 자식 계층의 World 변환을 따른다.

볼록 외피는 각 16 cm 구간 안의 오목한 부분을 메울 수 있고, 두께가 없는 평면 장식 조각은 충돌에서 제외한다. 제조사 CAD 충돌 모델과 비교 검증한 값은 아니므로 ImGui 충돌 표시와 실제 접촉 결과로 계속 확인한다.

---

## Robotiq 2F-85

코드 위치:

```text
modules/robotics/include/robotics/models/robotiq/TwoF85.h
```

### rPR / rSP / rFR — 제조사 자료

`0..255`는 Robotiq 장치가 사용하는 command 값이다.

```text
rPR  position request  0=open, 255=closed
rSP  speed request     0..255
rFR  force request     0..255
```

이 값은 meter나 radian이 아니다.
향후 Hardware backend는 장치 register에 매핑하고, Simulation backend는 모델별 기구학 상태로 변환한다.

### 0.7929 rad와 mimic 관계 — 기구학 참조 모델

`kTwoF85NominalClosedMasterRadians = 0.7929`는 공개 2F-85 kinematic model의 free-space nominal closed master joint 기준값이다.

중요한 해석:

```text
0.7929 rad = simulation linkage master joint angle
0.7929 rad != Robotiq 내부 motor shaft angle
```

`masterMultiplier = ±1`도 하나의 actuator command에서 좌우 knuckle/fingertip을 연동하기 위한 free-space mimic 관계다.
물체 접촉 이후 실제 2F-85의 under-actuated adaptive motion까지 고정 mimic으로 표현하는 값은 아니다.

### 회전 중심, Local -Z 축, 관절면 — 자산 기반

다음 값은 현재 `assets/HCR12A_2F-85.glb`에 맞춘 프로젝트 좌표다.

```text
joint plane Z = 0.0934257339 m
joint axis    = (0, 0, -1) in Gripper local frame
```

Pivot:

| Joint | Gripper-local pivot [m] |
|---|---|
| LeftOuterKnuckleJoint | `(-0.03060114, 0.05490452, 0.0934257339)` |
| RightOuterKnuckleJoint | `(+0.03060114, 0.05490452, 0.0934257339)` |
| LeftInnerKnuckleJoint | `(-0.01270000, 0.06142000, 0.0934257339)` |
| RightInnerKnuckleJoint | `(+0.01270000, 0.06142000, 0.0934257339)` |
| LeftFingerTipJoint | `(-0.06775864, 0.09832620, 0.0934257339)` |
| RightFingerTipJoint | `(+0.06775864, 0.09832620, 0.0934257339)` |

이 좌표들은 공개 kinematic frame을 기존 gripper geometry에 정렬해 만든 값이므로 **다른 2F-85 GLB에 그대로 적용하면 안 된다.**
Asset이 바뀌면 다시 검증해야 한다.

---

## 런타임 변환 흐름

아래 흐름의 상수는 로봇 모델 헤더에 정의되어 있다. 코드가 자세를 화면과 물리에 적용하는 순서는 [프로젝트 안내](ARCHITECTURE.md)에 정리했다.

HCR-12A:

```text
MANUFACTURER / CAD
      ↓ normalize
Hcr12a.h [m, rad, rad/s]
      ↓
SimRobotController -> RobotState q [rad]
      ↓
RobotKinematics
      q + joint pivot / local axis
      -> joint rotation + base-frame link poses
      ↓
      ├─ RobotTransformAdapter -> current ECS unit quaternion
      └─ RobotPhysicsAdapter -> Kinematic link proxies
      ↓
Flecs / GLB
```

2F-85의 현재 자유공간 런타임은 다음 경계를 사용한다.

```text
rPR [0..255]
      ↓
SimGripperController -> GripperState.closureFraction [0,1]
      ↓ GripperKinematics
master q = fraction * 0.7929 [rad]
      ↓ masterMultiplier
6 linkage Joint local delta rotations
      ↓ GripperTransformAdapter: bind * delta
authored GLB 관절 -> World 변환 -> 기존 Kinematic proxies
```

raw 위치를 fraction으로 선형 대응하는 식과 `SimGripperMotionSettings` 기본 master 속도 0.1..1.0 rad/s는 프로젝트의 시뮬레이션 가정이다. 제조사 위치 보정식이나 위 제조사 finger speed를 환산한 값이 아니다. raw speed 0..255를 이 각속도 범위에 선형 대응하며 raw 0도 양의 최소 속도다. `actualPosition`은 연속 위치의 표시용 반올림이며 관절 계산에 재입력하지 않는다.

4 ms 고정 갱신과 DLS IK 설명은 [프로젝트 안내](ARCHITECTURE.md)에서 읽을 수 있다. 현재 그리퍼는 물리 접촉 뒤 닫힘을 멈추고, 양쪽 손끝이 같은 물체에 닿으면 고정 constraint로 연결한다. force는 범위만 검사하며 전류·실제 힘·개별 손가락 적응은 계산하지 않는다. Viewer TCP는 열린 손끝 메시의 중심으로 정한 시뮬레이션 기준이며 제조사 보정값이 아니다.

## 작성 규칙

앞으로 model constant를 추가할 때 다음을 지킨다.

1. 주석에 `MANUFACTURER`, `CAD-DERIVED`, `KINEMATIC-REFERENCE`, `ASSET-DERIVED` 중 출처 성격을 적는다.
2. 숫자에는 단위를 적는다.
3. 좌표는 어느 frame인지 적는다.
4. 변환된 값이라면 원 단위와 runtime 단위를 함께 기록한다.
5. 공개 simulation model의 값을 제조사 공식 기계/모터 사양처럼 서술하지 않는다.
6. asset-specific 값은 대상 GLB 이름과 좌표계 contract를 함께 기록한다.
