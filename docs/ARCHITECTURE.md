# 프로젝트 구조

GraspLink는 Controller가 계산한 자세를 장면과 물리 엔진에 전달해 화면에 그립니다. 이 문서는 그 과정에 참여하는 모듈과 각자의 책임을 설명합니다.

## 모듈과 책임

```text
modules/
├── diagnostics/   비동기 로그, 수치 지표, 프로파일링 기록
├── robotics/       로봇 모델, Controller 계약, backend, 정기구학
├── physics/        Jolt 연동과 엔진 독립 물리 타입
├── viewer/         Flecs 장면, 변환, GLB 자산, OpenGL 렌더링
├── simulation/     Physics-ECS 연동, 로봇 충돌 프록시, 바닥 설정
└── gui/            ImGui 패널과 충돌 형상 표시
```

아래 화살표는 기능을 사용하는 쪽에서 의존하는 쪽으로 향합니다. CMake 타깃의 주요 의존 관계는 다음과 같습니다.

```mermaid
flowchart TD
    App[ViewerApp] --> GUI[GUI]
    App --> Simulation[시뮬레이션]
    App --> Viewer[뷰어]
    App --> Diagnostics[진단]
    GUI --> Simulation
    GUI --> ImGui
    Simulation --> Viewer
    Simulation --> Robotics
    Simulation --> Physics
    Simulation --> Diagnostics
    Viewer --> Robotics
    Viewer --> Graphics[OpenGL 렌더링]
    Viewer --> Flecs
    Physics --> Jolt
```

`modules/physics`는 Flecs, Viewer, OpenGL에 의존하지 않는다. Jolt 전용 타입도 이 모듈 안에 둔다. Flecs에서 사용하는 물리 설정과 Entity를 `PhysicsBodyHandle`에 연결하는 런타임 binding은 `modules/simulation`이 관리한다.

`modules/diagnostics`는 시뮬레이션이나 그래픽 모듈에 의존하지 않는다. `Logger`는 로그와 의미 지표를 크기가 제한된 큐에 복사하고, 전용 스레드가 파일에 기록한다. 실행 시간은 Logger에 섞지 않고 Tracy로 측정한다. `ViewerApp`은 Logger를 빌려 쓰는 시스템보다 오래 보관하며, 종료할 때 기록을 모두 비우고 작업자 스레드를 기다린다. API와 출력 형식은 [진단 도구](DIAGNOSTICS.md)에 설명했다.

## 로봇 자세가 화면에 반영되는 과정

```text
IRobotController
→ RobotState
→ RobotKinematics
→ RobotKinematicState
   ├── RobotTransformAdapter → GLB Joint Entity 변환
   └── RobotPhysicsAdapter → Kinematic 링크 충돌 Entity
```

`RobotKinematics`가 로봇 자세를 계산하는 공통 경로다. 연속된 관절의 base-frame bind pivot에서 부모 기준 offset을 구하고, 직렬 관절의 회전을 차례로 누적한다. Viewer와 Physics는 관절축과 각도를 따로 해석하지 않고 같은 계산 결과를 사용한다. Robotics 모델은 기본 scalar, vector, quaternion 타입으로 표현하므로 GLM, Flecs, Jolt에 의존하지 않는다.

Controller 호출 계약과 상태·오류 구분은 [Controller 인터페이스](CONTROLLER_INTERFACE.md)를, HCR-12A 및 2F-85 수치의 출처는 [모델 데이터 출처](MODEL_DATA_PROVENANCE.md)와 [시뮬레이션 사양](HCR12A_2F85_simulation_specs.md)을 참고한다.

FK는 모델에 ToolFrame이 있으면 `toolFrameInBaseFrame`을 제공한다. `DampedLeastSquaresIk`는 ToolFrame에 고정 공구 변환을 더해 TCP 목표를 만드는 관절각을 계산한다.

`SimRobotController::MovePose`는 현재 자세 seed의 IK 결과를 관절 공간에서 검증해 이동하며, 미션은 비동기 `BeginPosePlanning`을 사용한다. MoveJ는 관절 사이를 직접 보간해 표본을 검사하고, 경로가 막히면 우회하지 않고 실패한다. TCP 직선은 보장하지 않는다.

`MoveLinear`은 TCP의 직선 위치와 최단 회전 경로를 따라가며, 직전 표본 해를 seed로 이어갈 수 없을 때만 대체 IK 분기를 탐색한다. `tcpPoseValid=true`인 시뮬레이션 상태는 Robot base 기준 모델 FK 결과이며 실제 장치 측정값이 아니다.

관절과 TCP의 속도·가속도는 시뮬레이션 궤적 정책으로 제한하지만, 로봇 토크·질량·관성 기반 동역학은 구현하지 않았다. Hardware Gripper backend도 구현하지 않았다.

상세 계약은 [로봇 이동과 파지](ROBOT_MOTION_AND_GRASP.md)를 참고한다.

`RobotTransformAdapter`는 계산한 자세를 GLB 관절 계층에 적용한다. `RobotPhysicsAdapter`는 연결된 GLB 메시에서 각 링크의 Convex Hull을 만들고, 그리퍼 형상은 별도 설정 함수가 처리한다. Robot base에도 고정 Environment 충돌 형상을 둔다. `ConfigureTwoF85Colliders`는 GLB 메시를 축약한 hull로 Gripper-layer Kinematic proxy 7개를 만들며, 각 proxy의 위치는 소유 body 또는 joint를 기준으로 둔다. Robot과 Gripper의 충돌 프록시는 Kinematic이며 관절 동역학은 계산하지 않는다.

## 그리퍼 상태 흐름

```text
GUI 요청 → IGripperController / SimGripperController
→ GripperState.closureFraction
→ GripperKinematics: master angle / mimic Local rotation
→ GripperTransformAdapter: bind rotation * delta rotation
→ authored GLB 관절 → World 변환 → 기존 7개 Kinematic proxy
```

`GripperState`의 유효한 연속 위치가 개폐 자세의 기준이다. raw 0..255는 요청·표시용이며 반올림한 raw feedback을 기구학 입력으로 다시 사용하지 않는다.

여섯 관절은 분기형이므로 팔의 직렬 FK를 재사용하지 않고 Local 회전 변화만 계산한다. 어댑터는 원본 Local 위치·크기와 Gripper/ToolFrame 장착 변환을 보존한다.

raw 위치의 선형 fraction 매핑과 master 속도 기본값 0.1..1.0 rad/s는 시뮬레이션 가정이다. `GripperGraspAdapter`는 물리 손끝 접촉으로 개폐를 정지시키고 양쪽 손끝이 같은 Dynamic 물체를 반대 방향에서 만지면 고정 constraint로 파지한다.

힘·전류·개별 손가락 적응은 계산하지 않는다. 자세한 계약은 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)를 참고한다.

`GuiModule`은 ImGui context·GLFW/OpenGL backend와 `BeginFrame/EndFrame`, 입력 capture 조회를 담당한다. `ViewerApp`이 `GripperPanel`, `PhysicsDebugPanel`, `ColliderOverlay`를 별도로 만들고 `BeginFrame → GripperPanel → PhysicsDebugPanel → ColliderOverlay → EndFrame` 순서로 조립한다.

`GripperPanel`은 호출 중에만 Controller를 빌려 요청·상태를 표시하고, `PhysicsDebugPanel`은 collider 표시 체크박스 상태를 보관한다. `ColliderOverlay`는 World query와 화면 선 캐시를 소유하며 Camera는 Draw 호출 중에만 빌린다.

선은 Jolt 내부 형상 대신 ECS 설정의 깊이 없는 전경 투영이며, 첫 표시·resize와 100 ms 간격으로 갱신한다.

## 변환과 고정 주기 흐름

`TransformSystemModule::UpdateWorldTransforms()`는 공통 Local-to-World 계산 경로다. 4 ms 고정 tick에서 두 Controller 상태와 팔·그리퍼 자세를 갱신한 뒤 World 행렬, Kinematic 동기화, Jolt step, Dynamic 결과의 Local 반영 순서로 진행한다. step 뒤 World 행렬을 다시 계산하며 렌더 직전에도 갱신한다.

`Rotation, Local`과 `NodeData.rotation`은 `glm::quat`이며 `Entity::GetLocalRotation` / `SetLocalRotation` 및 `ComposeLocalMatrix(position, rotation, scale)`도 quaternion을 사용한다. glTF 배열 `[x,y,z,w]`는 로더에서 GLM 생성자 `(w,x,y,z)`로 옮긴다. `Rotation` 생성과 행렬 합성은 회전을 검증·단위 정규화하며 영 quaternion·NaN·Infinity를 거부한다. `q`와 `-q`는 같은 회전이므로 성분 부호만으로 자세가 다르다고 판단하지 않는다.

팔·그리퍼 FK 결과는 quaternion으로 Entity에 전달한다. Dynamic은 SceneRoot 또는 항등 grouping 조상만 허용해 World≈Local 계약을 사용하므로, Physics 결과의 위치·quaternion을 Local에 직접 저장한다. 부모 역행렬이나 Local 행렬 분해는 하지 않는다. GLB·FK·Physics에서 quaternion과 Euler 사이의 왕복 변환을 제거했다. Entity pose·변환 행렬은 float 정밀도를 유지한다. 제어용 관절각과 각속도는 rad·rad/s 스칼라이며, Orbit 카메라의 마우스 입력용 yaw/pitch도 rad 각도를 사용한다.

```text
FixedControlLoop
→ GripperGraspAdapter BeforePhysicsStep: 파지 해제와 Body 수명 확인
→ RobotCollisionGuard CaptureSafeJointPose
→ RobotController + GripperController Update
→ RobotKinematics + GripperKinematics
→ Simulation pose adapters: GLB 관절과 Kinematic collision Entity 갱신
→ TransformSystemModule World transforms
→ RobotCollisionGuard: 후보 충돌 검사, 겹치면 안전 자세 복원
→ PhysicsSystemModule::Step
   ├─ PrePhysicsSync: 설정 재구성 / Scene-driven 목표 전달
   ├─ PhysicsWorld Step
   └─ PostPhysicsSync: Physics-driven Dynamic 결과를 Local에 직접 저장
→ GripperGraspAdapter AfterPhysicsStep: 접촉 피드백과 양쪽 파지 연결
→ TransformSystemModule World transforms 재갱신
→ Render frame: TransformSystemModule / RenderSystemModule
```

Physics는 render FPS와 독립된 고정 간격으로 진행한다. `IsSceneDriven`은 Static·Kinematic, `IsPhysicsDriven`은 Dynamic을 분류한다.

Static은 Scene 목표가 바뀔 때만 `SetBodyTransform`을 호출한다. Kinematic은 같은 목표라도 실제 도달 전에는 `MoveKinematic`을 계속하고, 도달 뒤 `StopKinematic`을 한 번 호출해 잔류 속도를 지운 다음 같은 목표의 동기화를 생략한다.

위치·회전이 바뀌면 다시 이동한다. Kinematic 로봇은 환경과 겹쳐도 Jolt가 목표를 자동으로 막지 않는다.

Controller의 경로 검사는 후보 자세를 사전에 검사하며, 각 fixed tick 뒤 guard도 다음 물리 step 전에 현재 Robot·Gripper 형상의 환경 또는 허용되지 않은 자가 충돌을 검사한다. 겹침이 있으면 직전 안전 관절 자세로 복원하고 오류 code를 기록한 채 Controller를 Idle로 둔다.

이는 Fault 상태 전이나 사전 충돌 회피 경로 계획이 아니다.

물리 Entity와 조상은 unit scale을 사용하며 Static Environment 자신의 시각 scale만 예외다. Collider 치수·offset은 m 단위로 직접 지정한다. 모든 Body는 Dynamic 조상을 금지하고, Dynamic 자체는 조상 각각의 변환이 항등이어야 한다. 실행 중 이 scale·부모 계약이 깨지면 binding과 Body를 제거하고, 복구되면 현재 Scene World 자세로 다시 만든다. 공개 pose는 model/Entity 원점 기준이며 compound의 무게중심 보정은 Jolt 내부 책임이다.

## 장면 구성과 객체 수명

`ViewerApp`은 앱에 필요한 객체를 조립하는 진입점이다. 의존 순서에 맞춰 Window, renderer, scene, controller, `PhysicsWorld`, adapter, `PhysicsSystemModule`, `GuiModule`을 만들고 수명을 관리한다. 개별 장면 객체를 만드는 기능은 각 모듈에 둔다.

`grasplink_scene`, `grasplink_model_data`, `grasplink_simulation`은 OpenGL 없이 빌드할 수 있다. `grasplink_scene`은 Flecs Entity와 변환을 관리하고, `grasplink_model_data`는 CPU에서 GLB 데이터를 읽는다. `grasplink_simulation`은 이 장면·모델 데이터에 robotics와 Jolt를 연결한다. 화면 렌더링, GPU 자산 업로드, 시뮬레이터 창은 `GRASPLINK_BUILD_GRAPHICS`를 켰을 때 추가된다.

`ConfigureFloor`는 화면 바닥 아래에 고정 Box 충돌체를 추가한다. `PickPlaceScenario`는 작업에 필요한 상자와 놓을 영역을 만들고, `grasplink::simulation::scenario`는 재현 가능한 위치와 회전을 선택한다.

`PrefabFactory`는 GLB 노드에서 화면에 표시할 Entity를 구성한다. primitive가 하나인 Node는 렌더링 component를 직접 소유하고, primitive가 여러 개면 각각을 자식 Entity로 만든다.

GLB loader는 선택한 scene root 아래에 하나의 노드 트리를 요구한다. sparse accessor, 잘못된 byte 범위나 stride, 지원하지 않는 matrix 변환, 순환 참조, 여러 부모가 있는 구조는 거부한다.

정점에서 position, normal, UV만 읽으며 normal mapping은 지원하지 않는다.

`ViewerApp`은 Flecs World와 PhysicsWorld를 소유한다. `ColliderOverlay`의 World query와 패널을 먼저 해제하고, 어댑터·Scene Entity를 제거한 뒤 Physics integration과 World를 정리한다. Flecs 제거 observer가 Entity의 Jolt Body도 삭제한다. `GuiModule`은 Window의 OpenGL context가 살아 있을 때 backend와 ImGui context를 정리한다.

`Scene`은 생성될 때 `SceneRoot` 하나를 만들고, 소멸할 때 자식 Entity를 정리한다. Viewer는 실행 중 하나의 Scene을 사용하며 장면 전환이나 Scene별 갱신 수명 주기는 없다. 종료할 때는 Physics observer가 살아 있는 동안 adapter와 Scene Entity를 먼저 정리한 뒤 연동 객체와 World를 해제한다. CPU 쪽 `ModelResource`와 GPU 자산의 소유권은 분리돼 있다. `AssetManager`가 resource ID별 GPU Mesh를 보관하고 Entity는 공유 `MeshFilter`를 참조하므로, OpenGL context가 사라지기 전에 GPU 자산이 해제된다.

renderer는 물체 가장자리를 부드럽게 보이도록 멀티샘플 안티앨리어싱(MSAA)을 사용하고, 색상 렌더 패스를 한 번 실행한다. Shadow map이나 별도 depth pass는 만들지 않는다.

충돌 layer, 변환 좌표계, scale 조건과 아직 구현하지 않은 로봇 물리 기능은 [Physics와 Flecs 연결](PHYSICS_ECS_INTEGRATION.md)에서 더 자세히 설명한다.
