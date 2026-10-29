# Simulation 감사

감사 기준 리비전: d6515b6e. 이후 변경은 CHECKPOINT와 VERIFICATION에서 확인한다.

2026-10-29에 JSON 입력·결과 처리만 맡던 실행 코드를 `scenario` 모듈로 옮겼다. 아래 파일별 판정은 원래 조사 당시의 경로를 포함할 수 있으며, 현재 위치는 `modules/scenario`와 `tests/scenario`다. 이 이동은 SimulationCore의 내부 구조 변경을 뜻하지 않는다.

기준: 4대 고정 V M01, 분산 정찰 M02, 복귀 M03, 직접 통신·라이브 EO. 2026-10-29 사용자 요청으로 EO 영상·사진 파일 저장을 제거했다. 아래 처음 감사한 내용의 ‘로컬 녹화’ 언급은 당시 기준선이며 현재 결정은 아니다. inventory의 simulation 52개 전부 본문을 읽었고, 이번에 SimulationCore의 실제 시스템 순서와 PhysicsWorld 연결을 다시 확인해 아래 조사 기록을 보강한다. 해당 조사와 파일별 판정은 정적 확인이며 빌드·시험을 뜻하지 않는다.

## 중요한 교차 확인 사항

- ScenarioRunner.cpp:475–598 검색에서 임무 재할당을 확인했다. UpdateEnergyPolicy(81)에도 저배터리 복귀 정책이 있다. apps/simulator/AeroSimLabAppMission.cpp:273,456,617에서 앱도 경로 계획을 호출한다. 임무 책임의 중복 여부는 본문·호출 흐름 전체 미확인이라 결론을 보류한다.
- FormationPlanner.cpp:387–408은 요청 편대 경로 실패 때 Column을 시도한다. 고정 V 요구와 다르므로 단순화 후보이지만 경로 없음과 장애물 여유를 처리해야 한다.
- DroneModel.h:108의 1/60초와 SimulationCore.cpp:660–662,907,914의 누적·ECS 진행·배터리 갱신이 연결된다. SimulationCore.cpp에서 진행 순서와 소유권을 확인했으며 실행 검증은 하지 않았다.
- PhysicsWorld.cpp:49–53,485–487,915–916에서 좌표 변환, 전역 AirSim 시계, 별도 접촉 계산을 확인했다. 수명·동시 월드 및 충돌 정확성은 미검증이다.
- MapFlightRoute.cpp:164,281에 우선순위 큐와 경로 계획 함수가 있다. A*의 장애물 판정과 실패 반환을 확인했으며 경로 중간 지형 높이는 별도 검증이 필요하다.

## 파일별 범위와 판정
- `apps/headless/main.cpp`: SIMPLIFY — 현재 main은 입력 JSON으로 RunScenario를 호출하고 결과를 출력한다. 임무 결과에는 배정·경유점·완료 상태를 담는다. 과거 점검면·촬영 판정 필드는 2026-10-29 제거했다.
- `modules/simulation_core/include/simulation_core/BatteryModel.h`: KEEP — 1–46. 배터리 설정을 입력받아 고정 틱 소비량·잔량·복귀 조건을 제공하고 에너지와 복귀 표시를 소유한다(41–44). 오류 검사는 구현 참조.
- `modules/simulation_core/include/simulation_core/CommandRecording.h`: REMOVED — 당시 명령과 Tick을 저장하고 재생 범위를 span으로 빌려주던 기능이다. 2026-10-29 조종 명령 기록·재생 제품 기능 제거와 함께 삭제했다.
- `modules/simulation_core/include/simulation_core/DroneModel.h`: KEEP — 1–133. 명령·상태·제어값과 결과 유형, Apply 및 AdvanceOneTick을 정의한다(77–126). 위치 ENU, 각속도 FLU, tick은 1/60초(108). AirSim과 운동학 이중 경로의 실제 빌드 요구는 확인 필요.
- `modules/simulation_core/include/simulation_core/EoCameraModel.h`: SIMPLIFY — 현재는 카메라 모델·상태, Tick 프레임 증가, 기체에 붙은 카메라 자세, 켜기·끄기·고장·초기화를 제공한다. 과거 점검면·가림 형상과 촬영 가능 판정은 2026-10-29 제거했다.
- `modules/flight_planning/include/flight_planning/FormationPlanner.h`: 분리 완료 — 편대 위치와 편대 경로 계획의 공개 입력·결과를 선언한다. Simulation Core 자료형을 사용하지 않는다.
- `modules/scenario/include/scenario/ScenarioJson.h` (당시 `simulation_core/InspectionSurfaceJson.h`): REMOVED — 시나리오 JSON 안의 점검면 기하를 읽던 헤더. 자동 촬영 판정을 제거하면서 파서와 함께 삭제했다.
- `modules/simulation_core/include/simulation_core/PhysicsWorld.h`: KEEP — 확인 1–146. AirSim에 강체 생성·모터·지형·장애물·틱을 제공하며 Impl을 unique_ptr로 소유하고 복사를 금지한다(68–145). 잘못된 핸들의 DestroyBody는 false, 비유한 모터 입력은 거부(95,125). 충돌·좌표 계약 보존; 내부 수명은 구현 미완독.
- `modules/simulation_core/include/simulation_core/QuadrotorFlightController.h`: KEEP — 확인 1–51. 상태·목표·물성·이득을 받아 경유 속도와 4개 로터 목표를 값 반환(35–49). 자체 가변 상태 없음. 오류 동작은 구현 미확인; 강체 비행 제어에 필요.
- `modules/simulation_core/include/simulation_core/QuadrotorModel.h`: KEEP — 확인 1–60. 질량·관성·로터·충돌 상자·모터 계수를 값으로 소유하며 X500 기본 및 추정값을 반환한다(32–57). 오류·실제 호출은 구현 미확인. 별도 추정 모델은 MVP 사용 확인 후 정리 후보.
- `modules/scenario/include/scenario/ScenarioRunner.h` (당시 `simulation_core/ScenarioRunner.h`): SIMPLIFY — RunScenario가 JSON을 받아 드론·임무·궤적 결과를 값으로 돌려준다. 잘못된 문법·필수값은 예외로 알린다. 자동 촬영 재시도, 구도 판정 결과, 임무 배정 이력은 2026-10-29 제거했다. 임무 담당 드론은 입력에서 고정한다.
- `modules/simulation_core/include/simulation_core/ScopedDuration.h`: KEEP — 확인 1–52. 생성 때 시간 시작, 소멸 때 전달받은 double에 시간 대입·누적한다(21–38). 포인터는 비소유이며 대상이 범위보다 오래 살아야 한다. null은 측정 생략. 벤치마크 소비자 확인 후 제거 가능성 검토.
- `modules/simulation_core/include/simulation_core/SimulationCore.h`: KEEP — 확인 1–235. FixedTickAccumulator(55–66)는 잔여 시간을 소유하고 프레임 입력을 100ms로 제한한다. SimulationCore(72–234)는 ECS를 unique_ptr로 소유하고 명령·드론·카메라·배터리·충돌·틱 API를 제공한다. 없는 드론·배터리는 일부 예외, 마지막 드론은 제거 불가. 네이티브 AirSim과 웹 운동학 선택(31–38)은 호출 환경 확인 후 정리.
- `modules/simulation_core/src/BatteryModel.cpp`: KEEP — 1–91. 생성자(9)는 설정 범위 오류에 예외를 던지고 Advance(33)는 모드별 평균 전력과 고정 틱으로 잔량을 갱신한다. Reset(46)은 초기량과 복귀 표시를 복원한다. 복귀 자체는 실행하지 않는다.
- `modules/simulation_core/src/CollisionGeometry.cpp`: KEEP — 확인 1–200, 201–327. 외접구·AABB 및 분리축으로 상자 접촉을 계산하고(39,253), 외곽선 투영으로 footprint·prism 접촉을 반환한다(106,172,194). 유효하지 않거나 겹치지 않으면 nullopt; 값 기반 지역 상태. EO의 별도 다각형 판정과 목적이 달라 통합 전 경계 규칙 검토 필요.
- `modules/simulation_core/include/simulation_core/CollisionGeometry.h`: KEEP — 강체 상자·외곽선의 접촉 계산과 입력 유효성 검사를 선언한다. span은 비소유다. 접촉 실패와 일부 잘못된 입력이 `nullopt`로 합쳐지는 의미에 주의한다. 호출 모듈이 함께 사용하는 공개 포함 경로에 둔다.
- `modules/simulation_core/src/CommandRecording.cpp`: REMOVED — 당시 기록 경계 Tick과 재생 인덱스를 관리했다. 앱·HUD·UDP 호출처와 함께 삭제했다.
- `modules/simulation_core/src/DroneModel.cpp`: KEEP — 확인 1–200, 201–315. Apply(72)는 상태·명령을 검증해 결과를 반환하고 상태·제어값을 갱신한다. AdvanceOneTick(185)은 고정 간격으로 이륙·이동·착륙과 지면 접촉을 계산한다. 외부 상태를 참조로 수정하며 로터 명령은 강체 경로와 연결 확인 필요. 운동학과 강체의 동작 차이는 구현 검토 필요.
- `modules/simulation_core/src/EoCameraModel.cpp`: SIMPLIFY — 현재는 카메라 프레임·자세·상태를 갱신한다. 과거의 점검면 거리·각도·화면·해상도·건물 가림 판정은 2026-10-29 제거했다. 카메라 고장 상태에서 켜기 요청을 무시하는 동작은 유지한다.
- `modules/flight_planning/src/FormationPlanner.cpp`: 분리 완료 — V자 오프셋을 진행 방향에 맞춰 회전하고, 편대 전체 폭을 반영한 여유값으로 지형 경로를 요청한다. 현재 구현에서 다른 대형 전환 기능은 확인되지 않았다.
- `modules/scenario/src/ScenarioJson.cpp` (당시 `simulation_core/InspectionSurfaceJson.cpp`): REMOVED — 점검면 기하 JSON 파서. 촬영 조건 판정과 함께 2026-10-29 삭제했다.
- `modules/simulation_core/src/PhysicsWorld.cpp`: KEEP — 확인 1–200, 201–400, 401–600, 601–800, 801–1000, 1001–1200, 1201–1270; 남은 줄 없음. Impl은 강체와 AirSim 시계를 소유하고 엔진에 강체 주소를 빌려준다(403–510); 소멸 전 clear. Step(1150)은 접촉·모터·엔진을 순서대로 실행하며 좌표 변환과 지형·장애물 접촉을 처리한다. 입력 오류와 미존재 핸들은 예외. 전역 시계의 동시 월드 실행 안전성은 미검증.
- `modules/simulation_core/src/QuadrotorFlightController.cpp`: KEEP — 확인 1–233; 남은 줄 없음. WaypointVelocityTarget(150)은 위치에서 제한 속도를 반환하고 MotorTargets(170)는 상태·제어·물성에서 자세 토크와 로터 속도를 계산한다. 상태는 소유하지 않으며 로터 연립식이 특이하면 예외(127). 강체 제어와 운동학의 목표 동작 차이는 유지 여부 확인.
- `modules/simulation_core/src/QuadrotorModel.cpp`: KEEP — 확인 1–67; 남은 줄 없음. X500Gazebo(7)는 기체·로터 관성 및 9개 충돌 상자를 값으로 구성하고 X500MotorTableEstimate(56)는 계수 세 개를 추정값으로 대체한다. 가변 상태·오류 처리 없음. 별도 추정값이 필요한지 소비자 확인.
- `modules/scenario/src/ScenarioDefinition.cpp`: SIMPLIFY — 확인 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1329; 남은 줄 없음. ReadScenarioDefinition(1254)은 JSON을 검증해 값 소유 정의를 반환하고 잘못된 입력은 예외. GenerateSurveyTasks(610)는 드론별 영역과 왕복 경로를 생성; 앱 임무 생성과 중복 여부 확인. 시설 점검·재시도 및 통신 장애·물성 변경 설정은 범위 정리 후보이며 필수 조사 경로와 구분.
- `modules/scenario/src/ScenarioDefinition.h`: SIMPLIFY — 확인 1–143; 남은 줄 없음. 시나리오 정의는 드론·예약 명령·통신 장애·편대·임무 등을 vector/optional 값으로 소유하고 파서 결과를 실행기에 전달한다(104–141). 고급 재할당 관련 결과 모델과 확장 설정은 소비자 확인 후 축소 후보.
- `modules/scenario/src/ScenarioRunner.cpp`: SIMPLIFY — RunScenario는 시나리오 정의를 읽고 관측, 명령, 에너지, 임무, 편대, 회피와 Tick을 진행한다. 임무 완료는 경유점 도착으로 확인한다. 자동 촬영 판정·재시도와 통신·배터리 조건의 임무 재배정은 2026-10-29 제거했다. 통신 단절 작업은 원래 담당 드론에 남아 미확정으로 끝나며, 저전력 귀환은 해당 작업을 미완료 실패로 끝낸다. 앱 임무와 기능이 겹치는지는 별도 검토가 필요하다.
- `modules/simulation_core/src/SimulationCore.cpp`: KEEP — 확인 1–250, 251–500, 501–750, 751–1000, 1001–1179; 남은 줄 없음. EcsStorage(37)는 Flecs 엔티티와 물리 월드를 소유한다. 배터리는 설정된 드론 엔티티의 선택형 `BatteryModel` 컴포넌트다. AdvanceOneTick(895)은 ECS 진행 후 물리, 배터리, tick, 카메라 자세를 갱신하고 일시정지 시 반환. 생성 실패 시 물리 강체와 엔티티 정리(407). 운동학과 강체의 접촉 처리는 서로 다른 구현이며 플랫폼 요구 확인 후 축소 판단.

## Simulation Core의 실제 책임과 경계

- `SimulationCore`는 Flecs 세계와 드론 엔티티를 만들고, 명령 적용 및 단일 1/60초 틱을 외부에 제공한다. Flecs 컴포넌트는 별도 타입을 많이 두기보다 드론 상태·제어 목표·카메라 상태를 엔티티에 붙인다. 배터리도 기본 생성자를 제공한 `BatteryModel` 컴포넌트로 옮겼다. 식별자→엔티티 맵은 컴포넌트 데이터 복제가 아니라 외부의 연속 `DroneId`를 Flecs 엔티티로 찾는 색인이라 유지한다. 지형, 바람, 충돌 장애물과 계측값은 코어 소유 자료다. 바람은 모든 드론에 같은 환경값이므로 `DroneControlState`에 반복 저장하지 않는다. 운동학 모드에서는 코어가, 강체 모드에서는 `PhysicsWorld`가 각자 바람 값을 보유한다. 모든 시뮬레이션 데이터를 ECS로 표현하지는 않는다.
- Flecs 시스템은 운동학 모드에서 `DroneState`와 `DroneControlState`를 순회해 운동학 계산과 접촉 처리를 한다. 카메라 시스템은 `CameraState`를 갱신한다. 강체 모드에서는 Flecs 시스템이 목표 로터 속도를 AirSim의 물리 월드에 전달하고, `AdvanceOneTick()` 뒤 물리 월드를 한 번 진행해 결과를 각 드론 상태에 복사한다. 현재 구조는 ECS 시스템이 전체 물리 적분을 맡는 형태가 아니라, ECS가 드론 상태·제어를 보유하고 별도 `PhysicsWorld`가 강체를 소유하는 혼합 구조다.
- `AdvanceOneTick()`의 순서는 일시정지 확인 → Flecs 진행(제어·운동학/로터 입력·카메라 tick) → 강체 모드의 AirSim 적분과 상태 동기화 → 배터리 갱신 → Tick 증가 → 카메라 자세 동기화다. 배터리는 물리 갱신 뒤 `BatteryModel`과 `DroneState`가 함께 붙은 엔티티만 Flecs 조회로 순회한다. 이 순서는 앱·헤드리스 양쪽에서 공유된다.
- 화면 장면과 렌더링은 Simulation Core에 있지 않다. `TrainingRangeScene`과 그래픽 모듈이 장면 자산·그리기 데이터를 다루고, 앱이 시뮬레이션 상태를 렌더러 입력으로 넘긴다. 상세 모듈 책임·파일 배치·장면 관리 조사는 사용자 요청에 따라 다음 단계의 별도 구조 감사에서 한다.
- 공개 API는 명령·드론 생성/삭제·배터리·카메라·바람·로터 고장·물리 시간 계측·장애물·지형·시간을 한 `SimulationCore`에 모아 둔다. 사용처가 없던 Flecs 시스템별 시간, 전체 Flecs 진행 시간, 접촉 자료 생성 시간 측정 API는 2026-10-29 제거했다. 앱에서 보여 주는 AirSim 물리 단계 시간은 유지한다.
- `Dynamics()`, `IsPaused()`, `SetRotorMotorFault()`와 `IsRotorMotorFaulted()`는 저장소 호출처가 없어 SimulationCore 공개 API에서 제거했다. 실제 로터 고장 동작과 직접 시험은 `PhysicsWorld`에 남겨 두었다. 모드 선택은 생성자에 남아 있고, 일시정지는 `Pause()`·`Resume()` 동작으로 제어한다.
- `AddDrone`의 두 인자 오버로드는 지면 높이 0을 넣어 세 인자 오버로드를 다시 호출하던 얇은 중복이었다. 기본 지면 높이 인자를 둔 한 함수로 합쳤다. 인자 세 개를 넘겨 실제 지면 높이를 지정하는 호출도 유지된다.
- 강체 모드의 지형은 `PhysicsWorld`가 접촉 계산에 사용한다. `SimulationCore`에도 같은 격자를 복사해 두고 있었지만 강체 경로에서는 읽지 않았다. 이제 강체 모드에서는 소유권을 물리 월드로 넘기고, 운동학 모드에서만 코어가 지형을 보관한다. 따라서 강체 모드의 불필요한 격자 사본 하나와 복사 한 번을 없앴다.
- `ScenarioRunner`는 Simulation Core 틱을 반복 호출하는 별도 JSON 자동 시험 실행기다. JSON 해석은 `ScenarioDefinition`이 맡고 Runner는 명령·임무·관측·결과를 조합한다. 라이브 앱의 임무 코드는 `AeroSimLabAppMission.cpp`에 따로 있으므로 두 경로의 임무 기능이 일부 겹쳐도 같은 호출 경로라고 볼 수 없다. 중복을 줄이려면 같은 M01–M03 규칙이 실제로 양쪽에 중복 구현됐는지 추가 비교해야 한다.
- 2026-10-30 변경: JSON을 구조체로 바꾸는 단계와, 완성된 설정끼리 맞는지 확인하는 단계의 경계를 분명히 했다. 자료형·필수 항목·배열 해석은 읽기 쪽에, 명령의 드론 참조·Tick 순서·통신 단절 구간·임무 중복과 설정 조합은 검증 쪽에 둔다. 전체 시나리오 시험으로 기존 오류 처리를 확인했다.
- 2026-10-30 변경: JSON 임무 입력과 실행 결과를 분리했다. 파서는 이름·담당 드론·경유점·기한만 읽고, 실행기에서 별도의 진행 자료를 만들어 완료·실패 상태를 기록한다. 따라서 파서가 실행 결과 헤더를 끌어오지 않는다.
- `modules/terrain/include/terrain/MapBridges.h`: SIMPLIFY — 확인 1–46; 남은 줄 없음. GeoJSON 경로·지형을 받아 교량 외곽 값을 반환하고 교각을 추정한다(33–45). 반환 vector가 형상을 소유. 교량 점검이 MVP 범위 밖이면 시각화·충돌 호출처를 확인 후 제거 후보; 오류 세부는 구현 미확인.
- `modules/terrain/include/terrain/MapBuildings.h`: KEEP — 확인 1–87; 남은 줄 없음. 건물 외곽·높이와 출처를 값으로 소유하며 로딩·범위·충돌·외벽 계산 API를 제공한다. 경로·충돌용 건물은 유지, 시설 외벽 함수(76–85)는 사용처 확인 후 축소 후보. 로더 오류는 구현 미확인.
- `modules/terrain/include/terrain/MapFlightRoute.h`: SIMPLIFY — 확인 1–65; 남은 줄 없음. PlanMapFlightRoute(26)는 지형·장애물·시작·목표를 받아 경유점 또는 nullopt를 반환한다. 고정 V 및 조사·복귀 경로에 필요한 안전 경로는 유지하고 교량 하부·외벽 경로(43–64)는 시설 점검 범위 확인 후 축소 후보.
- `modules/terrain/include/terrain/MapRoads.h`: REMOVED — 현재 장면과 임무에서 사용하지 않는 Tancheon 지도용 도로 GeoJSON 로더다. 렌더링·충돌·경로에 연결되지 않아 제거했다.
- `modules/terrain/include/terrain/MapTrees.h`: REMOVED — 현재 나무는 장면의 GLB 배치로 그린다. 호출처가 없는 지도용 나무 로더를 제거했다.
- `modules/terrain/include/terrain/MapWaterBodies.h`: REMOVED — 현재 장면에서 수계 GeoJSON을 읽거나 물 경계를 판정하지 않는다. 호출처가 없는 지도 전용 코드를 제거했다.
- `modules/terrain/include/terrain/TerrainGrid.h`: KEEP — 확인 1–67; 남은 줄 없음. 높이 vector를 소유하며 로드·저장, 고도 보간, WGS84와 ENU 변환을 제공한다. 지형·경로·충돌의 공통 기준; 구현의 입력 오류 확인 필요.
- `modules/terrain/include/terrain/TrainingRangeTerrain.h`: KEEP — 확인 1–9; 남은 줄 없음. 500×400m 시험 지형을 값으로 반환하는 무인수 함수. 사용처 및 실제 범위는 구현 확인.
- `modules/terrain/src/MapBridges.cpp`: SIMPLIFY — 확인 1–200, 201–215; 남은 줄 없음. LoadMapBridgeFootprints(99)는 GeoJSON과 지형에서 이름 있는 교량을 값으로 만들고 파일·JSON·features 오류는 예외, 부적합 개별 형상은 건너뛴다. EstimateMapBridgePiers(150)는 간격 기준으로 추정 교각을 반환한다. 시설 점검 확장과 표시·충돌의 공유 소비자를 확인 후 축소.
- `modules/terrain/src/MapBuildings.cpp`: KEEP — 확인 1–200, 201–336; 남은 줄 없음. LoadMapBuildings(68)는 GeoJSON과 지형을 읽고 높이 또는 층수를 적용하며 오류는 예외. ContainsMapBuildingVolume(175), FindFirstPointOutsideMapBuildings(204)는 건물 검사, GetMapBuildingFacades(245)는 점검면을 반환한다. GetMapBuildingBoundsMeters(151)는 비어 있지 않고 높이 배열이 대응하는 입력을 전제하므로 호출 계약 확인. 건물 회피는 유지하고 외벽 점검은 별도 축소 후보.
- `modules/terrain/src/MapFlightRoute.cpp`: SIMPLIFY — 확인 1–200, 201–400, 401–595; 남은 줄 없음. FindGridPath(152)는 5m 격자 8방향 A*와 대각선 코너 차단을 적용하고 SimplifyPath(238)는 선분 검사를 사용한다. PlanMapFlightRoute(281)는 입력 오류에 예외, 경로 없음에 nullopt를 반환하고 지형·건물로 경로를 생성한다. 높이는 경유점에서 확인하므로 긴 간소화 선분의 중간 지형 여유는 별도 검증 필요. 교량 하부·외벽 경로(430,544)는 시설 점검 사용처 확인 후 축소 후보.
- `modules/terrain/src/MapGeometry.h`: KEEP — 확인 1–164; 남은 줄 없음. ContainsPointInRing(21), ClipPolygonToTerrain(44), ReadTerrainFootprint(108)가 지도 형상 포함·경계 절단·좌표 변환을 수행한다. 결과 vector는 값 소유 또는 호출자 출력 참조, 부적합 형상은 false. 충돌·EO의 다른 경계 판정과 목적이 달라 무조건 합치지 않는다.
- `modules/terrain/src/MapRoads.cpp`: REMOVED — 현재 장면과 임무에서 사용하지 않는 Tancheon 지도용 도로 GeoJSON 변환 코드다. 호출처와 자료가 없어 함께 제거했다.
- `modules/terrain/src/MapTrees.cpp`: REMOVED — 현재 장면은 GLB 나무 배치를 사용하며 이 지도 위치 변환기는 호출되지 않는다. 함께 제거했다.
- `modules/terrain/src/MapWaterBodies.cpp`: REMOVED — 현재 장면에 연결되지 않은 수계 GeoJSON 로더와 경계 판정 코드다. 함께 제거했다.
- `modules/terrain/src/TerrainGrid.cpp`: KEEP — 확인 1–200, 201–400, 401–430; 남은 줄 없음. Load/LoadAsciiGrid(115,185)는 파일 형식·수·유효 숫자를 검사해 지형 값을 반환하고 Save(282)는 파일을 쓴다. HeightAtLocalMeters(353)는 쌍선형, HeightAtRenderedSurfaceMeters(367)는 메시 삼각형 보간으로 값이 다를 수 있다. FindTerrainCell은 범위·구조 오류를 던진다. HeightAt(315)은 행·열만 검사하므로 직접 구성한 높이 vector의 길이 계약에 주의.
- `modules/terrain/src/TrainingRangeTerrain.cpp`: KEEP — 확인 1–53; 남은 줄 없음. CreateTrainingRangeTerrain(8)은 101×101 높이 vector에 언덕·능선 값을 채우고 최솟값을 0으로 조정해 반환한다. 외부 입력·상태 없음. 시험 지형 소비자를 확인해 실제 사용 범위에 맞춰 유지.
- `tests/simulation_core/CommandRecordingTests.cpp`: REMOVED — 명령 기록·재생 전용 시험이다. 제품 기능 삭제와 함께 삭제했으며, JSON 시나리오 시험은 유지한다.
- `tests/flight_planning/FormationPlannerTests.cpp`: 편대 계획 시험 — 4대 V 배치와 좁은 경로 거부를 확인한다. 경로·대형 알고리즘을 바꿀 때 이 모듈과 함께 검증한다.
- `tests/simulation_core/PhysicsWorldTests.cpp`: KEEP — 확인 1–250, 251–500, 501–750, 751–888; 남은 줄 없음. 모터·제어·좌표계·지면·장애물·드론 충돌과 범위 오류를 확인한다(44,274,489,511,582,744,782). AirSim 접촉과 수명 변경 시 중요한 회귀 범위이며 고장 주입 시험은 기능 범위 결정 후 정리. 실행하지 않음.
- `tests/simulation_core/QuadrotorModelTests.cpp`: KEEP — 확인 1–124; 남은 줄 없음. 제조사 표의 추정 계수와 X500 질량·관성·로터 배치(12,86)를 확인한다. 해당 모델을 유지할 때 물성 변경 검증에 필요. 실행하지 않음.
- `tests/scenario/ScenarioRunnerTests.cpp` (당시 `tests/simulation_core`): SIMPLIFY — 확인 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1500, 1501–1688; 남은 줄 없음. 시나리오·카메라·물성·통신·4대 분산 조사·충돌 회피·저배터리와 재할당을 검사한다(160,762,1000,1250,1568). 16–256대 확장 및 재할당 기대는 제거 후보; 4대 조사·통신 단절의 미확정 처리·복귀·충돌 안전 시험은 요구에 맞게 보존. 실행하지 않음.
- `tests/simulation_core/SimulationCoreTests.cpp`: KEEP — 기준선 시험은 배터리·고정 틱·운동학 장애물·지형·ECS 및 드론별 카메라 상태를 확인했다. 점검면 촬영 판정 시험은 기능 제거와 함께 삭제했다. 현재 시험 실행 결과는 `VERIFICATION.md`에 기록한다.
- `tests/terrain/TerrainGridTests.cpp`: KEEP — 확인 1–200, 201–284; 남은 줄 없음. 지형 크기·좌표·보간·파일 왕복과 오류 및 건물 우회·막힌 경로를 확인한다(26,40,75,89,150,203). 건물 경로 시험은 평지로 긴 경로의 중간 지형 장애를 확인하지 않는다. 외벽·교량 관련 기대는 기능 제거 시 함께 정리. 실행하지 않음.

## 시나리오 실행 경로 재검토 — 2026-10-29

이번 확인은 현재 작업 트리 기준이다. Graphify 자료는 `ScenarioRunner`를 이전 위치의 `simulation_core` 모듈로 가리키고 있어 근거로 사용하지 않았다. 파일 연결은 현재 CMake와 실제 호출처를 검색해 확인했다. Luna 사실 수집 에이전트는 세션 작업자 한도 때문에 시작되지 않아 이번 사실 수집은 현재 작업자가 직접 했다. Sol 판단은 사실 확인과 나눠 아래에 적었다. 빌드·시험은 실행하지 않았다.

### 파일별 사실 수집

| 파일 | 확인된 역할과 연결 | 구조상 확인 사항 |
|---|---|---|
| `modules/scenario/include/scenario/ScenarioRunner.h` (153줄) | `RunScenario(JSON 문자열)` 공개 함수와 최종 결과 형식을 선언한다. 결과에 드론·카메라·통신 상태·배터리·비행 거리·에너지·임무·궤적·충돌 간격이 함께 들어간다. | 결과 형식이 시나리오의 모든 선택 기능을 한 공개 자료형에 담는다. 카메라·에너지·임무·궤적 중 실제 목표에 남길 기능을 먼저 정해야 결과 계약도 줄일 수 있다. |
| `modules/scenario/src/ScenarioDefinition.h` (현재 내부 자료 구조) | JSON을 실행 가능한 값으로 읽는 내부 구조다. 드론 시작 상태, 명령, 직접 통신 단절, 충돌 회피, 배터리, 임무 작업, 물리·제어기 설정, 바람, 궤적, 카메라 이벤트와 기대 결과를 보관한다. | 통신 장애 뒤 별도 안전 동작은 없다. 끊기기 전 받은 경로를 계속 따르고 상태 보고만 끊는다. |
| `modules/scenario/src/ScenarioDefinition.cpp` (당시 1,027줄) | JSON 설정을 읽고 검증한다. 임무는 드론별 `tasks` 목록과 한 개 이상의 `waypoints_meters`만 받는다. 기대 결과도 이름이 있는 `expected.drones` 목록으로 읽는다. | 입력 파서는 여전히 물리 설정·배터리·카메라·통신·충돌·임무를 모두 다룬다. `survey_area` 자동 분할은 사용처가 시험 하나뿐이라 제거했다. 기대 결과의 단일 드론·다중 드론 문법을 하나로 통일했다. |
| `modules/scenario/src/ScenarioRunner.cpp` (당시 1,308줄) | 설정에서 `SimulationCore`를 만들고, 각 Tick마다 관측·명령·에너지 정책·임무 이동·충돌 회피를 처리한 뒤 기대 결과와 비교한다. 카메라 동작 시간표는 제거했다. | 시뮬레이션 실행, 임무, 에너지, 회피, 결과 판정이 한 파일에 모여 있다. 앱 임무와도 별도로 동작한다. 기대 결과는 드론 이름으로 검사한다. 임무 완료 수·진행률·전체 완료 Tick은 작업 상태에서 계산한다. |
| `apps/headless/main.cpp` (258줄) | `--scenario <JSON>` 파일을 최대 16 MiB까지 읽어 `RunScenario`에 넘기고 결과 JSON을 출력한다. 성공·실패·실행 오류에 서로 다른 종료 코드를 쓴다. | 얇은 CLI 경계다. 시뮬레이션 정책은 없고 결과 직렬화만 맡으므로 우선 축소 대상이 아니다. |
| `tests/scenario/ScenarioRunnerTests.cpp` (1,119줄) | 물성·제어기·바람·배터리·카메라·명령 전달·통신 단절·정찰 경로·충돌 회피·실패 조건을 검증한다. | GoogleTest `TEST` 하나가 많은 검증 함수를 순서대로 호출한다. 전체 CTest 수는 억제되지만, 이름으로 개별 검증만 실행할 수 없다. |

### 실제 호출과 시나리오 자료

- `RunScenario`의 제품 실행 호출자는 `apps/headless/main.cpp`다. 자동 시험은 `tests/scenario/ScenarioRunnerTests.cpp`에서 직접 부른다. 검색상 `apps/simulator`는 이 실행기를 부르지 않는다.
- 시뮬레이터의 M01·M02·M03 임무 흐름은 `apps/simulator/AeroSimLabAppMission.cpp`에서 별도로 시작·진행한다. 따라서 JSON Runner의 임무 시험이 화면에서 쓰는 임무 실행 코드를 검증하지 않는다.
- Runner의 통신 장애도 시뮬레이터가 쓰는 가상 통신망과 다른 코드다. 한쪽의 단절·복구 시험이 다른 쪽의 동작을 보증하지 않는다.
- 감사 뒤 `formation_links` 기능과 전용 사례를 제거했다. 현재 `scenarios/`에는 JSON 8개가 있고, CMake headless 연기 시험은 `rigid-body-flight`, `communication-outage`, `independent-flight`, `two-point-survey` 4개를 실행한다. `physical-model-override`, `four-drone-survey-outage`, `low-battery-return`은 GoogleTest에서 자료 파일로 읽는다. `normal-flight`는 코드나 CMake에서 참조를 찾지 못했다. 수동 예제로 쓰는지는 확인되지 않아 삭제 대상으로 단정하지 않는다.
- `survey_area`는 기존 자료 파일이나 앱에서 사용하지 않고 하나의 시험에서만 사용했다. 자동 영역 분할 코드를 제거하고 그 시험은 동일한 분할 결과를 `tasks`의 경유점으로 직접 적는다. 임무 실행과 완료율 계산은 계속 시험한다.
- 통신 장애의 `on_timeout` 옵션은 제자리 정지와 마지막 명령 지속 중 하나를 선택했다. 시나리오의 드론은 받은 경로를 계속 따르므로 이 선택지와 정지 코드를 제거했다. 상태가 끊기면 임무 완료 여부는 GCS에서 확인할 수 없게 둔다.
- 기대 결과는 예전에는 단일 드론일 때 이름 없는 `mode`·`position_meters`를 쓰고, 여러 대일 때만 `drones` 배열을 썼다. 파서와 결과 검사의 두 갈래 처리를 없애고 항상 이름이 포함된 `expected.drones` 배열을 받도록 통일했다. 저장소 시나리오와 시험 입력도 함께 바꿨다.
- JSON의 `camera_events` 시간표는 저장소 시나리오에서는 사용하지 않고 단위 시험에서 카메라 켜기·끄기·고장·복구를 예약하는 데만 쓰였다. 이번 범위의 기본 임무에는 필요하지 않아 파서·실행 처리와 전용 시험을 제거했다. 시나리오 결과의 카메라 상태·프레임 수 판정과 드론별 카메라 기능 자체는 유지했다.
- 충돌 회피는 두 드론의 목표 속도를 일정 시간 유지한다고 보고 상대 위치·속도로 최근접 거리를 구한다. 기존에는 각 드론이 경유점에 도착해 멈추는 시점까지 나눠 계산했지만, 간단한 훈련 시나리오의 목표에 비해 상태 추정 코드가 길어 이를 없앴다. 따라서 목표 지점에서 감속·정지하는 실제 궤적을 예측하지 않으며, 이 수치는 실제 비행 안전성의 증거가 아니다.
- 임무 완료 작업 수, 작업 완료율, 방문 경유점 비율과 전체 완료 Tick은 이미 각 작업의 완료·경유점 상태에서 구할 수 있었다. 실행 결과에 다시 저장하던 필드를 없애고 `Mission`의 계산 함수로 옮겼다. JSON 출력 항목 이름과 값은 유지한다.
- Tick마다 임무 명령을 줄 때 각 작업마다 이전 작업 목록을 다시 훑어 같은 드론의 순서를 확인하던 구조를 한 번의 목록 순회로 바꿨다. 드론마다 첫 미완료 작업만 선택해 처리한다. 기존 시험의 `duplicateMissionAssignment`는 실제로는 필수 경유점이 빠져 실패하고 있어 배정 규칙을 검증하지 못했다. 이를 두 작업을 한 드론이 차례로 완료하는 유효 시나리오로 교체했다.
- 충돌 회피에서는 진행 중인 임무를 찾는 같은 조건을 드론 쌍 판단부와 회피 명령 적용부가 각각 복사해 두고 있었다. 조건을 한 검색 함수로 모아 두 경로가 같은 작업 상태를 보도록 했다.
- 같은 활성 임무 검색을 드론 쌍마다 반복하던 부분도 회피 시작 때 드론별로 한 번만 계산하도록 바꿨다. 이후 쌍 계산은 저장한 연결 상태와 작업 포인터를 사용한다.

### Sol 판단

시나리오 쪽은 단순히 파일이 큰 문제가 아니다. **인터랙티브 앱 임무와 JSON Runner가 별도 임무·통신 처리를 갖고 있고, 시험 이름 하나에 여러 검증이 묶여 있다.** 그래서 현재 자동 시험이 통과해도 앱의 네 대 임무와 통신 차단 동작이 맞다고 말할 수 없다.

우선 보존할 가치는 `SimulationCore`를 반복 가능한 입력으로 실행해 강체 물리·기체 설정·제어기·고정 Tick을 점검하는 기능이다. 축소할 때는 다음 순서가 안전하다.

1. 제품 목표에서 남긴 정상 4대 임무와 직접 링크 차단/복구에 필요한 JSON 항목만 따로 표시한다.
2. 그 항목이 앱과 Runner에서 각각 어떤 구현을 타는지 구분한다. 공통 코드로 묶을 수 있는지는 실제 같은 규칙인지 확인한 뒤 판단한다.
3. 카메라 이벤트가 기본 임무에 필요한지 따로 판정한다. 저배터리 귀환은 현재 프로젝트 범위에 배터리 모델이 남아 있으므로 즉시 제거하지 않는다. 범위에서 제외하기로 한 항목은 관련 JSON·검증·출력까지 함께 정리하고, 물리 검증용 설정은 보존한다.
4. 테스트는 개수를 늘리지 않고 현재 `CoreScenarios`를 물리·임무·통신 핵심 묶음으로 나눠 실패 지점을 알아볼 수 있게 한다.

`ScenarioRunner`와 앱 임무 코드를 지금 한 엔진으로 합치는 것은 권하지 않는다. Runner의 통신 모델은 앱과 다르고, 앱 경로는 실시간 입력·UI·실제 가상망 상태를 사용한다. 먼저 공통으로 검증할 임무 규칙을 정해야 불필요한 범용 계층 없이 중복을 줄일 수 있다.

## 리더 추종 링크 모의 제거 — 2026-10-29

- JSON의 `formation_links`는 leader 위치에 오프셋을 더해 follower에게 매 Tick 새 경유점을 주고, 별도 링크 단절 구간을 적용했다. 시뮬레이터 앱이나 다른 시나리오가 이 기능을 부르지 않았고, 직접 통신 단절 모의도 따로 있었다.
- 정의 구조, JSON 파서·검증, Tick 처리, 전용 시험, headless 연기 시험과 `leader-following-outage.json`을 함께 제거했다. 총 268줄을 덜었다. 직접 상태 보고 장애(`communication_outages`)와 앱의 V자 편대 경로는 변경하지 않았다.
- Release의 `aerosimlab_headless`, `aerosimlab_scenario_tests` 타깃 빌드 성공. 전체 CTest 28/28 통과. 시나리오와 앱 임무가 별도 구현이라는 기존 경계는 그대로라서, 앱 임무 동작이 이 시험으로 검증된다고 볼 수 없다.

## 시나리오 명령 전달 모형 축소 — 2026-10-29

- 기존 JSON 명령은 `delay_ticks`, `drop`, `duplicate`를 받아 Runner 안에서 전달 시각을 다시 만들고, 순번 검사를 한 뒤 적용·유실·중복 결과를 별도로 기록했다. 이는 직접 통신 상태를 모의하는 `communication_outages`와 별개의 패킷 전달 구현이었다.
- 명령은 지정된 Tick에 한 번 적용하는 구조로 줄였다. Runner의 순번 맵과 지연·유실·중복 처리 함수를 없앴고, `independent-flight.json`도 즉시 전달 규칙을 사용한다.
- 이전 fault-injection fixture는 제거된 명령 필드에 기대고 있었다. 이를 두 드론의 직접 통신 단절 사례로 바꾸고 CTest 이름도 실제 범위를 나타내도록 고쳤다. stale 상태와 마지막 보고 위치가 유지되는 점을 확인한다.
- 첫 검증은 예상 비행 시간과 다중 드론 `expected.drones` 누락으로 실패했다. 입력과 기대값을 현재 Tick 규칙에 맞춘 뒤 관련 시험 2/2, 전체 CTest 28/28 통과.
- 변경은 JSON Runner에 한정된다. 앱 가상 통신망과 명령 전송 코드는 건드리지 않았으며, 시나리오 통과가 앱 통신의 동일 동작을 보증하지 않는다.

## 시나리오 결과 성공 여부 중복 제거 — 2026-10-29

- `ScenarioResult`와 임무 결과는 이미 `Passed`, `Failed`, `Inconclusive` 상태를 저장하면서 같은 의미의 `passed` 불리언도 따로 저장했다. 두 값이 달라질 수 있어 상태 불변식이 중복되어 있었다.
- 저장 필드를 제거하고 `IsPassed()`가 결과 열거형에서 값을 계산하게 했다. headless JSON의 `passed` 출력은 유지해 실행 결과 형식은 바뀌지 않는다.
- `aerosimlab_headless`와 시나리오 시험 타깃 빌드 성공, 전체 CTest 28/28 통과.

## 시나리오 목표 좌표 중복 제거 — 2026-10-29

- 각 임무 결과는 `waypointsMeters`를 보관하면서 첫 경유점과 같은 목표 위치도 별도 필드에 저장했다. 두 값이 어긋날 수 있어 JSON 출력이나 화면 표시에 서로 다른 목표가 나타날 여지가 있었다.
- 목표 위치 필드를 없애고, 단일 지점 임무는 입력한 목표를 경유점 목록에 넣으며 구역 임무는 생성한 경로를 그대로 보관하도록 했다. headless 출력은 경유점 첫 항목에서 목표 좌표를 만들고 기존 JSON 키를 유지한다.
- 경로 생성과 JSON 파싱은 경유점이 적어도 하나 있도록 보장한다. 빈 경로 입력은 기존 검증에서 거부되며, 이 불변식이 있는 경로에서만 첫 항목을 읽는다.
- 확인은 `VERIFICATION.md`의 Windows Release 전체 CTest 기록을 따른다.

## 최상위 기본 드론 상태 중복 제거 — 2026-10-30

- `ScenarioResult`는 드론별 마지막 상태를 이미 목록에 보관하면서 첫 드론의 비행 상태와 카메라 상태도 최상위에 복사하고 있었다.
- 복사 필드를 없애고 기대 카메라 판정과 Headless JSON 직렬화가 첫 드론 항목을 읽게 했다. JSON의 기존 최상위 항목은 유지한다.
- 기본 드론만 있는 시나리오와 이름을 지정한 여러 드론 시나리오 모두 결과 목록에 첫 항목이 생기는 흐름을 확인했다. 전체 CTest와 출력 비교는 `VERIFICATION.md`에 기록했다.

## 임무 완료 상태 중복 제거 — 2026-10-30

- `MissionTask::completed`는 참이 될 때마다 같은 Tick을 `completedTick`에 기록했다. 완료 시각이 없는 미완료 임무는 불리언도 거짓이므로, 두 필드가 완료 상태를 중복 표현하고 있었다.
- 불리언을 없애고 `IsCompleted()`가 `completedTick`의 유무를 반환하게 했다. 완료 판정, 결과 집계, JSON 출력과 시험은 이 함수를 사용한다. JSON의 참·거짓 항목은 유지한다.
- 기존 완료·실패·통신 단절 임무 시험이 Windows Release 전체 CTest에서 통과했다. 세부 결과는 `VERIFICATION.md`를 본다.

## 시나리오 전체 완료 시각과 작업별 완료 시각 구분 — 2026-10-30

- 집계 코드는 완료한 각 작업의 가장 늦은 시각을 전체 임무의 `completedTick`으로 기록했다. 일부 작업만 끝나도 전체 임무가 끝난 것처럼 보여 결과 해석이 틀릴 수 있었다.
- 작업 완료 수, 경유점 진행률, 실패 여부를 같은 반복문에서 모으도록 정리했다. 개별 작업의 시각은 계속 보관하되 전체 `completedTick`은 모든 작업이 완료됐을 때만 정한다.
- 기존 직접 통신 장애 시험이 3/4 완료 상태에서 전체 완료 시각을 비우는지 확인한다. 검증 조건은 `VERIFICATION.md`에 기록했다.

## 시나리오 드론 시작 상태 필드 통합 — 2026-10-30

- `ScenarioDrone`은 하나의 시작 상태를 위치·방향·속도·자세·모터 속도·비행 모드 필드로 따로 저장했다. 실행기는 다시 `DroneState`를 만들어 Simulation Core에 전달했다.
- `ScenarioDrone`이 `DroneState`를 직접 보관하도록 바꿨다. 위치와 방향 외의 초기 상태가 입력됐는지 나타내는 표시는 운동학 모드에서 강체 전용 입력을 거부하는 데 필요해 남겼다.
- 이 변경은 JSON 필드와 상태 기본값을 바꾸지 않는다. 강체 상태 재현, 설정 오류 검사와 전체 CTest를 확인했다. 자세한 결과는 `VERIFICATION.md`에 있다.

## 임무 결과 상태 중복 제거 — 2026-10-30

- `Mission::outcome`은 작업별 `completedTick`과 `failed`에서 정해지므로 별도로 저장할 필요가 없었다.
- `GetOutcome()`이 작업 목록을 순회해 실패를 우선 판정하고, 모두 끝났을 때 성공, 나머지는 미확정으로 돌려준다. `ScenarioResult::outcome`은 예상 조건 실패까지 담으므로 그대로 둔다.
- JSON 응답 이름과 기존 상태 결과를 유지했다. 빌드·시험 결과는 `VERIFICATION.md`에 기록했다.

## 시나리오 임무 입력 문법 통일 — 2026-10-30

- 임무 한 곳을 지정할 때 `target_position_meters`를 쓰고, 여러 곳을 지정할 때 `waypoints_meters`를 쓰는 두 입력 형식이 있었다. 실행기는 두 경우 모두 같은 경유점 목록으로 바꿔 처리했다.
- 모든 임무를 `waypoints_meters` 목록으로 읽도록 통일했다. 목록에 한 점만 넣으면 단일 목표와 같은 동작을 한다. 프로젝트의 시나리오 자료와 시험 입력을 새 형식으로 맞췄고, 예전 단일 목표 입력은 이제 거부되는지 확인한다.
- Headless 결과 JSON의 `target_position_meters` 항목은 사용하는 쪽과의 호환을 위해 그대로 둔다. 이 변경은 시나리오 입력 형식에만 적용했다.
- 결과: 파서와 시나리오 시험을 빌드하고 전체 CTest 28/28 통과. 환경과 명령은 `VERIFICATION.md`에 기록했다.

## 시나리오 실행기의 드론별 상태 정리 — 2026-10-30

- `RunScenario`는 드론 ID, 귀환 위치, 마지막 관측값, 이전 위치, 비행 거리와 시간을 이름 기준의 여섯 맵에 나눠 보관했다. 각 동작에서 같은 드론을 여러 맵으로 찾아 맞춰야 했다.
- 이를 `ScenarioDroneRuntime` 한 항목에 모았다. 함수에는 드론 상태 표 하나를 전달하고, 실제 비행 상태는 `id`, 관제에서 마지막으로 받은 값은 `observation`으로 구분한다. 시나리오 입력·출력과 Tick 순서는 바꾸지 않았다.
- 이 정리는 실행기 안의 임무·통신·배터리·충돌 회피 책임을 각각 분리한 것은 아니다. 큰 책임 분리는 동작 경계와 중복 구현을 더 확인한 뒤 별도로 판단한다.
- Windows Release 전체 CTest 28/28 통과. 세부 명령과 빌드 결과는 `VERIFICATION.md`에 기록한다.

## 시나리오 입력 흐름 정리 — 2026-10-30

- `ScenarioParser.cpp`는 JSON 문법·자료형·필수 항목을 확인해 실행 설정으로 바꾼다. 숫자가 유한한지와 정수 변환이 안전한지도 파싱 경계에서 확인한다.
- `ScenarioValidation.cpp`는 카메라·물리·배터리 값의 허용 범위와 명령의 드론 참조, 통신 단절 구간 겹침, 물리 방식 조합, 착륙에 필요한 배터리 예비량을 확인한다. 시나리오 실행 조건이므로 임의로 없애지 않았다.
- `ScenarioRunner.cpp`가 파싱과 검증을 차례로 호출한 뒤 Tick을 실행한다. 이전에는 파서 끝에서 검증기를 암묵적으로 불러 두 단계가 눈에 띄지 않았다. 함수와 소스 이름을 분명히 하고 실행 순서만 드러냈으며 검증 조건은 바꾸지 않았다.
- Windows Release에서 headless 실행기와 시나리오 시험 빌드 성공, 전체 CTest 28/28 통과. 정상 두 지점 임무 결과도 `passed`로 확인했다.
- 다음 구조 검토 대상은 실행기 안의 임무 명령, 에너지 귀환, 통신 관측, 예측 충돌 회피가 한 Tick에서 맡는 책임이다. Tick 처리 순서를 유지하면서 실제로 줄일 수 있는 중복부터 살핀다.

## 시나리오 충돌 회피 상태 축소 — 2026-10-30

- 드론별 회피 방향을 별도 표에 저장하고, 그 표를 회피가 끝날 때 정리하던 상태를 없앴다. 이동 방향은 매 Tick의 예측 경로에서 구하고 두 드론이 모두 임무 중이면 서로 반대쪽으로 비켜 간다.
- 드론 한 대만 회피해야 하는 경우에는 그 드론의 우회 목표만 유지한다. 회피 목표를 기억하는 상태와 회피 중인 드론 목록은 계속 필요해 남겼다.
- 한 대만 양보하도록 더 줄인 시도에서는 강체 비행 시험의 최소 간격이 설정값 8m보다 작아졌다. 그 방식은 채택하지 않았다. 양쪽 드론의 회피는 유지하고 방향 저장만 걷어낸 뒤, 강체·교차 경로 시험을 포함한 전체 CTest 28개가 통과하는 것을 확인했다.
- 이 예측 회피는 시나리오 시험용 간단한 모형이며 실제 비행 안전 보증으로 볼 수 없다.

## 시나리오 임무 처리의 중복 확인 제거 — 2026-10-30

- 임무를 읽을 때 배정 드론 이름은 이미 등록된 이름인지 검사한다. 실행 단계에서 빈 이름을 다시 확인하던 분기를 제거했다.
- 충돌 회피 속도 계산에서 한 번만 호출하던 경유점 선택 함수를 계산 위치에 합쳤다. 경유점과 기존 우회 목표 중 어느 쪽을 쓸지는 원래 조건 그대로 유지했다.
- Windows Release 실행기와 시나리오 시험을 빌드했고 전체 CTest 28개가 통과했다.

## 임무 작업 선택 분기 정리 — 2026-10-30

- 같은 Tick에서 임무 명령 처리는 작업 목록을 한 번 훑고, 충돌 회피는 드론마다 현재 진행 중인 작업을 다시 찾는다. 두 경로는 같은 포인터를 바로 공유하지 않는다. 임무 명령 처리에는 이륙 대기, 실패 후 다음 작업 진행, 통신 단절 조건이 있어 검색 단계를 억지로 합치면 상태 전달이 더 복잡해진다.
- 두 드론이 모두 연결되어 각자 임무 중일 때 충돌 회피는 양쪽을 함께 비켜 가게 한다. 이 경우 실제로 쓰지 않는 단일 양보 드론 포인터를 설정하던 분기를 없앴다. 한쪽만 움직이거나 상대가 통신 두절 상태일 때는 기존 선택을 유지했다.
- Windows Release 전체 CTest 28/28 통과. 임무 목록 순서와 통신 단절 동작은 바꾸지 않았다.

## 충돌 회피 상태 한곳에 모으기 — 2026-10-30

- `dronesAvoiding` 집합과 회피 목표 표는 Tick이 끝날 때 같은 드론 목록을 담고 있었다. 회피 중 여부는 목표 표의 항목 존재로 판정하고, 새 회피와 회피 종료 알림도 같은 표에서 확인하도록 했다.
- 회피 목표를 새로 넣었는지 여부로 진입 알림을 한 번만 기록한다. 두 상태 표가 서로 어긋날 가능성과 매 Tick 복사·대입을 없앴다.
- Windows Release 전체 CTest 28/28 통과. 충돌 회피 강체 시험의 최소 간격 조건과 임무 진입·종료 알림을 포함한 기존 검증을 유지했다.

## 시나리오 읽기와 설정 조합 검사 분리 — 2026-10-30

- JSON 항목을 읽는 동안 다른 항목의 값까지 검사하던 규칙을 분리했다. JSON 모양·숫자 범위·이름처럼 한 항목 자체가 잘못된 경우는 해당 값을 읽을 때 확인한다. 비행 방식과 물리 설정의 조합, 여러 드론을 전제로 한 옵션, 배터리 예비량처럼 전체 설정을 비교해야 하는 규칙은 읽기가 끝난 뒤 `ValidateScenarioDefinition`에서 확인한다.
- 궤적 기록 크기 제한, 강체 모드 전용 설정, 다중 드론 옵션, 배터리 착륙 예비량은 임의의 방어 분기가 아니라 현재 실행 코드의 한계와 입력 조건을 보호하므로 유지했다.
- 새 모듈은 만들지 않았다. 입력 해석과 설정 간 검증 순서가 별도 단계로 보이게 정리했다.
- Windows Release 전체 CTest 28/28 통과. 실제 JSON 형식과 실행 결과는 유지했으며 잘못된 설정은 실행 전에 거부한다.

## 기본 드론 처리 경로 통합 — 2026-10-30

- `drones` 입력을 생략하면 실행기 여러 곳에서 기본 드론 이름을 따로 처리했다. 파서가 기본 드론을 목록에 넣고, SimulationCore의 기존 기본 기체를 재사용하는지 표시하도록 바꿨다.
- 관측 갱신, 배터리 설정, 궤적·최종 결과 기록은 이제 같은 드론 목록을 순회한다. JSON의 생략 방식과 기본 이름은 그대로 지원한다.
- 궤적 자료 크기 제한도 파싱 도중과 설정 검증에서 중복 검사하던 것을 검증 단계 한 곳에 남겼다.
- Windows Release 전체 CTest 28/28 통과. 시뮬레이터 앱은 변경하지 않았다.

## Simulation Core 파일별 재감사 — 2026-10-30

기준 리비전은 `3b64c7c8`이다. 현재 저장소의 공개 헤더와 소스 17개, 관련 시험, CMake 등록 및 직접 호출부를 읽었다. 아래 내용은 코드 정적 조사다. 이번 조사에서는 빌드·시험·성능 측정을 실행하지 않았다. 좌표 표기는 별도 언급이 없으면 월드 위치 ENU(동·북·상), 기체 축 FLU(앞·왼쪽·위), 길이 m, 시간 s다.

### 실행 흐름과 핵심 판단

`SimulationCore::AdvanceOneTick()`이 `EcsStorage::AdvanceOneTick()`을 호출한다. 이 저장소는 운동학 모드에서 Flecs 시스템을 실행해 `DroneModel::AdvanceOneTick()`으로 상태를 전진시키고, 강체 모드에서는 비행 상태를 `QuadrotorFlightController::MotorTargets()`에 전달해 모터 목표를 만든다. 이어 `PhysicsWorld::Step(1/60)`이 AirSim 물리를 진행하고, 물리 상태를 ECS의 `DroneState`에 복사한다. 그 뒤 배터리, Tick, 카메라 자세를 갱신한다. 명령도 `SimulationCore::Apply()`에서 기본 입력을 확인한 다음 `DroneModel::Apply()`에 위임한다.

따라서 “SimulationCore가 수학 계산까지 전부 직접 한다”는 판단은 맞지 않는다. 비행 이동 계산은 `DroneModel`, 강체 자세·추력 제어는 `QuadrotorFlightController`, 강체 적분과 좌표 변환은 `PhysicsWorld`, SAT·다각형 투영은 `CollisionGeometry`가 맡는다. 다만 `SimulationCore.cpp`의 내부 저장소가 ECS 수명·물리 세계·두 비행 모드 연결뿐 아니라 운동학 모드의 지형/건물 접촉 보정과 속도 제거까지 직접 수행한다. 특히 한 파일에 통합·접촉 정책이 함께 있어 역할을 좁힐 여지는 확인된다.

### 파일별 책임과 적합성

| 파일 | 실제 역할·계산·데이터 소유 | 입력 검증·책임 적합성·시험 |
|---|---|---|
| `include/simulation_core/BatteryModel.h` | 배터리 용량, 초기·잔여 Wh, 평균 비행/호버 소비율과 저전력 복귀 표시를 소유한다. | 공개 설정 자료형과 모델 API라 위치가 맞다. 값 범위 검사는 생성자에 있다. 직접 시험은 `SimulationCoreTests.cpp`의 소비·초기값 오류 검사. |
| `src/BatteryModel.cpp` | 비행 모드별 평균 W를 고정 60Hz 틱의 Wh로 바꾸어 잔량을 감소시키고 0에서 제한한다. 실측 전압·전류 모델이 아니라 명시된 단순 추정이다. | 배터리 계산만 맡아 책임이 선명하다. 설정 범위 오류를 거부한다. `InitialWattHours()`는 현재 코드 검색상 시험에서만 쓰인다. 시험은 위 소비/리셋 케이스. |
| `include/simulation_core/DroneModel.h` | `DroneState`, `DroneCommand`, `DroneControlState`, 비행 모드와 `Apply`/`AdvanceOneTick` 계약을 정의한다. | 기본 상태·명령 타입이므로 공개 헤더 위치가 맞다. 위치는 ENU, yaw/자세는 라디안·쿼터니언, 틱 간격 1/60초로 기록돼 있다. 직접 호출은 SimulationCore와 시험, 일부 상태는 다른 모듈의 인터페이스에도 쓰인다. |
| `src/DroneModel.cpp` | 명령을 상태/제어 목표로 바꾸고 운동학 모드 위치·속도·고도 및 이륙/착륙 상태를 전진시킨다. | 비행 상태 계산 책임은 적합하다. 숫자 유한성, 비행 상태와 명령 조합을 확인한다. 제어/지면 접촉 동작은 `SimulationCoreTests.cpp`에서 시험한다. |
| `include/simulation_core/EoCameraModel.h` | 카메라 모델·상태와 장착/광축/위쪽 축 계산의 공개 API를 정의한다. | 센서 상태 자료와 API로 위치가 적절하다. 프레임 파일 저장 기능은 현재 책임이 아니다. 카메라 상태와 프레임 진행은 SimulationCore 시험 및 렌더러 직접 호출에서 확인된다. |
| `src/EoCameraModel.cpp` | 고정 기체 장착 위치·방향(기체 기준 m), 드론 쿼터니언으로 월드 자세를 계산하고 켜기/고장/복구 상태를 갱신한다. | 카메라 기하/상태만 담당해 모듈명과 맞는다. 모델 수치 유효성 검사는 `CameraModel::IsValid()`에서 한다. 독립 전용 시험 파일은 없고 SimulationCore 카메라 시험 및 앱 렌더 경로가 호출한다. |
| `include/simulation_core/PhysicsWorld.h` | 강체 핸들, 위치·자세·속도 상태, 모터/바람/지형/장애물/틱 API를 공개한다. | SimulationCore가 PhysicsWorld 구현을 직접 노출하지 않도록 인터페이스 경계 역할을 한다. 입력 오류 계약과 복사 금지는 선언에 있다. 전용 시험은 `PhysicsWorldTests.cpp`. |
| `src/PhysicsWorld.cpp` | AirSim API·기체 파라미터 어댑터, ENU/FLU↔NED/FRD 변환, 강체 수명, 모터 지연·고장, 충돌 후보와 AirSim 접촉, 지형 및 Tick을 구현한다. AirSim 객체는 `unique_ptr`가 소유되고 엔진에는 주소를 빌려준다. | 물리 연동 역할은 맞지만 어댑터·수명·좌표 변환·환경 충돌·모터 상태가 한 구현 파일에 모여 있다. 좌표·float 표현 가능성 확인은 AirSim 경계에 필요한 검사다. 1,256줄 규모지만 각 계산은 AirSim 연결 순서에 얽혀 있으므로 단순히 파일 분할부터 하는 것은 이득이 불확실하다. 광범위한 동역학·좌표·접촉 시험은 `PhysicsWorldTests.cpp`. |
| `include/simulation_core/QuadrotorFlightController.h` | 목표 경유점 속도와 4개 로터 목표를 계산하는 무상태 함수 및 제어 상수를 선언한다. | 비행 제어 API로 적합하다. 고정 쿼드로터 형식임이 API에서 드러난다. 시험은 `PhysicsWorldTests.cpp`의 직접 제어 입력/목표 확인과 SimulationCore 강체 케이스. |
| `src/QuadrotorFlightController.cpp` | 위치·속도 오차로 가속도를 제한하고, 추력 방향/헤딩으로 목표 자세를 만든다. 쿼터니언 자세 오차와 각속도 피드백으로 토크를 산출하고, 4×4 피벗 소거로 로터 추력을 구해 추력 계수로 RPM으로 바꾼다. | 수계산은 이 전용 파일에 있다. 책임 이름과 맞고 SimulationCore에 중복 구현된 제어식은 확인되지 않았다. 특이 연립식은 오류 처리한다. 수식·축 기준 설명을 보강할 여지는 있으나 core로 옮길 근거는 없다. 시험은 제어 입력, 이득, 관성/토크 반응 시험. |
| `include/simulation_core/QuadrotorModel.h` | 기체 질량·관성·로터 배치·모터 계수·충돌 상자를 값으로 보유하는 파라미터 타입과 X500 생성 API다. | 현재 X500 시뮬레이터 범위의 모델 정의로 역할이 명확하다. 충돌 모양까지 파라미터에 들어가는 것은 PhysicsWorld 생성에 필요하다. 시험은 `QuadrotorModelTests.cpp`. |
| `src/QuadrotorModel.cpp` | 공개 X500/Gazebo 기준값으로 관성·로터·충돌 상자를 구성하고, 별도 모터표 추정 설정을 만든다. | 기체 제원만 구성한다. 두 번째 추정 모델이 실제 시연 경로에서 필요한지는 앱 설정 호출과 요구 범위에 따라 판단할 후보이며, 코드만으로 dead code라 할 수 없다. 제원/배치/계수 시험 존재. |
| `include/simulation_core/ScopedDuration.h` | 범위 진입부터 종료까지 시간을 측정해 외부 double에 누적하는 RAII 도구다. 대상 포인터는 빌려 쓴다. | 계측 역할은 단순하다. 공개 헤더지만 앱 그래픽 코드도 사용하므로 core에 위치하는 것이 자연스러운지는 경계 검토 후보. null 대상은 측정 생략. 시험은 전용 CoreScenarios에 시간 성능 단언이 있는지 별도 확인 필요; 현재 사용은 PhysicsWorld와 앱. |
| `include/simulation_core/SimulationCore.h` | 외부가 사용하는 통합 API, 고정 틱 누산기, 드론/카메라/배터리/물리 환경 접근을 선언한다. ECS 내부 저장소는 `unique_ptr`로 감춘다. | 공개 조립 API로 위치가 맞으나 타입별 API가 넓다. `FixedTickAccumulator`는 틱 분수 상태와 프레임 경과시간을 보유하고 최대 0.1초를 소비한다. 시험은 전체 시뮬레이션 시험과 앱 세션 시험. |
| `src/SimulationCore.cpp` | `EcsStorage`가 Flecs 월드, 엔티티 ID 맵, 시스템, 선택형 배터리 컴포넌트와 카메라·물리 객체 연결을 맡는다. 운동학 충돌 자료와 접촉 보정은 `KinematicCollisionWorld`에 맡긴다. 공개 생성·초기화·명령·환경 API는 경계 입력을 확인한 뒤 적절한 모델이나 저장소를 호출한다. | 핵심 조립 지점이지만 긴 편이고, AirSim 강체 처리와 운동학 처리 사이의 플랫폼 분기가 많다. 제어·기하·적분 수식은 전용 코드에 있다. 유한 수치 검사는 서로 다른 외부 API 경계에 있어 전부 같은 중복으로 보지는 않는다. 접촉·틱·리셋·입력 오류·ECS/물리 동기화 시험은 `SimulationCoreTests.cpp`. |
| `include/simulation_core/CollisionGeometry.h` | 상자·외곽선·프리즘이 유효한지 검사하고 접촉 깊이·방향 또는 `optional` 결과를 계산하는 API다. span은 비소유다. | 강체·운동학 충돌 경로와 시험이 함께 쓰므로 공개 포함 경로에 뒀다. `collision` 네임스페이스로 용도를 드러낸다. |
| `src/CollisionGeometry.cpp` | 외접구/AABB broadphase, OBB SAT, footprint 투영, prism 높이 범위 접촉을 값으로 계산한다. | 충돌 기하 계산을 전용 파일에 둔 것은 적절하다. `FindPrismContact`→`ProjectToFootprint` 경로가 동일 외곽선 유효성 검사를 겹쳐 한다. `FindFootprintContact`도 매번 검사한다. 저장 시 검증된 기하를 재사용할 수 있는지는 호출 계약과 간결성 비교 후 판단할 최적화/단순화 후보다. 직접 시험은 SimulationCore/PhysicsWorld의 접촉 시나리오이며 별도 CollisionGeometry 시험은 확인되지 않았다. |

### ECS, 모듈 경계, 중복 후보

- Flecs는 실제로 사용된다. 위치/속도/자세/비행 모드와 제어 목표, 카메라, 선택 배터리를 컴포넌트로 만들고 시스템/반복문으로 갱신한다. 따라서 “Component를 전혀 쓰지 않는다”는 인상과 달리 드론의 반복 갱신 데이터는 컴포넌트다. 반면 정적 장애물·지형·AirSim 핸들 맵은 월드 자원 성격의 소유 데이터라 별도 저장한다. 이를 억지로 엔티티로 쪼개야 한다는 근거는 없다.
- 공개 헤더는 `include/simulation_core`, 구현은 `src`에 둔다. 공통 충돌 기하 헤더를 공개 포함 경로로 옮겼으므로 시험 타깃에만 붙던 `src` 직접 포함 경로도 제거했다. CMake 루트에서 공개 헤더 경로를 등록하며 `PhysicsWorld.cpp`와 해당 시험만 Emscripten 빌드에서 제외된다.
- 직접 호출자는 Simulator 앱/장면/HUD, Scenario, Manual Control, Communication, Ground Control 및 테스트다. 이 중 Scenario는 `QuadrotorFlightController::WaypointVelocityTarget()`을 직접 호출해 경유점 속도를 계산한다. 따라서 시나리오·시뮬레이션 간 제어 규칙 공유가 존재한다. 이는 같은 식을 두 번 구현한 중복은 아니지만, 임무 경로 계산이 물리 모듈 구현을 직접 참조하는 의존 경계다.
- 상자·프리즘 입력 검사가 두 물리 경로에 중복되어 있던 부분은 `collision::IsValidObstacleBox()`와 `collision::IsValidPrism()`으로 모았다. 외곽선 유효성도 `IsValidFootprint()`를 함께 쓴다. 위치·속도·자세 입력의 유한성 검사는 각 공개 경계에서 맡는 대상이 달라 유지한다.
- `SimulationCore.cpp`는 복잡하지만 “전처리와 수계산이 한데 섞여 물리식을 직접 구현한다”로 요약하면 부정확하다. 정확한 설명은 **플랫폼별 물리 경로를 연결하고 Flecs 상태를 맞추는 조립 코드**다. 운동학 접촉 계산은 `KinematicCollisionWorld`로 옮겼고, 비행 제어·충돌 기하·강체 적분 수식은 전용 코드에 있다. `__EMSCRIPTEN__` 분기는 여러 곳에 보이지만 웹과 네이티브의 물리 기능 차이를 컴파일 단계에서 표현한다. 분기 제거용 가짜 물리 구현이나 인터페이스 계층은 초심자 부담과 미지원 동작 오해를 늘릴 수 있어 이번에는 도입하지 않는다.

### 검토한 시험과 미확인 범위

`tests/simulation_core/SimulationCoreTests.cpp`는 배터리 소모/설정 오류, 고정 틱, 바람, 이륙·이동·착륙, 상자/기둥/지형 접촉, 명령 입력 오류, 초기 상태, 강체 모터 입력 제한, Flecs·PhysicsWorld 동시 생성/삭제, 개별 카메라를 포함한다. `QuadrotorModelTests.cpp`는 질량·관성·로터 배치·모터표 추정값, `PhysicsWorldTests.cpp`는 모터 지연/고장, 추력·중력·질량 영향, 제어기 응답, 좌표 변환, 지형/건물/드론 접촉, float 표현 한계를 다룬다. GoogleTest 묶음 수와 시나리오 함수 수는 별개이므로 테스트 개수만으로 중복·가치를 판정하지 않았다.

정적 읽기로 호출 순서와 자료형은 확인했지만, 실제 AirSim 적분 결과, 브라우저/네이티브 분기 동작, 동시 PhysicsWorld 안전성, 화면 표시와 성능은 런타임 확인하지 않았다. 다음 코드 변경 전에는 이 감사와 기존 테스트를 기준으로 “줄일 검사”와 “필수 경계 조건”을 구별해야 한다.

### 감사 후 판단과 첫 수정

- 확인된 비행 제어·이동 수식은 `DroneModel`, `QuadrotorFlightController`, `PhysicsWorld`, `CollisionGeometry`에 이미 나뉘어 있다. 이를 SimulationCore로 옮기거나 새 수학 추상화를 추가할 근거는 없다.
- SimulationCore 안에는 운동학 모드의 지면·장애물 접촉 응답과 속도 보정이 함께 있다. ECS 조립 코드와 별도 책임으로 읽히도록 이 접촉 처리를 구체적인 값 객체 하나로 분리하기로 했다. 인터페이스·상속은 추가하지 않는다.
- 프리즘 외곽선을 등록할 때 `IsValidFootprint()`가 이미 좌표 유한성을 확인한 뒤 같은 좌표를 다시 검사하는 코드를 제거했다. 충돌 기하 안에서도 프리즘 함수가 검증한 외곽선을 투영 함수가 다시 검사하던 중복을 한 번으로 줄였다. 각 함수 경계의 입력 거부와 계산식은 유지했다.
- 변경 커밋 `c334a84e`. Release 시뮬레이션 시험 6/6과 시뮬레이터 통합 시험 5/5 통과. 전체 CTest, Emscripten 빌드, 실제 화면은 확인하지 않았다.
- 접촉 보정과 운동학 장애물·지형 저장을 `KinematicCollisionWorld` 값 객체로 옮겼다. `SimulationCore`는 고정 Tick에서 운동학 계산 뒤 이 객체를 호출하고, 강체 모드는 그대로 `PhysicsWorld`가 처리한다. 공개 시뮬레이션 API의 입력 오류 검사와 물리 동작 순서는 유지했다.
- 변경 커밋 `a8ced502`. Release 시뮬레이션 시험 6/6, 시뮬레이터 통합 시험 5/5 통과. 전체 CTest, 웹 빌드, 화면 실행은 확인하지 않았다.

### `SimulationCore.cpp`의 플랫폼 전처리 분기 판단 — 2026-10-30

`SimulationCore.cpp`에는 `#if` 24개와 `#else` 2개가 있다. 이들은 크게 세 가지 일을 한다.

1. **AirSim 전용 코드 제외:** 헤더 포함, 강체 핸들과 `PhysicsWorld` 저장, 강체 생성·삭제·초기화, 로터 제어, 물리 상태 복사와 적분을 웹 빌드에서 제외한다.
2. **지원 차이를 명시:** 웹에서는 강체 모드를 지원하지 않는다. `Apply()`는 로터 명령에 `UnsupportedDynamics`를 반환한다. 이번 재확인 전에는 생성자에 강체 모드를 직접 지정하면 드론만 만들어지고 물리 시스템은 등록되지 않아 Tick이 진행되지 않는 경로가 남아 있었다. 이를 발견해 웹 빌드에서 강체 모드 선택 시 생성자에서 오류를 내도록 고쳤다. 초기 로터 속도도 강체 모드에서만 허용한다.
3. **환경 입력을 실제 계산 경로로 전달:** 바람·지형·장애물은 네이티브 강체 모드라면 AirSim에, 운동학 모드라면 `KinematicCollisionWorld`에 보낸다. Tick 갱신도 각 모드에 맞는 실행 경로를 사용한다.

루트 CMake는 네이티브에서만 `PhysicsWorld.cpp`와 AirSim 연결을 넣는다. 그래서 `SimulationCore.cpp`가 해당 타입을 직접 쓰는 곳에는 컴파일 분기가 필요하다. 공개 헤더의 기본 동역학 선택도 네이티브는 강체, 웹은 운동학으로 다르다. 즉 분기는 플랫폼과 물리 기능의 실제 차이를 표현하며, 단순히 같은 코드를 여러 번 감싼 장식은 아니다.

### 구현 선택지와 권고

| 선택 | 장점 | 부담·위험 |
|---|---|---|
| 현재의 네이티브 분기 유지 | 기존 구조와 “웹은 운동학, 네이티브는 AirSim” 동작이 바로 드러난다. 새 타입이나 가짜 물리 구현이 필요 없다. | 물리 객체의 생성·동기화·환경 전달 주변에 분기가 여러 번 보인다. |
| 별도 물리 인터페이스와 팩토리 도입 | 플랫폼 코드와 시뮬레이션 코드를 나눌 수 있다. | 현재는 물리 구현이 AirSim 하나뿐이다. 인터페이스·팩토리·구현체를 추가해도 런타임의 두 계산 경로는 사라지지 않아 구조가 더 어려워질 수 있다. |
| 공통 `PhysicsWorld` API와 웹 대체 구현 | CMake가 네이티브 구현 또는 웹 구현을 고르면 `SimulationCore.cpp`의 컴파일 분기를 줄일 수 있다. | 웹 대체 구현이 강체 기능을 조용히 무시하면 물리 결과를 잘못 믿게 된다. 현재 공개 API는 동작별 실패 반환이 충분하지 않아, 명시적 미지원 계약과 그에 맞는 호출 처리가 먼저 필요하다. |

**권고:** 지금은 현재 분기를 유지한다. 각 분기는 AirSim이 없는 웹에서 강체 코드가 링크되지 않게 하거나, 웹의 강체 요청을 명시적으로 거부한다. 이를 없애려고 물리 인터페이스나 팩토리를 추가하면 초심자가 따라가야 할 개념만 늘 수 있다. 특히 웹 대체 구현이 실제 강체 물리를 하는 것처럼 보이거나 입력을 버리는 형태는 허용하면 안 된다.

향후 분기 수를 줄이는 작업을 별도로 한다면 세 번째 선택지만 다시 검토한다. `PhysicsWorld`와 같은 메서드 모양을 갖추는 데 그치지 말고, 웹에서 강체 계산을 요청했을 때 명확한 오류로 끝나는 계약을 먼저 정해야 한다. 그 계약을 기존 API에 작은 수정으로 담을 수 없다면 현재 분기를 두는 편이 낫다. 물리 동작을 보존하기 위해 남겨야 하는 분기는 **AirSim 전용 타입·함수의 컴파일 제외, 웹 강체 모드 거부, 모드별 적분/환경 입력 전달**이다.

충돌 처리를 별도 객체로 옮긴 것은 기존 코드를 이동한 것이며, `PhysicsWorld`의 강체 충돌 코드를 복사한 작업은 아니다. `PhysicsWorld`는 접촉 정보를 AirSim에 넘겨 강체 반응을 계산하게 하고, `KinematicCollisionWorld`는 계산된 위치를 직접 밀어내고 안쪽 속도를 제거한다. 두 경로는 응답 방식이 다르며, 접촉 기하는 `CollisionGeometry`를 함께 쓴다. 환경 자료는 현재 동역학 모드에 해당하는 한 경로로 전달된다. 상자·프리즘 입력 검사는 이번에 공통 함수로 모았다. 각 경로의 충돌 형상 저장과 지형 등록은 구현 상태와 수명 방식이 다르므로 더 합치지 않았다.

웹의 강체 요청 오류 처리는 코드로 고쳤다. Windows Release에서 시뮬레이션 시험 6/6, 시뮬레이터 통합 시험 5/5 통과했다. 네이티브 빌드와 시험으로 웹 전용 오류 경로를 직접 확인할 수는 없으므로, Emscripten 빌드와 해당 경로 실행은 미검증이다.
