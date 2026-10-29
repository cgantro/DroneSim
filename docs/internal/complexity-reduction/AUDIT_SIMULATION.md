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
- `modules/simulation_core/src/CollisionGeometry.h`: KEEP — 확인 1–91. 강체 상자·외곽선 입력에서 접촉 값 또는 optional을 반환하는 내부 순수 계산 인터페이스(53–90). span은 비소유. 접촉 실패와 입력 오류가 nullopt로 합쳐지는 의미에 주의.
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
- `modules/terrain/include/terrain/MapBridges.h`: SIMPLIFY — 확인 1–46; 남은 줄 없음. GeoJSON 경로·지형을 받아 교량 외곽 값을 반환하고 교각을 추정한다(33–45). 반환 vector가 형상을 소유. 교량 점검이 MVP 범위 밖이면 시각화·충돌 호출처를 확인 후 제거 후보; 오류 세부는 구현 미확인.
- `modules/terrain/include/terrain/MapBuildings.h`: KEEP — 확인 1–87; 남은 줄 없음. 건물 외곽·높이와 출처를 값으로 소유하며 로딩·범위·충돌·외벽 계산 API를 제공한다. 경로·충돌용 건물은 유지, 시설 외벽 함수(76–85)는 사용처 확인 후 축소 후보. 로더 오류는 구현 미확인.
- `modules/terrain/include/terrain/MapFlightRoute.h`: SIMPLIFY — 확인 1–65; 남은 줄 없음. PlanMapFlightRoute(26)는 지형·장애물·시작·목표를 받아 경유점 또는 nullopt를 반환한다. 고정 V 및 조사·복귀 경로에 필요한 안전 경로는 유지하고 교량 하부·외벽 경로(43–64)는 시설 점검 범위 확인 후 축소 후보.
- `modules/terrain/include/terrain/MapRoads.h`: KEEP — 확인 1–39; 남은 줄 없음. GeoJSON 경로·지형에서 도로 중심선과 추정 폭 vector를 반환한다. 도로 형상은 값 소유이며 시각화용이고 충돌 형상은 아니다. 오류와 실제 필요성은 구현·호출처 확인 필요.
- `modules/terrain/include/terrain/MapTrees.h`: KEEP — 확인 1–21; 남은 줄 없음. 지도 경로·지형을 받아 나무 위치 vector를 반환한다. 값 소유, 오류는 구현 미확인. 시각화 사용처 확인 후 단순화 판단.
- `modules/terrain/include/terrain/MapWaterBodies.h`: KEEP — 확인 1–25; 남은 줄 없음. GeoJSON 경로·지형에서 물 외곽과 섬 경계 vector를 값으로 반환하고 ContainsWater로 포함 여부를 판정한다. 표시 사용처와 오류 처리는 구현 확인.
- `modules/terrain/include/terrain/TerrainGrid.h`: KEEP — 확인 1–67; 남은 줄 없음. 높이 vector를 소유하며 로드·저장, 고도 보간, WGS84와 ENU 변환을 제공한다. 지형·경로·충돌의 공통 기준; 구현의 입력 오류 확인 필요.
- `modules/terrain/include/terrain/TrainingRangeTerrain.h`: KEEP — 확인 1–9; 남은 줄 없음. 500×400m 시험 지형을 값으로 반환하는 무인수 함수. 사용처 및 실제 범위는 구현 확인.
- `modules/terrain/src/MapBridges.cpp`: SIMPLIFY — 확인 1–200, 201–215; 남은 줄 없음. LoadMapBridgeFootprints(99)는 GeoJSON과 지형에서 이름 있는 교량을 값으로 만들고 파일·JSON·features 오류는 예외, 부적합 개별 형상은 건너뛴다. EstimateMapBridgePiers(150)는 간격 기준으로 추정 교각을 반환한다. 시설 점검 확장과 표시·충돌의 공유 소비자를 확인 후 축소.
- `modules/terrain/src/MapBuildings.cpp`: KEEP — 확인 1–200, 201–336; 남은 줄 없음. LoadMapBuildings(68)는 GeoJSON과 지형을 읽고 높이 또는 층수를 적용하며 오류는 예외. ContainsMapBuildingVolume(175), FindFirstPointOutsideMapBuildings(204)는 건물 검사, GetMapBuildingFacades(245)는 점검면을 반환한다. GetMapBuildingBoundsMeters(151)는 비어 있지 않고 높이 배열이 대응하는 입력을 전제하므로 호출 계약 확인. 건물 회피는 유지하고 외벽 점검은 별도 축소 후보.
- `modules/terrain/src/MapFlightRoute.cpp`: SIMPLIFY — 확인 1–200, 201–400, 401–595; 남은 줄 없음. FindGridPath(152)는 5m 격자 8방향 A*와 대각선 코너 차단을 적용하고 SimplifyPath(238)는 선분 검사를 사용한다. PlanMapFlightRoute(281)는 입력 오류에 예외, 경로 없음에 nullopt를 반환하고 지형·건물로 경로를 생성한다. 높이는 경유점에서 확인하므로 긴 간소화 선분의 중간 지형 여유는 별도 검증 필요. 교량 하부·외벽 경로(430,544)는 시설 점검 사용처 확인 후 축소 후보.
- `modules/terrain/src/MapGeometry.h`: KEEP — 확인 1–164; 남은 줄 없음. ContainsPointInRing(21), ClipPolygonToTerrain(44), ReadTerrainFootprint(108)가 지도 형상 포함·경계 절단·좌표 변환을 수행한다. 결과 vector는 값 소유 또는 호출자 출력 참조, 부적합 형상은 false. 충돌·EO의 다른 경계 판정과 목적이 달라 무조건 합치지 않는다.
- `modules/terrain/src/MapRoads.cpp`: KEEP — 확인 1–200, 201–213; 남은 줄 없음. LoadMapRoads(171)는 GeoJSON과 지형에서 도로 구간을 값으로 반환하며 파일·JSON 오류는 예외, 범위 밖·잘못된 점에서 구간을 나눈다. RoadWidth(53)는 폭·차선·종류 순으로 추정한다. 건물 높이 숫자 파싱과 유사하지만 상한·의미가 달라 통합 전 검토.
- `modules/terrain/src/MapTrees.cpp`: KEEP — 확인 1–60; 남은 줄 없음. LoadMapTrees(11)는 GeoJSON의 나무 점을 지형 로컬 좌표로 변환해 값 vector 반환. 파일·JSON·features 오류는 예외, 형식이 다른 개별 항목은 건너뜀. 지도 표시 호출처 확인.
- `modules/terrain/src/MapWaterBodies.cpp`: KEEP — 확인 1–132; 남은 줄 없음. LoadMapWaterBodies(67)는 GeoJSON 물 외곽·섬 경계를 값으로 소유해 반환하며 파일·JSON·features 오류는 예외. ContainsWater(122)는 MapGeometry 포함 계산을 재사용해 섬을 제외. IntersectsTerrain(41)은 경계 상자 교차만 확인한다.
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
