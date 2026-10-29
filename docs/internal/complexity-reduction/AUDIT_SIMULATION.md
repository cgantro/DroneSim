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
- `modules/simulation_core/include/simulation_core/FormationPlanner.h`: SIMPLIFY — 1–82. 지형·장애물·리더 경로를 입력받아 값 소유 경로와 오프셋을 반환한다(23–79). V·Column·Line 및 전환 프로필은 고정 V 목표보다 넓다.
- `modules/scenario/include/scenario/ScenarioJson.h` (당시 `simulation_core/InspectionSurfaceJson.h`): REMOVED — 시나리오 JSON 안의 점검면 기하를 읽던 헤더. 자동 촬영 판정을 제거하면서 파서와 함께 삭제했다.
- `modules/simulation_core/include/simulation_core/PhysicsWorld.h`: KEEP — 확인 1–146. AirSim에 강체 생성·모터·지형·장애물·틱을 제공하며 Impl을 unique_ptr로 소유하고 복사를 금지한다(68–145). 잘못된 핸들의 DestroyBody는 false, 비유한 모터 입력은 거부(95,125). 충돌·좌표 계약 보존; 내부 수명은 구현 미완독.
- `modules/simulation_core/include/simulation_core/QuadrotorFlightController.h`: KEEP — 확인 1–51. 상태·목표·물성·이득을 받아 경유 속도와 4개 로터 목표를 값 반환(35–49). 자체 가변 상태 없음. 오류 동작은 구현 미확인; 강체 비행 제어에 필요.
- `modules/simulation_core/include/simulation_core/QuadrotorModel.h`: KEEP — 확인 1–60. 질량·관성·로터·충돌 상자·모터 계수를 값으로 소유하며 X500 기본 및 추정값을 반환한다(32–57). 오류·실제 호출은 구현 미확인. 별도 추정 모델은 MVP 사용 확인 후 정리 후보.
- `modules/scenario/include/scenario/ScenarioRunner.h` (당시 `simulation_core/ScenarioRunner.h`): SIMPLIFY — RunScenario가 JSON을 받아 드론·임무·궤적 결과를 값으로 돌려준다. 잘못된 문법·필수값은 예외로 알린다. 자동 촬영 재시도와 구도 판정 결과는 2026-10-29 제거했다. 임무 경로 재배정은 별도 정리 대상이다.
- `modules/simulation_core/include/simulation_core/ScopedDuration.h`: KEEP — 확인 1–52. 생성 때 시간 시작, 소멸 때 전달받은 double에 시간 대입·누적한다(21–38). 포인터는 비소유이며 대상이 범위보다 오래 살아야 한다. null은 측정 생략. 벤치마크 소비자 확인 후 제거 가능성 검토.
- `modules/simulation_core/include/simulation_core/SimulationCore.h`: KEEP — 확인 1–235. FixedTickAccumulator(55–66)는 잔여 시간을 소유하고 프레임 입력을 100ms로 제한한다. SimulationCore(72–234)는 ECS를 unique_ptr로 소유하고 명령·드론·카메라·배터리·충돌·틱 API를 제공한다. 없는 드론·배터리는 일부 예외, 마지막 드론은 제거 불가. 네이티브 AirSim과 웹 운동학 선택(31–38)은 호출 환경 확인 후 정리.
- `modules/simulation_core/src/BatteryModel.cpp`: KEEP — 1–91. 생성자(9)는 설정 범위 오류에 예외를 던지고 Advance(33)는 모드별 평균 전력과 고정 틱으로 잔량을 갱신한다. Reset(46)은 초기량과 복귀 표시를 복원한다. 복귀 자체는 실행하지 않는다.
- `modules/simulation_core/src/CollisionGeometry.cpp`: KEEP — 확인 1–200, 201–327. 외접구·AABB 및 분리축으로 상자 접촉을 계산하고(39,253), 외곽선 투영으로 footprint·prism 접촉을 반환한다(106,172,194). 유효하지 않거나 겹치지 않으면 nullopt; 값 기반 지역 상태. EO의 별도 다각형 판정과 목적이 달라 통합 전 경계 규칙 검토 필요.
- `modules/simulation_core/src/CollisionGeometry.h`: KEEP — 확인 1–91. 강체 상자·외곽선 입력에서 접촉 값 또는 optional을 반환하는 내부 순수 계산 인터페이스(53–90). span은 비소유. 접촉 실패와 입력 오류가 nullopt로 합쳐지는 의미에 주의.
- `modules/simulation_core/src/CommandRecording.cpp`: REMOVED — 당시 기록 경계 Tick과 재생 인덱스를 관리했다. 앱·HUD·UDP 호출처와 함께 삭제했다.
- `modules/simulation_core/src/DroneModel.cpp`: KEEP — 확인 1–200, 201–315. Apply(72)는 상태·명령을 검증해 결과를 반환하고 상태·제어값을 갱신한다. AdvanceOneTick(185)은 고정 간격으로 이륙·이동·착륙과 지면 접촉을 계산한다. 외부 상태를 참조로 수정하며 로터 명령은 강체 경로와 연결 확인 필요. 운동학과 강체의 동작 차이는 구현 검토 필요.
- `modules/simulation_core/src/EoCameraModel.cpp`: SIMPLIFY — 현재는 카메라 프레임·자세·상태를 갱신한다. 과거의 점검면 거리·각도·화면·해상도·건물 가림 판정은 2026-10-29 제거했다. 카메라 고장 상태에서 켜기 요청을 무시하는 동작은 유지한다.
- `modules/simulation_core/src/FormationPlanner.cpp`: SIMPLIFY — 1–413. FormationOffsets(260)는 잘못된 입력에 예외, 대형과 방향에서 좌표 벡터 계산. PlanFormationRoute(345)는 지형 경로를 호출하고 실패 시 nullopt. 387–408의 Column 우회와 96–207의 전환 프로필은 고정 V 범위 밖이다. 제거 시 V 경로 없음과 충돌 여유를 처리해야 한다.
- `modules/scenario/src/ScenarioJson.cpp` (당시 `simulation_core/InspectionSurfaceJson.cpp`): REMOVED — 점검면 기하 JSON 파서. 촬영 조건 판정과 함께 2026-10-29 삭제했다.
- `modules/simulation_core/src/PhysicsWorld.cpp`: KEEP — 확인 1–200, 201–400, 401–600, 601–800, 801–1000, 1001–1200, 1201–1270; 남은 줄 없음. Impl은 강체와 AirSim 시계를 소유하고 엔진에 강체 주소를 빌려준다(403–510); 소멸 전 clear. Step(1150)은 접촉·모터·엔진을 순서대로 실행하며 좌표 변환과 지형·장애물 접촉을 처리한다. 입력 오류와 미존재 핸들은 예외. 전역 시계의 동시 월드 실행 안전성은 미검증.
- `modules/simulation_core/src/QuadrotorFlightController.cpp`: KEEP — 확인 1–233; 남은 줄 없음. WaypointVelocityTarget(150)은 위치에서 제한 속도를 반환하고 MotorTargets(170)는 상태·제어·물성에서 자세 토크와 로터 속도를 계산한다. 상태는 소유하지 않으며 로터 연립식이 특이하면 예외(127). 강체 제어와 운동학의 목표 동작 차이는 유지 여부 확인.
- `modules/simulation_core/src/QuadrotorModel.cpp`: KEEP — 확인 1–67; 남은 줄 없음. X500Gazebo(7)는 기체·로터 관성 및 9개 충돌 상자를 값으로 구성하고 X500MotorTableEstimate(56)는 계수 세 개를 추정값으로 대체한다. 가변 상태·오류 처리 없음. 별도 추정값이 필요한지 소비자 확인.
- `modules/scenario/src/ScenarioDefinition.cpp`: SIMPLIFY — 확인 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1329; 남은 줄 없음. ReadScenarioDefinition(1254)은 JSON을 검증해 값 소유 정의를 반환하고 잘못된 입력은 예외. GenerateSurveyTasks(610)는 드론별 영역과 왕복 경로를 생성; 앱 임무 생성과 중복 여부 확인. 시설 점검·재시도 및 통신 장애·물성 변경 설정은 범위 정리 후보이며 필수 조사 경로와 구분.
- `modules/scenario/src/ScenarioDefinition.h`: SIMPLIFY — 확인 1–143; 남은 줄 없음. 시나리오 정의는 드론·예약 명령·통신 장애·편대·임무 등을 vector/optional 값으로 소유하고 파서 결과를 실행기에 전달한다(104–141). 고급 재할당 관련 결과 모델과 확장 설정은 소비자 확인 후 축소 후보.
- `modules/scenario/src/ScenarioRunner.cpp`: SIMPLIFY — RunScenario는 시나리오 정의를 읽고 관측, 명령, 에너지, 임무, 편대, 회피와 Tick을 진행한다. 임무 완료는 경유점 도착으로 확인한다. 자동 촬영 판정·재시도는 2026-10-29 제거했다. UpdateMissionCommands에는 통신·배터리 조건의 임무 재배정이 남아 있으며 별도 축소 후보이다. 앱에도 임무 실행이 있어 중복 책임 검토가 필요하다.
- `modules/simulation_core/src/SimulationCore.cpp`: KEEP — 확인 1–250, 251–500, 501–750, 751–1000, 1001–1179; 남은 줄 없음. EcsStorage(37)는 Flecs 엔티티·배터리와 물리 월드를 소유한다. AdvanceOneTick(895)은 ECS 진행 후 물리, 배터리, tick, 카메라 자세를 갱신하고 일시정지 시 반환. 생성 실패 시 물리 강체와 엔티티 정리(407). 운동학과 강체의 접촉 처리는 서로 다른 구현이며 플랫폼 요구 확인 후 축소 판단.

## Simulation Core의 실제 책임과 경계

- `SimulationCore`는 Flecs 세계와 드론 엔티티를 만들고, 명령 적용 및 단일 1/60초 틱을 외부에 제공한다. Flecs 컴포넌트는 별도 타입을 많이 두기보다 드론 상태·제어 목표·카메라 상태를 엔티티에 붙이며, 배터리와 식별자→엔티티 대응은 `EcsStorage`의 별도 맵에 둔다. 따라서 ECS는 실제로 사용되지만 모든 시뮬레이션 데이터를 컴포넌트로 표현하지는 않는다.
- Flecs 시스템은 운동학 모드에서 `DroneState`와 `DroneControlState`를 순회해 운동학 계산과 접촉 처리를 한다. 카메라 시스템은 `CameraState`를 갱신한다. 강체 모드에서는 Flecs 시스템이 목표 로터 속도를 AirSim의 물리 월드에 전달하고, `AdvanceOneTick()` 뒤 물리 월드를 한 번 진행해 결과를 각 드론 상태에 복사한다. 현재 구조는 ECS 시스템이 전체 물리 적분을 맡는 형태가 아니라, ECS가 드론 상태·제어를 보유하고 별도 `PhysicsWorld`가 강체를 소유하는 혼합 구조다.
- `AdvanceOneTick()`의 순서는 일시정지 확인 → Flecs 진행(제어·운동학/로터 입력·카메라 tick) → 강체 모드의 AirSim 적분과 상태 동기화 → 배터리 갱신 → Tick 증가 → 카메라 자세 동기화다. 이 순서가 앱·헤드리스 양쪽에서 공유되는 실행 단위다.
- 화면 장면과 렌더링은 Simulation Core에 있지 않다. `TrainingRangeScene`과 그래픽 모듈이 장면 자산·그리기 데이터를 다루고, 앱이 시뮬레이션 상태를 렌더러 입력으로 넘긴다. 상세 모듈 책임·파일 배치·장면 관리 조사는 사용자 요청에 따라 다음 단계의 별도 구조 감사에서 한다.
- 공개 API는 명령·드론 생성/삭제·배터리·카메라·바람·로터 고장·계측·장애물·지형·시간을 한 `SimulationCore`에 모아 둔다. 단일 틱 진입점 자체는 알기 쉽지만, 기능별 설정·계측까지 한 클래스의 공개 계약에 넓게 노출되어 있다. 실제 소비자와 제거 가능성을 확인하기 전에는 나누거나 삭제하지 않는다.
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
- `tests/simulation_core/FormationPlannerTests.cpp`: SIMPLIFY — 확인 1–140; 남은 줄 없음. 4대 V 회전 좌표(17)와 경로·Column 우회 및 V 복귀 프로필(32)을 검증한다. 고정 V 요구에 맞춰 전환 기대를 정리하되 4대 오프셋·경로 실패 안전 검사는 유지. 실행하지 않음.
- `tests/simulation_core/PhysicsWorldTests.cpp`: KEEP — 확인 1–250, 251–500, 501–750, 751–888; 남은 줄 없음. 모터·제어·좌표계·지면·장애물·드론 충돌과 범위 오류를 확인한다(44,274,489,511,582,744,782). AirSim 접촉과 수명 변경 시 중요한 회귀 범위이며 고장 주입 시험은 기능 범위 결정 후 정리. 실행하지 않음.
- `tests/simulation_core/QuadrotorModelTests.cpp`: KEEP — 확인 1–124; 남은 줄 없음. 제조사 표의 추정 계수와 X500 질량·관성·로터 배치(12,86)를 확인한다. 해당 모델을 유지할 때 물성 변경 검증에 필요. 실행하지 않음.
- `tests/scenario/ScenarioRunnerTests.cpp` (당시 `tests/simulation_core`): SIMPLIFY — 확인 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1500, 1501–1688; 남은 줄 없음. 시나리오·카메라·물성·통신·4대 분산 조사·충돌 회피·저배터리와 재할당을 검사한다(160,762,1000,1250,1568). 16–256대 확장 및 재할당 기대는 제거 후보; 4대 조사·통신 단절의 미확정 처리·복귀·충돌 안전 시험은 요구에 맞게 보존. 실행하지 않음.
- `tests/simulation_core/SimulationCoreTests.cpp`: KEEP — 기준선 시험은 배터리·고정 틱·운동학 장애물·지형·ECS 및 드론별 카메라 상태를 확인했다. 점검면 촬영 판정 시험은 기능 제거와 함께 삭제했다. 현재 시험 실행 결과는 `VERIFICATION.md`에 기록한다.
- `tests/terrain/TerrainGridTests.cpp`: KEEP — 확인 1–200, 201–284; 남은 줄 없음. 지형 크기·좌표·보간·파일 왕복과 오류 및 건물 우회·막힌 경로를 확인한다(26,40,75,89,150,203). 건물 경로 시험은 평지로 긴 경로의 중간 지형 장애를 확인하지 않는다. 외벽·교량 관련 기대는 기능 제거 시 함께 정리. 실행하지 않음.
