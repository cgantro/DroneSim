# 빌드와 시험 기록

## 시나리오 자동 구역 분할 제거 — 2026-10-30

- `survey_area` 입력으로 구역을 드론 수만큼 자동 분할하고 지그재그 경로를 만드는 기능을 제거했다. 저장소 시나리오나 앱에서는 쓰지 않았고 시험 한 곳에서만 호출했다. 시험은 같은 네 구역과 경유점을 명시해 임무 완료와 100% 경유점 진행을 계속 확인한다.
- Windows MSVC Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 통신 단절 뒤 정지 동작 제거 — 2026-10-30

- `on_timeout`은 통신 두절 뒤 제자리 정지 또는 마지막 명령 지속을 선택하게 했다. 요구한 동작은 이미 받은 경로를 계속 비행하는 것이므로 선택지와 정지 코드를 제거했다.
- 처음에는 최종 위치만 보고 시험이 실패했다. 최종 드론 상태는 GCS가 마지막으로 받은 위치였고, 임무 완료도 새 상태 보고가 있어야 갱신됐다. 실제 비행 궤적과 GCS 보고값을 각각 확인하도록 시험을 고쳤다. 단절 중 드론은 목표까지 비행하지만 GCS 상태와 임무 완료 판정은 미확정이다.
- Windows MSVC Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 시나리오 시험 1/1, 전체 CTest 28/28 통과.

## 시나리오 임무 입력 형식 통일 — 2026-10-30

- 단일 목표와 경유점 목록의 중복 입력 형식을 없애고 모든 임무가 한 점 이상을 담은 `waypoints_meters` 목록을 사용하게 했다. 기존 단일 목표 입력이 거부되는지도 확인했다. Headless 결과 JSON의 목표 좌표 항목은 유지했다.
- 확인: Windows, MSVC Release, Ninja. `aerosimlab_headless`와 `aerosimlab_scenario_tests` 빌드 성공, 전체 CTest 28/28 통과. Headless 임무 연기 시험 3개도 전체 CTest에 포함됐다.

## 임무 판정 계산 — 2026-10-30

- `Mission`의 결과 열거형은 완료·실패 작업 상태에서 계산할 수 있었다. 별도 필드를 제거하고 `GetOutcome()`에서 작업 목록으로 판정하도록 바꿨다. Headless JSON의 `outcome`과 `passed` 키는 유지한다.
- Windows Release `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 시나리오 드론 시작 상태 통합 — 2026-10-30

- 시나리오 정의에서 드론 시작 상태를 위치·방향·속도·자세·모터·모드 필드로 나눠 저장하고, 실행 중 다시 `DroneState`를 조립하던 부분을 제거했다. 이제 같은 형식을 그대로 보관하고 Simulation Core에 전달한다.
- Windows Release `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과. 강체 초기 속도·자세·로터 상태를 확인하는 기존 시나리오 시험이 포함됐다.

## 부분 완료 임무의 완료 시각 수정 — 2026-10-30

- 직접 통신 단절로 4개 작업 중 3개만 완료되는 경우, 이전에는 가장 늦게 끝난 작업의 시각이 전체 임무 완료 시각으로 노출됐다. 이제 전체 작업이 끝나지 않으면 `completed_tick`과 `completion_time_seconds`를 비운다.
- 기존 heartbeat 단절 시험에 전체 완료 시각이 없는지 확인을 추가했다. Headless 출력은 `inconclusive`, 3/4 완료, `completed_tick` 없음, `completion_time_seconds` 없음으로 확인했다. 이 시나리오 실행 파일은 미확정 결과를 반환하므로 종료 코드 1이 정상이다.
- Windows Release `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 임무 완료 상태 중복 제거 — 2026-10-30

- `MissionTask`의 `completed` 불리언은 완료되는 순간에만 `completedTick`과 함께 설정됐다. 불리언을 제거하고 `IsCompleted()`가 완료 Tick의 유무를 확인하도록 했다. JSON의 `completed` 항목은 같은 의미로 계속 출력한다.
- Windows Release `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 최상위 기본 드론 상태 중복 제거 — 2026-10-30

- `ScenarioResult`의 최상위 비행·카메라 복사본을 제거했다. 시나리오는 기본 드론을 포함해 적어도 한 대를 결과에 담으므로, Headless JSON은 첫 드론 상태를 최상위 `final_state`·`camera` 항목으로 출력한다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.
- `two-point-survey.json` 출력에서 최상위 비행 위치와 첫 드론 위치가 같고 카메라 상태도 둘 다 `active`인 것을 확인했다.

## 시나리오 목표 좌표 중복 제거 — 2026-10-29

- `MissionTask`에서 첫 경유점과 같은 값을 따로 저장하던 목표 위치를 제거했다. Headless JSON은 기존 `target_position_meters` 형식을 유지하고 첫 경유점 값을 출력한다.
- 첫 시험에서 구조체 헤더 변경이 시험 객체에 반영되지 않아 오래된 ABI로 실행되는 문제가 드러났다. Ninja 의존성 기록이 `#deps 0`이었고, CMake가 MSVC의 `참고: 포함 파일:` 출력을 잘못된 문자로 기록한 것이 원인이었다.
- UTF-8 콘솔에서 Ninja Release 빌드 설정을 다시 만들자 시험 객체의 헤더 의존성이 230개 기록됐다. 이후 `aerosimlab_headless`와 `aerosimlab_scenario_tests` 빌드 성공, 전체 CTest 28/28 통과.
- 확인 환경: Windows, MSVC Release, Ninja. 웹 빌드는 이번 변경에서 확인하지 않았다.

## 시나리오의 리더 추종 링크 모의 제거 — 2026-10-29

- JSON 파서·실행기의 리더 추종 링크와 별도 단절 구간, 전용 사례를 제거했다. 직접 통신 장애와 화면 앱의 편대 경로에는 손대지 않았다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과. 기존 ViewerSmoke도 포함됐다.
- 처음에는 Visual Studio 개발자 환경을 불러오지 않은 셸에서 MSVC 표준 헤더를 찾지 못했다. 개발자 명령 환경을 불러온 뒤 다시 빌드해 성공했다.

## 편대 임무 시작 조건 통합 — 2026-10-29

- 두 임무 시작 함수에 반복되던 진행 단계·정찰 지점·진행 중 정찰·드론 착륙 검사를 `FleetIsReadyForMission()`으로 모았다. 전체 임무의 드론 4대 조건은 별도 검사로 남겼다. 복귀 가능 여부의 중복 단계 검사는 `CompleteFleetMissionIsActive()` 호출로 통합했다.
- 기존 `CompleteMissionRequiresFourDrones` 시험에 진행 중 정찰 상태를 넣어, 모든 기체가 지상에 있어도 새 정찰과 전체 임무 시작을 막는지 확인했다.
- Release 전체 빌드 성공. 시뮬레이터 통합 시험 4/4 및 전체 CTest 29/29 통과.

## 드론 ID 목록 작성 단순화 — 2026-10-29

- `DroneIds()`는 ID→Flecs 엔티티 맵에서 ID만 복사한다. 사용하지 않는 엔티티 값의 구조화 바인딩과 `(void)` 처리를 제거했다. 반환 순서는 `std::map`이 보장한다.
- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과. 기존 SimulationCore 수명주기 시험이 드론 ID의 오름차순과 주 드론 제거 동작을 확인한다.

## 카메라 목록에서 장면 시점 바꾸기 — 2026-10-29

- 카메라 패널에서 드론 이름을 선택하면 장면 패널의 관찰 카메라가 해당 기체를 따라가도록 연결했다. 현재 시점 기체에는 `>`를 붙여 선택 상태를 표시한다. 기존 장면 시점 드롭다운도 유지했다.
- Windows Release 전체 빌드 성공. 전체 CTest 29/29 통과. 기존 `SimulatorSessionTests`에는 기체 ID를 바꿨을 때 관찰 카메라 목표가 바뀌는 확인이 있다. 이번에는 GUI 마우스 입력을 직접 자동화하지 않았다.

## 직접 통신 검사 중복 조회 제거 — 2026-10-29

- 가상 통신 검사에서 송수신자 존재 여부를 한 번만 찾도록 정리했다. 없는 노드 ID를 좌표 기반 연결 검사에 넣는 기존 시험 경로에서 계속 연결 불가로 처리되는지도 확인했다.
- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.

## 드론 ID 순서 관리 통합 — 2026-10-29

- `std::map` 을 사용해 드론 ID 목록의 순서를 자료구조에서 보장했다. 드론 ID 목록 생성 후 정렬과 주 드론 제거 후 최소 ID 검색을 제거했다.
- 기존 SimulationCore 수명주기 시험에 3개 ID가 오름차순으로 돌아오는지 확인을 추가했고, 최소 ID 주 드론이 삭제된 뒤 다음 ID가 주 드론이 되는 기존 확인은 유지했다.
- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.

## 운동학 접촉 단계 분리 — 2026-10-29

- `ResolveKinematicContacts` 안에 지형 경계·지면·상자·다각형 처리가 한데 연결되어 있어, 순서만 보이는 처리 조정자와 종류별 처리 함수로 분리했다. 함수 호출 순서는 원래와 같다.
- 기존 SimulationCore 시험에 운동학 상자 충돌 사례를 합쳐, 상자에 막힌 드론의 위치와 진행 속도가 제거되는지 확인했다.
- `Simulation.SimulationCoreTests.CoreScenarios` 1/1 통과. Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.

## 시작 자세에 따른 해당 카메라 갱신 — 2026-10-29

- 하나의 드론의 시작 자세를 변경할 때 전체 플릿의 카메라 자세를 다시 계산하던 처리를 해당 ECS 엔티티 하나만 갱신하도록 바꾸었다. Tick 마다 전체 카메라를 갱신하는 흐름은 유지했다.
- 기존 시작 자세 시험에 카메라 마운트와 드론의 상대 위치를 자세 설정 직후와 Reset 후에 확인하는 검사를 합치고 전체 테스트 실행에 포함했다.
- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.

## 좌표 벡터 유효성 검사 공유 — 2026-10-29

- `HasFiniteVector` 함수를 추가해 시작 위치, 속도, 각속도, 바람 속도, 장애물 위치의 세 좌표가 모두 유효한지 판정하는 중복 코드를 통합했다. 로터 속도·자세 정규화·양수 장애물 크기 검사는 기존 조건을 유지했다.
- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.

## 평면 장애물 접촉 처리 통합 — 2026-10-29

- 수정: 건물 외곽선과 높이 미상 금지 구역에 같은 XY 피해 및 내향 속도 제거 계산을 적용했다. 복제 코드를 `ResolvePlanarObstacleContact` 한 곳으로 통합했다.
- 관련 시험: `Simulation.SimulationCoreTests.CoreScenarios` 1/1 통과. 건물과 높이 미상 장애물, 굴때기 안에서 접촉 처리를 포함한다.
- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.

## 드론별 주 시점 선택 — 2026-10-29

- 확인 범위: 네이티브 시뮬레이터의 3D 장면에서 선택한 드론을 관찰 카메라가 따라가는 기능. 웹 화면은 변경하지 않았다.
- `SimulatorSessionTests` 기존 시나리오에서 두 드론의 상태를 다르게 설정하고 선택한 드론이 시점 목표가 되는지 확인했다. 새 CTest 항목은 늘리지 않았다.
- Windows Release 빌드 성공. 실패했던 통합 시험 4개와 ViewerSmoke를 앱 및 시험 산출물 전체 새로 빌드한 뒤 통과했고, 이어 전체 CTest 29/29가 통과했다.
- 증분 빌드 시 해당 소스 파일의 헤더 의존 기록이 비어 있었고, 해당 파일을 전부 재빌드한 뒤 오류가 사라졌다. 이번 시험은 전체 새 빌드로 통과했지만, 빌드 시스템 자체를 수정한 것은 아니다.

- 기준 HEAD: d6515b6ea1c4ee56a8d0bb2c730aa3152b0d6d21
- 이전 Release 앱 빌드 성공. 시뮬레이터 통합 시험 2개 통과.
- 헤드리스 스모크: 빌드 정리 뒤 실행 파일이 없어 미실행. 대상 재빌드 후 확인 필요.
- vector too long: 이전 시도에서 보고됨. 발생 위치와 원인은 미확인.

## 현재 작업 트리 기준선 — 2026-10-29T02:54:07

- Release 전체 빌드: `cmake --build build-ninja-release`, 종료 코드 0. 로그: `build-ninja-release/audit-baseline-build.log`.
- 전체 시험: `ctest --test-dir build-ninja-release --output-on-failure`, 종료 코드 0, 37/37 통과. 로그: `build-ninja-release/audit-baseline-ctest.log`. 헤드리스와 ViewerSmoke 포함.
- 현재 앱 두 시험은 재빌드 후 통과. 앞선 접근 위반의 정확한 원인은 미확인. vector too long과 동일 문제라고 판단할 근거 없음.

## 복잡도 기준 집계

- 대상: 추적된 apps/와 modules/ 아래 C/C++ 소스·헤더. 외부 코드, 시험, 자산, 문서 제외. 현재 미커밋 변경 포함.
- 파일 124개, 물리 줄 31098줄. 정규식 추정 선언: 클래스/구조체 230개, enum class 38개. 선언 추정치는 주석·문법에 따른 오차가 있어 동일 규칙으로만 비교.
- 테스트: CTest 37개. CMake 타깃·필드·직접 의존성은 감사 후 같은 방식으로 전후 집계할 것.

- 기존 앱 전체 임무 시험은 Windows 기본 강체 동역학을 사용하지만 가상망 기본값은 AdaptiveMesh다. 통과를 직접 통신만으로 네 대 임무를 완료했다는 근거로 사용하지 않는다.
- 헤드리스 협동 임무 시나리오의 dynamics는 kinematic이다. 강체 비행 검증으로 해석하지 않는다.

- App 헤더의 이름이 `_`로 끝나는 선언 정규식 추정 116개, 같은 헤더의 bool 선언 정규식 추정 26개. 선언 범위·배열·함수 표현에 따른 오차가 있으며 같은 패턴으로만 전후 비교한다.
- 루트와 cmake/의 add_library/add_executable 선언에서 중복을 제거한 타깃 이름 28개(조건부·외부 포함). 이름: Eigen3::Eigen, OpenGL::GL, aerosimlab_airsim, aerosimlab_communication, aerosimlab_communication_tests, aerosimlab_glad, aerosimlab_graphics, aerosimlab_ground_control_client, aerosimlab_gstreamer, aerosimlab_headless, aerosimlab_imgui, aerosimlab_manual_control, aerosimlab_manual_control_tests, aerosimlab_media, aerosimlab_media_tests, aerosimlab_sensor, aerosimlab_sensor_tests, aerosimlab_simulation_core, aerosimlab_simulation_tests, aerosimlab_simulator, aerosimlab_simulator_app, aerosimlab_simulator_integration_tests, aerosimlab_stb_image, aerosimlab_terrain, aerosimlab_terrain_grid_converter, aerosimlab_vision, aerosimlab_vision_tests, glfw. vcpkg 직접 선언 의존성은 OpenCV와 GStreamer다.

## 실행 측정 기준선

- 2026-10-29T13:08:22: Windows Release `DroneSim.exe --benchmark-visible --demo`, 현재 d6515b6e 미커밋 변경 포함, 가시 창 1280×720, RTX 3060 Ti, 준비 1초 후 5.0005초 1회. 네 대가 생성되는 현재 초기화 경로, demo의 주 드론 자동 비행이다. 네 대 편대 임무의 전체 실행 측정은 아니다.
- `AdvanceSimulationTick` 처리시간 300개: p95 49.4μs, 최대 104.1μs, 16.67ms 초과 0회. 임무 완료 시간은 측정하지 않았다. 반복 측정 전의 단일 표본이며 성능 개선 주장에 사용할 수 없다.
- 원본 `baseline-visible.json`. 원본 configuration 문자열은 이전 지형과 한 대라고 고정 표기하므로 사실 관계에 사용하지 않는다. 현재 코드의 시작 기체 생성과 훈련장 경로로 환경을 확인했다. 보고서 생성은 확인했으나 GUI 프로세스 종료 코드 별도 수집은 하지 않았다.

## 녹화 종료 동기화

- 2026-10-29T13:14:08: `EoVideoRecorder::Stop`에서 활성 인코더 종료 전에 저장 프레임 수를 읽지 않도록 변경. 비활성 녹화의 기존 출력값은 유지.
- Release 앱·시뮬레이터 통합 시험 대상 빌드 성공. `ctest --test-dir build-ninja-release -R Simulator.SimulatorSessionTests.CoreScenarios --output-on-failure`: 1/1 통과, 영상 저장 및 프레임 메타데이터 검사 포함. 기존 미커밋 변경이 함께 빌드된 상태다.
- 스레드 sanitizer나 경합 재현은 실행하지 않았다. 데이터 경쟁 제거는 스레드 join 전 읽기를 제거한 코드와 C++ 동기화 관계에 근거한다.

## 고정 리더·중계 복구 제거

- 2026-10-29T13:15:41: 이전 미커밋 5개 파일의 리더 교체·중계 위치 제어와 전용 시험 제거를 감사 후 확인. 임무 단계 3을 초과하는 텔레메트리·경로를 거부하고 텔레메트리 시험 입력의 누락 필드를 보완했다.
- Release 앱과 통합 시험 대상 빌드 성공. 관련 CTest 3/3 통과: 시뮬레이터 CoreScenarios, 저전력 복귀, 헤드리스 협동 스모크. 기존 가상망은 아직 AdaptiveMesh이며 직접 통신의 목표 동작 검증이 아니다.

## 정찰 첫 경유점

- 2026-10-29T13:16:31: `ReconnaissanceKeepsTheFirstWaypoint` 시험을 추가하고 수정 전 실행. 경로 탐색 결과 3개 중 실제 임무에는 뒤의 2개만 전달되어 실패(CTest 종료 코드 1). 첫 경유점이 빠진다는 코드 분석을 실제 앱 경로에서 확인했다.

- 2026-10-29T13:17:02: 정찰·재집결·일반 복귀·저전력 복귀에서 시작점을 제외한 반환 경로 전체를 전달하도록 수정. Release 앱과 통합 시험 빌드 후 새 첫 경유점 시험, 기존 임무·배터리 시험, 헤드리스 스모크, ViewerSmoke 합계 5/5 통과. 새 시험은 정찰 경로를 직접 비교하며 나머지 세 경로의 첫 점을 개별적으로 비교하지는 않는다. 실제 화면 비행 충돌 시험은 미실시.

## 저전력 정찰 판정

- 2026-10-29T13:18:23: 저전력으로 복귀한 드론을 정찰 완료로 간주하지 않도록 기대값을 바꿔 시험했다. 기존 판정은 진행(Advance)을 반환해 CTest 실패. 실제 드론 전체 임무의 재현은 아직 하지 않았다.

- 2026-10-29T13:18:53: 저전력 복귀 보고가 정찰 단계라면 전체 임무를 성공으로 진행하지 않고 중단으로 판정하도록 수정했다. Release 앱 및 통합 시험 빌드, 시뮬레이터 관련 CTest 3/3 통과. 실제 4대 저전력 장애 임무는 별도 실행하지 않았다. 현재 보고에는 정찰 지점 도착 전후가 구분되지 않아 어느 경우든 보수적으로 중단한다.

## 편대 대형 자동 전환 제거

- 2026-10-29T13:21:31: V 편대 경로가 막히면 자동 일렬 전환을 하지 않고 실패하게 변경. 자동 전환을 위한 경로 표본·프로필·보간 함수와 화면 문구를 제거했다. V 간격을 유지하는 넓은 경로와 좁은 통로 실패 시험을 확인했다.
- 기존 Release 빌드 폴더를 정리한 뒤 전체 빌드 성공. 편대 계산·시뮬레이터·배터리·첫 경유점·헤드리스 및 Viewer 관련 CTest 7/7 통과. 앱 정상 임무 시험은 아직 AdaptiveMesh 기준이다.
- Line/Column 열거형 및 기존 호출 인터페이스는 시나리오·UI 연결 정리 전까지 남아 있다. V 전용 기능 정리 완료는 아니다.

## V 편대와 네 대 시작 조건

- 2026-10-29T13:23:57: 편대 형태 열거형·선택 인자·상태를 제거하고 V자 오프셋만 계산한다. 전체 임무는 네 대가 아니면 시작하지 않도록 변경했다.
- Release 전체 정리 후 재빌드 성공. V 배치, 좁은 경로 실패, 기존 전체 임무, 저전력·첫 경유점, 세 대/다섯 대 시작 거부 및 Viewer 시험 CTest 7/7 통과. 기존 동적 기체 추가 및 별도 시나리오 경로는 아직 존재한다.

## 장애 시간표 제거

- 2026-10-29: 장애 예약·기록·재생과 전용 시험을 제거하고 수동 라디오·링크 조작과 영상 녹화를 남겼다.

- 2026-10-29: 카메라 영상·사진 저장 경로를 제거했다. 당시에는 EO/WebRTC 전송과 정찰 구도 판정이 남아 있었다. 웹 스모크에서는 파일 저장 확인을 빼고 임무 시작·정지·완료를 확인한다.
- Windows Release 전체 빌드와 CTest 결과는 아직 기록하지 않았다. JavaScript 구문 확인과 CTest를 빌드 후 추가한다.
- Windows Release 전체 재빌드 성공, CTest 39/39 통과. 기존 네 대 임무 시험은 여전히 AdaptiveMesh를 사용한다. 실제 창에서 차단·복구 조작은 확인하지 않았다.
- 처음 빌드 시 컴파일러 환경이 설정되지 않아 표준 헤더를 찾지 못했다. Visual Studio 개발 환경을 설정한 뒤 전체 재빌드했다.

## 기체별 관제 상태 전달

- 직접 통신 시험에서 리더→관제 링크만 차단하자 다른 세 기체의 상태가 관제에 도착하지 않아 실패했다. 당시 모든 상태가 리더를 통해 전달되었다.
- 기체별 관제 송신과 수신을 적용하고 이웃 발견 후 송신하도록 변경했다. Windows Release 앱·시험 빌드와 통합 시험, ViewerSmoke 2/2 통과. 연결된 시험망에서 리더의 관제 링크만 차단했을 때 나머지 기체의 직접 전달, AdaptiveMesh에서 특정 라디오 차단 중 마지막 수신 상태 유지와 복구 후 갱신을 확인했다. 실제 거리·가림과 4대 임무 전체는 별도 확인이 필요하다.

## 시뮬레이터 직접 연결 기본값

- 통신 방식 선택 UI를 없애고 시뮬레이터 기본 모드를 직접 연결로 바꿨다. 첫 네 정찰 지점을 25m 간격으로 배치했다.
- Windows Release 앱·시험 빌드 성공, CTest 39/39 통과. 네 대 전체 임무의 M01·M02·M03 완료를 확인했다.
- 당시에는 앱만 직접 연결로 설정했고, 통신망 내부의 경로 탐색·다중 홉 구현은 남아 있었다. 아래 기록에서 해당 구현과 옛 시험을 제거했다.

## 가상 통신망 직접 연결로 축소

- 경로 모드, HELLO 이웃 탐색, 경로 요청·응답·오류, 다중 홉 전달, 경로 대기열, 통신망 ACK·재전송, 무작위 손실·지연 흔들림 설정을 제거했다. 거리·건물 가림·전원·라디오·직접 링크 장애와 Tick 지연은 유지했다.
- HUD에서 통신 방식 선택, 방향 링크 장애, 연결 그래프, 다음 홉·경로 표와 중계 통계를 걷어냈다. 드론 한 대의 송수신 차단·복구, 지상통제 연결 상태와 전달·폐기 수치를 남겼다.
- 통신 단위 시험은 직접 전달 지연, 중간 드론을 통한 전달 불가, 링크·라디오 차단과 복구, 건물 가림, 직접 링크 상태를 확인하는 5개로 바꿨다.
- Windows Release 전체 빌드와 전체 CTest를 확인했다. CTest 32/32 통과. 직접 통신 시험 5개와 4대 임무 통합 시험을 포함하며, 통합 시험은 M01·M02·M03을 수행하고 착륙한다. ViewerSmoke도 통과했다.
- 기존 빌드에서 앱 헤더를 바꾼 뒤 앱 번역 단위 일부가 재컴파일되지 않아 통합 시험에서 접근 위반이 났다. 앱 소스 전체를 다시 빌드한 뒤 통과했다. 시험 실행 파일도 새로 링크했으며 기존 실행 파일 결과를 새 코드 검증으로 계산하지 않았다.
- 실제 창에서 드론 선택 후 라디오 차단·복구를 조작하는 과정은 확인하지 않았다. 웹 실행도 이번 변경에서 검증하지 않았다.

## 임무 상태 열거형

- 내부 임무 상태 저장·비교를 문자열에서 `FleetDroneStatus` 열거형으로 바꿨다. 드론 간 상태 메시지도 이 열거값을 전달한다. 기존 한국어 표시는 화면이나 GCS 메시지 경계에서만 만든다.
- 사용되지 않는 고장 착륙·리더 통신 장애·리더 재배정 대기 상태와 변환 분기를 제거했다.
- 한국어 문자열에서 상태값을 찾아내던 변환 함수와 분기를 제거했다. 내부 텔레메트리에서 알 수 없는 상태 숫자는 거부한다.
- 검증: Windows Release 앱·통합 시험 빌드 성공, 전체 CTest 32/32 통과. 처음에는 오래된 개체 파일과 새 열거형 값이 섞여 통합 시험 하나가 실패했다. 앱과 시험을 다시 컴파일한 뒤 재실행해 통과했다.

## 사용하지 않던 통신 상태 필드 제거

- 통신 단절 시각 필드는 선언과 초기화만 있었고 읽히지 않아 제거했다.
- 관제 텔레메트리에서 임무 상태를 표시 문구로 변환하는 경로도 확인했다.
- 검증: Windows Release 앱·통합 시험 재빌드 성공, 전체 CTest 32/32 통과. 첫 실행은 헤더 변경 전 시험 개체 파일이 남아 임무 통합 시험 하나가 실패했다. 해당 시험을 재컴파일하고 다시 실행해 통과했다.

## 카메라 녹화·파일 저장 제거 및 현재 기준선

- EO 카메라 동영상 녹화, 촬영 파일·메타데이터 저장, 브라우저 다운로드·저장 기능을 제거했다. 당시 라이브 EO 화면, WebRTC 전송, 카메라 시야 기반 정찰 판정과 명령 기록·재생은 유지했다.

## EO 영상 네트워크 전송 제거

- 2026-10-29: 사용자가 RTP 영상 전송도 제거하도록 요청했다. 네이티브 GStreamer RTP/RTCP 송수신 모듈과 전용 시험, 웹 WebRTC loopback을 삭제했다. GStreamer 패키지와 연결 설정도 빌드에서 빠졌다.
- 화면 안의 EO 보기 기능은 유지한다. 네이티브는 카메라 렌더 대상 텍스처를 화면에 직접 그리고, 웹은 시뮬레이터 캔버스의 카메라 영역을 같은 페이지의 카메라 캔버스에 복사한다. 이 동작은 영상 네트워크 송신이 아니다.
- Windows Release 전체 빌드 성공. CTest는 30/30 통과했다. 첫 전체 시험에서는 시뮬레이터 통합 시험 4개가 접근 위반 또는 `vector too long`으로 실패했다. 전체를 깨끗이 다시 빌드한 뒤 재실행하자 모두 통과했다.
- 웹 셸의 인라인 JavaScript 두 구간은 Node 구문 검사에 통과했다. 브라우저에서 직접 실행해 화면을 확인하지는 않았다.
- Windows Release 전체 빌드 성공. CTest 31/31 통과. `tools/web_runtime_smoke.cjs` Node 문법 검사와 `git diff --check`도 통과했다.
- 코드와 공개 문서에서 제거 대상 구현 참조를 검색했고 남은 항목은 없었다. 브라우저에서 웹 데모를 직접 실행하는 검증은 하지 않았다.
- 통신 통합 시험은 시험용 네트워크가 노드 목록을 만들기 전에 차단 명령을 보내 실패했다. 초기 노드 갱신·상태 전달 뒤 차단하도록 시험 순서를 바로잡고 전체 시험을 통과했다.

## 앱 내 조종 명령 기록·재생 제거 — 2026-10-29

- 앱에서 수동 조종 명령을 저장하고 다시 실행하던 기능을 제거했다. 관련 프로토콜 명령, 앱 상태, HUD 조작부, 전용 구현과 단위 시험도 함께 삭제했다.
- JSON 시나리오 실행·재생은 자동 임무 시험에 계속 사용하므로 유지했다. 게임패드 및 지상통제의 실시간 명령 입력도 유지했다.
- 앱의 네트워크 스모크 시험은 기록·재생 대신 주·보조 드론의 이륙·착륙과 정상 종료를 확인한다.
- Windows Release 전체 정리 빌드 성공. CTest 29/29 통과했으며 SimulatorSession의 CoreScenarios와 ViewerSmoke도 개별 확인했다.
- 헤더 변경 뒤 처음 실행한 증분 빌드 산출물로는 시뮬레이터 시험에서 접근 위반이 났다. 앱의 전체 정리 재빌드 후에는 재현되지 않았다. 따라서 확인된 사실은 낡은 개체 파일 조합과의 연관성이며, 그 접근 위반의 더 깊은 원인은 확정하지 않았다.
- 웹 브라우저 동작은 이번 작업에서 확인하지 않았다.
## 2026-10-29 구형 단일 드론 임무 제거

- Windows Release 전체 빌드 성공.
- SimulatorSession의 `CoreScenarios` 통과, ViewerSmoke 통과.
- 전체 CTest 28/29. `Communication.GroundControlClientTests.CoreScenarios`는 접근 위반으로 실패했다. 같은 시험의 이전 실행에서도 실패 기록이 있어 이번 임무 변경과의 인과는 확인되지 않았다.
- `ProtocolTests.CoreScenarios`는 통신 코드에서 사용하지 않는 ScenarioRunner 헤더를 제거한 뒤 다시 빌드해 통과했다.
- 웹 브라우저 직접 실행은 확인하지 않았다.

## Simulation Core와 시나리오 모듈 분리 — 2026-10-29

- JSON 시나리오 실행기·입력 파서·전용 시험을 `scenario` 모듈로 분리했다. Core 타깃의 소스와 링크 설정에서 시나리오 파일과 JSON 의존성이 빠진 것을 빌드 설정에서 확인했다.
- Release 빌드에서 scenario 시험, simulation 시험, headless 실행기와 simulator 앱이 성공했다.
- 관련 CTest 8/8 통과: 시뮬레이션 코어, 시나리오 실행기, 5개 headless 시나리오, ViewerSmoke.
- 전체 CTest는 이번 변경에서 실행하지 않았다. 웹 브라우저 동작도 확인하지 않았다.

## 자동 촬영 합격 판정 제거 — 2026-10-29

- Windows Release 빌드 성공: `aerosimlab_simulation_tests`, `aerosimlab_scenario_tests`, `aerosimlab_headless`, `aerosimlab_simulator`.
- 관련 CTest 3/3 통과: `Simulation.SimulationCoreTests.CoreScenarios`, `Scenario.ScenarioRunnerTests.CoreScenarios`, `HeadlessFormationOutageSmoke`.
- 코드·시나리오·시험·headless 결과에서 `InspectionSurface`, `CaptureAssessment`, `inspection_surface`, `inspection_occluders`, `max_capture_retries` 참조가 남지 않은 것을 검색으로 확인했다.
- 전체 CTest, 웹 빌드·브라우저 실행은 확인하지 않았다.

## 편대 경로 계산 모듈 분리 — 2026-10-29

- `FormationPlanner` 구현·공개 헤더·시험을 `flight_planning` 경계로 옮기고, `simulation_core` 타깃의 소스 목록에서 제거했다.
- Release 빌드 성공: `aerosimlab_simulation_tests`, `aerosimlab_headless`, `aerosimlab_simulator`.
- 관련 CTest 5/5 통과: SimulationCore, 두 편대 계획 시험, SimulatorSession, ViewerSmoke.
- 전체 CTest와 웹 빌드·브라우저 실행은 확인하지 않았다.

## 배터리 ECS 전환 검토 — 2026-10-29

- `BatteryModel`을 드론 컴포넌트로 저장하는 임시 변경은 Flecs의 기본 생성자 요구로 코어·시나리오 시험에서 중단됐다. 임시 변경을 되돌린 뒤 Release 빌드는 성공했다.
- 원복된 최종 코드에서 관련 CTest 10/10 통과: 코어, 편대 계획 2개, 시나리오, 앱 세션 4개, 헤드리스 임무, ViewerSmoke.
- 전환 코드는 남기지 않았다. 별도 배터리 맵을 유지하기로 한 판단과 이유를 구조 문서에 기록했다.

## 배터리 컴포넌트 재검증 — 2026-10-29

- `BatteryModel`에 Flecs가 요구하는 기본 생성자를 추가하고, 배터리 자료를 드론 엔티티에 직접 붙였다. 별도 `unordered_map`은 제거했다.
- Release 빌드 성공: 시뮬레이션 시험, 시나리오 시험, headless, simulator.
- 코어·편대 계획·시나리오·앱 세션·헤드리스 임무·ViewerSmoke 관련 CTest 10/10 통과. 배터리 시험은 설정 전·후 `HasBattery` 결과도 확인한다.
- 앞서 기록한 실패는 기본 생성자가 없던 상태의 결과다. 이번 변경은 그 조건을 해결했으며, 앞선 “배터리 맵 유지” 결론은 철회한다.

## 구형 단일 드론 점검 흐름 제거 — 2026-10-29

- Release에서 시뮬레이터와 관련 시험 타깃을 연결했다.
- 전체 CTest 29/29 통과. 첫 실행에서 `GroundControlClientTests.CoreScenarios`가 접근 위반을 냈다. 통신 코드는 바꾸지 않고 시험 실행 파일을 다시 빌드한 뒤 전체 시험을 재실행해 통과했고, 해당 시험을 10회 반복해 모두 통과했다. 처음 실패한 빌드 산출물의 정확한 원인은 확인하지 못했다.
- 웹 셸의 두 인라인 JavaScript 구간은 Node 구문 검사에서 통과했다. Emscripten 빌드와 브라우저 동작은 확인하지 않았다.

## 배터리 ECS 조회 정리 — 2026-10-29

- Windows Release 전체 빌드 성공.
- 관련 CTest 6/6, 전체 CTest 29/29 통과.
- 웹 빌드는 이번 변경에서 실행하지 않았다.

## SimulationCore 초기화 순회 정리 — 2026-10-29

- Windows Release 전체 빌드 성공.
- 전체 CTest 29/29 통과. 배터리 잔량 복원, 여러 드론의 카메라 초기화, 물리 상태 초기화가 포함된 관련 시험을 통과했다.
- 웹 빌드는 이번 변경에서 실행하지 않았다.

## 자동 임무 재배정 제거 — 2026-10-29

- Windows Release 전체 빌드 성공.
- 전체 CTest 29/29 통과. 통신 단절 중 담당 드론과 작업이 유지되고 임무 결과가 미확정으로 남는 시험, 저전력 귀환 후 담당 작업이 미완료 처리되는 시험을 확인했다.
- Headless 정상 임무 시험은 `two-point-survey.json`으로 실행했다. 웹 브라우저는 확인하지 않았다.

## 사용하지 않던 Simulation Core 계측 제거 — 2026-10-29

- Windows Release 전체 빌드 성공.
- 전체 CTest 29/29 통과. Flecs, 물리 월드, 시뮬레이터와 ViewerSmoke를 포함한다.
- Flecs·접촉 계측 API 이름의 코드 검색 결과 사용처가 남지 않았다. 사용자 화면에서 읽는 AirSim 물리 시간 측정은 보존했다.

## AddDrone 진입점 통합 — 2026-10-29

- Windows Release 전체 빌드 성공. `AddDrone` API를 호출하는 앱·시나리오·시험 실행 파일까지 다시 연결했다.
- 전체 CTest 29/29 통과.
- 처음에는 오래된 목적 파일이 제거된 2인자 함수 심볼을 참조해 링크에 실패했다. 해당 타깃들의 생성 파일을 다시 만든 후 검증을 마쳤다.

## 전역 바람 상태 중복 제거 — 2026-10-29

- Windows Release 전체 빌드 성공. SimulationCore 시험에 무풍·바람 조건을 비교하는 확인을 추가했다.
- 전체 CTest 29/29 통과.
- 증분 빌드에서는 `DroneControlState` 크기가 바뀐 뒤 이를 읽는 `QuadrotorFlightController.cpp`가 다시 컴파일되지 않아 강체 충돌 회피 시험이 실패했다. 관련 타깃을 정리한 뒤 해당 파일이 새 컴포넌트 배치로 다시 빌드됐고, 시나리오 시험과 전체 CTest가 통과했다.
- 강체 모드 바람 설정은 `PhysicsWorld`만 갱신하고, 운동학 모드 바람 설정은 `SimulationCore`만 갱신한다. Release 전체 빌드와 CTest 29/29 통과.

## 강체 모드 지형 격자 중복 제거 — 2026-10-29

- Windows Release 전체 빌드 성공.
- 전체 CTest 29/29 통과. 운동학 지형 접촉, 강체 물리, 통합 실행과 ViewerSmoke를 확인했다.
- 강체 모드 지형 소유권을 `PhysicsWorld` 하나로 옮겼다. 운동학 모드는 `SimulationCore`의 지형 격자를 그대로 사용한다.

## 사용되지 않던 SimulationCore API 제거 — 2026-10-29

- Windows Release 전체 빌드 성공.
- 전체 CTest 29/29 통과. 로터 고장 물리 동작은 기존 `PhysicsWorld` 시험에서 계속 확인한다.
- 저장소 전체에서 제거한 네 API의 선언 외 호출처가 없음을 다시 검색해 확인했다.

## 자세에서 yaw 계산 통합 — 2026-10-29

- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.
- 강체 상태 복사와 초기 상태 설정에 있던 동일한 쿼터니언→yaw 계산을 `DroneModel::YawRadians()`로 모았다. 두 호출 경로가 새 함수를 사용하고, 저장소에 계산식이 중복되지 않는 것을 확인했다.

## 카메라 자세 ECS 갱신 — 2026-10-29

- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.
- 카메라 자세 갱신이 ID 맵 대신 Flecs의 `CameraState`·`DroneState` 조회를 사용하도록 바꿨다. 엔티티 구성과 갱신 결과를 포함하는 기존 SimulationCore 및 카메라 시험을 통과했다.

## Reset ECS 조회 — 2026-10-29

- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.
- 드론 초기화가 ID 맵 대신 `DroneState`·`DroneControlState`·`CameraState`가 함께 붙은 엔티티를 조회한다. 같은 조회에서 카메라 초기화와 새 자세 반영까지 마쳐 재순회를 없앴다. SimulationCore의 초기 자세·배터리 초기화·다중 드론 생성·제거 시험이 포함된 전체 시험을 통과했다.

## 기본 드론 API 경로 통합 — 2026-10-29

- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.
- 기본 드론의 상태 조회, 카메라 조회·켜기/끄기가 ID 지정 API를 재사용한다. 기본 드론 선택 로직과 ID별 엔티티 조회를 중복 구현하지 않는다.

## 드론 제거 뒤 카메라 재계산 제거 — 2026-10-29

- Windows Release 전체 빌드 성공, 전체 CTest 29/29 통과.
- 주 드론 제거 후 주 식별값만 바뀌고 남은 기체 상태는 그대로이므로 카메라 전체 갱신을 생략했다. 드론 생성·이동·초기화 경로의 카메라 갱신은 보존했다.

## 시나리오 명령 전달 단순화 — 2026-10-29

- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 첫 시험 실행은 예상 비행 시간과 다중 드론 기대 결과가 맞지 않아 실패했고, 수정 후 재실행했다.
- `Scenario.ScenarioRunnerTests.CoreScenarios`와 `HeadlessCommunicationOutageSmoke` 2/2 통과.
- 전체 CTest 28/28 통과. 직접 통신 단절 시험에서 scout 상태가 `stale`로 표시되고, 기준 위치·비행 결과 기대값도 통과했다.
- CTest가 확인한 것은 JSON Runner 동작이다. 앱 가상망과 네이티브 시뮬레이터의 통신 장애 화면은 이번 변경에서 수정하거나 별도 실행하지 않았다.

## 시나리오 결과 상태 정리 — 2026-10-29

- 제거한 `passed` 저장 필드를 사용하는 곳이 남지 않았는지 앱·모듈·시험에서 검색했다. C++ 호출부는 `IsPassed()`를 쓰고, headless 결과 JSON의 기존 `passed` 항목은 그대로 출력한다.
- Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 시나리오 기대 상태 입력 통일 — 2026-10-30

- 단일 드론 시나리오에서는 기대 상태를 `mode`·`position_meters`로 직접 적고, 다중 드론은 `expected.drones` 목록을 쓰던 두 형식을 하나로 합쳤다. 이제 모든 기대 상태에 드론 이름을 포함한 `expected.drones` 목록이 필요하다.
- 저장소의 세 비행 예제와 시나리오 단위 시험 입력을 새 형식으로 바꿨다. 옛 단일 드론 형식을 거부하는 검사도 추가했다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 예약형 카메라 장애 제거 — 2026-10-30

- 기본 임무에 쓰이지 않던 시나리오 전용 카메라 켜기·끄기·고장·복구 시간표를 제거했다. 카메라 상태 및 프레임 수 기대값, 기본 카메라 동작은 남겼다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 시나리오 충돌 위험 추정 단순화 — 2026-10-30

- 경유점에서 멈추는 경우까지 나눠 예측하던 계산을 현재 목표 속도가 일정 시간 유지된다는 단순 계산으로 바꿨다. 회피 목표를 정하는 데 필요한 상대 위치, 속도, 최근접 시각만 남겼다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과. 두 드론 교차, 정지 드론 회피, 강체 비행 회피를 포함한 기존 시나리오 시험이 통과했다.
- 이 검증은 간단한 가상 시나리오 동작만 다룬다. 목표 지점 도착 뒤 감속·정지와 실제 비행 안전성은 시험하지 않았다.

## 임무 요약 상태 중복 제거 — 2026-10-30

- 작업 목록에서 계산할 수 있는 완료 작업 수, 완료율, 경유점 진행률, 전체 완료 Tick을 별도 저장하지 않도록 바꿨다. Headless JSON 출력 항목은 유지했다.
- 임무 일부 완료, 전체 완료, 경유점 진행률 확인을 포함해 Windows Release 전체 CTest 28/28 통과.

## 드론별 순차 임무 처리 — 2026-10-30

- 임무 명령 적용 때 앞선 작업을 매번 다시 검색하던 코드를 목록 한 번만 도는 방식으로 바꿨다. 각 드론은 앞 작업이 완료되거나 실패한 뒤 다음 작업을 처리한다.
- 이름만 `duplicateMissionAssignment`였던 검사는 필수 경유점이 빠진 입력을 던져 실제 배정 규칙을 확인하지 못했다. 유효한 경로 두 개를 같은 드론에 지정해 차례로 끝내는 시험으로 바꾸었다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 해당 시나리오 시험 2/2 및 전체 CTest 28/28 통과.

## 충돌 회피의 임무 검색 통합 — 2026-10-30

- 진행 중인 작업을 찾는 조건을 공통 함수로 모았다. 드론 쌍의 위험 계산과 회피 명령 적용이 같은 작업 기준을 사용한다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 드론별 회피 입력 재사용 — 2026-10-30

- 충돌 회피에서 드론 쌍을 검사할 때마다 연결 상태와 현재 임무를 다시 검색하던 처리를 없앴다. 회피 갱신 시작 때 드론별 정보를 한 번만 모아 쌍별 계산과 회피 명령에서 재사용한다.
- Windows Release에서 `aerosimlab_headless`, `aerosimlab_scenario_tests` 빌드 성공. 전체 CTest 28/28 통과.

## 시나리오 드론별 실행 상태 통합 — 2026-10-30

- 변경: `ScenarioRunner.cpp`의 드론별 ID, 귀환 위치, 관측 상태, 이전 위치, 비행 거리와 시간을 하나의 실행 상태 맵으로 통합했다.
- 빌드: Windows Release에서 `aerosimlab_headless`와 `aerosimlab_scenario_tests` 성공.
- 시험: 전체 Release CTest 28/28 통과.
- 범위: 시나리오 동작과 JSON 형식은 바꾸지 않았다. 웹 실행은 확인하지 않았다.

## 시나리오 설정 읽기와 조합 검증 분리 — 2026-10-30

- 변경: 항목 자체의 JSON 형식과 값 검사는 각 읽기 함수에 두고, 비행 방식·드론 수·배터리 예비량처럼 여러 설정을 함께 봐야 하는 규칙은 읽기가 끝난 뒤 한 단계에서 확인한다. 검사 규칙은 제거하지 않았다.
- 빌드: Windows Release에서 `aerosimlab_headless`와 `aerosimlab_scenario_tests` 성공.
- 시험: 전체 Release CTest 28/28 통과.
- 범위: 기존 시나리오 JSON 형식과 정상 실행 결과를 바꾸지 않았다. 웹 실행은 확인하지 않았다.
