# 빌드와 시험 기록

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
