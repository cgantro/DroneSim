# app 소유 파일 감사

이 감사표는 `d6515b6e` 기준 코드의 조사 기록이다. 조종 명령 기록·재생과 앱의 구형 단일 드론 점검 임무는 2026-10-29 제거했다. 현재 상태는 CHECKPOINT와 VERIFICATION의 최신 항목을 따른다.

## 현재 실행 경계 확인 — 2026-10-30

- `apps/ground_control`은 화면 프로그램이 아니라 `dronesim_ground_control_client` 정적 라이브러리다. 명령을 UDP로 보내고 응답을 기다리는 코드는 있지만, 현재 저장소의 실행 파일에서 이 라이브러리를 호출하는 곳은 없다. 따라서 독립 지상통제 프로그램으로는 바로 실행할 수 없다.
- 지금 화면에서 쓰는 지상통제 기능은 시뮬레이터 안의 ImGui 패널과 `DroneSimAppInput.cpp`의 UDP 서버 처리다. 이를 외부 앱용 클라이언트와 혼동하지 않도록 문서·실행 안내에서 구분해야 한다.
- `apps/simulator`는 여러 소스 파일로 나뉘어 있어도 앱 상태를 `DroneSimApp` 하나가 소유한다. `DroneSimAppMission.cpp`에는 편대·통신·임무 흐름이 함께 있고, 헤더에는 그래픽·제어·네트워크·임무 상태가 모여 있다. 파일을 더 쪼개기보다 먼저 각 상태의 소유자와 호출 경계를 줄이는 것이 필요하다.
- EO 화면은 GPU 프레임버퍼 텍스처를 패널에 표시한다. CPU 픽셀 읽기·화질 분석·종료 drain을 제거하고, 남은 장면·카메라 렌더링 코드를 역할에 맞는 이름으로 정리했다.

기준: d6515b6e와 당시 미커밋 변경을 포함한 작업 트리, INVENTORY.tsv의 owner=app 32개. 이 기록의 초기 로컬 녹화 관련 평가는 2026-10-29 사용자 요청으로 뒤집혔다. 카메라 파일 녹화·저장은 제거하고 라이브 EO 화면은 유지한다. 아래 파일별 조사 내용은 당시 코드 기준이며 이후 변경은 CHECKPOINT와 VERIFICATION에서 확인한다.

## 현재 작업 트리 대조 — 2026-10-29

현재 `apps/`에는 28개 파일이 있다. 위 표는 이전 감사 시점의 32개 목록이므로 삭제된 항목을 포함하며, 현재 목록과 섞어 읽지 않는다. 이번 시나리오 재검토에서 다시 확인한 앱 경계는 다음과 같다.

- `apps/headless/main.cpp`는 JSON 파일을 읽어 `scenario::RunScenario`를 실행하고 결과를 JSON으로 출력한다.
- `apps/simulator/DroneSimAppMission.cpp`는 화면 시뮬레이터의 편대·정찰·복귀 임무를 직접 진행한다. JSON Runner를 호출하지 않는다.
- 따라서 시나리오 자동 시험은 headless 경로를 검증하며, 화면에서 수행하는 앱 임무를 대신 검증하지 않는다. 앱 시험은 기존 `tests/simulator/SimulatorSessionTests.cpp` 및 Viewer 실행 경로를 별도로 확인해야 한다.

| 경로 | 판단 | 확인된 입출력·호출·상태/오류·중복 또는 범위 판단 |
|---|---|---|
| apps/simulator/DroneSimApp.cpp | SIMPLIFY | 생성자 178, 그래픽 초기화 271–350, 지형/시설 361–438, 4대 생성 441–466, tick 498, RenderFrame 678, 카메라 941/982. 자산과 simulation을 받아 렌더링·상태에 연결하고 unique_ptr로 그래픽 객체 소유. 미션·네트워크·웹 분기를 목표에 맞게 축소. CreateTerrainMesh 120–153은 격자 수 2 이상에 의존하므로 생산자 불변식 확인 필요. |
| apps/simulator/DroneSimApp.h | SIMPLIFY | 기준선에는 구형 점검 상태와 네 대 정찰·전체 임무 상태가 함께 있었다. 구형 상태·목표·카메라 판정 자료를 제거하고 정찰 지점 자료만 남겼다. 앱은 여전히 그래픽·제어·통신·임무를 함께 소유하므로 책임 검토가 남았다. |
| apps/simulator/DroneSimAppBenchmark.cpp | SIMPLIFY | RecordBenchmarkFrame 15, WriteBenchmarkReport 70: 화면 계측을 보고서로 출력한다. 고정 임무 범위에서 쓰지 않는 대규모 확장 측정은 제거하고 화면 측정만 남겼다. |
| apps/simulator/DroneSimAppCamera.cpp | MOVED | 2026-10-30 패널 카메라·프레임버퍼 책임을 `modules/gui/SimulatorView.cpp`로 옮기고 옛 앱 전용 파일을 삭제했다. |
| apps/simulator/DroneSimAppGamepad.cpp | KEEP | ScanForGamepad 27, PollGamepad 50, HandleGamepadDisconnected 118. 장치 상태를 입력으로 조종 명령을 만들고 연결 해제를 처리한다. 독립 기능이지만 MVP에서 수동 조종 필요 여부는 별도 결정. |
| apps/simulator/DroneSimAppInput.cpp | SIMPLIFY | 당시 PollGroundControl 184, ProcessPendingGroundCommand 727, CurrentTelemetry 840이 UDP 명령과 상태 응답을 처리했다. 기록·재생 제어는 이후 제거했다. 목표 임무와 직접 통신은 보존한다. |
| apps/simulator/DroneSimAppLifecycle.cpp | SIMPLIFY | Run 28, RunFrame 139, SmokeTestIsComplete 302, ValidateSmokeTestOutput 313. 프레임 입력→모의→렌더→swap을 묶고 웹 예외 및 smoke 시간 초과를 처리한다. 벤치마크·웹/네이티브 분기 정리는 EO 검증과 종료 경로를 보존하며 수행. |
| apps/simulator/DroneSimAppMission.cpp | SIMPLIFY | 기준선에는 단일 드론의 이륙·접근·카메라 정렬·촬영·착륙 처리와 네 대 임무가 함께 있었다. 구형 단일 드론 흐름을 제거했다. 남은 흐름은 편대 이동·드론별 정찰·재집결·복귀를 맡으며, 메시지 처리와 임무 판정은 아직 한 파일에 모여 있다. |
| apps/simulator/BenchmarkProcessMemory.h | SIMPLIFY | ProcessMemorySample 14, ReadProcessMemorySample 21은 프로세스 메모리 계측에 선택적 결과를 반환한다. 벤치마크 보존 여부와 함께 결정. |
| apps/simulator/BuildRevision.cpp | KEEP | 빌드 리비전 제공 구현. 호출·생성 연결과 실패 시 값은 추가 확인 필요. |
| apps/simulator/BuildRevision.h | KEEP | 리비전 인터페이스. 빌드 생성 및 표시 호출처 확인 필요. |
| apps/simulator/CooperativeMissionBenchmark.cpp | REMOVED | 2026-10-29: 확장 임무·재할당을 반복 측정하던 벤치마크 파일을 제거했다. 4대 기본 임무의 동작 시험은 별도 CTest로 유지한다. |
| apps/simulator/DroneScalingBenchmark.cpp | REMOVED | 2026-10-29: 1~1024대 드론 생성·메모리 확장 벤치마크 파일을 제거했다. 현재 목표는 4대 운용이다. |
| apps/simulator/EoVideoRecorder.cpp | REMOVE | 카메라 프레임을 MP4로 저장하던 구현. 2026-10-29 요청에 따라 녹화와 파일 저장을 제거하고 CMake·호출처와 함께 삭제했다. |
| apps/simulator/EoVideoRecorder.h | REMOVE | 카메라 녹화 API. 2026-10-29 요청에 따라 구현 및 호출처와 함께 삭제했다. |
| apps/simulator/ExecutableDirectory.h | KEEP | ExecutableDirectory 16은 실행 파일 기준 자산 경로를 제공한다. 플랫폼 오류 반환 동작 추가 확인 필요. |
| apps/simulator/FaultScenarioTimeline.cpp | REMOVE | StartRecording 116, LoadRecording 210, ScheduleEvent 336, TakeDueEvents 525. 장애 목록의 저장·검증·예약·재생과 상태를 보유한다. 로드 입력에 ID·시간·범위 검증 229–300이 있으나 장애 시간표 자체가 목표 밖. HUD와 입력 및 시험을 함께 제거. |
| apps/simulator/FaultScenarioTimeline.h | REMOVE | Event 18, Kind 20과 시간표 상태/API는 위 구현의 계약. 호출처 제거와 동시에 정리. |
| apps/simulator/FleetDroneSpawn.cpp | SIMPLIFY | FindFleetDroneSpawnPose 9: 지형을 입력받아 선택적 생성 위치를 반환한다. 4대 고정 시작 위치가 App.cpp 441–466에 이미 있으므로 동적 추가 경로와 책임 중복 여부를 확인하고 정리. |
| apps/simulator/FleetDroneSpawn.h | SIMPLIFY | FleetDroneSpawnPose 13 및 위치 탐색 계약. 동적 추가 제거 결정에 따라 호출처와 함께 정리. |
| apps/simulator/FleetMissionProgress.cpp | KEEP | 임무 진행 판정 구현. 현재 로컬 수정본으로 Mission.cpp의 상태 집계와 역할을 비교하고 M01/M02/M03 완료·착륙 조건을 검증해야 한다. |
| apps/simulator/FleetMissionProgress.h | KEEP | FleetMissionLeg 13, FleetDroneStatus 23, FleetMissionReport 61, FleetMissionProgressDecision 77. 진행 판정 데이터 계약. 현재 로컬 수정본. |
| apps/simulator/FleetTelemetry.h | SIMPLIFY | PeerTelemetry 13. 동료 상태 데이터 계약. 직접 통신에 필요한 필드 유지, 중계·선출 전용 필드 확인 후 제거. |
| apps/simulator/SimulationBenchmark.cpp | SIMPLIFY | Percentile 23, SummarizeDurations 50, BuildViewerBenchmarkReport 61. 측정값을 JSON 보고서로 변환. 가변 대수·장애 계측과 앱 계측 의존을 분리해 필요한 검증만 보존. |
| apps/simulator/SimulationBenchmark.h | SIMPLIFY | ViewerBenchmarkMeasurements를 선언한다. 드론 수 확장 벤치마크의 입력 형식과 함수 선언을 제거하고 화면 측정 결과만 남겼다. |
| apps/simulator/SimulatorHud.cpp | SIMPLIFY | HUD가 사용자 입력을 앱 제어에 연결하고 장면·EO·상태를 그린다. 명령 기록 UI와 카메라 영상 녹화 UI는 모두 제거했다. |
| apps/simulator/SimulatorHud.h | SIMPLIFY | SessionRecordingControls 47, SimulatorHud 85, PendingGpuQuery 124. UI 입력과 콜백/계측 상태 계약. 구현의 제거 대상과 동기화하고 GPU 쿼리 수명 확인. |
| apps/simulator/TrainingRangeScene.cpp | KEEP | Load 68은 자산과 지형·simulation을 받아 시설·장애물·정찰 지점을 만들고, Draw 124가 renderer에 전달한다. Obstacles 134, ReconPoints 140으로 임무에 연결. 자산 로드 실패 처리 추가 확인 필요. |
| apps/simulator/TrainingRangeScene.h | KEEP | ReconPoint 27, ModelGroup 50, TrainingRangeScene 24. 장면 모델과 정찰 위치를 소유해 렌더링 및 경로 계획에 제공한다. |
| apps/simulator/main.cpp | SIMPLIFY | RunSimulator 73, WinMain 182, main 210. 시작 옵션을 읽고 앱을 실행하며 시작 오류를 표시한다. 제거할 벤치마크·전송 옵션과 도움말을 함께 정리. |
| apps/simulator/web/web_shell_single_thread.html | SIMPLIFY | 웹 입력·상태와 카메라 패널을 연결한다. 2026-10-29 요청에 따라 브라우저 녹화·저장·WebRTC를 제거했다. 페이지 안에서 시뮬레이터 캔버스 일부를 카메라 패널에 복사한다. 실제 브라우저 검증은 별도 수행이 필요하다. |
| tests/simulator/SimulatorSessionTests.cpp | SIMPLIFY | 현재 4대 임무·직접 통신과 UDP 비행 명령을 시험한다. 명령 기록·재생과 카메라 저장 시험은 제거했다. |

## 우선 확인할 위험과 범위 제한

- 리더십·메시가 임무 진행과 뒤섞여 있어 파일 단위 삭제는 M01→M02→M03과 안전 착륙을 끊을 수 있다. 직접 통신 전환 후 4대 각 보고와 복귀 조건을 실제로 검증해야 한다.
- EO 렌더 readback, 인코더 스레드 및 종료 처리, 브라우저 저장이 서로 연결된다. 전송 제거 시 로컬 영상이 계속 저장되는지 네이티브·웹 각각 검증해야 한다.
- 동적 드론 생성과 장애 시간표는 UI·프로토콜·시험·빌드 참조를 확인한 뒤 단계별로 정리한다. 확장성 벤치마크는 실제 빌드·도움말 호출을 함께 제거했다.
- 32개 경로를 끝까지 읽었다. 호출처 전체와 실제 실행은 확인하지 않았으므로 판정은 정적 감사에 한정한다.

## 실제 본문 확인 구간

아래 범위만 연속 본문 확인을 주장한다. 함수명 검색은 본문 완독으로 세지 않는다. 첫 묶음 출력이 잘렸으므로 보수적으로 분류했다.

- `DroneSimApp.cpp`: 1–469, 470–700, 701–930, 931–1150, 1151–1264 확인.
- `DroneSimAppInput.cpp`: 1–250, 251–500, 501–726, 727–863 확인.
- `DroneSimAppLifecycle.cpp`: 1–337 확인, 미확인 없음.
- 다음 11개는 전체 확인: BenchmarkProcessMemory.h 1–36, BuildRevision.cpp 1–11, BuildRevision.h 1–9, ExecutableDirectory.h 1–36, FleetDroneSpawn.cpp 1–37, FleetDroneSpawn.h 1–24, FleetMissionProgress.cpp 1–119, FleetMissionProgress.h 1–92, FleetTelemetry.h 1–33, TrainingRangeScene.cpp 1–149, TrainingRangeScene.h 1–59.

추가 완독: DroneSimApp.h 1–220, 221–440, 441–604; DroneSimAppBenchmark.cpp 1–131; DroneSimAppEoCapture.cpp 1–150, 151–299; DroneSimAppGamepad.cpp 1–188. EoVideoRecorder.cpp 1–205, 205–659 확인. 추가 완독: DroneSimAppMission.cpp 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1500, 1501–1750, 1751–2000, 2001–2240; CooperativeMissionBenchmark.cpp 1–200, 201–400, 401–563; DroneScalingBenchmark.cpp 1–240, 241–480. 추가 완독: EoVideoRecorder.h 1–90; FaultScenarioTimeline.cpp 1–200, 201–400, 401–583; FaultScenarioTimeline.h 1–101; SimulationBenchmark.cpp 1–204; SimulationBenchmark.h 1–129. 추가 완독: SimulatorHud.cpp 1–250, 251–500, 501–750, 751–1000, 1001–1203; SimulatorHud.h 1–143; main.cpp 1–214; web/web_shell_single_thread.html 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1335; tests/simulator/SimulatorSessionTests.cpp 1–250, 251–500, 501–750, 751–941. 따라서 완독 32/32개, 미완독·누락 없음.

추가 확인: FleetMissionProgress.cpp 63–117은 보고 누락이나 단계 불일치 시 대기하고 정찰·복귀 실패 및 착륙을 판정한다. 상태를 한국어 문자열로 파싱하므로 문구 변경이 판정을 바꿀 수 있다(5–39). TrainingRangeScene.cpp 53–57의 첫 네 정찰 지점은 다중 홉 연결을 전제로 배치됐으므로 직접 통신 목표에 맞는지 재검토해야 한다. ExecutableDirectory.h 20–32는 경로 조회 실패 시 예외를 던지고, FleetDroneSpawn.cpp 21–28은 지형 범위 밖을 nullopt로 반환한다.


추가 분석(기준선): EoVideoRecorder::Stop의 카운터 경쟁을 확인해 `7bfce7e7`에서 수정했다. 이후 2026-10-29 카메라 파일 저장 기능을 제거했으므로 이 구현과 종료 경로는 현재 존재하지 않는다. 당시 AppInput의 상태 요청은 라이브 영상과 저장 파일 전송을 함께 시작했으나, 현재 저장 파일 전송은 제거되고 라이브 영상만 남는다. 나머지 줄 번호는 변경 전 기준이다.


마지막 다섯 파일의 범위도 위 구간에서 확인했다.

추가 분석: CompleteFleetMissionCanStart 333–350은 네 대 고정이 아닌 두 대 이상을 허용한다. StartFormationTransit 1173–1247은 경로에 따라 일렬 전환을 허용하고 PrepareFleetNetwork 1274–1283은 최대 5홉을 설정한다. M02 후 재집결과 M03은 1128–1146에서 연결되며, 보고 누락 시 1122–1125에서 대기한다. 이전 감사에서는 CooperativeMissionBenchmark.cpp와 DroneScalingBenchmark.cpp가 장애·재배정 임무 및 1–1024대 계측을 맡는 것을 확인했다. 2026-10-29에 이 두 대규모 성능 시험 구현과 앱 보고서 호출부를 제거했다. 이후에는 네 대 목표에 직접 필요한 성능 지표만 따로 선정한다.


이번 확인의 보강(기준선): EoVideoRecorder의 프레임 수·체크섬·전달 완료 API는 카메라 파일 저장과 함께 2026-10-29 제거했다. FaultScenarioTimeline.cpp 210–315는 JSON을 임시 지역 값으로 검증한 후 상태에 반영하고 오류를 문자열로 반환한다. 336–446은 시작·복구 이벤트를 정렬 삽입하며 시간 범위를 검사한다. 525–561은 현재 Tick에 도달한 이벤트와 대기 목록을 출력한다. 동일 장애 입력의 HUD 콜백과 AppInput 경로를 함께 제거해야 한다. SimulationBenchmark.cpp 50–58은 빈 표본을 0과 표본 수 0으로 보고하고, 184–196은 JSON 파일 열기·쓰기 오류를 예외로 반환한다. 71–74의 고정 설정 문자열은 현재 네 대 생성 및 훈련장 장면과 불일치한다. 성능 수치 자체는 측정하지 않았다.


마지막 확인 당시 SimulatorHud.cpp는 장면과 각 드론 카메라 화면을 렌더 콜백으로 표시했다. 2026-10-29 요청으로 웹 WebRTC와 네이티브 RTP 송수신을 제거했다. 웹 셸은 페이지 안에서 시뮬레이터 캔버스 일부를 같은 창 카메라 패널로 복사한다. 이전 파일별 줄 번호는 감사 기준 시점의 내용이며, 현재 호출 흐름의 증거로 쓰지 않는다.

## 화면·실행·측정 파일 재감사 — 2026-10-30

기준은 `b1c81dc8` 작업 트리다. 아래 11개 파일의 전체 본문과 CMake 연결, 관련 호출·시험을 읽었다. 정적 조사이며 이번 조사에서 빌드, 시험, 실제 화면 실행은 하지 않았다. 네이티브 HUD는 시뮬레이터 앱의 네이티브 빌드에만 포함되고, 웹 셸은 Emscripten 실행 파일의 HTML 틀로 쓰인다.

| 파일 | 현재 역할과 호출 | 책임·중복 판단 |
|---|---|---|
| `DroneSimAppBenchmark.cpp` | 앱의 프레임·느린 프레임 표본을 모으고 완료된 HUD GPU 시간과 결합한다. 종료 시 렌더러 정보, Tick·프레임 수와 CPU 시간을 모아 보고서 작성·저장을 호출한다. `DroneSimAppLifecycle.cpp`가 측정 실행 때 호출한다. | 앱의 계측값을 보고서 입력으로 옮기는 역할이다. 화면 그리기와 CPU 구간 측정값을 앱 여러 곳에서 받아 필드가 많지만 실제 보고서에 사용한다. GPU 결과는 프레임 번호로 늦게 도착하므로 별도 결합이 필요하다. 측정 항목을 줄일지는 요구하는 지표와 기존 결과의 용도를 대조해 결정한다. |
| `BenchmarkProcessMemory.h` | Windows에서 현재 프로세스의 작업 집합과 전용 메모리를 읽고 실패하거나 다른 환경이면 빈 값을 반환한다. `SimulationBenchmark.cpp`만 호출하며 Windows 빌드는 Psapi를 연결한다. | 단일 호출처를 위한 작은 플랫폼 함수다. 측정 항목을 유지하면 유지하고, 파일 수를 줄일 필요가 있으면 보고서 구현에 합칠 수 있다. 다른 플랫폼에서 값이 없다는 점은 현재 코드의 동작이다. |
| `BuildRevision.h` | 소스 버전을 돌려주는 `BuildRevision()`을 선언한다. | 앱 계측 코드가 생성 헤더를 직접 알지 않게 하는 얇은 경계다. 호출은 보고서 작성 한 곳이다. |
| `BuildRevision.cpp` | CMake가 생성한 헤더의 버전 문자열을 반환한다. | 함수 자체는 단순 전달이다. 두 파일을 없애려면 생성 헤더의 빌드 의존성과 변경 시 재컴파일도 함께 유지해야 한다. 단순히 호출이 한 곳이라는 이유만으로 제거하지 않는다. |
| `ExecutableDirectory.h` | Windows 실행 파일 경로, Linux `/proc/self/exe`, 웹의 빈 경로를 처리한다. 자산 경로와 Windows 기본 벤치마크 출력 경로가 사용한다. | 두 기능이 같은 실행 파일 위치를 필요로 하므로 공유할 근거가 있다. `DroneSimAppInput.cpp`의 이 헤더 포함은 실제 사용처가 없는 후보이다. 경로 조회 실패는 예외로 알린다. |
| `main.cpp` | 명령행 옵션을 해석하고 플랫폼별 시작 오류를 표시한 뒤 앱을 생성·실행한다. Windows에서는 유니코드 명령행을 UTF-8로 바꾸고 웹에서는 프레임 루프 동안 앱 수명을 유지한다. CMake가 실행 파일 진입점으로 사용한다. | 실행 진입점의 책임에 맞는다. 옵션 해석과 오류 표시, 플랫폼 진입이 한 파일에 있어 흐름을 읽어야 하지만 별도 클래스는 필요하지 않다. 옵션을 순서대로 처리하므로 `--smoke-test` 뒤에 `--udp-port`를 지정하면 UDP가 다시 활성화된다. 의도한 조합인지 확인이 필요하다. |
| `SimulationBenchmark.h` | HUD GPU 시간과 한 실행의 화면·Tick·느린 프레임 표본, JSON 결과 생성·저장 계약을 선언한다. 앱 계측과 HUD가 자료형을 사용한다. | 여러 구간의 상세 측정값을 모으는 자료 계약으로 역할에 맞지만 필드 수가 많다. `SummarizeDurations()`는 이 헤더 밖의 호출처가 없어 구현 내부 함수로 좁힐 후보이다. |
| `SimulationBenchmark.cpp` | 표본을 정렬해 p50·p95·p99·최댓값과 Tick 초과 횟수를 계산하고 JSON으로 저장한다. `DroneSimAppBenchmark.cpp`가 보고서 생성과 저장을 호출한다. | 현재 출력의 `configuration`은 드론 한 대와 예전 SRTM 지형·건물을 고정 문구로 기록해 현재 훈련장·네 대 초기화와 맞지 않는다. 이 문구를 현재 실행 조건으로 바꾸거나 확인되지 않은 세부 내용을 제거해야 한다. 이 보고서 함수 전용 시험은 검색에서 찾지 못했다. |
| `SimulatorHud.h` | HUD 생성·소멸과 화면 렌더 함수, 지상통제 수신 위치, HUD CPU·GPU 계측 자료를 선언한다. 앱이 소유하고 호출한다. | ImGui 수명과 GPU 쿼리 수명을 객체에 묶고 복사를 막는다. `Render()` 인자가 상태·명령 콜백·렌더 콜백·계측을 한 번에 받아 길다. `GroundControlDronePosition::droneId`는 채워지지만 HUD에서 읽히지 않는다. 현재는 앱과 HUD가 같은 `DroneIds()` 순서로 목록을 만들어 인덱스로 연결한다. |
| `SimulatorHud.cpp` | 도킹 배치, 관찰 드론 선택, 드론별 2열 카메라, 비행·관제 상태, 임무 시작·복귀와 가상 통신 차단·복구를 표시한다. ImGui 초기화·해제와 선택적인 GPU 시간 조회도 맡는다. | 화면 출력과 UI 입력 처리는 HUD 책임에 맞고 생성·소멸에서 ImGui와 GPU 쿼리 자원을 정리한다. 통신 사건의 송신·수신 ID를 모두 `DroneName()`으로 표시하므로 지상통제 노드의 최댓값 ID도 드론 번호로 표시된다. 화면에는 드론별 배터리, 임무 일시정지·중단 조작이 없다. 기존 ViewerSmoke는 이 세부 UI를 확인하지 않는다. |
| `web/web_shell_single_thread.html` | 웹 페이지의 장면·카메라·상태·조작 UI와 시작 상태, C++ 웹 함수 호출, EO 캔버스 복사를 묶는다. CMake의 Emscripten `--shell-file`이 사용하고 웹 스모크 도구가 시작·카메라 표시·수동 조작과 임무 일부를 확인한다. | 브라우저 의존 작업을 한곳에서 처리한다. HTML·CSS·JS가 함께 있어 길지만 이 파일은 배포 셸이다. EO 캔버스 하나와 주 드론 상태만 제공하며, 네이티브의 드론별 카메라 격자와 다르다. 페이지 안 `#scene-panel canvas`의 150% 너비는 C++이 그린 장면과 카메라 영역을 같은 캔버스에서 분리하는 구조와 함께 검토해야 하므로 단독 삭제 후보로 보지 않는다. |

추가 경계: 네이티브 HUD는 지상통제의 마지막 수신 위치와 실제 위치를 함께 표시하지만, 웹 UI는 주 드론의 현재 상태를 사용한다. 웹의 EO 표시와 조작이 실제 브라우저에서 정상인지 이번에는 실행하지 않았다. 파일 헤더는 현재 `apps/simulator`에 구현과 함께 놓여 있다. 헤더를 `include`로 옮길 때는 앱과 시험의 CMake include 경로를 함께 바꿔야 한다.

## 구형 단일 드론 점검 흐름 제거 — 2026-10-29

- 앱의 구형 단일 드론 점검 상태, 카메라 방향 맞춤·촬영 대기 단계, 점검면·가림 자료, 해당 경로 표시와 웹 시작·멈춤·재개 조작을 제거했다. 네 대가 편대 이동·분산 정찰·복귀를 하는 전체 임무는 남겼다.
- 정찰 지점 자료는 이름과 접근 위치만 가진다. 카메라 영상은 표시하고 화질을 읽되, 앱이 촬영 성공을 판정하지 않는다.
- Windows Release 앱·시험 타깃 링크 성공. 전체 CTest 29/29 통과. 관제 클라이언트 시험은 첫 실행에서 접근 위반이 났지만, 시험 실행 파일을 다시 빌드한 뒤 통과했고 같은 시험을 10회 반복해 모두 통과했다. 최초 실패 원인은 정확히 특정하지 못했다.
- 웹 셸의 인라인 JavaScript 구문 검사는 통과했다. Emscripten 빌드와 브라우저 실행은 확인하지 않았다.

## 앱 상태·임무·장면 파일 재감사 — 2026-10-30

기준은 `b1c81dc8` 작업 트리다. 아래 14개 파일을 끝까지 읽고 CMake 등록, 앱 내부 호출, 관련 시뮬레이터 시험을 확인했다. 이 절은 현재 코드에 대한 정적 조사다. 빌드·시험·실제 화면 실행은 이번 조사에서 하지 않았다. 위의 이전 감사는 당시 기록으로 남기며, 이전에 쓰인 영상 화질 분석은 `b1c81dc8`에서 제거했다.

| 파일 | 현재 역할과 판단 |
|---|---|
| `DroneSimApp.h` | 앱의 그래픽 자원, 입력, 임무, 가상 통신과 측정 상태를 소유한다. 자원은 주로 `unique_ptr`로 관리하지만 여러 책임과 임무 상태가 한 클래스에 모여 있다. `<unordered_map>`은 네이티브 조건에서만 포함하면서 웹에서도 해당 자료형을 사용한다. 다른 헤더의 간접 포함에 의존하지 않도록 확인이 필요하다. |
| `DroneSimApp.cpp` | 창·모델·지형·네 대 드론 초기화, 고정 Tick, 수동 조종과 장면 출력을 연결한다. `AdvanceSimulationTick()`의 정찰 진행 조건은 `FleetReconnaissanceIsActive()`와 같은 내용을 따로 계산한다. `ApplySimulationCommand()`는 현재 `simulation_.Apply()`에 그대로 전달한다. |
| `DroneSimAppMission.cpp` | 편대 이동, 드론별 정찰, 집결·복귀, 가상 통신 전송·수신·재전송과 단계 판정을 맡는다. 한 파일에 서로 다른 책임이 모여 있다. `FleetReconnaissanceStatus()`는 선언·정의 외 호출처가 없다. 이 함수에서만 읽는 `fleetLeadershipStatus_`와 상태 보고 경과 기준 상수도 제거 후보이다. 재전송 횟수는 재시도 제한에 사용한다. |
| `DroneSimAppInput.cpp` | UDP 요청을 해독하고 순서·중복을 확인해 명령을 적용하며 응답을 만든다. 드론 추가와 정찰 시작도 이 경로에서 처리한다. `ExecutableDirectory.h`와 `graphics/MultisampleFramebuffer.h`는 이 파일에서 사용처를 찾지 못했다. |
| `DroneSimAppLifecycle.cpp` | 네이티브 실행 루프, 웹 단계별 시작, 프레임 입력·모의·렌더링과 스모크 종료를 관리한다. 실행 수명을 맡는 역할에 맞는다. 웹 상태 전송 블록의 `camera` 지역 참조는 현재 읽히지 않는다. |
| `DroneSimAppCamera.cpp` | 당시 네이티브 장면과 드론별 카메라를 그렸다. 2026-10-30 카메라와 프레임버퍼 소유를 `modules/gui/SimulatorView.cpp`로 옮겼다. 웹 렌더링은 Emscripten 전용 경로로 앱에 남는다. |
| `DroneSimAppGamepad.cpp` | GLFW 게임패드 탐색, 버튼 변화 감지, 축 입력과 연결 해제 처리를 맡는다. 축을 비행 명령으로 바꾸는 계산은 `manual_control`에 있어 입력 수집과 계산이 구분된다. |
| `FleetDroneSpawn.h` | 추가 드론의 지면 위치와 높이 결과, 위치 탐색 함수의 계약이다. |
| `FleetDroneSpawn.cpp` | 주 드론 주변 슬롯의 지면 위치를 계산한다. UDP 드론 추가와 관련 시험에서 사용하므로 현재 미사용 파일은 아니다. 네 대 고정 정책을 적용할 때 호출처와 함께 검토한다. |
| `FleetMissionProgress.h` | 드론 임무 단계·상태와 보고·전체 진행 판정 자료를 선언한다. 표시 문구와 드론 상태 enum을 분리한다. |
| `FleetMissionProgress.cpp` | 드론 상태의 화면 문구를 만들고 모든 보고를 모아 정찰·집결·복귀의 대기·진행·실패를 판정한다. 관련 상태 및 착륙 조건 시험이 있다. |
| `FleetTelemetry.h` | 가상망에서 주고받는 드론 상태와 인코더·디코더 계약이다. `PeerTelemetry::assignmentLeaseExpiresAtTick`은 디코딩 후 앱에서 읽지 않고 왕복 시험에서만 확인한다. 경로 배정 메시지의 만료 시각과 구분해야 한다. |
| `TrainingRangeScene.h` | 훈련장 모델, 경로용 장애물과 정찰 지점을 소유한다. 모델 포인터 하나만 담는 `ModelGroup`은 작은 단순화 후보이다. |
| `TrainingRangeScene.cpp` | 하나의 배치 목록에서 GLB 인스턴스와 물리 충돌 상자·경로 장애물을 만들고 정찰 위치를 제공한다. 현재 앱은 장면 객체를 새로 만들어 한 번 `Load()`한다. 같은 객체의 재호출 동작은 이번 조사에서 확인하지 않았다. |

### 현재 코드와 요구사항 대조

- M01 편대 이동은 별도 `Phase` 값으로 도착·실패를 판정하도록 바꿨다. 화면 문구를 바꿔도 임무 진행 판정은 바뀌지 않으며, 기존 통합 시험에서 이 동작을 확인했다.
- 앱과 통신 규약의 런타임 `AddDrone` 명령 및 전용 배치 계산을 제거했다. 기본 임무는 네 대로 고정하고, 통신 상태 목록도 최대 네 대로 제한한다. `SimulationCore::AddDrone()`은 시뮬레이션 코어 자체의 여러 기체 시험에 쓰이므로 유지했다.
- 정찰 경로를 이미 받은 드론의 경유점 진행은 가상 링크 상태를 검사하지 않는다. 전체 임무는 끊긴 드론의 새 보고를 기다린다. M01 편대 이동에서는 보고가 없으면 즉시 실패하고, 오래된 보고는 일정 시간 회복을 기다린 뒤 실패한다. 실패 시 현재 코드는 모든 비행 기체에 착륙을 직접 적용하며 관련 시험도 이 동작을 기대한다. 정찰 중 경로 계속 수행과 편대 단계의 안전 착륙은 구분해 정책을 판단해야 한다.
- UDP는 외부 지상통제 프로그램이 시뮬레이터에 보내는 경로다. 이 경로의 드론 명령은 가상 드론 링크를 검사하지 않고 모의 상태에 적용하며, UDP 상태 응답도 실제 현재 상태를 읽는다. 앱 HUD의 지상통제 위치는 가상망으로 마지막에 받은 위치를 따로 표시한다. 외부 UDP 연결과 가상 드론 링크는 서로 다른 경계이므로, 통신 단절 시 명령과 상태를 어떻게 다룰지는 두 경로를 구분해 결정해야 한다.

기준선 이후 M01 판정과 런타임 드론 추가를 정리하고 통신 시험 7개와 시뮬레이터 통합 시험 5개를 실행해 통과했다. 이 작업에서는 실제 화면 배치나 웹 빌드를 확인하지 않았다. 헤더 위치와 include 경로는 아래 후속 정리에서 갱신했다.

### 후속 정리

- 앱 헤더 9개를 `apps/simulator/include`로 옮기고 CMake 공개 포함 경로를 바꿨다. `DroneSimApp.h`에서 웹 빌드에도 쓰는 `unordered_map`을 직접 포함하도록 수정했다.
- 입력 처리 파일에서 사용하지 않던 헤더 2개와 웹 상태 전송에서 읽지 않던 카메라 참조를 제거했다.
- 편대 상태 패킷에서 수신 측이 쓰지 않던 배정 만료 시각을 제거했다. 실제 경로 배정 메시지의 만료 시각과 만료 시 착륙 동작은 유지했다.
- Release 앱·통합시험 빌드 성공, 통신 시험 7/7 및 시뮬레이터 통합 시험 5/5 통과. Emscripten 빌드와 화면 실행은 확인하지 않았다.

