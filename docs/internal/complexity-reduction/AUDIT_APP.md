# app 소유 파일 감사

이 감사표는 `d6515b6e` 기준 코드의 조사 기록이다. 조종 명령 기록·재생과 앱의 구형 단일 드론 점검 임무는 2026-10-29 제거했다. 현재 상태는 CHECKPOINT와 VERIFICATION의 최신 항목을 따른다.

기준: d6515b6e와 당시 미커밋 변경을 포함한 작업 트리, INVENTORY.tsv의 owner=app 32개. 이 기록의 초기 로컬 녹화 관련 평가는 2026-10-29 사용자 요청으로 뒤집혔다. 카메라 파일 녹화·저장은 제거하고 라이브 EO 화면은 유지한다. 아래 파일별 조사 내용은 당시 코드 기준이며 이후 변경은 CHECKPOINT와 VERIFICATION에서 확인한다.

| 경로 | 판단 | 확인된 입출력·호출·상태/오류·중복 또는 범위 판단 |
|---|---|---|
| apps/simulator/AeroSimLabApp.cpp | SIMPLIFY | 생성자 178, 그래픽 초기화 271–350, 지형/시설 361–438, 4대 생성 441–466, tick 498, RenderFrame 678, 카메라 941/982. 자산과 simulation을 받아 렌더링·상태에 연결하고 unique_ptr로 그래픽 객체 소유. 미션·네트워크·웹 분기를 목표에 맞게 축소. CreateTerrainMesh 120–153은 격자 수 2 이상에 의존하므로 생산자 불변식 확인 필요. |
| apps/simulator/AeroSimLabApp.h | SIMPLIFY | 기준선에는 구형 점검 상태와 네 대 정찰·전체 임무 상태가 함께 있었다. 구형 상태·목표·카메라 판정 자료를 제거하고 정찰 지점 자료만 남겼다. 앱은 여전히 그래픽·제어·통신·임무를 함께 소유하므로 책임 검토가 남았다. |
| apps/simulator/AeroSimLabAppBenchmark.cpp | SIMPLIFY | RecordBenchmarkFrame 15, WriteBenchmarkReport 70: 화면 계측을 보고서로 출력한다. 고정 임무 범위에서 쓰지 않는 대규모 확장 측정은 제거하고 화면 측정만 남겼다. |
| apps/simulator/AeroSimLabAppEoCapture.cpp | SIMPLIFY | RenderScenePanel 16, CaptureEoFrame 99, ProcessCompletedEoReadback 105, QueueEoReadback 159, ProcessEoPixels 200, DrainEoReadbacks 281. 렌더 대상에서 픽셀을 읽어 라이브 화면과 EO 화질 정보를 만든다. 구형 촬영 구도 판정과 연결하던 카메라 상태 사본은 제거했다. 비동기 readback 수명과 종료 drain은 남겼다. |
| apps/simulator/AeroSimLabAppGamepad.cpp | KEEP | ScanForGamepad 27, PollGamepad 50, HandleGamepadDisconnected 118. 장치 상태를 입력으로 조종 명령을 만들고 연결 해제를 처리한다. 독립 기능이지만 MVP에서 수동 조종 필요 여부는 별도 결정. |
| apps/simulator/AeroSimLabAppInput.cpp | SIMPLIFY | 당시 PollGroundControl 184, ProcessPendingGroundCommand 727, CurrentTelemetry 840이 UDP 명령과 상태 응답을 처리했다. 기록·재생 제어는 이후 제거했다. 목표 임무와 직접 통신은 보존한다. |
| apps/simulator/AeroSimLabAppLifecycle.cpp | SIMPLIFY | Run 28, RunFrame 139, SmokeTestIsComplete 302, ValidateSmokeTestOutput 313. 프레임 입력→모의→렌더→swap을 묶고 웹 예외 및 smoke 시간 초과를 처리한다. 벤치마크·웹/네이티브 분기 정리는 EO 검증과 종료 경로를 보존하며 수행. |
| apps/simulator/AeroSimLabAppMission.cpp | SIMPLIFY | 기준선에는 단일 드론의 이륙·접근·카메라 정렬·촬영·착륙 처리와 네 대 임무가 함께 있었다. 구형 단일 드론 흐름을 제거했다. 남은 흐름은 편대 이동·드론별 정찰·재집결·복귀를 맡으며, 메시지 처리와 임무 판정은 아직 한 파일에 모여 있다. |
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

- `AeroSimLabApp.cpp`: 1–469, 470–700, 701–930, 931–1150, 1151–1264 확인.
- `AeroSimLabAppInput.cpp`: 1–250, 251–500, 501–726, 727–863 확인.
- `AeroSimLabAppLifecycle.cpp`: 1–337 확인, 미확인 없음.
- 다음 11개는 전체 확인: BenchmarkProcessMemory.h 1–36, BuildRevision.cpp 1–11, BuildRevision.h 1–9, ExecutableDirectory.h 1–36, FleetDroneSpawn.cpp 1–37, FleetDroneSpawn.h 1–24, FleetMissionProgress.cpp 1–119, FleetMissionProgress.h 1–92, FleetTelemetry.h 1–33, TrainingRangeScene.cpp 1–149, TrainingRangeScene.h 1–59.

추가 완독: AeroSimLabApp.h 1–220, 221–440, 441–604; AeroSimLabAppBenchmark.cpp 1–131; AeroSimLabAppEoCapture.cpp 1–150, 151–299; AeroSimLabAppGamepad.cpp 1–188. EoVideoRecorder.cpp 1–205, 205–659 확인. 추가 완독: AeroSimLabAppMission.cpp 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1500, 1501–1750, 1751–2000, 2001–2240; CooperativeMissionBenchmark.cpp 1–200, 201–400, 401–563; DroneScalingBenchmark.cpp 1–240, 241–480. 추가 완독: EoVideoRecorder.h 1–90; FaultScenarioTimeline.cpp 1–200, 201–400, 401–583; FaultScenarioTimeline.h 1–101; SimulationBenchmark.cpp 1–204; SimulationBenchmark.h 1–129. 추가 완독: SimulatorHud.cpp 1–250, 251–500, 501–750, 751–1000, 1001–1203; SimulatorHud.h 1–143; main.cpp 1–214; web/web_shell_single_thread.html 1–250, 251–500, 501–750, 751–1000, 1001–1250, 1251–1335; tests/simulator/SimulatorSessionTests.cpp 1–250, 251–500, 501–750, 751–941. 따라서 완독 32/32개, 미완독·누락 없음.

추가 확인: FleetMissionProgress.cpp 63–117은 보고 누락이나 단계 불일치 시 대기하고 정찰·복귀 실패 및 착륙을 판정한다. 상태를 한국어 문자열로 파싱하므로 문구 변경이 판정을 바꿀 수 있다(5–39). TrainingRangeScene.cpp 53–57의 첫 네 정찰 지점은 다중 홉 연결을 전제로 배치됐으므로 직접 통신 목표에 맞는지 재검토해야 한다. ExecutableDirectory.h 20–32는 경로 조회 실패 시 예외를 던지고, FleetDroneSpawn.cpp 21–28은 지형 범위 밖을 nullopt로 반환한다.


추가 분석(기준선): EoVideoRecorder::Stop의 카운터 경쟁을 확인해 `7bfce7e7`에서 수정했다. 이후 2026-10-29 카메라 파일 저장 기능을 제거했으므로 이 구현과 종료 경로는 현재 존재하지 않는다. 당시 AppInput의 상태 요청은 라이브 영상과 저장 파일 전송을 함께 시작했으나, 현재 저장 파일 전송은 제거되고 라이브 영상만 남는다. 나머지 줄 번호는 변경 전 기준이다.


마지막 다섯 파일의 범위도 위 구간에서 확인했다.

추가 분석: CompleteFleetMissionCanStart 333–350은 네 대 고정이 아닌 두 대 이상을 허용한다. StartFormationTransit 1173–1247은 경로에 따라 일렬 전환을 허용하고 PrepareFleetNetwork 1274–1283은 최대 5홉을 설정한다. M02 후 재집결과 M03은 1128–1146에서 연결되며, 보고 누락 시 1122–1125에서 대기한다. 이전 감사에서는 CooperativeMissionBenchmark.cpp와 DroneScalingBenchmark.cpp가 장애·재배정 임무 및 1–1024대 계측을 맡는 것을 확인했다. 2026-10-29에 이 두 대규모 성능 시험 구현과 앱 보고서 호출부를 제거했다. 이후에는 네 대 목표에 직접 필요한 성능 지표만 따로 선정한다.


이번 확인의 보강(기준선): EoVideoRecorder의 프레임 수·체크섬·전달 완료 API는 카메라 파일 저장과 함께 2026-10-29 제거했다. FaultScenarioTimeline.cpp 210–315는 JSON을 임시 지역 값으로 검증한 후 상태에 반영하고 오류를 문자열로 반환한다. 336–446은 시작·복구 이벤트를 정렬 삽입하며 시간 범위를 검사한다. 525–561은 현재 Tick에 도달한 이벤트와 대기 목록을 출력한다. 동일 장애 입력의 HUD 콜백과 AppInput 경로를 함께 제거해야 한다. SimulationBenchmark.cpp 50–58은 빈 표본을 0과 표본 수 0으로 보고하고, 184–196은 JSON 파일 열기·쓰기 오류를 예외로 반환한다. 71–74의 고정 설정 문자열은 현재 네 대 생성 및 훈련장 장면과 불일치한다. 성능 수치 자체는 측정하지 않았다.


마지막 확인 당시 SimulatorHud.cpp는 장면과 각 드론 카메라 화면을 렌더 콜백으로 표시했다. 2026-10-29 요청으로 웹 WebRTC와 네이티브 RTP 송수신을 제거했다. 웹 셸은 페이지 안에서 시뮬레이터 캔버스 일부를 같은 창 카메라 패널로 복사한다. 이전 파일별 줄 번호는 감사 기준 시점의 내용이며, 현재 호출 흐름의 증거로 쓰지 않는다.

## 구형 단일 드론 점검 흐름 제거 — 2026-10-29

- 앱의 구형 단일 드론 점검 상태, 카메라 방향 맞춤·촬영 대기 단계, 점검면·가림 자료, 해당 경로 표시와 웹 시작·멈춤·재개 조작을 제거했다. 네 대가 편대 이동·분산 정찰·복귀를 하는 전체 임무는 남겼다.
- 정찰 지점 자료는 이름과 접근 위치만 가진다. 카메라 영상은 표시하고 화질을 읽되, 앱이 촬영 성공을 판정하지 않는다.
- Windows Release 앱·시험 타깃 링크 성공. 전체 CTest 29/29 통과. 관제 클라이언트 시험은 첫 실행에서 접근 위반이 났지만, 시험 실행 파일을 다시 빌드한 뒤 통과했고 같은 시험을 10회 반복해 모두 통과했다. 최초 실패 원인은 정확히 특정하지 못했다.
- 웹 셸의 인라인 JavaScript 구문 검사는 통과했다. Emscripten 빌드와 브라우저 실행은 확인하지 않았다.

