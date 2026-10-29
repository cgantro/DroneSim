# 감사 결과

감사 당시 발견한 사실과 이후 수정 상태를 기록합니다. 개별 코드의 최신 위치는 변경될 수 있습니다.

## 영상 녹화

- 과거에는 인코더 스레드와 종료 처리 사이의 데이터 경쟁을 `7bfce7e7`에서 수정했다. 2026-10-29 사용자가 카메라 영상·사진 파일 저장 제거를 요청해 해당 녹화기와 저장 경로를 제거했다. 이 항목은 현재 코드의 활성 결함이 아니라 과거 조사 기록이다.

## 저전력 정찰 결과 판정

- `EvaluateFleetMissionProgress`는 정찰 중 저전력으로 지상에 복귀한 기체를 완료 보고처럼 건너뛴다(FleetMissionProgress.cpp:74–82). 기존 시험도 이 경우 단계 진행을 기대한다(SimulatorSessionTests.cpp:438–446). 판정 시험에서 재현했고 `693a7dbb`에서 정찰 단계의 저전력 복귀를 중단으로 처리했다. 지점 도착 전후를 구분하지 않는 보수적 처리이며 전체 임무 시나리오 검증은 필요하다.

## 경로 전달에서 첫 경유점 누락 가능성

- `PlanMapFlightRoute`는 출발점을 첫 항목으로 넣지 않고, 필요한 상승 지점이나 다음 수평 경유점부터 반환한다(MapFlightRoute.cpp:371–405). 수정 전 정찰·복귀·집결 경로 생성은 `begin()+1`부터 복사했다(AeroSimLabAppMission.cpp:467,622,742,861). 첫 상승 지점 또는 경유점이 생략됐다. 헤더도 시작점 제외·목표점 포함을 명시하며(MapFlightRoute.h:24), 독립 코드 검토에서 확인했다. 경로가 목표점 하나뿐이면 모든 경유점이 빠졌다. `ac98068c`에서 네 호출부를 수정했고 정찰 경로에서 수정 전 실패와 수정 후 성공을 확인했다. 실제 장애물 회피 실패 재현은 하지 않았다. 같은 시작·목표에서는 경로 한 점을 반환하므로 단순히 이 사례만으로 vector too long의 원인이라고 단정할 수 없다.

## Codex 설정

- `.codex/config.toml`: 주 모델 Sol medium, 하위 Terra medium, 최대 동시 2개. 제공 파일과 TOML 필드를 확인했고 strict-config doctor는 성공했다. 현재 실행 중인 대화의 모델 전환과 새 세션에서 실제 역할 적용은 확인하지 않았다.
- `.codex/agents/audit_app.toml`: 앱과 HUD 전수 조사 담당, 지정 보고서만 쓰도록 제한한다.
- `.codex/agents/audit_sim.toml`: 물리·시나리오·지형 조사 담당, 제품 코드 수정과 재위임을 금지한다.
- `.codex/agents/audit_infra.toml`: 통신·영상·빌드·문서 및 자산 조사 담당, 제품 코드 수정과 재위임을 금지한다.
- `.codex/agents/critical_review.toml`: 특정 오류·수명·상태 전이의 읽기 전용 Sol high 검토 담당이다.
- `.codex/agents/inventory_scan.toml`: 경로·사용처·줄 수를 확인하는 Luna low 담당이며 설계 판단은 맡지 않는다.

## 도구와 의존성

- `tools/px4/compare_reference.py` 전체: 시나리오 참조 표본과 결과 궤적을 입력받아 위치·속도·자세·각속도·로터 회전수 RMSE를 계산한다(compare_reference:41). 벡터 크기·유한값·시간 일치를 검사하고 오류를 CLI로 알린다. 유지 후보이며 사용자 요청 임무 시험과 별도의 물리 검증 도구다.
- `tools/px4/test_compare_reference.py` 전체: 기준 표본을 메모리에서 만들고 동일 상태·자세 부호·시간 불일치·운동학 입력 거부 등을 독립 검증한다(CompareReferenceTests:14). 위 비교 도구를 유지하면 시험도 유지한다. 이번 감사에서 실행하지 않았다.
- `tools/terrain/convert_ascii_grid.cpp` 전체: ASCII 지형 격자와 좌표를 읽어 변환 파일로 저장한다(main:21). 인자 수와 숫자 변환을 검사하고 실패 시 종료 코드 1 또는 2를 반환한다. 훈련장 외 기존 자산 변환에 쓰는지 호출처를 확인한 뒤 유지 여부를 정한다.
- 2026-10-29 영상 전송 제거: GStreamer는 RTP 송수신 외 사용처가 없어 vcpkg와 CMake 의존성에서 제거했다. OpenCV는 프레임 품질 계산에 남아 있다.

- `tools/px4/prepare_reference.py` 전체: pyulog에서 위치·자세·각속도·모터 로그를 읽어 60Hz 강체 재생 JSON을 만든다(prepare_scenario:408). NED/FRD를 ENU/FLU로 변환하고, 표본 간격·원점 변경·ESC 연결을 검사한다. 참조 기록의 민감 정보를 제외하며 기존 출력 파일은 거부한다. 물리 검증이 필요하므로 유지 후보이며 결과 해석은 실기체 정확도 주장과 구분한다.
- `tools/px4/test_prepare_reference.py` 전체: TopicSamples 보간·축 변환·고도 원점 및 ESC 상태 오류 등을 unittest로 확인한다(TopicSamplesTests:20). 변환기를 유지하면 관련 시험을 유지한다. 이번 감사에서 실행하지 않았다.

- `tools/web_runtime_smoke.cjs` 전체: 임시 로컬 HTTP 서버와 Playwright Chrome으로 시작·WebGL·EO·이륙·브라우저 녹화 및 다운로드 검사를 한다(run:61). 실패는 종료 코드 1로 알리고 finally에서 브라우저와 서버를 닫는다. 단일 시설물·WebRTC·촬영 판정 중심 부분은 4대 새 범위에 맞게 바꿔야 한다. 플랫폼 유지용 기본 브라우저 시험은 남긴다. 이번 감사에서는 브라우저를 실행하지 않았다.

## 외부 코드와 보관 자료

- `third_party/glad/include/glad/glad.h`, `third_party/glad/src/glad.c`: glad 0.1.36이 생성한 OpenGL 3.3 로더. CMake의 glad 대상과 그래픽 API에서 사용한다. 로컬 Git 이력은 도입 커밋 한 건이지만 외부 원본과 동일 여부는 미확인. 각 파일 내 라이선스 고지는 검색되지 않아 출처·조건 추가 확인 필요. 외부 코드로 분류해 자체 LOC에서 제외한다.
- `third_party/glad/include/KHR/khrplatform.h`: Khronos의 플랫폼 자료형과 라이선스 고지를 포함한다. glad 헤더에 필요하다. 외부 원본과의 차이는 미검증이다.
- `third_party/coi-serviceworker.min.js`: 0.1.7, 파일 헤더에 MIT 고지. 현재 앱·CMake·CI의 명시 참조는 검색되지 않았다. 배포 경로와 외부 원본 라이선스 파일 확인 전 삭제 여부는 UNKNOWN이다.
- `third_party/stb_image_LICENSE.txt` 전체: stb의 MIT 또는 public domain 조건. stb_image는 FetchContent로 준비되고 Texture2D.cpp에서 사용한다. 현재 로컬 파일과 배포 대상의 라이선스 포함 여부는 추후 확인한다.
- `docs/history/project-history.zip`: 기존 이력 보관 압축파일. 내부 53개 항목, 압축 해제 기준 573796바이트. 파일 내용은 외부 자료로 취급하고 펼치거나 삭제하지 않았다.
