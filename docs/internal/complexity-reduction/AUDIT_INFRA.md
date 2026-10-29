# 인프라 감사 — 첫 25개

기준: INVENTORY.tsv의 owner=infra, audit_status=UNREAD 행을 순서대로 25개 선택. 이 기록은 정적 확인이며 빌드·시험·렌더링을 실행하지 않았다. 텍스트 19개 완독, 바이너리 6개는 헤더·GLB JSON 메타데이터·호출 위치·같은 묶음의 출처/조건을 확인했다. 외부 원본의 라이선스는 독립 검증하지 않았다. 판정은 요청된 4대 V 편대→분산 정찰→복귀, 직접 통신, EO와 로컬 녹화에 대한 제안이며 실제 삭제 지시가 아니다. inventory 상태는 변경하지 않았다. 선택 당시 전체 infra UNREAD 171개 중 이후 146개는 이번 범위 밖이며 UNREAD다. 아래 호출 검색도 저장소 전체 의미 분석이 아니다.

## 파일별 기록

1. `.agents/skills/troubleshooting-postmortem/SKILL.md` — **KEEP**. 입력은 문제 관련 코드·시험·기록, 출력은 증상부터 검증까지 근거를 구분한 기록이다. AGENTS.md가 문제 해결 기록 작성 시 참조한다. 제품 런타임 소유권은 없다. 수치 비교 조건과 미검증 표시를 요구한다. 중복된 사례를 새 문서로 늘리지 않게 하는 지침이며 MVP 범위 축소와 무관하다.
2. `.gitattributes` — **KEEP**. Git이 grid·ppm·step을 바이너리 취급하는 설정이다. 해당 자료를 Git이 읽고 비교할 때 적용되며 런타임 입출력은 없다. PPM 자산을 텍스트로 변환하지 않도록 유지한다. 오류 처리나 중복 로직은 없다.
3. `.github/workflows/deploy-pages.yml` — **SIMPLIFY**. main 변경·PR·수동 실행에서 Emscripten 빌드, Chromium 시작 확인, Pages 산출과 배포를 연결한다. Actions가 산출물을 소유하며 PR에는 배포하지 않는다. 파일 존재 검사와 smoke 실패가 작업을 실패시킨다. 네이티브 CI와 의존성 설치/빌드가 일부 겹치지만 플랫폼이 다르다. 웹 지원 필요성 확인 후 유지 여부와 경로 필터를 줄이고, 제거 대상 기능이 빌드에 남지 않도록 조정한다.
4. `.github/workflows/native-ci.yml` — **SIMPLIFY**. push/PR에서 변환 도구 시험, 의존성 설치, Linux Debug 빌드, clang 분석, 가상 화면 CTest를 수행한다. CI 작업이 결과를 소유하고 각 명령 실패가 작업을 중단한다. quality와 패키지 설치 및 빌드가 겹친다. MVP 핵심 임무·직접 통신·녹화 검증은 유지하고 제거되는 광범위 대상의 시험 및 의존성을 재검토한다.
5. `.github/workflows/quality.yml` — **SIMPLIFY**. sanitizer 작업은 자동 실행, coverage는 수동 실행하여 단위/통합 시험과 보고서를 만든다. CI 산출물로 coverage를 업로드하며 업로드는 앞선 실패 때도 시도한다. native-ci와 설치·빌드가 중복되지만 검사 목적은 다르다. 제거되는 기능 시험과 영상 전송 패키지를 정리하되 유용한 메모리 검사까지 무조건 없애지는 않는다.
6. `.gitignore` — **KEEP**. 빌드 산출물, 개인 자료 및 로컬 설정 등을 Git 추적 후보에서 제외한다. Git이 소비하며 런타임 자원은 소유하지 않는다. 기존 추적 파일에는 소급 적용되지 않는다. 기존 빌드 위치와 중복되지 않는 보호 설정을 유지한다.
7. `.vscode/settings.json` — **KEEP**. VS Code CMake 확장에서 preset 사용과 병렬 작업 수를 읽는다. 빌드 설정만 제공하고 실행 자원은 소유하지 않는다. CMakePresets와 역할이 다르며 별도 오류 처리 없다.
8. `AGENTS.md` — **KEEP**. 저장소 작업 규칙과 기준 문서·검증·작업 공간 제약을 제공한다. 작업자가 읽으며 실행 자원이나 오류 처리 로직은 없다. 문제 해결 skill과 역할이 연결되나 범위가 달라 통합할 이유는 확인되지 않았다.
9. `CMakeLists.txt` — **SIMPLIFY**. CMake가 소스·의존성·플랫폼 옵션을 입력받아 라이브러리, 시뮬레이터, 자산 복사 및 테스트 대상을 만든다. 빌드 시스템이 산출물을 소유한다. 누락된 네이티브 실행 자산은 configure에서 FATAL_ERROR 처리한다. `aerosimlab_media`에 RTP 및 기록 전송이 들어가고 앱에 여러 benchmark 소스가 포함되며 mesh 통신과 관련 시험도 등록되어 있다. 제거 대상 기능과 시험을 해당 구현 및 호출부 정리와 함께 빼고, EO 로컬 녹화에 필요한 OpenCV videoio와 임무 시험은 보존해야 한다. GStreamer 의존성은 영상 전송 제거 후 다른 사용처를 확인하고 판단한다. 현재 빌드 성공 여부는 미검증.
10. `CMakePresets.json` — **SIMPLIFY**. CMake/IDE에 Windows Release, Linux debug·sanitizer·coverage 및 웹 구성/빌드 프리셋을 제공한다. 각 빌드 폴더에 출력하며 오류는 CMake에 맡긴다. 상속으로 설정 중복을 줄인다. 웹과 coverage는 사용자 지원 필요성에 맞춰 판단하고 전송 제거 시 desktop-video 의존성의 실제 구성을 재검토한다.
11. `GOAL.md` — **SIMPLIFY**. 목표·대표 임무·완료 기준을 독자에게 제공하며 프로그램에 호출되지 않는다. 현재 중계·장애 대응·비교가 중심에 포함되어 요청된 새 MVP와 충돌한다. 네 대 임무·직접 통신·EO 로컬 녹화를 중심으로 갱신해야 하며, 구현 확인은 SPEC과 코드에서 별도로 해야 한다.
12. `README.md` — **SIMPLIFY**. 실행 순서와 공개 문서 길잡이를 제공한다. 독자가 소비하고 런타임 소유권·오류 처리는 없다. 다중 홉·장애 대응·성능 측정 설명이 새 범위와 어긋난다. 제거 구현이 확정된 뒤 실행 안내와 함께 갱신해 상충을 피한다.
13. `SPEC.md` — **SIMPLIFY**. 구현과 시험 상태·한계를 독자에게 제공한다. 실제 실행을 수행하는 파일은 아니다. 현재 중계 복구·장애 자동 대응 및 영상 전달 계획을 담아 새 범위와 충돌한다. 남길 동작의 기존 시험 근거와 미검증 표시는 보존하고 구현 변경 후 범위를 재기록한다. 이 문서의 과거 통과 주장은 이번 감사에서 재실행하지 않았다.
14. `apps/ground_control/GroundControlClient.cpp` — **KEEP** (직접 UDP 명령이 MVP에서 필요하다는 조건). 호출자 소유 소켓과 주소·패킷·시간 제한·취소 토큰을 받아 400ms 간격 재전송, 상대 주소와 명령 ID/순번 확인 후 응답·왕복시간 반환. 잘못된 제한 시간, 통신/디코드 오류, 취소 및 시간 초과는 예외다. CMake 라이브러리 및 통신/시뮬레이터 시험에서 참조되며 검색된 제품 호출자는 확인되지 않았다. 목적지 확인·재시도 책임은 여기 있으므로 다른 통신 계층의 유사 재시도와 같은 의미인지 확인 전 합치지 않는다. 잘못된 응답 디코드 예외가 대기를 끝낼 가능성은 코드상 관찰이며 시험하지 않았다.
15. `apps/ground_control/GroundControlClient.h` — **KEEP** (위와 같은 조건). 결과 구조체, 제한 시간·취소 예외, SendCommand 계약을 노출한다. 소켓 수명은 호출자 책임이고 기본 제한 시간은 4초다. 구현과 시험이 포함하며 별도 중복 구현은 없다. 제품 호출이 필요한지 확인해 불필요한 독립 관제 경로라면 헤더와 구현을 함께 정리한다.
16. `assets/models/drones/x500/LICENSE` — **KEEP**. X500 묶음의 BSD 3-Clause 조건을 제공한다. CMake가 네이티브 복사와 웹 포함에 사용한다. 배포시 고지 조건이 있으므로 모델 유지 시 함께 유지한다. 오류 처리·실행 데이터는 없다; 카메라 별도 출처와 혼동하지 않는다.
17. `assets/models/drones/x500/action-camera.glb` — **KEEP**. 140,756바이트 glTF 2, 메시 8개·내장 이미지 4개. 앱이 기체 외장 모델로 읽고 CMake가 복사/웹 포함한다. 별도 출처 파일은 CC0라고 명시한다. 시각 자산이며 EO 센서 구현 자체가 아니다. 로더 오류와 표시 결과는 실행 미검증; 다른 기체 모델과 용도가 다르다.
18. `assets/models/drones/x500/carbon-weave.ppm` — **KEEP**. 49,167바이트 P6 128×128 질감; 앱이 별도로 읽고 CMake가 배포한다. X500 묶음의 BSD 라이선스 파일이 있으나 이 질감의 개별 출처는 추가 확인하지 않았다. 모델 내 이미지와의 중복 여부·로드 실패는 미검증.
19. `assets/models/drones/x500/visual.glb` — **KEEP**. 13,853,744바이트 glTF 2, 메시 1개, JSON에 이미지 항목 없음. 앱이 기체 모델로 읽고 CMake가 복사/웹 포함하며 같은 폴더 BSD 조건이 있다. 외장 카메라 모델과 역할이 다르다. 크기 축소 가능성은 측정하지 않았고 로드·렌더링 오류는 미검증.
20. `assets/models/drones/x500/카메라_출처.txt` — **KEEP**. 카메라 자산 출처와 CC0 1.0 정보를 제공한다. 런타임 호출은 검색되지 않았고 CMake 자산 목록에는 없다. 배포물에 출처 포함이 필요한지 별도 검토한다. X500 BSD 문서와 서로 다른 자산을 설명한다.
21. `assets/models/environment/README.md` — **KEEP**. 산업·자연 시각 모델의 출처와 가상 훈련장 용도를 설명하고 각 라이선스를 가리킨다. 사람이 읽는 문서이며 런타임 호출·오류 처리는 없다. 개별 라이선스와 역할이 다르다.
22. `assets/models/environment/industrial/LICENSE.txt` — **KEEP**. 산업 묶음의 Kenney CC0 조건을 기록한다. CMake가 네이티브 복사와 웹 포함을 한다. 묶음 모델을 유지하는 한 출처를 함께 보존한다. 런타임 계산·중복 구현은 없다.
23. `assets/models/environment/industrial/Textures/colormap.png` — **KEEP**. 11,986바이트 PNG, 헤더상 512×512. 산업 building-a/c GLB가 상대 URI로 참조하고 CMake가 자산 목록에 포함한다. 산업 묶음 CC0 문서가 있다. 텍스처 해석·실제 렌더링 실패는 미검증; 공유 질감이라 개별 건물과 중복 자산으로 보지 않는다.
24. `assets/models/environment/industrial/building-a.glb` — **KEEP**. 177,316바이트 glTF 2, 메시 1개, 외부 `Textures/colormap.png` 참조. TrainingRangeScene이 배치하고 CMake가 복사/웹 포함한다. 산업 묶음 CC0 조건을 확인했다. 경로 누락은 CMake의 네이티브 자산 검사 대상; 실제 화면 로딩과 충돌 형상은 미검증. building-c와 모델은 구분된다.
25. `assets/models/environment/industrial/building-c.glb` — **KEEP**. 175,844바이트 glTF 2, 메시 1개, 같은 외부 질감 참조. TrainingRangeScene에서 별도 위치에 배치하고 CMake가 복사/웹 포함한다. 산업 묶음 CC0 조건을 확인했다. 로드·충돌 동작은 미검증; a와 같은 공유 질감을 쓰지만 모델 삭제 근거는 아니다.

## 남은 확인

다음 대상은 inventory의 26번째 infra UNREAD부터다. 위 판정은 각 파일 정적 확인과 일부 참조 검색을 바탕으로 했으며 다른 파일의 전체 호출 관계, 런타임 동작, 외부 라이선스 원본 및 실제 빌드/시험 결과는 확인하지 않았다. 특히 GStreamer 제거 가능 여부와 관제 클라이언트의 제품 필요성은 관련 나머지 파일 감사 후 결정한다.

## 두 번째 묶음 — 기존 기록 다음 25개

선정 시 inventory의 infra UNREAD에서 기존 보고서에 있는 25개 다음 경로를 선택했다. 텍스트 6개를 완독하고 바이너리 19개는 파일 헤더·GLB JSON·참조·저장소 내 라이선스를 확인했다. 실행·렌더링, 바이너리 전체 의미 분석 및 외부 라이선스 원본 검증은 하지 않았다. 자산은 빌드가 배포하고 장면이 읽는 공유 입력이며 별도 런타임 소유 로직은 없다. 다음 판정은 삭제 명령이 아닌 MVP 제안이다.

### 산업 모델 공통 근거

다음 13개는 glTF 2, 각각 1개 메시(단 windmill은 2개)와 외부 `Textures/colormap.png` 이미지 참조를 가진다. `TrainingRangeScene.cpp`가 각각 별도 위치에 배치하고 `CMakeLists.txt`가 네이티브 자산 복사와 웹 포함 목록에 넣는다. 해당 묶음의 `industrial/LICENSE.txt`는 Kenney CC0로 명시한다. 빌드 시 네이티브 자산이 없으면 구성 실패하지만 로더 오류·그림·충돌 경계는 실행하지 않았다. 공유 질감은 중복 모델 자체를 뜻하지 않는다.

26. `assets/models/environment/industrial/building-f.glb` — **KEEP**, 134,916바이트. 별도 건물 배치에 쓰인다. 임무 환경의 시각 자산으로 유지; 다른 건물과 기능상 중복 여부는 장면 평가 전 미확정.
27. `assets/models/environment/industrial/building-j.glb` — **KEEP**, 80,792바이트. 별도 건물 배치 입력이다. 모델 형상 비교와 표시 결과는 미검증.
28. `assets/models/environment/industrial/building-m.glb` — **KEEP**, 138,920바이트. 별도 건물 배치 입력이다. 표시·충돌 형상 일치는 미검증.
29. `assets/models/environment/industrial/building-q.glb` — **KEEP**, 181,248바이트. 별도 건물 배치 입력이다. 표시·충돌 형상 일치는 미검증.
30. `assets/models/environment/industrial/building-t.glb` — **KEEP**, 139,896바이트. 별도 건물 배치 입력이다. 표시·충돌 형상 일치는 미검증.
31. `assets/models/environment/industrial/chimney-large.glb` — **KEEP**, 20,932바이트. 굴뚝 배치 입력이다. 건물 및 탱크와 시각 목적이 구별된다.
32. `assets/models/environment/industrial/detail-tank-large.glb` — **KEEP**, 47,452바이트. 큰 탱크 배치 입력이다. 작은 탱크와 형상·기능 중복은 실제 화면 확인 전 판단하지 않는다.
33. `assets/models/environment/industrial/detail-tank.glb` — **KEEP**, 31,436바이트. 탱크 배치 입력이다. 큰 탱크와 다른 장소에 놓인다.
34. `assets/models/environment/industrial/shipping-container-a.glb` — **KEEP**, 36,624바이트. 컨테이너 배치 입력이다. b·c와 모델 차이 및 장면 축소 가능성은 미검증.
35. `assets/models/environment/industrial/shipping-container-b.glb` — **KEEP**, 37,052바이트. 별도 컨테이너 배치 입력이다. 다른 변형과 공유 질감만 확인했다.
36. `assets/models/environment/industrial/shipping-container-c.glb` — **KEEP**, 36,620바이트. 별도 컨테이너 배치 입력이다. 다른 변형과 공유 질감만 확인했다.
37. `assets/models/environment/industrial/water-tower.glb` — **KEEP**, 93,836바이트. 급수탑 배치 입력이다. MVP 정찰 장면의 시각 환경을 구성한다.
38. `assets/models/environment/industrial/windmill.glb` — **KEEP**, 46,292바이트. 풍차 배치 입력이며 메시 2개다. 각 메시의 화면·애니메이션 동작은 미검증.

39. `assets/models/environment/nature/LICENSE.txt` — **KEEP**, 텍스트 완독. Kenney Nature Kit의 CC0 조건을 제공한다. CMake가 네이티브 복사 및 웹에 포함한다. 사람/배포물에 출처와 이용 조건을 알리는 자료이며 런타임 계산이나 오류 처리는 없다. 산업 묶음 라이선스와 자산 범위가 다르다.

### 자연 모델 공통 근거

다음 5개는 glTF 2, 메시 1개이며 GLB JSON에 이미지 항목이 없다. `TrainingRangeScene.cpp`가 배치하고 `CMakeLists.txt`가 실행 자료로 복사/웹에 포함한다. 위 Nature Kit 조건이 해당 폴더에 있다. 네이티브 빌드의 누락 자료 검사는 존재하나 실제 로드·화면과 충돌 경계는 미검증이다. 같은 종류의 자산이라도 서로 다른 모델로 배치되어 단순 중복 삭제 근거는 없다.

40. `assets/models/environment/nature/rock_largeA.glb` — **KEEP**, 7,552바이트. 바위 배치 입력이다.
41. `assets/models/environment/nature/rock_largeC.glb` — **KEEP**, 7,004바이트. 다른 위치의 바위 배치 입력이다.
42. `assets/models/environment/nature/rock_tallB.glb` — **KEEP**, 14,060바이트. 다른 형식의 바위 배치 입력이다.
43. `assets/models/environment/nature/tree_oak.glb` — **KEEP**, 14,644바이트. 여러 위치의 참나무 배치에 재사용된다.
44. `assets/models/environment/nature/tree_pineTallA_detailed.glb` — **KEEP**, 10,708바이트. 여러 위치의 소나무 배치에 재사용된다.
45. `assets/models/source/x500-v2/README.md` — **KEEP**, 텍스트 완독. 제조사 CAD 원본이 빌드에 없다는 설명과 원본 안내 링크를 제공한다. 독자가 소비하며 런타임 호출·오류 처리는 없다. 재배포 조건은 따로 확인하도록 명시한다. 현재 쓰는 X500 모델 라이선스와 역할이 다르다. 외부 링크와 조건은 확인하지 않았다.
46. `assets/shaders/Scene.glsl` — **KEEP**. 정점 위치·법선·인스턴스 행렬과 카메라/조명/재질 값을 입력받고 변환된 정점과 조명·텍스처·안개를 적용한 픽셀 색을 출력한다. 앱이 로드하고 CMake가 자산으로 복사하며 그래픽 시험이 참조한다. GPU 프로그램의 수명과 컴파일 오류 처리는 로더 소유이며 이 파일 자체에는 오류 보고가 없다. X500 삼축 질감, 지면·물 표시 등 분기들이 있어 중복 제거를 단정할 수 없다. EO와 장면 표시 유지에 필요하며 실제 컴파일·화면 품질은 미검증.
47. `cmake/Dependencies.cmake` — **KEEP**. 최상위 CMake에서 함수를 호출해 GLM, Flecs, Eigen/AirSim, JSON, stb 및 플랫폼별 GLFW/OpenGL을 준비하고 대상에 연결한다. FetchContent가 빌드 폴더의 다운로드와 추출물을 관리하고 AirSim 아카이브는 추출 후 삭제한다. OpenGL 필수 탐색과 AirSim 해시 검증을 설정한다. 같은 의존성의 제품 측 중복 선언은 이 범위에서 확인되지 않았다. 네이티브 물리와 화면에 필요하므로 유지하되 실제 기능 제거 뒤 미사용 의존성만 재검토한다. 다운로드·빌드는 실행하지 않았다.
48. `cmake/GenerateBuildRevision.cmake` — **SIMPLIFY**. 최상위 CMake가 Git 실행 파일·소스·경로·출력 위치를 전달해 짧은 revision과 지정 경로의 dirty 여부를 헤더로 생성한다. 출력 내용이 같으면 다시 쓰지 않는다. Git 오류 출력은 숨기고 빈 revision은 unknown으로 대체한다. Git status 실패는 명시적으로 구별하지 않아 clean처럼 보일 가능성이 있다(정적 추론). 빌드 식별이 필요하면 유지하되 광범위 벤치마크 제거 후 복잡한 추적이 여전히 필요한지 확인한다. 최상위 CMake의 의존 파일 추적과 책임은 분리된다.
49. `docs/public/images/simulator-overview.jpg` — **SIMPLIFY** (연결 여부 확인 후). JPEG 73,017바이트, 1282×752 헤더 확인. 공개 문서와 README에서 파일명·이미지 경로 검색 결과 참조가 없었다. 문서용 이미지로 추정하며 생성 경위·라이선스·내용 및 다른 외부 참조는 미확인이다. 런타임 자원이 아니다. 즉시 삭제하지 말고 실제 사용처·최신성·권리를 확인해 유지 또는 정리한다.
50. `docs/public/기술구조.md` — **SIMPLIFY**. 독자에게 입력→임무→통신→물리→화면과 기술 책임을 설명한다. 코드가 실행하는 문서는 아니며 오류 처리는 없다. 중계·장애 복구 설명이 새 MVP와 맞지 않고, 중계 전체 임무 검증 범위 문장은 SPEC의 최근 결과와 다르다. 또한 GStreamer가 녹화 파이프라인이라고 설명하지만 확인한 `EoVideoRecorder.cpp`는 `cv::VideoWriter`를 사용하고 GStreamer 코드는 RTP 전송에 있다. OpenCV 백엔드 의존성까지 추적하지 않았으므로 단정하지 말고 관련 구현 감사 후 문서와 SPEC을 함께 갱신한다. 기술 설명과 SPEC의 역할은 다르다.

이번 묶음 이후 미확인 infra 경로는 선정 당시 121개다. inventory는 다른 작업자가 갱신할 수 있으므로 다음 선택에는 현재 UNREAD 중 이 보고서에 이미 기록한 경로를 제외한다.

## 세 번째 묶음 — 현재 UNREAD 중 미기록 25개

선정 시 현재 inventory의 infra UNREAD에서 이 문서에 이미 기록한 경로를 제외했다. 아래 공개 문서 25개를 끝까지 읽었다. 각 문서는 독자가 읽는 과거 문제 해결 기록으로, 제품 런타임에 호출되지 않으며 파일·소켓·메모리를 소유하거나 직접 오류를 처리하지 않는다. `docs/public/문제해결.md`가 개별 글을 연결한다. 설명된 시험과 수정은 문서의 당시 주장이고 이번에 코드·원본 로그·시험을 재검증하지 않았다. KEEP은 역사 기록과 잔존 기능의 근거를 보존하라는 제안이며, 제거된 기능을 제품에 유지하라는 뜻은 아니다.

51. `docs/public/문제해결.md` — **SIMPLIFY**. 문제별 목차와 글의 링크를 출력한다. 세부 기록과 내용 책임이 달라 중복 구현은 없다. 제거 기능에 대한 문서의 보존 방침을 정한 뒤 링크 및 현재 상태 안내를 갱신한다. 링크 전체의 실제 유효성은 미검증.
52. `docs/public/문제해결/비행과 충돌/PX4-기록-재생-자세-차이.md` — **KEEP**. PX4 명령과 ESC 회전수의 차이, 입력 변경 전후 비교와 실제 궤적 미재현을 기록한다. 높은 정확도를 보증하지 않는 근거이며 외부 로그·실측은 이번에 확인하지 않았다. 높이 기준 기록과 비교 조건이 다르다.
53. `docs/public/문제해결/비행과 충돌/PX4-높이-기준.md` — **KEEP**. 지면 기준 보정과 로그 간 수직 원점 차이, 남은 자세 오차를 기록한다. 재생 기록과 원인이 다르다. 외부 로그·시험은 미검증.
54. `docs/public/문제해결/비행과 충돌/강체-이륙-임무-시작.md` — **KEEP**. 기한 내 이륙 완료를 기다리는 임무 시작 처리와 강체 시험 범위를 기록한다. 재할당 예제는 과거 맥락이며 제거 기능이 현재 지원된다는 뜻이 아니다.
55. `docs/public/문제해결/비행과 충돌/강체-지면-접촉-관통.md` — **KEEP**. 지면 고정 시 겹침 보정과 평지 시험 범위를 기록한다. 경사면·동시 접촉은 미검증으로 구분하며 지형 좌표 문제와 원인이 다르다.
56. `docs/public/문제해결/비행과 충돌/강체-충돌회피-안전거리-미달.md` — **KEEP**. 8m 설정에도 근접한 사례와 출발 조건을 바꾼 시험을 기록한다. 알고리즘의 보편 안전성은 확인되지 않았음을 명시한다. 다른 회피 사례와 조건이 다르다.
57. `docs/public/문제해결/비행과 충돌/기체-질량-관성-누락.md` — **SIMPLIFY**. 부품 물성 합산과 시험·실측 한계를 기록한다. 본문은 회귀 시험을 복원했다고 하면서 남은 확인에 자동 시험이 수치 변경을 감지하지 못한다고 적어 충돌 가능성이 있다. 현 시험을 대조한 뒤 문구를 정리한다. 시험은 이번에 실행하지 않았다.
58. `docs/public/문제해결/비행과 충돌/물리-수치-범위.md` — **KEEP**. double→float 경계 검사와 거부 전 상태 보존, 일부 경계 시험의 범위를 기록한다. 변환 범위와 물리값 유효성은 다른 책임이다. 실제 비행 사고 원인으로 단정하지 않는다.
59. `docs/public/문제해결/비행과 충돌/지형-경계-이탈.md` — **KEEP**. 강체·운동학 경계 정책 차이와 공통 위치 제한, 웹 빌드 미검증을 기록한다. 지면 높이 처리와 경계 정책은 별개다.
60. `docs/public/문제해결/비행과 충돌/지형-높이와-충돌.md` — **KEEP**. 지형 고도·건물 축·운동학 높이 처리를 구분하고 시험 축소 이후 한계를 남긴다. 비행과 충돌에 필요한 기준 기록이며 개별 지면 접촉 기록과 원인이 다르다.
61. `docs/public/문제해결/비행과 충돌/충돌-경계-접촉-누락.md` — **KEEP**. 후보 검사와 정밀 검사에서 정확히 접촉한 경계의 판정 차이를 기록한다. 성능 글과 연결되지만 정확성 결함이 별도다. 무작위·다른 플랫폼 범위는 확인되지 않았다.
62. `docs/public/문제해결/비행과 충돌/통신단절-충돌회피-거리미달.md` — **KEEP**. 단절 상태 추정과 회피 목표의 과거 문제 및 조건별 재검증을 기록한다. 제거 예정인 자동 장애 복구의 역사 기록으로 남길지 공개 문서 정책에 따라 결정한다. 수치의 원본과 현재 회피 기능은 미검증.
63. `docs/public/문제해결/비행과 충돌/호버-드론-충돌회피-누락.md` — **KEEP**. 임무 없는 호버 기체가 예측에서 빠진 원인과 당시 회귀 범위를 기록하고 이후 안전거리 기록을 연결한다. 서로 다른 결함이므로 무리한 통합은 불필요하다.
64. `docs/public/문제해결/시험과 성능/다중드론-충돌-성능.md` — **SIMPLIFY**. 충돌 후보 계산과 여러 규모의 과거 측정·한계를 기록한다. 광범위 벤치마크는 새 MVP 밖이므로 공개 목차와의 관계 및 역사 기록 유지 여부를 정리한다. 원본 측정 자료가 삭제됐다고 문서가 밝히므로 이번 성능 재검증은 불가능하다. 접촉 정확성 관련 기록은 별도 가치가 있다.
65. `docs/public/문제해결/시험과 성능/통합시험-UDP-시작-시간초과.md` — **KEEP**. GLFW 스레드와 UDP 준비 순서의 후보, Windows 재시험 및 Linux sanitizer 미검증을 구분한다. 지상통제 창의 시간 초과와 사건이 달라 별도 유지한다.
66. `docs/public/문제해결/임무와 통신/명령-기록-재생.md` — **KEEP**. 기록 경계 Tick과 입력 소유권, 남은 운동학·UDP 시험 범위를 설명한다. EO 로컬 영상 녹화와 조종 명령 기록은 별개다. 기록 재생의 제품 범위는 추가 결정이 필요하다.
67. `docs/public/문제해결/임무와 통신/접근위반-미확정.md` — **KEEP**. 덤프가 원인을 확정하지 못한 이유와 필요한 증거를 기록한다. 다른 스택 오류와 같은 원인으로 묶지 않는다. 당시 덤프·기호는 이번에 열지 않았다.
68. `docs/public/문제해결/임무와 통신/정찰-재배정-카메라-방향.md` — **KEEP**. 목표 위치 도착과 EO 촬영 방향을 구분한 과거 수정·시험을 기록한다. 자동 재배정은 제거 대상이지만 촬영 방향의 교훈은 남는 정찰 경로에도 적용될 수 있다. 실제 화면은 당시에도 미검증이다.
69. `docs/public/문제해결/임무와 통신/정찰-지점-반복-재배정.md` — **KEEP**. 고장 기체의 만료 권한 때문에 지점이 반복 배정된 원인과 당시 시험을 기록한다. 자동 장애 복구 제거 후 역사 자료임을 명확히 유지한다.
70. `docs/public/문제해결/임무와 통신/조종기-연결-감지.md` — **KEEP**. 장치 감지 실패의 원인과 수정이 미확정임을 기록한다. 연결 해제 정책 문제와 단계가 다르다. 실제 장치는 확인하지 않았다.
71. `docs/public/문제해결/임무와 통신/조종기-연결해제-안전동작.md` — **KEEP**. 프레임과 고정 Tick 사이 안전 명령 수명, 입력 소유권과 시험 한계를 기록한다. 실제 장치 분리·대기열 보존은 당시 미검증이다. 새 MVP에서 수동 조종 지원 여부는 별도 판단한다.
72. `docs/public/문제해결/임무와 통신/지상통제-UDP-응답.md` — **KEEP**. 소켓 수명·응답 식별 및 미확정 원인을 기록한다. 직접 UDP와 관련되나 과거 Windows 창 존재 여부·실제 네트워크는 이번에 확인하지 않았다. CI 시작 시간 초과와 다른 사례다.
73. `docs/public/문제해결/화면과 카메라/웹-화면-로딩-멈춤.md` — **KEEP**. 웹 스택 크기와 당시 Chromium 첫 화면 검사 범위를 기록한다. 웹을 유지할지 결정 전 역사 근거로 보존한다. 외부 CI 실행과 현재 브라우저는 확인하지 않았다.
74. `docs/public/문제해결/화면과 카메라/정찰-이륙-상태와-화면시험.md` — **KEEP**. 오래된 객체의 스택 손상, 카메라 가림, 원인 미확정 임무 정지를 구분한다. 화면 스모크 통과가 임무 정지 해결을 뜻하지 않는다는 한계를 남긴다.
75. `docs/public/문제해결/화면과 카메라/카메라-기울기-반영.md` — **KEEP**. EO 카메라 위치·시선·위쪽 축에 같은 자세를 적용한 기록이다. 뷰 행렬 단위 시험과 화면 통합 미검증을 구분하며 정찰 촬영 방향 기록과 원인이 다르다.

선정 시 미기록 UNREAD는 121개였으므로 이번 묶음 이후 96개가 미확인이다. 다른 작업자가 inventory를 갱신할 수 있어 다음 선정은 현재 UNREAD와 본문 기록 경로를 다시 대조한다. 문서에 서술된 코드·시험·외부 자료는 이번 묶음에서 독립 검증하지 않았으며 조사 못한 범위는 UNREAD다.

## 네 번째 묶음 — 현재 UNREAD 중 미기록 20개

현재 inventory에서 infra UNREAD이고 이 보고서에 없는 경로의 앞 20개를 선택했다. 20개 모두 텍스트 전체를 읽었다. 관련 참조는 `rg` 검색 결과 일부를 확인했다. 외부 원본·라이브러리의 라이선스나 원격 실행은 확인하지 않았고, 코드 실행·빌드·시험·sanitizer는 수행하지 않았다. C++ 검토는 선택된 파일의 정적 검토이며 호출자·구현 전체를 검증한 보안 감사는 아니다. 판정은 MVP 제안이다.

76. `docs/public/문제해결/화면과 카메라/카메라-재생-프레임.md` — **KEEP**. Tick 재설정 때 기존 GPU 읽기와 CPU 프레임의 수명을 정리한 과거 문제·한계를 독자에게 제공한다. 런타임 호출·자원 소유는 없고 명령 기록 글과 다른 GPU 문제다. 과거 10회 확인이며 현재 픽셀 회귀 시험이 없다고 밝힌다.
77. `docs/public/문제해결/화면과 카메라/화면-프레임-지연.md` — **KEEP**. EO 픽셀 읽기, 지속 매핑과 남은 긴 프레임의 측정·원인 미확정을 설명한다. 사람을 위한 과거 기록이며 벤치마크 원자료는 삭제됐다고 명시한다. 광범위 벤치마크 기능을 유지할 근거는 아니고 EO 구현의 한계 기록으로 가치가 있다. 수치 재측정은 안 했다.
78. `docs/public/성능검증.md` — **SIMPLIFY**. 비교 조건과 측정 항목을 독자에게 제공한다. 중계 복구·드론 수 확장 시험 항목이 새 목표 밖이다. 네 대 임무와 EO 녹화에 필요한 확인만 남기고 기록이 없는 기준선을 추정하지 않는다. 실행 코드나 자원 소유는 없다.
79. `docs/public/시작하기.md` — **SIMPLIFY**. Windows 실행, 임무, 영상 저장과 장애 조작을 안내한다. 사용자가 입력을 제공하고 프로그램은 이 문서를 호출하지 않는다. 다중 홉·자동 복구 제거 후 화면에 맞게 안내를 정리하고 직접 통신·EO 로컬 저장은 보존한다. 안내된 화면을 실행해 확인하지 않았다.
80. `modules/communication/include/communication/Protocol.h` — **SIMPLIFY**. UDP 명령·응답 구조와 직렬화 계약, 순번 가드를 선언한다. 값·문자열·벡터를 소유하고 소켓은 소유하지 않는다. 앱·관제 클라이언트·ProtocolTests가 사용한다. 카메라 장애 명령 등 제거 기능에 연결된 변형을 구현/호출부와 함께 정리하되 비행·직접 명령·녹화는 보존한다. 오류는 디코더 예외이며 1200바이트·최대 8대 제한이다.
81. `modules/communication/include/communication/UdpSocket.h` — **KEEP**. 비차단 IPv4 UDP 송수신 인터페이스; unique_ptr 구현체가 소켓 수명을 소유하며 복사·이동을 금지한다. 빈 수신은 optional, 시스템 오류는 구현에서 예외다. 비슷한 가상망은 실제 OS UDP가 아니므로 중복으로 합치지 않는다. 관제와 시험이 사용한다.
82. `modules/communication/include/communication/UdpSocketPlatform.h` — **KEEP**. 플랫폼별 송수신 추상 인터페이스와 팩토리·IPv4 변환을 선언한다. unique_ptr 반환으로 구현체 소유권을 넘기며 가상 소멸자를 둔다. Windows/POSIX 구현과 공통 래퍼가 소비한다. 플랫폼 분기는 중복처럼 보여도 API가 달라 무작정 합치지 않는다.
83. `modules/communication/include/communication/VirtualMeshNetwork.h` — **SIMPLIFY**. 노드·장애물·메시지·설정 입력과 Tick별 전달·이벤트·통계를 선언하고 내부 큐·경로 상태를 소유한다. Direct에서도 같은 경로/이웃 체계를 사용한다. 다중 홉·리더 경유·장애 적응 및 관련 상태를 정리할 때 직접 링크, 명령 확인·재전송 필요성은 먼저 분리한다. 반환 span은 내부 벡터 수명/변경에 종속된다.
84. `modules/communication/src/Protocol.cpp` — **SIMPLIFY**. JSON 버전·종류·길이·숫자·배열 크기를 검사하고 명령/응답으로 변환한다. 자체 JSON 값으로 소유하며 잘못된 자료는 예외, 응답은 크기 제한을 확인한다. ProtocolTests가 왕복과 일부 오류를 확인하도록 작성되어 있으나 실행하지 않았다. 제거된 장애 명령만 정리하고 메시지 제한·직접 명령·녹화를 보존한다. EncodeCommand와 DecodeCommand의 유효성 검사 범위는 같지 않으므로 계약 변경 때 양쪽을 확인한다.
85. `modules/communication/src/UdpSocket.cpp` — **KEEP**. 공통 구현체가 플랫폼 팩토리와 송수신·주소 변환을 위임한다. unique_ptr로 플랫폼 객체를 소유하고 예외를 전파한다. 플랫폼과 책임이 분리되어 있으며 실제 UDP와 가상망은 중복 책임이 아니다.
86. `modules/communication/src/UdpSocketPosix.cpp` — **KEEP**. POSIX 소켓 생성·bind·비차단 수신·전송·IPv4 변환을 구현한다. RAII 핸들이 실패 경로에서도 소켓을 닫는다. 수신 없음/중단은 빈 값, 다른 오류는 예외이며 크기 초과 검출을 위해 한 바이트 더 받는다. Windows 구현과 유사하나 OS 자원과 오류가 다르다. Linux 시험은 미실행.
87. `modules/communication/src/UdpSocketWindows.cpp` — **KEEP**. Winsock 초기화와 UDP 소켓을 RAII로 관리하고 공통 인터페이스를 구현한다. 파괴 순서는 소켓 종료 후 Winsock 정리다. 큰 수신은 제한 초과 표시값을 반환하고 일시 오류는 빈 값, 나머지는 예외다. POSIX와 플랫폼 API가 다르며 Windows 실행은 미검증.
88. `modules/communication/src/VirtualMeshNetwork.cpp` — **SIMPLIFY**. 노드·장애물에서 거리/가림 링크를 만들고 HELLO, 경로 요청·응답·오류, 전송 예약, 확인 응답·재시도와 이벤트를 Tick별 처리한다. 컨테이너가 상태를 소유하며 설정 오류는 예외, 큐 포화·링크 실패는 통계와 이벤트로 기록한다. Direct도 HELLO와 경로 탐색을 거치므로 중계만 삭제할 때 직접 통신 의미와 테스트를 보존해야 한다. 정적 관찰: 중요 메시지 재시도에서 `SendMessage`가 계속 false이면 시도 횟수가 늘지 않아 제한 횟수 실패가 늦어질 수 있다. 재현·영향 확인 전 확정 결함으로 보지 않는다. 라우팅/직접/재시도 시험 파일 존재, 실행하지 않음.
89. `modules/graphics/include/graphics/Camera.h` — **KEEP**. 위치·목표·위쪽·시야 및 평면을 값으로 보유하고 뷰/투영 행렬을 제공한다. 생성자 계약은 화면 비율·평면 오류 예외를 밝히며 유한값/시야각 유효성은 호출자 책임이다. 앱·Renderer·CameraTests에 관계한다. EO와 장면 화면에 필요하며 구현 및 실제 화면은 미검증.
90. `modules/graphics/include/graphics/GlApi.h` — **KEEP**. 웹 GLES3 또는 네이티브 glad 헤더를 플랫폼 매크로에 따라 선택한다. 데이터·자원 소유나 별도 오류 처리는 없다. 같은 그래픽 호출에 플랫폼별 선언을 제공하므로 중복이 아니다.
91. `modules/graphics/include/graphics/GlbModel.h` — **KEEP**. GLB 경로로 로드해 파트들을 unique_ptr로 소유하고 변환·재질·텍스처 입력으로 그리기 및 인스턴스 설정을 선언한다. fallback 포인터는 비소유로 사용 시 수명 유지가 필요하다. 앱의 기체·환경 자산에 필요하다. 로드 실패 처리와 구현은 이 묶음에서 보지 않았다. Mesh::LoadGlb와 로딩 범위가 달라 중복 여부는 구현 대조 후 결정한다.
92. `modules/graphics/include/graphics/IndexBuffer.h` — **KEEP**. CPU 인덱스를 GPU 버퍼로 복사하고 핸들을 소유하며 소멸 시 해제한다. null/빈 입력 오류를 계약에 명시하지만 인덱스의 정점 범위는 호출자 책임이다. Mesh가 사용한다. GPU context 수명은 호출자 책임이고 실행 확인은 하지 않았다.
93. `modules/graphics/include/graphics/Material.h` — **KEEP**. 기본색·금속성·거칠기·물 효과를 값으로 보관해 Renderer/셰이더에 제공한다. 생성 시 값을 그대로 저장하고 셰이더가 일부 범위를 제한한다. 별도 자원 소유나 오류 처리는 없으며 GPU 인덱스/메시와 책임이 다르다.
94. `modules/graphics/include/graphics/Mesh.h` — **KEEP**. 정점·인덱스를 복사해 VAO/VBO/EBO와 인스턴스 버퍼를 unique_ptr로 소유하고 그리기 상태를 제공한다. 빈 입력·크기 초과 예외를 문서화하며 인덱스 범위 검사는 호출자 책임이다. GLB 경로와 GlbModel의 중복은 구현 대조가 필요하다. GPU context 생명주기와 로더는 미검증.
95. `modules/graphics/include/graphics/MultisampleFramebuffer.h` — **KEEP**. 다중 표본 색·깊이와 해결 텍스처 및 비동기 RGB 읽기 상태를 소유한다. 앱이 화면/EO 픽셀을 소비하며 빈 완료 상태는 false, 크기·구성 오류는 예외로 계약화한다. 출력의 이전 OpenGL 상태를 복원하지 않는 조건과 종료 시 대기 작업을 마쳐야 한다는 수명 계약이 중요하다. 타이밍 필드는 광범위 벤치마크 축소 때 실제 참조를 확인해 정리할 후보지만 EO 읽기 자체는 유지한다. 구현·GPU 실행은 미검증.

선정 시 미기록 infra UNREAD는 96개이며 이번 20개 이후 76개는 미확인이다. 외부 문서 원본·라이선스, 실행 결과, 그래픽 구현체 및 모든 호출 경로는 미검증이다. 다음 선택 시 변경된 inventory와 이 보고서를 다시 대조한다.

## 다섯 번째 묶음 — 현재 UNREAD 중 미기록 20개

선정 당시 inventory의 infra UNREAD 중 기존 기록 경로를 제외했다. 텍스트 20개 전체를 읽고 관련 호출을 일부 검색했다. 외부 원본 및 라이선스는 확인하지 않았으며 빌드·시험·sanitizer·OpenGL 실행은 하지 않았다. GPU 자원은 모두 현재 OpenGL context가 유효할 때 생성·해제해야 한다. 아래 판단은 MVP 제안이며 관련 구현과 시험 전수 확인이 아니다.

96. `modules/graphics/include/graphics/Renderer.h` — **KEEP**. 메시·재질·카메라 행렬과 비소유 텍스처 입력으로 장면을 그리는 인터페이스. 네이티브 framebuffer와 웹 화면 경로를 분기한다. 값으로 뷰 상태를 소유하며 GPU 리소스 소유는 각 객체다. 오류 계약은 선언에 없다. 앱/모델에서 사용하며 EO와 장면에 필요하다.
97. `modules/graphics/include/graphics/Shader.h` — **KEEP**. 파일 또는 소스에서 GPU 프로그램을 만들고 uniform을 전달한다. 프로그램 핸들과 이름별 위치 캐시를 소유하며 복사를 막는다. 파일·컴파일·연결 실패는 예외 계약이다. 셰이더/Renderer 책임은 다르다.
98. `modules/graphics/include/graphics/Texture2D.h` — **KEEP**. PPM/PNG/JPEG 파일이나 메모리 및 RGB 입력을 GPU 텍스처로 변환하는 계약이다. GPU 핸들을 소유하고 복사를 금지한다. GLB/앱에서 호출하며 파일/크기 오류는 구현이 예외 처리한다.
99. `modules/graphics/include/graphics/VertexArray.h` — **KEEP**. GPU 정점 형식·인덱스 연결 설정을 소유하며 Bind/UnBind를 제공한다. Mesh가 사용하고 CPU 정점 데이터는 소유하지 않는다. 별도 오류 보고는 없다; VBO와 역할이 다르다.
100. `modules/graphics/include/graphics/VertexBuffer.h` — **KEEP**. CPU byte를 GPU 버퍼로 복사하고 핸들을 소유한다. 음수 크기는 예외, 입력 포인터 범위는 호출자 책임이다. Mesh의 정점·인스턴스 버퍼가 사용한다.
101. `modules/graphics/include/graphics/Window.h` — **KEEP**. GLFW 창·context 수명과 입력·화면 교환을 제공한다. 생성 실패는 예외이며 GPU 객체는 창보다 먼저 정리해야 한다. 앱에 한 창이라는 계약이 있고 Renderer와 책임이 다르다.
102. `modules/graphics/src/graphics/Camera.cpp` — **KEEP**. 카메라 값으로 GLM view/projection을 계산하고 설정값을 갱신한다. 생성은 양의 화면 비율과 평면 순서를 검사하고 FOV setter는 유한 범위를 확인한다. 생성자의 FOV·NaN 및 시선/위쪽 벡터 유효성은 계약상 호출자 책임이다. 그래픽 시험 존재, 실행하지 않았다.
103. `modules/graphics/src/graphics/GlbData.h` — **KEEP**. GLB 파일 전체를 읽어 JSON·BIN을 소유하고 accessor의 범위·stride·정점·삼각형 인덱스를 검사해 값 배열로 만든다. 지원하지 않는 형식과 범위 오류는 예외다. GlbModel과 Mesh가 같은 파서를 재사용하므로 파싱 중복을 줄인다. 신뢰할 수 없는 거대 파일에 대한 자원 한도와 모든 입력의 안전성은 미검증이다.
104. `modules/graphics/src/graphics/GlbModel.cpp` — **KEEP**. GLB 노드 트리·변환·재질·텍스처를 파싱하고 각 Mesh/Texture를 unique_ptr로 소유해 Renderer로 그린다. 순환 노드·범위·지원하지 않는 형식은 예외다. Mesh 단일 GLB 경로와 `GlbData.h`를 공유한다. 정적 관찰: 외부 이미지 URI에서 절대 경로만 거부해 `..` 상대 경로가 모델 폴더 밖을 가리킬 수 있다. 신뢰되지 않은 GLB를 읽는 경우 경로 제한을 추가 검토해야 하며 악용·재현은 확인하지 않았다. 앱 기체·훈련장 모델이 호출한다.
105. `modules/graphics/src/graphics/IndexBuffer.cpp` — **KEEP**. 인덱스 배열을 GPU에 복사하고 소멸 시 삭제한다. null·0·GL 크기 제한을 검사해 예외를 낸다. Mesh가 사용하며 VAO에 EBO 연결을 남기는 계약이 있다. GL 호출 성공 자체는 확인하지 않는다.
106. `modules/graphics/src/graphics/Material.cpp` — **KEEP**. 색·금속성·거칠기·효과를 값으로 저장해 반환한다. 인자 검사는 수행하지 않고 셰이더 계산이 일부 범위를 제한한다. Renderer의 GPU 제출과 책임이 다르다.
107. `modules/graphics/src/graphics/Mesh.cpp` — **KEEP**. 정점·인덱스·인스턴스를 GPU 버퍼/VAO에 복사해 unique_ptr로 관리하고 단순 GLB 또는 정육면체를 만든다. null·크기·형식 오류는 예외다. 단일 GLB는 법선·인덱스가 필요하고 GlbModel은 장면·재질을 지원하므로 같은 파서라도 동일 책임은 아니다. 일반 생성자의 인덱스 유효성은 호출자 책임이다.
108. `modules/graphics/src/graphics/MultisampleFramebuffer.cpp` — **KEEP**. MSAA 색·깊이와 resolve 질감, 세 PBO 슬롯과 fence를 소유해 EO RGB를 비동기로 가져온다. 불완전 framebuffer는 정리 후 예외, 슬롯 부족/미완료/매핑 실패는 false, fence 실패는 완료 대기 후 정리한다. context와 PBO 수명이 핵심이며 앱 EO capture와 그래픽 시험이 호출한다. 시간 계측은 벤치마크 정리 시 참조 확인 후 줄일 후보이나 EO 기능은 유지한다. GPU 실행 미검증.
109. `modules/graphics/src/graphics/Renderer.cpp` — **KEEP**. 카메라·메시·재질·셰이더를 받아 viewport와 uniform을 설정하고 일반/인스턴스 그리기를 호출한다. 참조 자원을 소유하지 않는다. Debug에서 GL 오류를 종류별 한 번 보고하며 Release 확인은 없다. 웹 직접 픽셀 읽기와 네이티브 framebuffer는 플랫폼 경로가 다르다.
110. `modules/graphics/src/graphics/Shader.cpp` — **KEEP**. 단계별 GLSL 파일을 읽고 컴파일·연결하며 uniform 위치를 캐시한다. 임시 shader/program은 RAII로 실패 시 해제하고 성공 후 프로그램을 소유한다. 컴파일/링크 실패는 로그와 예외다. 웹용 GLSL 변환도 담당하며 셰이더 파일과 책임이 다르다.
111. `modules/graphics/src/graphics/Texture2D.cpp` — **KEEP**. PPM과 stb PNG/JPEG를 RGB로 읽어 GPU에 올리고 핸들을 삭제한다. stb 이미지는 unique_ptr deleter로 해제하고 크기·넘침·잘린 입력에 예외를 낸다. 파일/메모리 경로는 공통 업로드를 공유한다. GL 할당 실패 및 비신뢰 이미지 자원 한도는 실행 검증하지 않았다. stb의 외부 원본 라이선스는 이번에 조사하지 않았다.
112. `modules/graphics/src/graphics/VertexArray.cpp` — **KEEP**. VAO 생성·선택·해제 및 삭제를 담당한다. Mesh가 소유한다. GL 실패는 별도 검사하지 않으며 VBO와 자원이 다르다.
113. `modules/graphics/src/graphics/VertexBuffer.cpp` — **KEEP**. CPU byte를 GL 버퍼로 업로드하고 삭제한다. 음수 크기는 예외, 포인터 범위는 호출자 책임이다. Mesh의 정점 및 인스턴스에 재사용되며 인덱스 버퍼와 바인딩 대상이 다르다.
114. `modules/graphics/src/graphics/Window.cpp` — **KEEP**. GLFW 초기화·창/context 생성, 콜백·입력·화면 교환 및 종료를 담당한다. 초기화 실패 시 생성된 창과 GLFW를 정리한다. 한 창 사용을 전제하며 다른 GPU 객체의 수명이 더 짧아야 한다. 실제 창 생성과 스레드 조건은 미검증.
115. `modules/manual_control/include/manual_control/GamepadCommandMapper.h` — **KEEP**. 패드 축과 기체 yaw를 월드 속도·회전 목표로 변환하고 입력 소유권·연결 해제 정책 계약을 정의한다. 값만 보유하고 장치/시뮬레이션은 소유하지 않는다. 앱과 단위 시험에서 호출한다. 구현은 이번 범위 밖이므로 비정상 입력·안전 동작은 미검증; 자동 장애 복구와 별개인 수동 입력 정책이다.

선정 당시 미기록 infra UNREAD 66개 중 이번 20개 이후 46개가 미확인이다. 모든 외부 원본, 실제 GPU 동작, 전체 호출 관계 및 테스트 결과는 미검증이다. 조사하지 않은 경로는 UNREAD로 유지한다.

## 여섯 번째 묶음 — 현재 UNREAD 중 미기록 20개

선정 당시 현재 inventory의 infra UNREAD에서 기존 감사 기록을 제외했다. 20개 텍스트 전체를 읽고 호출·시험 참조를 일부 검색했다. 빌드·시험·네트워크·영상 실행 및 외부 라이선스 원본은 확인하지 않았다. 시나리오는 ScenarioRunner/headless 또는 시험이 읽는 입력과 기대값이며 프로그램·자원을 소유하지 않는다. 판정은 MVP 제안이고 삭제 작업은 수행하지 않았다.

116. `modules/manual_control/src/GamepadCommandMapper.cpp` — **KEEP**. 축 입력과 yaw로 데드존·감도 곡선 및 월드 속도/회전 목표를 반환하고 입력 소유권·연결 해제 시 정지/착륙을 선택한다. 값만 사용하고 `noexcept`다. 앱과 단위 시험이 호출한다. 연결 해제 정책은 영상 전송·다중 홉 자동 복구와 다른 수동 입력 책임이다. 비유한 축 입력은 명시적으로 거르지 않으며 실제 장치 실행은 미검증.
117. `modules/media/include/media/RecordedCaptureTransfer.h` — **REMOVE**. 녹화 영상·메타데이터·목록을 TCP로 전송·수신하고 CRC 결과를 돌려주는 계약이다. 수신기는 구현체를 unique_ptr로 소유하고 실패는 bool과 오류 문자열로 전달한다. 앱 전송 호출과 통합 시험이 있다. 로컬 녹화와 분리된 영상 전송이므로 관련 호출·빌드·시험과 함께 제거 후보.
118. `modules/media/include/media/RtpVideoStream.h` — **REMOVE**. EO RGB 프레임의 RTP/RTCP 송수신·최신 프레임 및 통계를 선언한다. 송수신기 구현체를 unique_ptr로 소유하며 실패를 bool/오류로 전달한다. 앱과 전송 시험이 사용한다. 로컬 프레임 보관과는 다른 전송 기능이다.
119. `modules/media/src/RecordedCaptureTransfer.cpp` — **REMOVE**. 세 파일의 이름·크기·CRC 헤더와 TCP 바이트를 보내고 수신 작업자는 부분 파일에 저장·검증 후 확정한다. 소켓·스레드 및 결과를 관리하며 실패는 오류 문자열, 부분 파일은 실패 시 삭제한다. 앱과 테스트 호출을 함께 정리해야 한다. CRC는 손상 검출이지 인증이 아니다. 정적 관찰로 수신 파일 크기 상한과 총 수신 제한은 보이지 않는다; 실제 악용 가능성은 미검증. 로컬 MP4 녹화와 책임이 다르다.
120. `modules/media/src/RtpVideoStream.cpp` — **REMOVE**. GStreamer H.264 RTP/RTCP 송신과 수신·디코딩, 최신 프레임 큐를 구현한다. 작업 스레드·파이프라인·버퍼를 관리하며 오류 문자열과 상태를 반환한다. 앱/통합 시험이 사용하므로 호출과 CMake GStreamer 의존성을 함께 정리한다. EO 로컬 녹화에 쓰는 OpenCV VideoWriter는 별도 경로다. 코덱·실제 전송 및 외부 라이선스는 미검증.
121. `modules/sensor/include/sensor/RgbFrame.h` — **KEEP**. 프레임 크기·Tick·RGB 바이트와 행 순서를 값으로 소유하며 유효성 확인 및 뒤집기 계약을 제공한다. EO 캡처·분석·녹화에서 쓸 수 있는 공통 자료이며 전송 구조체와 중복이 아니다.
122. `modules/sensor/src/RgbFrame.cpp` — **KEEP**. 크기 곱셈을 확인해 정확한 픽셀 수를 검사하고 유효한 프레임의 행을 뒤집는다. 잘못된 입력에서 뒤집기는 아무 작업을 하지 않는다. 픽셀 vector는 프레임이 소유한다. 시험 파일 존재, 실행하지 않았다.
123. `modules/vision/include/vision/RgbFrameQuality.h` — **KEEP**. RGB의 밝기·표준편차·라플라시안 분산 분석 계약과 재사용 작업 영상을 제공한다. 인스턴스는 작업 Mat을 소유하므로 한 스레드에서 순차 호출해야 하고 잘못된 입력은 예외다. 품질 지표는 전송이나 자동 객체 판정과 다르다.
124. `modules/vision/include/vision/RgbFrameView.h` — **KEEP**. 유효한 프레임을 복사 없이 OpenCV Mat으로 빌려 보여 준다. 픽셀 소유자는 원본이고 원본 수명·재할당 전까지만 유효하다. 행 순서와 RGB 해석은 호출자 책임, 오류는 예외다. 앱 EO 분석에서 호출한다.
125. `modules/vision/src/RgbFrameQuality.cpp` — **KEEP**. RGB 형식을 검사하고 회색 변환·평균/표준편차·라플라시안 분산을 계산한다. 작업 Mat을 재사용하며 잘못된 형식은 예외다. 품질 지표는 로컬 EO 화면에 관련되며 객체 탐지는 아니다. 시험은 실행하지 않았다.
126. `modules/vision/src/RgbFrameView.cpp` — **KEEP**. 프레임 검증 후 원본 픽셀을 참조하는 Mat을 만든다. 소유권을 이전하지 않으며 잘못된 크기는 예외다. 영상 복사·품질 계산과 책임이 다르다. 호출자의 프레임 수명은 별도 확인해야 한다.
127. `scenarios/fault-injection/fault-injection.json` — **REMOVE**. 카메라 장애·명령 중복/지연/손실을 포함한 운동학 입력과 예상 상태를 제공한다. CMake headless smoke가 참조한다. 제거될 장애 주입에 초점을 둔 시험이므로 해당 기능 제거와 함께 정리; 정상 EO·직접 통신 회귀를 별도로 보존한다.
128. `scenarios/flight/normal-flight.json` — **KEEP**. 한 기체의 이륙·경유·착륙과 카메라 상태 기대값을 제공한다. 비행 기초 회귀 자료이며 장애 시나리오와 다르다. 이번 검색에서 직접 호출 위치는 확인하지 못했다; 사용 여부와 실제 실행은 미검증.
129. `scenarios/flight/physical-model-override.json` — **KEEP**. 강체 제원 재정의·이륙 입력과 허용 위치를 제공하며 ScenarioRunnerTests가 참조한다. 물리 제원 입력 경로를 검증하는 독립 사례다. 기대 위치 허용치가 넓으므로 정확한 물리 성능 증거는 아니다.
130. `scenarios/flight/rigid-body-flight.json` — **KEEP**. 강체 모드의 바람·이륙·착륙 및 예상 위치를 제공하며 CMake headless smoke가 참조한다. 편대 임무와 다른 기초 물리 확인이다. 실행 결과는 미검증.
131. `scenarios/multi-drone/energy-and-outage-reallocation.json` — **REMOVE**. 저전력·통신 단절에서 임무 재배정을 시험하는 네 기체 입력과 기대값이다. ScenarioRunnerTests와 CooperativeMissionBenchmark가 참조한다. 자동 장애 복구와 광범위 benchmark 제거 때 관련 테스트와 함께 정리한다.
132. `scenarios/multi-drone/four-drone-survey-outage.json` — **SIMPLIFY**. 네 기체 정찰·촬영면·단절·재배정·최소 간격 기대값을 제공하며 headless smoke, ScenarioRunnerTests 및 benchmark가 참조한다. 단절·자동 재배정은 제거 대상이지만 네 기체 정찰과 EO 촬영 조건은 남길 가치가 있어 정상 임무 시험으로 대체/분리 여부를 확인한다. 현재 시험은 실행하지 않았다.
133. `scenarios/multi-drone/independent-flight.json` — **KEEP**. 두 기체의 독립 이륙·이동과 최소 간격을 기술하며 CMake smoke가 참조한다. 정상 다중 기체 기본 동작과 장애 재배정은 다른 범위다.
134. `scenarios/multi-drone/leader-following-outage.json` — **SIMPLIFY**. 리더·추종 오프셋과 일시 단절, 최종 위치를 기술하며 CMake smoke가 참조한다. 정상 편대 추종은 필요하지만 장애 복구 요구는 제외하므로 직접 통신의 정상 추종 확인과 분리한다.
135. `scenarios/multi-drone/low-battery-reallocation.json` — **REMOVE**. 저전력 착륙과 다른 기체의 목표 인수를 위한 입력/기대값이며 ScenarioRunnerTests가 참조한다. 자동 장애 재배정 제거 범위에 해당한다. 정상 비행의 배터리 표시·기본 동작까지 삭제한다는 뜻은 아니다.

선정 당시 미기록 infra UNREAD 43개 중 이번 20개 이후 23개가 미확인이다. 외부 라이선스·실행·모든 호출 관계는 미검증이며 조사하지 않은 파일은 UNREAD다.

## 일곱 번째 묶음 — 남은 infra UNREAD 17개

선정 시 현재 inventory의 infra UNREAD 중 보고서에 없는 17개였다. 일반 텍스트 11개는 전체 구간을 읽었다. 두 수치 JSON은 파일 전체를 로컬 JSON 파싱해 구조·표본 수·출처·참조를 확인한 **METADATA_ONLY**이며 숫자 배열의 개별 값은 감사하지 않았다. 비공개 문서 4개는 전체 파일을 기계적으로 읽어 크기·줄 수만 확인했으며 본문 내용 감사는 **UNREAD**다. 외부 원본·라이선스, 빌드·시험·실행은 검증하지 않았다. 다음 판정은 제안이며 제품 변경을 뜻하지 않는다.

136. `scenarios/multi-drone/rigid-body-energy-and-outage.json` — **REMOVE**, 완독 1–84행. 강체 4기 저전력·단절 재배정과 최소 거리 기대값을 제공한다. ScenarioRunnerTests와 CooperativeMissionBenchmark가 읽는다. 자동 장애 재배정·광범위 benchmark 제거에 해당하며 정상 강체 비행 확인은 별도 유지한다. 입력 데이터로서 자원 소유나 자체 오류 처리는 없다.
137. `scenarios/multi-drone/two-point-survey.json` — **KEEP**, 완독 1–43행. 두 기체의 정상 지점 분담 및 기대 위치를 제공하는 운동학 시나리오다. 검색에서 시나리오 이름 외 직접 참조는 확인하지 못했다. 4기 MVP 시험과 중복인지 전체 호출·실행을 확인한 뒤 판단한다. 자원 소유·자체 오류 처리는 없다.
138. `tests/communication/GroundControlClientTests.cpp` — **KEEP**, 완독 1–67행. 로컬 UDP 소켓 두 개와 작업자를 사용해 명령·일치 응답 및 IPv4 변환을 확인한다. 소켓/스레드는 시험 범위 수명이며 실패는 GoogleTest 검사로 나타난다. 직접 통신 클라이언트가 유지되면 정상 응답 회귀가 필요하다. 재전송·취소·실제 네트워크는 다루지 않는다.
139. `tests/communication/ProtocolTests.cpp` — **SIMPLIFY**, 완독 1–205행. 명령·응답 왕복, 크기·형식 거부, 중복·역순 판정을 검사한다. 자료는 시험 값이 소유하고 예상 오류는 예외 검사다. 카메라 장애 등 제거된 기능의 사례는 정리하되 직접 명령·로컬 녹화·응답 크기 검사는 유지한다.
140. `tests/communication/VirtualMeshNetworkTests.cpp` — **SIMPLIFY**, 완독 1–300행 및 301–594행. 가상망 경로 비교·대체 경로·중계·거리/가림·손실·재시도·중복 억제·큐 제한을 시험한다. 네트워크와 메시지는 시험 값이 소유하고 GoogleTest로 실패를 보고한다. 다중 홉·자동 복구 사례는 제거하되 직접 링크 전달·가림·필요한 확인 응답 사례를 분리해 보존한다. 실행은 미검증.
141. `tests/data/px4_x500_position_replay_12_05.json` — **KEEP, METADATA_ONLY**. 전체 125,413바이트 JSON 파싱 성공; 드론 1개, 명령 240개, 참조 표본 241개 및 좌표·입력 메타데이터가 있다. 저장된 출처는 PX4 Flight Review이고 라이선스 표기는 CC-BY PX4다. ScenarioRunnerTests가 참조한다. 실제 비행 기록과 비교하는 기준 자료이며 유사한 다른 기록과 입력 출처가 다르다. 외부 원본·라이선스 독립 확인 및 개별 수치 검증은 하지 않았다.
142. `tests/data/px4_x500_position_replay_2026_10_16_100s.json` — **KEEP, METADATA_ONLY**. 전체 152,286바이트 JSON 파싱 성공; 드론 1개, 명령 240개, 참조 표본 241개 및 ESC 회전수·지면 기준 메타데이터가 있다. 저장된 출처는 PX4 Flight Review, 라이선스 표기는 CC-BY PX4다. ScenarioRunnerTests가 참조한다. 앞 파일과 측정 입력이 다르므로 단순 중복이 아니다. 외부 원본·라이선스 독립 확인과 개별 수치는 미검증.
143. `tests/graphics/CameraTests.cpp` — **KEEP**, 완독 1–261행. 카메라 위쪽 축, GLB 텍스처 좌표와 임시 GLB를 숨김 창에서 렌더링해 픽셀을 읽는 과정을 검사한다. 시험은 임시 파일·창·GPU 자원을 수명 순서로 관리하며 GoogleTest가 실패를 보고한다. EO/장면 유지에 유용하다. 실행과 실제 기체 카메라 화면 통합은 미검증.
144. `tests/manual_control/GamepadCommandMapperTests.cpp` — **KEEP**, 완독 1–48행. 데드존·yaw 회전과 연결 해제 정책을 값 단위로 검사한다. 장치·창을 소유하지 않으며 실제 게임패드 분리와 입력 대기열은 확인하지 않는다. 수동 조종을 유지하면 독립 회귀 가치가 있다.
145. `tests/media/RtpVideoStreamTests.cpp` — **REMOVE**, 완독 1–154행. 로컬 RTP 인코딩·수신과 TCP 녹화 파일 전달·CRC를 검사한다. 시험은 임시 폴더·송수신기·포트를 소유하며 실패는 assertion으로 보고한다. 영상 전송 제거 시 구현·CMake 대상과 함께 정리하고 로컬 녹화 시험을 별도 보존한다. 실행하지 않았다.
146. `tests/sensor/RgbFrameTests.cpp` — **KEEP**, 완독 1–41행. RGB 크기 검사와 행 뒤집기·원상 복원 및 잘못된 큰 크기를 검사한다. 프레임 벡터를 시험이 소유하고 잘못된 입력은 유효하지 않다고 판정한다. EO 로컬 영상의 기본 회귀다.
147. `tests/vision/RgbFrameQualityTests.cpp` — **KEEP**, 완독 1–51행. 작은 RGB 입력의 밝기·대비·선명도와 위아래 반전 불변성·잘못된 형식 거부를 검사한다. 시험이 Mat을 소유하며 예외를 검사한다. 앱의 EO 품질 측정과 관련되고 대상 인식 시험은 아니다.
148. `tests/vision/RgbFrameViewTests.cpp` — **KEEP**, 완독 1–31행. OpenCV 뷰가 원본 픽셀을 공유하고 잘못된 프레임을 거부하는지 검사한다. 원본이 픽셀을 소유하며 뷰는 빌린다. 품질 계산과 별개인 수명/공유 계약을 확인한다.
149. `docs/private/개발기록.md` — **UNREAD**, 전체 파일 기계적 읽기만 완료. 이름상 개인 개발 기록이며 내용·호출·중복·오류·MVP 판정은 검토하지 않았다. 본문과 개인 정보를 이 보고서에 옮기지 않는다.
150. `docs/private/개발안내.md` — **UNREAD**, 전체 파일 기계적 읽기만 완료. 이름상 개인 개발 안내이며 본문 역할·상태 및 MVP 판정은 미검토다. 비공개 내용을 옮기지 않는다.
151. `docs/private/기술설계.md` — **UNREAD**, 전체 파일 기계적 읽기만 완료. 이름상 개인 기술 설계 자료이며 본문·현재 코드와의 관계 및 MVP 판정은 미검토다. 비공개 내용을 옮기지 않는다.
152. `docs/private/문제해결.md` — **UNREAD**, 전체 파일 기계적 읽기만 완료. 이름상 개인 문제 해결 기록이며 본문·공개 기록과의 중복 및 MVP 판정은 미검토다. 비공개 내용을 옮기지 않는다.

이번 선택 17개 중 텍스트 완독 11개, JSON 전체 파싱·메타데이터 확인 2개, 본문 감사 UNREAD 4개다. 비공개 문서를 기계적으로 읽은 사실은 의미·호출 관계까지 감사했다는 뜻이 아니다. 공통 inventory의 상태는 변경하지 않았다.

## 비공개 문서 본문 확인 — 앞선 UNREAD 상태 갱신

아래 네 문서를 끝까지 읽었다. 비공개 내용은 옮기지 않고 문서 역할과 현재 MVP에 대한 판단만 적는다. 이 판단은 문서 정적 검토이며 원본 기록, 현재 구현과 시험의 독립 검증은 하지 않았다. 앞선 149–152항의 UNREAD 표시는 이 구간 확인으로 대체한다. 문서 자체의 변경·삭제는 하지 않았다.

149. `docs/private/개발기록.md` — **KEEP**, 완독 1–101행. 과거 개발·검증 이력을 모은 내부 기록이다. 현 MVP의 구현 기준으로 사용하기보다 당시 근거와 한계를 보존할 자료다. 과거 기능을 제거해도 기록을 바로 삭제할 이유는 확인되지 않았다. 현재 상태로 오해될 여지가 있는 표현은 별도 문서 정리 시 기준 시점을 명확히 할 후보이나, 세부 항목은 여기 옮기지 않는다.
150. `docs/private/개발안내.md` — **SIMPLIFY**, 완독 1–91행. 개발자가 작업을 이어갈 때 보는 내부 안내다. 현 MVP 및 현재 구현과 시점 차이가 있어 최신 기준 문서와 대조해 갱신하거나 역사 문서로 명확히 표시할 필요가 있다. 공개 실행 안내와 역할은 다르지만 중복 가능성이 있어 새 문서를 늘리기보다 기존 안내를 정리한다.
151. `docs/private/기술설계.md` — **SIMPLIFY**, 완독 1–87행. 과거 구조와 데이터 흐름을 설명하는 내부 설계 자료다. 현재 구현 및 MVP와 시점 차이가 있으므로 코드 확인 후 갱신하거나 기록 시점을 명시할 필요가 있다. 공개 기술 설명과 목적은 비슷해 서로 다른 시점의 설명을 현재 사실로 혼동하지 않도록 정리한다.
152. `docs/private/문제해결.md` — **SIMPLIFY**, 완독 1–66행. 개발 환경에서 문제를 좁히는 내부 안내다. 남는 빌드·직접 통신·EO 흐름을 기준으로 유효성을 대조하고 제거되는 기능의 절차와 낡은 설명은 정리할 후보이다. 공개 문제 해결 기록과 대상 독자는 다르며 현재 절차는 실행해 확인하지 않았다.

네 문서 본문은 모두 완독했다. 내용의 사실성, 이력에 적힌 시험 결과, 민감 세부 사항의 공개 적합성은 별도 검증하지 않았다. 공통 inventory는 수정하지 않았다.
