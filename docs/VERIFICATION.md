# 검증과 측정

## 자동 시험

Windows MSVC Release 빌드와 CTest **19개가 모두 통과했습니다.** 시험은 드론 비행과 재현성, 카메라 상태와 RGB 행 순서, OpenCV와 프레임 메모리 공유, JSON 장애 시나리오, UDP 메시지와 지상통제 명령 왕복, 화면에서 카메라 픽셀을 읽는지 확인합니다.

다시 실행하려면 Visual Studio 개발자 PowerShell에서 다음을 입력합니다.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

정상 비행과 장애 시나리오 결과는 화면 없이 확인할 수 있습니다.

```powershell
.\build\aerosimlab_headless.exe --scenario scenarios/normal-flight.json
.\build\aerosimlab_headless.exe --scenario scenarios/fault-injection.json
```

장애 시나리오는 이륙 명령 중복, 경유점 명령 지연, 착륙 명령 손실, 카메라 고장과 복구를 재현합니다. 결과에는 마지막 드론·카메라 상태와 이벤트 기록이 포함됩니다.

## Release 측정

측정은 2026년 10월 21일, Windows x64와 MSVC Release에서 수행했습니다. CPU는 13th Gen Intel Core i5-13600KF이고 그래픽카드는 NVIDIA GeForce RTX 3060 Ti입니다. 1280×720 OpenGL 창을 화면에 표시하지 않고 VSync를 끈 채 매번 5초 동안 실행했습니다.

| 재측정 | 모의 Tick | Tick p50 / p95 / p99 | 16.67ms 초과 | 숨김 그리기 속도 | 메모리 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1회 | 298 | 0.3 / 0.5 / 0.9µs | 0 | 402.2 FPS | 63.6MB |
| 2회 | 298 | 0.3 / 0.5 / 0.7µs | 0 | 404.7 FPS | 63.6MB |
| 3회 | 297 | 0.3 / 0.5 / 1.0µs | 0 | 403.2 FPS | 63.6MB |

Tick 지연은 드론과 카메라의 모의 갱신 함수만 잰 값입니다. 숨김 그리기 속도는 실제 화면에 표시할 때의 FPS가 아닙니다. CPU 비율은 프로세스 CPU 시간을 측정 시간으로 나눈 값입니다. 여러 코어를 쓰면 100%를 넘을 수 있습니다.

첫 측정에서는 숨김 그리기 속도가 26~39 FPS였지만, 같은 날 재측정한 결과는 약 403 FPS였습니다. 두 기록의 차이를 설명할 GPU·OpenGL 정보와 코드 버전이 첫 측정에는 없어 원인을 확정할 수 없습니다. 따라서 두 값을 성능 향상이나 회귀의 근거로 비교하지 않습니다. 재측정부터 화면 크기와 GPU·OpenGL 정보를 함께 출력하도록 바꿨습니다.

측정 원본과 재측정값은 [Release 결과 JSON](../benchmarks/results/aerosimlab-mvp-release.json)에 보관했습니다. 이후 성능 변화는 같은 실행 파일 설정과 같은 환경에서 다시 측정해 판단해야 합니다. 이전 GraspLink 측정과 입력·환경이 달라 직접 비교할 수 없습니다.

## WebAssembly 확인

Emscripten 4.0.19로 화면 프로그램과 헤드리스 실행 파일을 빌드했습니다. Node.js에서 정상 비행과 장애 주입 시나리오를 모두 실행해 `passed: true`를 확인했습니다. 정상 비행은 300 Tick 뒤 착륙했고, 장애 시나리오는 카메라 고장·복구와 명령 지연·손실을 재현했습니다. 브라우저에서 화면 프로그램과 관련 파일을 모두 내려받았지만, 화면 표시와 입력 조작은 확인하지 못했습니다.

## 확인하지 않은 항목

- 실제 모니터에 표시되는 FPS와 GPU 사용량
- 브라우저에서 화면을 띄우고 입력을 조작하는 시험
- 다른 PC, Debug 빌드, 장시간 실행과 반복 종료
- 실제 네트워크 단절 뒤 재접속을 포함한 UDP 복구
- 영상 파일 저장과 밝기·화질 분석
- 실제 드론 기체의 비행 성능과 공기역학
