# 검증과 측정

## 자동 시험

Windows MSVC Release 빌드와 CTest **20개가 모두 통과했습니다.** 시험은 드론 비행과 재현성, 카메라 상태와 RGB 행 순서, OpenCV 프레임 공유와 밝기·대비 계산, JSON 장애 시나리오, UDP 메시지와 지상통제 명령 왕복, 화면에서 카메라 픽셀을 읽는지 확인합니다.

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

OpenCV 분석을 연결한 뒤 같은 환경에서 1회 더 측정했습니다. 297 Tick 동안 화면을 2,017번 그렸고, 생성된 RGB 영상 292개를 분석했습니다.

| 모드 | 모의 Tick | 분석 영상 수 | 밝기 평균 | 대비 평균 | 분석 시간 p50 / p95 / p99 | 숨김 그리기 속도 | 메모리 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| OpenCV 분석 포함 | 297 | 292 | 165.03 | 71.33 | 642 / 712 / 892µs | 403.4 FPS | 64.1MB |

분석 시간은 프레임 픽셀을 읽어 평균 밝기와 채널별 표준편차를 계산하는 데 걸린 시간입니다. 밝기와 대비는 이 시나리오에서 나온 값이며, 영상 품질의 일반 점수는 아닙니다. 분석 포함 측정은 1회뿐이라 이전 세 번의 평균과 FPS만 비교해 성능 차이를 단정하지 않습니다. 원본 측정값과 실행 정보는 같은 [Release 결과 JSON](../benchmarks/results/aerosimlab-mvp-release.json)에 보관했습니다.

Tick 지연은 드론과 카메라의 모의 갱신 함수만 잰 값입니다. 숨김 그리기 속도는 실제 화면에 표시할 때의 FPS가 아닙니다. CPU 비율은 프로세스 CPU 시간을 측정 시간으로 나눈 값입니다. 여러 코어를 쓰면 100%를 넘을 수 있습니다.

첫 측정에서는 숨김 그리기 속도가 26~39 FPS였지만, 같은 날 재측정한 결과는 약 403 FPS였습니다. 두 기록의 차이를 설명할 GPU·OpenGL 정보와 코드 버전이 첫 측정에는 없어 원인을 확정할 수 없습니다. 따라서 두 값을 성능 향상이나 회귀의 근거로 비교하지 않습니다. 재측정부터 화면 크기와 GPU·OpenGL 정보를 함께 출력하도록 바꿨습니다.

측정 원본과 재측정값은 [Release 결과 JSON](../benchmarks/results/aerosimlab-mvp-release.json)에 보관했습니다. 이후 성능 변화는 같은 실행 파일 설정과 같은 환경에서 다시 측정해 판단해야 합니다. 이전 GraspLink 측정과 입력·환경이 달라 직접 비교할 수 없습니다.

## WebAssembly 확인

Emscripten SDK 환경을 활성화한 터미널에서 다음 명령으로 빌드할 수 있습니다.

```powershell
cmake --preset emscripten-single-thread
cmake --build --preset emscripten-single-thread
```

Emscripten 4.0.19로 화면 프로그램과 헤드리스 실행 파일을 빌드했습니다. Node.js에서 정상 비행과 장애 주입 시나리오를 모두 실행해 `passed: true`를 확인했습니다. 정상 비행은 300 Tick 뒤 착륙했고, 장애 시나리오는 카메라 고장·복구와 명령 지연·손실을 재현했습니다. 브라우저에서 화면 프로그램과 관련 파일을 모두 내려받았지만, 화면 표시와 입력 조작은 확인하지 못했습니다.

## 확인하지 않은 항목

- 실제 모니터에 표시되는 FPS와 GPU 사용량
- 브라우저에서 화면을 띄우고 입력을 조작하는 시험
- 다른 PC, Debug 빌드, 장시간 실행과 반복 종료
- 실행 중 네트워크가 끊긴 뒤 같은 연결이 자동으로 복구되는 동작과 실제 패킷 손실·지연
- 영상 파일 저장과 여러 조건에서 반복한 OpenCV 분석 성능 비교
- 실제 드론 기체의 비행 성능과 공기역학
