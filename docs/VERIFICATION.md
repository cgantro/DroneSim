# 검증과 측정

## 자동 시험

Windows MSVC Release 빌드와 CTest **25개가 모두 통과했습니다.** 시험은 드론 비행과 재현성, 카메라 상태와 RGB 행 순서, JSON 장애 시나리오, UDP 메시지 검사, 실제 지상통제 명령 왕복, 화면에서 카메라 픽셀을 읽는지 확인합니다.

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

측정은 2026년 10월 21일, Windows x64와 MSVC Release에서 수행했습니다. CPU는 13th Gen Intel Core i5-13600KF입니다. 1280×720 OpenGL 창을 화면에 표시하지 않고 VSync를 끈 채 매번 5초 동안 실행했으며 세 번 반복했습니다.

| 실행 | 모의 Tick | Tick p50 / p95 / p99 | 16.67ms 초과 | 숨김 그리기 속도 | 메모리 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1회 | 299 | 0.1 / 0.5 / 0.6µs | 0 | 35.1 FPS | 59.2MB |
| 2회 | 299 | 0.1 / 0.5 / 0.7µs | 0 | 38.8 FPS | 59.2MB |
| 3회 | 299 | 0.0 / 0.5 / 0.6µs | 0 | 26.1 FPS | 59.2MB |

Tick 지연은 드론과 카메라의 모의 갱신 함수만 잰 값입니다. 숨김 그리기 속도는 실제 화면에 표시할 때의 FPS가 아니며 실행마다 차이가 컸습니다. CPU 비율은 프로세스 CPU 시간을 측정 시간으로 나눈 값입니다. 여러 코어를 쓰면 100%를 넘을 수 있습니다.

측정 원본은 [Release 결과 JSON](../benchmarks/results/aerosimlab-mvp-release.json)에 보관했습니다. 이 자료는 새 코드의 첫 기준값입니다. 이전 GraspLink 측정과 입력·환경이 달라 성능 개선 비교에는 쓸 수 없습니다.

## 확인하지 않은 항목

- 실제 모니터에 표시되는 FPS와 GPU 사용량
- Emscripten 브라우저 빌드와 웹 실행
- 다른 PC, Debug 빌드, 장시간 실행과 반복 종료
- 실제 네트워크 단절 뒤 재접속을 포함한 UDP 복구
- 영상 파일 저장과 OpenCV 영상 처리
- 실제 드론 기체의 비행 성능과 공기역학
