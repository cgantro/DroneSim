# AeroSimLab

AeroSimLab은 가상 드론과 탑재 카메라를 시험하는 C++ 시뮬레이터입니다. 지금은 화면과 분리된 드론 운동 모형과 JSON 시나리오 실행 기능이 있습니다. 카메라 영상, 지상통제 프로그램, UDP 통신은 아직 구현하지 않았습니다.

현재 화면 프로그램에는 이전 로봇 시연이 남아 있습니다. 드론이 화면에서 움직이는 기능은 아직 연결하지 않았습니다.

## 빌드와 실행

Windows에서는 Visual Studio 개발자 PowerShell에서 다음 명령을 실행합니다.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

화면 없이 예제 시나리오를 실행합니다.

```powershell
.\build\aerosimlab_headless.exe --scenario scenarios/normal-flight.json
```

출력의 `passed`가 `true`면 명령 재생과 예상 상태 비교가 통과한 것입니다. `--help`는 실행 방법을 보여 줍니다.

기존 화면 프로그램은 다음과 같이 실행합니다.

```powershell
.\build\aerosimlab_simulator.exe
```

## 시험

```powershell
ctest --test-dir build --output-on-failure
```

시나리오 실행 결과는 시작 위치, 목표 위치, 비행 상태가 정해진 값과 맞는지 확인합니다. 현재 시험에는 이전 로봇 기능도 남아 있으며, 드론 요구에 맞춰 정리할 예정입니다.

## 문서

[전환 전 코드 감사](docs/AEROSIMLAB_AUDIT.md) · [문서 안내](docs/README.md)
