# AeroSimLab

AeroSimLab은 가상 드론과 탑재 RGB 카메라를 운용하고 시험하는 작은 C++ 시뮬레이터입니다. 화면 프로그램, 별도 지상통제 프로그램, 화면 없이 실행하는 시나리오 시험을 제공합니다.

## 지금 되는 기능

- 60Hz 고정 간격으로 드론의 이륙·이동·경유점 비행·착륙을 계산합니다.
- 왼쪽에는 주변을 보는 화면, 오른쪽에는 드론에 붙은 EO 카메라 시점을 그립니다.
- EO 화면의 RGB 픽셀을 Tick 번호와 함께 메모리 프레임으로 읽습니다.
- 카메라 켜기·끄기, 고장 주입과 복구를 모의합니다. 카메라가 꺼지거나 고장 나면 오른쪽 영상이 멈춥니다.
- 지상통제 프로그램에서 UDP로 드론과 카메라를 제어하고, 적용 Tick과 상태를 확인합니다.
- GLFW가 인식하는 게임패드로 이륙·착륙과 수동 비행을 할 수 있습니다. 연결이 끊기면 비행 중 속도 명령을 지웁니다.
- JSON 시나리오에서 통신 지연·손실·중복과 카메라 고장·복구를 재생합니다.

드론은 상자 모양이고 운동은 단순한 위치·속도 계산입니다. 공기역학, 실제 비행 성능, GLB 모델 불러오기, 영상 파일 저장과 OpenCV 분석은 구현하지 않았습니다.
게임패드는 GLFW의 표준 게임패드 매핑을 사용합니다. A는 이륙, B는 착륙, 왼쪽 스틱은 이동, 오른쪽 스틱 가로축은 방향 회전, 트리거는 높이 조절입니다. 실제 조종기 연결 시험은 아직 하지 않았습니다.

## 빌드

Windows에서는 Visual Studio 개발자 PowerShell에서 실행합니다.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## 실행

시뮬레이터는 실행 파일이 있는 폴더에서 시작합니다. 기본 UDP 포트는 27015입니다.

```powershell
Push-Location .\build
.\aerosimlab_simulator.exe
Pop-Location
```

자동 비행 시연을 보려면 `--demo`를 추가합니다. 다른 포트를 쓰려면 `--udp-port 번호`를 지정합니다.

별도 PowerShell 창에서 지상통제 명령을 보냅니다.

```powershell
.\build\aerosimlab_ground_control.exe --command takeoff
# 드론이 뜰 때까지 1.5초 정도 기다립니다.
.\build\aerosimlab_ground_control.exe --command waypoint 2 0 1.5
.\build\aerosimlab_ground_control.exe --camera fault
.\build\aerosimlab_ground_control.exe --camera recover
.\build\aerosimlab_ground_control.exe --command land
```

다른 시뮬레이터 주소나 포트는 `--host IPv4`와 `--port 번호`로 지정합니다. 지상통제 프로그램은 명령 결과, 적용 Tick, 드론 상태와 카메라 상태를 출력합니다.

화면 없이 정상 비행이나 장애 시나리오를 실행할 수 있습니다.

```powershell
.\build\aerosimlab_headless.exe --scenario scenarios/normal-flight.json
.\build\aerosimlab_headless.exe --scenario scenarios/fault-injection.json
```

## 성능 확인

아래 명령은 숨김 OpenGL 창에서 5초 동안 모의와 그리기를 측정하고 JSON 결과를 출력합니다.

```powershell
Push-Location .\build
.\aerosimlab_simulator.exe --benchmark
Pop-Location
```

현재 세 번 측정한 결과는 [측정 기록](benchmarks/results/aerosimlab-mvp-release.json)에 있습니다. 숨김 창의 FPS는 실제 화면 출력 속도를 뜻하지 않으며, 이전 GraspLink 수치와 비교할 수도 없습니다.

## 관련 문서

[구조 안내](docs/ARCHITECTURE.md) · [요구사항과 상태](docs/REQUIREMENTS.md) · [검증과 측정](docs/VERIFICATION.md)
