# AeroSimLab

AeroSimLab은 가상 드론의 비행과 탑재 카메라 화면을 시험하는 C++ 시뮬레이터입니다. 현재는 60Hz 운동 모형, JSON 시나리오 실행, 간단한 3D 화면을 제공합니다. 별도 지상통제 프로그램과 UDP 연동은 아직 없습니다.

화면 왼쪽에는 드론과 주변 환경이 보이고, 오른쪽에는 기체 방향을 따라가는 RGB 카메라 화면이 보입니다. 드론과 건물은 단순한 상자 모양이며, 실제 공기 흐름이나 모터 힘은 계산하지 않습니다.
화면은 이착륙과 경유점 이동을 자동으로 반복합니다. 조종 입력은 아직 없습니다.

## 빌드

Windows에서는 Visual Studio 개발자 PowerShell에서 실행합니다.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## 실행

화면 없이 예제 비행을 시험합니다.

```powershell
.\build\aerosimlab_headless.exe --scenario scenarios/normal-flight.json
```

결과의 `passed`가 `true`면 명령 재생과 예상 상태 비교가 통과한 것입니다.

화면 프로그램은 실행 파일이 있는 폴더에서 시작합니다.

```powershell
Push-Location .\build
.\aerosimlab_simulator.exe
Pop-Location
```

## 시험

```powershell
ctest --test-dir build --output-on-failure
```

12개 시험에서 드론 상태 전이, 명령, 시나리오 반복 결과와 화면 시작을 확인합니다. UDP 통신, 센서 고장, 성능 분포는 아직 시험하지 않습니다.

## 문서

[구조 안내](docs/ARCHITECTURE.md) · [요구사항](docs/REQUIREMENTS.md) · [검증 결과](docs/VERIFICATION.md)
