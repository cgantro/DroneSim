# GraspLink

Hanwha HCR-12A와 Robotiq 2F-85를 시뮬레이션하는 C++17 프로젝트입니다. 정·역기구학, 충돌 검사, 픽앤플레이스 동작을 구현했으며 Windows와 웹에서 실행할 수 있습니다. 실제 로봇을 제어하지 않습니다.

## 시연

[![GraspLink 시연](assets/시연.gif)](assets/시연.webm)

## 빌드

### Windows

CMake, Ninja, MSVC가 설치된 Visual Studio Developer PowerShell에서 실행합니다.

```powershell
cmake --preset ninja
cmake --build --preset ninja-debug
cd build-ninja-debug
.\grasplink_simulator.exe
```

### 웹

Emscripten SDK를 활성화한 뒤 빌드합니다.

```powershell
emcmake cmake --preset emscripten-web
cmake --build --preset emscripten-web
```

결과물은 `build-emscripten-web/index.html`입니다. 멀티스레드 빌드는 `file://`에서 실행할 수 없습니다. [배포 사이트에서 실행](https://cgantro.github.io/Portfolio/minibcg/index.html)

GitHub Pages 배포 주소: [GraspLink 웹 시뮬레이터](https://cgantro.github.io/GraspLink/)

단일 스레드 빌드는 `emscripten-single-thread` preset으로 설정하고 빌드합니다.

```powershell
emcmake cmake --preset emscripten-single-thread
cmake --build --preset emscripten-single-thread
```

## 문서

[처음 읽기](docs/ARCHITECTURE.md) · [IK와 MoveL 따라 하기](docs/IK_MOVEL_TUTORIAL.md) · [구현 범위](docs/REQUIREMENTS.md) · [성능 기록](docs/PERFORMANCE.md) · [문서 안내](docs/README.md)
