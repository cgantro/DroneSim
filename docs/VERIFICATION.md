# 검증 결과

## 현재 확인한 항목

Windows Release 구성에서 C++ 빌드와 CTest 12개가 통과했습니다. 시험은 드론 상태 전이, 잘못된 명령, 속도 제한, 경유점, 일시정지·초기화, 같은 입력의 재현, JSON 시나리오 판정, 화면 시작을 확인합니다.

예제 비행은 300 Tick(5초) 뒤 원점에서 3m 이동하고 착륙합니다. 실행 결과는 `passed: true`였습니다.

## 다시 확인하는 방법

Visual Studio 개발자 PowerShell에서 실행합니다.

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
.\build\aerosimlab_headless.exe --scenario scenarios/normal-flight.json
```

화면 시작 시험은 창을 표시하지 않고 몇 프레임을 그린 뒤 종료합니다. 실제 창에서 카메라 영상을 확인하려면 실행 안내의 화면 실행 방법을 따릅니다.

## 아직 검증하지 않은 항목

- 다른 컴퓨터와 Debug 빌드
- 장시간 실행과 반복 초기화·종료
- 카메라 프레임 저장, 센서 고장·복구
- UDP 메시지 정상·오류·지연·손실·중복 처리
- Tick p50/p95/p99, CPU·메모리, 화면 FPS
- 단순 도형을 실제 드론·환경 모델로 교체한 뒤 좌표가 맞는지 여부

현재 성능 개선을 주장할 측정 자료는 없습니다. 새 기능을 넣은 뒤 같은 컴퓨터와 입력으로 반복 측정해 결과를 남깁니다.
