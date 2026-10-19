# 성능 측정

이 문서는 실제 실행 결과와 측정 상태를 구분한다. 측정값은 실행한 바이너리와 조건에만 적용된다. Release 결과가 없으면 성능 개선이나 250 Hz 달성을 주장하지 않는다.

## 측정 도구

네이티브 Release 빌드에서 Viewer의 일반 시뮬레이션 tick을 계측하려면 다음 명령을 사용한다.

```powershell
cmake --preset ninja-release
cmake --build --preset ninja-release
build-ninja-release\grasplink_simulator.exe --profile-ticks 10000
```

실행은 `FixedTickProfile` JSON 객체를 출력한다. tick 작업 시간 p50/p95/p99/max, 4 ms 초과 횟수, tick 시작 간격, 실제·시뮬레이션 경과 시간, 프레임 및 누적기 표본을 기록한다. 이 프로파일은 초기 장면의 Viewer 경로를 측정하며 계획 중, 로봇 이동 중, 자동 Pick & Place 중 별도 구간을 선택하지 않는다. tick 작업 예산은 실제 4 ms 주기 스케줄링이나 hard real-time 보장을 뜻하지 않는다.

Planner 및 반복 임무 측정은 [벤치마크 실행 안내](PICK_PLACE_STRESS.md)와 [Pick & Place 기록](PICK_PLACE_STRESS_RESULT_2026-10-19.md)을 참고한다. dummy validity callback을 사용하는 알고리즘 측정은 실제 Jolt 충돌 성능으로 해석하지 않는다.

## 현재 Release 결과

2026-10-20에 MSVC 19.50.35717 Release 빌드에서 초기 정지 장면을 세 번 측정했다. 각 실행은 10,000 tick이며 숨긴 Viewer 창에서 렌더 프레임 경로를 유지했다. Tracy, benchmark target, 테스트 target은 껐다. CPU는 Intel Core i5-13600KF이며 메모리는 32 GiB다. 실행 파일 SHA와 작업 트리 C++ 소스 목록 SHA는 [원시 결과](../benchmarks/results/fixed_tick_release_2026-10-20.json)에 기록했고, 각 실행의 표준 출력도 같은 폴더에 보관했다.

세 실행에서 tick 계산 p50은 0.7393–0.7435 ms, p95는 0.7970–0.8304 ms, p99는 0.8825–0.9275 ms였다. 관측한 30,000 tick 중 계산 시간이 4 ms를 넘은 tick은 없었다. Simulation speed는 1.0021–1.0022였다.

실제 tick 시작 간격 p95는 4.6750–4.7719 ms였고 최댓값은 10.7408–14.3860 ms였다. 따라서 이 측정은 tick 계산이 4 ms 예산 안에 든 것을 보여 주지만, 실제 250 Hz tick 주기를 입증하지 않는다. 로봇 이동, 경로 계획, Pick & Place, 부하가 있는 장면은 이번 실행에 포함되지 않았다. Tracy는 껐지만 `steady_clock` 표본과 저장 비용이 측정에 주는 영향은 별도로 비교하지 않았다.

최적화 전 동일 조건 기준선이 없어 성능 전후 개선율은 계산하지 않았다.

## 사용 가능한 과거 결과

2026-10-19 기록에는 seed 1로 50회 실행한 Pick & Place 결과가 있다. 50회 성공, 실패·timeout 0회, 총 210.76초였다. 기록된 바이너리는 2026-10-15에 만들어진 Debug 실행 파일이며 현재 소스에서 새로 만든 Release 성능 측정이 아니다. 따라서 이 값은 해당 Debug 실행의 재현 기록으로만 사용한다.

현재 체크아웃 Release 빌드의 고정 tick 성능과 최적화 전후 비교는 별도 Release 측정을 실행한 뒤에만 기입한다.
