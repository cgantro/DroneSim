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

같은 Release 실행 파일로 Pick & Place stress mode를 seed 1, 2, 3마다 50회씩 실행했다. 총 150회 모두 성공했고 실패·timeout은 없었다. 미션 경과시간의 nearest-rank p50은 1,478.2 ms, p95는 1,962.7 ms, p99는 2,560.9 ms, 최댓값은 2,592.3 ms였다. seed별 시도와 원시 출력은 [요약 JSON](../benchmarks/results/pick_place_release_2026-10-20.json)과 같은 폴더의 `pick_place_release_2026-10-20_seed*_50runs.txt`에 저장했다.

이는 세 개의 고정 seed가 만든 150개 시나리오 표본이다. 실행 도구는 난이도 구간, 파지·운반·배치 단계별 실패, TCP 및 최종 물체 오차, 충돌 복구 원인을 출력하지 않는다. 따라서 이 표본은 일반 성공 확률이나 모든 목표의 성공, 물리적 실시간 cycle time을 나타내지 않는다.

## 사용 가능한 과거 결과

동일한 Release 빌드의 단발 Google Benchmark 표본도 기록했다. `DLS_IK`는 solve당 4.354 µs, 동기 `LinearPathPlanner`는 path당 46.676 µs, `IncrementalLinearPathPlanner`는 path당 62.644 µs였다. Incremental planner의 frame slice p95는 46.5 µs, 최댓값은 178.8 µs였다. Jolt를 실제 호출하는 `JoltMoveJCollisionPath`는 경로당 136.052 µs, 평균 Jolt overlap query 473회였고, 해당 표본에서 896/896 경로가 완료됐다.

위 `JoltMoveJCollisionPath` 결과는 RRT-Connect 우회가 있던 이전 구현의 기록이다. 현재 MoveJ 검증기는 막힌 직접 경로를 거부하며, 새 `JoltMoveJDirectPath` 벤치마크 결과와 직접 비교할 수 없다.

알고리즘 stress는 충돌 callback이 항상 통과하는 dummy 설정임을 결과에 표시한다. `RuntimeLinearPathStress/64`는 요청 64건 중 51건을 완료하고 13건을 거부했다. `PosePlanEndpointStress/64`는 64건을 수락했지만 둘 다 실제 환경 충돌 검사 성능이 아니다. 단발 벤치마크 결과에는 반복 간 신뢰 구간이 없으며, MoveL과 PhysicsStep의 실제 Jolt 비용은 아직 계측하지 않았다. 모든 원시 결과, callback 표시와 빌드 hash는 [Release robotics benchmark 기록](../benchmarks/results/robotics_benchmarks_release_2026-10-20.json) 및 그 파일이 참조하는 결과 파일에서 확인할 수 있다.

이 algorithm/Jolt 단발 표본에도 동일 조건 최적화 전 기준선은 없으므로 개선율을 계산하지 않았다.

2026-10-19 기록에는 seed 1로 50회 실행한 Pick & Place 결과가 있다. 50회 성공, 실패·timeout 0회, 총 210.76초였다. 기록된 바이너리는 2026-10-15에 만들어진 Debug 실행 파일이며 현재 소스에서 새로 만든 Release 성능 측정이 아니다. 따라서 이 값은 해당 Debug 실행의 재현 기록으로만 사용한다.

현재 체크아웃 Release 빌드의 고정 tick 성능과 최적화 전후 비교는 별도 Release 측정을 실행한 뒤에만 기입한다.
