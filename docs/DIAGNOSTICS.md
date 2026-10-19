# 진단 도구

이 문서는 두 가지 도구를 다룬다. `Logger`는 로그와 임무 결과 같은 기록을 파일에 남기고, Tracy는 프레임과 코드 구간에 걸린 시간을 잰다. 둘은 목적이 다르므로 Logger 기록을 실행 시간 측정으로 읽으면 안 된다.

Logger는 기록을 크기가 제한된 큐에 복사한 뒤, 전용 작업자 스레드 하나로 `logs/grasplink.jsonl`에 JSON Lines 형식으로 쓴다. 큐가 가득 차면 제어 주기가 디스크 쓰기를 기다리지 않도록 새 Debug/Info 로그나 metric을 버릴 수 있다. Error 로그는 큐 안의 낮은 우선순위 기록을 대신할 수 있다. `Flush()`는 접수된 기록을 모두 쓸 때까지 기다리고, `Shutdown()`은 새 기록을 받지 않은 뒤 큐를 비우고 작업자 스레드를 종료한다. Logger를 참조하는 시스템이 모두 끝날 때까지 Logger를 유지해야 한다.

구현은 `modules/diagnostics`에, benchmark 정의는 `benchmarks/RoboticsBenchmarks.cpp`에 있다. 아래 설명은 각 counter가 무엇을 뜻하는지 안내한다. 측정 결과는 실행 시점과 구성에 따라 달라지며, 이 문서가 현재 성능을 보증하지는 않는다.

## 목차

- [RuntimeLinearPathStress](#runtimelinearpathstress)
- [InteractiveLinearPathStress](#interactivelinearpathstress)
- [PosePlanEndpointStress](#poseplanendpointstress)
- [오프라인 계획과 대화형 계획 비교](#오프라인-계획과-대화형-계획-비교)
- [선형 계획 작업량](#선형-계획-작업량)
- [JoltMoveJDirectPath](#joltmovejdirectpath)
- [그 밖의 카운터와 callback](#그-밖의-카운터와-callback)
- [Viewer Tracy 구간](#viewer-tracy-구간)

실행 중 성능은 Tracy Profiler에서 확인한다. 시뮬레이터는 프레임 전체와 `Frame`, `FixedTick`, `RobotUpdate`, `CollisionCheck`, `PhysicsStep`, `Render` 구간을 기록한다. `TRACY_ON_DEMAND`가 켜져 있으면 Profiler가 연결된 뒤 측정을 시작한다. 시뮬레이터와 Profiler를 실행한 다음 서로 연결하면 각 구간에 걸린 시간을 볼 수 있다. Tracy CMake client는 `v0.14.1`을 사용한다.

## `RuntimeLinearPathStress` 경로 스트레스

다음 명령으로 고정 seed를 사용하는 경로 스트레스 벤치마크를 실행한다. `/64`와 `/256`은 한 번의 반복에서 생성하는 목표 수다.

```powershell
build-ninja-release/grasplink_robotics_benchmarks.exe --benchmark_filter=RuntimeLinearPathStress
```

이 벤치마크는 합법적인 관절값에서 FK로 TCP 끝점을 만든다. `fk_generated_reachable_endpoints`는 끝점이 실제 관절값에서 만들어졌다는 뜻이고, 그 끝점까지의 MoveL 경로가 가능하다는 뜻은 아니다.

`endpoint_ik_successes_from_start_seed`는 현재 시작 자세 하나에서만 IK가 수렴한 수다. `joint_limit_valid_movej_segments`는 두 끝점 사이의 관절 직선이 관절 한계 안에 있는지 나타내며, 실제 환경 충돌 검사는 포함하지 않는다.

`planner_rejections`는 전체 TCP 직선 경로 계획 실패 수다. `planner_ik_limit_stalls`는 IK 반복 중 관절 한계의 영향을 받은 국소 정체 수이며, 전역적으로 해가 없음을 뜻하지 않는다.

`planner_verified_joint_limit_violations`는 검증한 경로 관절값에서 실제 한계 초과를 확인한 수다. `dummy_state_validity_callback_calls`는 항상 유효를 반환하는 시험용 callback 호출 수이므로 Jolt 환경 충돌 성능으로 해석하면 안 된다.

완료된 경로의 최대 TCP 오차, MoveJ 실행 완료율, endpoint IK 성공률도 별도 counter로 출력된다.

## `InteractiveLinearPathStress` 대화형 경로 스트레스

`InteractiveLinearPathStress/64`는 같은 고정 seed의 64개 목표를 `LinearPathPlanningJob`에 넣고 Viewer와 같은 방식으로 계획한다. 각 가상 프레임에서 `Advance(1)`을 반복 호출하다가 1.5 ms deadline에 도달하거나 계획이 끝나면 다음 프레임으로 넘어간다.

프레임당 계획량은 작업 단위 하나로 고정하지 않고 렌더 loop처럼 시간으로 제한한다. 이 벤치마크는 실행 중인 시뮬레이터나 Jolt를 호출하지 않는다.

상태 검사 callback은 항상 유효를 반환하므로 완료율을 실제 환경 충돌을 고려한 임무 성공률로 볼 수 없다. `interactive_completed`, `interactive_rejected`, `interactive_timeouts`, `average_work_frames`, `worst_work_frames`는 계획 결과와 프레임 수를 보여 준다.

`frame_slice_p50_us`, `frame_slice_p95_us`, `frame_slice_p99_us`, `frame_slice_max_us`는 각 가상 프레임에서 실제로 소요된 계획 시간이다. 한 번의 작업 단위가 deadline을 넘길 수 있으므로 `frame_slice_max_us`가 예산보다 클 수 있다.

`ik_solves_per_request`, `ik_iterations_per_request`, `tcp_refinements_per_request`, `tcp_straightness_checks_per_request`, `joint_path_validity_checks_per_request`, `validity_state_checks_per_request`는 요청 하나당 계획 작업량이다. `max_tcp_position_error_mm`와 `max_tcp_orientation_error_mrad`는 완료 경로를 FK로 확인한 최대 오차다.

벤치마크 이름과 `collision_benchmark_mode_dummy_always_valid`는 항상 유효 callback을 쓴다는 점을 표시한다.

## `PosePlanEndpointStress` 목표 자세 계획 스트레스

`PosePlanEndpointStress/64`와 `/256`은 위와 같은 고정 seed로 생성한 FK 도달 가능 TCP 목표에 대해 `SimRobotController::BeginPosePlanning()`을 호출한다. 각 Controller는 영점 시작 관절각에서 계획을 시작하고, Viewer와 같은 1.5 ms 프레임 deadline 동안 `AdvanceMotionPlanning(1)`을 반복한다.

이는 PTP 끝점 IK와 관절 경로 계획을 확인하며, TCP가 직선을 따라가는지 평가하는 MoveL 검사가 아니다. `pose_plan_accepted`는 Controller가 목표 IK와 MoveJ 경로 계획을 마치고 관절 이동을 시작한 건수다.

로봇 관절이 실제 목표에 도착했는지는 측정하지 않는다. `pose_plan_rejected`, `pose_plan_timeouts`, `pose_plan_unreachable`, `pose_plan_joint_limit_rejected`, `pose_plan_ik_did_not_converge`는 끝점 계획 실패를 이유별로 나눈다.

이 벤치마크도 항상 유효를 반환하는 dummy 상태 검사기를 사용하므로 환경 충돌 성공률을 뜻하지 않는다. `frame_slice_*_us`와 `pose_plan_*_work_frames`로 계획이 몇 프레임에 걸렸는지와 한 프레임 계산 시간을 함께 확인할 수 있다.

```powershell
build-ninja-release/grasplink_robotics_benchmarks.exe --benchmark_filter=PosePlanEndpointStress
```

2026-10-09 Release 측정에서는 PTP 끝점 계획이 64개와 256개 모두를 수락했고 거절과 timeout은 0건이었다. IK는 현재 관절 자세를 첫 시작값으로 사용하고, 실패할 때 결정론적 Halton 표본에서 만든 대체 시작값을 최대 63개 더 시도한다.

따라서 한 목표의 IK 시작값은 최대 64개이며, 무한 재시도가 아니다. 측정 표본의 평균 계획 프레임은 각각 1.047과 1.012였고 최악은 모두 4프레임이었다.

`frame_slice_*_us`는 `steady_clock`으로 계획 계산 구간을 잰 경과 시간이다. 64개 측정에서 p50/p95/p99/max는 58.6/470.4/1509.8/1509.8 μs였고, 256개 측정에서는 49.9/215.6/1500.7/1507.9 μs였다.

p99와 최댓값이 1.5 ms보다 큰 것은 단일 계획 작업이 deadline 뒤에 끝날 수 있기 때문이다. 이 결과는 정해진 64개와 256개 FK 생성 표본에서 PTP 계획이 완료됐음을 보여 줄 뿐, 모든 가능한 관절 자세에서 성공을 보장하지는 않는다.

## 오프라인 계획과 대화형 계획 비교

Interactive 결과를 Offline 스트레스의 64개 목표와 비교하려면 두 벤치마크를 함께 실행한다. Offline 경로는 기본 seed 재시도 정책으로 계획과 Controller 실행을 진행하고, Interactive 경로는 화면용 정책으로 계획만 진행하며 각 프레임에서 1.5 ms까지 작업한다. 따라서 두 벤치마크의 전체 시간은 같은 일을 재는 값이 아니다. `moveL_completion_rate`와 `interactive_success_rate`는 계획 완료율로 비교하되, 두 수치 모두 dummy checker를 사용하므로 환경 충돌을 통과하는 비율은 아니다.

```powershell
build-ninja-release/grasplink_robotics_benchmarks.exe --benchmark_filter="(RuntimeLinearPathStress/64|InteractiveLinearPathStress/64)" --benchmark_min_time=0.001s
```

2026-10-09 Release 비교 실행에서는 Offline과 Interactive가 각각 51/64 (79.6875%) 경로를 완료했다. Interactive 계획은 평균 7.14 프레임, 최악 35 프레임이었고 frame slice 시간은 p50 1.501 ms, p95 1.541 ms, p99 1.562 ms, 최대 1.632 ms였다. 한 작업 호출이 남은 프레임 예산보다 길면 최대값은 1.5 ms를 넘을 수 있다. 완료 경로의 최대 TCP 오차는 위치 0.321436 mm, 방향 0.997137 mrad였다. 이 값은 측정 환경과 코드 변경에 따라 달라질 수 있으므로 성능 비교 때마다 다시 실행한다.

## 선형 계획 작업량

`LinearPathPlanner`는 기본 16 seed 재시도 정책으로 한 번에 계산하고 `IncrementalLinearPathPlanner`는 화면 응답성을 위한 12 seed 정책과 프레임별 1.5 ms 작업 예산으로 계산한다. 두 정책은 충돌 표본 간격, TCP 오차 허용치, 후보 수와 세분화 기준이 같고 seed 재시도 상한만 다르다.

전체 경로 계획 시간은 Google Benchmark가 출력하는 `Time`과 `CPU`를 사용한다. `work_frames_per_path` 및 `estimated_latency_ms_at_60hz_*`는 증분 계획이 몇 프레임에 걸쳐 끝났는지와 60 Hz에서의 예상 대기 시간을 보여 준다.

프레임별 `slice_p50_us`, `slice_p95_us`, `slice_p99_us`, `slice_max_us`는 각 프레임에서 계획 계산에 걸린 시간을 별도로 측정한다. 한 작업 단위 자체가 예산을 넘으면 실제 slice 시간도 예산을 넘을 수 있다.

## `JoltMoveJDirectPath` 직접 경로 검사

`JoltMoveJDirectPath`는 관절 공간 직선이 Environment Box와 겹치도록 만들고 `JointPathPlanningJob`이 이를 거부할 때까지 걸리는 시간을 잰다. 상태 검사기는 실제 `PhysicsWorld::OverlapsEnvironmentAt()`을 호출하므로 Jolt 충돌 쿼리 수와 거부 지연을 확인할 수 있다.

이 벤치마크는 전체 로봇 링크의 collider 조립이나 Viewer의 Scene 통합을 측정하지 않고, HCR-12A 관절 공간에 연결한 단일 Box proxy와 고정 장애물만 사용한다. 따라서 이 결과를 전체 로봇 환경 충돌 성능으로 일반화하지 않는다.

`jolt_direct_movej_rejection_rate`와 `jolt_environment_overlap_queries`가 직접 경로의 거부율과 충돌 확인 횟수를 보여 준다. 이 벤치마크는 우회 경로를 찾지 않는다.

기본 CTest는 `JoltMoveJDirectPath`를 포함해 Jolt 충돌 쿼리를 연결한 직접 경로 검증 벤치마크가 실행되는지 확인한다. 직접 실행하려면 다음과 같이 한다.

```powershell
build-ninja-release/grasplink_robotics_benchmarks.exe --benchmark_filter=JoltMoveJDirectPath
```

## 그 밖의 카운터와 callback

`ik_solves_per_path`, `ik_iterations_per_path`, `tcp_refinements_per_path`, `tcp_straightness_checks_per_path`와 `planner_state_checks_per_path`는 경로 하나를 만드는 데 사용한 계산량을 보여 준다. TCP 경로 오차는 각 관절 구간의 25%, 50%, 75% 지점에서 FK로 계산한 실제 자세와 요청 직선 경로의 차이를 측정한다.

`max_fk_tcp_position_error_mm`와 `max_fk_tcp_orientation_error_mrad`가 그 최대 오차다. Incremental benchmark에는 실제 Jolt 충돌 검사기가 연결되지 않으므로 `planner_state_checks_per_path`는 상태 검사 호출 수일 뿐 충돌 쿼리 수가 아니다.

스트레스 벤치마크의 Controller에는 매 상태 검증 호출마다 횟수를 올리는 callback을 등록한다. `dummy_state_validity_callback_calls`는 callback 총 호출 수이고 `dummy_state_validity_callback_calls_per_case`는 목표 하나당 평균 호출 수다. Callback은 모든 상태를 유효하다고 반환하므로 실제 환경 충돌이나 Jolt 검사는 하지 않는다. 이 수치는 상태 검증 경계의 호출량을 나타내며 충돌 검사 횟수나 소요 시간을 뜻하지 않는다.

## Viewer에서 기록하는 Tracy 구간

실제 Viewer에서 Tracy 캡처를 시작하면 충돌 검증 구간은 `CollisionCandidateFK`, `CollisionCheck`, `SelfCollisionCheck`, `JoltEnvironmentOverlap`, `JoltPairShapeOverlap`으로 나뉘어 표시된다. `CollisionCandidateFK`는 Scene을 수정하지 않고 FK에서 Jolt 입력 변환까지 만드는 시간이며, 두 Jolt 구간은 각각 환경 형상과 로봇 형상 쌍을 실제 검사한 시간이다. `SelfCollisionCheck`에는 허용 충돌 행렬과 각 쌍 반복 비용도 포함된다.

나머지 endpoint IK, MoveJ 실행, MoveL stress 벤치마크는 Jolt 환경 충돌을 포함하지 않는다. `JoltMoveJDirectPath`도 단순화한 한 개 proxy의 직접 경로 검사만 포함한다. 어느 benchmark의 결과도 전체 Pick-and-Place 임무 성공률로 해석하면 안 된다.
