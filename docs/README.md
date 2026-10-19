# GraspLink 문서 안내

처음 읽을 문서는 [아키텍처](ARCHITECTURE.md)입니다. 요구사항 상태는 [요구사항](REQUIREMENTS.md), 실측값과 측정 조건은 [성능](PERFORMANCE.md)을 참고하고, 특정 기능은 아래 표에서 찾을 수 있습니다.

## 어디서 시작할까?

| 궁금한 내용 | 먼저 볼 문서 | 더 자세한 내용 |
| --- | --- | --- |
| 모듈이 어떻게 나뉘고 연결되는가 | [아키텍처](ARCHITECTURE.md) | [Physics와 Flecs 연결](PHYSICS_ECS_INTEGRATION.md) |
| Controller 명령을 보내고 결과를 읽는 법 | [Controller 인터페이스](CONTROLLER_INTERFACE.md) | [IK와 MoveL 따라 하기](IK_MOVEL_TUTORIAL.md) |
| 로봇이 목표까지 움직이는 방식 | [로봇 이동과 파지](ROBOT_MOTION_AND_GRASP.md) | [요구사항과 구현 범위](CONTROL_SIMULATION_GOALS.md) |
| 그리퍼의 상태와 접촉 동작 | [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md) | [HCR-12A와 2F-85 사양](HCR12A_2F85_simulation_specs.md) |
| GLB 모델 좌표를 맞춘 기준 | [GLB 정규화](HCR12A_GLB_NORMALIZATION.md) | [모델 데이터 출처](MODEL_DATA_PROVENANCE.md) |
| 로그와 성능 측정 결과를 읽는 법 | [진단 도구](DIAGNOSTICS.md) | [픽앤플레이스 반복 검증](PICK_PLACE_STRESS.md) |
| 자가 충돌 검사 범위 | [자가 충돌 검사](SELF_COLLISION.md) | [Physics와 Flecs 연결](PHYSICS_ECS_INTEGRATION.md) |

## 문서를 읽을 때 참고할 점

기술 문서는 현재 코드에서 확인할 수 있는 동작과 한계를 설명한다. 모델 수치의 출처가 제조사 자료인지, CAD에서 얻은 값인지, 공개 기구학 모델이나 현재 GLB에 맞춘 값인지는 [모델 데이터 출처](MODEL_DATA_PROVENANCE.md)에 정리했다.

문서에 테스트나 성능 측정이 언급돼도 최근에 실행했다는 뜻은 아니다. 결과를 직접 확인하려면 저장소의 테스트나 성능 측정을 실행해야 한다. 항상 통과하도록 만든 시험용 충돌 검사 결과는 실제 환경의 충돌 성능을 보여 주지 않는다.

## 자주 나오는 용어

| 용어 | 뜻 |
| --- | --- |
| Base frame | 로봇 베이스를 기준으로 삼는 좌표계 |
| Local / World | 부모 기준 좌표와 장면 전체 기준 좌표 |
| Kinematic / Dynamic | 목표 자세를 코드가 정하는 물체와 물리 결과에 따라 움직이는 물체 |
| MoveJ / MoveL | 관절 공간 이동과 TCP가 직선을 따라가는 이동 |
| TCP | 작업 위치와 방향을 지정하는 공구 기준점 |
| Collision proxy | 충돌 검사에 쓰는 단순화 형상. 화면에 보이는 메시와 따로 관리한다. |
