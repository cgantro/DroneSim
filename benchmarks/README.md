# 성능 기록

`archive/`에는 이전 GraspLink의 로봇·그리퍼 측정 원본을 보관합니다. 새 드론 시뮬레이터의 값과 실행 조건이 달라 서로 비교할 수 없습니다.

현재 AeroSimLab 측정은 `results/aerosimlab-mvp-release.json`에 있습니다. 화면 프로그램에서 `--benchmark`를 실행하면 5초 동안 Tick 지연 분포, 숨김 OpenGL 그리기 속도, CPU 시간과 메모리를 출력합니다. 검증 방법과 결과 해석은 [검증 안내](../docs/VERIFICATION.md)를 보세요.
