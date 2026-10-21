# 성능 기록

`archive/`에는 이전 GraspLink의 로봇·그리퍼 측정 원본을 보관합니다. 새 드론 시뮬레이터의 값과 실행 조건이 달라 서로 비교할 수 없습니다.

현재 AeroSimLab 측정은 `results/aerosimlab-mvp-release.json`에 있습니다. Visual Studio에서 성능 측정 설정으로 실행하면 5초 동안 Tick 지연, 새 EO 프레임의 OpenCV 처리 시간, 밝기·대비·선명도 평균, 숨김 OpenGL 그리기 속도, CPU 시간과 메모리를 JSON으로 출력합니다. 결과는 Visual Studio의 디버그 출력 창에서 확인할 수 있습니다.

선명도는 회색 영상의 라플라시안 응답 분산으로 추정합니다. 장면의 모서리와 노이즈가 결과에 영향을 주므로 절대 품질 점수로 해석하지 않습니다. 검증 방법과 측정 조건은 [검증 안내](../docs/VERIFICATION.md)를 보세요.
