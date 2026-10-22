# X500 모델 자료 출처

화면에 그리는 X500 형상은 [PX4 Gazebo 모델 저장소](https://github.com/PX4/PX4-gazebo-models/tree/main/models/x500_base)의 공개 메시를 바탕으로 만든 GLB입니다. PX4 메시의 이용 조건은 BSD 3-Clause입니다.

X500 V2 프레임과 2216 모터의 STEP CAD는 [Holybro 공식 다운로드 페이지](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2/download)에서 받았습니다. CAD 파일별 재배포 조건은 페이지에 표시되어 있지 않으므로, 이 자료를 저장소 밖으로 배포할 때는 제조사 조건을 먼저 확인해야 합니다.

시뮬레이터는 PX4 화면 모델을 GLB로 읽습니다. STEP CAD는 원본 설계 참고 자료로 보관하며 실행 중에는 읽지 않습니다.

PX4에서 받은 화면 형상에는 재질과 이미지 좌표가 들어 있지 않습니다. 시뮬레이터는 X500 메시의 표면 방향에 맞춰 탄소 섬유 무늬를 세 방향으로 투영합니다. 이 무늬는 화면용 근사 재질이며, 제조사의 실제 표면 무늬를 재현한 자료는 아닙니다.
