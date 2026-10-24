# 성능 측정 결과 찾아보기

각 시험의 목적과 해석 범위는 [성능 검증과 측정 결과](../../docs/public/성능검증.md)에 정리했습니다. 아래 표에서 실행별 JSON 원자료를 찾을 수 있습니다.

## 어떤 시험의 자료인가요?

2026년 10월 24일 Release 전체 측정은 작업 폴더가 깨끗하지 않은 상태로 표시된 `26c7fce1-dirty`에서 실행했습니다. 숨김 창의 5초 화면 기록과 운동학·강체 확장성, 통신 장애 후 임무 재할당, 배터리 부족과 충돌 회피를 포함한 임무를 담았습니다. 네 대 협동 임무는 모의 방식별로 세 번씩 실행했고 모두 과제를 마쳤습니다. 강체 모의 배터리 시험은 매회 회피 동작을 세 번 기록했고 최소 중심 간 거리는 3.60m였습니다. 세부 입력과 반복 결과는 [최신 전체 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-26c7fce1.json)에서 확인할 수 있습니다. 앞서 `1c0c60dd-dirty`, `483b3c49-dirty`, 깨끗한 `06c6c7b1`에서 실행한 기록은 이전 자료로 보관합니다. 다른 리비전이나 조건의 결과는 현재 기록과 직접 비교하지 않습니다. 시험 목적과 해석 범위는 [성능 검증 문서](../../docs/public/성능검증.md)에서 확인할 수 있습니다.

| 알고 싶은 것 | 시험 목적 | 결과 자료 |
| --- | --- | --- |
| 현재 Release 리비전에서 전체 성능·협동 임무 결과는 어떤가? | 화면 처리 5초 기록 뒤 여러 드론 수, 임무 재할당, 통신 장애·배터리·충돌 회피 시나리오를 실행 | [2026년 10월 24일 전체 측정, `26c7fce1-dirty`](archive/aerosimlab-full-benchmark-release-2026-10-24-26c7fce1.json) |
| 실제 창에서 앱 프레임이 얼마나 걸리고 긴 지연이 있는가? | 장면·EO 카메라·HUD를 켜고 자동 시연을 실행해 프레임 처리 시간을 5초씩 3회 측정 | [2026년 10월 24일 화면 측정 3회](archive/aerosimlab-visible-benchmark-release-2026-10-24-edd2a04d.json), [이전 반복 측정](aerosimlab-visible-warmup-repeats-release-2026-10-23-aac03875.json) |
| 드론 수가 늘면 비행·충돌 계산이 얼마나 무거워지는가? | 숨김 창으로 운동학·AirSim 강체·밀집 배치의 Tick 비용을 비교 | [최신 전체 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-26c7fce1.json), [이전 전체 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-1c0c60dd.json), [정렬 변경 전후 측정](archive/aerosimlab-full-benchmark-release-2026-10-23-69ecab93-after-sort-reuse.json) |
| 편대가 커져도 각 드론의 사전 할당 임무가 끝나는가? | 드론마다 과제 하나를 미리 지정해 비행·목표 도달·완료 판정 비용과 모의 완료 시간을 측정. 조정기의 과제 선택·재할당은 포함하지 않음 | [최신 전체 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-26c7fce1.json) |
| 통신이 끊긴 4대 협동 임무가 끝나고 안전거리를 지키는가? | 운동학 320 Tick·강체 1,800 Tick을 각각 준비 1회와 측정 3회 실행. PC 실행 시간과 모의 임무 시간을 각각 기록 | [최신 전체 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-26c7fce1.json) |
| 배터리 부족과 통신 단절이 함께 있을 때 임무가 어떻게 끝나는가? | 네 대 임무에서 운동학 720 Tick·강체 1,800 Tick을 각각 준비 1회와 측정 3회 실행. 완료 과제·귀환·충돌 회피·에너지 사용량·계산 시간을 기록 | [최신 전체 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-26c7fce1.json), [이전 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-483b3c49-dirty.json) |
| 재할당이 포함된 시나리오 실행 시간은 드론 수에 따라 어떻게 달라지는가? | 운동학 시나리오에 4·16·64·256대와 25% 동시 장애를 적용해 전체 실행 시간을 측정. 각 규모별로 준비 1회와 측정 3회 실행 | [최신 재할당 측정 원자료](archive/aerosimlab-full-benchmark-release-2026-10-24-26c7fce1.json), [이전 측정](archive/aerosimlab-full-benchmark-release-2026-10-24-1c0c60dd.json) |
| 충돌 후보를 찾는 방식을 바꾸면 계산이 달라지는가? | 서로 다른 배치에서 변경 전후의 충돌 계산 비용을 비교 | [충돌 후보 쌍 측정](aerosimlab-drone-collision-pair-sweep-release-2026-10-23-8623be5c.json), [정렬 재사용 변경 전후 자료와 조건](../../docs/public/성능검증.md) |
| EO 카메라 픽셀을 가져오는 방식이 긴 프레임과 관련 있는가? | GPU 픽셀 읽기 경로의 호출 방식별 지연과 프레임 지연의 연관성을 조사 | [비동기 픽셀 읽기 자료](aerosimlab-visible-pbo-persistent-map-2026-10-23.json) |
| 화면을 그릴 때 CPU와 GPU 중 어디서 시간이 걸리는가? | 화면 처리 구간을 나눠 다음 조사 대상을 찾는 진단 | [화면 처리 진단 자료](aerosimlab-visible-imgui-gpu-timing-2026-10-23.json) |
| 임무 결과 검색 방식이 실행 시간에 영향을 주는가? | 협동 임무에서 예상 결과를 찾는 변경 전후를 비교 | [시나리오 결과 검색 비교](aerosimlab-scenario-result-index-2026-10-23.json) |

## 과거 원자료

이전 기준선, 중간 진단 실행과 같은 시험의 추가 반복은 [보관 자료](archive/)에서 볼 수 있습니다.

결과를 비교할 때는 코드 리비전, 빌드 종류, 입력, 장면과 측정 방법을 함께 확인하세요. `dirty`는 실행 당시 Git 작업 폴더에 커밋하지 않은 변경이 있었다는 표시입니다. 조건이나 코드 리비전이 다르면 수치만으로 성능 향상을 판단하지 마세요.
