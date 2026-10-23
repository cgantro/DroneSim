# 성능 측정 결과 찾아보기

먼저 [기술 검토](../../docs/public/기술검토.md)에서 주요 결과와 해석을 읽어 주세요. 이곳의 JSON은 각 실행에서 나온 원자료입니다.

## 대표 결과

| 무엇을 확인하나요? | 결과 |
| --- | --- |
| 최신 전체 Release 기준선과 화면 없는 확장 시험 | [전체 측정 결과](aerosimlab-full-benchmark-release-2026-10-23-4cb66d99.json) |
| 보이는 창 프레임 시간 3회 측정 | [화면 반복 측정](aerosimlab-visible-warmup-repeats-release-2026-10-23-aac03875.json) |
| 시나리오 예상 결과의 이름 조회 변경 전후 | [협동 임무 비교](aerosimlab-scenario-result-index-2026-10-23.json) |
| 여러 드론이 가까이 있을 때의 충돌 후보 검사 | [충돌 쌍 측정](aerosimlab-drone-collision-pair-sweep-release-2026-10-23-8623be5c.json) |
| EO 영상 픽셀을 GPU에서 읽어 오는 방식의 변경 | [비동기 픽셀 읽기 측정](aerosimlab-visible-pbo-persistent-map-2026-10-23.json) |
| 화면 그리기 구간의 CPU·GPU 측정 | [화면 처리 진단](aerosimlab-visible-imgui-gpu-timing-2026-10-23.json) |

## 과거 원자료

이전 기준선, 중간 진단 실행, 같은 시험의 추가 반복은 [보관 자료](archive/)에 옮겼습니다. 원본 JSON은 삭제하거나 내용을 고치지 않았습니다. 기존 기술 문서의 링크도 새 위치로 연결됩니다.

결과를 비교할 때는 파일에 적힌 코드 리비전, 빌드 종류, 입력, 장면과 측정 방법을 함께 확인해 주세요. 파일 이름에 `dirty`가 있으면 빌드 당시 작업 트리에 커밋되지 않은 변경이 있었다는 뜻입니다. 조건이나 코드 리비전이 다르면 숫자만으로 성능이 나아졌다고 판단할 수 없습니다.
