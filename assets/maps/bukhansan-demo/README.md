# 북한산 지형 시험 구역

이 자료는 서울 북쪽 북한산 일대의 높이 변화를 화면과 강체 충돌에 함께 쓰는 작은 시험 지형입니다.

## 자료 출처와 이용 조건

- 자료: NASA SRTM Global 1 Arc-Second, Version 3
- 원본 타일: N37E126
- 원본 간격: 위도·경도 1초 간격, 이 지역에서 남북 약 31m·동서 약 24m
- 높이 기준: EGM96, 미터
- 제품 정보: [NASA SRTM 자료](https://doi.org/10.5067/MEaSUREs/SRTM/SRTMGL1.003), [USGS 자료 설명](https://www.usgs.gov/centers/eros/science/usgs-eros-archive-digital-elevation-shuttle-radar-topography-mission-srtm-non)
- 내려받은 공개 타일: [ESA STEP 자료 미러](https://step.esa.int/auxdata/dem/SRTMGL1/)
- 이용 조건: [NASA Earthdata의 자료 이용 안내](https://www.earthdata.nasa.gov/engage/open-data-services-software/data-use-policy)에 따라 별도 제한이 표시되지 않은 NASA 자료를 재사용할 수 있습니다. 출처 표기를 남깁니다.

## 잘라낸 범위와 변환

- 중심: 북위 37.661111°, 동경 126.983889°
- 범위: 북위 37.670000°부터 37.652222°까지, 동경 126.975000°부터 126.992778°까지
- 격자: 65 × 65 높이값
- 기준 높이: 중심점의 504m
- 저장 높이: 각 높이에서 기준 높이를 뺀 상대 높이
- 중심점은 시뮬레이션 좌표 `(0, 0, 0)`에 둡니다. 동쪽은 `+X`, 북쪽은 `+Y`, 위쪽은 `+Z`입니다.
- 위도·경도는 중심점을 기준으로 평면에 근사해 동쪽·북쪽 미터 좌표로 바꿉니다. 이 근사는 약 2km 크기의 시험 구역에 사용합니다.

## 알아둘 점

SRTM은 땅 표면뿐 아니라 나무와 건물의 영향을 받을 수 있는 디지털 표면 모델입니다. 이 구역은 약 30m 간격이라 작은 구조물과 급경사를 정밀하게 나타내지 못합니다. 이 자료로 실제 지형의 정확도나 비행 안전성을 보증하지 않습니다.

출처 표기: NASA/JPL, NASA Shuttle Radar Topography Mission Global 1 Arc Second, Version 3.

## 건물 외곽선 자료

같은 범위의 OpenStreetMap 건물 외곽선을 받아 `buildings.geojson`에 넣었습니다. 2026년 10월 21일 자료에는 닫힌 외곽선 17개가 있습니다. 사찰 7개, 일반 건물 9개, 주차장 1개입니다.

원본에는 건물 높이와 층수가 없습니다. 자료의 `height_m`와 `building_levels`가 `null`인 까닭입니다. 확인되지 않은 값을 실제 높이처럼 채우지 않았습니다. 건물 외곽선은 아직 화면이나 충돌 계산에 연결하지 않았고, 화면에는 기존 임시 상자가 남아 있습니다.

OpenStreetMap 자료는 [Open Database License(ODbL)](https://www.openstreetmap.org/copyright)를 따릅니다. 출처는 `© OpenStreetMap contributors`입니다. 원본 조회 주소도 GeoJSON에 기록했습니다.
