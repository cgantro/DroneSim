"""NGII 1:1,000 건물 도형을 탄천 장면의 GeoJSON에 반영한다."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import struct
import tempfile
import unicodedata
import zipfile
from pathlib import Path
from typing import Any


BUILDING_LAYER = "N1A_A0010000"
BRIDGE_LAYER = "N1A_A0070000"
TREE_LAYER = "N1P_D0030000"
WGS84_A = 6_378_137.0
WGS84_F = 1.0 / 298.257222101
E2 = WGS84_F * (2.0 - WGS84_F)
EP2 = E2 / (1.0 - E2)
CENTRAL_MERIDIAN = math.radians(127.0)
LATITUDE_ORIGIN = math.radians(38.0)
FALSE_EASTING = 200_000.0
FALSE_NORTHING = 600_000.0


def _meridional_arc(latitude: float) -> float:
    return WGS84_A * (
        (1.0 - E2 / 4.0 - 3.0 * E2**2 / 64.0 - 5.0 * E2**3 / 256.0)
        * latitude
        - (3.0 * E2 / 8.0 + 3.0 * E2**2 / 32.0 + 45.0 * E2**3 / 1024.0)
        * math.sin(2.0 * latitude)
        + (15.0 * E2**2 / 256.0 + 45.0 * E2**3 / 1024.0)
        * math.sin(4.0 * latitude)
        - (35.0 * E2**3 / 3072.0) * math.sin(6.0 * latitude)
    )


def epsg5186_to_wgs84(easting: float, northing: float) -> tuple[float, float]:
    """EPSG:5186 평면 좌표를 GeoJSON 경도·위도로 바꾼다."""
    m = _meridional_arc(LATITUDE_ORIGIN) + northing - FALSE_NORTHING
    mu = m / (WGS84_A * (1.0 - E2 / 4.0 - 3.0 * E2**2 / 64.0 - 5.0 * E2**3 / 256.0))
    e1 = (1.0 - math.sqrt(1.0 - E2)) / (1.0 + math.sqrt(1.0 - E2))
    phi1 = (
        mu
        + (3.0 * e1 / 2.0 - 27.0 * e1**3 / 32.0) * math.sin(2.0 * mu)
        + (21.0 * e1**2 / 16.0 - 55.0 * e1**4 / 32.0) * math.sin(4.0 * mu)
        + 151.0 * e1**3 / 96.0 * math.sin(6.0 * mu)
        + 1097.0 * e1**4 / 512.0 * math.sin(8.0 * mu)
    )

    sin_phi = math.sin(phi1)
    cos_phi = math.cos(phi1)
    tan_phi = math.tan(phi1)
    n1 = WGS84_A / math.sqrt(1.0 - E2 * sin_phi**2)
    t1 = tan_phi**2
    c1 = EP2 * cos_phi**2
    r1 = WGS84_A * (1.0 - E2) / (1.0 - E2 * sin_phi**2) ** 1.5
    d = (easting - FALSE_EASTING) / n1

    latitude = phi1 - (n1 * tan_phi / r1) * (
        d**2 / 2.0
        - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1**2 - 9.0 * EP2) * d**4 / 24.0
        + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1**2 - 252.0 * EP2 - 3.0 * c1**2)
        * d**6
        / 720.0
    )
    longitude = CENTRAL_MERIDIAN + (
        d
        - (1.0 + 2.0 * t1 + c1) * d**3 / 6.0
        + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1**2 + 8.0 * EP2 + 24.0 * t1**2)
        * d**5
        / 120.0
    ) / cos_phi
    return math.degrees(longitude), math.degrees(latitude)


def _read_dbf_ids(data: bytes) -> list[str]:
    rows = _read_dbf_rows(data)
    if not rows or "UFID" not in rows[0]:
        raise ValueError("건물 속성표에 UFID 항목이 없습니다.")
    return [row.get("UFID", "") for row in rows]


def _positive_number(value: str) -> float:
    try:
        number = float(value.replace(",", ""))
        return number if math.isfinite(number) and number > 0.0 else 0.0
    except (AttributeError, ValueError):
        return 0.0


def _normalize_building_name(value: str) -> str:
    return "".join(
        character for character in unicodedata.normalize("NFKC", value).casefold()
        if character.isalnum()
    )


def _match_building_register(
    building: dict[str, str],
    parcel_shape_count: int,
    register_rows: list[dict[str, str]],
) -> tuple[dict[str, str] | None, str]:
    if parcel_shape_count == 1 and len(register_rows) == 1:
        return register_rows[0], "one-to-one"

    names = {
        _normalize_building_name(building.get("BLD_NM", "")),
        _normalize_building_name(building.get("DONG_NM", "")),
    } - {""}
    name_matches = [
        row for row in register_rows
        if names & {
            _normalize_building_name(row.get("건물명", "")),
            _normalize_building_name(row.get("동명", "")),
        } - {""}
    ]
    if len(name_matches) == 1:
        return name_matches[0], "name"
    if name_matches:
        return None, "ambiguous-name"

    agreed_values: dict[str, str] = {}
    for field in ("높이", "지상층수"):
        values = [_positive_number(row.get(field, "")) for row in register_rows]
        if values and values[0] > 0.0 and all(value == values[0] for value in values):
            agreed_values[field] = f"{values[0]:g}"
    if agreed_values:
        return agreed_values, "parcel-consensus"
    return None, "unmatched"


def _read_dbf_rows(data: bytes) -> list[dict[str, str]]:
    """SHP 속성표를 열어 각 도형에 붙은 문자·숫자 정보를 읽는다."""
    count, header_size, row_size = struct.unpack_from("<IHH", data, 4)
    fields: list[tuple[str, int, int]] = []
    offset = 1
    for position in range(32, header_size - 1, 32):
        descriptor = data[position : position + 32]
        if descriptor[0] == 0x0D:
            break
        name = descriptor[:11].split(b"\0", 1)[0].decode("cp949", "replace")
        width = descriptor[16]
        fields.append((name, offset, width))
        offset += width

    rows = []
    for index in range(count):
        record = data[header_size + index * row_size : header_size + (index + 1) * row_size]
        if len(record) != row_size or record[0] == ord("*"):
            rows.append({})
            continue
        rows.append({
            name: record[start : start + width].decode("cp949", "replace").strip()
            for name, start, width in fields
        })
    return rows


def _read_building_register(
    csv_path: Path, legal_dong_path: Path
) -> dict[str, list[dict[str, str]]]:
    """법정동과 번지로 건축물대장 행을 필지번호에 연결한다."""
    legal_dongs = json.loads(legal_dong_path.read_text(encoding="utf-8"))
    records: dict[str, list[dict[str, str]]] = {}
    with csv_path.open(encoding="utf-8-sig", newline="") as source:
        for row in csv.DictReader(source):
            dong_code = legal_dongs.get(row.get("법정동", ""))
            land_type = row.get("대지구분", "").strip()
            lot_number = row.get("번", "").strip()
            sub_number = row.get("지", "").strip()
            if (row.get("시도") != "서울특별시" or row.get("시군구") != "송파구"
                    or not dong_code or land_type not in ("대지", "산")
                    or not lot_number.isdigit() or not sub_number.isdigit()):
                continue
            if len(lot_number) > 4 or len(sub_number) > 4:
                continue

            # PNU는 법정동 코드, 대지 구분, 본번, 부번을 이어 붙인 필지 식별자다.
            parcel_id = (
                dong_code
                + ("2" if land_type == "산" else "1")
                + lot_number.zfill(4)
                + sub_number.zfill(4)
            )
            records.setdefault(parcel_id, []).append(row)
    return records


def _read_vworld_buildings(
    path: Path,
    bounds: dict[str, float],
    register: dict[str, list[dict[str, str]]],
) -> list[dict[str, Any]]:
    """송파구 건물 외곽선에 확인 가능한 건축물대장 정보를 붙인다."""
    with zipfile.ZipFile(path) as archive:
        members = {Path(name).name.lower(): name for name in archive.namelist()}
        shp_name = next((name for key, name in members.items() if key.endswith(".shp")), None)
        dbf_name = next((name for key, name in members.items() if key.endswith(".dbf")), None)
        prj_name = next((name for key, name in members.items() if key.endswith(".prj")), None)
        if not shp_name or not dbf_name or not prj_name:
            raise ValueError(f"건물 자료에 SHP, DBF, 좌표계 파일이 모두 필요합니다: {path.name}")
        projection = archive.read(prj_name).decode("cp949", "replace")
        if not any(name in projection for name in (
            "Korea_2000_Central_Belt_2010",
            "Korea_2000_Korea_Central_Belt_2010",
        )):
            raise ValueError(f"EPSG:5186 좌표계가 아닌 건물 자료입니다: {path.name}")
        rows = _read_dbf_rows(archive.read(dbf_name))
        shapes, _ = _read_shapefile(archive.read(shp_name))

    if len(shapes) != len(rows):
        raise ValueError(f"건물 도형 수와 속성 수가 다릅니다: {path.name}")

    parcel_counts: dict[str, int] = {}
    for row in rows:
        parcel_id = row.get("PNU", "")
        if parcel_id:
            parcel_counts[parcel_id] = parcel_counts.get(parcel_id, 0) + 1

    features = []
    for ring, row in zip(shapes, rows, strict=True):
        if not ring or not row:
            continue
        clipped = _clip_ring(ring, bounds)
        if not clipped:
            continue

        parcel_id = row.get("PNU", "")
        register_row, register_match = _match_building_register(
            row, parcel_counts.get(parcel_id, 0), register.get(parcel_id, [])
        )
        shape_height = row.get("HEIGHT", "")
        register_height = register_row.get("높이", "") if register_row else ""
        shape_floors = row.get("GRND_FLR", "")
        register_floors = register_row.get("지상층수", "") if register_row else ""
        shape_name = row.get("BLD_NM", "") or row.get("DONG_NM", "")
        register_name = (
            (register_row.get("건물명", "") or register_row.get("동명", ""))
            if register_row else ""
        )

        tags: dict[str, str] = {"building": "yes"}
        if register_row:
            tags["building_register_match"] = register_match
        name = register_name or shape_name
        if name:
            tags["name"] = name
        height = _positive_number(shape_height) or _positive_number(register_height)
        if height > 0:
            tags["height"] = f"{height:g}"
            tags["height_source"] = "vworld-footprint" if _positive_number(shape_height) else "building-register"
        if _positive_number(register_height) > 0:
            tags["building_register_height"] = f"{_positive_number(register_height):g}"
            if (height > 0 and _positive_number(shape_height) > 0
                    and abs(_positive_number(shape_height) - _positive_number(register_height)) > 0.5):
                tags["height_conflict"] = "true"
        floors = _positive_number(register_floors) or _positive_number(shape_floors)
        if floors > 0:
            tags["building:levels"] = f"{floors:g}"

        coordinates = [list(epsg5186_to_wgs84(x, y)) for x, y in clipped]
        coordinates.append(coordinates[0])
        features.append({
            "type": "Feature",
            "id": row.get("UFID", ""),
            "properties": {
                "osm_id": f"fac:{row.get('UFID', '')}",
                "parcel_id": parcel_id,
                "building_register_pk": register_row.get("PK", "") if register_row else "",
                "tags": tags,
            },
            "geometry": {"type": "Polygon", "coordinates": [coordinates]},
        })
    return features


def _combine_building_sources(
    detailed_features: list[dict[str, Any]],
    building_features: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """겹치는 건물은 1:1 수치지도 외곽선을 쓰고, 나머지는 건물 자료로 채운다."""
    candidates: dict[int, list[int]] = {index: [] for index in range(len(building_features))}
    reverse: dict[int, list[int]] = {index: [] for index in range(len(detailed_features))}
    for building_index, building in enumerate(building_features):
        building_ring = building["geometry"]["coordinates"][0]
        building_center = _center(building_ring)
        for detailed_index, detailed in enumerate(detailed_features):
            detailed_ring = detailed["geometry"]["coordinates"][0]
            detailed_center = _center(detailed_ring)
            if (_point_in_ring(building_center, detailed_ring)
                    or _point_in_ring(detailed_center, building_ring)):
                candidates[building_index].append(detailed_index)
                reverse[detailed_index].append(building_index)

    combined = []
    consumed: set[int] = set()
    for building_index, building in enumerate(building_features):
        matches = candidates[building_index]
        if len(matches) == 1 and len(reverse[matches[0]]) == 1:
            detailed_index = matches[0]
            detailed = detailed_features[detailed_index]
            combined.append({
                **building,
                "geometry": detailed["geometry"],
                "properties": {
                    **building["properties"],
                    "osm_id": detailed["properties"]["osm_id"],
                },
            })
            consumed.add(detailed_index)
        elif not matches:
            combined.append(building)

    # 여러 도형이 한 건물로 겹치면 어느 도형의 속성인지 단정하지 않는다.
    combined.extend(
        feature for index, feature in enumerate(detailed_features)
        if index not in consumed
    )
    return combined


def _read_shapefile(data: bytes) -> tuple[list[list[tuple[float, float]]], tuple[float, ...]]:
    shape_type = struct.unpack_from("<i", data, 32)[0]
    if shape_type != 5:
        raise ValueError(f"건물 도형은 Polygon이어야 합니다. 실제 형식: {shape_type}")
    bounds = struct.unpack_from("<4d", data, 36)
    records: list[list[tuple[float, float]]] = []
    position = 100
    while position < len(data):
        content_size = struct.unpack_from(">i", data, position + 4)[0] * 2
        content = data[position + 8 : position + 8 + content_size]
        position += 8 + content_size
        if len(content) < 44 or struct.unpack_from("<i", content, 0)[0] == 0:
            records.append([])
            continue
        record_type = struct.unpack_from("<i", content, 0)[0]
        if record_type != 5:
            raise ValueError(f"건물 도형 안에 예기치 않은 형식이 있습니다: {record_type}")

        part_count, point_count = struct.unpack_from("<ii", content, 36)
        if part_count < 1 or point_count < 3:
            records.append([])
            continue
        parts = struct.unpack_from(f"<{part_count}i", content, 44)
        points_offset = 44 + 4 * part_count
        points = [
            struct.unpack_from("<2d", content, points_offset + index * 16)
            for index in range(point_count)
        ]
        records.append(points[parts[0] : parts[1]] if part_count > 1 else points[parts[0] :])
    return records, bounds


def _read_tree_points(data: bytes) -> tuple[list[tuple[float, float]], tuple[float, ...]]:
    """수치지도 나무 점의 좌표를 읽는다."""
    shape_type = struct.unpack_from("<i", data, 32)[0]
    if shape_type != 1:
        raise ValueError(f"나무 도형은 Point여야 합니다. 실제 형식: {shape_type}")
    bounds = struct.unpack_from("<4d", data, 36)
    points = []
    position = 100
    while position < len(data):
        content_size = struct.unpack_from(">i", data, position + 4)[0] * 2
        content = data[position + 8 : position + 8 + content_size]
        position += 8 + content_size
        if len(content) < 20 or struct.unpack_from("<i", content, 0)[0] == 0:
            points.append((math.nan, math.nan))
            continue
        if struct.unpack_from("<i", content, 0)[0] != 1:
            raise ValueError("나무 도형 안에 예기치 않은 형식이 있습니다.")
        points.append(struct.unpack_from("<2d", content, 4))
    return points, bounds


def _clip_ring(
    ring: list[tuple[float, float]], bounds: dict[str, float]
) -> list[tuple[float, float]]:
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring = ring[:-1]
    edges = (
        (0, bounds["west"], True),
        (0, bounds["east"], False),
        (1, bounds["south"], True),
        (1, bounds["north"], False),
    )
    polygon = ring
    for axis, edge, keep_greater in edges:
        if not polygon:
            return []

        def inside(point: tuple[float, float]) -> bool:
            return point[axis] >= edge if keep_greater else point[axis] <= edge

        def cross(first: tuple[float, float], second: tuple[float, float]) -> tuple[float, float]:
            amount = (edge - first[axis]) / (second[axis] - first[axis])
            return (first[0] + (second[0] - first[0]) * amount,
                    first[1] + (second[1] - first[1]) * amount)

        clipped: list[tuple[float, float]] = []
        previous = polygon[-1]
        previous_inside = inside(previous)
        for current in polygon:
            current_inside = inside(current)
            if current_inside:
                if not previous_inside:
                    clipped.append(cross(previous, current))
                clipped.append(current)
            elif previous_inside:
                clipped.append(cross(previous, current))
            previous, previous_inside = current, current_inside
        polygon = clipped
    cleaned = []
    for point in polygon:
        if not cleaned or math.dist(point, cleaned[-1]) > 1e-8:
            cleaned.append(point)
    if len(cleaned) > 1 and math.dist(cleaned[0], cleaned[-1]) <= 1e-8:
        cleaned.pop()
    return cleaned if len(cleaned) >= 3 else []


def _point_in_ring(point: tuple[float, float], ring: list[list[float]]) -> bool:
    inside = False
    previous = ring[-1]
    for current in ring:
        if (current[1] > point[1]) != (previous[1] > point[1]):
            x_cross = (previous[0] - current[0]) * (point[1] - current[1]) / (
                previous[1] - current[1]
            ) + current[0]
            if point[0] < x_cross:
                inside = not inside
        previous = current
    return inside


def _center(ring: list[list[float]]) -> tuple[float, float]:
    points = ring[:-1] if ring and ring[0] == ring[-1] else ring
    return (sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points))


def _footprint_key(feature: dict[str, Any]) -> tuple[tuple[float, float], ...]:
    """도엽 경계에서 되풀이된 같은 건물 외곽선을 같은 키로 만든다."""
    ring = feature.get("geometry", {}).get("coordinates", [[]])[0]
    points = [(round(point[0], 8), round(point[1], 8)) for point in ring]
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    if not points:
        return ()

    # 시작점이나 진행 방향이 다른 같은 외곽선도 같은 키로 본다.
    candidates = []
    for ordered in (points, list(reversed(points))):
        start = min(range(len(ordered)), key=ordered.__getitem__)
        candidates.append(tuple(ordered[start:] + ordered[:start]))
    return min(candidates)


def _deduplicate_footprints(
    features: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    unique: dict[tuple[tuple[float, float], ...], dict[str, Any]] = {}
    for feature in features:
        unique.setdefault(_footprint_key(feature), feature)
    return list(unique.values())


def _deduplicate_site_buildings(
    features: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    unique: dict[tuple[tuple[float, float], ...], dict[str, Any]] = {}
    retained = []
    for feature in features:
        properties = feature.get("properties", {})
        geometry = feature.get("geometry", {})
        is_building = (
            geometry.get("type") == "Polygon"
            and properties.get("tags", {}).get("building")
        )
        if not is_building:
            retained.append(feature)
            continue
        unique.setdefault(_footprint_key(feature), feature)

    retained.extend(unique.values())
    return retained


def _ring_bounds(ring: list[list[float]]) -> tuple[float, float, float, float]:
    longitudes, latitudes = zip(*ring, strict=True)
    return min(longitudes), min(latitudes), max(longitudes), max(latitudes)


def _bounds_overlap_ratio(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    left = max(first[0], second[0])
    bottom = max(first[1], second[1])
    right = min(first[2], second[2])
    top = min(first[3], second[3])
    overlap = max(0.0, right - left) * max(0.0, top - bottom)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - overlap
    return overlap / union if union > 0.0 else 0.0


def _ring_area(ring: list[list[float]]) -> float:
    return abs(sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(ring, ring[1:] + ring[:1], strict=True)
    ) / 2.0)


def _is_bridge_duplicate_building(
    building: dict[str, Any], bridges: list[dict[str, Any]],
) -> bool:
    building_ring = building["geometry"]["coordinates"][0]
    building_bounds = _ring_bounds(building_ring)
    building_area = _ring_area(building_ring)
    bridge_groups: dict[str, list[list[list[float]]]] = {}
    for bridge in bridges:
        bridge_ring = bridge["geometry"]["coordinates"][0]
        bridge_area = _ring_area(bridge_ring)
        if bridge_area <= 0.0 or building_area <= 0.0:
            continue
        area_ratio = min(building_area, bridge_area) / max(building_area, bridge_area)
        if (area_ratio >= 0.85
                and _bounds_overlap_ratio(building_bounds, _ring_bounds(bridge_ring)) >= 0.9):
            return True
        tags = bridge.get("properties", {}).get("tags", {})
        name = tags.get("name:ko") or tags.get("name")
        if name:
            bridge_groups.setdefault(name, []).append(bridge_ring)

    # 나란한 차도 두 면을 건물 하나로 묶어 그린 도형도 중복으로 걸러낸다.
    for rings in bridge_groups.values():
        if len(rings) < 2 or not all(
            _point_in_ring(_center(ring), building_ring) for ring in rings
        ):
            continue
        bridge_area = sum(_ring_area(ring) for ring in rings)
        bridge_bounds = (
            min(_ring_bounds(ring)[0] for ring in rings),
            min(_ring_bounds(ring)[1] for ring in rings),
            max(_ring_bounds(ring)[2] for ring in rings),
            max(_ring_bounds(ring)[3] for ring in rings),
        )
        area_ratio = min(building_area, bridge_area) / max(building_area, bridge_area)
        if (area_ratio >= 0.85
                and _bounds_overlap_ratio(building_bounds, bridge_bounds) >= 0.9):
            return True
    return False


def _read_archive(path: Path, bounds: dict[str, float]) -> list[dict[str, Any]]:
    with zipfile.ZipFile(path) as archive:
        members = {Path(name).name.lower(): name for name in archive.namelist()}
        layer = BUILDING_LAYER.lower()
        shp_name, dbf_name, prj_name = (members.get(layer + suffix) for suffix in (".shp", ".dbf", ".prj"))
        if not shp_name or not dbf_name or not prj_name:
            return []
        projection = archive.read(prj_name).decode("cp949", "replace")
        if "Korea_2000_Central_Belt_2010" not in projection:
            raise ValueError(f"EPSG:5186 좌표계가 아닌 도엽입니다: {path.name}")

        shapes, _ = _read_shapefile(archive.read(shp_name))
        ids = _read_dbf_ids(archive.read(dbf_name))
        if len(shapes) != len(ids):
            raise ValueError(f"도형 수와 속성 수가 다릅니다: {path.name}")

        features = []
        for ring, feature_id in zip(shapes, ids, strict=True):
            if not ring or not feature_id:
                continue
            clipped = _clip_ring(ring, bounds)
            if not clipped:
                continue
            coordinates = [list(epsg5186_to_wgs84(x, y)) for x, y in clipped]
            coordinates.append(coordinates[0])
            features.append({
                "type": "Feature",
                "id": feature_id,
                "properties": {"osm_id": f"ngii:{feature_id}", "tags": {"building": "yes"}},
                "geometry": {"type": "Polygon", "coordinates": [coordinates]},
            })
        return features


def _read_tree_archive(path: Path, bounds: dict[str, float]) -> list[dict[str, Any]]:
    """1:1,000 수치지도에 표시된 나무 위치를 GeoJSON 점으로 바꾼다."""
    with zipfile.ZipFile(path) as archive:
        members = {Path(name).name.lower(): name for name in archive.namelist()}
        layer = TREE_LAYER.lower()
        shp_name, dbf_name, prj_name = (
            members.get(layer + suffix) for suffix in (".shp", ".dbf", ".prj")
        )
        if not shp_name or not dbf_name or not prj_name:
            return []
        projection = archive.read(prj_name).decode("cp949", "replace")
        if "Korea_2000_Central_Belt_2010" not in projection:
            raise ValueError(f"EPSG:5186 좌표계가 아닌 도엽입니다: {path.name}")

        points, _ = _read_tree_points(archive.read(shp_name))
        rows = _read_dbf_rows(archive.read(dbf_name))
        if len(points) != len(rows):
            raise ValueError(f"나무 도형 수와 속성 수가 다릅니다: {path.name}")

        features = []
        for point, row in zip(points, rows, strict=True):
            easting, northing = point
            if not math.isfinite(easting) or not math.isfinite(northing):
                continue
            if not (bounds["west"] <= easting <= bounds["east"]
                    and bounds["south"] <= northing <= bounds["north"]):
                continue
            identifier = row.get("UFID", "")
            if not identifier:
                continue
            features.append({
                "type": "Feature",
                "id": identifier,
                "properties": {
                    "osm_id": f"ngii-tree:{identifier}",
                    "tags": {"natural": "tree", "source": "NGII 1:1,000"},
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": list(epsg5186_to_wgs84(easting, northing)),
                },
            })
        return features


def _read_bridge_archive(path: Path, bounds: dict[str, float]) -> list[dict[str, Any]]:
    """1:1,000 수치지도에 기록된 교량 외곽선을 장면 좌표로 바꾼다."""
    with zipfile.ZipFile(path) as archive:
        members = {Path(name).name.lower(): name for name in archive.namelist()}
        layer = BRIDGE_LAYER.lower()
        shp_name, dbf_name, prj_name = (
            members.get(layer + suffix) for suffix in (".shp", ".dbf", ".prj")
        )
        if not shp_name or not dbf_name or not prj_name:
            return []
        projection = archive.read(prj_name).decode("cp949", "replace")
        if "Korea_2000_Central_Belt_2010" not in projection:
            raise ValueError(f"EPSG:5186 좌표계가 아닌 도엽입니다: {path.name}")

        shapes, _ = _read_shapefile(archive.read(shp_name))
        rows = _read_dbf_rows(archive.read(dbf_name))
        if len(shapes) != len(rows):
            raise ValueError(f"교량 도형 수와 속성 수가 다릅니다: {path.name}")

        features = []
        for ring, row in zip(shapes, rows, strict=True):
            if len(ring) < 4 or not row.get("UFID"):
                continue
            bridge_width = _positive_number(row.get("폭"))
            if bridge_width > 0:
                span_x = max(point[0] for point in ring) - min(point[0] for point in ring)
                span_y = max(point[1] for point in ring) - min(point[1] for point in ring)
                # 도형보다 작은 지도 표기 기호는 속성표의 실제 교량 폭으로 걸러낸다.
                if math.hypot(span_x, span_y) < bridge_width:
                    continue
            clipped = _clip_ring(ring, bounds)
            if not clipped:
                continue
            name = row.get("명칭", "").strip()
            tags = {
                "man_made": "bridge",
                "source": "국토지리정보원 1:1,000 수치지도",
            }
            if name:
                tags.update({"name": name, "name:ko": name, "bridge:name": name})
            for source_field, tag in (
                ("종류", "bridge:type"), ("연장", "bridge:length_m"),
                ("폭", "bridge:width_m"), ("설치연도", "bridge:installed"),
                ("재질", "bridge:material"), ("하천명", "bridge:waterway"),
            ):
                value = row.get(source_field, "").strip()
                if value:
                    tags[tag] = value
            coordinates = [list(epsg5186_to_wgs84(x, y)) for x, y in clipped]
            if coordinates[-1] != coordinates[0]:
                coordinates.append(coordinates[0])
            identifier = row["UFID"]
            features.append({
                "type": "Feature",
                "id": identifier,
                "properties": {
                    "osm_id": f"ngii-bridge:{identifier}",
                    "tags": tags,
                },
                "geometry": {"type": "Polygon", "coordinates": [coordinates]},
            })
        return features


def _merge_site(
    site: dict[str, Any], official_features: list[dict[str, Any]],
    ngii_bridge_features: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    ngii_bridge_features = ngii_bridge_features or []
    # 이전 변환 결과를 다시 입력해도 수치지도 도형이 계속 쌓이지 않게 한다.
    previous_official = {}
    for feature in site["features"]:
        properties = feature.get("properties", {})
        source_id = str(properties.get("osm_id", ""))
        if source_id.startswith(("ngii:", "ngii-tree:", "ngii-bridge:", "fac:")):
            previous_official[source_id] = properties
    source_features = [
        feature for feature in site["features"]
        if not str(feature.get("properties", {}).get("osm_id", "")).startswith(
            ("ngii:", "ngii-tree:", "ngii-bridge:", "fac:")
        )
    ]
    source_features = _deduplicate_site_buildings(source_features)
    osm_buildings = []
    source_bridge_features = []
    for feature in source_features:
        properties = feature.get("properties", {})
        geometry = feature.get("geometry", {})
        if (geometry.get("type") == "Polygon"
                and properties.get("tags", {}).get("building")):
            osm_buildings.append(feature)
        if (geometry.get("type") == "Polygon"
                and properties.get("tags", {}).get("man_made") == "bridge"):
            source_bridge_features.append(feature)

    official_bridge_names = {
        feature.get("properties", {}).get("tags", {}).get("name:ko")
        or feature.get("properties", {}).get("tags", {}).get("name")
        for feature in ngii_bridge_features
    }
    bridge_features_for_buildings = [
        feature for feature in source_bridge_features
        if (
            feature.get("properties", {}).get("tags", {}).get("name:ko")
            or feature.get("properties", {}).get("tags", {}).get("name")
        ) not in official_bridge_names
    ] + ngii_bridge_features

    matched_osm_ids: set[str] = set()
    official_features = _deduplicate_footprints(official_features)
    # 교량 상판과 같은 외곽선이 수치지도 건물로 중복 등록되면 하부 경로가 막힌다.
    official_features = [
        feature for feature in official_features
        if not _is_bridge_duplicate_building(feature, bridge_features_for_buildings)
    ]
    for official in official_features:
        source_id = str(official.get("properties", {}).get("osm_id", ""))
        old_properties = previous_official.get(source_id, {})
        if old_properties:
            new_properties = official.get("properties", {})
            old_tags = old_properties.get("tags", {})
            new_tags = new_properties.get("tags", {})
            official["properties"] = {
                **old_properties,
                **new_properties,
                "tags": {**old_tags, **new_tags},
            }
        official_ring = official["geometry"]["coordinates"][0]
        official_center = _center(official_ring)
        source_properties: dict[str, Any] = {}
        source_tags: dict[str, Any] = {}
        for osm in osm_buildings:
            properties = osm["properties"]
            osm_id = str(properties.get("osm_id", ""))
            if osm_id in matched_osm_ids:
                continue
            osm_ring = osm["geometry"]["coordinates"][0]
            osm_center = _center(osm_ring)
            if (_point_in_ring(official_center, osm_ring)
                    or _point_in_ring(osm_center, official_ring)):
                if not source_properties:
                    source_properties = properties
                source_tags.update(properties.get("tags", {}))
                if osm_id:
                    matched_osm_ids.add(osm_id)
        if source_properties:
            official_properties = official.get("properties", {})
            official_tags = official_properties.get("tags", {})
            official["properties"] = {
                **source_properties,
                **official_properties,
                "osm_id": source_properties.get(
                    "osm_id", official_properties.get("osm_id", "")
                ),
                "tags": {**source_tags, **official_tags},
            }
            official["geometry"] = {
                "type": "Polygon", "coordinates": [official_ring]
            }

    retained = [
        feature for feature in source_features
        if str(feature.get("properties", {}).get("osm_id", "")) not in matched_osm_ids
    ]
    if official_bridge_names:
        retained = [
            feature for feature in retained
            if not (
                feature.get("geometry", {}).get("type") == "Polygon"
                and feature.get("properties", {}).get("tags", {}).get("man_made") == "bridge"
                and (
                    feature.get("properties", {}).get("tags", {}).get("name:ko")
                    or feature.get("properties", {}).get("tags", {}).get("name")
                ) in official_bridge_names
            )
        ]
    return {
        **site,
        "features": _deduplicate_site_buildings(
            retained + official_features + ngii_bridge_features
        ),
    }


def convert(
    source_dir: Path,
    site_path: Path,
    bounds_path: Path,
    output_path: Path,
    building_data_path: Path | None = None,
    register_path: Path | None = None,
    legal_dong_path: Path | None = None,
) -> int:
    bounds = json.loads(bounds_path.read_text(encoding="utf-8"))["bounds_meters"]
    source_files = sorted(source_dir.glob("*.zip"))
    if not source_files:
        raise ValueError(f"수치지도 ZIP이 없습니다: {source_dir}")

    official_features = []
    bridge_features = []
    tree_features = []
    for source in source_files:
        official_features.extend(_read_archive(source, bounds))
        bridge_features.extend(_read_bridge_archive(source, bounds))
        tree_features.extend(_read_tree_archive(source, bounds))
    if building_data_path and building_data_path.exists():
        if not register_path or not legal_dong_path:
            raise ValueError("건축물대장 CSV와 법정동 코드 자료도 지정해야 합니다.")
        register = _read_building_register(register_path, legal_dong_path)
        building_features = _read_vworld_buildings(building_data_path, bounds, register)
        official_features = _combine_building_sources(official_features, building_features)
    if not official_features:
        raise ValueError(
            "받은 도엽의 건물 도형이 현재 장면과 겹치지 않습니다. "
            "기존 지도를 보존했습니다. 장면 경계와 좌표가 겹치는 도엽을 추가해 주세요."
        )

    site = json.loads(site_path.read_text(encoding="utf-8"))
    merged = _merge_site(site, official_features, bridge_features)
    retained_features = [
        feature for feature in merged["features"]
        if not str(feature.get("properties", {}).get("osm_id", "")).startswith("ngii-tree:")
    ]
    merged["features"] = retained_features + tree_features
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", newline="\n", dir=output_path.parent,
        prefix=output_path.name + ".", suffix=".tmp", delete=False
    ) as temporary:
        temporary_path = Path(temporary.name)
        json.dump(merged, temporary, ensure_ascii=False, separators=(",", ":"))
        temporary.write("\n")
    os.replace(temporary_path, output_path)
    return len(official_features)


def main() -> int:
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-dir", type=Path,
        default=repository / "assets/maps/tancheon/source/ngii-map-1-1000",
        help="국토지리정보원 1:1,000 수치지도 ZIP 폴더",
    )
    parser.add_argument(
        "--site", type=Path, default=repository / "assets/maps/tancheon/site.geojson",
        help="기존 도로·건물 자료",
    )
    parser.add_argument(
        "--bounds", type=Path,
        default=repository / "assets/maps/tancheon/orthophoto-source.json",
        help="장면 좌표 범위",
    )
    parser.add_argument(
        "--output", type=Path, default=repository / "assets/maps/tancheon/site.geojson",
        help="변환 결과를 저장할 GeoJSON 경로",
    )
    source = repository / "assets/maps/tancheon/source"
    parser.add_argument(
        "--building-data", type=Path, default=source / "vworld-buildings-songpa.zip",
        help="선택한 송파구 건물 외곽선 자료 ZIP",
    )
    parser.add_argument(
        "--building-register", type=Path, default=source / "building-register-songpa.csv",
        help="건축물대장 CSV",
    )
    parser.add_argument(
        "--legal-dong-codes", type=Path, default=source / "legal-dong-codes-songpa.json",
        help="법정동 이름과 코드 대응 자료",
    )
    args = parser.parse_args()
    try:
        count = convert(
            args.source_dir, args.site, args.bounds, args.output,
            args.building_data, args.building_register, args.legal_dong_codes,
        )
    except (OSError, ValueError, zipfile.BadZipFile, json.JSONDecodeError) as error:
        parser.exit(1, f"지도 변환 실패: {error}\n")
    print(f"건물 {count}개를 장면 범위에 맞춰 {args.output}에 반영했습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
