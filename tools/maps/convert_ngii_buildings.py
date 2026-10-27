"""NGII 1:1,000 건물 도형을 탄천 장면의 GeoJSON에 반영한다."""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import tempfile
import zipfile
from pathlib import Path
from typing import Any


BUILDING_LAYER = "N1A_A0010000"
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
    count, header_size, row_size = struct.unpack_from("<IHH", data, 4)
    fields: list[tuple[str, int, int]] = []
    offset = 1
    for position in range(32, header_size - 1, 32):
        descriptor = data[position : position + 32]
        if descriptor[0] == 0x0D:
            break
        name = descriptor[:11].split(b"\0", 1)[0].decode("ascii")
        width = descriptor[16]
        fields.append((name, offset, width))
        offset += width

    id_field = next((field for field in fields if field[0] == "UFID"), None)
    if id_field is None:
        raise ValueError("건물 속성표에 UFID 항목이 없습니다.")
    _, id_offset, id_width = id_field

    ids = []
    for row in range(count):
        record = data[header_size + row * row_size : header_size + (row + 1) * row_size]
        if len(record) != row_size or record[0] == ord("*"):
            ids.append("")
            continue
        ids.append(record[id_offset : id_offset + id_width].decode("ascii").strip())
    return ids


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


def _merge_site(
    site: dict[str, Any], official_features: list[dict[str, Any]]
) -> dict[str, Any]:
    # 이전 변환 결과를 다시 입력해도 수치지도 도형이 계속 쌓이지 않게 한다.
    source_features = [
        feature for feature in site["features"]
        if not str(feature.get("properties", {}).get("osm_id", "")).startswith("ngii:")
    ]
    source_features = _deduplicate_site_buildings(source_features)
    osm_buildings = []
    for feature in source_features:
        properties = feature.get("properties", {})
        geometry = feature.get("geometry", {})
        if (geometry.get("type") == "Polygon"
                and properties.get("tags", {}).get("building")):
            osm_buildings.append(feature)

    matched_osm_ids: set[str] = set()
    official_features = _deduplicate_footprints(official_features)
    for official in official_features:
        official_ring = official["geometry"]["coordinates"][0]
        official_center = _center(official_ring)
        for osm in osm_buildings:
            properties = osm["properties"]
            osm_id = str(properties.get("osm_id", ""))
            if osm_id in matched_osm_ids:
                continue
            osm_ring = osm["geometry"]["coordinates"][0]
            osm_center = _center(osm_ring)
            if (_point_in_ring(official_center, osm_ring)
                    or _point_in_ring(osm_center, official_ring)):
                official["properties"] = dict(properties)
                official["geometry"] = {
                    "type": "Polygon", "coordinates": [official_ring]
                }
                if osm_id:
                    matched_osm_ids.add(osm_id)
                break

    retained = [
        feature for feature in source_features
        if str(feature.get("properties", {}).get("osm_id", "")) not in matched_osm_ids
    ]
    return {
        **site,
        "features": _deduplicate_site_buildings(retained + official_features),
    }


def convert(source_dir: Path, site_path: Path, bounds_path: Path, output_path: Path) -> int:
    bounds = json.loads(bounds_path.read_text(encoding="utf-8"))["bounds_meters"]
    source_files = sorted(source_dir.glob("*.zip"))
    if not source_files:
        raise ValueError(f"수치지도 ZIP이 없습니다: {source_dir}")

    official_features = []
    for source in source_files:
        official_features.extend(_read_archive(source, bounds))
    if not official_features:
        raise ValueError(
            "받은 도엽의 건물 도형이 현재 장면과 겹치지 않습니다. "
            "기존 지도를 보존했습니다. 장면 경계와 좌표가 겹치는 도엽을 추가해 주세요."
        )

    site = json.loads(site_path.read_text(encoding="utf-8"))
    merged = _merge_site(site, official_features)
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
    args = parser.parse_args()
    try:
        count = convert(args.source_dir, args.site, args.bounds, args.output)
    except (OSError, ValueError, zipfile.BadZipFile, json.JSONDecodeError) as error:
        parser.exit(1, f"지도 변환 실패: {error}\n")
    print(f"건물 {count}개를 장면 범위에 맞춰 {args.output}에 반영했습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
