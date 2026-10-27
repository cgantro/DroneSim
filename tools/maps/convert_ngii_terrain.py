"""1:1,000 등고선·표고점을 기존 탄천 DEM에 반영한다."""

from __future__ import annotations

import argparse
import json
import math
import struct
import zipfile
from pathlib import Path

from convert_ngii_buildings import _read_dbf_rows


HEADER_SIZE = 40
GRID_SPACING_METERS = 5.0
SAMPLE_RADIUS_METERS = 100.0
SAMPLE_BIN_METERS = 25.0
NEIGHBOR_COUNT = 4


def read_shapes(data: bytes, shape_type: int):
    """SHP의 선 또는 점 좌표를 속성표 행 순서대로 읽는다."""
    if struct.unpack_from("<i", data, 32)[0] != shape_type:
        raise ValueError("수치지도 도형 종류가 예상한 형식과 다릅니다.")
    position = 100
    while position < len(data):
        content_size = struct.unpack_from(">i", data, position + 4)[0] * 2
        content = data[position + 8 : position + 8 + content_size]
        position += 8 + content_size
        record_type = struct.unpack_from("<i", content, 0)[0]
        if record_type == 0:
            yield []
        elif record_type == 1 and shape_type == 1:
            yield [struct.unpack_from("<2d", content, 4)]
        elif record_type == 3 and shape_type == 3:
            part_count, point_count = struct.unpack_from("<ii", content, 36)
            parts = struct.unpack_from(f"<{part_count}i", content, 44)
            points_offset = 44 + part_count * 4
            points = [
                struct.unpack_from("<2d", content, points_offset + index * 16)
                for index in range(point_count)
            ]
            yield [
                points[parts[index] : parts[index + 1] if index + 1 < part_count else point_count]
                for index in range(part_count)
            ]
        else:
            raise ValueError("수치지도 안에 예상하지 못한 도형이 있습니다.")


def load_contours_and_heights(source_dir: Path, bounds: dict[str, float]):
    """장면 범위에 들어오는 등고선과 표고점을 읽는다."""
    contours: list[tuple[float, float, float]] = []
    spot_heights: list[tuple[float, float, float]] = []
    for archive_path in sorted(source_dir.glob("*.zip")):
        with zipfile.ZipFile(archive_path) as archive:
            members = {Path(name).name.lower(): name for name in archive.namelist()}
            contour_shape = members.get("n1l_f0010000.shp")
            contour_table = members.get("n1l_f0010000.dbf")
            if contour_shape and contour_table:
                shape_data = archive.read(contour_shape)
                west, south, east, north = struct.unpack_from("<4d", shape_data, 36)
                if not (west > bounds["east"] or east < bounds["west"]
                        or south > bounds["north"] or north < bounds["south"]):
                    rows = _read_dbf_rows(archive.read(contour_table))
                    for parts, row in zip(
                        read_shapes(shape_data, 3), rows, strict=True
                    ):
                        elevation = float(row.get("등고수치", "0") or 0)
                        for line in parts:
                            for start, end in zip(line, line[1:]):
                                length = math.dist(start, end)
                                steps = max(1, math.ceil(length / 2.5))
                                for step in range(steps + 1):
                                    fraction = step / steps
                                    x = start[0] + (end[0] - start[0]) * fraction
                                    y = start[1] + (end[1] - start[1]) * fraction
                                    if (bounds["west"] <= x <= bounds["east"]
                                            and bounds["south"] <= y <= bounds["north"]):
                                        contours.append((x, y, elevation))

            point_shape = members.get("n1p_f0020000.shp")
            point_table = members.get("n1p_f0020000.dbf")
            if point_shape and point_table:
                rows = _read_dbf_rows(archive.read(point_table))
                for points, row in zip(
                    read_shapes(archive.read(point_shape), 1), rows, strict=True
                ):
                    if not points or not row.get("수치"):
                        continue
                    x, y = points[0]
                    if (bounds["west"] <= x <= bounds["east"]
                            and bounds["south"] <= y <= bounds["north"]):
                        spot_heights.append((x, y, float(row["수치"])))
    if not contours or not spot_heights:
        raise ValueError("장면 범위에서 등고선과 표고점을 찾지 못했습니다.")
    return contours, spot_heights


def load_base_grid(path: Path, source_elevation: float):
    """기존 격자를 읽고 높이 조회에 필요한 정보를 반환한다."""
    data = path.read_bytes()
    if data[:8] not in (b"AEROTRN1", b"AEROTRN2") or len(data) < HEADER_SIZE:
        raise ValueError("기존 지형 격자 파일을 읽지 못했습니다.")
    count = struct.unpack_from("<I", data, 8)[0]
    spacing_x, spacing_y, reference = struct.unpack_from("<fff", data, 12)
    latitude, longitude = struct.unpack_from("<dd", data, 24)
    code = "f" if data[:8] == b"AEROTRN2" else "h"
    values = struct.unpack_from(f"<{count * count}{code}", data, HEADER_SIZE)
    if code == "h":
        values = tuple(float(value) for value in values)
    return (count, spacing_x, spacing_y, reference, latitude, longitude,
            source_elevation, values)


def base_height(x, y, grid, center_x, center_y):
    count, spacing_x, spacing_y, _, _, _, source_elevation, values = grid
    column = (x - (center_x - (count - 1) * spacing_x / 2)) / spacing_x
    row = ((center_y + (count - 1) * spacing_y / 2) - y) / spacing_y
    column_before = max(0, min(count - 2, int(column)))
    row_before = max(0, min(count - 2, int(row)))
    column_fraction = column - column_before
    row_fraction = row - row_before
    at = lambda r, c: source_elevation + values[r * count + c]
    north = at(row_before, column_before) * (1 - column_fraction) + at(
        row_before, column_before + 1
    ) * column_fraction
    south = at(row_before + 1, column_before) * (1 - column_fraction) + at(
        row_before + 1, column_before + 1
    ) * column_fraction
    return north * (1 - row_fraction) + south * row_fraction


def add_to_bins(points, center_x, center_y):
    bins: dict[tuple[int, int], list[tuple[float, float, float]]] = {}
    for x, y, elevation in points:
        key = (math.floor((x - center_x) / SAMPLE_BIN_METERS),
               math.floor((y - center_y) / SAMPLE_BIN_METERS))
        bins.setdefault(key, []).append((x, y, elevation))
    return bins


def estimate_adjustment(x, y, contour_bins, spot_bins, center_x, center_y):
    """가까운 등고선·표고점과 기존 DEM의 높이 차이를 보간한다."""
    bin_x = math.floor((x - center_x) / SAMPLE_BIN_METERS)
    bin_y = math.floor((y - center_y) / SAMPLE_BIN_METERS)
    radius = math.ceil(SAMPLE_RADIUS_METERS / SAMPLE_BIN_METERS)
    samples = []
    for bins in (contour_bins, spot_bins):
        nearby = []
        for row in range(bin_y - radius, bin_y + radius + 1):
            for column in range(bin_x - radius, bin_x + radius + 1):
                nearby.extend(bins.get((column, row), ()))
        nearby.sort(key=lambda item: (item[0] - x) ** 2 + (item[1] - y) ** 2)
        samples.extend(nearby[:NEIGHBOR_COUNT])

    weighted_adjustment = 0.0
    total_weight = 0.0
    for sample_x, sample_y, residual in samples:
        distance_squared = (sample_x - x) ** 2 + (sample_y - y) ** 2
        weight = 1.0 / max(distance_squared, 1.0)
        weighted_adjustment += residual * weight
        total_weight += weight
    return weighted_adjustment / total_weight if total_weight else 0.0


def make_samples(contours, spot_heights, base_grid, center_x, center_y, omitted_spot=None):
    contours_with_residuals = [
        (x, y, elevation - base_height(x, y, base_grid, center_x, center_y))
        for x, y, elevation in contours
    ]
    spots_with_residuals = [
        (x, y, elevation - base_height(x, y, base_grid, center_x, center_y))
        for index, (x, y, elevation) in enumerate(spot_heights)
        if index != omitted_spot
    ]
    return (add_to_bins(contours_with_residuals, center_x, center_y),
            add_to_bins(spots_with_residuals, center_x, center_y))


def validate(contours, spot_heights, base_grid, center_x, center_y):
    """표고점 하나씩 빼고 예측해 기존 DEM과 오차를 비교한다."""
    baseline_errors = []
    corrected_errors = []
    for index, (x, y, elevation) in enumerate(spot_heights):
        baseline = base_height(x, y, base_grid, center_x, center_y)
        baseline_errors.append(baseline - elevation)
        contour_bins, spot_bins = make_samples(
            contours, spot_heights, base_grid, center_x, center_y, index
        )
        estimate = baseline + estimate_adjustment(
            x, y, contour_bins, spot_bins, center_x, center_y
        )
        corrected_errors.append(estimate - elevation)

    def metrics(errors):
        return (sum(abs(error) for error in errors) / len(errors),
                math.sqrt(sum(error * error for error in errors) / len(errors)))

    return metrics(baseline_errors), metrics(corrected_errors)


def write_grid(output_path, terrain, contours, spot_heights, bounds, center_x, center_y):
    count = round((bounds["east"] - bounds["west"]) / GRID_SPACING_METERS) + 1
    contour_bins, spot_bins = make_samples(
        contours, spot_heights, terrain, center_x, center_y
    )
    _, _, _, reference, latitude, longitude, _, _ = terrain
    center_height = base_height(center_x, center_y, terrain, center_x, center_y)
    center_height += estimate_adjustment(
        center_x, center_y, contour_bins, spot_bins, center_x, center_y
    )
    offsets = []
    for row in range(count):
        y = bounds["north"] - row * GRID_SPACING_METERS
        for column in range(count):
            x = bounds["west"] + column * GRID_SPACING_METERS
            height = base_height(x, y, terrain, center_x, center_y)
            height += estimate_adjustment(x, y, contour_bins, spot_bins, center_x, center_y)
            offsets.append(height - center_height)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as output:
        output.write(b"AEROTRN2")
        output.write(struct.pack("<Ifffdd", count, GRID_SPACING_METERS,
            GRID_SPACING_METERS, reference, latitude, longitude))
        output.write(struct.pack(f"<{len(offsets)}f", *offsets))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--terrain", type=Path, required=True)
    parser.add_argument("--terrain-metadata", type=Path, required=True)
    parser.add_argument("--orthophoto-metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    metadata = json.loads(args.orthophoto_metadata.read_text(encoding="utf-8"))
    bounds = metadata["bounds_meters"]
    center_x = (bounds["west"] + bounds["east"]) / 2
    center_y = (bounds["south"] + bounds["north"]) / 2
    terrain_metadata = json.loads(args.terrain_metadata.read_text(encoding="utf-8"))
    terrain = load_base_grid(
        args.terrain, terrain_metadata["center_elevation_meters_from_dem"]
    )
    contours, spot_heights = load_contours_and_heights(args.source_dir, bounds)
    baseline, corrected = validate(
        contours, spot_heights, terrain, center_x, center_y
    )
    print(f"읽은 등고선 표본: {len(contours):,}개, 표고점: {len(spot_heights)}개")
    print(f"표고점 제외 검증 MAE: {baseline[0]:.2f}m → {corrected[0]:.2f}m")
    print(f"표고점 제외 검증 RMSE: {baseline[1]:.2f}m → {corrected[1]:.2f}m")
    if corrected[0] >= baseline[0] or corrected[1] >= baseline[1]:
        raise RuntimeError("등고선 보정이 기존 격자보다 나아지지 않아 저장을 중단합니다.")
    write_grid(args.output, terrain, contours, spot_heights,
        bounds, center_x, center_y)


if __name__ == "__main__":
    main()
