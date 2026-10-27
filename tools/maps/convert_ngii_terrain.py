"""원본 DEM에 1:1,000 등고선과 표고점을 반영해 탄천 지형 격자를 만든다."""

from __future__ import annotations

import argparse
import json
import math
import struct
import zipfile
from pathlib import Path
from typing import Iterable

import numpy as np
import rasterio
from pyproj import Transformer

from convert_ngii_buildings import _read_dbf_rows


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


def load_contours_and_heights(source_dirs: Iterable[Path], bounds: dict[str, float]):
    """장면 범위에 들어오는 등고선과 표고점을 읽는다."""
    contours: list[tuple[float, float, float]] = []
    spot_heights: list[tuple[float, float, float]] = []
    archive_paths = sorted(
        archive_path
        for source_dir in source_dirs
        for archive_path in source_dir.glob("*.zip")
    )
    for archive_path in archive_paths:
        with zipfile.ZipFile(archive_path) as archive:
            members = {Path(name).name.lower(): name for name in archive.namelist()}
            for layer in ("n1l_f0010000", "n3l_f0010000"):
                contour_shape = members.get(f"{layer}.shp")
                contour_table = members.get(f"{layer}.dbf")
                if not contour_shape or not contour_table:
                    continue
                try:
                    shape_data = archive.read(contour_shape)
                    rows = _read_dbf_rows(archive.read(contour_table))
                except zipfile.BadZipFile:
                    continue
                west, south, east, north = struct.unpack_from("<4d", shape_data, 36)
                if (west > bounds["east"] or east < bounds["west"]
                        or south > bounds["north"] or north < bounds["south"]):
                    continue
                for parts, row in zip(read_shapes(shape_data, 3), rows, strict=True):
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

            for layer in ("n1p_f0020000", "n3p_f0020000"):
                point_shape = members.get(f"{layer}.shp")
                point_table = members.get(f"{layer}.dbf")
                if not point_shape or not point_table:
                    continue
                try:
                    rows = _read_dbf_rows(archive.read(point_table))
                    shapes = read_shapes(archive.read(point_shape), 1)
                    for points, row in zip(shapes, rows, strict=True):
                        if not points or not row.get("수치"):
                            continue
                        x, y = points[0]
                        if (bounds["west"] <= x <= bounds["east"]
                                and bounds["south"] <= y <= bounds["north"]):
                            spot_heights.append((x, y, float(row["수치"])))
                except zipfile.BadZipFile:
                    continue
    if not contours or not spot_heights:
        raise ValueError("장면 범위에서 등고선과 표고점을 찾지 못했습니다.")
    return contours, spot_heights


def load_base_dem(path: Path, metadata: dict, reference: float):
    """원본 DEM을 장면 좌표로 읽고 중심 기준 높이 차를 만든다."""
    bounds = metadata["bounds_meters"]
    count_value = (bounds["east"] - bounds["west"]) / GRID_SPACING_METERS
    if not math.isclose(count_value, round(count_value), abs_tol=1.0e-6):
        raise ValueError("장면 너비가 지형 격자 간격으로 나누어떨어지지 않습니다.")
    count = round(count_value) + 1

    with rasterio.open(path) as source:
        if source.crs is None or source.transform.a <= 0 or source.transform.e >= 0:
            raise ValueError("DEM에 좌표계나 북쪽 기준 격자 정보가 없습니다.")
        target_crs = metadata["target_crs"].split()[0]
        transform = Transformer.from_crs(target_crs, source.crs, always_xy=True)
        columns, rows = np.meshgrid(np.arange(count), np.arange(count))
        target_x = bounds["west"] + columns * GRID_SPACING_METERS
        target_y = bounds["north"] - rows * GRID_SPACING_METERS
        source_x, source_y = transform.transform(target_x, target_y)

        source_column = (source_x - source.transform.c) / source.transform.a - 0.5
        source_row = (source_y - source.transform.f) / source.transform.e - 0.5
        if (np.any(source_x < source.bounds.left)
                or np.any(source_x > source.bounds.right)
                or np.any(source_y < source.bounds.bottom)
                or np.any(source_y > source.bounds.top)):
            raise ValueError("DEM이 필요한 장면 범위를 모두 덮지 않습니다.")
        source_column = np.clip(source_column, 0.0, source.width - 1.0)
        source_row = np.clip(source_row, 0.0, source.height - 1.0)

        column_before = np.floor(source_column).astype(int)
        row_before = np.floor(source_row).astype(int)
        column_after = np.minimum(column_before + 1, source.width - 1)
        row_after = np.minimum(row_before + 1, source.height - 1)
        values = source.read(1)
        if source.nodata is not None and any(
            np.any(values[row_index, column_index] == source.nodata)
            for row_index, column_index in (
                (row_before, column_before), (row_before, column_after),
                (row_after, column_before), (row_after, column_after))
        ):
            raise ValueError("DEM 장면 범위에 높이 자료가 비어 있는 곳이 있습니다.")
        column_fraction = source_column - column_before
        row_fraction = source_row - row_before
        north = (values[row_before, column_before] * (1.0 - column_fraction)
                 + values[row_before, column_after] * column_fraction)
        south = (values[row_after, column_before] * (1.0 - column_fraction)
                 + values[row_after, column_after] * column_fraction)
        elevations = north * (1.0 - row_fraction) + south * row_fraction

        if not np.all(np.isfinite(elevations)):
            raise ValueError("DEM에 유효하지 않은 높이가 있습니다.")

        center_elevation = float(elevations[count // 2, count // 2])
        offsets = (elevations - center_elevation).astype("<f4").ravel()
        latitude = metadata["target_center_wgs84"]["latitude"]
        longitude = metadata["target_center_wgs84"]["longitude"]
        return (count, GRID_SPACING_METERS, GRID_SPACING_METERS, reference,
                latitude, longitude, center_elevation, offsets)


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
    parser.add_argument(
        "--source-dir", type=Path, action="append", required=True,
        help="등고선·표고점이 든 수치지도 ZIP 폴더. 여러 폴더를 지정할 수 있습니다.",
    )
    parser.add_argument("--dem", type=Path, required=True)
    parser.add_argument("--terrain-metadata", type=Path, required=True)
    parser.add_argument("--orthophoto-metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    metadata = json.loads(args.orthophoto_metadata.read_text(encoding="utf-8"))
    bounds = metadata["bounds_meters"]
    center_x = (bounds["west"] + bounds["east"]) / 2
    center_y = (bounds["south"] + bounds["north"]) / 2
    terrain_metadata = json.loads(args.terrain_metadata.read_text(encoding="utf-8"))
    terrain = load_base_dem(
        args.dem, metadata, terrain_metadata["simulation_reference_height_meters"]
    )
    contours, spot_heights = load_contours_and_heights(args.source_dir, bounds)
    baseline, corrected = validate(
        contours, spot_heights, terrain, center_x, center_y
    )
    print(f"DEM 기준 중심 높이: {terrain[6]:.3f}m")
    print(f"읽은 등고선 표본: {len(contours):,}개, 표고점: {len(spot_heights)}개")
    print(f"표고점 제외 검증 MAE: {baseline[0]:.2f}m → {corrected[0]:.2f}m")
    print(f"표고점 제외 검증 RMSE: {baseline[1]:.2f}m → {corrected[1]:.2f}m")
    if corrected[0] >= baseline[0] or corrected[1] >= baseline[1]:
        raise RuntimeError("등고선 보정이 기존 격자보다 나아지지 않아 저장을 중단합니다.")
    write_grid(args.output, terrain, contours, spot_heights,
        bounds, center_x, center_y)


if __name__ == "__main__":
    main()
