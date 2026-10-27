import importlib.util
import json
import struct
import tempfile
import unittest
import zipfile
from pathlib import Path


SCRIPT = Path(__file__).with_name("convert_ngii_buildings.py")
SPEC = importlib.util.spec_from_file_location("convert_ngii_buildings", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
converter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(converter)


class NgiiBuildingConversionTests(unittest.TestCase):
    def test_integrated_building_archive_streams_name_height_and_floors(self):
        bounds = {"west": 209000.0, "south": 543795.0,
                  "east": 209015.0, "north": 543810.0}
        ring = [
            (209004.0, 543799.0), (209008.0, 543799.0),
            (209008.0, 543803.0), (209004.0, 543803.0),
            (209004.0, 543799.0),
        ]
        fields = [("A1", 28), ("A2", 19), ("A16", 18),
                  ("A24", 254), ("A25", 254), ("A26", 9)]
        row = {
            "A1": "building-001",
            "A2": "1171010500102230000",
            "A16": "35.4",
            "A24": "탄천 아파트",
            "A25": "101동",
            "A26": "10",
        }

        with tempfile.TemporaryDirectory() as temporary:
            archive = Path(temporary) / "integrated.zip"
            self._write_building_archive(archive, ring, fields, row)

            self.assertTrue(converter._is_integrated_building_archive(archive))
            features = converter._read_integrated_buildings(archive, bounds)

        self.assertEqual(len(features), 1)
        tags = features[0]["properties"]["tags"]
        self.assertEqual(features[0]["id"], "building-001")
        self.assertEqual(tags["name"], "탄천 아파트")
        self.assertEqual(tags["height"], "35.4")
        self.assertEqual(tags["building:levels"], "10")

    def test_integrated_building_metadata_keeps_existing_name_and_records_conflicts(self):
        ring = [[127.0, 37.0], [127.001, 37.0],
                [127.001, 37.001], [127.0, 37.001], [127.0, 37.0]]
        existing = {
            "type": "Feature",
            "properties": {
                "osm_id": "map-building-1",
                "tags": {
                    "building": "yes", "name": "기존 건물명",
                    "height": "28", "building:levels": "8",
                },
            },
            "geometry": {"type": "Polygon", "coordinates": [ring]},
        }
        integrated = {
            "type": "Feature", "id": "source-1",
            "properties": {"parcel_id": "pnu-1", "tags": {
                "building": "yes", "name": "새 자료 이름",
                "height": "30", "building:levels": "9",
                "height_source": "building-integrated-data",
            }},
            "geometry": {"type": "Polygon", "coordinates": [ring]},
        }

        result = converter._enrich_building_metadata([existing], [integrated])
        tags = result[0]["properties"]["tags"]

        self.assertEqual(len(result), 1)
        self.assertEqual(tags["name"], "기존 건물명")
        self.assertEqual(tags["height"], "30")
        self.assertEqual(tags["building_previous_height"], "28")
        self.assertEqual(tags["building:levels"], "9")
        self.assertEqual(tags["building_previous_levels"], "8")
        self.assertEqual(result[0]["properties"]["building_integrated_id"], "source-1")

    def test_scene_origin_projects_back_to_recorded_location(self):
        longitude, latitude = converter.epsg5186_to_wgs84(
            209006.56251788812, 543801.2429522893
        )

        self.assertAlmostEqual(longitude, 127.1018462, places=8)
        self.assertAlmostEqual(latitude, 37.4936231, places=8)

    def test_register_matches_a_unique_building_name_within_one_parcel(self):
        rows = [
            {"동명": "가동", "높이": "18.2", "지상층수": "7", "PK": "one"},
            {"동명": "나동", "높이": "18.2", "지상층수": "7", "PK": "two"},
        ]

        result, match = converter._match_building_register(
            {"DONG_NM": "가 동", "BLD_NM": ""}, 2, rows
        )

        self.assertEqual(match, "name")
        self.assertIs(result, rows[0])

    def test_register_uses_only_values_shared_by_every_building_on_a_parcel(self):
        rows = [
            {"동명": "가동", "높이": "18.2", "지상층수": "7", "PK": "one"},
            {"동명": "나동", "높이": "18.2", "지상층수": "7", "PK": "two"},
        ]

        result, match = converter._match_building_register(
            {"DONG_NM": "다동"}, 3, rows
        )

        self.assertEqual(match, "parcel-consensus")
        self.assertEqual(result, {"높이": "18.2", "지상층수": "7"})

    def test_register_does_not_guess_when_buildings_on_a_parcel_disagree(self):
        rows = [
            {"동명": "가동", "높이": "18.2", "지상층수": "7"},
            {"동명": "나동", "높이": "24.0", "지상층수": "8"},
        ]

        result, match = converter._match_building_register(
            {"DONG_NM": "다동"}, 3, rows
        )

        self.assertIsNone(result)
        self.assertEqual(match, "unmatched")

    def test_polygon_is_clipped_to_the_scene_rectangle(self):
        bounds = {"west": 0.0, "south": 0.0, "east": 10.0, "north": 10.0}
        clipped = converter._clip_ring(
            [(-5.0, 5.0), (5.0, -5.0), (15.0, 5.0), (5.0, 15.0)],
            bounds,
        )

        self.assertEqual(len(clipped), 4)
        self.assertTrue(all(0.0 <= x <= 10.0 and 0.0 <= y <= 10.0 for x, y in clipped))

    def test_archive_conversion_keeps_matching_osm_name_and_id(self):
        bounds = {"west": 209000.0, "south": 543795.0,
                  "east": 209015.0, "north": 543810.0}
        projected_ring = [
            (209004.0, 543799.0), (209008.0, 543799.0),
            (209008.0, 543803.0), (209004.0, 543803.0),
            (209004.0, 543799.0),
        ]
        geographic_ring = [
            list(converter.epsg5186_to_wgs84(easting, northing))
            for easting, northing in projected_ring
        ]

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            self._write_building_archive(source / "building.zip", projected_ring)
            site = root / "site.geojson"
            site.write_text(json.dumps({
                "type": "FeatureCollection",
                "features": [{
                    "type": "Feature",
                    "properties": {
                        "osm_id": "keep-this-id",
                        "tags": {"building": "apartments", "name": "시험 건물"},
                    },
                    "geometry": {"type": "Polygon", "coordinates": [geographic_ring]},
                }],
            }), encoding="utf-8")
            bounds_file = root / "bounds.json"
            bounds_file.write_text(json.dumps({"bounds_meters": bounds}), encoding="utf-8")
            output = root / "merged.geojson"

            count = converter.convert(source, site, bounds_file, output)
            result = json.loads(output.read_text(encoding="utf-8"))

        self.assertEqual(count, 1)
        self.assertEqual(len(result["features"]), 1)
        properties = result["features"][0]["properties"]
        self.assertEqual(properties["osm_id"], "keep-this-id")
        self.assertEqual(properties["tags"]["name"], "시험 건물")

    def test_non_overlapping_archive_does_not_replace_existing_output(self):
        ring = [(204870.0, 543960.0), (204874.0, 543960.0),
                (204874.0, 543964.0), (204870.0, 543964.0),
                (204870.0, 543960.0)]
        bounds = {"west": 209000.0, "south": 543795.0,
                  "east": 209015.0, "north": 543810.0}

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            self._write_building_archive(source / "building.zip", ring)
            site = root / "site.geojson"
            site.write_text('{"type":"FeatureCollection","features":[]}', encoding="utf-8")
            bounds_file = root / "bounds.json"
            bounds_file.write_text(json.dumps({"bounds_meters": bounds}), encoding="utf-8")
            output = root / "merged.geojson"
            output.write_text("keep existing data", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "겹치지 않습니다"):
                converter.convert(source, site, bounds_file, output)

            self.assertEqual(output.read_text(encoding="utf-8"), "keep existing data")

    def test_bridge_deck_is_not_imported_as_a_building(self):
        bridge_ring = [
            [127.1000, 37.4930], [127.1030, 37.4930],
            [127.1030, 37.4940], [127.1000, 37.4940],
            [127.1000, 37.4930],
        ]
        duplicate_ring = [
            [127.10001, 37.49301], [127.10299, 37.49301],
            [127.10299, 37.49399], [127.10001, 37.49399],
            [127.10001, 37.49301],
        ]
        bridge = {
            "type": "Feature",
            "properties": {"osm_id": "bridge-1", "tags": {"man_made": "bridge"}},
            "geometry": {"type": "Polygon", "coordinates": [bridge_ring]},
        }
        duplicate = {
            "type": "Feature",
            "properties": {"osm_id": "ngii:deck-1", "tags": {"building": "yes"}},
            "geometry": {"type": "Polygon", "coordinates": [duplicate_ring]},
        }
        separate_building = {
            "type": "Feature",
            "properties": {"osm_id": "ngii:house-1", "tags": {"building": "yes"}},
            "geometry": {"type": "Polygon", "coordinates": [[
                [127.1040, 37.4930], [127.1042, 37.4930],
                [127.1042, 37.4932], [127.1040, 37.4932],
                [127.1040, 37.4930],
            ]]},
        }

        result = converter._merge_site(
            {"type": "FeatureCollection", "features": [bridge]},
            [duplicate, separate_building],
        )
        ids = {feature["properties"]["osm_id"] for feature in result["features"]}

        self.assertIn("bridge-1", ids)
        self.assertIn("ngii:house-1", ids)
        self.assertNotIn("ngii:deck-1", ids)

    def test_road_ramp_outlines_are_not_imported_as_buildings(self):
        def building(feature_id):
            offset = 0.01 if feature_id == "ordinary-building" else 0.0
            return {
                "type": "Feature",
                "id": feature_id,
                "properties": {
                    "osm_id": f"ngii:{feature_id}",
                    "tags": {"building": "yes"},
                },
                "geometry": {"type": "Polygon", "coordinates": [[
                    [127.0 + offset, 37.0], [127.001 + offset, 37.0],
                    [127.001 + offset, 37.001], [127.0 + offset, 37.001],
                    [127.0 + offset, 37.0],
                ]]},
            }

        retained_building_id = "ngii:ordinary-building"
        result = converter._merge_site(
            {"type": "FeatureCollection", "features": []},
            [
                building("1000037709004A00110000000000003425"),
                building("1000377090220A00110000000000000058"),
                building("ordinary-building"),
            ],
        )

        ids = {feature["properties"]["osm_id"] for feature in result["features"]}
        self.assertEqual(ids, {retained_building_id})

    def test_official_bridge_outline_replaces_same_named_osm_polygon(self):
        osm = {
            "type": "Feature",
            "properties": {
                "osm_id": "osm-bridge",
                "tags": {"man_made": "bridge", "name:ko": "탄천교"},
            },
            "geometry": {"type": "Polygon", "coordinates": [[
                [127.1000, 37.4930], [127.1010, 37.4930],
                [127.1010, 37.4940], [127.1000, 37.4940],
                [127.1000, 37.4930],
            ]]},
        }
        official = {
            **osm,
            "id": "map-bridge",
            "properties": {
                "osm_id": "ngii-bridge:map-bridge",
                "tags": {"man_made": "bridge", "name:ko": "탄천교"},
            },
        }

        result = converter._merge_site(
            {"type": "FeatureCollection", "features": [osm]}, [], [official]
        )

        self.assertEqual(
            [feature["properties"]["osm_id"] for feature in result["features"]],
            ["ngii-bridge:map-bridge"],
        )

    def test_bridge_lanes_remove_a_single_building_outline_covering_both(self):
        def feature(ring):
            return {
                "properties": {
                    "tags": {"name:ko": "탄천교"},
                },
                "geometry": {"type": "Polygon", "coordinates": [ring]},
            }

        building = feature([
            [0.0, 0.0], [10.0, 0.0], [10.0, 2.0],
            [0.0, 2.0], [0.0, 0.0],
        ])
        bridges = [
            feature([
                [0.0, 0.0], [4.5, 0.0], [4.5, 2.0],
                [0.0, 2.0], [0.0, 0.0],
            ]),
            feature([
                [5.5, 0.0], [10.0, 0.0], [10.0, 2.0],
                [5.5, 2.0], [5.5, 0.0],
            ]),
        ]

        self.assertTrue(converter._is_bridge_duplicate_building(building, bridges))

    @staticmethod
    def _write_building_archive(path, ring, fields=None, row=None):
        xs, ys = zip(*ring)
        shape_bounds = min(xs), min(ys), max(xs), max(ys)
        content = struct.pack("<i4d2i", 5, *shape_bounds, 1, len(ring))
        content += struct.pack("<i", 0)
        content += b"".join(struct.pack("<2d", x, y) for x, y in ring)
        shape_header = bytearray(100)
        struct.pack_into(">i", shape_header, 0, 9994)
        struct.pack_into(">i", shape_header, 24, (100 + 8 + len(content)) // 2)
        struct.pack_into("<ii", shape_header, 28, 1000, 5)
        struct.pack_into("<4d", shape_header, 36, *shape_bounds)
        shape_data = shape_header + struct.pack(">ii", 1, len(content) // 2) + content

        fields = fields or [("UFID", 254)]
        row = row or {"UFID": "test-id"}
        header_size = 32 + 32 * len(fields) + 1
        row_size = 1 + sum(width for _, width in fields)
        dbf_header = bytearray(header_size)
        dbf_header[0] = 3
        struct.pack_into("<IHH", dbf_header, 4, 1, header_size, row_size)
        dbf_offset = 32
        for name, width in fields:
            dbf_field = bytearray(32)
            dbf_field[: len(name)] = name.encode("ascii")
            dbf_field[11] = ord("C")
            dbf_field[16] = width
            dbf_header[dbf_offset : dbf_offset + 32] = dbf_field
            dbf_offset += 32
        dbf_header[dbf_offset] = 0x0D
        dbf_record = bytearray(b" ")
        for name, width in fields:
            dbf_record.extend(row.get(name, "").encode("cp949").ljust(width))
        dbf_data = dbf_header + dbf_record + b"\x1a"
        projection = (
            'PROJCS["Korea_2000_Central_Belt_2010",'
            'PARAMETER["False_Easting",200000.0]]'
        )
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("N1A_A0010000.shp", shape_data)
            archive.writestr("N1A_A0010000.dbf", dbf_data)
            archive.writestr("N1A_A0010000.prj", projection)


if __name__ == "__main__":
    unittest.main()
