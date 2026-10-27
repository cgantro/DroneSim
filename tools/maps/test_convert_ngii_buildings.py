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

    @staticmethod
    def _write_building_archive(path, ring):
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

        dbf_header = bytearray(65)
        dbf_header[0] = 3
        struct.pack_into("<IHH", dbf_header, 4, 1, 65, 255)
        dbf_field = bytearray(32)
        dbf_field[:4] = b"UFID"
        dbf_field[11] = ord("C")
        dbf_field[16] = 254
        dbf_header[32:64] = dbf_field
        dbf_header[64] = 0x0D
        dbf_data = dbf_header + b" " + b"test-id".ljust(254) + b"\x1a"
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
