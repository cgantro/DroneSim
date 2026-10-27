import importlib.util
import unittest
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

    def test_polygon_is_clipped_to_the_scene_rectangle(self):
        bounds = {"west": 0.0, "south": 0.0, "east": 10.0, "north": 10.0}
        clipped = converter._clip_ring(
            [(-5.0, 5.0), (5.0, -5.0), (15.0, 5.0), (5.0, 15.0)],
            bounds,
        )

        self.assertEqual(len(clipped), 4)
        self.assertTrue(all(0.0 <= x <= 10.0 and 0.0 <= y <= 10.0 for x, y in clipped))


if __name__ == "__main__":
    unittest.main()
