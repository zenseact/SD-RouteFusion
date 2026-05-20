import importlib.util
import unittest
from unittest.mock import patch

HAS_TRANSFORM_DEPS = all(
    importlib.util.find_spec(name) is not None
    for name in ["geopy", "numpy", "shapely", "utm"]
)

if HAS_TRANSFORM_DEPS:
    from utils.transformations import global_to_vehicle_coordinates


def fake_from_latlon(lat, lon, *_zone):
    return lat * 1000.0, lon * 1000.0, 32, "V"


@unittest.skipUnless(HAS_TRANSFORM_DEPS, "coordinate transform dependencies are not installed")
class CoordinateTransformTest(unittest.TestCase):
    def test_global_to_vehicle_coordinates_translates_without_rotation(self):
        with patch("utils.transformations.utm.from_latlon", side_effect=fake_from_latlon):
            points = global_to_vehicle_coordinates(
                [[10.005, 20.002]],
                [10.0, 20.0],
                0.0,
            )

        self.assertAlmostEqual(points[0][0], 5.0, places=6)
        self.assertAlmostEqual(points[0][1], 2.0, places=6)

    def test_global_to_vehicle_coordinates_applies_heading_rotation(self):
        with patch("utils.transformations.utm.from_latlon", side_effect=fake_from_latlon):
            points = global_to_vehicle_coordinates(
                [[10.010, 20.0]],
                [10.0, 20.0],
                90.0,
            )

        self.assertAlmostEqual(points[0][0], 0.0, places=6)
        self.assertAlmostEqual(points[0][1], 10.0, places=6)


if __name__ == "__main__":
    unittest.main()
