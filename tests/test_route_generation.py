import importlib.util
import unittest
from unittest.mock import patch

HAS_ROUTE_DEPS = all(
    importlib.util.find_spec(name) is not None
    for name in ["geopy", "numpy", "osmnx", "shapely"]
)

if HAS_ROUTE_DEPS:
    from shapely.geometry import LineString
    from route_generation import InputData, RouteGenerator


class FakeTree:
    def __init__(self, links, wrapper, vehicle_data):
        self.links = links
        self.wrapper = wrapper
        self.vehicle_data = vehicle_data
        self.routes = [object(), object()]

    def insert_start_points(self, max_distance, iter):
        self.max_distance = max_distance
        self.iter = iter

    def find_possible_routes(self):
        return None

    def get_routes_as_linestrings(self):
        return [
            LineString([(0.0, 20.0), (200.0, 20.0)]),
            LineString([(0.0, 0.0), (200.0, 0.0)]),
        ]


@unittest.skipUnless(HAS_ROUTE_DEPS, "route-generation dependencies are not installed")
class RouteGenerationTest(unittest.TestCase):
    def test_generic_fallback_route_shape_and_properties(self):
        self.assertEqual(RouteGenerator.get_generic_route_coords(), [[0.0, 0.0], [0.0, 500.0]])
        properties = RouteGenerator.get_generic_route_properties()

        self.assertFalse(properties["valid_route"])
        self.assertEqual(properties["road_class"], "UNCLASSIFIED")

    def test_create_route_selects_candidate_closest_to_future_path(self):
        input_data = InputData(
            pred_time={"lat": 57.0, "lon": 12.0, "heading": 0.0},
            fp={
                "local_lat": [0.0, 200.0],
                "local_lon": [0.0, 0.0],
            },
            sequence_id="selection_case",
        )

        def fake_properties(_self, _tree, best_route_index, _vehicle_data):
            return {"valid_route": True, "best_route_index": best_route_index}

        generator = RouteGenerator(wrapper=None, map_max_l1_dist=1000)
        with patch("route_generation.Tree", FakeTree), patch.object(
            RouteGenerator,
            "create_map",
            return_value=["1-2", "2-3"],
        ), patch.object(
            RouteGenerator,
            "get_route_properties",
            autospec=True,
            side_effect=fake_properties,
        ):
            route_coords, route_properties, map_links, _map_time, _route_time = generator.create_route(input_data)

        self.assertEqual(route_coords, [[0.0, 0.0], [200.0, 0.0]])
        self.assertEqual(route_properties, {"valid_route": True, "best_route_index": 1})
        self.assertEqual(map_links, ["1-2", "2-3"])


if __name__ == "__main__":
    unittest.main()
