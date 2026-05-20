import json
import os
from types import SimpleNamespace
import tempfile
import unittest

from errors import NoRoutesFoundError
from generic_jsonl import SchemaValidationError, process_jsonl
from main import run_generic_jsonl


def fake_coordinate_converter(points, ego_latlon, ego_heading):
    return [[float(index), float(index) + 0.5] for index, _point in enumerate(points)]


class StubRouteGenerator:
    def __init__(self, fail=False):
        self.fail = fail

    def create_route(self, input_data):
        if self.fail:
            raise NoRoutesFoundError("no route")
        return (
            [[0.0, 0.0], [1.0, 1.0]],
            {
                "valid_route": True,
                "speed_limit": float("nan"),
                "sequence": input_data.sequence_id,
                "local_lat": input_data.fp["local_lat"],
            },
            ["1-2"],
            0.0,
            0.0,
        )

    @staticmethod
    def get_generic_route_coords():
        return [[0.0, 0.0], [0.0, 500.0]]

    @staticmethod
    def get_generic_route_properties():
        return {"valid_route": False}


def write_jsonl(path, rows):
    with open(path, "w", encoding="utf-8") as output_file:
        for row in rows:
            if isinstance(row, str):
                output_file.write(row + "\n")
            else:
                output_file.write(json.dumps(row) + "\n")


class GenericJSONLTest(unittest.TestCase):
    def test_run_generic_jsonl_writes_route_results(self):
        sample = {
            "sample_id": "demo_001",
            "ego": {"lat": 57.0, "lon": 12.0, "heading": 0.0},
            "future_path": [
                [57.0001, 12.0001],
                [57.0002, 12.0002],
            ],
            "metadata": {"split": "val", "timestamp_us": 123456},
        }

        with tempfile.TemporaryDirectory() as tmp_dir:
            input_path = os.path.join(tmp_dir, "input.jsonl")
            output_path = os.path.join(tmp_dir, "output.jsonl")
            write_jsonl(input_path, [sample])

            args = SimpleNamespace(
                input=input_path,
                output=output_path,
                cache_location="None",
                map_query_max_l1=1000,
            )
            run_generic_jsonl(
                args,
                route_generator_factory=lambda _cache, _dist: StubRouteGenerator(),
                coordinate_converter=fake_coordinate_converter,
            )

            with open(output_path, "r", encoding="utf-8") as input_file:
                rows = [json.loads(line) for line in input_file]

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["sample_id"], "demo_001")
        self.assertEqual(rows[0]["route_coords"], [[0.0, 0.0], [1.0, 1.0]])
        self.assertEqual(rows[0]["map_links"], ["1-2"])
        self.assertFalse(rows[0]["fallback"])
        self.assertIsNone(rows[0]["error_type"])
        self.assertIsNone(rows[0]["route_properties"]["speed_limit"])
        self.assertEqual(rows[0]["route_properties"]["local_lat"], [0.0, 1.0])
        self.assertEqual(
            rows[0]["input_metadata"],
            {"split": "val", "timestamp_us": 123456},
        )

    def test_legacy_object_points_remain_supported(self):
        sample = {
            "sample_id": "legacy_object_points",
            "ego": {"lat": 57.0, "lon": 12.0, "heading": 0.0},
            "future_path": [
                {"lat": 57.0001, "lon": 12.0001},
                {"lat": 57.0002, "lon": 12.0002},
            ],
        }

        with tempfile.TemporaryDirectory() as tmp_dir:
            input_path = os.path.join(tmp_dir, "input.jsonl")
            output_path = os.path.join(tmp_dir, "output.jsonl")
            write_jsonl(input_path, [sample])

            process_jsonl(
                input_path,
                output_path,
                route_generator=StubRouteGenerator(),
                coordinate_converter=fake_coordinate_converter,
            )

            with open(output_path, "r", encoding="utf-8") as input_file:
                result = json.loads(input_file.readline())

        self.assertEqual(result["sample_id"], "legacy_object_points")
        self.assertEqual(result["route_properties"]["local_lat"], [0.0, 1.0])
        self.assertNotIn("input_metadata", result)

    def test_route_generation_failure_writes_fallback(self):
        sample = {
            "sample_id": "fallback_case",
            "ego": {"lat": 57.0, "lon": 12.0, "heading": 0.0},
            "future_path": [
                {"lat": 57.0001, "lon": 12.0001},
                {"lat": 57.0002, "lon": 12.0002},
            ],
        }

        with tempfile.TemporaryDirectory() as tmp_dir:
            input_path = os.path.join(tmp_dir, "input.jsonl")
            output_path = os.path.join(tmp_dir, "output.jsonl")
            write_jsonl(input_path, [sample])

            process_jsonl(
                input_path,
                output_path,
                route_generator=StubRouteGenerator(fail=True),
                coordinate_converter=fake_coordinate_converter,
            )

            with open(output_path, "r", encoding="utf-8") as input_file:
                result = json.loads(input_file.readline())

        self.assertEqual(result["route_coords"], [[0.0, 0.0], [0.0, 500.0]])
        self.assertEqual(result["route_properties"], {"valid_route": False})
        self.assertEqual(result["map_links"], [])
        self.assertTrue(result["fallback"])
        self.assertEqual(result["error_type"], "NoRoutesFoundError")

    def test_schema_validation_fails_fast(self):
        invalid_rows = [
            {"sample_id": "missing_ego", "future_path": [{"lat": 1.0, "lon": 2.0}, {"lat": 1.1, "lon": 2.1}]},
            {"sample_id": "missing_future", "ego": {"lat": 1.0, "lon": 2.0, "heading": 0.0}},
            {"sample_id": "too_short", "ego": {"lat": 1.0, "lon": 2.0, "heading": 0.0}, "future_path": [{"lat": 1.0, "lon": 2.0}]},
            {"sample_id": "bad_pair_length", "ego": {"lat": 1.0, "lon": 2.0, "heading": 0.0}, "future_path": [[1.0, 2.0, 3.0], [1.1, 2.1]]},
            {"sample_id": "bad_future_type", "ego": {"lat": 1.0, "lon": 2.0, "heading": 0.0}, "future_path": "not-a-list"},
            {"sample_id": "bad_future_number", "ego": {"lat": 1.0, "lon": 2.0, "heading": 0.0}, "future_path": [[1.0, 2.0], ["nan-ish", 2.1]]},
            {"sample_id": "bad_ego_number", "ego": {"lat": float("inf"), "lon": 2.0, "heading": 0.0}, "future_path": [[1.0, 2.0], [1.1, 2.1]]},
            {"sample_id": "bad_metadata", "ego": {"lat": 1.0, "lon": 2.0, "heading": 0.0}, "future_path": [[1.0, 2.0], [1.1, 2.1]], "metadata": "not-an-object"},
        ]

        for row in invalid_rows:
            with self.subTest(row=row["sample_id"]):
                with tempfile.TemporaryDirectory() as tmp_dir:
                    input_path = os.path.join(tmp_dir, "input.jsonl")
                    output_path = os.path.join(tmp_dir, "output.jsonl")
                    write_jsonl(input_path, [row])

                    with self.assertRaises(SchemaValidationError):
                        process_jsonl(
                            input_path,
                            output_path,
                            route_generator=StubRouteGenerator(),
                            coordinate_converter=fake_coordinate_converter,
                        )

    def test_invalid_json_fails_fast(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            input_path = os.path.join(tmp_dir, "input.jsonl")
            output_path = os.path.join(tmp_dir, "output.jsonl")
            write_jsonl(input_path, ['{"sample_id":'])

            with self.assertRaises(SchemaValidationError):
                process_jsonl(
                    input_path,
                    output_path,
                    route_generator=StubRouteGenerator(),
                    coordinate_converter=fake_coordinate_converter,
                )


if __name__ == "__main__":
    unittest.main()
