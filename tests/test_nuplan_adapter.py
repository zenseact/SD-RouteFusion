import json
import math
import os
import sqlite3
from contextlib import closing
from types import SimpleNamespace
import tempfile
import unittest

from errors import NoLinksInQueriedAreaError
from main import run_nuplan
from nuplan_adapter import process_nuplan, projected_points_to_local, yaw_from_quaternion


def token(index):
    return index.to_bytes(8, byteorder="big")


def yaw_quaternion(yaw_rad):
    return math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)


class FakeTransformer:
    def transform(self, x, y):
        return x / 1000.0, y / 1000.0


def fake_transformer_factory(epsg):
    if epsg != 32610:
        raise AssertionError(f"Unexpected EPSG: {epsg}")
    return FakeTransformer()


class StubRouteGenerator:
    def __init__(self, fail=False):
        self.fail = fail
        self.inputs = []

    def create_route(self, input_data):
        self.inputs.append(input_data)
        if self.fail:
            raise NoLinksInQueriedAreaError("no links")
        return (
            [[0.0, 0.0], [2.0, 3.0]],
            {"valid_route": True, "future_points_seen": len(input_data.fp["local_lat"])},
            ["10-11"],
            0.0,
            0.0,
        )

    @staticmethod
    def get_generic_route_coords():
        return [[0.0, 0.0], [0.0, 500.0]]

    @staticmethod
    def get_generic_route_properties():
        return {"valid_route": False}


def create_nuplan_db(path, num_poses=4, yaw_rad=0.0):
    qw, qx, qy, qz = yaw_quaternion(yaw_rad)
    with closing(sqlite3.connect(path)) as connection:
        connection.execute(
            """
            CREATE TABLE log (
                token BLOB NOT NULL,
                map_version TEXT
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE scene (
                token BLOB NOT NULL,
                name TEXT
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE ego_pose (
                token BLOB NOT NULL,
                log_token BLOB NOT NULL,
                timestamp INTEGER,
                x REAL,
                y REAL,
                qw REAL,
                qx REAL,
                qy REAL,
                qz REAL,
                epsg INTEGER
            )
            """
        )
        connection.execute(
            """
            CREATE TABLE lidar_pc (
                token BLOB NOT NULL,
                ego_pose_token BLOB NOT NULL,
                scene_token BLOB,
                timestamp INTEGER
            )
            """
        )
        connection.execute("INSERT INTO log VALUES (?, ?)", (token(1), "us-test-map"))
        connection.execute("INSERT INTO scene VALUES (?, ?)", (token(2), "scene-001"))

        for index in range(num_poses):
            ego_pose_token = token(100 + index)
            timestamp_us = index * 1_000_000
            connection.execute(
                "INSERT INTO ego_pose VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    ego_pose_token,
                    token(1),
                    timestamp_us,
                    1000.0 + 10.0 * index,
                    2000.0,
                    qw,
                    qx,
                    qy,
                    qz,
                    32610,
                ),
            )
            connection.execute(
                "INSERT INTO lidar_pc VALUES (?, ?, ?, ?)",
                (token(200 + index), ego_pose_token, token(2), timestamp_us),
            )
        connection.commit()


def read_jsonl(path):
    with open(path, "r", encoding="utf-8") as input_file:
        return [json.loads(line) for line in input_file]


class NuPlanAdapterTest(unittest.TestCase):
    def test_run_nuplan_writes_route_results_and_metadata(self):
        route_generator = StubRouteGenerator()
        with tempfile.TemporaryDirectory() as tmp_dir:
            db_path = os.path.join(tmp_dir, "sample.db")
            output_path = os.path.join(tmp_dir, "routes.jsonl")
            create_nuplan_db(db_path, yaw_rad=math.pi / 2.0)

            args = SimpleNamespace(
                input=db_path,
                output=output_path,
                cache_location="None",
                map_query_max_l1=1000,
                nuplan_future_horizon_s=2.0,
                nuplan_stride=1,
                nuplan_max_samples=1,
            )
            run_nuplan(
                args,
                route_generator_factory=lambda _cache, _dist: route_generator,
                transformer_factory=fake_transformer_factory,
            )
            rows = read_jsonl(output_path)

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["dataset"], "nuPlan")
        self.assertEqual(rows[0]["sample_id"], "sample:00000000000000c8")
        self.assertEqual(rows[0]["lidar_pc_token"], "00000000000000c8")
        self.assertEqual(rows[0]["scene_name"], "scene-001")
        self.assertEqual(rows[0]["map_name"], "us-test-map")
        self.assertEqual(rows[0]["epsg"], 32610)
        self.assertEqual(rows[0]["future_horizon_s"], 2.0)
        self.assertEqual(rows[0]["future_points"], 3)
        self.assertEqual(rows[0]["route_coords"], [[0.0, 0.0], [2.0, 3.0]])
        self.assertEqual(rows[0]["map_links"], ["10-11"])
        self.assertFalse(rows[0]["fallback"])
        self.assertIsNone(rows[0]["error_type"])

        self.assertEqual(route_generator.inputs[0].pred_time["lat"], 2.0)
        self.assertEqual(route_generator.inputs[0].pred_time["lon"], 1.0)
        self.assertAlmostEqual(route_generator.inputs[0].pred_time["heading"], 90.0)
        self.assertAlmostEqual(route_generator.inputs[0].fp["local_lat"][1], 0.0, places=6)
        self.assertAlmostEqual(route_generator.inputs[0].fp["local_lon"][1], 10.0, places=6)

    def test_stride_and_max_samples_limit_processed_anchors(self):
        route_generator = StubRouteGenerator()
        with tempfile.TemporaryDirectory() as tmp_dir:
            db_path = os.path.join(tmp_dir, "sample.db")
            output_path = os.path.join(tmp_dir, "routes.jsonl")
            create_nuplan_db(db_path, num_poses=6)

            process_nuplan(
                input_path=db_path,
                output_path=output_path,
                route_generator=route_generator,
                future_horizon_s=2.0,
                stride=2,
                max_samples=2,
                transformer_factory=fake_transformer_factory,
            )
            rows = read_jsonl(output_path)

        self.assertEqual([row["lidar_pc_token"] for row in rows], ["00000000000000c8", "00000000000000ca"])
        self.assertEqual(len(route_generator.inputs), 2)

    def test_skips_anchors_with_insufficient_future_poses(self):
        route_generator = StubRouteGenerator()
        with tempfile.TemporaryDirectory() as tmp_dir:
            db_path = os.path.join(tmp_dir, "sample.db")
            output_path = os.path.join(tmp_dir, "routes.jsonl")
            create_nuplan_db(db_path, num_poses=1)

            count = process_nuplan(
                input_path=db_path,
                output_path=output_path,
                route_generator=route_generator,
                future_horizon_s=8.0,
                transformer_factory=fake_transformer_factory,
            )
            rows = read_jsonl(output_path)

        self.assertEqual(count, 0)
        self.assertEqual(rows, [])
        self.assertEqual(route_generator.inputs, [])

    def test_route_failure_writes_fallback_row(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            db_path = os.path.join(tmp_dir, "sample.db")
            output_path = os.path.join(tmp_dir, "routes.jsonl")
            create_nuplan_db(db_path)

            process_nuplan(
                input_path=db_path,
                output_path=output_path,
                route_generator=StubRouteGenerator(fail=True),
                future_horizon_s=2.0,
                max_samples=1,
                transformer_factory=fake_transformer_factory,
            )
            rows = read_jsonl(output_path)

        self.assertEqual(rows[0]["route_coords"], [[0.0, 0.0], [0.0, 500.0]])
        self.assertEqual(rows[0]["route_properties"], {"valid_route": False})
        self.assertEqual(rows[0]["map_links"], [])
        self.assertTrue(rows[0]["fallback"])
        self.assertEqual(rows[0]["error_type"], "NoLinksInQueriedAreaError")

    def test_yaw_and_local_coordinate_conversion(self):
        heading = yaw_from_quaternion(*yaw_quaternion(math.pi / 2.0))
        points = projected_points_to_local(
            [(0.0, 0.0), (10.0, 0.0)],
            anchor_x=0.0,
            anchor_y=0.0,
            heading_rad=heading,
        )

        self.assertAlmostEqual(points[1][0], 0.0, places=6)
        self.assertAlmostEqual(points[1][1], 10.0, places=6)


if __name__ == "__main__":
    unittest.main()
