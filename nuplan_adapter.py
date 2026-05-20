import json
import math
import os
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path
import sqlite3
import tempfile

from errors import NoLinksInQueriedAreaError, NoRoutesFoundError
from generic_jsonl import to_jsonable


class NuPlanAdapterError(RuntimeError):
    pass


@dataclass
class NuPlanInputData:
    pred_time: dict
    fp: dict
    sequence_id: str


def process_nuplan(
    input_path,
    output_path,
    route_generator,
    future_horizon_s=8.0,
    stride=1,
    max_samples=None,
    transformer_factory=None,
):
    validate_options(future_horizon_s, stride, max_samples)
    db_files = discover_db_files(input_path)
    transformer_factory = cached_transformer_factory(transformer_factory or default_transformer_factory)

    output_dir = os.path.dirname(output_path) or "."
    temp_file = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=output_dir,
        delete=False,
    )
    count = 0
    reached_limit = False
    try:
        with temp_file as output_file:
            for db_file in db_files:
                if reached_limit:
                    break
                sample_iterator = iter_nuplan_samples(
                    db_file,
                    future_horizon_s=future_horizon_s,
                    stride=stride,
                    transformer_factory=transformer_factory,
                )
                try:
                    for sample in sample_iterator:
                        result = generate_nuplan_route_result(sample, route_generator)
                        output_file.write(json.dumps(to_jsonable(result), sort_keys=True) + "\n")
                        count += 1
                        if max_samples is not None and count >= max_samples:
                            reached_limit = True
                            break
                finally:
                    sample_iterator.close()
        os.replace(temp_file.name, output_path)
    except Exception:
        try:
            os.remove(temp_file.name)
        except FileNotFoundError:
            pass
        raise
    return count


def validate_options(future_horizon_s, stride, max_samples):
    if future_horizon_s <= 0:
        raise NuPlanAdapterError("--nuplan_future_horizon_s must be positive")
    if stride < 1:
        raise NuPlanAdapterError("--nuplan_stride must be at least 1")
    if max_samples is not None and max_samples < 1:
        raise NuPlanAdapterError("--nuplan_max_samples must be at least 1 when provided")


def discover_db_files(input_path):
    path = Path(input_path)
    if path.is_file() and path.suffix == ".db":
        return [path]
    if path.is_dir():
        db_files = sorted(path.rglob("*.db"))
        if db_files:
            return db_files
    raise NuPlanAdapterError(f"No nuPlan .db files found at {input_path}")


def iter_nuplan_samples(db_file, future_horizon_s, stride, transformer_factory):
    try:
        with closing(sqlite3.connect(db_file)) as connection:
            connection.row_factory = sqlite3.Row
            anchors = list(query_anchor_rows(connection))
            for anchor_index, anchor in enumerate(anchors):
                if anchor_index % stride != 0:
                    continue
                sample = build_sample_from_anchor(
                    db_file=db_file,
                    connection=connection,
                    anchor=anchor,
                    future_horizon_s=future_horizon_s,
                    transformer_factory=transformer_factory,
                )
                if sample is not None:
                    yield sample
    except sqlite3.Error as exc:
        raise NuPlanAdapterError(f"Failed to read nuPlan database {db_file}: {exc}") from exc


def query_anchor_rows(connection):
    query = """
        SELECT
            lp.token AS lidar_pc_token,
            lp.timestamp AS timestamp_us,
            lp.scene_token AS scene_token,
            ep.token AS ego_pose_token,
            ep.log_token AS log_token,
            ep.timestamp AS ego_timestamp_us,
            ep.x AS x,
            ep.y AS y,
            ep.qw AS qw,
            ep.qx AS qx,
            ep.qy AS qy,
            ep.qz AS qz,
            ep.epsg AS epsg,
            scene.name AS scene_name,
            log.map_version AS map_name
        FROM lidar_pc AS lp
        INNER JOIN ego_pose AS ep
            ON lp.ego_pose_token = ep.token
        LEFT JOIN scene
            ON lp.scene_token = scene.token
        LEFT JOIN log
            ON ep.log_token = log.token
        ORDER BY lp.timestamp ASC
    """
    return connection.execute(query)


def build_sample_from_anchor(db_file, connection, anchor, future_horizon_s, transformer_factory):
    if anchor["epsg"] is None:
        raise NuPlanAdapterError(
            f"Missing EPSG code for lidar_pc {token_to_str(anchor['lidar_pc_token'])} in {db_file}"
        )

    future_rows = query_future_pose_rows(
        connection=connection,
        log_token=anchor["log_token"],
        anchor_ego_timestamp_us=anchor["ego_timestamp_us"],
        future_horizon_s=future_horizon_s,
    )
    projected_points = [(anchor["x"], anchor["y"])] + [(row["x"], row["y"]) for row in future_rows]
    if len(projected_points) < 2:
        return None

    heading_rad = yaw_from_quaternion(anchor["qw"], anchor["qx"], anchor["qy"], anchor["qz"])
    local_points = projected_points_to_local(
        projected_points,
        anchor_x=anchor["x"],
        anchor_y=anchor["y"],
        heading_rad=heading_rad,
    )
    lat, lon = projected_to_wgs84(
        anchor["x"],
        anchor["y"],
        epsg=anchor["epsg"],
        transformer_factory=transformer_factory,
    )

    lidar_pc_token = token_to_str(anchor["lidar_pc_token"])
    input_data = NuPlanInputData(
        pred_time={
            "lat": lat,
            "lon": lon,
            "heading": math.degrees(heading_rad),
        },
        fp={
            "relative_time": [index for index, _point in enumerate(local_points)],
            "local_lat": [point[0] for point in local_points],
            "local_lon": [point[1] for point in local_points],
        },
        sequence_id=f"{Path(db_file).stem}:{lidar_pc_token}",
    )

    return {
        "input_data": input_data,
        "metadata": {
            "sample_id": input_data.sequence_id,
            "dataset": "nuPlan",
            "db_file": str(db_file),
            "lidar_pc_token": lidar_pc_token,
            "timestamp_us": int(anchor["timestamp_us"]),
            "scene_token": token_to_str(anchor["scene_token"]),
            "scene_name": anchor["scene_name"],
            "map_name": anchor["map_name"],
            "epsg": int(anchor["epsg"]),
            "future_horizon_s": float(future_horizon_s),
            "future_points": len(projected_points),
        },
    }


def query_future_pose_rows(connection, log_token, anchor_ego_timestamp_us, future_horizon_s):
    horizon_end_us = int(anchor_ego_timestamp_us + future_horizon_s * 1e6)
    query = """
        SELECT
            x,
            y,
            timestamp
        FROM ego_pose
        WHERE log_token = ?
            AND timestamp > ?
            AND timestamp <= ?
        ORDER BY timestamp ASC
    """
    return list(connection.execute(query, (log_token, anchor_ego_timestamp_us, horizon_end_us)))


def generate_nuplan_route_result(sample, route_generator):
    metadata = sample["metadata"]
    input_data = sample["input_data"]
    try:
        (
            route_coords,
            route_properties,
            map_links,
            _map_retrieval_time,
            _route_generation_time,
        ) = route_generator.create_route(input_data)
        fallback = False
        error_type = None
    except (NoLinksInQueriedAreaError, NoRoutesFoundError) as exc:
        route_coords = route_generator.get_generic_route_coords()
        route_properties = route_generator.get_generic_route_properties()
        map_links = []
        fallback = True
        error_type = type(exc).__name__

    return {
        **metadata,
        "route_coords": route_coords,
        "route_properties": route_properties,
        "map_links": map_links,
        "fallback": fallback,
        "error_type": error_type,
    }


def projected_points_to_local(points, anchor_x, anchor_y, heading_rad):
    cos_heading = math.cos(heading_rad)
    sin_heading = math.sin(heading_rad)
    local_points = []
    for x, y in points:
        dx = x - anchor_x
        dy = y - anchor_y
        local_points.append(
            [
                cos_heading * dx - sin_heading * dy,
                sin_heading * dx + cos_heading * dy,
            ]
        )
    return local_points


def projected_to_wgs84(x, y, epsg, transformer_factory):
    transformer = transformer_factory(epsg)
    lon, lat = transformer.transform(x, y)
    return float(lat), float(lon)


def default_transformer_factory(epsg):
    try:
        from pyproj import Transformer
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing pyproj dependency for nuPlan coordinate conversion. Install public "
            "dependencies with `pip install -r requirements.txt`."
        ) from exc
    return Transformer.from_crs(f"EPSG:{int(epsg)}", "EPSG:4326", always_xy=True)


def cached_transformer_factory(transformer_factory):
    cache = {}

    def get_transformer(epsg):
        epsg = int(epsg)
        if epsg not in cache:
            cache[epsg] = transformer_factory(epsg)
        return cache[epsg]

    return get_transformer


def yaw_from_quaternion(qw, qx, qy, qz):
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def token_to_str(value):
    if value is None:
        return None
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, bytearray):
        return bytes(value).hex()
    return str(value)
