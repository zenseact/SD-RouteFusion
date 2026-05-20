import json
import math
import numbers
import os
from dataclasses import dataclass
import tempfile

from errors import NoLinksInQueriedAreaError, NoRoutesFoundError


class SchemaValidationError(ValueError):
    pass


@dataclass
class GenericInputData:
    pred_time: dict
    fp: dict
    sequence_id: str
    metadata: dict = None


def process_jsonl(input_path, output_path, route_generator, coordinate_converter=None):
    count = 0
    output_dir = os.path.dirname(output_path) or "."
    temp_file = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=output_dir,
        delete=False,
    )
    try:
        with open(input_path, "r", encoding="utf-8") as input_file, temp_file as output_file:
            for line_number, line in enumerate(input_file, start=1):
                if not line.strip():
                    continue
                try:
                    sample = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise SchemaValidationError(f"Line {line_number}: invalid JSON: {exc}") from exc

                input_data = sample_to_input_data(
                    sample,
                    line_number=line_number,
                    coordinate_converter=coordinate_converter,
                )
                result = generate_route_result(input_data, route_generator)
                output_file.write(json.dumps(to_jsonable(result), sort_keys=True) + "\n")
                count += 1
        os.replace(temp_file.name, output_path)
    except Exception:
        try:
            os.remove(temp_file.name)
        except FileNotFoundError:
            pass
        raise
    return count


def sample_to_input_data(sample, line_number=None, coordinate_converter=None):
    prefix = f"Line {line_number}: " if line_number is not None else ""
    validate_sample(sample, prefix)

    ego = sample["ego"]
    future_path = normalize_future_path(sample["future_path"], prefix)
    pred_time = {
        "lat": ego["lat"],
        "lon": ego["lon"],
        "heading": ego["heading"],
    }

    global_future_points = future_path
    converter = coordinate_converter or default_coordinate_converter
    local_future_points = converter(
        global_future_points,
        [pred_time["lat"], pred_time["lon"]],
        pred_time["heading"],
    )

    local_lat = []
    local_lon = []
    for point in local_future_points:
        local_lat.append(float(point[0]))
        local_lon.append(float(point[1]))

    return GenericInputData(
        pred_time=pred_time,
        fp={
            "relative_time": list(range(len(local_lat))),
            "local_lat": local_lat,
            "local_lon": local_lon,
        },
        sequence_id=str(sample["sample_id"]),
        metadata=sample.get("metadata"),
    )


def validate_sample(sample, prefix=""):
    if not isinstance(sample, dict):
        raise SchemaValidationError(f"{prefix}sample must be a JSON object")

    require_key(sample, "sample_id", prefix)
    require_key(sample, "ego", prefix)
    require_key(sample, "future_path", prefix)

    ego = sample["ego"]
    if not isinstance(ego, dict):
        raise SchemaValidationError(f"{prefix}ego must be an object")
    for key in ["lat", "lon", "heading"]:
        require_key(ego, key, prefix + "ego.")
        require_number(ego[key], prefix + f"ego.{key}")

    future_path = sample["future_path"]
    if not isinstance(future_path, list):
        raise SchemaValidationError(f"{prefix}future_path must be a list")
    if len(future_path) < 2:
        raise SchemaValidationError(f"{prefix}future_path must contain at least two points")
    normalize_future_path(future_path, prefix)

    if "metadata" in sample and not isinstance(sample["metadata"], dict):
        raise SchemaValidationError(f"{prefix}metadata must be an object")


def normalize_future_path(future_path, prefix=""):
    normalized_points = []
    for point_index, point in enumerate(future_path):
        name = f"{prefix}future_path[{point_index}]"
        if isinstance(point, dict):
            for key in ["lat", "lon"]:
                require_key(point, key, name + ".")
                require_number(point[key], name + f".{key}")
            normalized_points.append([float(point["lat"]), float(point["lon"])])
            continue

        if isinstance(point, list):
            if len(point) != 2:
                raise SchemaValidationError(f"{name} must contain exactly two values")
            require_number(point[0], name + "[0]")
            require_number(point[1], name + "[1]")
            normalized_points.append([float(point[0]), float(point[1])])
            continue

        raise SchemaValidationError(
            f"{name} must be a [lat, lon] array or an object with lat and lon"
        )
    return normalized_points


def require_key(data, key, prefix):
    if key not in data:
        raise SchemaValidationError(f"{prefix}missing required key: {key}")


def require_number(value, name):
    if not isinstance(value, numbers.Real) or isinstance(value, bool):
        raise SchemaValidationError(f"{name} must be a number")
    if not math.isfinite(float(value)):
        raise SchemaValidationError(f"{name} must be finite")


def default_coordinate_converter(points, ego_latlon, ego_heading):
    from utils.transformations import global_to_vehicle_coordinates

    return global_to_vehicle_coordinates(points, ego_latlon, ego_heading)


def generate_route_result(input_data, route_generator):
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

    result = {
        "sample_id": input_data.sequence_id,
        "route_coords": route_coords,
        "route_properties": route_properties,
        "map_links": map_links,
        "fallback": fallback,
        "error_type": error_type,
    }
    if input_data.metadata is not None:
        result["input_metadata"] = input_data.metadata
    return result


def to_jsonable(value):
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    if hasattr(value, "item"):
        try:
            return to_jsonable(value.item())
        except ValueError:
            pass
    if isinstance(value, numbers.Real) and not isinstance(value, bool):
        float_value = float(value)
        if not math.isfinite(float_value):
            return None
        if isinstance(value, numbers.Integral):
            return int(value)
        return float_value
    return value
