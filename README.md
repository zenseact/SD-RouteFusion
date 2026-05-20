# SD-RouteFusion Route Generation Toolkit

This repository contains the SD-map route-prior generation component used by
SD-RouteFusion. It generates route candidates from OpenStreetMap (OSM) around
an ego vehicle pose, selects the candidate that best matches the future ego
trajectory, and writes the resulting SD-route prior for ego-trajectory
prediction experiments.

This is not the full SD-RouteFusion neural network. It is the dataset
preprocessing tool that creates the route input used by route-conditioned
ego-trajectory prediction models.

## Generic JSONL Interface

`GenericJSONL` is the recommended public interface for arbitrary datasets. Use
it when you can export each prediction anchor as an ego pose, heading, and
future ego trajectory in WGS84 latitude/longitude.

The public `GenericJSONL` entrypoint expects one JSON object per line:

```json
{"sample_id":"demo_001","ego":{"lat":57.70887,"lon":11.97456,"heading":0.0},"future_path":[[57.70920,11.97458],[57.71060,11.97462]]}
```

Required fields:

- `sample_id`: unique sample identifier.
- `ego.lat`, `ego.lon`: ego vehicle latitude and longitude at prediction time.
- `ego.heading`: ego heading in degrees, using the same convention as the
  dataset pose source.
- `future_path`: at least two future ego trajectory points. The canonical form
  is `[lat, lon]` arrays.

For compatibility with earlier snapshots, future points may also be objects:

```json
{"future_path":[{"lat":57.70920,"lon":11.97458},{"lat":57.71060,"lon":11.97462}]}
```

Optional fields:

- `metadata`: a JSON object copied to the output as `input_metadata`. Use this
  for dataset split, log id, timestamp, camera path, or other sample metadata
  that should survive preprocessing.

Metadata example:

```json
{"sample_id":"demo_002","ego":{"lat":57.70887,"lon":11.97456,"heading":0.0},"future_path":[[57.70920,11.97458],[57.71060,11.97462]],"metadata":{"split":"val","log_id":"demo_log","timestamp_us":123456}}
```

## Output Schema

The output is also JSONL. Each row contains:

- `sample_id`: copied from the input.
- `route_coords`: generated SD-route coordinates as `[x_m, y_m]` points in the
  local ego-vehicle frame, in meters. The x-axis points forward and the y-axis
  points left.
- `route_properties`: route metadata. `valid_route` is the required validity
  flag. Successful rows include OSM-derived fields such as `road_class`,
  `num_lanes`, `speed_limit`, `crossings`, `num_links`, `num_branches`,
  `has_bridge`, `is_tunnel`, and `is_highway`.
- `map_links`: queried OSM edge identifiers in `u-v` string form.
- `fallback`: `true` when no usable OSM route was found and a straight-line
  fallback route was emitted.
- `error_type`: route-generation failure type when `fallback` is `true`;
  otherwise `null`.
- `input_metadata`: copied from input `metadata` when provided.

## Installation

Python 3.11 is recommended.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Reproducibility

OSM data is queried live through OSMnx unless `--cache_location` points to an
OSMnx cache directory. Live OpenStreetMap data can change over time, so repeated
runs without a cache may not produce bit-identical route candidates.

For reproducible preprocessing runs, pass a cache directory and preserve it with
the generated outputs:

```bash
python main.py \
  --dataset=GenericJSONL \
  --input=examples/sample_input.jsonl \
  --output=examples/sample_output.jsonl \
  --map_query_max_l1=1000 \
  --cache_location=osm_cache
```

For exact paper reproduction, also preserve the dependency versions in
`requirements.txt`.

## Runnable Example

Generic JSONL:

```bash
python main.py \
  --dataset=GenericJSONL \
  --input=examples/sample_input.jsonl \
  --output=examples/sample_output.jsonl \
  --map_query_max_l1=1000
```

The command writes JSONL results to `examples/sample_output.jsonl`. Re-running
the command overwrites that output file.

Optional nuPlan adapter:

```bash
python main.py \
  --dataset=nuPlan \
  --input=/path/to/nuplan/dbs \
  --output=outputs/nuplan_routes.jsonl \
  --map_query_max_l1=1000 \
  --nuplan_future_horizon_s=8.0
```

For `nuPlan`, `--input` may point to a single official `.db` log or to a
directory containing `.db` logs. The adapter reads `lidar_pc`, `ego_pose`,
`scene`, and `log` records directly from SQLite and does not require a nuPlan
HD-map root, sensor root, or scenario-builder configuration.
It writes one JSONL row per eligible `lidar_pc` anchor.

Additional nuPlan controls:

- `--nuplan_future_horizon_s`: future ego-trajectory horizon in seconds
  (`8.0` by default).
- `--nuplan_stride`: process every Nth `lidar_pc` anchor (`1` by default).
- `--nuplan_max_samples`: optional cap on the number of JSONL rows written.

Each nuPlan output row includes the common route fields plus metadata such as
`db_file`, `lidar_pc_token`, `timestamp_us`, `scene_name`, `map_name`, `epsg`,
`future_horizon_s`, and `future_points`.

## Dataset Adapters

The public workflows are:

- `GenericJSONL`, the primary dataset-agnostic workflow for any dataset that
  provides ego pose, heading, and future ego trajectory points.
- `nuPlan`, an optional convenience adapter for official nuPlan SQLite `.db`
  logs.

Legacy/internal dataset adapter code used during development is quarantined
under `legacy/` and is not part of the public submission workflow.

## License and Citation

This repository is released under the MIT License. If retained, vendored or
third-party directories keep their own license files and are not relicensed by
the top-level license.

If you use this software, cite the associated SD-RouteFusion paper using the
metadata in `CITATION.cff`.

## Notes

- Route-generation failures caused by missing OSM links or disconnected route
  candidates are represented in the output with `fallback: true`.
- The route generator emits SD-map route priors only; it does not train or run
  an ego-trajectory prediction model.
