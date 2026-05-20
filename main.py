import argparse
import os


def build_parser():
    parser = argparse.ArgumentParser(
        description="Generate SD-map route priors for ego-trajectory prediction datasets."
    )
    parser.add_argument("--input", type=str, required=True, help="Input dataset path.")
    parser.add_argument("--output", type=str, required=True, help="Output path.")
    parser.add_argument(
        "--dataset",
        type=str,
        required=True,
        choices=["GenericJSONL", "nuPlan"],
        help="Dataset adapter to use.",
    )
    parser.add_argument(
        "--map_query_max_l1",
        type=int,
        required=True,
        help="Retain OSM nodes within this distance from the vehicle when querying the map.",
    )
    parser.add_argument(
        "--cache_location",
        type=str,
        default="None",
        help='OSMnx cache location. Use "None" to disable the on-disk cache.',
    )
    parser.add_argument(
        "--nuplan_future_horizon_s",
        type=float,
        default=8.0,
        help="nuPlan future ego-trajectory horizon in seconds.",
    )
    parser.add_argument(
        "--nuplan_stride",
        type=int,
        default=1,
        help="Process every Nth nuPlan lidar_pc anchor.",
    )
    parser.add_argument(
        "--nuplan_max_samples",
        type=int,
        default=None,
        help="Optional maximum number of nuPlan output samples.",
    )
    return parser


def prepare_cache(cache_location):
    if cache_location is not None and cache_location != "None":
        os.makedirs(cache_location, exist_ok=True)
    return cache_location


def build_route_generator(cache_location, map_query_max_l1):
    try:
        from osm_wrapper import OSMWrapper
        from route_generation import RouteGenerator
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing route-generation dependency. Install public dependencies with "
            "`pip install -r requirements.txt`."
        ) from exc

    wrapper = OSMWrapper(cache_location)
    return RouteGenerator(wrapper, map_query_max_l1)


def prepare_output_file(output_path):
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)


def run_generic_jsonl(args, route_generator_factory=None, coordinate_converter=None):
    def default_route_generator_factory(cache_location, map_query_max_l1):
        return build_route_generator(cache_location, map_query_max_l1)

    from generic_jsonl import process_jsonl

    route_generator_factory = route_generator_factory or default_route_generator_factory
    cache_location = prepare_cache(args.cache_location)

    prepare_output_file(args.output)

    route_generator = route_generator_factory(cache_location, args.map_query_max_l1)
    count = process_jsonl(
        input_path=args.input,
        output_path=args.output,
        route_generator=route_generator,
        coordinate_converter=coordinate_converter,
    )
    print(f"Wrote {count} route result(s) to {args.output}")


def run_nuplan(args, route_generator_factory=None, transformer_factory=None):
    def default_route_generator_factory(cache_location, map_query_max_l1):
        return build_route_generator(cache_location, map_query_max_l1)

    from nuplan_adapter import process_nuplan

    route_generator_factory = route_generator_factory or default_route_generator_factory
    cache_location = prepare_cache(args.cache_location)
    prepare_output_file(args.output)

    route_generator = route_generator_factory(cache_location, args.map_query_max_l1)
    count = process_nuplan(
        input_path=args.input,
        output_path=args.output,
        route_generator=route_generator,
        future_horizon_s=args.nuplan_future_horizon_s,
        stride=args.nuplan_stride,
        max_samples=args.nuplan_max_samples,
        transformer_factory=transformer_factory,
    )
    print(f"Wrote {count} nuPlan route result(s) to {args.output}")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        if args.dataset == "GenericJSONL":
            run_generic_jsonl(args)
        elif args.dataset == "nuPlan":
            run_nuplan(args)
    except Exception as exc:
        parser.exit(status=1, message=f"error: {exc}\n")


if __name__ == "__main__":
    main()
