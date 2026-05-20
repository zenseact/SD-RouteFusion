import math
import pickle
import h5py
import sys
import traceback

import numpy as np
from shapely import LineString

import zcd.load_dataset as load_dataset 

from osm_wrapper import OSMWrapper
from errors import DataValidityError, NoRoutesFoundError, ShortGroundTruthError, NoLinksInQueriedAreaError, FuturePathSimplificationError
from route_generation import InputData, RouteGenerator
from utils.transformations import global_to_vehicle_coordinates


def load_recursively(hdf_obj, keys_to_extract: list = []):
    data = {}
    # Extract attributes
    for name, attr_data in hdf_obj.attrs.items():
        if not keys_to_extract or name in keys_to_extract:
            data[name] = attr_data
    for key in hdf_obj.keys():
        # print(f"Key: {key}")  # Print the key
        if isinstance(hdf_obj[key], h5py.Group):
            # Extract everything in the group
            data[key] = load_recursively(hdf_obj[key], keys_to_extract)
            if not data[key]:
                data.pop(key)
        if isinstance(hdf_obj[key], h5py.Dataset):
            if not keys_to_extract or key in keys_to_extract:
                save_data = hdf_obj[key][()]
                if isinstance(save_data, np.ndarray):
                    data[key] = save_data.tolist()
                else:
                    data[key] = save_data

    return data

load_dataset.load_recursively = load_recursively

def writer(filename, data):
    with open(filename + ".pkl", "wb") as handle:
        pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        
def get_future_coords_from_lcm_data(lcm_data, middle_frame, data_source):
    """
    Used for either GT generation if called with data_source="oxts" or FP generation if called with data_source="lcm".
    """
    
    if data_source == "oxts":
        key_lat, key_lon, key_heading = "oxts_lat", "oxts_lon", "oxts_heading"
    elif data_source == "lcm":
        key_lat, key_lon, key_heading = "lcm_gnss_lat", "lcm_gnss_lon", "lcm_gnss_heading"
        
    # Extract and prepare ground truth data using only oxts-based keys
    keys = [key_lat, key_lon, "lcm_egomotion_timestamp"]
    future_data = {key: lcm_data[key][middle_frame:] for key in keys}

    # Calculate relative timestamps
    relative_time = [
        ts - future_data["lcm_egomotion_timestamp"][0] for ts in future_data["lcm_egomotion_timestamp"]
    ]

    # Use the first frame (after slicing) as the ego reference point
    ego_ref_lat = lcm_data[key_lat][middle_frame]
    ego_ref_lon = lcm_data[key_lon][middle_frame]
    ego_ref_heading = lcm_data[key_heading][middle_frame]
    
    pred_time_data = {
        "lat": ego_ref_lat,
        "lon": ego_ref_lon,
        "heading": ego_ref_heading,
    }
    
    # Convert global ground truth coordinates to the ego vehicle's local frame
    relative_coords = global_to_vehicle_coordinates(
        np.column_stack([future_data[key_lat], future_data[key_lon]]),
        [ego_ref_lat, ego_ref_lon],
        ego_ref_heading,
    )

    # Add processed ground truth data to out_dict
    coords_dict = {
        "relative_time": relative_time,
        "local_lat": relative_coords[:, 0].tolist(),
        "local_lon": relative_coords[:, 1].tolist(),
    }
    return coords_dict, pred_time_data

def calculate_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the straight-line distance between two points in 2D space.
    """
    return math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)

def simplify_fp(fp, num_fp_waypoints):
    """
    Simplify the future path to the desired number of waypoints.
    
    Args:
        fp (dict): Dictionary containing relative_time, local_lat, and local_lon keys.
        num_fp_waypoints (int): Desired number of waypoints (None to return full dictionary). 1 for just the end point.
        
    Returns:
        dict: Simplified dictionary with specified number of waypoints.
    """
    if num_fp_waypoints is None:
        return fp
    assert num_fp_waypoints >= 1, "Number of waypoints must be at least 1"

    # Extract data from the dictionary
    relative_time = fp['relative_time']
    local_lat = fp['local_lat']
    local_lon = fp['local_lon']
    
    # Determine point A (first point at most 200 meters from start, as-crow-flies)
    start_lat, start_lon = local_lat[0], local_lon[0]
    point_A_index = 0
    for i in range(1, len(local_lat)):
        distance = calculate_distance(start_lat, start_lon, local_lat[i], local_lon[i])
        if distance >= 200:
            point_A_index = i
            break
    else:
        # If no point exceeds 200m, use the last point
        point_A_index = len(local_lat) - 1
    
    waypoints_indices = [0]  # Start point
    step = point_A_index / num_fp_waypoints
    for i in range(1, num_fp_waypoints):
        waypoints_indices.append(int(round(i * step)))
    waypoints_indices.append(point_A_index)  # Point A
    
    # Create the simplified dictionary
    simplified_fp = {
        'relative_time': [relative_time[i] for i in waypoints_indices],
        'local_lat': [local_lat[i] for i in waypoints_indices],
        'local_lon': [local_lon[i] for i in waypoints_indices],
    }
    
    return simplified_fp
    
        
def retrieve_gt_fp_kinematics(scene_data, location_source, num_fp_waypoints):
    out_dict = {}
    lcm_data = scene_data["lcm_data"]
    
    ### 1. GROUND TRUTH ###
    if min(lcm_data["oxts_valid"]) != 1:
        raise DataValidityError()  # this failure would mean bad gt
    
    len_oxts = len(lcm_data["oxts_heading"])
    middle_frame = int(len_oxts / 2)
    
    # Populate ground truth, irrespective of the location source (always use oxts)
    gt_coords, pred_time_oxts = get_future_coords_from_lcm_data(lcm_data, middle_frame, "oxts")
    out_dict["gt"] = gt_coords
    
    # check if gt is too short
    gt_coords_linestring = LineString(np.column_stack([gt_coords["local_lat"], gt_coords["local_lon"]]))
    if gt_coords_linestring.length < 200:
        raise ShortGroundTruthError()
    
    ### 2. FUTURE PATH ###
    if location_source == "oxts":
        fp = out_dict["gt"]
        out_dict['pred_time'] = pred_time_oxts
    elif location_source == "lcm":
        fp_coords, pred_time_lcm = get_future_coords_from_lcm_data(lcm_data, middle_frame, "lcm")
        fp = fp_coords
        out_dict['pred_time'] = pred_time_lcm
        
    # populate fp, taking into account the number of waypoints
    out_dict["fp"] = simplify_fp(fp, num_fp_waypoints)
    

    ### 2. KINEMATIC DATA ###
    # This depends on the location source
    
    quality_keys = [key for key in lcm_data.keys() if key.startswith("lcm") and "quality" in key]
    quality_dict = {key: value for key, value in lcm_data.items() if key in quality_keys}
    if not all([min(value) == 3 for value in quality_dict.values()]):
        raise DataValidityError()  # lcm data not valid
    
    pred_time_lcm = {
        "lat": lcm_data["lcm_gnss_lat"][middle_frame],
        "lon": lcm_data["lcm_gnss_lon"][middle_frame],
        "heading": lcm_data["lcm_gnss_heading"][middle_frame],
    }
    out_dict['pred_time'] = pred_time_lcm if location_source == "lcm" else pred_time_oxts # pred time used later to 
    
    # proceed if data is valid
    logs_per_second = 50
    step_size = 5
    observation_window = 3  # seconds
    start_index = int(middle_frame - observation_window * logs_per_second)
    
    
    # Define kinematic keys based on the location source
    kinematic_keys = [
        "lcm_egomotion_timestamp",
        "lcm_lat_acceleration",
        "lcm_lat_velocity",
        "lcm_lon_acceleration",
        "lcm_lon_velocity",
        "lcm_yaw_rate",
    ]

    # Add location-specific keys with consistent naming for lat, lon, and heading
    if location_source == "oxts":
        kinematic_keys.extend(["oxts_lat", "oxts_lon", "oxts_heading"])
        lat_key, lon_key, heading_key = "oxts_lat", "oxts_lon", "oxts_heading" # to rename later
    elif location_source == "lcm":
        kinematic_keys.extend(["lcm_gnss_lat", "lcm_gnss_lon", "lcm_gnss_heading"])
        lat_key, lon_key, heading_key = "lcm_gnss_lat", "lcm_gnss_lon", "lcm_gnss_heading" # to rename later

    # Extract and process kinematic data
    kinematic_data = {
        key: value[start_index : middle_frame + 1 : step_size]
        for key, value in lcm_data.items()
        if key in kinematic_keys
    }

    # Standardize keys for lat, lon, and heading
    kinematic_data["lat"] = kinematic_data.pop(lat_key)
    kinematic_data["lon"] = kinematic_data.pop(lon_key)
    kinematic_data["heading"] = kinematic_data.pop(heading_key)

    
    ### MAKE KINEMATICS RELATIVE TO THE EGO VEHICLE ###
    # make time relative
    kinematic_data["relative_time"] = [
        i - kinematic_data["lcm_egomotion_timestamp"][-1] for i in kinematic_data["lcm_egomotion_timestamp"]
    ]
    kinematic_data.pop("lcm_egomotion_timestamp")

    # move kinematics position to local coords
    kin_pos = np.vstack([kinematic_data["lat"], kinematic_data["lon"]]).T
    kin_pos_local_coords = global_to_vehicle_coordinates(
        kin_pos,
        [out_dict["pred_time"]["lat"], out_dict["pred_time"]["lon"]],
        out_dict["pred_time"]["heading"],
    )
    kinematic_data["lat"] = kin_pos_local_coords[:, 0].tolist()
    kinematic_data["lon"] = kin_pos_local_coords[:, 1].tolist()

    # make heading relative to the ego vehicle
    kinematic_data["heading"] = [
        heading - out_dict["pred_time"]["heading"] for heading in kinematic_data["heading"]
    ]

    out_dict["kinematics"] = kinematic_data
    
    
    ### 4. ADD META DATA ###
    meta_keys = [
        "FC_ant_tlc_data_image_raw_path_kw",
        "frame_timestamp_date",
        "sequence_id",
        "suite_id",
        "vehicle",
        "route",
    ]
    for key in meta_keys:
        out_dict[key] = scene_data[key]
        
    return out_dict

def get_generic_outputs():
    route_coords = RouteGenerator.get_generic_route_coords()
    route_properties = RouteGenerator.get_generic_route_properties()
    map_links = []
    return route_coords, route_properties, map_links


def worker_zcd(worker_data, location_source, map_query_max_l1, num_fp_waypoints, output, existing_files, cache_location, debug):

    file_name, worker_id = worker_data
    # print(f"worker {worker_id} started processing {file_name}")
        
    wrapper = OSMWrapper(cache_location)
    route_generator = RouteGenerator(wrapper, map_query_max_l1)

    df = load_dataset.DatasetFile(file_name)
    group_information = df.get_file_information()

    count = 0
    already_exists = 0
    no_lcm_data = 0
    incomplete_lcm_data = 0
    not_valid = 0
    short_gt = 0
    no_links_in_area = 0
    no_routes_close_by = 0
    other_route_errors = 0
    map_retrieval_time = 0
    route_generation_time = 0
    
    for data_point in group_information["groups"]:
        if data_point + ".pkl" in existing_files:
            # print(f"worker {worker_id} skipped {data_point}, already exists")
            already_exists += 1
            continue
        
        data = df.load_sample(data_point)
        # remove root group
        data = dict(ele for sub in data.values() for ele in sub.items())
        if "lcm_data" not in data:
            no_lcm_data += 1
            continue
        if "lcm_lat_acceleration" not in data["lcm_data"]:
            print(f"worker {worker_id} skipped {data['sequence_id']}, lcm data contains only oxts data")
            incomplete_lcm_data += 1
            continue
        
        try:
            data_out = retrieve_gt_fp_kinematics(data, location_source, num_fp_waypoints)
        except DataValidityError:
            not_valid += 1
            continue
        except ShortGroundTruthError:
            short_gt += 1
            continue
        except Exception as e:
            other_route_errors += 1
            # if debug:
            other_route_errors += 1
            print("UNEXPECTED ERROR - Retrieval", e, data["sequence_id"], "SKIPPING!!!", sep="\n")    
            # print traceback
            traceback.print_exc()
            sys.stdout.flush()            
            continue

        input_data = InputData(data_out["pred_time"], data_out["fp"], data_out["sequence_id"])
        try:
            route_coords, route_properties, map_links, map_retrieval_time_cur, route_generation_time_cur = route_generator.create_route(input_data)
            map_retrieval_time += map_retrieval_time_cur
            route_generation_time += route_generation_time_cur
        except NoRoutesFoundError:
            no_routes_close_by += 1
            route_coords, route_properties, map_links = get_generic_outputs()
        except NoLinksInQueriedAreaError:
            no_links_in_area += 1
            route_coords, route_properties, map_links = get_generic_outputs()
        except Exception as e:
            other_route_errors += 1
            # if debug:
            print("UNEXPECTED ERROR - Route generation", e, data["sequence_id"], "SKIPPING!!!", sep="\n")    
            # print traceback
            traceback.print_exc()
            sys.stdout.flush()            
            continue
        
        data_out["route_coords"] = route_coords
        data_out["route_properties"] = route_properties
        data_out["map_links"] = map_links
        
        count += 1
        writer(f'{output + "/" + data_out["sequence_id"]}', data_out)
        if debug:
            break

    del wrapper
    return count, no_lcm_data, incomplete_lcm_data, not_valid, already_exists, short_gt, no_routes_close_by, no_links_in_area, other_route_errors, map_retrieval_time, route_generation_time