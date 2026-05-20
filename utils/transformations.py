
import math
import utm

import numpy as np
from shapely.geometry import Point

from geopy import Point as GeoPoint



def rotate(origin, angle, point):
    """
    Rotate a point around a given origin in UTM coordinates (m), with angle (degrees) in the camera frame of reference (x-axis is forward-facing, y-axis is facing left).
    """
    ox, oy = origin.x, origin.y
    px, py = point.x, point.y
    qx = ox + math.cos(np.deg2rad(angle)) * (px - ox) - math.sin(np.deg2rad(angle)) * (py - oy)
    qy = oy + math.sin(np.deg2rad(angle)) * (px - ox) + math.cos(np.deg2rad(angle)) * (py - oy)
    return Point(qx, qy)


def translate(point, translation):
    """
    Translate a point to the reference system of the ego_vehicle.

    :param: point, to be translated point in UTM coordinates (m).
    :param: translation,  should equal the inverse of the ego_vehicle in UTM coordinates (m).
    """
    return Point(
        point.x + translation[0],
        point.y + translation[1],
    )


def global_to_vehicle_coordinates(points:list, oxts_latlon, oxts_heading):
    """From global to vehicle coordinates.

    Args:
        points (list): List of points to be transformed in format [[lat, lon], [lat, lon], ...]
        oxts_latlon (list): [lat, lon] of the ego vehicle
        oxts_heading (float): Heading of the ego vehicle

    Returns:
        np.array: List of transformed points in vehicle coordinates
    """
    origin_x, origin_y, utm_zone_num, utm_zone_letter = utm.from_latlon(oxts_latlon[0], oxts_latlon[1])
    origin = Point(origin_x, origin_y)
    aft_origin = Point(0, 0)
    rot_pts = []

    pts = np.array(
        [
            utm.from_latlon(
                point[0],
                point[1],
                utm_zone_num,
                utm_zone_letter,
            )[:2]
            for point in points
        ]
    )

    inverse_origin = [-origin.x, -origin.y]

    # Homogeneous transformation -> translate, rotate
    for pt in pts:
        point = translate(Point(pt), inverse_origin)
        rot_pt = rotate(aft_origin, oxts_heading, point)
        rot_pts.append([rot_pt.x, rot_pt.y])

    return np.array(rot_pts)


def transform_to_vehicle_coordinates(vehicle_data, link):
    """Transforms the link points from lat lon to the vehicle coordinate system."""
    ego_vehicle_lat = vehicle_data["ego_vehicle_lat"]
    ego_vehicle_lon = vehicle_data["ego_vehicle_lon"]
    ego_vehicle_yaw = vehicle_data["ego_vehicle_yaw"]
    origin_x, origin_y, utm_zone_num, utm_zone_letter = utm.from_latlon(ego_vehicle_lat, ego_vehicle_lon)
    origin = Point(origin_x, origin_y)
    aft_origin = Point(0, 0)
    rot_pts = []

    pts = np.array(
        [
            utm.from_latlon(
                geopoint.latitude,
                geopoint.longitude,
                utm_zone_num,
                utm_zone_letter,
            )[:2]
            for geopoint in link.get_geopoints()
        ]
    )

    inverse_origin = [-origin.x, -origin.y]

    # Homogeneous transformation -> translate, rotate
    for pt in pts:
        point = translate(Point(pt), inverse_origin)
        rot_pt = rotate(aft_origin, ego_vehicle_yaw, point)
        rot_pts.append([rot_pt.x, rot_pt.y])
    return rot_pts

def convert_geopoint_to_vehicle_coords(geopoint: GeoPoint, vehicle_data: dict):
    ego_vehicle_lat = vehicle_data["ego_vehicle_lat"]
    ego_vehicle_lon = vehicle_data["ego_vehicle_lon"]
    ego_vehicle_yaw = vehicle_data["ego_vehicle_yaw"]
    origin_x, origin_y, utm_zone_num, utm_zone_letter = utm.from_latlon(ego_vehicle_lat, ego_vehicle_lon)
    origin = Point(origin_x, origin_y)
    aft_origin = Point(0, 0)
    pt = utm.from_latlon(
        geopoint.latitude,
        geopoint.longitude,
        utm_zone_num,
        utm_zone_letter,
    )[:2]

    inverse_origin = [-origin.x, -origin.y]

    # Homogeneous transformation -> translate, rotate
    point = translate(Point(pt), inverse_origin)
    rot_pt = rotate(aft_origin, ego_vehicle_yaw, point)
    return [rot_pt.x, rot_pt.y]


def transform_to_origin_coords(link):
    vehicle_data = {
        "ego_vehicle_lat": 0,
        "ego_vehicle_lon": 0,
        "ego_vehicle_yaw": 0,
    }
    return transform_to_vehicle_coordinates(vehicle_data, link)
