import time

from dataclasses import dataclass
from typing import Tuple
import shapely
from shapely import reverse
from shapely.ops import split, linemerge, substring

import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

from geopy import Point as GeoPoint


from osm_wrapper import Link
from utils.enums import Direction, MapObjectId
from utils.transformations import convert_geopoint_to_vehicle_coords, transform_to_origin_coords, transform_to_vehicle_coordinates
from osm_wrapper import OSMWrapper

from errors import NoLinksInQueriedAreaError, NoRoutesFoundError

def link_id_object(string_id):
    return MapObjectId(int(string_id.split("-")[0]), int(string_id.split("-")[1]))


def get_link_by_id(link_id, wrapper):
    return wrapper.get_sd_object_by_id(link_id_object(link_id))[0]


class Node:
    def __init__(self, node_id, start_point=False):
        self.connected_nodes = {}
        self.node_id = node_id
        self.start_point = start_point

    def __str__(self):
        return f"Node {self.node_id}"

    def __repr__(self):
        return f"Node {self.node_id}"

    def add_connection_from_map(self, node, wrapper, vehicle_data):
        current_link_obj, reversed = get_link_connecting_nodes(self, node, wrapper)
        if current_link_obj is None:
            print(f"Could not find link connecting {self.node_id} and {node.node_id}")
            return
        link_in_local_coords = transform_to_vehicle_coordinates(vehicle_data, current_link_obj)
        geometry_connection = LineString(link_in_local_coords)
        if reversed:
            geometry_connection = reverse(geometry_connection)

        self.connected_nodes[node.node_id] = geometry_connection

    def add_custom_connection(self, node, geometry_connection, vehicle_data, wrapper):
        # local_node_coords = transform_to_vehicle_coordinates(vehicle_data, node)
        # o = self.get_relative_coords(vehicle_data, wrapper)
        self.connected_nodes[node.node_id] = geometry_connection

    def get_connections(self):
        return self.connected_nodes

    def remove_connection(self, node_id):
        self.connected_nodes.pop(node_id)

    def get_relative_coords(self):
        node_coords = []
        for node_id, connection in self.connected_nodes.items():
            # check that the first point of the connection is the same for all outgoing connections
            node_coords.append(connection.coords[0])
        assert (
            len(set(node_coords)) == 1
        ), f"Node {self.node_id} has different relative coordinates for its connections."
        return node_coords[0]


class Tree:
    def __init__(self, links, wrapper, vehicle_data):
        self.links = links
        self.wrapper = wrapper
        self.nodes: list[Node] = []
        self.vehicle_data = vehicle_data
        for link_id in self.links:
            id_1 = link_id.split("-")[0]
            id_2 = link_id.split("-")[1]
            if id_1 == id_2:
                print(f"Link {link_id} is a loop, skipping.")
                continue
            node_1 = self.get_node(id_1)
            node_2 = self.get_node(id_2)
            if not node_1:
                node_1 = Node(id_1)
                self.nodes.append(node_1)
            if not node_2:
                node_2 = Node(id_2)
                self.nodes.append(node_2)
            # change later to see if bidirectional
            node_1.add_connection_from_map(node_2, self.wrapper, self.vehicle_data)
            node_2.add_connection_from_map(node_1, self.wrapper, self.vehicle_data)

    def get_node(self, node_id) -> Node:
        node_list = [node for node in self.nodes if node.node_id == node_id]
        if not node_list:
            return None
        return node_list[0]

    def get_start_nodes(self):
        return [node for node in self.nodes if node.start_point == True]

    def insert_start_points(self, max_distance, iter):
        """
        Find connections (LineString) where any point is closer than max_distance to the ego vehicle, call it relevant_connections.
        Find the closest point on the link to the ego vehicle (so under max_distance), call it point X.
        For each relevant_connection, break the link at point X and insert a node at that point.
        Reassign the connections of the two broken links to the new node.

        """
        ego_in_local_coords = Point([0.0, 0.0])
        connections_to_remove = []
        connections_to_add: list[tuple[Node, Node, Node, LineString, LineString]] = []
        inserted_node_id = 0
        for node in self.nodes:
            for next_node_id, connection in node.get_connections().items():
                # connection is always from node to next_node
                if (
                    set([node.node_id, next_node_id]) not in connections_to_remove
                    and connection.distance(ego_in_local_coords) < max_distance
                ):  # not yet broken and relevant
                    closest_point = nearest_points(connection, ego_in_local_coords)[0]
                    buff = closest_point.buffer(
                        0.01
                    )  # https://stackoverflow.com/questions/50194077/shapely-unable-to-split-line-on-point-due-to-precision-issues
                    try:
                        first_seg, buff_seg, last_seg = split(connection, buff).geoms
                    except ValueError:
                        # continue
                        # print(f"Could not split connection {connection} at point {closest_point}")
                        # this means that the closest point is at the start or end of the line
                        connection_start = connection.interpolate(0)
                        connection_end = connection.interpolate(connection.length)
                        if connection_start.distance(closest_point) < connection_end.distance(closest_point):
                            # closest point is at the start
                            node.start_point = True
                        else:
                            # closest point is at the end
                            next_node = self.get_node(next_node_id)
                            next_node.start_point = True
                        continue
                    connections_to_remove.append(set([node.node_id, next_node_id]))
                    # make sure the linestring is connected by changing the last point of the first segment to the first point of the last segment
                    new_first_seg = list(first_seg.coords)[:-1]
                    new_first_seg.append(list(last_seg.coords)[0])
                    first_seg = LineString(new_first_seg)
                    # create new node
                    new_node = Node(f"inserted_node_{inserted_node_id}", start_point=True)
                    inserted_node_id += 1
                    # add new node to the tree
                    # add new connections
                    next_node = self.get_node(next_node_id)
                    connections_to_add.append([node, new_node, next_node, first_seg, last_seg])

        # remove the connections that were broken
        for connection in connections_to_remove:
            node_1 = self.get_node(list(connection)[0])
            node_2 = self.get_node(list(connection)[1])
            node_1.remove_connection(node_2.node_id)
            node_2.remove_connection(node_1.node_id)

        for node, new_node, next_node, first_seg, last_seg in connections_to_add:
            self.nodes.append(new_node)
            node.add_custom_connection(new_node, first_seg, self.vehicle_data, self.wrapper)
            new_node.add_custom_connection(node, reverse(first_seg), self.vehicle_data, self.wrapper)
            new_node.add_custom_connection(next_node, last_seg, self.vehicle_data, self.wrapper)
            next_node.add_custom_connection(new_node, reverse(last_seg), self.vehicle_data, self.wrapper)
        
        if len(self.get_start_nodes())==0 and iter < 100: # try to get any route, even if it is bad
            iter += 1
            self.insert_start_points(max_distance+10, iter)
            if iter % 10 == 0:
                print(f"Expanded search radius to {max_distance+10} meters.")
            

    def find_possible_routes(self):
        self.routes = []
        self.visited_nodes_per_route = []
        for start_node in self.get_start_nodes():
            visited = set([start_node.node_id])
            self.explore_routes([], start_node, visited, 0)
        # print("Routes found:", self.routes)

    def explore_routes(self, current_route, current_node, visited, total_route_length):
        if total_route_length >= 200:
            self.routes.append(current_route)
            self.visited_nodes_per_route.append(visited)
            return
        for next_node_id, connection in current_node.get_connections().items():
            if next_node_id not in visited:
                new_route = current_route.copy()
                new_route.append(connection)
                new_total_route_length = total_route_length + connection.length
                next_node = self.get_node(next_node_id)
                new_visited = visited.copy()
                new_visited.add(next_node_id)
                self.explore_routes(new_route, next_node, new_visited, new_total_route_length)

    def get_routes_as_linestrings(self) -> list[tuple[Node, LineString]]:
        linestrings = []
        for route in self.routes:
            coords = []
            for i, node in enumerate(route):
                if i != 0:
                    assert all(
                        node.coords._coords[0] == coords[-1]
                    ), f"Node {node.node_id} and {route[i-1].node_id} have different relative coordinates."
                    coords.extend(node.coords._coords[1:])
                else:
                    coords.extend(node.coords._coords)
            linestrings.append(LineString(coords))

        return linestrings

    def get_routes_as_nodes(self) -> list[tuple[Node]]:
        """Only for debugging purposes."""
        node_list = [[] for i in range(len(self.routes))]
        for i, route in enumerate(self.routes):
            for linestring in route:
                start_point = linestring.coords[0]
                node_list[i].append(self.find_node_by_coords_close_by(start_point))
        return node_list

    def get_route_as_clean_nodes(self, route_index) -> list[tuple[Node]]:
        """Used to get properties of the best route. Clean means without inserted nodes."""
        
        node_list = []
        for linestring in self.routes[route_index]:
            start_point = linestring.coords[0]
            node_list.append(self.find_node_by_coords_close_by(start_point))
        # append the end point
        end_point = self.routes[route_index][-1].coords[-1]
        node_list.append(self.find_node_by_coords_close_by(end_point))

        # any link with inserted node has been broken to contain a start point
        # cure the link by removing the start point and adding the point on the other side of the inserted node
        new_node_list = []
        for i, node in enumerate(node_list):
            if "inserted_node" in node.node_id:
                if i == 0 or i == len(node_list) - 1:
                    # find connections
                    con = node.get_connections()
                    # find the connection that is not in the node list
                    index_other = i + 1 if i == 0 else i - 1
                    con_other = [key for key in con.keys() if key != node_list[index_other].node_id]
                    assert (
                        len(con_other) == 1
                    ), f"Starting node {node.node_id} has more than two connections, investigate."
                    the_other_connection = con_other[0]
                    new_node_list.append(self.get_node(the_other_connection))
                # don't do anything if the node is in the middle, the connection will be cured by the next node
            else:
                new_node_list.append(node)
        # determine distance along the line to the starting point of the resulting node list
        start_point = Point(self.routes[route_index][0].coords[0])
        connection = self.find_clean_connection_from_map(new_node_list[0], new_node_list[1])
        distance = shapely.line_locate_point(connection, start_point)
        assert distance < connection.length, f"Distance {distance} is longer than the connection length {connection.length}"
        return new_node_list, distance
    
    def find_clean_connection_from_map(self, node_start, node_end):
        current_link_obj, reversed = get_link_connecting_nodes(node_start, node_end, self.wrapper)
        link_in_local_coords = transform_to_vehicle_coordinates(self.vehicle_data, current_link_obj)
        geometry_connection = LineString(link_in_local_coords)
        if reversed:
            geometry_connection = reverse(geometry_connection)
        return geometry_connection

    def find_node_by_coords_close_by(self, coords, max_distance=0.1):
        closest_node = None
        closest_distance = float("inf")
        for node in self.nodes:
            distance = Point(node.get_relative_coords()).distance(Point(coords))
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node
        if closest_distance < max_distance:
            return closest_node

    def inspect_connections(self):
        """
        Make sure that each connection goes both ways. Only for debugging.

        """
        for node in self.nodes:
            for next_node_id, connection in node.get_connections().items():
                next_node = self.get_node(next_node_id)
                if node.node_id not in next_node.get_connections():
                    print(f"Node {node.node_id} is connected to {next_node.node_id} but not the other way around.")
                assert (
                    next_node.get_connections()[node.node_id].coords[0] == connection.coords[-1]
                ), f"Node {node.node_id} and {next_node.node_id} have different relative coordinates."
                assert (
                    next_node.get_connections()[node.node_id].coords[-1] == connection.coords[0]
                ), f"Node {node.node_id} and {next_node.node_id} have different relative coordinates."
            loc = node.get_relative_coords()  # this one also has an assert

@dataclass       
class InputData:
    """
    Args:
        pred_time: dict with keys "lat", "lon", "heading" for the vehicle state at the prediction time
        fp: dict with keys "local_lat", "local_lon" for the future path of the vehicle, each key is a list of floats
        sequence_id: int
    
    """
    pred_time: dict
    fp: dict
    sequence_id: int
    
class RouteGenerator:
    def __init__(self, wrapper, map_max_l1_dist):
        self.wrapper: OSMWrapper = wrapper
        self.map_max_l1_dist = map_max_l1_dist # area of interest to be queried from the map around the ego vehicle, like radius but l1
        
    def create_route(self, inputs: InputData) -> Tuple[list, dict, list]:
        # check if the length of the ground truth is less than 200 meters
        local_fp = [
            [lat, lon] for lat, lon in zip(inputs.fp["local_lat"], inputs.fp["local_lon"])
        ]
        fp_linestring = LineString(local_fp)
        # if fp_linestring.length < 200:
        #     raise ShortGroundTruthError(f"Ground truth is less than 200 meters for sequence {inputs.sequence_id}")
        if fp_linestring.length > 200:
            # we allow for fp to be under 200 meters, specifically when we use a 2-point future path
            # TODO: when we use the full gt, fp should still be at least 200 meters long
            # TODO: make sure this is the case by printing
            # ground truth 
            fp_linestring = substring(fp_linestring, 0, 200)
        
        vehicle_data = {
            "ego_vehicle_lat": inputs.pred_time["lat"],
            "ego_vehicle_lon": inputs.pred_time["lon"],
            "ego_vehicle_yaw": inputs.pred_time["heading"],
        }
        start_map_retrieval = time.perf_counter()
        try:
            map_links = self.create_map(inputs.pred_time["lat"], inputs.pred_time["lon"], self.map_max_l1_dist)
        except NoLinksInQueriedAreaError:
            map_links = self.create_map(inputs.pred_time["lat"], inputs.pred_time["lon"], self.map_max_l1_dist*2)

        start_route_creation = time.perf_counter()
        map_retrieval_time = (start_route_creation - start_map_retrieval)/60
        tree = Tree(map_links, self.wrapper, vehicle_data)
        # tree.inspect_connections() # for debugging
        tree.insert_start_points(15, iter=0)
        tree.find_possible_routes()

        if not tree.routes:
            raise NoRoutesFoundError(f"No routes found for sequence {inputs.sequence_id}")

        shortest_frechet_distance = float("inf")
        best_route_linestring = None
        best_route_index = None
        
        # node_routes = tree.get_routes_as_nodes() #remove later, only for debugging
        linestrings = tree.get_routes_as_linestrings()
        if len(linestrings) == 0:
            raise NoRoutesFoundError(f"No routes found for sequence {inputs.sequence_id}")
        
        for i, route_linestring in enumerate(linestrings):
            route_of_interest = substring(route_linestring, 0, 200)
            frechet_distance_value = RouteGenerator.get_area_between_lines(route_of_interest, fp_linestring)
            if frechet_distance_value < shortest_frechet_distance:
                shortest_frechet_distance = frechet_distance_value
                best_route_linestring = route_linestring
                best_route_index = i
                
        route_coords = best_route_linestring.coords._coords.tolist()
        route_properties = self.get_route_properties(tree, best_route_index, vehicle_data)
        route_generation_time = (time.perf_counter() - start_route_creation)/60
        
        return route_coords, route_properties, map_links, map_retrieval_time, route_generation_time


    def get_route_properties(self, tree:Tree, best_route_index, vehicle_data):
        props = {}
        route_nodes, distance = tree.get_route_as_clean_nodes(best_route_index) # nodes without inserted nodes, so madmaps should have data on every link between them
        
        # first convert route nodes to link objects
        route_links = []
        reversed_list = []
        total_branches = 0
        has_bridge = False
        crossings = []
        total_distance = -distance
        for i, node in enumerate(route_nodes):
            if i == 0:
                continue
            link, reversed = get_link_connecting_nodes(route_nodes[i-1], node, self.wrapper)
            link_in_local_coords = transform_to_vehicle_coordinates(vehicle_data, link)
            link_length = LineString(link_in_local_coords).length
            
            route_links.append(link)
            reversed_list.append(reversed)
            if link.is_bridge():
                has_bridge = True
            
            # record how far are the pedestrian crossings
            crossings_current_link = link.get_pedestrian_crossings()
            for crossing in crossings_current_link:
                geometry_connection = tree.find_clean_connection_from_map(route_nodes[i-1], node)
                local_crossing_coords = convert_geopoint_to_vehicle_coords(crossing, vehicle_data)
                distance_along_path = shapely.line_locate_point(geometry_connection, Point(local_crossing_coords))
                distance_from_ego = distance_along_path + total_distance
                if distance_from_ego > 0:
                    crossings.append(float(distance_from_ego))
                
            # figure out how many branches there are, deliberately avoiding the first node
            for _, _ in node.get_connections().items():
                total_branches += 1
            total_distance += link_length
        
        link_we_are_on = route_links[0]
        are_we_reversed = reversed_list[0]
        props["road_class"] = link_we_are_on.get_road_class().name
        props["is_tunnel"] = link_we_are_on.is_tunnel()
        props["is_highway"] = link_we_are_on.is_highway()
        
        direction = Direction(are_we_reversed)
        props["num_lanes"] = link_we_are_on.get_lane_count(direction)
        
        speed_limit_units = link_we_are_on.get_speed_limit(direction).get_unit()
        speed_limit = link_we_are_on.get_speed_limit(direction).get_value()
        if type(speed_limit) == str:
            if speed_limit == "None":
                speed_limit = None
                speed_limit_units.name = "KPH"
            elif "mph" in speed_limit:
                speed_limit = float(speed_limit.split("mph")[0])
                speed_limit_units.name = "MPH"
        if speed_limit_units.name == "MPH":
            speed_limit = speed_limit * 1.60934
    
        props["num_links"] = len(route_links)
        props["num_branches"] = total_branches
        props["has_bridge"] = has_bridge
        props["speed_limit"] = speed_limit
        props["crossings"] = crossings
        props["valid_route"] = True
        

        # get the number of branches
        return props
    

    def create_map(self, lat, lon, max_l1_dist):
        center = GeoPoint(lat, lon)
        links_in_area = self.wrapper.get_links(center, max_l1_dist)
        links = []
        for link in links_in_area:
            links.append(link.get_ID())
        return links

    # @staticmethod
    # def get_area_between_lines(line1: LineString, line2: LineString):
    #     """
    #     Calculate the area that is between two lines. Discretize the lines into points and calculate total distance between the points.
    #     """
    #     assert np.isclose(line1.length, line2.length, atol=200), "Lines must be 200 meters long"

    #     # Generate interpolation distances
    #     distances = np.arange(0, 200, 2)

    #     # Interpolate points on both lines
    #     line1_points = np.array([line1.interpolate(distance).coords[0] for distance in distances])
    #     line2_points = np.array([line2.interpolate(distance).coords[0] for distance in distances])

    #     # Calculate distances between corresponding points using NumPy for vectorized operations
    #     total_distance = np.sum(np.sqrt(np.sum((line1_points - line2_points) ** 2, axis=1)))

    #     return total_distance
    
    def get_area_between_lines(line1: LineString, line2: LineString):
        """
        Calculate the area that is between two lines. Discretize the lines into points and calculate total distance between the points.
        Handles lines of different lengths by normalizing the interpolation parameter.
        """
        num_points = 101  # if length is 200, this will be 2 meters apart including the start and end points

        # Generate interpolation parameters (normalized)
        params = np.linspace(0, 1, num_points)

        # Interpolate points on both lines using normalized parameters
        line1_points = np.array([line1.interpolate(line1.length * param).coords[0] for param in params])
        line2_points = np.array([line2.interpolate(line2.length * param).coords[0] for param in params])

        # Calculate distances between corresponding points using NumPy for vectorized operations
        total_distance = np.sum(np.sqrt(np.sum((line1_points - line2_points) ** 2, axis=1)))

        return total_distance
    
    @staticmethod
    def get_generic_route_properties():
        return {
            "road_class": "UNCLASSIFIED",
            "is_tunnel": False,
            "is_highway": False,
            "num_lanes": -1,
            "num_links": -1,
            "num_branches": -1,
            "has_bridge": False,
            "speed_limit": -1,
            "crossings": [],
            "valid_route": False,
        }
    
    @staticmethod
    def get_generic_route_coords():
        # straight line 
        return [[0.0, 0.0], [0.0, 500.0]]
    
    
# TODO: see if we want to refactor this into the tree class
def get_link_connecting_nodes(node_1, node_2, wrapper: OSMWrapper) -> Tuple[Link, bool]:
    """
    Returns the link connecting two nodes. if the link is stored as node_1-node_2, the second return value is False, otherwise True.
    """
    link_string_1 = f"{node_1.node_id}-{node_2.node_id}"
    link_string_2 = f"{node_2.node_id}-{node_1.node_id}"
    out = []
    reversed_list = []
    try:
        l1_out = wrapper.get_sd_object_by_id(link_id_object(link_string_1))
        out.extend(l1_out)
        reversed_list.extend([False for i in range(len(l1_out))])
    except:
        pass
    try:
        l2_out = wrapper.get_sd_object_by_id(link_id_object(link_string_2))
        out.extend(l2_out)
        reversed_list.extend([True for i in range(len(l2_out))])
    except:
        pass

    if len(out) == 1:
        return out[0], reversed_list[0]
    elif len(out) > 1:
        # find shortest link
        lengths = [LineString(transform_to_origin_coords(link)).length for link in out]
        best_index = np.argmin(lengths)
        return out[best_index], reversed_list[best_index]
    else:
        return None, None
    
            
    
    