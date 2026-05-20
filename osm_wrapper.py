import osmnx as ox

from errors import NoLinksInQueriedAreaError
from utils.enums import Direction, RoadClass, SpeedUnit, MapObjectId
from shapely.geometry import LineString

from geopy import Point as GeoPoint
from geopy.distance import distance

from math import nan

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
        
class SpeedLimit:
    def __init__(self, speed_limit, speed_unit):
        assert type(speed_unit) == SpeedUnit
        self.speed_limit = speed_limit
        self.unit = speed_unit
        
    def get_unit(self) -> SpeedUnit:
        return self.unit
    
    def get_value(self):
        return self.speed_limit
    
def parse_maxspeed(data):
    """
    Parse the maxspeed data from osm and return a SpeedLimit object.

    Parameters:
        data (dict): Dictionary containing the "maxspeed" key.

    Returns:
        SpeedLimit: An object containing the maximum speed and its unit.
    """
    max_speed = float("nan")  # Default to NaN
    unit = SpeedUnit.UNKNOWN  # Default unit

    if "maxspeed" in data:
        maxspeed_data = data["maxspeed"]

        # Handle if maxspeed is a list
        if isinstance(maxspeed_data, list):
            # Extract numeric values, ignore invalid entries
            numeric_speeds = []
            for entry in maxspeed_data:
                if "mph" in entry:
                    entry = entry.replace("mph", "").strip()
                    unit = SpeedUnit.MPH
                elif "kph" in entry:
                    entry = entry.replace("kph", "").strip()
                    unit = SpeedUnit.KPH
                elif "signals" in entry or "none" in entry or "variable" in entry:
                    continue
                
                # entry should be a number now and if its not we want to fail anyway
                numeric_speeds.append(int(entry))

            # Take the mean valid value, if any
            if numeric_speeds:
                max_speed = sum(numeric_speeds) / len(numeric_speeds)
            else:
                max_speed = float("nan")  # No valid numeric entries found
        else:
            # Handle if maxspeed is a string
            if isinstance(maxspeed_data, str):
                if "mph" in maxspeed_data:
                    unit = SpeedUnit.MPH
                    maxspeed_data = maxspeed_data.replace("mph", "")
                elif "kph" in maxspeed_data:
                    unit = SpeedUnit.KPH
                    maxspeed_data = maxspeed_data.replace("kph", "")
                else:
                    unit = SpeedUnit.KPH  # Default to KPH if no unit specified
                
                maxspeed_data = maxspeed_data.strip(" -")
                    
                if "+ maxspeed" in maxspeed_data:
                    # e.g. 50 + maxspeed:conditional=30 @ 07:00-17:00
                    maxspeed_data = maxspeed_data.split("+ maxspeed")[0].strip()
                
                if "@" in maxspeed_data:
                    # e.g. 50 @ signals
                    maxspeed_data = maxspeed_data.split("@")[0].strip()

                # Handle special cases
                if maxspeed_data in ["signals", "none", "variable", "unposted", "non"]:
                    max_speed = float("nan")
                elif maxspeed_data in ["walk", "DE:living_street", "DE:walk", "AT:walk"]:
                    max_speed = 7
                    unit = SpeedUnit.KPH
                elif maxspeed_data in ["DE:zone:30", "30g", "AT:zone:30"]:
                    max_speed = 30
                    unit = SpeedUnit.KPH
                elif maxspeed_data in ["DE:urban", "PL:urban", "IT:urban", "FR:urban"]:
                    max_speed = 50
                    unit = SpeedUnit.KPH
                elif maxspeed_data in ["FR:rural"]:
                    max_speed = 80
                    unit = SpeedUnit.KPH
                elif maxspeed_data in ["PL:rural"]:
                    max_speed = 90
                    unit = SpeedUnit.KPH
                elif maxspeed_data == "DE:rural":
                    max_speed = 100
                    unit = SpeedUnit.KPH
                elif maxspeed_data == "DE:motorway":
                    max_speed = 140 # setting to None would downplay its significance
                    unit = SpeedUnit.KPH
                elif ";" in maxspeed_data:
                    # Handle multiple values
                    values = maxspeed_data.split(";")
                    values_int = [int(value.strip()) for value in values]
                    max_speed = sum(values_int) / len(values_int)
                else:
                    max_speed = int(float(maxspeed_data)) # should be a number now

    return SpeedLimit(max_speed, unit)

class Link:
    def __init__(self, id:str, data:dict):
        self.id = MapObjectId(id.split("-")[0], id.split("-")[1])
        assert "geometry" in data
        self.geometry = data["geometry"]
        self.num_lanes = data.get("lanes", nan)
        self.length = data.get("length", nan)
        self.speed_limit = parse_maxspeed(data)
        self.highway = "highway" in data
        self.tunnel = "tunnel" in data
        self.bridge = "bridge" in data
        
        # Determine road class
        if "highway" in data:
            highway = data["highway"]
            if highway == "motorway":
                self.road_class = RoadClass.MOTORWAY
            elif highway == "trunk":
                self.road_class = RoadClass.TRUNK
            elif highway == "primary":
                self.road_class = RoadClass.PRIMARY
            elif highway == "secondary":
                self.road_class = RoadClass.SECONDARY
            elif highway == "tertiary":
                self.road_class = RoadClass.TERTIARY
            elif highway == "road":
                self.road_class = RoadClass.UNCLASSIFIED
            elif highway == "residential":
                self.road_class = RoadClass.BUILDING
            else:
                self.road_class = RoadClass.IGNORED
        
        else:
            self.road_class = RoadClass.IGNORED
        
    
    def get_ID(self) -> str:
        return f"{self.id.node_id_a}-{self.id.node_id_b}"
    
    def get_ID_object(self) -> MapObjectId:
        return self.id
        
    def get_road_class(self):
        return self.road_class
    
    def get_pedestrian_crossings(self) -> list[GeoPoint]:
        return []
    
    def get_lane_count(self, direction: Direction) -> int:
        return self.num_lanes
    
    def get_speed_limit(self, direction: Direction) -> SpeedLimit:
        return self.speed_limit
    
    def is_highway(self) -> bool:
        return self.highway
    
    def is_tunnel(self) -> bool:
        return self.tunnel
    
    def is_bridge(self) -> bool:
        return self.bridge
    
    def get_length(self) -> float:
        return self.length
    
    def get_geopoints(self) -> list[GeoPoint]:
        out = []
        for point in self.geometry.coords:
            out.append(GeoPoint(point[1], point[0]))
        return out    

class OSMWrapper:
    def __init__(self, cache_path=None):
        if cache_path is not None and cache_path != "None":
            # Set the cache folder for OSMnx
            ox.settings.cache_folder = cache_path
        else:
            ox.settings.use_cache = False
    
    def get_graph_from_point(self, lat, lon, dist, network_type='drive_service'):
        # truncate by edge is set to True to get edges that originate within the dist but may extend beyond it
        try:
            return ox.graph_from_point((lat, lon), dist=dist, network_type=network_type, retain_all=True, truncate_by_edge=True, simplify=False)
        except ValueError as e:
            if "Found no graph nodes within the requested polygon" in str(e):
                raise NoLinksInQueriedAreaError("No nodes found in the queried area")
    def project_graph(self, graph):
        return ox.project_graph(graph)
    
    def get_sd_object_by_id(self, link_id:MapObjectId) -> list[Link]:
        """Return a list with the only element being the Link with the matching link id"""
        if self.links is None:
            raise Exception("No links have been loaded yet")
        for link in self.links:
            if link.get_ID_object() == link_id:
                return [link]
    
    def get_links(self, center: GeoPoint, max_l1_dist: float) -> list[Link]:
        graph = self.get_graph_from_point(center.latitude, center.longitude, max_l1_dist)
        if graph is None:
            raise NoLinksInQueriedAreaError("No nodes found in the queried area")
        links = []
        for u, v, data in graph.edges(data=True):
            if "geometry" not in data:
                # If there is no geometry, the edge is a straight line, but we still need to create a LineString object and add it to the data
                from_ = GeoPoint(latitude=graph.nodes[u]["y"], longitude=graph.nodes[u]["x"])
                to_ = GeoPoint(latitude=graph.nodes[v]["y"], longitude=graph.nodes[v]["x"])
                data["geometry"] = LineString([[from_.longitude, from_.latitude], [to_.longitude, to_.latitude]])
            id = f"{u}-{v}"
            link = Link(id, data)
            links.append(link)
        self.links = links
        return links
    