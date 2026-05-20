from enum import Enum


class Direction(Enum):
    FORWARD = 0
    BACKWARD = 1
    
class RoadClass(Enum):
    UNCLASSIFIED = 0
    MOTORWAY = 1
    TRUNK = 2
    PRIMARY = 3
    SECONDARY = 4
    TERTIARY = 5
    IGNORED = 6
    BUILDING = 7
    
class SpeedUnit(Enum):
    UNKNOWN = 0
    KPH = 1
    MPH = 2
    
class MapObjectId:
    def __init__(self, node_id_a, node_id_b):
        self.node_id_a = int(node_id_a)
        self.node_id_b = int(node_id_b)
    
    def __str__(self): 
        return f"{self.node_id_a}-{self.node_id_b}"
    
    def __eq__(self, other):
        return self.node_id_a == other.node_id_a and self.node_id_b == other.node_id_b