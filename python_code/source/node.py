from dataclasses import dataclass
import math

from . constants import Const
from . vector import Vector


@dataclass
class Node:
    
    coord: tuple()
    cost2come: int = 0
    cost2go: int = 0
    
    def __init__(self, coord):
        self.coord = coord
        self.childNodes = []
        self.cost2come = 0
        self.cost2go = 0
        self.calculate_cost2Go()
        self.parent = None
        
    def add_child_node(self, node):
        vec = Vector(self.coord, node.coord)
        node.cost2come = vec.magnitude + self.cost2come
        node.parent = self
        self.childNodes.append(node)
        
    def __lt__(self, other):
        return self.cost2come + self.cost2go < other.cost2come + other.cost2go
        
    def __le__(self, other):
        return self.cost2come + self.cost2go <= other.cost2come + other.cost2go
    
    def calculate_cost2Go(self):
        x, y = self.coord
        x1, y1 = Const.GOAL_NODE
        
        #manhattan heuristic
        # self.cost2go = abs(x1 - x) + abs(y1 - y)
        
        #euclidean heuristic
        self.cost2go = round(math.sqrt((x1 - x)**2 + (y1 - y)**2))
        
    def __str__(self):
        return f"Node({round(self.coord[0],2)},{round(self.coord[1], 2)})"