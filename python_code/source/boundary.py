import numpy as np
from dataclasses import dataclass
from . vector import Vector
from . obstacle import Obstacle
@dataclass
class Boundary(Obstacle):
    
    def __init__(self, pts):
        self.pts = pts
        self.vectors = []
        for i in range(len(pts)):
            if i == len(pts) - 1:
                # reverse vector rotation from the obstacle
                tail = pts[i]
                head = pts[0]
            else:
                # reverse vector rotation from the obstacle
                tail = pts[i]
                head = pts[i + 1]
                
            self.vectors.append(Vector(head, tail))
        
            
        