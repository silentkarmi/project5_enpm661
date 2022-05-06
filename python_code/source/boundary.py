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
            
    def is_inside(self, pt):
        first_line_eq = self.vectors[0].get_line_equation(pt)
        second_line_eq = self.vectors[1].get_line_equation(pt)
        third_line_eq = self.vectors[2].get_line_equation(pt)
        fourth_line_eq = self.vectors[3].get_line_equation(pt)
        
        return (first_line_eq < 0 and 
                second_line_eq > 0 and
                third_line_eq > 0 and
                fourth_line_eq < 0)
        
            
        