import numpy as np
from dataclasses import dataclass
import random

from . constants import Const

from . vector import Vector

@dataclass
class CellGrid:
    
    def __init__(self, pts):
        self.pts = pts
        self.vectors = []
        for i in range(len(pts)):
            if i == len(pts) - 1:
                head = pts[i]
                tail = pts[0]
            else:
                head = pts[i]
                tail = pts[i + 1]
                
            self.vectors.append(Vector(head, tail))
    
    def get_vertexes(self):
        vertex_list = []
        vertex_list.append(self.vectors[0].head)
        vertex_list.append(self.vectors[1].head)
        vertex_list.append(self.vectors[3].head)
        vertex_list.append(self.vectors[2].head)
        
        return vertex_list
            
    def get_draw_vector_form(self):
        x_pos = []
        y_pos = []
        x_direct = []
        y_direct = []
        
        for vector in self.vectors:
            x_pos.append(vector.head[0])
            y_pos.append(vector.head[1])
            x_direct.append(vector.direction[0])
            y_direct.append(vector.direction[1])
            
        return (x_pos, y_pos, x_direct, y_direct)
    
    def is_inside(self, pt):
        first_line_eq = self.vectors[0].get_line_equation(pt)
        second_line_eq = self.vectors[1].get_line_equation(pt)
        third_line_eq = self.vectors[2].get_line_equation(pt)
        fourth_line_eq = self.vectors[3].get_line_equation(pt)
        
        return (first_line_eq > 0 and 
                second_line_eq > 0 and
                third_line_eq < 0 and
                fourth_line_eq < 0)
        
    def does_vector_collide(self, vector):
        y1 = min(vector.head[1], vector.tail[1])
        y2 = max(vector.head[1], vector.tail[1])
        
        flag = False
        for y in np.arange(y1, y2, Const.RESOLUTION * 0.1):
            x = vector.get_corresponding_x(y)
            
            # print(f"({x},{y}) is inside flag={self.is_inside((x,y))}")
            
            if self.is_inside((x,y)):
                flag = True
                break
        
        return flag
        
    def get_random_point_within_me(self):
        vertexes = self.get_vertexes()
        
        Y_TOLERANCE_CLEARANCE = 0
        
        # short_magnitude = float('inf')
        # shortest_vector = None
        # for vector in self.vectors:
        #     if vector.magnitude < short_magnitude:
        #         short_magnitude = vector.magnitude
        #         shortest_vector = vector
        
        vertex_x = [float(vertex[0]) for vertex in vertexes]
        vertex_y = [float(vertex[1]) for vertex in vertexes]
        
        maxima_x = max(vertex_x)
        # try:
        #     while True:
        #         vertex_x.remove(maxima_x)
        # except ValueError:
        #     pass
        
        # maxima_x -= Const.CLEARANCE
        
        minima_x = min(vertex_x)
        # minima_x += Const.CLEARANCE
        
        maxima_y = max(vertex_y)
        # vertex_y = list(filter(lambda a: round(a,3) != round(maxima_y, 3), vertex_y))
        
        # maxima_y -= (Const.CLEARANCE + Y_TOLERANCE_CLEARANCE)
        
        minima_y = min(vertex_y)
        # minima_y += (Const.CLEARANCE + Y_TOLERANCE_CLEARANCE)
        
        pt = ((minima_x + maxima_x) / 2, (minima_y + maxima_y) / 2)
        
           
        # for i in range(50):
        #     x_random = random.uniform(minima_x, maxima_x)
        #     y_random = random.uniform(minima_y, maxima_y)
            
        #     pt = (x_random, y_random)
        #     if self.is_inside(pt):
        #         break
        # else:
        #     pt = None
            
        # print("Random Pt inside Grid:")
        # print(pt)
            
        return pt      
        
        
            
        