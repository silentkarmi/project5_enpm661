import numpy as np
from dataclasses import dataclass
from . vector import Vector
@dataclass
class Obstacle:
    
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
            
    def get_draw_vector_form(self):
        x_pos = []
        y_pos = []
        x_direct = []
        y_direct = []
        
        for vector in self.vectors:
            x_pos.append(vector.head[0])
            y_pos.append(vector.head[1])
            x_direct.append(vector.component_form[0])
            y_direct.append(vector.component_form[1])
            
        return (x_pos, y_pos, x_direct, y_direct)
        
       
        # vector_list = []
        # for vector in self.vectors:
        #     x_comp = []
        #     y_comp = []
        #     x_comp.append(vector.head[0])
        #     x_comp.append(vector.tail[0])
        #     y_comp.append(vector.head[1])
        #     y_comp.append(vector.tail[1])
        #     vector_list.append((x_comp, y_comp))
        
    
        # return vector_list
        
        
            
        