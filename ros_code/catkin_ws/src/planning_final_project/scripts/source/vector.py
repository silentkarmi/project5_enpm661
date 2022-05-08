from dataclasses import dataclass
import math
from numpy import NaN

from . utility import print_partition
from .constants import Const

@dataclass
class Vector:
    INFINITY = float('inf')
     
    def __init__(self, head, tail):
        self.head = head # first vertex of line segment
        self.tail = tail # second vertex of line segment
        self.direction = self._calculate_direction() # component form of vector
        self.magnitude = self._calculate_magnitude()
        self.slope, self.intercept = self._calculate_line_parameters()
        
    def _calculate_magnitude(self):
        ui, vj = self.direction
        magnitude = math.sqrt(ui**2 + vj**2)
        return magnitude
        
    def _calculate_direction(self):
        x1, y1 = self.head
        x2, y2 = self.tail
        
        direction = []
        direction.append(x2 - x1)
        direction.append(y2 - y1)
        return direction # return the component form of the vector
    
    def _calculate_line_parameters(self):
        x_dir, y_dir = self.direction
        
        if y_dir == 0:
            slope = 0
        elif x_dir == 0:
            slope = Vector.INFINITY
        else:
            slope = y_dir / x_dir
        
        x, y = self.head
        c = y - slope * x
        
        return (slope, c)
    
    def get_line_equation(self, pt):
        x, y = pt
        line_eq = 0
        
        if self.slope == Vector.INFINITY:
            line_eq = x - self.head[0]
        else:
            line_eq = y - (self.slope * x + self.intercept)
            
        return line_eq
    
    def get_corresponding_y(self, x):
        return (self.slope * x + self.intercept)
    
    def get_corresponding_x(self, y):
        x = 0
        # if self.slope == 0:
        #     x = self.head[0]
        if self.slope == float('inf'):
            x = self.intercept
        else:
            x = (y - self.intercept) / self.slope
        return x
    
    def get_mid_point(self):
        pt = ((self.head[0] + self.tail[0]) * 0.5,
              (self.head[1] + self.tail[1]) * 0.5)
        
        return pt
    
    def is_point_on_vector(self, pt):
        # print_partition()
        flag = False
        x, y = pt
        # x = round(x, Const.ROUND_DECIMAL_POINT)
        # y = round(y, Const.ROUND_DECIMAL_POINT)
        
        minima_x = min(self.head[0], self.tail[0])
        maxima_x = max(self.head[0], self.tail[0])
        minima_y = min(self.head[1], self.tail[1])
        maxima_y = max(self.head[1], self.tail[1])
        
        if ((x >= minima_x and x <= maxima_x)
            or (y >= minima_y and y <= maxima_y)):
                value = round((y - (self.slope * x + self.intercept)), Const.ROUND_DECIMAL_POINT)
                if value == 0:
                    flag = True
                
                # print(f"Point to check:({x}, {y}) on {self}")
                # print(f"Value = {value} | flag = {flag} ")
                
        # print_partition()
                
        return flag
    
    def __str__(self):
        return f"(HEAD:{self.head}, TAIL:{self.tail}, magnitude = {self.magnitude}, thetha={self.slope})" 
    
    # def does_this_vector_intersect_with(self, vector_to_check):
    #     x, _ = self.head
    #     _, y1 = vector_to_check.head
        
    #     intersect_flag = vector_to_check.is_point_on_vector((x,y1))
        
    #     vector_to_intersect_pt = None
        
    #     if intersect_flag:
    #         vector_to_intersect_pt = Vector(self.head, (x, y1))
            
    #     return intersect_flag, vector_to_intersect_pt
       

        
        