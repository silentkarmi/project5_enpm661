import numpy as np
from dataclasses import dataclass

from . cell import CellGrid
from . vector import Vector
@dataclass
class Obstacle(CellGrid):
    
    def __init__(self, pts):
        super(Obstacle, self).__init__(pts)
        
    def __str__(self):
        return ','.join(f"({round(e[0], 2)},{round(e[1], 2)})" for e in self.pts)
        
        
        
            
        