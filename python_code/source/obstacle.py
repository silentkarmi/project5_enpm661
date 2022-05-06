import numpy as np
from dataclasses import dataclass

from . cell import CellGrid
from . vector import Vector
@dataclass
class Obstacle(CellGrid):
    
    def __init__(self, pts):
        super(Obstacle, self).__init__(pts)
        
        
        
            
        