from dataclasses import dataclass
from . boundary import Boundary
from . obstacle import Obstacle
from . constants import Const

@dataclass
class World:
    
    def __init__(self):
        self.obstacles = []
        obstacle = Obstacle(((4,5), (6,3), (8,5), (6,7)))
        self.obstacles.append(obstacle)
        
        self.boundary = Boundary(((0, Const.CANVAS_HEIGHT), 
                            (0,0),
                            (Const.CANVAS_WIDTH, 0),
                            (Const.CANVAS_WIDTH, Const.CANVAS_HEIGHT)))