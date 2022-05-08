from dataclasses import dataclass

@dataclass
class Const:
    #Everything in meters
    
    CANVAS_WIDTH = 50
    CANVAS_HEIGHT = 50
    
    ORIGIN_X = 0
    ORIGIN_Y = 0
    
    

    ROUND_DECIMAL_POINT = 1
    RESOLUTION = 0.1
    
    ROBOT_DIA = 0.178
    WALL_CLEARANCE = 0.05
    
    JITTER_VALUE = 0
    
    SKEW_WIDTH = RESOLUTION
    
    GOAL_THRESOLD = RESOLUTION
    
    TREES_REQUIRED = 0
    
    
    
    CLEARANCE = ROBOT_DIA + WALL_CLEARANCE + JITTER_VALUE
    
    TREE_RADIUS = 0.3 + CLEARANCE
    
    START_NODE = (0.5, 0.5)
    GOAL_NODE = (40.0,
                 40.0)
    
    SMOOTHEN_STRENGTH = 8
    
    # GOAL_NODE = (49.00,
    #              25.00)
    