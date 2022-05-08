
#!/usr/bin/python3

from source.canvas import Canvas
from source.constants import Const
from source.world import World
from source.node import Node

import time

# Defining main function
def main():
    world = World()
    start_time = time.time()
    
    
    # vertical decomposition has to be done only once
    world.perform_vertical_decomposition_algorithm() 
    world.generate_way_points(Const.START_NODE, 
                              Const.GOAL_NODE,
                              Const.TREES_REQUIRED)    
    
    # world.check_vector_collision_with_obstacle()
    # print(f"Total Nodes Searched:{len(world.closed_list)}")
    print("--- %s seconds for running the algorithm ---" % (time.time() - start_time))
    
    canvas = Canvas(world)
    canvas.draw_world()
  
if __name__=="__main__":
    main()