
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
    
    
    if world.is_in_obstacle_space(Const.START_NODE):
        print("INVALID START NODE")
        return
    
    if world.is_in_obstacle_space(Const.GOAL_NODE):
        print("INVALID GOAL NODE")
        return
    
    if not world.boundary.is_inside(Const.START_NODE):
        print("START NODE OUTSIDE BOUNDARY")
        
    if not world.boundary.is_inside(Const.GOAL_NODE):
        print("GOAL NODE OUTSIDE BOUNDARY")
        
    world.traverse_node_tree()
    world.back_track()
    # world.check_vector_collision_with_obstacle()
    # print(f"Total Nodes Searched:{len(world.closed_list)}")
    print("--- %s seconds for running the algorithm ---" % (time.time() - start_time))
    
    canvas = Canvas(world)
    canvas.draw_world()
  
if __name__=="__main__":
    main()