
#!/usr/bin/python3

from source.canvas import Canvas
from source.constants import Const
from source.world import World

# Defining main function
def main():
    world = World()
    world.perform_vertical_decomposition_algorithm()
    
    canvas = Canvas(world)
    canvas.draw_world()
  
if __name__=="__main__":
    main()