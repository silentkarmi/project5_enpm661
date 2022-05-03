
#!/usr/bin/python3

from source.canvas import Canvas
from source.constants import Const
from source.world import World

# Defining main function
def main():
    world = World()
    canvas = Canvas(world)
    canvas.draw_world()
    # canvas.draw_single_vector()
  
if __name__=="__main__":
    main()