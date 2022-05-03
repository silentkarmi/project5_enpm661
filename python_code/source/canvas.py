from dataclasses import dataclass
from email.base64mime import header_length
import matplotlib.pyplot as plt
import numpy as np
from . vector import Vector
from . constants import Const

@dataclass
class Canvas:
    def __init__(self, world):
       self.world = world
       
    def draw_world(self):
        for obstacle in self.world.obstacles:
            fig, ax = plt.subplots()

            origin = np.array([[0, 0, 0],[0, 0, 0]]) # origin point
            # vector_list = \
            #     obstacle.get_draw_vector_form()
                
            x_pos, y_pos, x_dir, y_dir = \
               obstacle.get_draw_vector_form()
                
            ax.quiver(x_pos,
                      y_pos,
                      x_dir,
                      y_dir, 
                      angles='xy', 
                      scale_units='xy', 
                      scale=1)
            # end of for loop
            
        x_pos, y_pos, x_dir, y_dir = \
               self.world.boundary.get_draw_vector_form()
               
        ax.quiver(x_pos,
                      y_pos,
                      x_dir,
                      y_dir, 
                      angles='xy', 
                      scale_units='xy', 
                      scale=1)
        
        ax.axis([Const.ORIGIN_X, 
                    Const.CANVAS_WIDTH, 
                    Const.ORIGIN_Y,
                    Const.CANVAS_HEIGHT])
            

        plt.show()
            
    # test function to draw vector
    @staticmethod
    def draw_single_vector():
        vector = Vector((4,5), (6,3))
               
        fig, ax = plt.subplots()

        x_pos, y_pos = vector.head

        ax.quiver(x_pos,y_pos,vector.component_form[0],vector.component_form[1], angles='xy', scale_units='xy', scale=1)
        
        ax.axis([Const.ORIGIN_X - 10, 
                 Const.CANVAS_WIDTH, 
                 Const.ORIGIN_Y - 10,
                 Const.CANVAS_HEIGHT])
        
        # x_direct, y_direct = vector.component_form
        # x_direct, y_direct = vector.tail
        # plt.arrow(x_pos,y_pos,vector.component_form[0],vector.component_form[1], length_includes_head = True, head_width = 0.2, head_length = 0.1)
        # plt.plot([x_pos, x_direct],[y_pos, y_direct])

        plt.show()