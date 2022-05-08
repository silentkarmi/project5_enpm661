from dataclasses import dataclass
from email.base64mime import header_length
from matplotlib import widgets
import matplotlib.pyplot as plt
import numpy as np
from . vector import Vector
from . constants import Const

@dataclass
class Canvas:
    def __init__(self, world):
       self.world = world
       
    def draw_world(self):
        print("Draw World...")
        fig, ax = plt.subplots()
        
        vector_draw_width = 0.001
        
        # BOUNDARY
        x_pos, y_pos, x_dir, y_dir = \
               self.world.boundary.get_draw_vector_form()
               
        ax.quiver(x_pos,
                      y_pos,
                      x_dir,
                      y_dir, 
                      angles='xy', 
                      scale_units='xy', 
                      scale=1,
                      width = vector_draw_width)
        
        # OBSTACLES
        for obstacle in self.world.obstacles:
            x_pos, y_pos, x_dir, y_dir = \
               obstacle.get_draw_vector_form()
                
            ax.quiver(x_pos,
                      y_pos,
                      x_dir,
                      y_dir, 
                      angles='xy', 
                      scale_units='xy', 
                      scale=1,
                      color = 'r',
                      width = vector_draw_width)
            
        
        # # UP-DOWN PROJECTION VECTORS
        # for vector in self.world.intersection_vectors:
        #     ax.quiver(vector.head[0],
        #               vector.head[1],
        #               vector.direction[0],
        #               vector.direction[1],
        #               angles='xy', 
        #               scale_units='xy', 
        #               scale=1,
        #               color='b',
        #               width = vector_draw_width)
            
        # ROADMAP POINTS
        # for pt in self.world.roadman_points:
        #     x_pos, y_pos = pt
        #     plt.plot(x_pos, y_pos, marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="white")
        
        # ax.axis([Const.ORIGIN_X, 
        #             Const.CANVAS_WIDTH, 
        #             Const.ORIGIN_Y,
        #             Const.CANVAS_HEIGHT])
            
        # BEFORE SMOOTHENING TRAJECTORY
        for i in range(len(self.world.before_smoothening_way_points) - 1):
            vector = Vector(self.world.before_smoothening_way_points[i],
                            self.world.before_smoothening_way_points[i + 1])
            ax.quiver(vector.head[0],
                      vector.head[1],
                      vector.direction[0],
                      vector.direction[1],
                      angles='xy', 
                      scale_units='xy', 
                      scale=1,
                      color='orange',
                      width = vector_draw_width)
            
        # AFTER SMOOTHENING TRAJECTORY
        for i in range(len(self.world.way_points) - 1):
            vector = Vector(self.world.way_points[i],
                            self.world.way_points[i + 1])
            ax.quiver(vector.head[0],
                      vector.head[1],
                      vector.direction[0],
                      vector.direction[1],
                      angles='xy', 
                      scale_units='xy', 
                      scale=1,
                      color='g',
                      width = vector_draw_width)
            
        # CELL-GRIDS
        # for cell in self.world.cell_grids:
        #     x_pos, y_pos, x_dir, y_dir = \
        #        cell.get_draw_vector_form()
                
        #     ax.quiver(x_pos,
        #               y_pos,
        #               x_dir,
        #               y_dir, 
        #               angles='xy', 
        #               scale_units='xy', 
        #               scale=1,
        #               color = 'g',
        #               width = vector_draw_width)
        
        if self.world.way_points:
            plt.plot(self.world.way_points[0][0], self.world.way_points[0][1], marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="orange")
        
        if self.world.solution_node is not None:
            plt.plot(self.world.way_points[-1][0], self.world.way_points[-1][1], marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")


        plt.show()
        
        
#===================== UNIT TEST CASES ======================================
        
    # test function to draw vector
    @staticmethod
    def draw_single_vector():
        vector = Vector((4,5), (6,3))
               
        fig, ax = plt.subplots()

        x_pos, y_pos = vector.head

        ax.quiver(x_pos,y_pos,vector.direction[0],vector.direction[1], angles='xy', scale_units='xy', scale=1)
        
        ax.axis([Const.ORIGIN_X - 10, 
                 Const.CANVAS_WIDTH, 
                 Const.ORIGIN_Y - 10,
                 Const.CANVAS_HEIGHT])

        plt.show()
    
    # test function to check if point inside the obstacle or not
    def test_case_check_point_inside_obstacle(self):
        print("check_point_inside_obstacle")
        
        pt_outside = (4.41, 4.17)
        flag = self.world.obstacles[0].is_inside(pt_outside)
        print(f"Is inside ({pt_outside[0]}, {pt_outside[1]})(No)?: {flag}")
        
        pt_inside = (4.3,5)
        flag = self.world.obstacles[0].is_inside(pt_inside)
        print(f"Is inside ({pt_inside[0]}, {pt_inside[1]})(Yes)?: {flag}")
        
    # test function to check if point inside the obstacle or not
    def test_case_check_point_inside_boundary(self):
        print("check_point_inside_boundary")
        pt_outside = (0.26, 8.5)
        pt_inside = (4.3,5)
        pt_inside1 = (4.41, 4.17)
        
        flag = self.world.boundary.is_inside(pt_outside)
        print(f"Is inside ({pt_outside[0]}, {pt_outside[1]})(No)?: {flag}")
        
        flag = self.world.boundary.is_inside(pt_inside)
        print(f"Is inside ({pt_inside[0]}, {pt_inside[1]})(Yes)?: {flag}")
        
        flag = self.world.boundary.is_inside(pt_inside1)
        print(f"Is inside ({pt_inside1[0]}, {pt_inside1[1]})(Yes)?: {flag}")
        
    def test_case_check_if_point_on_a_edge(self):
        pt_on_line = (4.88, 4.12)
        flag = self.world.obstacles[0].vectors[0].is_point_on_vector(pt_on_line)
        print(f"Is on the line ({pt_on_line[0]}, {pt_on_line[1]})(Yes)?: {flag}")
        
        pt_not_on_line = (1, 2)
        flag = self.world.obstacles[0].vectors[0].is_point_on_vector(pt_not_on_line)
        print(f"Is on the line ({pt_not_on_line[0]}, {pt_not_on_line[1]})(No)?: {flag}")
        
        pt_not_on_line = (8.49, 0.51)
        flag = self.world.obstacles[0].vectors[0].is_point_on_vector(pt_not_on_line)
        print(f"Is on the line ({pt_not_on_line[0]}, {pt_not_on_line[1]})(No)?: {flag}")
    
#===============================================================================