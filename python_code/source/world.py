from dataclasses import dataclass
from typing_extensions import Self

from source.cell import CellGrid
from . boundary import Boundary
from . obstacle import Obstacle
from . constants import Const
from . vector import Vector
from . utility import print_partition
from . node import Node

import heapq

@dataclass
class World:
    
    def __init__(self):
        self.obstacles = []
        obstacle = Obstacle(((4,5), (6,3), (8,5), (6,7)))
        self.obstacles.append(obstacle)
        
        self.boundary = Boundary(((0 + Const.SKEW_WIDTH, Const.CANVAS_HEIGHT), 
                            (0,0),
                            (Const.CANVAS_WIDTH - Const.SKEW_WIDTH, 0),
                            (Const.CANVAS_WIDTH, Const.CANVAS_HEIGHT)))
        
        self.intersection_vectors = []
        self.up_vectors = []
        self.down_vectors = []
        
        self.cell_grids = []
        
        self.roadman_points = []
        self.node_list = []
        self.first_node = None
        
        self.traversal_path = []
        
        self.open_list = []
        self.closed_list = []
        
        self.solution_path = []
        self.solution_node = None
        
    def is_in_obstacle_space(self, pt):
        flag = False
        for obstacle in self.obstacles:
            if obstacle.is_inside(pt):
                flag = True
                break
            
        return flag
        
    def get_all_vertexes(self):
        vertexes = []
        
        # for edge in self.boundary.vectors:
        vertexes.append(self.boundary.vectors[0].head)
        vertexes.append(self.boundary.vectors[3].head)
        
        for obstacle in self.obstacles:
            vertexes.extend(obstacle.get_vertexes())
                
        vertexes.append(self.boundary.vectors[1].head)
        vertexes.append(self.boundary.vectors[2].head)
            
        return vertexes
            
    def get_all_edges(self):
        edges = []
        for obstacle in self.obstacles:
            for edge in obstacle.vectors:
                edges.append(edge)
                
        for edge in self.boundary.vectors:
            edges.append(edge)
            
        return edges
            
    def perform_vertical_decomposition_algorithm(self):       
        for vertex in self.get_all_vertexes():
            self.project_orthogonal_vectors_for(vertex)
        
        self.sample_roadman_points_and_create_node_tree()
        
        #test single vertex
        # self.project_orthogonal_vectors_for((0 + Const.SKEW_WIDTH, Const.CANVAS_HEIGHT))
    
    def project_orthogonal_vectors_for(self, vertex):
        # does the up vector from the vector collides with obstacle
        # does it collides with the boundary
        x_const = vertex[0]
        project_in_up_dir = vertex[1] 
        project_in_down_dir = vertex[1]
        pt_up = (x_const, project_in_up_dir)
        pt_down = (x_const, project_in_down_dir)
        up_vector_found = False
        down_vector_found = False
        up_vector = None
        down_vector = None
    
        while(not up_vector_found or
              not down_vector_found):
            
            # project orthagonal vectors
            if not up_vector_found:
                # project in upwards direction
                project_in_up_dir += Const.RESOLUTION
                pt_up = (x_const, project_in_up_dir)

            if not down_vector_found:
                # project in downwards direction
                project_in_down_dir -= Const.RESOLUTION 
                pt_down = (x_const, project_in_down_dir)
                
            # now does the point intersect any edges
            for edge in self.get_all_edges():
                
                if not up_vector_found:
                    if edge.is_point_on_vector(pt_up):
                        up_vector = Vector(vertex, pt_up)
                        up_vector_found = True
                        
                if not down_vector_found:
                    if edge.is_point_on_vector(pt_down):
                        down_vector = Vector(vertex, pt_down)
                        down_vector_found = True
                        
                        
            if not up_vector_found:                
                # we stop projecting if we are inside obstacle
                if self.is_in_obstacle_space(pt_up):
                    up_vector = None
                    up_vector_found = True
                        
                # we stop projecting if we go outside boundary
                elif not self.boundary.is_inside(pt_up):
                    up_vector = None
                    up_vector_found = True
                
            if not down_vector_found:               
                # we stop projecting if we are inside obstacle
                if self.is_in_obstacle_space(pt_down):
                    down_vector = None
                    down_vector_found = True
                        
                # we stop projecting if we go outside boundary
                elif not self.boundary.is_inside(pt_down):
                    down_vector = None
                    down_vector_found = True
                        
        print_partition()
        print(f"up_vector: {up_vector}" )
        print(f"down_vector: {down_vector}") 
        print_partition()
        if up_vector is not None:
            self.intersection_vectors.append(up_vector)
            self.up_vectors.append(up_vector)
            
        if down_vector is not None:
            self.intersection_vectors.append(down_vector)
            self.down_vectors.append(down_vector)
        
    def sample_roadman_points_and_create_node_tree(self):
        parent_node_down = None
        child_node_down = None
        first_node_down = None
        
        parent_node_up = None
        child_node_up = None
        first_node_up = None
        
        for i in range(len(self.up_vectors) - 1):
            first_vector_down = self.down_vectors[i]
            second_vector_down = self.down_vectors[i + 1]
            
            cell = CellGrid(((first_vector_down.head),
                                            (first_vector_down.tail),
                                            (second_vector_down.tail),
                                            second_vector_down.head))
            
            self.cell_grids.append(cell)
            
            #down vector mid point
            self.roadman_points.append(first_vector_down.get_mid_point())
            child_node_down = Node(self.roadman_points[-1])
            
            if parent_node_down is not None:
                 parent_node_down.add_child_node(child_node_down)
            else:
                self.first_node = child_node_down
                 
            parent_node_down = child_node_down
            
            #cell grid random point
            self.roadman_points.append(cell.get_random_point_within_me())
            child_node_down = Node(self.roadman_points[-1])
            parent_node_down.add_child_node(child_node_down)
            parent_node_down = child_node_down
            
            
            
            first_vector_up = self.up_vectors[i]
            second_vector_up = self.up_vectors[i + 1]
            
            cell = CellGrid(((first_vector_up.tail),
                                            (first_vector_up.head),
                                            (second_vector_up.head),
                                            second_vector_up.tail))
            
            self.cell_grids.append(cell)
            
            #up vector mid point
            self.roadman_points.append(first_vector_up.get_mid_point())
            child_node_up = Node(self.roadman_points[-1])
            
            if parent_node_up is not None:
                 parent_node_up.add_child_node(child_node_up)
            else:
                parent_node_down.add_child_node(child_node_up)
                 
            parent_node_up = child_node_up
            
            #cell grid random point
            self.roadman_points.append(cell.get_random_point_within_me())
            child_node_up = Node(self.roadman_points[-1])
            parent_node_up.add_child_node(child_node_up)
            parent_node_up = child_node_up

        
        # last down vector
        self.roadman_points.append(second_vector_down.get_mid_point())
        last_down_node = Node(self.roadman_points[-1])
        parent_node_down.add_child_node(last_down_node)
        
        # last up vector
        self.roadman_points.append(second_vector_up.get_mid_point())
        print(second_vector_up.get_mid_point())
        parent_node_up.add_child_node(Node(self.roadman_points[-1]))
        last_down_node.add_child_node(parent_node_up)
        
    def traverse_node_tree(self):
        self.open_list = []
        self.closed_list = []
        self.open_list.append(self.first_node)
        
        close_len_to_start = float("inf")
        close_len_to_end = float("inf")
        
        closest_start_node_found = False
        closest_goal_node_found = False
        
        closest_start_node = None
        
        while (not closest_start_node_found or
               not closest_goal_node_found):
            parent_node = self.open_list.pop(0)
            
            if not closest_start_node_found:
                vec = Vector(Const.START_NODE, parent_node.coord)
                if vec.magnitude < close_len_to_start:
                    close_len_to_start = vec.magnitude
                    closest_start_node = parent_node
                elif vec.magnitude > close_len_to_start:
                    closest_start_node_found = True
                    self.first_node = Node(Const.START_NODE)
                    closest_start_node.parent = self.first_node
                    self.first_node.parent = None
                    
            if not closest_goal_node_found:
                vec = Vector(Const.GOAL_NODE, parent_node.coord)
                if vec.magnitude < close_len_to_end:
                    close_len_to_end = vec.magnitude
                    closest_goal_node = parent_node
                elif vec.magnitude > close_len_to_end:
                    closest_goal_node_found = True
                    self.solution_node = Node(Const.GOAL_NODE)
                    closest_goal_node.add_child_node(self.solution_node)
                    self.solution_node.parent = closest_goal_node
                    
                
            for child_node in parent_node.childNodes:
                child_node.parent = parent_node
                heapq.heappush(self.open_list, child_node)
                
    def is_this_goal_node(self, node):
        return (round(node.coord[0], 2) == round(Const.GOAL_NODE[0], 2) and 
                round(node.coord[1], 2) == round(Const.GOAL_NODE[1], 2))
    
    #traverse nodes to put the start node and end node connection    
    # def traverse_node_tree(self):
    #     self.open_list = []
    #     self.closed_list = []
    #     self.open_list.append(self.first_node)
        
    #     while self.open_list:
    #         parent_node = self.open_list.pop(0)
            
    #         for child_node in parent_node.childNodes:
    #             child_node.parent = parent_node
    #             heapq.heappush(self.open_list, child_node)
                
    #         self.closed_list.append(parent_node)
            
    #         if self.is_this_goal_node(parent_node):
    #             self.solution_node = parent_node
    #             break
            
    def back_track(self):
        print("Backtracking...")
        tempNode = self.solution_node
        while tempNode.parent != None:
            self.traversal_path.insert(0, Vector(tempNode.parent.coord, tempNode.coord))
            tempNode = tempNode.parent
            

            
        
        

            
        
    
            
                            
