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
import yaml
import random
import math

@dataclass
class World:
    
    def __init__(self):
        self.obstacles = []
        
        file_path = 'source/forest30.yaml' #10, 20, 30, 50

        with open(file_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader) # data is a dict
 

        tree_coords = []
        for tree in data['trees_position']:
            tree_coords.append(tree)
            
        tree_coords.sort()
            
        i = 0
        for tree in tree_coords:
            i+=1
            x, y = tree
            
            x_jitter = random.uniform(-Const.JITTER_VALUE, Const.JITTER_VALUE)
            y_jitter = random.uniform(-Const.JITTER_VALUE, Const.JITTER_VALUE)
            
            x += x_jitter
            y += y_jitter
            
            obstacle = Obstacle(((x - Const.TREE_RADIUS , y), 
                                 (x , y - Const.TREE_RADIUS), 
                                 (x + Const.TREE_RADIUS, y), 
                                 (x, y + Const.TREE_RADIUS)))
            self.obstacles.append(obstacle)
            
            # if i == 3:
            #     break
            
            # obstacle = Obstacle(((4,5), (6,3), (8,5), (6,7)))
            # self.obstacles.append(obstacle)
            # break
        
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
        
        self.total_nodes_searched = 0
        
        
        
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
        print("Start perform_vertical_decomposition_algorithm")
               
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
                if not self.boundary.is_inside(pt_up):
                    up_vector = None
                    up_vector_found = True
                
            if not down_vector_found:               
                # we stop projecting if we are inside obstacle
                if self.is_in_obstacle_space(pt_down):
                    down_vector = None
                    down_vector_found = True
                        
                # we stop projecting if we go outside boundary
                if not self.boundary.is_inside(pt_down):
                    down_vector = None
                    down_vector_found = True
                        
        # print_partition()
        # print("project_orthogonal_vectors_for")
        # print(vertex)
        # print(f"up_vector: {up_vector}" )
        # print(f"down_vector: {down_vector}") 
        # print_partition()
        if up_vector is not None:
            self.intersection_vectors.append(up_vector)
            self.up_vectors.append(up_vector)
            
        if down_vector is not None:
            self.intersection_vectors.append(down_vector)
            self.down_vectors.append(down_vector)
            
    def do_vector_collide_with_obstacles(self, vector):
        flag = False
        for obstacle in self.obstacles:
            if obstacle.does_vector_collide(vector):
                flag = True
                break
            
        return flag
            
        
    def sample_roadman_points_and_create_node_tree(self):
        print("Creating sample_roadman_points_and_create_node_tree..")
        parent_node_down = None
        child_node_down = None
        
        parent_node_up = None
        child_node_up = None
        first_node_up = None
        
        len_vector = max(len(self.up_vectors), len(self.down_vectors))
        
        first_vector_down = None
        second_vector_down = None
        for i in range(len_vector - 1):
            if i + 1 < len(self.down_vectors):
                first_vector_down = self.down_vectors[i]
                second_vector_down = self.down_vectors[i + 1]
                
                cell = CellGrid(((first_vector_down.head),
                                                (first_vector_down.tail),
                                                (second_vector_down.tail),
                                                second_vector_down.head))
                
                self.cell_grids.append(cell)
                
                #down vector mid point
                self.roadman_points.append(first_vector_down.get_mid_point())
                
                if parent_node_down is None:
                    parent_node_down = Node(self.roadman_points[-1])
                    self.first_node = parent_node_down
                else:
                    child_node_down = Node(self.roadman_points[-1])
                    if parent_node_up is not None:
                        vec = Vector(parent_node_up.coord, child_node_down.coord)
                        if vec.head[0] < vec.tail[0]:
                            if not self.do_vector_collide_with_obstacles(vec):
                                parent_node_up.add_child_node(child_node_down)
                    
                    vec = Vector(parent_node_down.coord, child_node_down.coord)
                    if vec.head[0] < vec.tail[0]:
                        if not self.do_vector_collide_with_obstacles(vec):
                                parent_node_down.add_child_node(child_node_down)
                                parent_node_down = child_node_down
                    
                #cell grid random point
                self.roadman_points.append(cell.get_random_point_within_me())
                child_node_down = Node(self.roadman_points[-1])
                
                
                vec = Vector(parent_node_down.coord, child_node_down.coord)
                if vec.head[0] < vec.tail[0]:
                    if not self.do_vector_collide_with_obstacles(vec):
                        parent_node_down.add_child_node(child_node_down)
                        parent_node_down = child_node_down
            
            
            if i + 1 < len(self.up_vectors):
                # ----------------- UP VECTOR CODE ---------------
                
                first_vector_up = self.up_vectors[i]
                second_vector_up = self.up_vectors[i + 1]
                
                cell = CellGrid(((first_vector_up.tail),
                                                (first_vector_up.head),
                                                (second_vector_up.head),
                                                second_vector_up.tail))
                
                self.cell_grids.append(cell)
                
                #up vector mid point
                self.roadman_points.append(first_vector_up.get_mid_point())
                
                if parent_node_up is None:
                    parent_node_up = Node(self.roadman_points[-1])
                    vec = Vector(parent_node_down.coord, parent_node_up.coord)
                    if vec.head[0] < vec.tail[0]:
                        if not self.do_vector_collide_with_obstacles(vec):
                            parent_node_down.add_child_node(parent_node_up)
                else:
                    child_node_up = Node(self.roadman_points[-1])
                    vec = Vector(parent_node_down.coord, child_node_up.coord)
                    if vec.head[0] < vec.tail[0]:
                        if not self.do_vector_collide_with_obstacles(vec):
                            parent_node_down.add_child_node(child_node_up)
                        
                    vec = Vector(parent_node_up.coord, child_node_up.coord)
                    if vec.head[0] < vec.tail[0]:
                        if not self.do_vector_collide_with_obstacles(vec):
                            parent_node_up.add_child_node(child_node_up)
                            parent_node_up = child_node_up
                    
                #cell grid random point
                self.roadman_points.append(cell.get_random_point_within_me())
                child_node_up = Node(self.roadman_points[-1])
                
                vec = Vector(parent_node_up.coord, child_node_up.coord)
                if vec.head[0] < vec.tail[0]:
                    if not self.do_vector_collide_with_obstacles(vec):
                        parent_node_up.add_child_node(child_node_up)
                        parent_node_up = child_node_up

        
        # last down vector - LAST VECTOR TOO
        self.roadman_points.append(self.down_vectors[-1].get_mid_point())
        last_down_node = Node(self.roadman_points[-1])
        
        
        
        if not self.do_vector_collide_with_obstacles(Vector(parent_node_down.coord,
                                                      last_down_node.coord)):
            parent_node_down.add_child_node(last_down_node)
        
        # last up vector
        self.roadman_points.append(self.up_vectors[-1].get_mid_point())
        last_up_node = Node(self.roadman_points[-1])
        if not self.do_vector_collide_with_obstacles(Vector(parent_node_up.coord,
                                                      last_up_node.coord)):
            
            parent_node_up.add_child_node(last_up_node)
            
        if not self.do_vector_collide_with_obstacles(Vector(last_up_node.coord,
                                                      last_down_node.coord)):
            
            last_up_node.add_child_node(last_down_node)
            
        # Const.GOAL_NODE = parent_node_up.coord
        # print(Const.GOAL_NODE)
        
    def traverse_node_tree(self):
        print("traverse_node_tree..")
        self.open_list = []
        self.open_list.append(self.first_node)
        
        closest_start_node_found = False
        closest_goal_node_found = False
        
        
        while (not closest_start_node_found):
            
            if len(self.open_list) == 0:
                break
            
            parent_node = heapq.heappop(self.open_list)
            
            if not closest_start_node_found:
                if parent_node.coord[0] > Const.START_NODE[0]:
                    
                     if not self.do_vector_collide_with_obstacles(
                         Vector(parent_node.coord, Const.START_NODE)):
                        closest_start_node_found = True
                        self.first_node = Node(Const.START_NODE)
                        parent_node.parent = None
                        self.first_node.add_child_node(parent_node)
                        break
                    
                
            for child_node in parent_node.childNodes:
                child_node.parent = parent_node
                heapq.heappush(self.open_list, child_node)
                
                
        self.open_list = []
        self.open_list.append(self.first_node)
        
        closest_start_node_found = False
        closest_goal_node_found = False
        
        
        while (not closest_goal_node_found):
            
            if len(self.open_list) == 0:
                break
            
            parent_node = heapq.heappop(self.open_list)
            
                    
            if not closest_goal_node_found:
                if parent_node.coord[0] > Const.GOAL_NODE[0]:
                    if not self.do_vector_collide_with_obstacles(
                         Vector(parent_node.coord, Const.GOAL_NODE)):
                        closest_goal_node_found = True
                        self.solution_node = Node(Const.GOAL_NODE)
                        parent_node.parent.add_child_node(self.solution_node)
                        break
                    
                
            for child_node in parent_node.childNodes:
                child_node.parent = parent_node
                heapq.heappush(self.open_list, child_node)
     
    def is_node_in_closed_list_then_resolve(self, check_node):
        flag = False
        for node in self.closed_list:
            if (round(node.coord[0], Const.ROUND_DECIMAL_POINT) == 
                round(check_node.coord[0], Const.ROUND_DECIMAL_POINT) and
                round(node.coord[1], Const.ROUND_DECIMAL_POINT) == 
                round(check_node.coord[1], Const.ROUND_DECIMAL_POINT)):
                
                if check_node.cost2come > node.cost2come:
                    node = check_node
                    flag = True
                    break
     
    # traverse nodes to put the start node and end node connection    
    def traverse_node_tree_simple_version(self):
        print("traverse_node_tree_simple_version")
        self.open_list = []
        self.closed_list = []
        self.open_list.append(self.first_node)
        
        while self.open_list:
            parent_node = heapq.heappop(self.open_list)
            
            # if self.is_node_in_closed_list_then_resolve(parent_node):
            #     continue
            # else:
            # if parent_node.coord in self.closed_list:
            #     continue
            
            # self.closed_list.append(parent_node.coord)
            
            # if parent_node.parent is not None:
            #     self.traversal_path.append(Vector(parent_node.parent.coord, parent_node.coord))
            
            # self.total_nodes_searched += 1
            if self.is_this_goal_node(parent_node):
                self.solution_node = parent_node
                break
            
            for child_node in parent_node.childNodes:
                # child_node.parent = parent_node
                heapq.heappush(self.open_list, child_node)
                
            
               
    def is_this_goal_node(self, nodeToCheck):
        xcentre, ycentre = Const.GOAL_NODE
        x, y = nodeToCheck.coord
        in_goal = (x - xcentre)**2 + (y -ycentre)**2 - (Const.GOAL_THRESOLD)**2 < 0
        
        return in_goal
            
    def back_track(self):
        if self.solution_node is None:
            print("NO SOLUTION FOUND!!!")
        else:
            print("Backtracking...")
            tempNode = self.solution_node
            while tempNode.parent != None:
                self.traversal_path.insert(0, Vector(tempNode.parent.coord, tempNode.coord))
                tempNode = tempNode.parent
                
            #     if self.first_node.coord == tempNode.coord:
            #         break
            
            # tempNode = self.first_node
            # while tempNode.childNodes:
            #     self.traversal_path.insert(0, Vector(tempNode.coord, tempNode.childNodes[0].coord))
            #     tempNode = tempNode.childNodes[0]
                
            #     if self.is_this_goal_node(tempNode):
            #         break
            # for i in range(len(self.closed_list) - 1):
            #      self.traversal_path.insert(0, Vector(self.closed_list[i], self.closed_list[i+1]))
            
    #============== UNIT TEST CASE ===============
    def check_vector_collision_with_obstacle(self):
        vector = Vector((5.76, 7.54), (4.68, 25.94))
        
        
        print(self.obstacles[1])
        
        print(self.obstacles[1].is_inside((4.8, 15.12)))
        
        print(self.obstacles[1].does_vector_collide(vector))
        
        # flag = self.do_vector_collide_with_obstacles(vector)
        # print(f"{vector} collision for:\n Collision={flag}")
            

            
        
        

            
        
    
            
                            
