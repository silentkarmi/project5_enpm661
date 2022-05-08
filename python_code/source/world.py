from dataclasses import dataclass
from doctest import TestResults
from typing_extensions import Self

from soupsieve import closest

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
        
        file_path = 'source/forest100.yaml' #10, 20, 30, 50

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
        self.first_node_in_graph = None
        

        self.solution_node = None
        
        self.total_nodes_searched = 0
        
        self.before_smoothening_way_points = []
        self.way_points = []
        
    def object_in_interest(self, vec_path):
        objects_in_interest = []
        
        x1_pt, y1_pt = vec_path.head
        x2_pt, y2_pt = vec_path.tail
        
        x1 = min(x1_pt, x2_pt) - Const.TREE_RADIUS * 2
        x2 = max(x1_pt, x2_pt) + Const.TREE_RADIUS * 2
        
        # y1 = min(y1_pt, y2_pt)
        # y2 = max(y1_pt, y2_pt)
        
        for obstacle in self.obstacles:
            is_object_of_interest = False
            for vector in obstacle.vectors:
                if not is_object_of_interest:
                    x,y = vector.head
                    if (x >= x1 and x <= x2):
                        objects_in_interest.append(obstacle)
                        is_object_of_interest = True
                    
        
        return objects_in_interest
        
    def is_in_obstacle_space(self, pt):
        flag = False
        for obstacle in self.obstacles:
            if obstacle.is_inside(pt):
                flag = True
                break
            
        return flag
    
    def nearest_obstacle(self, pt):
        min_len = float('inf')
        closest_obstacle = None
        for obstacle in self.obstacles:
            diff = (obstacle.vectors[0].head[0] - pt[0])
            
            len = 0
            len += abs(diff)
            len += abs(obstacle.vectors[0].head[1] - pt[1])
            if len < min_len and not obstacle.already_traversed and diff > 0:
                min_len = len
                closest_obstacle = obstacle
                
        closest_obstacle.already_traversed = True
        near_tree_pt = (closest_obstacle.vectors[0].head[0] - Const.JITTER_VALUE,
                        closest_obstacle.vectors[0].head[1])
        return near_tree_pt
                
        
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
            
    def do_vector_collide_with_obstacles(self, vector, resolution = Const.RESOLUTION * 0.05):
        flag = False
        obects_of_interest = self.object_in_interest(vector)
        for obstacle in obects_of_interest:
            if obstacle.does_vector_collide(vector, resolution):
                flag = True
                break
            
        return flag
            
        
    def sample_roadman_points_and_create_node_tree(self):
        print("Creating sample_roadman_points_and_create_node_tree..")
        
        parent_node_down = None
        child_node_down = None
        
        parent_node_up = None
        child_node_up = None
        first_node_in_graph_up = None
        
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
                    self.first_node_in_graph = parent_node_down
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
        
    def traverse_node_tree(self,first_node_in_graph, start_coord, goal_coord):
        # print("traverse_node_tree..")
        
        open_list = []
        open_list.append(first_node_in_graph)
        
        closest_start_node_found = False
        closest_goal_node_found = False
        
        solution_node = None
        
        
        while (not closest_start_node_found):
            
            if len(open_list) == 0:
                break
            
            parent_node = heapq.heappop(open_list)
            
            if not closest_start_node_found:
                if parent_node.coord[0] > start_coord[0]:
                    
                     if not self.do_vector_collide_with_obstacles(
                         Vector(parent_node.coord, start_coord)):
                        closest_start_node_found = True
                        first_node_in_graph = Node(start_coord)
                        parent_node.parent = None
                        first_node_in_graph.add_child_node(parent_node)
                        break
                    
                
            for child_node in parent_node.childNodes:
                child_node.parent = parent_node
                heapq.heappush(open_list, child_node)
                
                
                
        open_list = []
        open_list.append(first_node_in_graph)
        
        closest_start_node_found = False
        closest_goal_node_found = False
        
        
        while (not closest_goal_node_found):
            
            if len(open_list) == 0:
                break
            
            parent_node = heapq.heappop(open_list)
            
                    
            if not closest_goal_node_found:
                if parent_node.coord[0] > goal_coord[0]:
                    if not self.do_vector_collide_with_obstacles(
                         Vector(parent_node.coord, goal_coord)):
                        closest_goal_node_found = True
                        solution_node = Node(goal_coord)
                        parent_node.parent.add_child_node(solution_node)
                        break
                    
                
            for child_node in parent_node.childNodes:
                child_node.parent = parent_node
                heapq.heappush(open_list, child_node)
                
        return solution_node                         
               
    def is_this_goal_node(self, nodeToCheck):
        xcentre, ycentre = Const.GOAL_NODE
        x, y = nodeToCheck.coord
        in_goal = (x - xcentre)**2 + (y -ycentre)**2 - (Const.GOAL_THRESOLD)**2 < 0
        
        return in_goal
            
    def back_track(self, solution_node):
        traversal_path = []
        if solution_node is None:
            print("NO SOLUTION FOUND!!!")
        else:
            # print("Backtracking...")
            tempNode = solution_node
            while tempNode.parent != None:
                traversal_path.insert(0, Vector(tempNode.parent.coord, tempNode.coord))
                tempNode = tempNode.parent
                
        return traversal_path
                
    def generate_solution_path(self, traversal_path):
        solution_path = []
        for segment in traversal_path:
            solution_path.append(segment.tail)
            
        return solution_path
            
    def generate_way_points(self, final_start_coord, final_goal_coord, trees_required):
        print("Generating Path...")
        if self.is_in_obstacle_space(final_start_coord):
            print("INVALID START NODE")
            return
    
        if self.is_in_obstacle_space(final_goal_coord):
            print("INVALID GOAL NODE")
            return
        
        if not self.boundary.is_inside(final_start_coord):
            print("START NODE OUTSIDE BOUNDARY")
        
            
        if not self.boundary.is_inside(final_goal_coord):
            print("GOAL NODE OUTSIDE BOUNDARY") 
        
        final_way_points = []
        start_coord = final_start_coord
        first_node_in_graph = self.first_node_in_graph
        trees_done = 1
        count = 0
        print("Finding path to trees...")
        while trees_done < trees_required:
            count += 1
            if count == 50:
                break
            
            goal_coord = self.nearest_obstacle(start_coord)
            
            if self.is_in_obstacle_space(goal_coord):
                continue
            
            if not self.do_vector_collide_with_obstacles(
                         Vector(start_coord, goal_coord), Const.RESOLUTION * 0.005):
                # print(f"Find Path between:{start_coord} and {goal_coord}")
                final_way_points.append(start_coord)
                final_way_points.append(goal_coord)
                start_coord = goal_coord
                # first_node_in_graph = solution_node
                # first_node_in_graph.add_child_node(solution_node.parent.childNodes[0])
                trees_done += 1
            else:
                continue
            
        print("Finding path to final goal node...")
        solution_node = self.traverse_node_tree(first_node_in_graph, start_coord, final_goal_coord)
        if solution_node:
            path = self.back_track(solution_node)
            way_points = self.generate_solution_path(path)
            self.before_smoothening_way_points = way_points
            
            print("Smoothening waypoints...")
            
            new_way_points = self.smoothen_waypoints(way_points)
            
            for number_of_time in range(Const.SMOOTHEN_STRENGTH - 1):
                new_way_points = self.smoothen_waypoints(new_way_points)

            final_way_points.extend(new_way_points)
            
            self.way_points = final_way_points
            print(final_way_points)
            
            self.way_points.insert(0, final_start_coord)
            
            self.solution_node = solution_node
        else:
            print("PATH SOLUTION not FOUND")    
        
        
        return final_way_points
    
    def smoothen_waypoints(self, waypts):
        new_waypts = []
        for i in range(0, len(waypts) - 2, 3):
            first_pt = waypts[i]
            second_pt = waypts[i + 1]
            third_pt = waypts[i + 2]
            vec = Vector(first_pt, third_pt)
            if not self.do_vector_collide_with_obstacles(vec, Const.RESOLUTION * 0.009):
                new_waypts.append(first_pt)
                new_waypts.append(third_pt)
            else:
                new_waypts.append(first_pt)
                new_waypts.append(second_pt)
                new_waypts.append(third_pt)
                
        if third_pt != waypts[-1]:
            new_waypts.append(waypts[-1])
            
        return new_waypts
     
     
     
     
     
             
    #============== UNIT TEST CASE ===============
    def check_vector_collision_with_obstacle(self):
        vector = Vector((5.76, 7.54), (4.68, 25.94))
        
        
        print(self.obstacles[1])
        
        print(self.obstacles[1].is_inside((4.8, 15.12)))
        
        print(self.obstacles[1].does_vector_collide(vector))
        
        # flag = self.do_vector_collide_with_obstacles(vector)
        # print(f"{vector} collision for:\n Collision={flag}")
            

            
        
        

            
        
    
            
                            
