from dataclasses import dataclass
from typing_extensions import Self

from source.cell import CellGrid
from . boundary import Boundary
from . obstacle import Obstacle
from . constants import Const
from . vector import Vector
from . utility import print_partition

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
        
        self.random_points_in_grid = []
        
    def is_in_obstacle_space(self, pt):
        flag = False
        for obstacle in self.obstacles:
            if obstacle.is_inside(pt):
                flag = True
                break
            
        return flag
        
        
    # ToDo: Sweep Edges
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
        
        self.sample_random_points_in_grid()
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
            
    # def create_grids(self):
        
    def sample_random_points_in_grid(self):
        
        for i in range(len(self.up_vectors) - 1):
            first_up_vector = self.up_vectors[i]
            second_up_vector = self.up_vectors[i + 1]
            
            cell = CellGrid(((first_up_vector.tail),
                                            (first_up_vector.head),
                                            (second_up_vector.head),
                                            second_up_vector.tail))
            
            self.cell_grids.append(cell)
            
            self.random_points_in_grid.append(cell.get_random_point_within_me())
        
    
            
                            
