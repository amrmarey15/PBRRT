import copy
import numpy as np
import Periodic_KDTree
from Node import Node
from StaticObstacle import StaticObstacle

class PBRRT_STAR:
    def __init__(self, PBRRT_params: dict, map: dict, estimator_params: dict):
        self.PBRRT_params = copy.deepcopy(PBRRT_params) # copy.deepcopy points to a whole new dictionary that is identical to the original dictionary
        self.map = copy.deepcopy(map)
        self.estimator_params = copy.deepcopy(estimator_params)
        self.nodes_in_tree = Periodic_KDTree(PBRRT_params["start"], PBRRT_params["N"])
    
    def Nearest(self, sampled_node): # Return index for nearest node
        return Node.all[self.nodes_in_tree.nearest(sampled_node.pos)]
    
    def StaticCollisionFree(self,old_sample, new_sample): #Check colisions for nearest obstacles. Will assume obstacles are rectangles for the demo
        for line_sample_parameter in self.line_sample_parameters:
            x_coordinate = old_sample.x + line_sample_parameter*(new_sample.x - old_sample.x)
            y_coordinate = old_sample.y + line_sample_parameter*(new_sample.y - old_sample.y)
            for static_obstacle in StaticObstacle.all:
                if((x_coordinate> static_obstacle.x_min and x_coordinate < static_obstacle.x_max) and (y_coordinate> static_obstacle.y_min and y_coordinate < static_obstacle.y_max)):
                    return False
        return True
    
    def Steer(self, nearest_sample_in_tree, sampled_node): #Steer towards sampled node (Like RRT*)
        np_nearest_sample_in_tree = nearest_sample_in_tree.pos
        np_sampled_node = sampled_node.pos
        unit_vector = (np_sampled_node - np_nearest_sample_in_tree)/np.linalg.norm(np_nearest_sample_in_tree - np_sampled_node)
        new_node_np = np_nearest_sample_in_tree + self.step_size*(unit_vector)
        return Node(float(new_node_np[0]), float(new_node_np[1]))
    
    def Near(self, sample, R):
        near_nodes_indices = self.nodes_in_tree.query_ball_point(sample.pos, R)
        


