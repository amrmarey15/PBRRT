import copy
import numpy as np
from Periodic_KDTree import *
from Node import *
from StaticObstacle import *
from DynamicObstacle import *
import random
from KF_Circular_Obstacle_Pos_Estimator import *
class PBRRT_STAR:
    def __init__(self, PBRRT_params: dict, map: dict, estimators: list):
        self.PBRRT_params = copy.deepcopy(PBRRT_params) # copy.deepcopy points to a whole new dictionary that is identical to the original dictionary
        self.map = copy.deepcopy(map)
        self.estimators = copy.deepcopy(estimators)
        self.Tree = Periodic_KDTree(PBRRT_params["start"], PBRRT_params["N"])
    
    def Nearest(self, sampled_node: Node): # Return index for nearest node
        return self.Tree.Nodes_in_Tree[self.Tree.nearest(sampled_node.pos)]
    
    def StaticCollisionFree(self,old_sample: Node, new_sample: Node): #Check colisions for nearest obstacles. Will assume obstacles are rectangles for the demo
        for line_sample_parameter in self.line_sample_parameters: #How many times you want to check for collisions (usually set it to 1)
            x_coordinate = old_sample.x + line_sample_parameter*(new_sample.x - old_sample.x)
            y_coordinate = old_sample.y + line_sample_parameter*(new_sample.y - old_sample.y)
            for static_obstacle in StaticObstacle.all:
                if((x_coordinate> static_obstacle.x_min and x_coordinate < static_obstacle.x_max) and (y_coordinate> static_obstacle.y_min and y_coordinate < static_obstacle.y_max)):
                    return False
        return True
    
    def Steer(self, nearest_sample_in_tree: Node, sampled_node: Node): #Steer towards sampled node (Like RRT*)
        np_nearest_sample_in_tree = nearest_sample_in_tree.pos
        np_sampled_node = sampled_node.pos
        unit_vector = (np_sampled_node - np_nearest_sample_in_tree)/np.linalg.norm(np_nearest_sample_in_tree - np_sampled_node)
        new_node_np = np_nearest_sample_in_tree + self.step_size*(unit_vector)
        return Node(float(new_node_np[0]), float(new_node_np[1]))
    
    def Near(self, sample: Node, R: float): #Determine nodes within radius R of query node
        near_nodes_indices = self.Tree.query_ball_point(sample, R)
        return self.Tree.Nodes_in_Tree[near_nodes_indices]
    
    def DynamicCollisionFree(self, sample_arrival: Node):
        min_probability_collision = 1 #minimum probability of collision initialization
        best_k_arrival_time = -1
        for k_arrival in range(self.PBRRT_params["K_limits"]):
            no_collision_P_list = np.empty(len(DynamicObstacle.all)) #This list will contain the probabilities that collision will happen with each one of the obstacles at arrival time
            for estimator, i in zip(KF_Circular_Obstacle_Pos_Estimator.all, range(len(DynamicObstacle.all))):
                P_obstacle = estimator.calculate_probability_of_collision(sample_arrival, k_arrival) #Probability obstacle will cause collision at node
                no_collision_P_list[i] = 1 - P_obstacle #Probability of no collision
            probabilty_collision = 1 - np.prod(no_collision_P_list)
            if probabilty_collision < min_probability_collision:
                min_probability_collision = probabilty_collision
                best_k_arrival_time = k_arrival
            
        return min_probability_collision, best_k_arrival_time

                
            
        
    
    def initial_plan(self): # Developing the path without a prior tree or path considered
        map_size = self.PBRRT_params["Map_Size"]
        for i in range(self.PBRRT_params["Max_Iterations"]):
            sampled_node = Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))
            nearest_sample_in_tree = self.Nearest(sampled_node)
            new_sample = self.Steer(nearest_sample_in_tree, sampled_node)
            if self.StaticCollisionFree(nearest_sample_in_tree, new_sample): #If there is no static obstacle collision from the Steer earlier (Still need to check dynamic obstacles)
                prob_collision, best_k_arrival_time = self.DynamicCollisionFree(new_sample)
                if prob_collision > self.PBRRT_params["P_coll"]:
                    break
            