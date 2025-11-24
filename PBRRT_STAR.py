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

        self.current_node_location = PBRRT_params["start"] #Where you are at right now
    def Nearest(self, sampled_node: Node): # Return nearest node
        return self.Tree.Nodes_in_Tree[self.Tree.nearest(sampled_node.pos)]
    
    def StaticCollisionFree(self,old_sample: Node, new_sample: Node): #Check colisions for nearest obstacles. Will assume obstacles are rectangles for the demo
        for line_sample_parameter in self.PBRRT_params["Line_Sample_Parameters"]: #How many times you want to check for collisions (usually set it to 1)
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
        new_node_np = np_nearest_sample_in_tree + self.PBRRT_params["Step_Size"]*(unit_vector)
        return Node(new_node_np.astype(float))
    
    def Near(self, sample: Node, R: float): #Determine nodes within radius R of query node
        near_nodes_indices = self.Tree.query_ball_point(sample, R)
        return self.Tree.Nodes_in_Tree[near_nodes_indices]
    
    def DynamicCollisionFree(self, sample_arrival: Node):
        min_probability_collision = 1 #minimum probability of collision initialization
        best_k_arrival_time = -1
        for k_arrival in range(self.PBRRT_params["K_limits"]):
            no_collision_P_list = np.empty(len(DynamicObstacle.all)) #This list will contain the probabilities that collision will happen with each one of the obstacles at arrival time
            for dynamic_obstacle, i in zip(DynamicObstacle.all, range(len(DynamicObstacle.all))):
                estimator = dynamic_obstacle.estimator
                P_obstacle = estimator.calculate_probability_of_collision(sample_arrival, k_arrival) #Probability obstacle will cause collision at node
                no_collision_P_list[i] = 1 - P_obstacle #Probability of no collision
            probabilty_collision = 1 - np.prod(no_collision_P_list)
            if probabilty_collision < min_probability_collision:
                min_probability_collision = probabilty_collision
                best_k_arrival_time = k_arrival
            
        return min_probability_collision, best_k_arrival_time

    def calc_num_generations_to_ancestor_node(child_node, ancestor_node):
        num_generations = 0
        while child_node != ancestor_node:
            child_node = child_node.parent
            num_generations = num_generations + 1
            if child_node == None:
                raise Exception("Nodes are not connected")
        return num_generations
    

        
    
            
        
    
    def initial_plan(self): # Developing the path without a prior tree or path considered
        map_size = np.array(self.map["Map_Size"])
        map_dim  = len(map_size)
        max_iter = self.PBRRT_params["Max_Iterations"]
        M = self.PBRRT_params["M"]
        gamma = self.PBRRT_params["gamma"]
        P_coll = self.PBRRT_params["P_coll"] #
        R = self.PBRRT_params["R"] #radius of search
        prod_gamma_numgen_prob_param =  self.PBRRT_params["alpha"] #if gamma**num_generations is less than this number no point in computing the probability because it will begin to have no effect
        for i in range(max_iter):
            sampled_node = Node(np.random.rand(map_dim) @ map_size)
            nearest_sample_in_tree = self.Nearest(sampled_node)
            new_sample = self.Steer(nearest_sample_in_tree, sampled_node)
            if self.StaticCollisionFree(nearest_sample_in_tree, new_sample): #If there is no static obstacle collision from the Steer earlier (Still need to check dynamic obstacles)
                num_generations = self.calc_num_generations(self.current_node_location, nearest_sample_in_tree) #Number of generations ahead of where robot is at so far
                if gamma**num_generations < prod_gamma_numgen_prob_param:
                    prob_collision = 0 #We are so far ahead in the node generation planning that we have no idea if collisions will actually happen or not
                    best_k_arrival_time = 1
                    c_line = np.linalg.norm(new_sample.pos - nearest_sample_in_tree.pos)
                else:
                    prob_collision, best_k_arrival_time = self.DynamicCollisionFree(new_sample)
                    if prob_collision > P_coll:
                        continue
                    else:
                        c_line = prob_collision*M*(gamma**num_generations) + (1-prob_collision*(gamma**num_generations))*np.linalg.norm(new_sample.pos - nearest_sample_in_tree.pos)
                near_nodes_list = self.Near(new_sample, R)
                self.Tree.add_point(new_sample)
                new_sample.parent = nearest_sample_in_tree
                new_sample.node_prob_collision = prob_collision
                new_sample.k_star = best_k_arrival_time
                sample_min = nearest_sample_in_tree
                cost_min = nearest_sample_in_tree.cost + c_line
                new_sample.cost = cost_min
                for node in near_nodes_list:
                    if self.StaticCollisionFree(node, new_sample):
                        c_line = prob_collision*M*(gamma**num_generations) + (1-(self.gamma**num_generations)*prob_collision)*np.linalg.norm(node.pos - new_sample.pos)
                        cost = new_sample.cost + c_line
                        if node.cost < cost:
                            node.parent = new_sample
                            node.cost = cost
                            node.k_star = best_k_arrival_time
                            node.node_prob_collision = prob_collision
                

                        

                        




            