import copy
import numpy as np
from Periodic_KDTree import *
from Node import *
from StaticObstacle import *
from DynamicObstacle import *
import random
from KF_Circular_Obstacle_Pos_Estimator import *
import time
from utils import *
class PBRRT_STAR:
    def __init__(self, PBRRT_params: dict, estimator_params: dict):
        self.PBRRT_params = copy.deepcopy(PBRRT_params) # copy.deepcopy points to a whole new dictionary that is identical to the original dictionary
        self.estimator_params = copy.deepcopy(estimator_params)
        self.Tree = Periodic_KDTree(PBRRT_params["start"], int(PBRRT_params["N"]), max_samples_in_tree = 1_000_000)
        self.current_node_location = PBRRT_params["start"] #Where you are at right now
        self.Line_Sample_collision_checks = np.linspace(0,1,self.PBRRT_params["Line_Sample_collision_checks"])
        self.path_planned = [] #Path we plan to execute (nothing yet)
        self.path_executed = [] #
        self.num_replans = -1
        self.k_current = 0 #real time under execution
        self.rng = np.random.default_rng(seed=42)
        self.p_collision_list = []

    def Nearest(self, sampled_node: Node): # Return nearest node
        return self.Tree.Nodes_in_Tree[self.Tree.nearest(sampled_node)]

    def StaticCollisionFree(self,old_sample: Node, new_sample: Node): #Check colisions for nearest obstacles. Will assume obstacles are rectangles for the demo
        for line_sample_parameter in self.Line_Sample_collision_checks: #How many times you want to check for collisions (usually set it to 1)
            vec = np.array(old_sample.pos) + line_sample_parameter*(new_sample.pos - old_sample.pos)
            for static_obstacle in StaticObstacle.all:
                if((vec[0]> static_obstacle.x_min and vec[0] < static_obstacle.x_max) and (vec[1] > static_obstacle.y_min and vec[1] < static_obstacle.y_max)): #2D case only
                    return False
        return True
    
    def Steer(self, nearest_sample_in_tree: Node, sampled_node: Node): #Steer towards sampled node (Like RRT*)
        if isinstance(nearest_sample_in_tree, Node):
            np_nearest_sample_in_tree = nearest_sample_in_tree.pos
        else:
            np_nearest_sample_in_tree = nearest_sample_in_tree

        np_sampled_node = sampled_node.pos
        unit_vector = (np_sampled_node - np_nearest_sample_in_tree)/np.linalg.norm(np_nearest_sample_in_tree - np_sampled_node)
        new_node_np = np_nearest_sample_in_tree + self.PBRRT_params["Step_Size"]*(unit_vector)
        new_node = Node(new_node_np)
        return new_node
    
    def Near(self, sample: Node, R: float): #Determine nodes within radius R of query node
        near_nodes_indices = self.Tree.query_ball_point(sample, R)
        return [self.Tree.Nodes_in_Tree[i] for i in near_nodes_indices]
    
    def DynamicCollisionFree(self, sample_arrival: Node, k_start: int):
        min_probability_collision = 1 #minimum probability of collision initialization
        best_k_arrival_time = -1
        for k in range(1, self.PBRRT_params["K_limits"], 1):
            k_arrival = k + k_start
            no_collision_P_list = np.empty(len(DynamicObstacle.all)) #This list will contain the probabilities that collision will happen with each one of the obstacles at arrival time
            for dynamic_obstacle, i in zip(DynamicObstacle.all, range(len(DynamicObstacle.all))):
                P_obstacle = dynamic_obstacle.estimator.calculate_probability_of_collision(sample_arrival.pos, k_arrival, dynamic_obstacle.locations_list[k_arrival]) #Probability obstacle will cause collision at node

                no_collision_P_list[i] = 1 - P_obstacle #Probability of no collision
            probabilty_collision = 1 - np.prod(no_collision_P_list)
            if probabilty_collision < min_probability_collision:
                min_probability_collision = probabilty_collision
                best_k_arrival_time = k_arrival
                if min_probability_collision < 0.01 and best_k_arrival_time > 0: #If you find a good enough solution, return that instead to save time
                    return min_probability_collision, best_k_arrival_time - k_start




            k = best_k_arrival_time - k_start
        return min_probability_collision, k

    def calc_num_generations_to_ancestor_node(self, ancestor_node, child_node):
        num_generations = 0
        while child_node != ancestor_node:
            child_node = child_node.parent
            num_generations = num_generations + 1
            if child_node == None:
                raise Exception("Nodes are not connected")
        return num_generations
    
    def calc_time_between_two_nodes(self, ancestor_node: Node, child_node: Node):
        k_total = 0
        while child_node != ancestor_node:
            k_total = k_total + child_node.k_star
            child_node = child_node.parent
            if child_node == None:
                raise Exception("Nodes not connected! You can't calculate time between these two nodes")
        return k_total
    
    def is_goal_reached(self, sample: Node):
        if np.linalg.norm(self.PBRRT_params["goal"].pos - sample.pos) < self.PBRRT_params["Final_Radius_Limit"] and sample.parent !=None:
            return True
        else:
            return False
    
    def CalcFinalPath(self, initial_sample, final_sample):
        path = []
        if not self.is_goal_reached(final_sample):
            print("Path not reached")
            return path
        else:
            node = final_sample
            while node != initial_sample:
                path.append(node)
                node = node.parent
            path.append(node)
            path.reverse()
            return path
    

        
    def CheckPathandFix(self): #Returns False and deletes nodes of Tree if path is broken, other wise returns True if proposed path (self.path_planned) is still good
        if np.linalg.norm(self.PBRRT_params["goal"].pos - self.current_node_location.pos) < self.PBRRT_params["Final_Radius_Limit"]:
            return False
        gen = -1
        k_start = 0
        path_broken_index = -1
        for i in range(len(self.path_planned)-1):
            prob_collision, k_star = self.DynamicCollisionFree(self.path_planned[i+1], k_start+self.path_planned[i+1].k_star)
            gen = gen + 1
            if prob_collision*(self.PBRRT_params["gamma"]**i) < self.PBRRT_params["P_coll"]:
                self.path_planned[i+1].node_prob_collision = prob_collision
                self.path_planned[i+1].k_star = k_star
                updated_cost_line = prob_collision*self.PBRRT_params["M"]*(self.PBRRT_params["gamma"]**gen) + (1-prob_collision*(self.PBRRT_params["gamma"]**gen))*np.linalg.norm(self.path_planned[i+1].pos - self.path_planned[i].pos)
                self.path_planned[i+1].cost = self.path_planned[i].cost + updated_cost_line
                self.path_planned[i+1].cost_recomp = self.path_planned[i+1].cost_recomp + 1
                k_start = k_start + k_star
            else:
                path_broken_index = i
                break
        
        if path_broken_index > 0:
            broken_path = self.path_planned[(i+1):]
            broken_path.reverse()
            for node in broken_path: #Note broken path is reversed here
                for node_child in node.children:
                    self.Tree.remove_point_from_tree(node_child)
                node.children = []
                self.Tree.remove_point_from_tree(node)
                node.parent.children.remove(node)
            return False

        return True




    def ExecuteStep(self):
        if len(self.path_planned) <= 1:
            self.current_node_location = self.path_planned[0]
            return
        
        self.current_node_location = self.path_planned[0] #Initially Child
        self.path_executed.append(self.current_node_location)
        self.path_planned = self.path_planned[1:]
        #self.current_node_location = self.path_planned[0] #Initially Child
        self.current_node_location.k_star_exec = self.current_node_location.k_star
        self.current_node_location.k_star = 0
        self.current_node_location.cost = 0 #Start should always have zero cost
        self.current_node_location.parent = None
        for Dobs in DynamicObstacle.all:
            del Dobs.pos_list[0:self.current_node_location.k_star_exec]
        # i = 0
        # for Dobs in DynamicObstacle.all:
        #     Dobs.pos = self.rng.multivariate_normal(Dobs.pos, self.estimator_params['Q'])
        #     Dobs.pos_list.append(Dobs.pos)
        #     KF_Circular_Obstacle_Pos_Estimator.all[i].gaussian_estimate(Dobs.pos)
        #     i = i+1
        





        # node_to_remove = self.current_node_location
        # node_to_remove_children = self.current_node_location.children
        # node_to_remove_children.remove(self.path_planned[1])
        # subtree_remove_nodes = []
        # for node in node_to_remove_children:
        #     subtree_remove_nodes.append(self.Tree.collect_subtree_bfs(node))
        # subtree_remove_nodes.append


    

        # for node in node_to_remove_children:
        #     remove_subtree(node)

            
        # node_to_remove.children = []
        # self.Tree.remove_point_from_tree(node_to_remove)
        

















        ####### Will delete probably

        # self.prev_start = self.current_node_location #initially Parent
        # self.prev_start.parent = self.path_planned[1]
        # self.path_executed.append(self.current_node_location)
        # self.path_planned = self.path_planned[1:]
        # self.current_node_location = self.path_planned[0] #Initially Child
        # self.current_node_location.cost = 0 #Start should always have zero cost
        # self.current_node_location.cost_recomp = self.current_node_location.cost_recomp + 1

        #####


        

    
    def RunPBRRT(self):
        while True:
            if np.linalg.norm(self.PBRRT_params["goal"].pos - self.current_node_location.pos) < self.PBRRT_params["Final_Radius_Limit"]:
                return 0
            self.num_replans = self.num_replans + 1
            self.path_planned = self.InitialPlan()
            print(self.current_node_location)
            while self.CheckPathandFix():
                if self.is_goal_reached(self.current_node_location):
                    return 0
                self.ExecuteStep()
                i = 0
                k = 0
                while i < 50 and i < len(self.path_planned):
                    k = k + self.path_planned[i].k_star
                    for Dobs in DynamicObstacle.all:
                        if (np.linalg.norm(Dobs.pos_list[k] - self.path_planned[i].pos)) < Dobs.r + self.PBRRT_params["Step_Size"]:
                            self.path_planned = self.InitialPlan()
                    i = i + 1
                print(self.current_node_location)
                
        






    



        

    
            
        
    
    def InitialPlan(self): # Developing the path without a prior tree or path considered
        self.Tree = Periodic_KDTree(self.current_node_location, int(self.PBRRT_params["N"]), max_samples_in_tree = 1_000_000)
        map_size = np.array(self.PBRRT_params["map_size"])
        map_dim  = len(map_size)
        max_iter = self.PBRRT_params["Max_Iterations"]
        M = self.PBRRT_params["M"]
        gamma = self.PBRRT_params["gamma"]
        P_coll = self.PBRRT_params["P_coll"] #
        R = self.PBRRT_params["R"] #radius of search
        prod_gamma_numgen_prob_param =  self.PBRRT_params["alpha"] #if gamma**num_generations is less than this number no point in computing the probability because it will begin to have no effect
        path_final_samples_initial_plan = []

        for i in range(max_iter):
            
            sampled_node = Node(np.random.rand(map_dim) * map_size)
            nearest_sample_in_tree = self.Nearest(sampled_node)
            new_sample = self.Steer(nearest_sample_in_tree, sampled_node)
            print(i)
            if self.StaticCollisionFree(nearest_sample_in_tree, new_sample): #If there is no static obstacle collision from the Steer earlier (Still need to check dynamic obstacles)
                
                num_generations = self.calc_num_generations_to_ancestor_node(self.current_node_location, nearest_sample_in_tree) #Number of generations ahead of where robot is at so far
                
                if gamma**num_generations < prod_gamma_numgen_prob_param:
                    prob_collision = 0 #We are so far ahead in the node generation planning that we have no idea if collisions will actually happen or not
                    best_k_arrival_time = 1
                    c_line = np.linalg.norm(new_sample.pos - nearest_sample_in_tree.pos)
                else:
                    
                    k_start = self.calc_time_between_two_nodes(self.current_node_location, nearest_sample_in_tree)
                    prob_collision, best_k_arrival_time = self.DynamicCollisionFree(new_sample, k_start)
                    if np.abs(prob_collision) < 0.01:
                        prob_collision = 0 #Prevent numerical instabilities

                    if prob_collision > P_coll:
                        continue
                    else:
                        c_line = prob_collision*M*(gamma**num_generations) + (1-prob_collision*(gamma**num_generations))*np.linalg.norm(new_sample.pos - nearest_sample_in_tree.pos)
                near_nodes_list = self.Near(new_sample, R)


                sample_min = nearest_sample_in_tree
                cost_min = nearest_sample_in_tree.cost + c_line
                new_sample.parent = sample_min
                new_sample.cost = cost_min
                for node in near_nodes_list:
                    if self.StaticCollisionFree(node, new_sample):
                        c_line = prob_collision*M*(gamma**num_generations) + (1-(gamma**num_generations)*prob_collision)*np.linalg.norm(node.pos - new_sample.pos)
                        cost = new_sample.cost + c_line
                        if cost < cost_min and node.parent != new_sample:
                            sample_min = node
                            cost_min = cost
                
                new_sample.parent = sample_min
                sample_min.children.append(new_sample)
                new_sample.cost = cost_min
                new_sample.node_prob_collision = prob_collision
                new_sample.k_star = best_k_arrival_time
                self.Tree.add_point(new_sample)

                k_start = self.calc_time_between_two_nodes(self.current_node_location, new_sample)
                num_generations = self.calc_num_generations_to_ancestor_node(self.current_node_location, new_sample)

                for node in near_nodes_list:   
                    prob_collision, best_k_arrival_time = self.DynamicCollisionFree(node, k_start)
                    if self.StaticCollisionFree(new_sample, node) and prob_collision < P_coll:
                        c_line = prob_collision*M*(gamma**num_generations) + (1-prob_collision*(gamma**num_generations))*np.linalg.norm(node.pos - new_sample.pos)
                        cost = new_sample.cost + c_line
                        if  cost < node.cost and new_sample.parent != node: #Check later
                            if node.parent.children:
                                node.parent.children.remove(node)

                            node.parent = new_sample
                            new_sample.children.append(node)
                            node.cost = cost
                            node.k_star = best_k_arrival_time
                            node.node_prob_collision = prob_collision

            if self.is_goal_reached(new_sample):
                if self.num_replans > 50: #Change and test
                    return self.CalcFinalPath(self.current_node_location, new_sample)
                else:
                    path_final_samples_initial_plan.append(new_sample)
            
        sample_min = path_final_samples_initial_plan[0]
        for node in path_final_samples_initial_plan:
            if node.cost < sample_min.cost:
                sample_min = node
        
        return self.CalcFinalPath(self.current_node_location, sample_min)


            
        
        





            
            
            
                
            
            

                

                        

                        




            