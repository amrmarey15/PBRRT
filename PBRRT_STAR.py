import copy
import numpy as np
import Periodic_KDTree
class PBRRT_STAR:
    def __init__(self, PBRRT_params: dict, map: dict, estimator_params: dict):
        self.PBRRT_params = copy.deepcopy(PBRRT_params) # copy.deepcopy points to a whole new dictionary that is identical to the original dictionary
        self.map = copy.deepcopy(map)
        self.estimator_params = copy.deepcopy(estimator_params)
        self.nodes_in_tree = Periodic_KDTree(PBRRT_params["start"], PBRRT_params["N"])
    
    def Nearest(self, sampled_node):
        return self.nodes_in_tree.nearest(sampled_node)
    
    def
        