import copy
class PBRRT_STAR:
    def __init__(self, RRT_params: dict, map: dict, estimator_params: dict):
        self.RRT_params = copy.deepcopy(RRT_params) # copy.deepcopy points to a whole new dictionary that is identical to the original dictionary
        self.map = copy.deepcopy(map)
        self.estimator_params = copy.deepcopy(estimator_params)
        
        