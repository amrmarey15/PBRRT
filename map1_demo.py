import numpy as np
from StaticObstacle import *
from DynamicObstacle import *
from Node import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import *

np.random.seed(42)


   

obstacle1 = StaticObstacle([7, 13, 8, 20])

obstacle2 = StaticObstacle([7, 13, 0, 6])



PBRRT_params = {
    "K_limits": 20,
    "start": Node(1,1),
    "goal": Node(19,19),
    "N": 1e3,
    "Line_Sample_Parameters": 3,
    "Step_Size": 0.2,
    "P_coll": 0.5,
    "Final_Radius_Limit": 0.2,
    "M": 2,
    "gamma": 0.9,
    "Max_Iterations": 1e6,
    "alpha": 0.01,
    'map_size': [20,20],
    'static_obstacles_list': [obstacle1, obstacle2],
    'num_d_obstacles': 10
}



estimator_params = {
    'Q': 0.05*np.eye(len(PBRRT_params["map_size"])),
    'V': 0.05*np.eye(len(PBRRT_params["map_size"])),
    'P_Post': 0.05*np.eye(len(PBRRT_params["map_size"]))
}
