import numpy as np
from StaticObstacle import *
from DynamicObstacle import *
from Node import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import *
from PBRRT_STAR import *
import time

np.random.seed(42)


   

obstacle1 = StaticObstacle([7, 13, 8, 20])
obstacle2 = StaticObstacle([7, 13, 0, 6])



PBRRT_params = {
    "K_limits": 20,
    "start": Node([1,18]),
    "goal": Node([19,19]),
    "N": 1e3,
    "Line_Sample_collision_checks": 3,
    "Step_Size": 0.25,
    "P_coll": 0.2,
    "Final_Radius_Limit": 0.25,
    "M": 0,
    "gamma": 0.9,
    "Max_Iterations": int(5e3),
    "alpha": 0.01,
    'map_size': [20,20],
    'static_obstacles_list': [obstacle1, obstacle2],
    'num_d_obstacles': 10,
    'R': 1
}



estimator_params = {
    'Q': 0.05*np.eye(len(PBRRT_params["map_size"])),
    'V': 0.05*np.eye(len(PBRRT_params["map_size"])),
    'P_Post': 0.05*np.eye(len(PBRRT_params["map_size"]))
}

circles = generate_circle_dynamic_obstacles(PBRRT_params, estimator_params)
initial_generate_map_2D(PBRRT_params, estimator_params)

PBRRT_instance = PBRRT_STAR(PBRRT_params, estimator_params)
print("YEEE")
PBRRT_instance.RunPBRRT()
#show_init_path2D(PBRRT_params, estimator_params, PBRRT_instance.path)
#show_init_tree(PBRRT_instance, PBRRT_params, estimator_params)