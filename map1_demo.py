import numpy as np
from StaticObstacle import *
from DynamicObstacle import *
from Node import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import *
from PBRRT_STAR import *
import time
from gif_save import *
from results_script import *


   

obstacle1 = StaticObstacle([7, 13, 10, 20])
obstacle2 = StaticObstacle([7, 13, 0, 6])



PBRRT_params = {
    "K_limits": 2,
    "start": Node([0,18]),
    "goal": Node([18,0]),
    "N": 1e3,
    "Line_Sample_collision_checks": 3,
    "Step_Size": 0.25,
    "P_coll": 0.05,
    "Final_Radius_Limit": 2,
    "M": 5,
    "gamma": 0.9,
    "Max_Iterations": int(1e4),
    "alpha": 0.0001,
    'map_size': [20,20],
    'static_obstacles_list': [obstacle1, obstacle2],
    'num_d_obstacles': 8,
    'R': 0.25
}



estimator_params = {
    'Q': 0.1*np.eye(len(PBRRT_params["map_size"])),
    'V': 0.1*np.eye(len(PBRRT_params["map_size"])),
    'P_Post': 0.1*np.eye(len(PBRRT_params["map_size"]))
}

circles = generate_circle_dynamic_obstacles(PBRRT_params, estimator_params)
initial_generate_map_2D(PBRRT_params, estimator_params)

PBRRT_instance = PBRRT_STAR(PBRRT_params, estimator_params)
print("YEEE")
t0 = time.time()
PBRRT_instance.RunPBRRT()
print(time.time() - t0)
robot_traj = show_gif(PBRRT_instance)
show_probability_graph(PBRRT_instance)
distance_to_nearest_obstacle_graph(PBRRT_instance, robot_traj)
#show_init_path2D(PBRRT_params, estimator_params, PBRRT_instance.path)
#show_init_tree(PBRRT_instance, PBRRT_params, estimator_params)