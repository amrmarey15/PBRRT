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
import pickle

   

obstacle1 = StaticObstacle([4, 7, 12, 20])
obstacle2 = StaticObstacle([12, 16, 0, 8])

PBRRT_params = {
    "K_limits": 2,
    "start": Node([2,18]),
    "goal": Node([18,2]),
    "N": 1e3,
    "Line_Sample_collision_checks": 3,
    "Step_Size": 0.15,
    "P_coll": 0.1,
    "Final_Radius_Limit": 2,
    "M": 15,
    "gamma": 0.9,
    "Max_Iterations": int(5e4),
    "alpha": 0.001,
    'map_size': [20,20],
    'static_obstacles_list': [obstacle1, obstacle2],
    'num_d_obstacles': 12,
    'R': 0.2,
    'Lim': 5
}

sigma_squared = 0.1

estimator_params = {
    'Q': sigma_squared*np.eye(len(PBRRT_params["map_size"])),
    'V': sigma_squared*np.eye(len(PBRRT_params["map_size"])),
    'P_Post': sigma_squared*np.eye(len(PBRRT_params["map_size"]))
}

circles = generate_circle_dynamic_obstacles(PBRRT_params, estimator_params)

PBRRT_instance = PBRRT_STAR(PBRRT_params, estimator_params)
PBRRT_instance.RunPBRRT()
print("Number of Plans:", PBRRT_instance.num_plans)
print("Number of Samples for Initial Plan:", PBRRT_instance.samples_num[0])
robot_traj = show_gif(PBRRT_instance, "map_2")
prob_list = show_probability_graph(PBRRT_instance)
min_dist_list = distance_to_nearest_obstacle_graph(robot_traj)

min_index_dist, min_dist = min(enumerate(min_dist_list), key=lambda x: x[1])
max_index_prob, max_prob = max(enumerate(prob_list), key=lambda x: x[1])


print("Closest Distance to Obstacle and its index:", min_dist, min_index_dist)
print("Maximum Probability of Collision and its index:", max_prob, max_index_prob)