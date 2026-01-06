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

   

obstacle1 = StaticObstacle([10, 17.4, 12, 15])


PBRRT_params = {
    "K_limits": 2,
    "start": Node([19,11]),
    "goal": Node([19,19]),
    "N": 1e3,
    "Line_Sample_collision_checks": 3,
    "Step_Size": 0.15,
    "P_coll": 0.5,
    "Final_Radius_Limit": 1,
    "M": 15,
    "gamma": 0.9,
    "Max_Iterations": int(1e5),
    "alpha": 0.001,
    'map_size': [20,20],
    'static_obstacles_list': [obstacle1],
    'num_d_obstacles': 9,
    'R': 0.2,
    'Lim': 3
}



rng_gen = np.random.default_rng(seed=5)

sigma_squared = 0.1

estimator_params = {
    'Q': sigma_squared*np.eye(len(PBRRT_params["map_size"])),
    'V': sigma_squared*np.eye(len(PBRRT_params["map_size"])),
    'P_Post': sigma_squared*np.eye(len(PBRRT_params["map_size"]))
}

circles = [DynamicObstacle(np.array([18.75,14]), 1, 0)]
estimator = KF_Circular_Obstacle_Pos_Estimator(circles[0], estimator_params['Q'], estimator_params['V'], estimator_params['P_Post'])
for Dobs in DynamicObstacle.all:
        Dobs.generate_trajectory(rng_gen)

PBRRT_instance = PBRRT_STAR(PBRRT_params, estimator_params)
PBRRT_instance.RunPBRRT()
print("Number of Plans:", PBRRT_instance.num_plans)
print("Number of Samples for Initial Plan:", PBRRT_instance.samples_num[0])
robot_traj = show_gif(PBRRT_instance, "map_4")
prob_list = show_probability_graph(PBRRT_instance)
min_dist_list = distance_to_nearest_obstacle_graph(robot_traj)

min_index_dist, min_dist = min(enumerate(min_dist_list), key=lambda x: x[1])
max_index_prob, max_prob = max(enumerate(prob_list), key=lambda x: x[1])


print("Closest Distance to Obstacle and its index:", min_dist, min_index_dist)
print("Maximum Probability of Collision and its index:", max_prob, max_index_prob)