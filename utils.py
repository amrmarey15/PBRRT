import numpy as np
from StaticObstacle import *
from DynamicObstacle import *
from Node import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from KF_Circular_Obstacle_Pos_Estimator import *

def initial_generate_map_2D(PBRRT_params: dict, estimator_params: dict):
    map_size = PBRRT_params["map_size"]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_aspect('equal')
    ax.grid(False)
    
    rects = []
    for static_obstacle in StaticObstacle.all:
        rects.append(patches.Rectangle((static_obstacle.x_min, static_obstacle.y_min), static_obstacle.x_max - static_obstacle.x_min ,  static_obstacle.y_max - static_obstacle.y_min, color='gray', alpha=0.6))
    
    for rect in rects:
        ax.add_patch(rect)
    
    circ_patches = []
    for circle in DynamicObstacle.all:
        circ_patches.append(patches.Circle((circle.pos[0], circle.pos[1]), radius=circle.r, color='gray', alpha=0.6))
    
    for circ_patch in circ_patches:
        ax.add_patch(circ_patch)
    
    ax.set_title("2D Obstacle Map")
    plt.show()


def show_init_path2D(PBRRT_params: dict, estimator_params: dict, path: list):

    path_np = np.empty((len(path), 2))
    for i in range(len(path)):
        path_np[i,:] = path[i].pos
    


    map_size = PBRRT_params["map_size"]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_aspect('equal')
    ax.grid(False)
    
    rects = []
    for static_obstacle in StaticObstacle.all:
        rects.append(patches.Rectangle((static_obstacle.x_min, static_obstacle.y_min), static_obstacle.x_max - static_obstacle.x_min ,  static_obstacle.y_max - static_obstacle.y_min, color='gray', alpha=0.6))
    
    for rect in rects:
        ax.add_patch(rect)
    
    circ_patches = []
    for circle in DynamicObstacle.all:
        circ_patches.append(patches.Circle((circle.pos[0], circle.pos[1]), radius=circle.r, color='gray', alpha=0.6))
    
    for circ_patch in circ_patches:
        ax.add_patch(circ_patch)
    
    path = path_np
    # Now plot the path
    ax.plot(path[:,0], path[:,1], '-r', linewidth=2, label='Path')
    ax.scatter(path[:,0], path[:,1], c='red', s=20)  # points
    ax.set_title("2D Obstacle Map")
    plt.show()



def show_init_tree(PBRRT_inst, PBRRT_params: dict, estimator_params: dict):



    map_size = PBRRT_params["map_size"]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_aspect('equal')
    ax.grid(False)
    

    for node in PBRRT_inst.Tree.Nodes_in_Tree:      # list of Node objects
        if node.parent is None:
            continue
        ax.plot(
            [node.parent.pos[0], node.pos[0]],
            [node.parent.pos[1], node.pos[1]],
            color='gray', linewidth=1
    )



    rects = []
    for static_obstacle in StaticObstacle.all:
        rects.append(patches.Rectangle((static_obstacle.x_min, static_obstacle.y_min), static_obstacle.x_max - static_obstacle.x_min ,  static_obstacle.y_max - static_obstacle.y_min, color='gray', alpha=0.6))
    
    for rect in rects:
        ax.add_patch(rect)
    
    circ_patches = []
    for circle in DynamicObstacle.all:
        circ_patches.append(patches.Circle((circle.pos[0], circle.pos[1]), radius=circle.r, color='gray', alpha=0.6))
    
    for circ_patch in circ_patches:
        ax.add_patch(circ_patch)
    





    ax.set_title("2D Obstacle Map")
    plt.show()
        

def generate_circle_dynamic_obstacles(PBRRT_params: dict, estimator_params: dict):
    circles = []
    Q = estimator_params['Q']
    V = estimator_params['V']
    P_Post = estimator_params['P_Post']
    for i in range(PBRRT_params["num_d_obstacles"]):
        R = np.random.uniform(0.5, 2) # radius of obstacle
        p = sample_outside_start_and_end_node(PBRRT_params["start"].pos, PBRRT_params["goal"].pos, R, PBRRT_params["map_size"])
        v = np.random.uniform(0.25, 5) # radius of obstacle
        d = DynamicObstacle(p, R, v)
        estimator = KF_Circular_Obstacle_Pos_Estimator(d, Q, V, P_Post)
        circ = [p[0], p[1], R]
        circles.append(circ)
    
    return circles
        
        
        
        
        


def sample_outside_start_and_end_node(c1, c2, r, map_size):  #Make sure you generate random dynamic obstacles not at starting node and end node
    while True:
        # uniform point in bounding box
        
        p = np.empty(len(map_size))
        for i in range(len(map_size)):
            p[i] = np.random.uniform(0, map_size[i])
        
        # distances to circles
        d1 = np.linalg.norm(p - c1)
        d2 = np.linalg.norm(p - c2)
        
        # accept if outside both circles
        if d1 > r and d2 > r:
            return p