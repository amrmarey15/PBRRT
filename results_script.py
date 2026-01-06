from DynamicObstacle import *
from PBRRT_STAR import *
from Node import *
import matplotlib.pyplot as plt

def show_probability_graph(PBRRT_inst):
    prob_list = []
    path_executed = PBRRT_inst.path_executed
    for node in path_executed:
        for i in range(node.k_star_exec):
            prob_list.append(node.node_prob_collision)
    
    plt.figure()
    plt.plot(prob_list)
    plt.xlabel(r"$t$")
    plt.ylabel(r"$P_{c}$")
    plt.title("Probability of Collision over Time")
    plt.show()
    return prob_list

def distance_to_nearest_obstacle_graph(robot_traj):
    risk_list = []
    for i in range(robot_traj.shape[0]):
        min_dist = np.inf
        for Dobs in DynamicObstacle.all:
            dist = np.linalg.norm(robot_traj[i,:] - Dobs.locations_list[i]) - Dobs.r
            if dist < min_dist:
                min_dist = dist
        risk_list.append(min_dist)
    plt.figure()
    plt.plot(risk_list)
    plt.xlabel(r"$t$")
    plt.ylabel(r"$D_{min}$")
    plt.title("Distance to nearest Dynamic Obstacle over Time")
    plt.show()
    return risk_list
        

    

    
