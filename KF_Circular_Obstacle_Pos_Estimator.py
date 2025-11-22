import numpy as np
from Node import *
from DynamicObstacle import *

class KF_Circular_Obstacle_Pos_Estimator: #Calculates probability a circular obstacle will be at a given position using a discrete Kalman Filter
    #State space eqn modeled to predict obstacle location at time k+1 is x(k+1) = I*x(k) + u where u = x(k) - x(k-1) (approximate linear velocity) 
    def __init__(self, obstacle: DynamicObstacle, Q: np.ndarray, V: np, P_posteriori_0: np.ndarray):
        self.u = 0 #assume u(0) = 0
        self.Q = Q
        self.V = V
        self.dim = Q.shape[0] #dimension of state space
        self.x_prev = obstacle.pos0.reshape((self.dim, 1)) #Assume we have correct initial position of dynamic obstacle
        self.x_priori = obstacle.pos0.reshape((self.dim, 1))
        self.x_posteriori = obstacle.pos0.reshape((self.dim, 1)) + np.random.multivariate_normal(mean=np.zeros(self.dim), cov=V) # estimated state value
        self.P_priori = P_posteriori_0
        self.P_posteriori_0 = P_posteriori_0
        self.P_posteriori = P_posteriori_0
        
        i = 0
        for dynamic_obstacle in DynamicObstacle.all:
            if obstacle == dynamic_obstacle:
                self.which_dynamic_obstacle = i #determines which obstacle im referring to in DynamicObstacle.all
                break
            i=i+1
    
        self.r = obstacle.r # Radius (Needed to calculate probability of collision)

    def KF_update(self, x_measured: np.ndarray): #Given sensor value x_measured
        self.x_priori = self.x_posteriori + self.x_posteriori - self.x_prev
        self.P_priori = self.P_posteriori + self.Q
        self.K_star = self.P_priori @ np.linalg.inv(self.P_priori+self.V)
        self.x_posteriori = self.x_priori +self.K_star @ (x_measured - self.x_priori)
        
        M = np.eye(self.dim) - self.K_star
        self.P_posteriori = M @ self.P_priori @ np.transpose(M) + self.K_star @ self.V @ np.transpose(self.K_star)
        
    def calculate_probability_of_collision(self, x_query: np.ndarray):
        
        
        
        
    
    
    
    