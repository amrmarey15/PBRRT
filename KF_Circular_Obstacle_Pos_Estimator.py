import numpy
from Node import *
from DynamicObstacle import *

class KF_Circular_Obstacle_Pos_Estimator: #Calculates probability a circular obstacle will be at a given position using a discrete Kalman Filter
    #State space eqn modeled to predict obstacle location at time k+1 is x(k+1) = I*x(k) + u where u = x(k) - x(k+1) (approximate linear velocity) 
    def __init__(self, obstacle: DynamicObstacle):
        self.x_prev = obstacle.pos0 #Assume we have correct initial position of dynamic obstacle
        self.x_measured = obstacle.pos0 #observed state value
        self.x_model = obstacle.pos0 # estimated state value
        self.u = 0 #assume u(0) = 0
    
    def state_space_model(self):
        pass
        
    
    
    
    