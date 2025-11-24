import numpy as np
from Node import *
from DynamicObstacle import *
from scipy.stats import ncx2

class KF_Circular_Obstacle_Pos_Estimator: #Calculates probability a circular obstacle will be at a given position using a discrete Kalman Filter
    #State space eqn modeled to predict obstacle location at time k+1 is x(k+1) = I*x(k) + u where u = x(k) - x(k-1) (approximate linear velocity)
    #We will assume Q, V are a istopic matrix to work for def calculate_probability_of_collision to work properly
    all = []
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
        KF_Circular_Obstacle_Pos_Estimator.all.append(self) # Add to list of Dynamic estimators
        i = 0
        for dynamic_obstacle in DynamicObstacle.all:
            if obstacle == dynamic_obstacle:
                self.which_dynamic_obstacle = i #determines which obstacle im referring to in DynamicObstacle.all
                dynamic_obstacle.estimator = self
                break
            i=i+1

        self.r = obstacle.r # Radius (Needed to calculate probability of collision)


    def KF_update(self, x_measured: np.ndarray): #Given sensor value x_measured
        self.x_priori = self.x_posteriori + self.x_posteriori - self.x_prev
        self.P_priori = self.P_posteriori + self.Q
        self.K_star = self.P_priori @ np.linalg.inv(self.P_priori+self.V)
        self.x_prev = self.x_posteriori
        self.x_posteriori = self.x_priori +self.K_star @ (x_measured - self.x_priori)
        
        M = np.eye(self.dim) - self.K_star
        self.P_posteriori = M @ self.P_priori @ np.transpose(M) + self.K_star @ self.V @ np.transpose(self.K_star)
        
    def calculate_probability_of_collision(self, x_query: np.ndarray, timestep):
        mu = self.x_posteriori + timestep*(self.x_posteriori - self.x_prev)
        x_q = np.asarray(x_q)
        d = self.dim
        sigma_squared = (self.P_posteriori[0,0])**(timestep+1)
        R = self.r
            
            
        
        # noncentrality parameter λ = ||mu - x_q||^2 / sigma^2
        lam = np.sum((mu - x_q)**2) / (sigma_squared)

        # upper limit for the noncentral chi-square
        t = (R**2) / (sigma_squared)

        # CDF of noncentral chi-square
        return ncx2.cdf(t, df=d, nc=lam)
        
        
        
        
        
    
    
    
    