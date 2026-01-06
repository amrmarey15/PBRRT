#Dynamic Obstacles are circles
import numpy as np
class DynamicObstacle:
    all = []
    def __init__(self, pos: np.ndarray, radius, v: np.ndarray, dt = 1):
        self.pos0 = pos #Initial Conditions
        self.pos = pos #position array
        self.v = v #velocity array
        self.r = radius
        self.dt = dt
        DynamicObstacle.all.append(self) # Add to list of Dynamic Obstacles
        self.estimator = None
        self.pos_list = [] #Graph
        self.locations_list = []

    def update_state(self, future_pos: np.ndarray):
        self.pos = future_pos
    
    def __eq__(self, other):
        if isinstance(other, DynamicObstacle):
            return np.linalg.norm(self.pos - other.pos) < 0.001 and self.r == other.r
        return False
        
    def __repr__(self):
        return f"({self.pos}, {self.r})"
    
    def generate_trajectory(self, rng, T_max = 100000):
        pos_x = self.pos[0]
        pos_y = self.pos[1]
        for t in range(T_max):
            pos_x = pos_x + rng.uniform(-0.15,0.15)*np.cos(rng.uniform(-np.pi,np.pi)*t)
            pos_y = pos_y + rng.uniform(-0.15,0.15)*np.sin(rng.uniform(-np.pi,np.pi)*t)
            pos = np.array([pos_x, pos_y])
            self.locations_list.append(pos)
            self.pos_list.append(pos)


