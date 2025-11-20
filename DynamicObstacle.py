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

    def update_state(self, future_pos: np.ndarray):
        self.pos = future_pos
        
    def __repr__(self):
        return f"({self.x},{self.y}, {self.r})"