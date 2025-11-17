#Dynamic Obstacles are circles
class DynamicObstacle:
    all = []
    def __init__(self, x,y, radius, vx, vy, dt):
        
        #Initial Conditions
        self.x0 = x
        self.y0 = y
        self.vx0 = vx
        self.vy0 = vy
        self.r = radius
        
        
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        
        self.dt = dt
        DynamicObstacle.all.append(self) # Add to list of nodes

    def update_state(self, acceleration):
        self.vx = self.vx + acceleration[0]*self.dt
        self.vy = self.vy + acceleration[1]*self.dt
        self.x = self.x + self.vx*self.dt + 0.5* acceleration[0]*self.dt**2
        self.y = self.y + self.vy*self.dt + 0.5* acceleration[1]*self.dt**2
    
    def __repr__(self):
        return f"({self.x},{self.y}, {self.r})"