
#Static obstacles are rectangles
class StaticObstacle: #Will assume static obstacles are rectangles for now
    all = []
    def __init__(self, vertices):
        self.x_min = vertices[0]
        self.x_max = vertices[1]
        self.y_min = vertices[2]
        self.y_max = vertices[3]
        
        self.width = self.x_max -self.x_min
        self.height = self.y_max -self.y_min
        StaticObstacle.all.append(self) # Add to list of nodes
        
    def __repr__(self):
        return f"({self.x_min},{self.x_max}, {self.y_min},{self.y_max})"