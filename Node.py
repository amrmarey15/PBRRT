class Node:
    #all = []
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0     
        self.children = None
        self.k_star = 1 #Arrival time from parent
        #Node.all.append(self) # Add to list of nodes
        
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False

    def __repr__(self):
        return f"({self.x},{self.y})"