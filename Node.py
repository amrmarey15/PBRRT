import numpy as np

class Node:
    nodes_in_tree = []
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.pos = np.array([x,y])
        self.parent = None
        self.cost = 0.0     
        self.children = None
        self.k_star = 1 #Arrival time from parent
        self.in_Tree = False
        
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False

    def __repr__(self):
        return f"({self.x},{self.y})"
    
    def append_to_tree(self): #Not going to use this anymore
        self.in_Tree = True
        Node.nodes_in_tree.append(self) # Add to list of nodes
    
    def update_parent(self, parent):
        self.parent = parent