import numpy as np

class Node:
    nodes_in_tree = []
    def __init__(self, pos):
        self.pos = pos
        self.parent = None
        self.cost = 0.0     
        self.children = None
        self.k_star = 1 #Arrival time from parent
        self.in_Tree = False
        self.node_prob_collision = 0 # probability that node will not be in Cfree at the arrival time from parent node
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.pos == other.pos
        return False

    def __repr__(self):
        return f"({self.pos})"
    
    def append_to_tree(self): #Not going to use this anymore
        self.in_Tree = True
        Node.nodes_in_tree.append(self) # Add to list of nodes
    
    def update_parent(self, parent):
        self.parent = parent
    
    def Get_Node_Time_Between_Two_Nodes(self, Node_Ancestor, Node_Child): #Get discrete time between ancestor node and child node
        k = 0
        while(Node_Child != Node_Ancestor):
            k = k + Node_Child.k_star
            Node_Child = Node_Child.parent
            if Node_Child == None:
                raise Exception("Nodes are not connected")
        return k
    
    def calc_num_generations_to_ancestor_node(self, ancestor_node): #Returns 0 between first generation difference, 1 between second generation difference etc
        num_generations = 0
        current_node = self
        while current_node != ancestor_node:
            current_node = current_node.parent
            num_generations = num_generations + 1
            if current_node == None:
                raise Exception("Nodes are not connected")
        return num_generations
            
            
        
        
            
            