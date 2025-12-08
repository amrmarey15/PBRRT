from scipy.spatial import KDTree
import numpy as np
from Node import Node

class Periodic_KDTree:
    def __init__(self, start_point: Node, N: int, max_samples_in_tree: int = 1_000_000): #N is after how many number of points you rebuild KDTree
        self.Nodes_in_Tree = [start_point]
        start_point = start_point.pos #Convert to numpy position representation
        self.num_points = 1 
        self.N = N
        self.points_in_tree_PreAlloc = np.empty([max_samples_in_tree, start_point.size]) #Preallocated Matrix Initialization
        self.points_in_tree_PreAlloc[0, :] = start_point
        start_point = start_point.reshape((1,len(start_point)))
        self.tree = KDTree(start_point)
        self.points_not_in_tree_PreAlloc = np.empty([N, start_point.size]) #Preallocated Matrix Initialization
        self.number_of_points_outside_Tree = 0
        self.dim = self.points_in_tree_PreAlloc.shape[1]

    def rebuild_tree(self):
        self.tree = KDTree(self.points_in_tree_PreAlloc[0:self.num_points, :])
        self.number_of_points_outside_Tree = 0
        self.points_not_in_tree_PreAlloc = np.empty([self.N, self.dim])


    def add_point(self, p: Node):
        self.Nodes_in_Tree.append(p)
        p = p.pos #Convert to numpy position representation
        self.points_in_tree_PreAlloc[self.num_points, :] = p
        self.num_points = self.num_points + 1
        self.points_not_in_tree_PreAlloc[self.number_of_points_outside_Tree, :] = p
        self.number_of_points_outside_Tree = self.number_of_points_outside_Tree + 1
        if self.number_of_points_outside_Tree == self.N:
            self.rebuild_tree()
    
    def nearest(self, p: Node): #Return nearest neigbour index
        p = p.pos #Convert to numpy position representation
        min_dist_tree, min_idx_tree = self.tree.query(p)
        if self.number_of_points_outside_Tree > 0:
            sq_dists_outside_tree = np.sum((self.points_not_in_tree_PreAlloc[0:self.number_of_points_outside_Tree,:] - p)**2, axis=1)
            idx_outside_tree = sq_dists_outside_tree.argmin()
            min_dist_outside_tree = np.sqrt(sq_dists_outside_tree[idx_outside_tree])
            if min_dist_tree < min_dist_outside_tree:
                return min_idx_tree
            else:
                return self.num_points - self.number_of_points_outside_Tree + idx_outside_tree
        else:
            return min_idx_tree

    def query_ball_point(self, p: Node, r: float): # return all the points within radius r of query node p
        p = p.pos #Convert to numpy position representation
        near_points_indices_tree = self.tree.query_ball_point(p, r)
        
        sq_dist_outside_tree = np.sum((self.points_not_in_tree_PreAlloc[0:self.number_of_points_outside_Tree,:] - p)**2, axis=1)
        idx = np.where(sq_dist_outside_tree < r*r)[0]
        near_points_indices_outside_tree = []
        for i in idx:
            point_id = self.num_points - self.number_of_points_outside_Tree + i
            near_points_indices_outside_tree.append(point_id)
        return near_points_indices_tree + near_points_indices_outside_tree
    

    def remove_point_from_tree(self, p: Node):
        self.Nodes_in_Tree.remove(p)
        p = p.pos
        dist, idx = self.tree.query(p)

        if dist < 0.001: #The nearest neigbor returns the same point
            self.points_in_tree_PreAlloc = self.points_in_tree_PreAlloc[np.arange(self.points_in_tree_PreAlloc.shape[0]) != idx]
            
        else:
            mask = np.all(np.isclose(self.points_not_in_tree_PreAlloc, p), axis=1)
            if not np.any(mask): #if no point matches
                return
            idx = np.where(mask)[0][0]
            self.points_in_tree_PreAlloc = self.points_in_tree_PreAlloc[np.arange(self.points_in_tree_PreAlloc.shape[0]) != idx+self.num_points]

        self.num_points = self.num_points - 1
        self.rebuild_tree()

        

            

        
        
        
    


        

                
            
            

    