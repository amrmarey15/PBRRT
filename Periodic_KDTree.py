from scipy.spatial import KDTree
import numpy as np

class Periodic_KDTree:
    def __init__(self, points, N, max_samples_in_tree = 1e6): #N is after how many number of points you rebuild KDTree
        self.num_points = points.shape[0]
        self.old
        self.N = N
        
        self.points_in_tree_PreAlloc = np.empty((max_samples_in_tree, points.shape[1])) #Preallocated Matrix Initialization
        self.points_in_tree_PreAlloc[0:self.num_points, :] = points
        self.tree = KDTree(points)
        self.points_not_in_tree_PreAlloc =  np.empty((N, points.shape[1])) #Preallocated Matrix Initialization
        self.number_of_points_outside_Tree = 0

    def rebuild_tree(self):
        self.tree = KDTree(self.points_in_tree_PreAlloc[0:self.num_points, :])
        self.number_of_points_outside_Tree = 0


    def add_point(self, p):
        self.points_in_tree_PreAlloc[self.num_points, :] = p
        self.num_points = self.num_points + 1
        self.points_not_in_tree_PreAlloc[self.number_of_points_outside_Tree, :] = p
        self.number_of_points_outside_Tree = self.number_of_points_outside_Tree + 1
        if self.number_of_points_outside_Tree == self.N:
            self.rebuild_tree()
    
    def nearest(self, p): #Return nearest neigbour index
        min_dist_tree, min_idx_tree = self.tree.query(p)
        if self.number_of_points_outside_Tree > 0:
            sq_dists_outside_tree = np.sum((self.points_not_in_tree_PreAlloc[0:self.number_of_points_outside_Tree,:] - p)**2, axis=1)
            idx_outside_tree = sq_dists_outside_tree.argmin()
            min_dist_outside_tree = np.sqrt(sq_dists_outside_tree[idx_outside_tree])
            if min_dist_tree < min_dist_outside_tree:
                return min_idx_tree
            else:
                return self.num_points - self.N + idx_outside_tree
        else:
            return min_idx_tree

            


        

                
            
            

    