from scipy.spatial import KDTree
import numpy as np

class Periodic_KDTree:
    def __init__(self, points, N, max_samples_in_tree = 1e6): #N is after how many number of points you rebuild KDTree
        self.num_points_in_tree = points.shape[0]
        
        self.N = N
        
        self.points_in_tree_PreAlloc = np.empty((max_samples_in_tree, points.shape[1])) #Preallocated Matrix Initialization
        self.points_in_tree_PreAlloc[0:self.num_points_in_tree, :] = points
        self.tree = KDTree(points)
        self.points_not_in_tree_PreAlloc =  np.empty((N, points.shape[1])) #Preallocated Matrix Initialization
        self.number_of_points_outside_Tree = 0

    def rebuild_tree(self):
        self.tree = KDTree(self.points_in_tree_PreAlloc[0:self.num_points_in_tree, :])
        self.number_of_points_outside_Tree = 0


    
    def add_point(self, p):
        self.points_in_tree_PreAlloc[self.num_points_in_tree, :] = p
        self.num_points_in_tree = self.num_points_in_tree + 1
        self.points_not_in_tree_PreAlloc[self.number_of_points_outside_Tree, :] = p
        self.number_of_points_outside_Tree = self.number_of_points_outside_Tree + 1
        
        
            

            
    
    def calc_cost(self,p1, p2):
        return np.linalg.norm(p1-p2)
    
    def nearest(self, p):
        pass

            


        

                
            
            

    