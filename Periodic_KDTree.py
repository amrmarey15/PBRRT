from scipy.spatial import KDTree
import numpy as np

class Periodic_KDTree:
    def __init__(self, points, N): #N is after how many number of points you rebuild KDTree
        self.points = points
        self.N = N
        self.tree = KDTree(self.points)
        self.num_points_in_tree = self.points.shape[0]
        self.points_not_in_tree = np.array([])

    def rebuild_tree(self):
        self.tree = KDTree(self.points)
        self.num_points_in_tree = self.points.shape[0]
        self.points_not_in_tree = np.array([])

    
    def add_point(self, p):
        self.points = np.vstack([self.points, p])
        if self.points_not_in_tree.size == 0:
            self.points_not_in_tree = p
        else:
            self.points_not_in_tree = np.vstack([self.points_not_in_tree, p])
        if self.points_not_in_tree.shape[0] == self.N:
            self.rebuild_tree()
            
    
    def calc_cost(self,p1, p2):
        return np.linalg.norm(p1-p2)
    
    def nearest(self, p):
        dist, idx = self.tree.query(p)
        for i in range(.):

            


        

                
            
            

    