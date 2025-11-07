from scipy.spatial import KDTree
import numpy as np

class Periodic_KDTree:
    def __init__(self, points, N): #N is after how many number of points you rebuild KDTree
        self.points = points
        self.N = N
        self.tree = KDTree(self.points)
        self.NonTreePoints = np.array([]) #points not added to tree yet

    def rebuild_tree(self):
        self.points = np.vstack([self.points, self.NonTreePoints])
        self.NonTreePoints = np.array([])
        self.tree = KDTree(self.points)
    
    def add_point(self, p):
        if self.NonTreePoints.size == 0:
            self.NonTreePoints = p
        else:
            self.NonTreePoints = np.vstack([self.NonTreePoints, p])
            if self.NonTreePoints.shape[0] == self.N:
                self.rebuild_tree()
                self.NonTreePoints = np.array([])
    
    def calc_cost(self,p1, p2):
        pass
    
    def nearest(self, p):
        min_dist, idx = self.tree.query(p, 1) #find nearest point in tree
        for i in range(self.NonTreePoints.shape[0]):
            x = self.NonTreePoints[i,:]
            dist = np.linalg.norm(p - x)
            if dist < min_dist:
                min_dist = dist
            

            
            
        

                
            
            

    