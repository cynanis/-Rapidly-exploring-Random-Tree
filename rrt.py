import numpy as np
import matplotlib.pyplot as plt
from random import randint



class Node:

    def __init__(self,data):
        """ data = {"V": (x,y), ,"E":edge} """
        self.data = data
        self.children = []
        self.parent = None
        self.extended = False

    def add_child(self,node):
        node.parent = self
        self.children.append(node)
        self.extended = True

    def getxy(self):
        return self.data["V"]

class RRT:
    def __init__(self, map, x_star, x_goal, step_size):
        """ 
            x_star = (x,y)
            x_goal = (x,y)
        
        """
        self.map = map
        self.tree = Node(data = {"V":x_star})
        self.goal = Node(data = {"V": x_goal})
        self.step_size = step_size

    def buildRRT(self,steps):
        for i in range(steps):
            x_rand = self.randomSample()
            x_nearest, node_nearest, ecludian_dist = self.nearest(x_rand)
            x_new = self.steer(x_nearest,x_rand)
            if self.isCollisionFree(x_nearest,x_new): 
                node_new = Node(data = {"V":x_new, "E":ecludian_dist})
                node_nearest.add_child(node_new)

                if self.goalReached(x_new):
                    return self.tree




    def randomSample(self):
        x = randint(len(self.map))
        y = randint(len(self.map[0]))
        return (x,y)

    def nearest(self,x_rand):

        
        root = self.tree
        node_nearest = root
        ecludian_dist = np.inf
        if root is not None:
            ecludian_dist, node_nearest = self.treeTraversal(root,x_rand,ecludian_dist)
        
        return node_nearest.data["V"], node_nearest, ecludian_dist
             

    
    def steer(self, x_nearest, x_rand):
        
        if self.ecludian(x_n=x_nearest,x_r=x_rand) < self.step_size :
            return x_rand
        else:
            x1,y1 = x_nearest
            x2,y2 = x_rand
            a = (y2-y1)/(x2-x1)
            x3 = (self.step_size**2/(a**2 +1))**0.5 + x1
            y3 =  a * ((self.step_size**2/(a**2 +1))**0.5) + y1
        
        return (int(x3),int(y3))

    def isCollisionFree(self,x_nearest,x_new):
        return False
    
    def goalReached(self,x_new):
        return False

    def ecludian(self, x_n, x_r):
        x1 , y1 = x_n
        x2, y2 = x_r
        return np.sqrt((x2-x1)**2 + (y2 - y1)**2)

    def treeTraversal(self ,node ,x_rand, e_dist):
        
        
        node_nearest = node
        min_dist = e_dist
        temp_dist = self.ecludian(x_n = node.getxy(),x_r = x_rand)
        
        if temp_dist < min_dist:
            min_dist = temp_dist
            node_nearest = node
            
        for node_ in node.children:
            min_dist, node_nearest = self.treeTraversal(node_,x_rand,e_dist,node_nearest)
        
        return min_dist, node_nearest

        

        
    


