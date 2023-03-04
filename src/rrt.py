import numpy as np
from random import randint
import random
from .tree import Node
from .model import BicycleModel
from .control2d import steer
from .cfg import params
import cv2 as cv

class RRT:
    def __init__(self,map,goal_threshold=8,name="RRT"):


        self.goal_threshold = goal_threshold
        self.tree = Node(q = map.q_start,w=0,u=0)
        self.map = map
        self.name=name
        self.map_img = map.map_img
        self.path = []


    def build(self,steps):
        x_best = None
        for i in range(steps):
            #get a random location sample in the map
            q_rand = self.sample_free()
            #find the nearest node to the random sample
            _,x_nearest = self.nearest_neighbor(x = self.tree,q_rand = q_rand)
            #genertate a steering trajectory from nearest node to random sample
            q_new, line_n = self.steer(x_nearest.q,q_rand)
            #check for collision
            if self.collision_free(line_n): 
                #add node to the tree
                x_new = Node(q=q_new,w=self.c(line_n))
                x_nearest.add_child(x_new)
                
                #the newly created tree branch
                self.map.draw_line(line_n,color=self.map.tree_color,width=1)
                
                #if goal reached draw path
                if self.in_goal_region(x_new.q):
                    if x_best != None:
                        if self.cost(x_new) >= self.cost(x_best):
                            continue
                    #draw path
                    print("===> drawing new path")
                    self.map.draw_point(self.map.q_goal,raduis=self.map.end_points_raduis,color=self.map.ends_color)
                    #erase old path
                    self.map.erase_path(self.path,width=2)
                    #extract the new path from new goal
                    self.path = self.extract_path(x_new)   
                    #draw the new path 
                    self.map.draw_path(self.path,width=2)
                    x_best = x_new

            if i%20 == 0:
                self.map.show_map(self.name)
            
        return self.tree



    def sample_free(self):
        q_rand = {"x":randint(1,self.map_img.shape[1]-1),"y":randint(1,self.map_img.shape[0]-1),
                  "theta":random.uniform(-np.pi,np.pi)}
        if self.point_collision_free(q_rand):
            return q_rand
        else:
            return self.sample_free()

    def nearest_neighbor(self,x,q_rand,e_dist = np.inf,x_nearest_prev=None):
        x_nearest = None
        min_dist = None
        if x is not None:
            temp_dist = self.ecludian(x.q,q_rand)
            if temp_dist < e_dist:
                x_nearest = x
                min_dist = temp_dist
            else:
                x_nearest = x_nearest_prev
                min_dist = e_dist
     
            for x_ in x.children:
                min_dist, x_nearest = self.nearest_neighbor(x_,q_rand,min_dist,x_nearest)
                
        return min_dist,x_nearest
             

    @staticmethod
    def steer(q_nearest, q_rand):
        line = steer(q_nearest,q_rand)
        return line[-1], line

    def collision_free(self,line):
        for q in line:
            if not self.point_collision_free(q):
                return False
        return True
    
    def point_collision_free(self,q):
        x, y = q["x"],q["y"]
        if (x>0 and y>0) and x < self.map_img.shape[1] and y < self.map_img.shape[0]:
            if not np.array_equal(self.map_img[y][x],self.map.obstacles_color):
                return True
        return False
        
    def in_goal_region(self,q):
      
        if self.ecludian(q,self.map.q_goal) <= self.goal_threshold+1:
            return True
        return False   
    
    @staticmethod
    def ecludian(q_n, q_r):
        x1 , y1, theta1 = q_n["x"],q_n["y"],q_n["theta"]
        x2, y2, theta2 = q_r["x"], q_r["y"],q_r["theta"]
        return ((x2-x1)**2 + (y2 - y1)**2 + 0.2*(theta2 - theta1)**2)**0.5 

    def cost(self,v):
        if v is None:
            return 0
        elif v.w is not None:
            return v.w + self.cost(v.parent)
        else:
            return self.c(self.steer(v.parent.q,v.q)) + self.cost(v.parent)

    def c(self,line):
        dist = 0
        len_ = len(line)
        for i in range(len_):
            if i < len_ - 1:
                dist += self.ecludian(line[i],line[i+1])
        return dist
    

    
    def extract_path(self,x_goal):
        path = []
        if x_goal == None:
            return path
        elif x_goal.parent == None:
            return path
        
        path.extend(self.extract_path(x_goal.parent))
        path.append((x_goal.parent.q,x_goal.q))
        return path

    @staticmethod
    def tree_len(node):
        i = 0
        if node is not None:
            i += 1
        for node_ in node.children:
            i += RRT.tree_len(node_)
            
        return i

    
    
    
    
    
    
    


