import numpy as np
from random import randint
import random
import pygame
from .utils import *
from .tree import Node
from .model import BicycleModel
from .control2d import lateral_control_stanly
from .cfg import params
import cv2 as cv

class RRT:
    def __init__(self, map, q_start, q_goal,goal_threshold=8, obstacles_color= (0,255,0,255)):
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        
        """
        self.map = map
        self.tree = Node(q = q_start)
        self.q_start = q_start
        self.q_goal =  q_goal
        self.goal_threshold = goal_threshold
        self.obstacles_color = obstacles_color
        self.motion_Model = BicycleModel(q_init=q_start)

    def build(self,steps):
        for i in range(steps):
            #get a random location sample in the map
            q_rand = self.sample_free()
            #find the nearest node to the random sample
            _,x_nearest = self.nearest_neighbor(x = self.tree,q_rand = q_rand)
            #genertate a steering trajectory from nearest node to random sample
            q_new, trajectory = self.steer(x_nearest.q,q_rand)
            #check for collision
            if self.collision_free(trajectory): 
                #add node to the tree
                x_new = Node(q_new)
                x_nearest.add_child(x_new)
                #the newly created tree branch
                draw_trajectoy(self.map,trajectory,width=1,color=(0,0,0))
                #if goal reached draw path
                if self.in_goal_region(x_new.q):
                    draw_point(self.map,self.q_goal,raduis=self.goal_threshold,width=self.goal_threshold,color=(0,0,255))
                    draw_trajectories_path(self.map,x_new,width=2,color=(0,0,255))

            if cv.waitKey(1) == ord('q'):
                break
            
        return self.tree



    def sample_free(self):
        q_rand = {"x":randint(1,self.map.shape[1]-1),"y":randint(1,self.map.shape[0]-1),
                  "theta":random.uniform(-np.pi,np.pi)}
        if self.point_collision_free(q_rand):
            return q_rand
        else:
            return self.sample_free()

    def nearest_neighbor(self,x,q_rand,e_dist = np.inf,x_nearest_prev=None):
        x_nearest = None
        min_dist = None
        if x is not None:
            temp_dist = ecludian(x.q,q_rand)
            if temp_dist < e_dist:
                x_nearest = x
                min_dist = temp_dist
            else:
                x_nearest = x_nearest_prev
                min_dist = e_dist
     
            for x_ in x.children:
                min_dist, x_nearest = self.nearest_neighbor(x_,q_rand,min_dist,x_nearest)
                
        return min_dist,x_nearest
             

    
    def steer(self, q_nearest, q_rand):
        trj = [q_nearest]
        for i in range(10):
            self.motion_Model.update_state(q_state = trj[i])
            v = params["v_max"]
            delta = lateral_control_stanly(trj[i],q_rand,v)
            trj.append(self.motion_Model.forward(v,delta))
            
        return trj[-1], trj

    def collision_free(self,line):
        for q in line:
            if not self.point_collision_free(q):
                return False
        return True
    
    def point_collision_free(self,q):
        x, y = q["x"],q["y"]
        if (x>0 and y>0) and x < self.map.shape[1] and y < self.map.shape[0]:
            if not np.array_equal(self.map[y][x],self.obstacles_color):
                return True
        return False
        
    def in_goal_region(self,q):
      
        if ecludian(q,self.q_goal) <= self.goal_threshold+1:
            return True
        return False    
    
    def tree_len(self,node):
        i = 0
        if node is not None:
            i += 1
        for node_ in node.children:
            i += self.tree_len(node_)
            
        return i

        
        
        
        

if __name__=="__main__":

    # Initializing surface
    map = np.ones((500,500,3), dtype=np.uint8)*255
    
    #draw star and goal points
    q_start={"x":100,"y":20,"theta":0.3,"delta":0.1,"beta":0.01}
    q_goal={"x":350,"y":420,"theta":0,"delta":0,"beta":0}
    star_color = (255,0,255)
    goal_color = (0,0,255)
    cv.circle(map,center=(q_start["y"],q_start["x"]),radius=5,color=star_color,thickness=-1)
    cv.circle(map,center=(q_goal["y"],q_goal["x"]),radius=7,color=goal_color,thickness=-1)


    #draw obstacles
    obstacle_color = (0,255,0)
    cv.rectangle(map,(255,255),(300,300),obstacle_color,thickness=-1)
    cv.rectangle(map,(150,150),(200,200),obstacle_color,thickness=-1)
    cv.rectangle(map,(100,300),(170,325),obstacle_color,thickness=-1)
    cv.rectangle(map,(100,80),(150,120),obstacle_color,thickness=-1)
    cv.rectangle(map,(10,80),(50,120),obstacle_color,thickness=-1)
    cv.rectangle(map,(210,320),(240,350),obstacle_color,thickness=-1)
    cv.circle(map,center=(400,60),radius=40,color=obstacle_color,thickness=-1)
    cv.circle(map,center=(250,60),radius=40,color=obstacle_color,thickness=-1)



    # Initializing RTT
    rrt = RRT(map=map,q_start=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
    #Build RRT
    rrt.build(int(1e6))
    input('Press ENTER to exit')
    
    
    
    
    
    


