import numpy as np
import matplotlib.pyplot as plt
from random import randint
import pygame
from utils import *

class Node:

    def __init__(self,q):
        """ q = (x,y) """
        self.q = q
        self.children = []
        self.cost = None
        self.parent = None

    def add_child(self,child_x):
        child_x.parent = self
        self.children.append(child_x)
    
    def remove_child(self,child_x):
        self.children = [child for child in self.children if child is not child_x]
    
    def add_cost(self,cost):
        self.cost = cost    

class RRT:
    def __init__(self, map, q_star, q_goal, step_size = 10,goal_threshold=8, obstacles_color= (0,255,0,255),goal_color = (0,0,255,255)):
        """ 
            q_star = (x,y)
            q_goal = (x,y)
        
        """
        self.map = map
        self.tree = Node(q = q_star)
        self.tree.add_cost(0)
        self.goal =  q_goal
        self.step_size = step_size
        self.goal_threshold = goal_threshold
        self.obstacles_color = obstacles_color
        self.goal_color = goal_color

    def build_rrt(self,steps):
        goal_reached = False
        for i in range(steps):
            q_rand = self.sample_free()
            _,x_nearest = self.nearest(self.tree,q_rand)
            q_new = self.steer(x_nearest.q,q_rand)
            if self.is_collision_free(line(x_nearest.q,q_new)): 
                # find list of x nodes that lie in the circle centerd at q_new
                X_near = near(self.tree,q_new,20)     
                
                 #connect x_new node to the near x node that result in a minimum-cost path
                x_new = Node(q = q_new)
                x_min = x_nearest
                c_min = cost(x_nearest) + c(line(x_nearest.q,q_new))
                for x_near in X_near:
                    if self.is_collision_free(line(x_near.q,q_new)):
                        if cost(x_near) + c(line(x_near.q,q_new)) < c_min:
                            x_min = x_near
                            c_min = cost(x_near) + c(line(x_near.q,q_new))
                x_min.add_child(x_new)
                x_new.add_cost(c_min)

                #check if goal reached
                if self.goal_reached(line(x_min.q,x_new.q)):
                    print("goal reached")
                    goal_reached = True
                
                #visulize the updated tree
                draw_branch(self.map,x_min.q,q_new)
                        
                #rewrite the tree 
                for x_near in X_near:
                    if self.is_collision_free(line(q_new,x_near.q)):
                        if cost(x_new) + c(line(q_new,x_near.q)) < cost(x_near):
                            x_parent = parent(x_near)
                            x_parent.remove_child(x_near)
                            x_new.add_child(x_near)
                            x_near.add_cost(cost(x_new) + c(line(q_new,x_near.q)))
                
                            #check if goal reached
                            if self.goal_reached(line(x_new.q,x_near.q)):
                                print("goal reached")
                                goal_reached = True

                            #visulize the updated tree
                            delete_branch(self.map,x_parent.q,x_near.q)
                            draw_branch(self.map,x_new.q,x_near.q)

                if goal_reached:
                    pygame.draw.circle(self.map,(255,0,0),self.goal,radius =5,width=5)
                    pygame.display.update()
                    return self.tree


    def sample_free(self):
        q_rand = (randint(1,self.map.get_width()-1),randint(1,self.map.get_height()-1))
        if self.is_point_collision_free(q_rand):
            return q_rand
        else:
            return self.sample_free()

    def nearest(self,x,q_rand,e_dist = np.inf,x_nearest_prev=None):
        x_nearest = None
        min_dist = None
        if x is not None:
            temp_dist = ecludian(x.q,q_rand)
            if temp_dist < e_dist:
                min_dist = temp_dist
                x_nearest = x
            else:
                x_nearest = x_nearest_prev
                min_dist = e_dist
                    
            for x_ in x.children:
                min_dist, x_nearest = self.nearest(x_,q_rand,min_dist,x_nearest)
        return min_dist,x_nearest
             

    
    def steer(self, q_nearest, q_rand):
        step = self.step_size
        if ecludian(q_nearest,q_rand) < self.step_size :
            return q_rand
        else:
            x1,y1 = q_nearest
            x2,y2 = q_rand
            a = (y2-y1)/(x2-x1+np.finfo(np.float32).eps)
            x3 = (step**2/(a**2 +1))**0.5 + x1
            y3 =  a * ((step**2/(a**2 +1))**0.5) + y1
        
        return (round(x3),round(y3))

    def is_collision_free(self,line):
        if self.is_point_collision_free(line[-1]) and self.is_point_collision_free(line[0]):
            for q in line:
                if self.map.get_at(q) == self.obstacles_color:
                    return False
            return True
        return False
    
    def is_point_collision_free(self,q):
        x, y = q
        if (x>0 and y>0) and x < self.map.get_width() and y < self.map.get_height():
            if self.map.get_at(q) != self.obstacles_color:
                return True
        return False
        
    def goal_reached(self,line):
        for q in reversed(line):
            if self.map.get_at(q) == self.goal_color:
                return True
        return False
        
        
        

if __name__=="__main__":
    

        # Initializing Pygame
    pygame.init()
    
    # Initializing surface
    map = pygame.display.set_mode((600,600))

    map.fill((255,255,255))
    
    #draw star and goal points
    q_star=(100,20)
    q_goal=(400,500)
    star_color = (255,0,255,255)
    goal_color = (0,0,255,255)
    pygame.draw.circle(map,star_color,center=q_star,radius=5,width=5)
    pygame.draw.circle(map,goal_color,center=q_goal,radius=5,width=5)

    #draw obstacles
    obstacle_color = (0,255,0,255)
    pygame.draw.rect(map,obstacle_color,[300,300,50,50],width=50)
    pygame.draw.rect(map,obstacle_color,[100,400,50,50],width=50)
    pygame.draw.circle(map,obstacle_color,center=(400,60),radius=40,width=40)
    pygame.draw.circle(map,obstacle_color,center=(250,60),radius=40,width=40)
    pygame.draw.rect(map,obstacle_color,[100,80,50,50],width=50)
    pygame.draw.rect(map,obstacle_color,[600,400,50,50],width=50)
    pygame.draw.rect(map,obstacle_color,[10,80,50,50],width=50)
    pygame.draw.rect(map,obstacle_color,[300,130,50,50],width=50)


    # Initializing RTT
    rrt = RRT(map=map,q_star=q_star,q_goal=q_goal,step_size=25,goal_threshold=7, obstacles_color=obstacle_color,goal_color = goal_color)
    #Build RRT
    rrt.build_rrt(int(1e6))
    input('Press ENTER to exit')
    
    
    


