import numpy as np
import matplotlib.pyplot as plt
from random import randint
import pygame
from utils import *
from tree import Node
from rrt import RRT


class RRTStar(RRT):
    def __init__(self,map, q_star, q_goal,goal_threshold=8, rewire_radius = 8, obstacles_color= (0,255,0,255)):
        """ 
            q_star : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        
        """
        super().__init__(map, q_star, q_goal,goal_threshold, obstacles_color)
        self.tree.add_cost(0)
        self.rewire_radius = rewire_radius

    def build_rrt(self,steps):
        goal_reached = False
        for i in range(steps):
            q_rand = self.sample_free()
            _,x_nearest = self.nearest(self.tree,q_rand)
            q_new, trajectory_ = self.steer(x_nearest.q,q_rand)
            
            if self.is_collision_free(trajectory_): 
                # find list of x nodes that lie in the circle centerd at q_new
                X_near = self.near(self.tree,q_new,self.rewire_radius)     
                
                 #connect x_new node to the near x node that result in a minimum-cost path
                x_new = Node(q = q_new)
                x_min = x_nearest
                c_min = cost(x_nearest) + ecludian(x_nearest.q,q_new)
                for x_near in X_near:
                    if self.is_collision_free(trajectory(x_near.q,q_new)):
                        if cost(x_near) + ecludian(x_near.q,q_new) < c_min:
                            x_min = x_near
                            c_min = cost(x_near) + ecludian(x_near.q,q_new)
                x_min.add_child(x_new)
                x_new.add_cost(c_min)

                #check if goal reached
                if is_goal_reached(trajectory(x_min.q,x_new.q),self.goal,self.goal_threshold):
                    goal_reached = True
                
                #visulize the updated tree
                draw_trajectoy(self.map,trajectory(x_min.q,x_new.q),width=2)
                        
                #rewrite the tree 
                for x_near in X_near:
                    if self.is_collision_free(trajectory(x_new.q,x_near.q)):
                        if cost(x_new) + ecludian(x_new.q,x_near.q) < cost(x_near):
                            x_parent = parent(x_near)
                            x_parent.remove_child(x_near)
                            x_new.add_child(x_near)
                            x_near.add_cost(cost(x_new) + ecludian(x_new.q,x_near.q))
                
                            #check if goal reached
                            if is_goal_reached(trajectory(x_new.q,x_near.q),self.goal,self.goal_threshold):
                                goal_reached = True

                            #visulize the updated tree
                            delete_trajectory(self.map,trajectory(x_parent.q,x_near.q),width=2)
                            draw_trajectoy(self.map,trajectory(x_new.q,x_near.q),width=2)

                if goal_reached:
                    draw_point(self.map,self.goal,raduis=self.goal_threshold,width=self.goal_threshold,color=(255,0,0))
                    draw_trajectories_path(self.map,x_new,width=4,color=(255,0,0))
                    return self.tree

    
    def near(self,x,q,r):
        """
        returns list of nodes lie in the cirlce centered at q with raduis r

        Args:
            x (Node): root node
            q dict: {"x":x,"y":y} center node
            r (Int): circle raduis

        Returns:
            Xnear: list of near nodes
        """
        X_near = []
        dist = ecludian(x.q,q)
        if(dist < r):
            X_near.append(x)
        for x_ in x.children:
            X_near.extend(self.near(x_,q,r))
            
        return X_near
        
        
        

if __name__=="__main__":
    

        # Initializing Pygame
    pygame.init()
    
    # Initializing surface
    map = pygame.display.set_mode((600,600))

    map.fill((255,255,255))
    
    #draw star and goal points
    q_start={"x":100,"y":20,"theta":0.3,"delta":0.1,"beta":0.01}
    q_goal={"x":400,"y":500,"theta":0,"delta":0,"beta":0}
    star_color = (255,0,255,255)
    goal_color = (0,0,255,255)
    pygame.draw.circle(map,star_color,center=(q_start["x"],q_start["y"]),radius=5,width=5)
    pygame.draw.circle(map,goal_color,center=(q_goal["x"],q_goal["y"]),radius=7,width=7)

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
    rrt = RRTStar(map=map,q_star=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
    #Build RRT
    rrt.build_rrt(int(1e6))
    input('Press ENTER to exit')
    
    
    


