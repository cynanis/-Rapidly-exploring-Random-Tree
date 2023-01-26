import numpy as np
import matplotlib.pyplot as plt
from random import randint
import pygame
from utils import *
from tree import Node
from rrt import RRT


class RRTStar(RRT):
    def __init__(self, map, q_star, q_goal, step_size = 10,goal_threshold=8,rewire_radius = 8, obstacles_color= (0,255,0,255)):
        """ 
            q_star = (x,y)
            q_goal = (x,y)
        
        """
        super().__init__(map, q_star, q_goal, step_size,goal_threshold, obstacles_color)
        self.tree.add_cost(0)
        self.rewire_radius = rewire_radius

    def build_rrt(self,steps):
        goal_reached = False
        for i in range(steps):
            q_rand = self.sample_free()
            _,x_nearest = self.nearest(self.tree,q_rand)
            q_new = self.steer(x_nearest.q,q_rand)
            if self.is_collision_free(line(x_nearest.q,q_new)): 
                # find list of x nodes that lie in the circle centerd at q_new
                X_near = self.near(self.tree,q_new,self.rewire_radius)     
                
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
                if is_goal_reached(line(x_min.q,x_new.q),self.goal,self.goal_threshold):
                    print("goal reached")
                    goal_reached = True
                
                #visulize the updated tree
                draw_branch(self.map,x_min.q,x_new.q)
                        
                #rewrite the tree 
                for x_near in X_near:
                    if self.is_collision_free(line(x_new.q,x_near.q)):
                        if cost(x_new) + c(line(x_new.q,x_near.q)) < cost(x_near):
                            x_parent = parent(x_near)
                            x_parent.remove_child(x_near)
                            x_new.add_child(x_near)
                            x_near.add_cost(cost(x_new) + c(line(x_new.q,x_near.q)))
                
                            #check if goal reached
                            if is_goal_reached(line(x_new.q,x_near.q),self.goal,self.goal_threshold):
                                print("goal reached")
                                goal_reached = True

                            #visulize the updated tree
                            delete_branch(self.map,x_parent.q,x_near.q)
                            draw_branch(self.map,x_new.q,x_near.q)

                if goal_reached:
                    pygame.draw.circle(self.map,(255,0,0),self.goal,radius =5,width=5)
                    pygame.display.update()
                    self.draw_path(x_new,width=3)

                    return self.tree

    
    def near(self,x,q,r):
        """
        returns list of nodes lie in the cirlce centered at q with raduis r

        Args:
            x (Node): root node
            q (Typle): (x,y) circle center
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
    rrt = RRTStar(map=map,q_star=q_star,q_goal=q_goal,step_size=25,goal_threshold=7, obstacles_color=obstacle_color)
    #Build RRT
    rrt.build_rrt(int(1e6))
    input('Press ENTER to exit')
    
    
    


