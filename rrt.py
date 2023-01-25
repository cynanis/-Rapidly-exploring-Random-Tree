import numpy as np
import matplotlib.pyplot as plt
from random import randint
import pygame


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

    def get_xy(self):
        return self.data["V"]

class RRT:
    def __init__(self, map, x_star, x_goal, step_size = 10,goal_threshold=8, obstacles_color= (0,255,0,255)):
        """ 
            x_star = (x,y)
            x_goal = (x,y)
        
        """
        self.map = map
        self.tree = Node(data = {"V":x_star})
        self.goal = Node(data = {"V": x_goal})
        self.step_size = step_size
        self.goal_threshold = goal_threshold
        self.obstacles_color = obstacles_color

    def build_rrt(self,steps,show_map = True):
        for i in range(steps):
            x_rand = self.random_sample()
            x_nearest, node_nearest, ecludian_dist = self.nearest(x_rand)
            x_new = self.steer(x_nearest,x_rand)
            if self.is_collision_free(x_nearest,x_new): 
                node_new = Node(data = {"V":x_new, "E":ecludian_dist})
                node_nearest.add_child(node_new)
                if show_map:
                    self.visulize_tree(x_new,x_nearest)
                if self.goal_reached(x_new):
                    pygame.draw.circle(self.map,(255,0,0),self.goal.data["V"],radius =5,width=5)
                    pygame.display.update()
                    return self.tree




    def random_sample(self):
        x = randint(1,self.map.get_width()-1)
        y = randint(1,self.map.get_height()-1)
        return (x,y)

    def nearest(self,x_rand):

        
        root = self.tree
        node_nearest = root
        ecludian_dist = np.inf
        if root is not None:
            ecludian_dist, node_nearest = self.tree_traversal(root,x_rand,ecludian_dist,node_nearest)
        
        return node_nearest.data["V"], node_nearest, ecludian_dist
             

    
    def steer(self, x_nearest, x_rand):
        step = self.step_size
        if self.ecludian(x_n=x_nearest,x_r=x_rand) < self.step_size :
            return x_rand
        else:
            x1,y1 = x_nearest
            x2,y2 = x_rand
            a = (y2-y1)/(x2-x1+np.finfo(np.float32).eps)
            x3 = (step**2/(a**2 +1))**0.5 + x1
            y3 =  a * ((step**2/(a**2 +1))**0.5) + y1

        return (int(x3),int(y3))

    def is_collision_free(self,x_nearest,x_new):
        x, y = x_new
        if (x>0 and y>0) and x < self.map.get_width() and y < self.map.get_height():
            if self.map.get_at(x_new) != self.obstacles_color:
                return True
        return False
    
    def goal_reached(self,x_new):
        if self.ecludian(x_new,self.goal.data["V"]) < self.goal_threshold:
            return True
        return False

    def ecludian(self, x_n, x_r):
        x1 , y1 = x_n
        x2, y2 = x_r
        return np.sqrt((x2-x1)**2 + (y2 - y1)**2)

    def tree_traversal(self ,node ,x_rand, e_dist,pre_node_nearest):
        
        node_nearest = None
        min_dist = None
        temp_dist = self.ecludian(x_n = node.get_xy(),x_r = x_rand)
        if temp_dist < e_dist:
            min_dist = temp_dist
            node_nearest = node
        else:
            node_nearest = pre_node_nearest
            min_dist = e_dist
                
        for node_ in node.children:
            min_dist, node_nearest = self.tree_traversal(node_,x_rand,min_dist,node_nearest)
        
        return min_dist, node_nearest

    def visulize_tree(self,x_nearest, x_new):
        color = (0,0,0)
        # Drawing Rectangle
        pygame.draw.circle(self.map,color,center=x_new,radius=2,width=2)
        pygame.draw.line(self.map,color,start_pos=x_nearest,end_pos=x_new,width=1)
        pygame.display.update() 
        
        
        
        

if __name__=="__main__":
    

        # Initializing Pygame
    pygame.init()
    
    # Initializing surface
    map = pygame.display.set_mode((600,600))

    map.fill((255,255,255))
    
    #draw star and goal points
    x_star=(100,20)
    x_goal=(400,500)
    pygame.draw.circle(map,(0,0,255),center=x_star,radius=5,width=5)
    pygame.draw.circle(map,(0,0,255),center=x_goal,radius=5,width=5)

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
    rrt = RRT(map=map,x_star=x_star,x_goal=x_goal,step_size=25,goal_threshold=7, obstacles_color=obstacle_color)
    #Build RRT
    rrt.build_rrt(int(1e6))
    input('Press ENTER to exit')
    
    
    


