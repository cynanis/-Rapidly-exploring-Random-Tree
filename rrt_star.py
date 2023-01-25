import numpy as np
import matplotlib.pyplot as plt
from random import randint
import pygame
from utils import line, cost, parent, c, ecludian

class Node:

    def __init__(self,data):
        """ data = (x,y) """
        self.data = data
        self.children = []
        self.cost = None
        self.parent = None

    def add_child(self,child_node):
        child_node.parent = self
        self.children.append(child_node)
    
    def remove_child(self,child_node):
        self.children = [child for child in self.children if child is not child_node]
    
    def set_cost(self,cost):
        self.cost = cost    

class RRT:
    def __init__(self, map, x_star, x_goal, step_size = 10,goal_threshold=8, obstacles_color= (0,255,0,255)):
        """ 
            x_star = (x,y)
            x_goal = (x,y)
        
        """
        self.map = map
        self.tree = Node(data = x_star)
        self.goal =  x_goal
        self.step_size = step_size
        self.goal_threshold = goal_threshold
        self.obstacles_color = obstacles_color

    def build_rrt(self,steps):
        for i in range(steps):
            x_rand = self.sample_free()
            node_nearest, x_nearest = self.nearest(x_rand)
            x_new = self.steer(x_nearest,x_rand)
            if self.is_collision_free(line(x_nearest,x_new)): 
                # find list of nodes that lie in the circle centerd at x_new
                X_near = self.near(self.tree,x_new,20)     
                
                 #connect x_new to the near node that result in a minimum-cost path
                node_new = Node(data = x_new)
                node_new_parent = node_nearest
                c_min = cost(node_nearest) + c(line(x_nearest,x_new))
                for node_near in X_near:
                    if self.is_collision_free(line(node_near.data,x_new)):
                        if cost(node_near) + c(line(node_near.data,x_new)) < c_min:
                            node_new_parent = node_near
                            c_min = cost(node_near) + c(line(node_near.data,x_new))
                node_new_parent.add_child(node_new)
                node_new.set_cost(c_min)
                #visulize the updated tree
                self.visulize_tree(node_new_parent.data,x_new)
                    
                #rewrite the tree 
                for node_near in X_near:
                    if self.is_collision_free(line(x_new,node_near.data)):
                        if cost(node_new) + c(line(x_new,node_near.data)) < cost(node_near):
                            parent_near = parent(node_near)
                            parent_near.remove_child(node_near)
                            node_new.add_child(node_near)
                            node_near.set_cost(cost(node_new) + c(line(x_new,node_near.data)))
                            #visulize the updated tree
                            self.visulize_deleted_branch(parent_near.data,node_near.data)
                            self.visulize_tree(node_new.data,node_near.data)
                
                if self.goal_reached(x_new):
                    pygame.draw.circle(self.map,(255,0,0),self.goal,radius =5,width=5)
                    pygame.display.update()
                    return self.tree




    def sample_free(self):
        x_rand = (randint(1,self.map.get_width()-1),randint(1,self.map.get_height()-1))
        if self.is_point_collision_free(x_rand):
            return x_rand
        else:
            return self.sample_free()

    def nearest(self,x_rand):
        
        root = self.tree
        node_nearest = root
        ecludian_dist = np.inf
        if root is not None:
            ecludian_dist, node_nearest = self.tree_traversal(root,x_rand,ecludian_dist,node_nearest)
        
        return node_nearest , node_nearest.data
             

    
    def steer(self, x_nearest, x_rand):
        step = self.step_size
        if ecludian(x_n=x_nearest,x_r=x_rand) < self.step_size :
            return x_rand
        else:
            x1,y1 = x_nearest
            x2,y2 = x_rand
            a = (y2-y1)/(x2-x1+np.finfo(np.float32).eps)
            x3 = (step**2/(a**2 +1))**0.5 + x1
            y3 =  a * ((step**2/(a**2 +1))**0.5) + y1

        return (int(x3),int(y3))

    def is_collision_free(self,line):
        if self.is_point_collision_free(line[-1]) and self.is_point_collision_free(line[0]):
            for x_ in line:
                if self.map.get_at(x_) == self.obstacles_color:
                    return False
            return True
        return False
    
    def is_point_collision_free(self,x_):
        x, y = x_
        if (x>0 and y>0) and x < self.map.get_width() and y < self.map.get_height():
            if self.map.get_at(x_) != self.obstacles_color:
                return True
        return False
    
    def near(self,node,x,r):
        list = []
        dist = ecludian(x,node.data)
        if(dist < r):
            list.append(node)
        for node_ in node.children:
            list.extend(self.near(node_,x,r))
        return list
        
        
    def goal_reached(self,x_new):
        if ecludian(x_new,self.goal) < self.goal_threshold:
            return True
        return False


    def tree_traversal(self ,node ,x_rand, e_dist,prev_node_nearest):
        
        node_nearest = None
        min_dist = None
        temp_dist = ecludian(x_n = node.data,x_r = x_rand)
        if temp_dist < e_dist:
            min_dist = temp_dist
            node_nearest = node
        else:
            node_nearest = prev_node_nearest
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
    
    def visulize_deleted_branch(self,x_1, x_2):
        color = (255,255,255)
        # Drawing Rectangle
        pygame.draw.line(self.map,color,start_pos=x_1,end_pos=x_2,width=1)
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
    
    
    


