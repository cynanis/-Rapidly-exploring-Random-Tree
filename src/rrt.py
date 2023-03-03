import numpy as np
from random import randint
import random
from .utils import *
from .tree import Node
from .model import BicycleModel
from .control2d import steer
from .cfg import params
import cv2 as cv

class RRT:
    def __init__(self, size, q_start, q_goal,goal_threshold=8, obstacles_color= (0,255,0,255),end_points_colors=(255,0,0)):
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        
        """
        self.size = size
        self.ends_color = end_points_colors
        self.q_start = q_start
        self.q_goal =  q_goal
        self.goal_threshold = goal_threshold
        self.obstacles_color = obstacles_color
        self.tree = Node(q = q_start,w=0,u=0)
        self.path = []
        self.motion_Model = BicycleModel(q_init=q_start)
        self.map = None
        

    def build(self,steps,name="RRT"):
        self.map = self.init_map(name=name)
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
                draw_line(self.map,line_n,width=1,color=(0,0,0))
                
                #if goal reached draw path
                if self.in_goal_region(x_new.q):
                    if x_best != None:
                        if self.cost(x_new) >= self.cost(x_best):
                            continue
                    #draw path
                    print("===> new path cost: {:.3f}".format(self.cost(x_new)))
                    print("===> drawing new path")
                    draw_point(self.map,self.q_goal,raduis=self.goal_threshold,width=self.goal_threshold,color=(255,0,0),name=name)
                    #erase old path
                    self.erase_path(self.path,name=name)
                    #extract the new path from new goal
                    self.path = self.extract_path(x_new)   
                    #draw the new path 
                    self.draw_path(self.path,color=(255,0,0),width=2,name=name)
                    x_best = x_new
                    
            cv.imshow(name,self.map)
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
        if (x>0 and y>0) and x < self.map.shape[1] and y < self.map.shape[0]:
            if not np.array_equal(self.map[y][x],self.obstacles_color):
                return True
        return False
        
    def in_goal_region(self,q):
      
        if ecludian(q,self.q_goal) <= self.goal_threshold+1:
            return True
        return False    

    def cost(self,v):
        if v is None:
            return 0
        elif v.w is not None:
            return v.w + self.cost(v.parent)
        else:
            return self.c(self.steer(parent(v).q,v.q)) + self.cost(parent(v))

    def c(self,line):
        dist = 0
        len_ = len(line)
        for i in range(len_):
            if i < len_ - 1:
                dist += ecludian(line[i],line[i+1])
        return dist
    
    def init_map(self,name="RRT"):
        # Initializing surface
        map = np.ones(self.size, dtype=np.uint8)*255
        draw_end_points(map,self.q_start,self.q_goal,self.ends_color,raduis=self.goal_threshold,name=name)
        draw_obstacles(map,self.obstacles_color,name=name)
        return map
    
    def reset_map(self,name="RRT",draw_tree=True):
        print("==> deleting old path")
        self.map[:][:][:] = 255
        draw_end_points(self.map,self.q_start,self.q_goal,self.ends_color,raduis=5,name=name)
        draw_obstacles(self.map,self.obstacles_color,name=name)
        if draw_tree:
            redraw_tree(self.map,self.tree,color=(0,0,0),width=1,name=name)
    
    def extract_path(self,x_goal):
        path = []
        if x_goal == None:
            return path
        elif x_goal.parent == None:
            return path
        
        path.extend(self.extract_path(x_goal.parent))
        path.append((x_goal.parent.q,x_goal.q))
        return path

    def draw_path(self,path,color=(255,0,0),width=2,name="RRT"):
        for q1,q2 in path:
            _,line = self.steer(q1,q2)
            draw_line(self.map,line,color=color,width=width,name=name)
            
    def erase_path(self,path,color=(0,0,0),width=2,name="RRT"):
        for q1,q2 in path:
            _,line = self.steer(q1,q2)
            delete_line(self.map,line,width=width,color=(255,255,255),name=name)
            draw_line(self.map,line,color=color,width=1,name=name)   
                    

        return path
    
    @staticmethod
    def tree_len(node):
        i = 0
        if node is not None:
            i += 1
        for node_ in node.children:
            i += RRT.tree_len(node_)
            
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
    
    
    
    
    
    


