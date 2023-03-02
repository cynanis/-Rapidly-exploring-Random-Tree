import numpy as np
from random import randint
import copy
from .utils import *
from .tree import Node
from .rrt import RRT


class RRTStar(RRT):
    def __init__(self,size, q_start, q_goal,goal_threshold=8, rewire_radius = 6, obstacles_color= (0,255,0,255),end_points_colors=(255,0,0)):
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        """
        super().__init__(size, q_start, q_goal,goal_threshold, obstacles_color,end_points_colors)
        self.tree.add_weight(0)
        self.rewire_radius = rewire_radius

    def build(self,steps,name="RRT*"):
        self.map = self.init_map(name=name)
        x_goal = None
        c_goal = np.inf
        for i in range(steps):
            #get a random location sample in the map
            q_rand = self.sample_free()
            #find the nearest node to the random sample
            _,x_nearest = self.nearest_neighbor(self.tree,q_rand)
            #genertate a steering trajectory from nearest node to random sample
            q_new, line_n = self.steer(x_nearest.q,q_rand)
            #check for collision
            if self.collision_free(line_n): 
                # find list of x nodes that lie in the circle centerd at q_new
                r = self.search_raduis(self.tree,self.rewire_radius)
                X_near = self.near(self.tree,q_new,r)     
                 #connect the x_new node to the near x_min=x_near node that result in a minimum-cost c_min path
                x_new = Node(q = q_new)
                x_min = x_nearest
                c_min = self.cost(x_nearest) + self.c(line_n)
                for x_near in X_near:
                    _,line_n = self.steer(x_near.q,x_new.q)
                    c_new = self.cost(x_near) + self.c(line_n)
                    if self.collision_free(line_n):
                        if c_new < c_min:
                            x_min = x_near
                            c_min = c_new
                
                #update the the new node weight
                _,line_n = self.steer(x_min.q,x_new.q)
                x_new.add_weight(self.c(line_n))
                #add x_new node to the tree
                x_min.add_child(x_new)                
                #visulize the updated tree
                draw_line(self.map,line_n,width=1,color=(0,0,0),name=name)
                        
                #rewrite the tree 
                for x_near in X_near:
                    c_near = self.cost(x_near)
                    _,line_n2r = self.steer(x_new.q,x_near.q)
                    c_new = self.cost(x_new) + self.c(line_n2r) 
                    if self.collision_free(line_n2r):
                        
                        #if near node achieve less cost change its parent 
                        if c_new < c_near:
                            #replace rewriten edges in path
                            if (x_near.parent.q, x_near.q) in self.path:
                                # erase x_near from path
                                path_to_x_near_old = self.extract_path(x_near)
                                self.erase_path(path_to_x_near_old,color=(0,0,0),width=2,name=name)    
                                idx = len(path_to_x_near_old)
                                self.path = self.path[idx:]                    

                                # change x_near parent in the tree
                                x_near.change_parent(to=x_new)
                                x_near.add_weight(self.c(line_n2r))

                                # replace the erased x_near path with x_new path
                                print("==> rewrite path")
                                path_to_x_near_new = self.extract_path(x_near)
                                self.path = path_to_x_near_new + self.path
                                self.draw_path(self.path,color=(255,0,0),width=2,name=name)

                            else:
                                # erase x_near branch from map
                                _,line_old = self.steer(x_near.parent.q,x_near.q)
                                delete_line(self.map,line_old,width=1,color=(255,255,255),name=name)
                                # change x_near parent in the tree
                                x_near.change_parent(to=x_new)
                                x_near.add_weight(self.c(line_n2r))
                                #visulize the updated brach
                                draw_line(self.map,line_n2r,color=(0,0,0),width=1,name=name)
                    
                if self.in_goal_region(x_new.q):   
                    if x_goal != None:
                        if self.cost(x_new) >= self.cost(x_goal):
                            continue
                        
                    #erase prev path
                    self.erase_path(self.path,color=(0,0,0),width=2,name=name)
                    
                    #extract new path
                    self.path = self.extract_path(x_new)

                    #draw new path
                    print("===> new path cost: {:.3f}".format(self.cost(x_new)))
                    print("===> drawing new path")
                    self.draw_path(self.path,color=(255,0,0),width=2,name=name)
                    draw_point(self.map,self.q_goal,raduis=self.goal_threshold,width=self.goal_threshold,color=(255,0,0),name=name)
                    x_goal = x_new


            if cv.waitKey(1) == ord('q'):
                break
        return self.tree            
              
    
    @staticmethod
    def near(x,q,r):
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
            X_near.extend(RRTStar.near(x_,q,r))
        return X_near
        
    @staticmethod
    def search_raduis(tree,rewire_raduis):
        n = RRT.tree_len(tree) + 1
        r = max(min(rewire_raduis * np.sqrt((np.log(n) / n)),params["v_max"]*params["sample_time"]*params["step_samples"]),2)
        return r