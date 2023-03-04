import numpy as np
from random import randint
import copy
from .tree import Node
from .rrt import RRT
from .cfg import params


class RRTStar(RRT):
    def __init__(self,map,goal_threshold=8, rewire_radius = 6,  name="RRT*"):

        super().__init__(map,goal_threshold,name)
        self.tree.add_weight(0)
        self.rewire_radius = rewire_radius

    def build(self,steps):
        x_goal = None
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
                   
                 #connect the x_new node to the near node "x_min=x_near"  which results in a minimum-cost c_min path
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
                self.map.draw_line(line_n,color=self.map.tree_color,width=1)
                        
                #rewrite the tree 
                for x_near in X_near:
                    #evaluate the x_near cost vs x_near through x_new cost
                    c_near = self.cost(x_near)
                    _,line_n2r = self.steer(x_new.q,x_near.q)
                    c_new = self.cost(x_new) + self.c(line_n2r) 
                    if self.collision_free(line_n2r):
                        #if near node achieve less cost change its parent 
                        if c_new < c_near:
                           self.rewrite(x_near,x_new,line_n2r)
                    
                if self.in_goal_region(x_new.q):   
                    if x_goal != None:
                        if self.cost(x_new) >= self.cost(x_goal):
                            continue
                    #erase prev path
                    self.map.draw_point(self.map.q_goal,raduis=self.map.end_points_raduis,color=self.map.path_color)
                    #erase old path
                    self.map.erase_path(self.path,width=2)
                    #extract the new path from new goal
                    self.path = self.extract_path(x_new)   
                    #draw the new path
                    print("===> draw new path")
                    self.map.draw_path(self.path,width=2)
                    x_goal = x_new

            if i%20 == 0:
                self.map.show_map(self.name)
              
    
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
        dist = RRT.ecludian(x.q,q)
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
    
    def rewrite(self,x_near,x_new,line_n2r,config=None,c_best=None):
        #replace rewriten edges in path
        if (x_near.parent.q, x_near.q) in self.path:
            
            #store path to x_near before rewrite
            path_to_x_near = self.extract_path(x_near)
            idx = len(path_to_x_near)

            # erase path to x_near from map
            self.map.erase_path(path_to_x_near,width=2)

            # change x_near parent in the tree
            x_near.change_parent(to=x_new)
            x_near.add_weight(self.c(line_n2r))
            
            #add the new x_near portion of the path to the whole path
            print("==> rewrite path")
            self.path = self.extract_path(x_near) + self.path[idx:] 

            #draw the new path
            self.map.draw_path(self.path,width=2)
            if config != None:
                cmin, Q_center, _ = config
                self.map.draw_ellipse(self.map.q_start,self.map.q_goal,c_best,cmin,Q_center)
        else:
            #old brach
            _,line_r = self.steer(x_near.parent.q,x_near.q)
            
            # erase old x_near branch from map
            self.map.draw_line(line_r,width=1,color=(255,255,255))
            
            # change x_near parent in the tree
            x_near.change_parent(to=x_new)
            x_near.add_weight(self.c(line_n2r))

            # draw the new x_near branch 
            self.map.draw_line(line_n2r,color=self.map.tree_color,width=1)