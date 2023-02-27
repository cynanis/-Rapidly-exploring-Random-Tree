import numpy as np
from random import randint
from .utils import *
from .tree import Node
from .rrt import RRT

class RRTStar(RRT):
    def __init__(self,map, q_start, q_goal,goal_threshold=8, rewire_radius = 8, obstacles_color= (0,255,0,255)):
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        """
        super().__init__(map, q_start, q_goal,goal_threshold, obstacles_color)
        self.tree.add_weight(0)
        self.rewire_radius = rewire_radius

    def build(self,steps):
        x_goal_old = None
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
                X_near = self.near(self.tree,q_new,self.rewire_radius)     
                 #connect the x_new node to the near x_min=x_near node that result in a minimum-cost c_min path
                x_new = Node(q = q_new)
                x_min = x_nearest
                c_min = cost(x_nearest) + c(line_n)
                for x_near in X_near:
                    line_n = trajectory(x_near.q,x_new.q)
                    c_new = cost(x_near) + c(line_n)
                    if self.collision_free(line_n):
                        if c_new < c_min:
                            x_min = x_near
                            c_min = c_new
                
                #update the the new node weight
                x_new.add_weight(c(line_n))
                #add x_new node to the tree
                x_min.add_child(x_new)                
                #visulize the updated tree
                draw_line(self.map,trajectory(x_min.q,x_new.q),width=1,color=(0,0,0))
                        
                #rewrite the tree 
                for x_near in X_near:
                    c_near = cost(x_near)
                    line_n2r = trajectory(x_new.q,x_near.q)
                    c_new = cost(x_new) + c(line_n2r) 
                    if self.collision_free(line_n2r):
                        #if near node achieve less cost change its parent 
                        if c_new < c_near:
                            x_near_parent = parent(x_near)
                            x_near.parent.remove_child(x_near)
                            x_near.add_weight(c(line_n2r))
                            x_new.add_child(x_near)                

                            #visulize the updated tree
                            delete_line(self.map,trajectory(x_near_parent.q,x_near.q),width=1)
                            draw_line(self.map,trajectory(x_new.q,x_near.q),width=1)

                if self.in_goal_region(x_new.q):
                    draw_point(self.map,self.q_goal,raduis=self.goal_threshold,width=self.goal_threshold,color=(0,0,255))
                    draw_path(self.map,x_new,width=2,color=(0,0,255))

            if cv.waitKey(1) == ord('q'):
                break
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
        
        
    