import numpy as np
from random import randint
from .utils import *
from .tree import Node
from .rrt_star import RRTStar

class InformedRRTStar(RRTStar):
    def __init__(self,map, q_start, q_goal,goal_threshold=8, rewire_radius = 8, obstacles_color= (0,255,0,255)):
        """ 
            q_start : starting state {"x":x,"y":y,"theta"=theta,"delta":delta,"beta":beta}
            q_goal : goal state {"x":x,"y"=y,"theta"=theta,"delta":delta,"beta":beta}
        """
        super().__init__(map, q_start, q_goal,goal_threshold,rewire_radius, obstacles_color)
        self.sample_config = self.hyperellipsoid_config()
        
    def build(self,steps):
        X_soln = set()
        for i in range(steps):
            #the transverse diameter is cbest of the special hyperellipsoid
            c_best = self.transverse_diameter(X_soln)
            #get a random location sample in the map
            q_rand = self.sample(c_best)
            #find the nearest node to the random sample
            _,x_nearest = self.nearest_neighbor(self.tree,q_rand)
            #genertate a steering trajectory from nearest node to random sample
            q_new, line_n  = self.steer(x_nearest.q,q_rand)
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
                draw_trajectoy(self.map,trajectory(x_min.q,x_new.q),width=1)
                        
                #rewrite the tree 
                for x_near in X_near:
                    c_near = cost(x_near)
                    line_n2r = trajectory(x_new.q,x_near.q)
                    c_new = cost(x_new) + c(line_n2r) 
                    if self.collision_free(line_n2r):
                        #if near node achieve less cost change its parent 
                        if c_new < c_near:
                            x_parent = parent(x_near)
                            x_parent.remove_child(x_near)
                            x_new.add_child(x_near)                
                            x_near.add_weight(c(line_n2r))
                            #visulize the updated tree
                            # delete_trajectory(self.map,trajectory(x_parent.q,x_near.q),width=1)
                            draw_trajectoy(self.map,trajectory(x_new.q,x_near.q),width=1)

                if self.in_goal_region(x_new.q):
                    X_soln.add(x_new)
                    draw_point(self.map,self.q_goal,raduis=self.goal_threshold,width=self.goal_threshold,color=(255,0,0))
                    draw_trajectories_path(self.map,x_new,width=4,color=(255,0,0))
                    # return self.tree
            # if i%20 == 0:
                

    
    def transverse_diameter(self,X_soln):
        c_best = np.inf
        if X_soln:
            costs = {node: cost(node) for node in X_soln}
            x_best = min(costs, key=costs.get)
            c_best = costs[x_best]    
        return c_best
        
    def sample(self,c_max):
        if c_max < np.inf:
            c_min, Q_center, C = self.sample_config
            r = [c_max / 2.0,
                 np.sqrt(c_max ** 2 - c_min ** 2) / 2.0,
                 np.sqrt(c_max ** 2 - c_min ** 2) / 2.0]
            L = np.diag(r)
            draw_ellipse(self.map,self.q_start,self.q_goal,c_max,c_min,Q_center)
            while True:
                Q_ball = self.sample_unit_nball()
                Q_rand = np.dot(np.dot(C,L),Q_ball) + Q_center
                q_rand = {"x":round(Q_rand[0][0]),"y":round(Q_rand[1][0]),"theta":np.random.uniform(-np.pi,np.pi)}
                if self.point_collision_free(q_rand):
                    return q_rand 
        else:
            return self.sample_free()    

    def rotaion_to_world_frame(self):
        a1 = np.array([
                        [(self.q_goal["x"]-self.q_start["x"])/ecludian(self.q_goal,self.q_start)],
                        [(self.q_goal["y"]-self.q_start["y"])/ecludian(self.q_goal,self.q_start)],
                        [0.0],
                    ])
        I1 = np.array([[1.0],[0],[0]])
        M = np.dot(a1,I1.T)
        U,_,V_T = np.linalg.svd(M)
        C = np.dot(np.dot(U,np.diag([1.0,1.0,np.linalg.det(U)*np.linalg.det(V_T.T)])),V_T)
        return C
    
    def sample_unit_nball(self):
        while True:
            x , y = np.random.uniform(-1,1),np.random.uniform(-1,1)
            if x**2 + y**2 < 1.0:
                return np.array([[x],[y],[0.0]])
    
    def hyperellipsoid_config(self):
            c_min = ecludian(self.q_goal,self.q_start)
            Q_center = np.array([[(self.q_start["x"]+self.q_goal["x"]) / 2.0],
                            [(self.q_start["y"]+self.q_goal["y"]) / 2.0], [0.0]])
            C = self.rotaion_to_world_frame()
            return (c_min,Q_center,C)
    
  