import numpy as np
from cfg import params

class BicycleModel():
    def __init__(self, q_init):
        
        """ q = {"x":x,"y":y,"theta":theta,"delta":delta,"beta":beta} """
        
        self.q_state = q_init
        
    def update_state(self,q_state):
        self.q_state = q_state    
        
    def forward(self, v, delta):
        #w steering angle rate  
        #v bicycle speed
        # ==================================

        
        if v > 0:
            v = min(v,params["v_max"])  
        else:
            v = max(v,-params["v_max"])  
        
        if delta >0:
            delta = min(delta,params["steer_max"])
        else:
            delta = max(delta,-params["steer_max"])
        #print("q_nearest: ",self.q_state)
        xc,yc,theta,delta_,beta =self.q_state.values()
        #setup the next states using the differential equations     
        xc = xc + (v*np.cos(theta + beta))*params["sample_time"]
        yc  = yc + (v*np.sin(theta + beta))*params["sample_time"]
        theta = theta + (v*np.cos(beta)*np.tan(delta_)/params["L"]) * params["sample_time"]
        #delta = delta + w * params["sample_time"]


        beta = np.arctan(params["lr"] * np.tan(delta) / params["L"])
        # ==================================
        q_new = {"x":round(xc),"y":round(yc),"theta":theta,"delta":delta,"beta":beta}
        #print("q_new: ",q_new)
        return q_new

    