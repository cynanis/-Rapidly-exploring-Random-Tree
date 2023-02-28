import numpy as np
from .cfg import params
from .model import BicycleModel

# def pure_pursuit(q_state,q_d,v):
#         xd,yd,theta = q_d["x"],q_d["y"]
#         # steering angle rate pure pursuit control
#         # alpha: look ahead direction
#         alpha = np.arctan((yd - q_state["y"])/(xd - q_state["x"]+1e-7)) - q_state["y"]
#         delta = np.arctan((2*params["L"]*np.sin(alpha))/(params["kp_ld"]*v))
#         #w = (delta_d - self.q_state["delta"])/params["sample_time"]
        # return v, delta
    
def lateral_control_stanly(q_state,q_d,v):
        # Stanly Controller
        xd,yd,theta = q_d["x"],q_d["y"],q_d["theta"]
        xc,yc,theta_c = q_state["x"],q_state["y"],q_state["theta"]
        
        ### HEADING ERROR ###
        
        #path equation aX + bY + c = 0 => Y = -a/b*X -c
        #path slop (Yf - Yi)/(Xi - Xf)
        path_slop = np.tan(theta)

        #heading of the path
        path_heading = theta
        
        #vehicle heading
        vehicle_heading = theta_c

        #(yaw angle) heading of the vehicle with respect to the path
        heading_error = path_heading - vehicle_heading  
        if heading_error > np.pi:
            heading_error -= 2*np.pi
        elif heading_error < -np.pi:
            heading_error += 2*np.pi
        #print("heading error {}".format(heading_error))
            
        # #CROSSTRACK ERROR
        k_err = params["kp_crss"]
        
        a = -path_slop
        b = 1.0
        c = (path_slop*xd) - yd
    
        
        # cross track error 
        crosstrack_error = (a*xc + b*yc + c)/(np.sqrt(a**2 + b**2))
        vehicle_path_angle = np.arctan2(yc-yd, xc-xd)
        path_to_vehicle_diff = path_heading - vehicle_path_angle
        if path_to_vehicle_diff > np.pi:
            path_to_vehicle_diff -= 2 * np.pi
        if path_to_vehicle_diff < - np.pi:
            path_to_vehicle_diff += 2 * np.pi
        if path_to_vehicle_diff > 0:
            crosstrack_error = abs(crosstrack_error)
        else:
            crosstrack_error = - abs(crosstrack_error)
        #print("cross_error {}".format(crosstrack_error))
        
        cross_track_steering = np.arctan(k_err*crosstrack_error/v)
        #print("steer_cross_track {}".format(cross_track_steering))

        # Change the steer output with the lateral controller. 
        steer = heading_error +  cross_track_steering        
        if steer > np.pi:
            steer -= 2*np.pi
        elif steer < -np.pi:
            steer += 2*np.pi
 
        #print("steer {}".format(steer))
        return steer
    
def Longitudinal_control_pid(q_state,v_d):
    return v_d

def steer(q1, q2,steps=params["step_samples"],v_d=params["v_max"]):
    line = [q1]
    motion_model = BicycleModel(q_init=q1)
    for i in range(steps):
        motion_model.update_state(q_state = line[i])
        v = Longitudinal_control_pid(q_state=line[i],v_d=v_d)
        delta = lateral_control_stanly(line[i],q2,v)
        q_new = motion_model.forward(v,delta)
        line.append(q_new)
    return line