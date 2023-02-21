import pygame
from model import BicycleModel
from control2d import lateral_control_stanly
from cfg import params

def straight_line(q1, q2):
    """denote the straight-line path from x1 to x2"""
    x_1 , y_1 = q1["x"],q2["y"]
    x_2, y_2  = q2["x"],q2["y"]
    a = (y_2 - y_1)/(x_2 - x_1+1e-7)
    start = x_1
    end = x_2+1
    step = 1
    if(x_1 > x_2):
        start = x_1+1
        end = x_2
        step = -1
    line_  = [{"x":x, "y":round(a*(x-x_1)+y_1)} for x in range(start,end,step)]
    return line_


def trajectory(q1,q2):
    line_ = [q1]
    motion_model = BicycleModel(q_init=q1)
    for i in range(10):
        motion_model.update_state(q_state = line_[i])
        v = params["v_max"]
        delta = lateral_control_stanly(line_[i],q2,v)
        line_.append(motion_model.forward(v,delta))
        
    return line_

def parent(v):
    if v.parent is not None:
        return v.parent
    return v

def cost(v):
    if v.cost is not None:
        return v.cost
    elif v.parent is None:
        return 0
    
    return cost(parent(v)) + c(line(parent(v).q,v.q))

def c(line):
    return ecludian(line[0],line[-1])

def ecludian(q_n, q_r):
    x1 , y1, = q_n["x"],q_n["y"]
    x2, y2 = q_r["x"], q_r["y"]
    return ((x2-x1)**2 + (y2 - y1)**2)**0.5


def is_goal_reached(line,q_goal,goal_threshold):

    for q in reversed(line):
        if ecludian(q,q_goal) < goal_threshold:
            return True
    return False


def draw_branch(map,q_start, q_end,color = (0,0,0),width = 1):
    # Drawing line
    pygame.draw.line(map,color,start_pos=(q_start["x"],q_start["y"]),end_pos=(q_end["x"],q_end["y"]),width=width)
    pygame.display.update() 

def delete_branch(map,q_start, q_end,color = (255,255,255),width=1):
    # delete line
    pygame.draw.line(map,color,start_pos=(q_start["x"],q_start["y"]),end_pos=(q_end["x"],q_end["y"]),width=width)
    pygame.display.update() 

def draw_point(map,q_point,color = (0,0,0),raduis = 0.5,width=1):
    # Drawing point
    pygame.draw.circle(map,color,center=(q_point["x"],q_point["y"]),radius = raduis,width=width)
    pygame.display.update() 
    
def draw_path(map,x_goal,color=(255,0,0),width=2):

    if x_goal is None:
        return
    draw_path(map,x_goal.parent,color,width)
    if x_goal.parent is not None:
        draw_branch(map,x_goal.parent.q,x_goal.q,color=color,width =width)

def draw_trajectoy(map,trajectory,color = (0,0,0),width=1):
    len_ = len(trajectory)
    for i in range(len_):
        draw_branch(map,trajectory[i],trajectory[i+1],color,width)
        if i == len_-2:
            return

def draw_trajectories_path(map,x_goal,color=(255,0,0),width=2):
    if x_goal is None:
        return
    draw_trajectories_path(map,x_goal.parent,color,width)
    if x_goal.parent is not None:
        trajectory_ = trajectory(x_goal.parent.q,x_goal.q)
        draw_trajectoy(map,trajectory_,color,width)