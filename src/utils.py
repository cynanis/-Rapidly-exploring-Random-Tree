import pygame
from .model import BicycleModel
from .control2d import lateral_control_stanly
from .cfg import params
import numpy as np
import cv2 as cv

def line(q1, q2):
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
    return v.parent

def cost(v):
    if v is None:
        return 0
    elif v.w is not None:
        return v.w + cost(v.parent)
    else:
        return c(trajectory(parent(v).q,v.q)) + cost(parent(v))

def c(line):
    dist = 0
    len_ = len(line)
    for i in range(len_):
        if i < len_ - 1:
            dist += ecludian(line[i],line[i+1])
    return dist

def ecludian(q_n, q_r):
    x1 , y1, = q_n["x"],q_n["y"]
    x2, y2 = q_r["x"], q_r["y"]
    return ((x2-x1)**2 + (y2 - y1)**2)**0.5

def draw_branch(map,q_start, q_end,color = (0,0,0),width = 1,name="RRT"):
    # Drawing line
    cv.line(map,pt1=(q_start["x"],q_start["y"]),pt2=(q_end["x"],q_end["y"]),color=color,thickness=width)
    cv.imshow(name,map)
    
def delete_branch(map,q_start, q_end,color = (255,255,255),width=1,name="RRT"):
    # delete line
    draw_branch(map,q_start,q_end,color=(255,255,255),width=width,name=name)

def draw_point(map,q_point,color = (0,0,0),raduis = 0.5,width=1,name="RRT"):
    # Drawing point
    cv.circle(map,center=(q_point["x"],q_point["y"]),radius = raduis,color=color,thickness=-1)
    cv.imshow(name,map)
     
def draw_path(map,x_goal,color=(0,0,255),width=2,name="RRT"):

    if x_goal is None:
        return
    draw_path(map,x_goal.parent,color,width,name)
    if x_goal.parent is not None:
        draw_branch(map,x_goal.parent.q,x_goal.q,color,width,name)

def draw_trajectoy(map,trajectory,color = (0,0,0),width=1,name="RRT"):
    len_ = len(trajectory)
    for i in range(len_):
        draw_branch(map,trajectory[i],trajectory[i+1],color,width,name)
        if i == len_-2:
            return

def delete_trajectory(map,trajectory,width=1,name="RRT"):
    len_ = len(trajectory)
    for i in range(len_):
        if i == len_-1:
            return
        draw_branch(map,trajectory[i],trajectory[i+1],(255,255,255),width,name)
    draw_point(map,trajectory[0],width=width,raduis=width,name=name)
    draw_point(map,trajectory[-1],width=width,raduis=width,name=name)


def draw_trajectories_path(map,x_goal,color=(0,0,255),width=2,name="RRT"):
    if x_goal is None:
        return
    draw_trajectories_path(map,x_goal.parent,color,width,name)
    if x_goal.parent is not None:
        trajectory_ = trajectory(x_goal.parent.q,x_goal.q)
        draw_trajectoy(map,trajectory_,color,width,name)
        
def draw_ellipse(map,q_start,q_goal,c_max,c_min,Q_center,color=(0,0,255),name="RRT"):
    a = np.sqrt(c_max ** 2 - c_min ** 2) 
    b = c_max 
    angle = np.arctan2((q_goal["x"] - q_start["x"]),(q_goal["y"] - q_start["y"]))*180/np.pi
    cx = Q_center[0][0]# - a/2.0
    cy = Q_center[1][0]# - b / 2.0
    # target_rect = pygame.Rect((cx,cy,a,b))
    # shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    # pygame.draw.ellipse(shape_surf, (0,0,255), (0, 0, *target_rect.size), 1)
    # rotated_surf = pygame.transform.rotate(shape_surf, angle)
    # map.blit(rotated_surf, rotated_surf.get_rect(center = target_rect.center))
    cv.ellipse(img=map,center=(cx,cy),axes=(b/2.0,a/2.0),angle=angle, startAngle=0,endAngle=360,
                color=color,
                thickness=1,
                line_type=8)