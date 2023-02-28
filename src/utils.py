from .model import BicycleModel
from .control2d import steer
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

def parent(v):
    return v.parent

def ecludian(q_n, q_r):
    x1 , y1, = q_n["x"],q_n["y"]
    x2, y2 = q_r["x"], q_r["y"]
    return ((x2-x1)**2 + (y2 - y1)**2)**0.5


def draw_end_points(map,q_start,q_goal,color=(255,0,0),raduis=5,name="RRT"):
    cv.circle(map,center=(q_start["x"],q_start["y"]),radius=raduis,color=tuple(reversed(color)),thickness=-1,lineType=8)
    cv.circle(map,center=(q_goal["x"],q_goal["y"]),radius=raduis,color=tuple(reversed(color)),thickness=-1,lineType=8)
    cv.imshow(name,map)

def draw_obstacles(map,obstacle_color=(0,255,0),name="RRT"):
    #draw obstacles
    color = tuple(reversed(obstacle_color))
    cv.rectangle(map,(255,255),(300,300),color,thickness=-1)
    cv.rectangle(map,(150,150),(200,200),color,thickness=-1)
    cv.rectangle(map,(100,300),(170,325),color,thickness=-1)
    cv.rectangle(map,(100,80),(150,120),color,thickness=-1)
    cv.rectangle(map,(10,80),(50,120),color,thickness=-1)
    cv.rectangle(map,(210,320),(240,350),color,thickness=-1)
    cv.circle(map,center=(400,60),radius=40,color=color,thickness=-1)
    cv.circle(map,center=(250,60),radius=40,color=color,thickness=-1)
    cv.imshow(name,map)
    cv.waitKey(1)


def redraw_tree(map,tree,color,width=1,name="RRT"):
    if tree == None:
        return
    for node_ in tree.children:
        redraw_tree(map,node_,color,width,name)
        draw_line(map,steer(tree.q,node_.q),color=tuple(reversed(color)),width=width,name=name)

def draw_branch(map,q_start, q_end,color = (0,0,0),width = 1,name="RRT"):
    # Drawing line
    cv.line(map,pt1=(q_start["x"],q_start["y"]),pt2=(q_end["x"],q_end["y"]),color=tuple(reversed(color)),thickness=width)
    cv.imshow(name,map)
    
def delete_branch(map,q_start, q_end,color = (255,255,255),width=1,name="RRT"):
    # delete line
    draw_branch(map,q_start,q_end,color=color,width=width,name=name)

def draw_point(map,q_point,color = (0,0,0),raduis = 0.5,width=1,name="RRT"):
    # Drawing point
    cv.circle(map,center=(q_point["x"],q_point["y"]),radius = raduis,color=tuple(reversed(color)),thickness=-1)
    cv.imshow(name,map)
     
def draw_line(map,line,color = (0,0,0),width=1,name="RRT"):
    len_ = len(line)
    for i in range(len_):
        if i == len_-1:
            break
        draw_branch(map,line[i],line[i+1],color,width,name)

def delete_line(map,line,width=1,color = (255,255,255),name="RRT"):
    len_ = len(line)
    for i in range(len_):
        if i == len_-1:
            break
        draw_branch(map,line[i],line[i+1],color,width,name)
    # draw_point(map,line[0],width=width,raduis=width,name=name)
    # draw_point(map,line[-1],width=width,raduis=width,name=name)


def draw_path(map,x_goal,color=(0,0,255),width=2,name="RRT"):
    if x_goal is None:
        return
    draw_path(map,x_goal.parent,color,width,name)
    if x_goal.parent is not None:
        line = steer(x_goal.parent.q,x_goal.q)
        draw_line(map,line,color,width,name)
        cv.waitKey(1)
        
def delete_path(map,x_goal,color=(255,255,255),width=2,name="RRT"):
    draw_path(map,x_goal,color=color,width=width,name=name)
    
        
def draw_ellipse(map,q_start,q_goal,c_max,c_min,Q_center,color=(0,0,255),name="RRT"):
    a = np.sqrt(c_max ** 2 - c_min ** 2) 
    b = c_max 
    angle = np.arctan2((q_goal["y"] - q_start["y"]),(q_goal["x"] - q_start["x"]))*180/np.pi
    cx = round(Q_center[0][0])# - a/2.0
    cy = round(Q_center[1][0])# - b / 2.0
    # target_rect = pygame.Rect((cx,cy,a,b))
    # shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    # pygame.draw.ellipse(shape_surf, (0,0,255), (0, 0, *target_rect.size), 1)
    # rotated_surf = pygame.transform.rotate(shape_surf, angle)
    # map.blit(rotated_surf, rotated_surf.get_rect(center = target_rect.center))
    cv.ellipse(img=map,center=(cx,cy),axes=(round(b/2),round(a/2)),angle=angle, startAngle=0,endAngle=360,
                color=tuple(reversed(color)),
                thickness=1,
                lineType=8)