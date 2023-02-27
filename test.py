import pygame
from src.rrt import RRT
from src.rrt_star import RRTStar
from src.informed_rrt_star import InformedRRTStar
import argparse
import numpy as np
import cv2 as cv

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-t', "--type", type=str, default='rrt',
                    help='rrt or rrt_star')
args = parser.parse_args()

# Initializing surface
map = np.ones((600,600,3), dtype=np.uint8)*255

#draw star and goal points
q_start={"x":100,"y":20,"theta":0.3,"delta":0.1,"beta":0.01}
q_goal={"x":350,"y":420,"theta":0,"delta":0,"beta":0}
star_color = (255,0,255)
goal_color = (255,0,0)

cv.circle(map,center=(q_start["x"],q_start["y"]),radius=5,color=star_color,thickness=-1,lineType=8)
cv.circle(map,center=(q_goal["x"],q_goal["y"]),radius=7,color=goal_color,thickness=-1,lineType=8)


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
rrt = None
if args.type == "rrt":
    rrt = RRT(map=map,q_start=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
elif args.type == "rrt_star":
    rrt = RRTStar(map=map,q_start=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
elif args.type == "informed_rrt_star":
    rrt = InformedRRTStar(map=map,q_start=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
else:
    print("incorrect algorithm name")
#Build RRT
rrt.build(int(1e6))

input('Press ENTER to exit')