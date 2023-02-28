from src.rrt import RRT
from src.rrt_star import RRTStar
from src.informed_rrt_star import InformedRRTStar
import argparse
import numpy as np
import cv2 as cv
from src.utils import *
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-t', "--type", type=str, default='rrt',
                    help='rrt or rrt_star')
args = parser.parse_args()

# Initializing surface
map_size = (450,450,3)

#draw star and goal points
q_start={"x":100,"y":20,"theta":0.3,"delta":0.1,"beta":0.01}
q_goal={"x":350,"y":420,"theta":0,"delta":0,"beta":0}
#goal color
color = (255,0,0)
#draw obstacles
obstacle_color=(0,255,0)
# Initializing RTT
rrt = None
if args.type == "rrt":
    rrt = RRT(size=map_size,q_start=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
elif args.type == "rrt_star":
    rrt = RRTStar(size=map_size,q_start=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
elif args.type == "informed_rrt_star":
    rrt = InformedRRTStar(size=map_size,q_start=q_start,q_goal=q_goal,goal_threshold=7, obstacles_color=obstacle_color)
else:
    print("incorrect algorithm name")
#Build RRT
rrt.build(int(1e6))

input('Press ENTER to exit')